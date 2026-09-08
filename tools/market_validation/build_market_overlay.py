"""Build a local incremental experiment against an existing, recorded archive.

This does not produce a clean release or modify CMake/dependency guards. Only
listed translation units are replaced; the other archive objects are prebuilt.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--build', default='build/macos-release')
parser.add_argument('--output', default='output/market-performance/overlay')
parser.add_argument('--targets', default='run_gui_server,test_southern_market')
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
build = (root / args.build).resolve()
output = (root / args.output).resolve()
output.mkdir(parents=True, exist_ok=True)
for library in (build / 'tests').glob('*.dylib'):
    shutil.copy2(library, output / library.name)
commands = []

def compile_source(source, flags_file, destination):
    flags = flags_file.read_text()
    values = [re.search(r'^CXX_' + field + r' = (.*)$', flags, re.M).group(1)
              for field in ('DEFINES', 'INCLUDES', 'FLAGS')]
    compiler = re.search(r'^CMAKE_CXX_COMPILER:FILEPATH=(.*)$',
                         (build / 'CMakeCache.txt').read_text(), re.M).group(1)
    command = [compiler, *shlex.split(' '.join(values)), '-Wall', '-Wextra',
               '-Wpedantic', '-c', str(source), '-o', str(destination)]
    commands.append(command)
    subprocess.run(command, cwd=build, check=True)

models = []
sources = ('southern_market', 'market_operation', 'southern_boundary', 'market_forecast')
for source in sources:
    model = output / f'{source}.cpp.o'
    compile_source(root / f'src/market/{source}.cpp',
                   build / 'CMakeFiles/hacdcpf.dir/flags.make', model)
    models.append(model)
adapter_source = root.parent / 'MIPSolvers/src/engine/solver/external/adapters.cpp'
adapter_object = output / 'adapters.cpp.o'
compile_source(adapter_source,
               build / '_deps/mipsolvers_build/CMakeFiles/mipsolvers.dir/flags.make', adapter_object)
models.append(adapter_object)
for target in args.targets.split(','):
    directory = build / f'tests/CMakeFiles/{target}.dir'
    command = shlex.split((directory / 'link.txt').read_text())
    if target.startswith('test_') or target == 'run_gui_server':
        obj = output / f'{target}.cpp.o'
        compile_source(root / f'tests/{target}.cpp', directory / 'flags.make', obj)
        command = [str(obj) if part == f'CMakeFiles/{target}.dir/{target}.cpp.o'
                   else part for part in command]
    command[command.index('-o') + 1] = str(output / target)
    for model in models:
        command.insert(command.index('../libhacdcpf.a'), str(model))
    commands.append(command)
    subprocess.run(command, cwd=build / 'tests', check=True)

def digest(file):
    value = hashlib.sha256()
    with file.open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()

evidence = {'scope': 'incremental local experiment; listed market/server/adapter units rebuilt, remaining objects and libraries prebuilt',
            'adapter_source_sha256': digest(adapter_source),
            'adapter_header_sha256': digest(root.parent / 'MIPSolvers/include/mipsolvers/engine/solver/external/adapters.hpp'),
            'server_source_sha256': digest(root / 'tests/run_gui_server.cpp'),
            'build': str(build), 'commands': commands,
            'source_sha256': {name: digest(root / f'src/market/{name}.cpp')
                              for name in sources},
            'archive_sha256': digest(build / 'libhacdcpf.a'),
            'dependency_archive_sha256': digest(build / '_deps/mipsolvers_build/libmipsolvers.a'),
            'binaries': {name: digest(output / name) for name in args.targets.split(',')}}
(output / 'build.json').write_text(json.dumps(evidence, indent=2) + '\n')
print(json.dumps(evidence['binaries'], indent=2))
