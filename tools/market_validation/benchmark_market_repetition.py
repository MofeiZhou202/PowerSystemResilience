"""Serial browser protocol: five fresh processes and five same-session repeats per binary."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--baseline', required=True)
parser.add_argument('--candidate', required=True)
parser.add_argument('--input', required=True)
parser.add_argument('--output', required=True)
args = parser.parse_args()
output = Path(args.output)
output.mkdir(parents=True, exist_ok=True)
commands = []
results = {}
for label, binary in [('baseline', args.baseline), ('candidate', args.candidate)]:
    samples = []
    for process in range(1, 6):
        destination = output / label / f'process-{process:02d}'
        command = ['node', 'tools/market_validation/profile_market_scale.mjs',
                   '--server', binary, '--input', args.input, '--threads', '8',
                   '--browser', '--runs', '6' if process == 1 else '1', '--output', str(destination)]
        commands.append(command)
        (output / 'commands.json').write_text(json.dumps(commands, indent=2))
        print(f'{label} process {process}/5', flush=True)
        subprocess.run(command, check=True)
        series = json.loads((destination / 'series.json').read_text())
        samples.extend(series['reports'])
    results[label] = {}
    for state in ['fresh_server', 'same_session_repeat']:
        selected = [sample for sample in samples if sample['process_state'] == state]
        if len(selected) != 5:
            raise RuntimeError(f'Expected five {state} samples')
        values = [sample['wall_sec'] for sample in selected]
        results[label][state] = {'samples':selected, 'mean_wall_sec':sum(values)/len(values),
                                'max_wall_sec':max(values), 'all_within_60_sec':max(values)<=60}
    (output / 'summary.json').write_text(json.dumps(results, indent=2))
print(json.dumps(results, indent=2), flush=True)
