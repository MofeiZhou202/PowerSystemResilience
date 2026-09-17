from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_gridlabd_windows_command_contract() -> None:
    source = (ROOT / "src/io/gridlabd_bridge.cpp").read_text(encoding="utf-8")
    process = source.split("int run_process_posix", 1)[1]
    windows = process.split("#ifdef _WIN32", 1)[1].split("#else", 1)[0]
    assert 'std::string command = "\\\"";' in windows
    assert 'command += "set \\\"" + name + "=" + value + "\\\" && ";' in windows
    assert 'stderr_path.string() + "\\\"\\\""' in windows
    assert "(void)env" not in windows


def test_io_path_contracts() -> None:
    etap = (ROOT / "src/io/etap_io.cpp").read_text(encoding="utf-8")
    svg = (ROOT / "src/io/svg_distribution_io.cpp").read_text(encoding="utf-8")
    assert "unique_etap_fidelity_path()" in etap
    assert "ScopedFileRemoval cleanup(tmp)" in etap
    assert 'temp_directory_path() / "hacdcpf_etap_fidelity.xlsx"' not in etap
    assert "path.u8string()" in svg
    assert "options.model_name = path_utf8(path.stem())" in svg


if __name__ == "__main__":
    test_gridlabd_windows_command_contract()
    test_io_path_contracts()
