# Harmonics Validation

This folder contains external validation harnesses for the harmonic power-flow
module.

## Device-Level OpenDSS Validation

Build the C++ emitter:

```bash
cmake --build build/macos-release --target validate_harmonics_xref
```

Run the OpenDSS-backed device validation:

```bash
tools/harmonics_validation/.venv/bin/python \
  tools/harmonics_validation/compare_device_opendss.py \
  --cpp-bin build/macos-release/tests/validate_harmonics_xref
```

When the local OpenDSS Python environment is present, CMake also exposes the
same workflow as:

```bash
cmake --build build/macos-release --target validate_harmonics_device_opendss
```

The AC/DC path uses the HACDCPF VSC/HarmonicNIC model and validates the harmonic
AC network response against OpenDSS. OpenDSS is driven with the converter
harmonic current and a Reactor-based network, so both tools use
`Z(h) = R + j h X`.

The script also probes the native OpenDSS `VSConverter` class and records its
availability in `device_opendss_report.json`. The DSS C-API build used here does
not expose a native DC/DC converter class, so DC/DC validation is reported as
not natively available rather than silently substituting another OpenDSS device.
