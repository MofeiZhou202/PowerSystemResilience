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

## Cross-Engine Benchmark Matrix

The comprehensive matrix covers single/positive-sequence AC, balanced and
unbalanced phase-domain AC, pure DC ripple, and hybrid AC/DC converter cases.
It compares complex bus-voltage phasors, magnitudes, angles, IHD, THD, branch
current magnitudes, symmetrical components, and sequence-unbalance indices where
each metric applies.

```bash
cmake --build build/macos-release --target validate_harmonics_cross_engine_matrix
```

The generated reports are:

- `external_data/harmonics_validation/cross_engine_matrix.json`
- `external_data/harmonics_validation/cross_engine_matrix.md`

OpenDSS is exercised numerically for directly representable AC networks. Pure
DC and hybrid DC-port results are checked with an independently assembled dense
complex nodal solve. GridLAB-D 5.3 is reported as unsupported for native
frequency-domain harmonic power flow; ordinary fundamental power flow is never
presented as harmonic validation. The cross-engine matrix now executes a separate
GridLAB-D complex steady-state network for every representable AC harmonic order.
It explicitly authors `R(h)+j h X` and recovers transfer impedance from recorded
complex voltage/current. This closes the decoupled frequency-slice comparison;
EMT/PWM waveform validation would still require a separately qualified deltamode
and integer-cycle FFT experiment.

## Real-Feeder Harmonic Penetration (IEEE13)

`validate_harmonics_ieee13.cpp` loads a real OpenDSS feeder through the
phase-domain bridge (`load_three_phase_system_from_opendss`), injects a
balanced six-pulse spectrum at one bus, and emits per-order per-phase complex
bus voltages. `compare_ieee13_opendss.py` replays the identical injections in
OpenDSS harmonic mode (three single-phase ISources per order, Vsource kept
enabled) and compares phasors.

```bash
cmake --build build/macos-release --target validate_harmonics_ieee13
tools/harmonics_validation/.venv/bin/python \
  tools/harmonics_validation/compare_ieee13_opendss.py \
  --out /tmp/ieee13_report.json
```

Current status on IEEE13 (164 bus/phase/order points, injection 100 A at bus
675): max complex voltage deviation 1.652e-3 pu, below the fixed 2e-3 pu gate; driving-point impedance
|Z(675, h)| matches OpenDSS across h=2..25 including the h4/h8 resonances.
Residual deviation is dominated by regulator tap ratios, which the harmonic
study does not model (taps are held at ratio 1.0).

Setup notes learned the hard way:

- Keep the OpenDSS `Vsource` enabled in harmonic mode — it is the network's
  harmonic reference impedance; disabling it detaches the network.
- OpenDSS `AllBusVolts` reports only the most recently solved harmonic; solve
  and sample each order separately.
- `Bus.kVBase` is already line-neutral; do not divide by sqrt(3) again.
- `Circuit.AllBusVolts` returns flat re/im pairs, not complex objects.
