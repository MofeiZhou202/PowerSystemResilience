# Fixed R3 reference evidence

These files are byte-for-byte copies of existing measurements, not new baselines.
`performance.json` is the summary from
`reports/windows_performance_program_20260913_baseline/`, performance reference
commit `78939619f31ebe19b9399c8f52d5c35f6d6c06ba`.
`recovery.json` and the three block files come from
`reports/r3_joint_backend_model_20260913/`, benchmark SHA256
`08d1f4eee980c902ceb480c94a9712586643afb20990552ac739dccff4360ba3`.

The contract pins summary hashes. Block files supply real parser test inputs;
their original schema is historical and cannot pass the R3 release gate.
The test suite migrates schema fields only in a temporary fixture. Those
fixtures are never candidate release evidence. Full raw logs remain in the
original report directories. Predictions and interpretation are maintained in
`docs/archive/general_solver_performance_program_2026-09-13.md`.
