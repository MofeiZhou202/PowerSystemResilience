# Runtime license sources

These notices accompany the Windows local-source package. They do not change
the licenses of the embedded dependencies.

- `HiGHS-LICENSE.txt` and `HiGHS-THIRD_PARTY_NOTICES.md`: upstream HiGHS v1.15.1,
  copied from the dependency upgrade's upstream source archive.
- `HiGHS-{pdqsort,zstr,amd,metis,rcm}-LICENSE*`: corresponding `extern/`
  license files from that same upstream v1.15.1 source archive.
- `Ipopt-LICENSE`: https://raw.githubusercontent.com/coin-or/Ipopt/releases/3.14.20/LICENSE
  (Eclipse Public License 2.0; version confirmed by the generated Ipopt configuration).
- `SCIP-Apache-2.0.txt`: https://www.apache.org/licenses/LICENSE-2.0.txt
  (Apache 2.0, as declared in the embedded `scip/scip/scip.h` copyright notice).

The package also copies bundled Eigen, fmt, JSON, Catch2, Boost/PaPILO,
MUMPS, SuiteSparse and sequential oneMKL notices from MIPSolvers.
