/* mumps_compat.h — minimal stub generated for embedded Ipopt build.
 * Matches the COIN-OR ThirdParty-Mumps interface used by homebrew Ipopt.
 * MUMPS integer type: always plain 'int' for the sequential version. */
#ifndef MUMPS_COMPAT_H
#define MUMPS_COMPAT_H
#ifndef MUMPS_INT
typedef int MUMPS_INT;
#endif
#ifndef MUMPS_INT8
typedef long long MUMPS_INT8;
#endif
#ifndef MUMPS_REAL
typedef float MUMPS_REAL;
#endif
#ifndef MUMPS_COMPLEX
typedef struct { float r, i; } MUMPS_COMPLEX;
#endif
#ifndef MUMPS_DCOMPLEX
typedef struct { double r, i; } MUMPS_DCOMPLEX;
#endif
/* Version string required by IpMumpsSolverInterface.cpp */
#ifndef MUMPS_VERSION
#define MUMPS_VERSION "5.7.3"
#endif
#endif /* MUMPS_COMPAT_H */
