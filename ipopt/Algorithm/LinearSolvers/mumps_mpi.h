/* mumps_mpi.h — minimal sequential MPI stub for embedded Ipopt build.
 * Replaces the ThirdParty-Mumps generated header for use with
 * the sequential MUMPS (libmpiseq) from the homebrew ipopt cellar. */
#ifndef MUMPS_MPI_H
#define MUMPS_MPI_H
#include "mumps_compat.h"
typedef int MPI_Fint;
typedef int MPI_Comm;
#define MPI_COMM_WORLD 0
#define MPI_COMM_SELF  0
#define MPI_INTEGER MPI_INT
#define MPI_DOUBLE  MPI_DOUBLE_PRECISION
#endif /* MUMPS_MPI_H */
