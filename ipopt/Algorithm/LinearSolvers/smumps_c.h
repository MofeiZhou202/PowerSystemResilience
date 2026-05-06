/* smumps_c.h — SMUMPS_STRUC_C stub (single precision MUMPS).
 * Ipopt includes this but only uses it when IPOPT_SINGLE is defined.
 * Provides a minimal ABI-compatible structure. */
#ifndef SMUMPS_C_H
#define SMUMPS_C_H
#include "mumps_compat.h"
#include "mumps_mpi.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  MUMPS_INT   sym, par, job;
  MPI_Fint    comm_fortran;
  MUMPS_INT   icntl[60];
  float       cntl[15];
  MUMPS_INT   n, nnz;
  MUMPS_INT  *irn, *jcn;
  float      *a;
  MUMPS_INT   nrhs, lrhs;
  float      *rhs;
  MUMPS_INT   nz_rhs;
  float      *rhs_sparse;
  MUMPS_INT  *irhs_sparse, *irhs_ptr;
  MUMPS_INT   lsol_loc;
  float      *sol_loc;
  MUMPS_INT  *isol_loc;
  MUMPS_INT   infog[80];
  float       rinfog[20];
  MUMPS_INT   info[80];
  float       rinfo[40];
  MUMPS_INT  *pivnul_list, npiv;
  MUMPS_INT  *perm_in, *sym_perm, *uns_perm;
  MUMPS_INT  *listvar_schur, size_schur;
  float      *schur;
  void       *instance_number_;
  MUMPS_INT   pad[60];
} SMUMPS_STRUC_C;

void smumps_c(SMUMPS_STRUC_C* id);

#ifdef __cplusplus
}
#endif
#endif /* SMUMPS_C_H */
