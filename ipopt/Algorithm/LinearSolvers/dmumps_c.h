/* dmumps_c.h — DMUMPS_STRUC_C stub for embedded Ipopt build.
 * Structure layout matches MUMPS 5.7 ABI (the version bundled with
 * homebrew ipopt 3.14.19 on macOS). All array sizes are standard
 * MUMPS 5.x values.  The smumps analogue is in smumps_c.h. */
#ifndef DMUMPS_C_H
#define DMUMPS_C_H
#include "mumps_compat.h"
#include "mumps_mpi.h"
#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------
 * Double-precision MUMPS structure (job-level view). Only fields
 * actually used by IpMumpsSolverInterface.cpp are documented here;
 * the rest are padding to maintain ABI compatibility.
 * ------------------------------------------------------------------- */
typedef struct {
  /* job / problem type */
  MUMPS_INT   sym;          /* 0=unsym, 1=SPD, 2=general sym  */
  MUMPS_INT   par;          /* 0=host is worker, 1=host is not */
  MUMPS_INT   job;          /* -1=init, -2=destroy, 1=analyse, 2=factor, 3=solve, 4=1+2, 5=1+2+3, 6=2+3 */
  MPI_Fint    comm_fortran; /* MPI_COMM_WORLD in Fortran integer form */

  /* analysis */
  MUMPS_INT   icntl[60];    /* integer control parameters  */
  double      cntl[15];     /* real control parameters     */
  MUMPS_INT   n;            /* order of A                  */

  /* assembled triplet input */
  MUMPS_INT   nz;           /* number of entries (deprecated MUMPS 5.x, use nnz for >2^31) */
  MUMPS_INT8  nnz;          /* number of entries (MUMPS_INT8, MUMPS 5.1+) */
  MUMPS_INT  *irn;          /* row indices (1-based)        */
  MUMPS_INT  *jcn;          /* column indices (1-based)     */
  double     *a;            /* values                       */

  /* RHS / solution */
  MUMPS_INT   nrhs;         /* number of right-hand sides   */
  MUMPS_INT   lrhs;         /* leading dimension of rhs     */
  double     *rhs;          /* RHS and solution on return   */

  /* sparse RHS (optional) */
  MUMPS_INT   nz_rhs;
  double     *rhs_sparse;
  MUMPS_INT  *irhs_sparse;
  MUMPS_INT  *irhs_ptr;

  /* distributed solution (optional) */
  MUMPS_INT   lsol_loc;
  double     *sol_loc;
  MUMPS_INT  *isol_loc;

  /* output info */
  MUMPS_INT   infog[80];    /* global info (root process)   */
  double      rinfog[20];   /* global real info (root)      */
  MUMPS_INT   info[80];     /* local info per process       */
  double      rinfo[40];    /* local real info per process  */

  /* pivoting */
  MUMPS_INT  *pivnul_list;  /* list of pivoting null variables */
  MUMPS_INT   npiv;         /* number of null pivots        */

  /* ordering */
  MUMPS_INT  *perm_in;      /* user-provided ordering       */
  MUMPS_INT  *sym_perm;     /* symmetric permutation (output) */
  MUMPS_INT  *uns_perm;     /* unsymmetric permutation      */
  MUMPS_INT  *listvar_schur;/* list of variables for Schur  */
  MUMPS_INT   size_schur;   /* size of the Schur complement */
  double     *schur;        /* Schur complement             */

  /* internal workspace (opaque) — must remain at end to preserve layout */
  void       *instance_number_; /* internal */
  MUMPS_INT   pad[60];      /* ABI padding for future fields */
} DMUMPS_STRUC_C;

/* Alias: older Ipopt code uses nz, newer uses nnz; both alias the same field.
 * IpMumpsSolverInterface.cpp uses ->nz in some versions. */
#ifndef mumps_nz_is_nnz
#define mumps_nz_is_nnz 1
#endif

void dmumps_c(DMUMPS_STRUC_C* id);

#ifdef __cplusplus
}
#endif
#endif /* DMUMPS_C_H */
