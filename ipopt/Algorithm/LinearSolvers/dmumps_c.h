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
 * Double-precision MUMPS structure — EXACT MUMPS 5.6.2 ABI layout.
 * Must byte-for-byte match the libdmumps.dylib that homebrew ipopt
 * 3.14.19 links (MUMPS 5.6.2).  Field order/sizes are copied verbatim
 * from MUMPS_5.6.2/include/dmumps_c.h; DMUMPS_REAL and DMUMPS_COMPLEX
 * are both `double` for the real double-precision variant.
 * Earlier this struct was a hand-written guess (claimed "5.7", padded
 * with pad[60]); its wrong field offsets corrupted the heap because the
 * linked MUMPS is 5.6.2, not 5.7.x.
 * ------------------------------------------------------------------- */
typedef struct {
  MUMPS_INT   sym, par, job;
  MUMPS_INT   comm_fortran;       /* Fortran communicator */
  MUMPS_INT   icntl[60];
  MUMPS_INT   keep[500];
  double      cntl[15];
  double      dkeep[230];
  MUMPS_INT8  keep8[150];
  MUMPS_INT   n;
  MUMPS_INT   nblk;

  MUMPS_INT   nz_alloc;           /* matlab interface */

  /* Assembled entry */
  MUMPS_INT   nz;
  MUMPS_INT8  nnz;
  MUMPS_INT  *irn;
  MUMPS_INT  *jcn;
  double     *a;

  /* Distributed entry */
  MUMPS_INT   nz_loc;
  MUMPS_INT8  nnz_loc;
  MUMPS_INT  *irn_loc;
  MUMPS_INT  *jcn_loc;
  double     *a_loc;

  /* Element entry */
  MUMPS_INT   nelt;
  MUMPS_INT  *eltptr;
  MUMPS_INT  *eltvar;
  double     *a_elt;

  /* Matrix by blocks */
  MUMPS_INT  *blkptr;
  MUMPS_INT  *blkvar;

  /* Ordering, if given by user */
  MUMPS_INT  *perm_in;

  /* Orderings returned to user */
  MUMPS_INT  *sym_perm;
  MUMPS_INT  *uns_perm;

  /* Scaling */
  double     *colsca;
  double     *rowsca;
  MUMPS_INT   colsca_from_mumps;
  MUMPS_INT   rowsca_from_mumps;

  /* RHS, solution, output data and statistics */
  double     *rhs, *redrhs, *rhs_sparse, *sol_loc, *rhs_loc;
  MUMPS_INT  *irhs_sparse, *irhs_ptr, *isol_loc, *irhs_loc;
  MUMPS_INT   nrhs, lrhs, lredrhs, nz_rhs, lsol_loc, nloc_rhs, lrhs_loc;
  MUMPS_INT   schur_mloc, schur_nloc, schur_lld;
  MUMPS_INT   mblock, nblock, nprow, npcol;
  MUMPS_INT   info[80], infog[80];
  double      rinfo[40], rinfog[40];

  /* Null space */
  MUMPS_INT   deficiency;
  MUMPS_INT  *pivnul_list;
  MUMPS_INT  *mapping;

  /* Schur */
  MUMPS_INT   size_schur;
  MUMPS_INT  *listvar_schur;
  double     *schur;

  /* Internal parameters */
  MUMPS_INT   instance_number;
  double     *wk_user;

  /* Version number: MUMPS_VERSION_MAX_LEN(=30) + 1 (\0) + 1 (alignment) */
  char        version_number[30 + 1 + 1];
  /* Out-of-core */
  char        ooc_tmpdir[256];
  char        ooc_prefix[64];
  /* Matrix-market dump */
  char        write_problem[256];
  MUMPS_INT   lwk_user;
  /* Save/restore */
  char        save_dir[256];
  char        save_prefix[256];

  /* Metis options */
  MUMPS_INT   metis_options[40];
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
