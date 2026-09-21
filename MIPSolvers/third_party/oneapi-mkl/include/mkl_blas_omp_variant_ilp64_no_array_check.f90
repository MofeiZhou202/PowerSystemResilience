!===============================================================================
! Copyright (C) 2021 Intel Corporation
!
! This software and the related documents are Intel copyrighted  materials,  and
! your use of  them is  governed by the  express license  under which  they were
! provided to you (License).  Unless the License provides otherwise, you may not
! use, modify, copy, publish, distribute,  disclose or transmit this software or
! the related documents without Intel's prior written permission.
!
! This software and the related documents  are provided as  is,  with no express
! or implied  warranties,  other  than those  that are  expressly stated  in the
! License.
!===============================================================================

!  Content:
!      Intel(R) oneAPI Math Kernel Library (oneMKL) FORTRAN interface for BLAS
!      OpenMP offload
!*******************************************************************************

  interface

     ! Matrix transposition and copy API

     subroutine mkl_blas_simatcopy_batch_strided_omp_offload_ilp64 ( ordering, trans, rows,       &
          &cols, alpha, ab, lda, ldb, stride, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering, trans
       integer,intent(in)             :: rows, cols, lda, ldb, stride, batch_size
       real,intent(in)                :: alpha
       !dec$ attributes no_arg_check  :: ab
       real,intent(inout)             :: ab( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_simatcopy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_dimatcopy_batch_strided_omp_offload_ilp64 ( ordering, trans, rows,       &
          &cols, alpha, ab, lda, ldb, stride, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering, trans
       integer,intent(in)             :: rows, cols, lda, ldb, stride, batch_size
       double precision,intent(in)    :: alpha
       !dec$ attributes no_arg_check  :: ab
       double precision,intent(inout) :: ab( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dimatcopy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_cimatcopy_batch_strided_omp_offload_ilp64 ( ordering, trans, rows,       &
          &cols, alpha, ab, lda, ldb, stride, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering, trans
       integer,intent(in)             :: rows, cols, lda, ldb, stride, batch_size
       complex,intent(in)             :: alpha
       !dec$ attributes no_arg_check  :: ab
       complex,intent(inout)          :: ab( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cimatcopy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_zimatcopy_batch_strided_omp_offload_ilp64 ( ordering, trans, rows,       &
          &cols, alpha, ab, lda, ldb, stride, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering, trans
       integer,intent(in)             :: rows, cols, lda, ldb, stride, batch_size
       complex*16,intent(in)          :: alpha
       !dec$ attributes no_arg_check  :: ab
       complex*16,intent(inout)       :: ab( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zimatcopy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_somatcopy_batch_strided_omp_offload_ilp64 ( ordering, trans, rows,       &
          &cols, alpha, a, lda, stridea, b, ldb, strideb, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering, trans
       integer,intent(in)             :: rows, cols, lda, ldb
       integer,intent(in)             :: stridea, strideb, batch_size
       real,intent(in)                :: alpha
       !dec$ attributes no_arg_check  :: a
       real,intent(in)                :: a( lda, * )
       !dec$ attributes no_arg_check  :: b
       real,intent(inout)             :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_somatcopy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_domatcopy_batch_strided_omp_offload_ilp64 ( ordering, trans, rows,       &
          &cols, alpha, a, lda, stridea, b, ldb, strideb, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering, trans
       integer,intent(in)             :: rows, cols, lda, ldb
       integer,intent(in)             :: stridea, strideb, batch_size
       double precision,intent(in)    :: alpha
       !dec$ attributes no_arg_check  :: a
       double precision,intent(in)    :: a( lda, * )
       !dec$ attributes no_arg_check  :: b
       double precision,intent(inout) :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_domatcopy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_comatcopy_batch_strided_omp_offload_ilp64 ( ordering, trans, rows,       &
          &cols, alpha, a, lda, stridea, b, ldb, strideb, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering, trans
       integer,intent(in)             :: rows, cols, lda, ldb
       integer,intent(in)             :: stridea, strideb, batch_size
       complex,intent(in)             :: alpha
       !dec$ attributes no_arg_check  :: a
       complex,intent(in)             :: a( lda, * )
       !dec$ attributes no_arg_check  :: b
       complex,intent(inout)          :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_comatcopy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_zomatcopy_batch_strided_omp_offload_ilp64 ( ordering, trans, rows,       &
          &cols, alpha, a, lda, stridea, b, ldb, strideb, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering, trans
       integer,intent(in)             :: rows, cols, lda, ldb
       integer,intent(in)             :: stridea, strideb, batch_size
       complex*16,intent(in)          :: alpha
       !dec$ attributes no_arg_check  :: a
       complex*16,intent(in)          :: a( lda, * )
       !dec$ attributes no_arg_check  :: b
       complex*16,intent(inout)       :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zomatcopy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_somatadd_batch_strided_omp_offload_ilp64 ( ordering, transa, transb,     &
          &rows, cols, alpha, a, lda, stridea, beta, b, ldb, strideb, c, ldc, stridec,            &
          &batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering, transa, transb
       integer,intent(in)             :: rows, cols, lda, ldb, ldc
       integer,intent(in)             :: stridea, strideb, stridec, batch_size
       real,intent(in)                :: alpha, beta
       !dec$ attributes no_arg_check  :: a
       real,intent(in)                :: a( lda, * )
       !dec$ attributes no_arg_check  :: b
       real,intent(in)                :: b( ldb, * )
       !dec$ attributes no_arg_check  :: c
       real,intent(inout)             :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_somatadd_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_domatadd_batch_strided_omp_offload_ilp64 ( ordering, transa, transb,     &
          &rows, cols, alpha, a, lda, stridea, beta, b, ldb, strideb, c, ldc, stridec,            &
          &batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering, transa, transb
       integer,intent(in)             :: rows, cols, lda, ldb, ldc
       integer,intent(in)             :: stridea, strideb, stridec, batch_size
       double precision,intent(in)    :: alpha, beta
       !dec$ attributes no_arg_check  :: a
       double precision,intent(in)    :: a( lda, * )
       !dec$ attributes no_arg_check  :: b
       double precision,intent(in)    :: b( ldb, * )
       !dec$ attributes no_arg_check  :: c
       double precision,intent(inout) :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_domatadd_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_comatadd_batch_strided_omp_offload_ilp64 ( ordering, transa, transb,     &
          &rows, cols, alpha, a, lda, stridea, beta, b, ldb, strideb, c, ldc, stridec,            &
          &batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering, transa, transb
       integer,intent(in)             :: rows, cols, lda, ldb, ldc
       integer,intent(in)             :: stridea, strideb, stridec, batch_size
       complex,intent(in)             :: alpha, beta
       !dec$ attributes no_arg_check  :: a
       complex,intent(in)             :: a( lda, * )
       !dec$ attributes no_arg_check  :: b
       complex,intent(in)             :: b( ldb, * )
       !dec$ attributes no_arg_check  :: c
       complex,intent(inout)          :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_comatadd_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_zomatadd_batch_strided_omp_offload_ilp64 ( ordering, transa, transb,     &
          &rows, cols, alpha, a, lda, stridea, beta, b, ldb, strideb, c, ldc, stridec,            &
          &batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering, transa, transb
       integer,intent(in)             :: rows, cols, lda, ldb, ldc
       integer,intent(in)             :: stridea, strideb, stridec, batch_size
       complex*16,intent(in)          :: alpha, beta
       !dec$ attributes no_arg_check  :: a
       complex*16,intent(in)          :: a( lda, * )
       !dec$ attributes no_arg_check  :: b
       complex*16,intent(in)          :: b( ldb, * )
       !dec$ attributes no_arg_check  :: c
       complex*16,intent(inout)       :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zomatadd_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_simatcopy_batch_omp_offload_ilp64 ( ordering, trans_array, rows_array,          &
          &cols_array, alpha_array, ab_array, lda_array, ldb_array, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering
       character*1,intent(in)         :: trans_array(*)
       integer,intent(in)             :: rows_array(*), cols_array(*), lda_array(*), ldb_array(*), group_count, group_size(*)
       real,intent(in)                :: alpha_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  ::  ab_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_simatcopy_batch_omp_offload_ilp64

     subroutine mkl_blas_dimatcopy_batch_omp_offload_ilp64 ( ordering, trans_array, rows_array,          &
          &cols_array, alpha_array, ab_array, lda_array, ldb_array, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering
       character*1,intent(in)         :: trans_array(*)
       integer,intent(in)             :: rows_array(*), cols_array(*), lda_array(*), ldb_array(*), group_count, group_size(*)
       double precision,intent(in)                :: alpha_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  ::  ab_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dimatcopy_batch_omp_offload_ilp64

     subroutine mkl_blas_cimatcopy_batch_omp_offload_ilp64 ( ordering, trans_array, rows_array,          &
          &cols_array, alpha_array, ab_array, lda_array, ldb_array, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering
       character*1,intent(in)         :: trans_array(*)
       integer,intent(in)             :: rows_array(*), cols_array(*), lda_array(*), ldb_array(*), group_count, group_size(*)
       complex,intent(in)                :: alpha_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  ::  ab_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cimatcopy_batch_omp_offload_ilp64

     subroutine mkl_blas_zimatcopy_batch_omp_offload_ilp64 ( ordering, trans_array, rows_array,          &
          &cols_array, alpha_array, ab_array, lda_array, ldb_array, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering
       character*1,intent(in)         :: trans_array(*)
       integer,intent(in)             :: rows_array(*), cols_array(*), lda_array(*), ldb_array(*), group_count, group_size(*)
       complex*16,intent(in)                :: alpha_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  ::  ab_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zimatcopy_batch_omp_offload_ilp64

     subroutine mkl_blas_somatcopy_batch_omp_offload_ilp64 ( ordering, trans_array, rows_array,          &
          &cols_array, alpha_array, a_array, lda_array, b_array, ldb_array, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering
       character*1,intent(in)         :: trans_array(*)
       integer,intent(in)             :: rows_array(*), cols_array(*), lda_array(*), ldb_array(*), group_count, group_size(*)
       real,intent(in)                :: alpha_array(*)
       integer(KIND=C_INTPTR_T),intent(in)  ::  a_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  ::  b_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_somatcopy_batch_omp_offload_ilp64

     subroutine mkl_blas_domatcopy_batch_omp_offload_ilp64 ( ordering, trans_array, rows_array,          &
          &cols_array, alpha_array, a_array, lda_array, b_array, ldb_array, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering
       character*1,intent(in)         :: trans_array(*)
       integer,intent(in)             :: rows_array(*), cols_array(*), lda_array(*), ldb_array(*), group_count, group_size(*)
       double precision,intent(in)                :: alpha_array(*)
       integer(KIND=C_INTPTR_T),intent(in)  ::  a_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  ::  b_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_domatcopy_batch_omp_offload_ilp64

     subroutine mkl_blas_comatcopy_batch_omp_offload_ilp64 ( ordering, trans_array, rows_array,          &
          &cols_array, alpha_array, a_array, lda_array, b_array, ldb_array, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering
       character*1,intent(in)         :: trans_array(*)
       integer,intent(in)             :: rows_array(*), cols_array(*), lda_array(*), ldb_array(*), group_count, group_size(*)
       complex,intent(in)                :: alpha_array(*)
       integer(KIND=C_INTPTR_T),intent(in)  ::  a_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  ::  b_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_comatcopy_batch_omp_offload_ilp64

     subroutine mkl_blas_zomatcopy_batch_omp_offload_ilp64 ( ordering, trans_array, rows_array,          &
          &cols_array, alpha_array, a_array, lda_array, b_array, ldb_array, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)         :: ordering
       character*1,intent(in)         :: trans_array(*)
       integer,intent(in)             :: rows_array(*), cols_array(*), lda_array(*), ldb_array(*), group_count, group_size(*)
       complex*16,intent(in)                :: alpha_array(*)
       integer(KIND=C_INTPTR_T),intent(in)  ::  a_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  ::  b_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zomatcopy_batch_omp_offload_ilp64

     ! BLAS Level3

     subroutine mkl_blas_dgemm_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: transa, transb
       integer,intent(in)                 :: m, n, k, lda, ldb, ldc
       double precision,intent(in)        :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       double precision,intent(in)        :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       double precision,intent(inout)     :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dgemm_omp_offload_ilp64

     subroutine mkl_blas_sgemm_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: transa, transb
       integer,intent(in)                  :: m, n, k, lda, ldb, ldc
       real,intent(in)                     :: alpha, beta
       !dec$ attributes no_arg_check       :: a, b
       real,intent(in)                     :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       real,intent(inout)                  :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sgemm_omp_offload_ilp64

     subroutine mkl_blas_zgemm_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: transa, transb
       integer,intent(in)                 :: m, n, k, lda, ldb, ldc
       complex*16,intent(in)              :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       complex*16,intent(in)              :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       complex*16,intent(inout)           :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zgemm_omp_offload_ilp64

     subroutine mkl_blas_cgemm_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: transa, transb
       integer,intent(in)                  :: m, n, k, lda, ldb, ldc
       complex,intent(in)                  :: alpha, beta
       !dec$ attributes no_arg_check       :: a, b
       complex,intent(in)                  :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       complex,intent(inout)               :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cgemm_omp_offload_ilp64

     subroutine mkl_blas_zgemm3m_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: transa, transb
       integer,intent(in)                 :: m, n, k, lda, ldb, ldc
       complex*16,intent(in)              :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       complex*16,intent(in)              :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       complex*16,intent(inout)           :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zgemm3m_omp_offload_ilp64

     subroutine mkl_blas_cgemm3m_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: transa, transb
       integer,intent(in)                  :: m, n, k, lda, ldb, ldc
       complex,intent(in)                  :: alpha, beta
       !dec$ attributes no_arg_check       :: a, b
       complex,intent(in)                  :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       complex,intent(inout)               :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cgemm3m_omp_offload_ilp64
     
     subroutine mkl_blas_gemm_s8u8s32_omp_offload_ilp64 ( transa, transb, offsetc, m, n, k,       &
          &alpha, a, lda, ao, b, ldb, bo, beta, c, ldc, co, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)          :: transa, transb, offsetc
       integer,intent(in)              :: m, n, k, lda, ldb, ldc
       integer*1,intent(in)            :: ao, bo
       real,intent(in)                 :: alpha, beta
       !dec$ attributes no_arg_check   :: a, b
       integer*1,intent(in)            :: a( lda, * ), b (ldb, * )
       !dec$ attributes no_arg_check   :: c
       integer*4,intent(inout)         :: c( ldc, * )
       !dec$ attributes no_arg_check   :: co
       integer*4,intent(in)            :: co( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_gemm_s8u8s32_omp_offload_ilp64

     subroutine mkl_blas_dsymm_omp_offload_ilp64 ( side, uplo, m, n, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo
       integer,intent(in)                 :: m, n, lda, ldb, ldc
       double precision,intent(in)        :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       double precision,intent(in)        :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       double precision,intent(inout)     :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dsymm_omp_offload_ilp64

     subroutine mkl_blas_ssymm_omp_offload_ilp64 ( side, uplo, m, n, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: side, uplo
       integer,intent(in)                  :: m, n, lda, ldb, ldc
       real,intent(in)                     :: alpha, beta
       !dec$ attributes no_arg_check       :: a, b
       real,intent(in)                     :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       real,intent(inout)                  :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ssymm_omp_offload_ilp64

     subroutine mkl_blas_zsymm_omp_offload_ilp64 ( side, uplo, m, n, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo
       integer,intent(in)                 :: m, n, lda, ldb, ldc
       complex*16,intent(in)              :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       complex*16,intent(in)              :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       complex*16,intent(inout)           :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zsymm_omp_offload_ilp64

     subroutine mkl_blas_csymm_omp_offload_ilp64 ( side, uplo, m, n, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: side, uplo
       integer,intent(in)                  :: m, n, lda, ldb, ldc
       complex,intent(in)                  :: alpha, beta
       !dec$ attributes no_arg_check       :: a, b
       complex,intent(in)                  :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       complex,intent(inout)               :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_csymm_omp_offload_ilp64

     subroutine mkl_blas_zhemm_omp_offload_ilp64 ( side, uplo, m, n, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo
       integer,intent(in)                 :: m, n, lda, ldb, ldc
       complex*16,intent(in)              :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       complex*16,intent(in)              :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       complex*16,intent(inout)           :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zhemm_omp_offload_ilp64

     subroutine mkl_blas_chemm_omp_offload_ilp64 ( side, uplo, m, n, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: side, uplo
       integer,intent(in)                  :: m, n, lda, ldb, ldc
       complex,intent(in)                  :: alpha, beta
       !dec$ attributes no_arg_check       :: a, b
       complex,intent(in)                  :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       complex,intent(inout)               :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_chemm_omp_offload_ilp64

     subroutine mkl_blas_dsyrk_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          &beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: uplo, trans
       integer,intent(in)                 :: n, k, lda, ldc
       double precision,intent(in)        :: alpha, beta
       !dec$ attributes no_arg_check      :: a
       double precision,intent(in)        :: a( lda, * )
       !dec$ attributes no_arg_check      :: c
       double precision,intent(inout)     :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dsyrk_omp_offload_ilp64

     subroutine mkl_blas_ssyrk_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          &beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo, trans
       integer,intent(in)                  :: n, k, lda, ldc
       real,intent(in)                     :: alpha, beta
       !dec$ attributes no_arg_check       :: a
       real,intent(in)                     :: a( lda, * )
       !dec$ attributes no_arg_check       :: c
       real,intent(inout)                  :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ssyrk_omp_offload_ilp64

     subroutine mkl_blas_zsyrk_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          &beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: uplo, trans
       integer,intent(in)                 :: n, k, lda, ldc
       complex*16,intent(in)              :: alpha, beta
       !dec$ attributes no_arg_check      :: a
       complex*16,intent(in)              :: a( lda, * )
       !dec$ attributes no_arg_check      :: c
       complex*16,intent(inout)           :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zsyrk_omp_offload_ilp64

     subroutine mkl_blas_csyrk_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          &beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo, trans
       integer,intent(in)                  :: n, k, lda, ldc
       complex,intent(in)                  :: alpha, beta
       !dec$ attributes no_arg_check       :: a
       complex,intent(in)                  :: a( lda, * )
       !dec$ attributes no_arg_check       :: c
       complex,intent(inout)               :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_csyrk_omp_offload_ilp64

     subroutine mkl_blas_zherk_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          &beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: uplo, trans
       integer,intent(in)                 :: n, k, lda, ldc
       double precision,intent(in)        :: alpha, beta
       !dec$ attributes no_arg_check      :: a
       complex*16,intent(in)              :: a( lda, * )
       !dec$ attributes no_arg_check      :: c
       complex*16,intent(inout)           :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zherk_omp_offload_ilp64

     subroutine mkl_blas_cherk_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          &beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo, trans
       integer,intent(in)                  :: n, k, lda, ldc
       real,intent(in)                     :: alpha, beta
       !dec$ attributes no_arg_check       :: a
       complex,intent(in)                  :: a( lda, * )
       !dec$ attributes no_arg_check       :: c
       complex,intent(inout)               :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cherk_omp_offload_ilp64

     subroutine mkl_blas_dsyr2k_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: uplo, trans
       integer,intent(in)                 :: n, k, lda, ldb, ldc
       double precision,intent(in)        :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       double precision,intent(in)        :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       double precision,intent(inout)     :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dsyr2k_omp_offload_ilp64

     subroutine mkl_blas_ssyr2k_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo, trans
       integer,intent(in)                  :: n, k, lda, ldb, ldc
       real,intent(in)                     :: alpha, beta
       !dec$ attributes no_arg_check       :: a, b
       real,intent(in)                     :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       real,intent(inout)                  :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ssyr2k_omp_offload_ilp64

     subroutine mkl_blas_zsyr2k_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: uplo, trans
       integer,intent(in)                 :: n, k, lda, ldb, ldc
       complex*16,intent(in)              :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       complex*16,intent(in)              :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       complex*16,intent(inout)           :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zsyr2k_omp_offload_ilp64

     subroutine mkl_blas_csyr2k_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo, trans
       integer,intent(in)                  :: n, k, lda, ldb, ldc
       complex,intent(in)                  :: alpha, beta
       !dec$ attributes no_arg_check       :: a, b
       complex,intent(in)                  :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       complex,intent(inout)               :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_csyr2k_omp_offload_ilp64

     subroutine mkl_blas_zher2k_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: uplo, trans
       integer,intent(in)                 :: n, k, lda, ldb, ldc
       double precision,intent(in)        :: beta
       complex*16,intent(in)              :: alpha
       !dec$ attributes no_arg_check      :: a, b
       complex*16,intent(in)              :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       complex*16,intent(inout)           :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zher2k_omp_offload_ilp64

     subroutine mkl_blas_cher2k_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo, trans
       integer,intent(in)                  :: n, k, lda, ldb, ldc
       real,intent(in)                     :: beta
       complex,intent(in)                  :: alpha
       !dec$ attributes no_arg_check       :: a, b
       complex,intent(in)                  :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       complex,intent(inout)               :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cher2k_omp_offload_ilp64

     subroutine mkl_blas_dtrmm_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda,        &
          &b, ldb, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, ldb
       double precision,intent(in)        :: alpha
       !dec$ attributes no_arg_check      :: a
       double precision,intent(in)        :: a( lda, * )
       !dec$ attributes no_arg_check      :: b
       double precision,intent(inout)     :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dtrmm_omp_offload_ilp64

     subroutine mkl_blas_strmm_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda,        &
          &b, ldb, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: side, uplo, trans, diag
       integer,intent(in)                  :: m, n, lda, ldb
       real,intent(in)                     :: alpha
       !dec$ attributes no_arg_check       :: a
       real,intent(in)                     :: a( lda, * )
       !dec$ attributes no_arg_check       :: b
       real,intent(inout)                  :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_strmm_omp_offload_ilp64

     subroutine mkl_blas_ztrmm_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda,        &
          &b, ldb, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, ldb
       complex*16,intent(in)              :: alpha
       !dec$ attributes no_arg_check      :: a
       complex*16,intent(in)              :: a( lda, * )
       !dec$ attributes no_arg_check      :: b
       complex*16,intent(inout)           :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ztrmm_omp_offload_ilp64

     subroutine mkl_blas_ctrmm_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda,        &
          &b, ldb, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: side, uplo, trans, diag
       integer,intent(in)                  :: m, n, lda, ldb
       complex,intent(in)                  :: alpha
       !dec$ attributes no_arg_check       :: a
       complex,intent(in)                  :: a( lda, * )
       !dec$ attributes no_arg_check       :: b
       complex,intent(inout)               :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ctrmm_omp_offload_ilp64

     subroutine mkl_blas_strmm_oop_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha,        &
          &a, lda, b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, ldb, ldc
       real,intent(in)                    :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       real,intent(in)                    :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       real,intent(inout)                 :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_strmm_oop_omp_offload_ilp64

     subroutine mkl_blas_dtrmm_oop_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha,        &
          &a, lda, b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, ldb, ldc
       double precision,intent(in)        :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       double precision,intent(in)        :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       double precision,intent(inout)     :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dtrmm_oop_omp_offload_ilp64

     subroutine mkl_blas_ctrmm_oop_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha,        &
          &a, lda, b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, ldb, ldc
       complex,intent(in)                 :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       complex,intent(in)                 :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       complex,intent(inout)              :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ctrmm_oop_omp_offload_ilp64

     subroutine mkl_blas_ztrmm_oop_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha,        &
          &a, lda, b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, ldb, ldc
       complex*16,intent(in)              :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       complex*16,intent(in)              :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       complex*16,intent(inout)           :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ztrmm_oop_omp_offload_ilp64

     subroutine mkl_blas_dtrsm_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda,        &
          &b, ldb, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, ldb
       double precision,intent(in)        :: alpha
       !dec$ attributes no_arg_check      :: a
       double precision,intent(in)        :: a( lda, * )
       !dec$ attributes no_arg_check      :: b
       double precision,intent(inout)     :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dtrsm_omp_offload_ilp64

     subroutine mkl_blas_strsm_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda,        &
          &b, ldb, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: side, uplo, trans, diag
       integer,intent(in)                  :: m, n, lda, ldb
       real,intent(in)                     :: alpha
       !dec$ attributes no_arg_check       :: a
       real,intent(in)                     :: a( lda, * )
       !dec$ attributes no_arg_check       :: b
       real,intent(inout)                  :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_strsm_omp_offload_ilp64

     subroutine mkl_blas_ztrsm_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda,        &
          &b, ldb, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, ldb
       complex*16,intent(in)              :: alpha
       !dec$ attributes no_arg_check      :: a
       complex*16,intent(in)              :: a( lda, * )
       !dec$ attributes no_arg_check      :: b
       complex*16,intent(inout)           :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ztrsm_omp_offload_ilp64

     subroutine mkl_blas_ctrsm_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda,        &
          &b, ldb, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: side, uplo, trans, diag
       integer,intent(in)                  :: m, n, lda, ldb
       complex,intent(in)                  :: alpha
       !dec$ attributes no_arg_check       :: a
       complex,intent(in)                  :: a( lda, * )
       !dec$ attributes no_arg_check       :: b
       complex,intent(inout)               :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ctrsm_omp_offload_ilp64

     subroutine mkl_blas_strsm_oop_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha,        &
          &a, lda, b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, ldb, ldc
       real,intent(in)                    :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       real,intent(in)                    :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       real,intent(inout)                 :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_strsm_oop_omp_offload_ilp64

     subroutine mkl_blas_dtrsm_oop_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha,        &
          &a, lda, b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, ldb, ldc
       double precision,intent(in)        :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       double precision,intent(in)        :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       double precision,intent(inout)     :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dtrsm_oop_omp_offload_ilp64

     subroutine mkl_blas_ctrsm_oop_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha,        &
          &a, lda, b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, ldb, ldc
       complex,intent(in)                 :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       complex,intent(in)                 :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       complex,intent(inout)              :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ctrsm_oop_omp_offload_ilp64

     subroutine mkl_blas_ztrsm_oop_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha,        &
          &a, lda, b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, ldb, ldc
       complex*16,intent(in)              :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       complex*16,intent(in)              :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       complex*16,intent(inout)           :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ztrsm_oop_omp_offload_ilp64

     subroutine mkl_blas_dgemmt_omp_offload_ilp64 ( uplo, transa, transb, m, n, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: uplo, transa, transb
       integer,intent(in)                 :: m, n, lda, ldb, ldc
       double precision,intent(in)        :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       double precision,intent(in)        :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       double precision,intent(inout)     :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dgemmt_omp_offload_ilp64

     subroutine mkl_blas_sgemmt_omp_offload_ilp64 ( uplo, transa, transb, m, n, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo, transa, transb
       integer,intent(in)                  :: m, n, lda, ldb, ldc
       real,intent(in)                     :: alpha, beta
       !dec$ attributes no_arg_check       :: a, b
       real,intent(in)                     :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       real,intent(inout)                  :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sgemmt_omp_offload_ilp64

     subroutine mkl_blas_zgemmt_omp_offload_ilp64 ( uplo, transa, transb, m, n, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: uplo, transa, transb
       integer,intent(in)                 :: m, n, lda, ldb, ldc
       complex*16,intent(in)              :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       complex*16,intent(in)              :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       complex*16,intent(inout)           :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zgemmt_omp_offload_ilp64

     subroutine mkl_blas_cgemmt_omp_offload_ilp64 ( uplo, transa, transb, m, n, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo, transa, transb
       integer,intent(in)                  :: m, n, lda, ldb, ldc
       complex,intent(in)                  :: alpha, beta
       !dec$ attributes no_arg_check       :: a, b
       complex,intent(in)                  :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       complex,intent(inout)               :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cgemmt_omp_offload_ilp64

     ! BLAS Level2

     subroutine mkl_blas_dgbmv_omp_offload_ilp64 ( trans, m, n, kl, ku, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: alpha, beta
       integer,intent(in)                  :: incx, incy, kl, ku, lda, m, n
       character*1,intent(in)              :: trans
       !dec$ attributes no_arg_check       :: a, x
       double precision,intent(in)         :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       double precision,intent(inout)      :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dgbmv_omp_offload_ilp64

     subroutine mkl_blas_sgbmv_omp_offload_ilp64 ( trans, m, n, kl, ku, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: alpha, beta
       integer,intent(in)                  :: incx, incy, kl, ku, lda, m, n
       character*1,intent(in)              :: trans
       !dec$ attributes no_arg_check       :: a, x
       real,intent(in)                     :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       real,intent(inout)                  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sgbmv_omp_offload_ilp64

     subroutine mkl_blas_zgbmv_omp_offload_ilp64 ( trans, m, n, kl, ku, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)               :: alpha, beta
       integer,intent(in)                  :: incx, incy, kl, ku, lda, m, n
       character*1,intent(in)              :: trans
       !dec$ attributes no_arg_check       :: a, x
       complex*16,intent(in)               :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       complex*16,intent(inout)            :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zgbmv_omp_offload_ilp64

     subroutine mkl_blas_cgbmv_omp_offload_ilp64 ( trans, m, n, kl, ku, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                  :: alpha, beta
       integer,intent(in)                  :: incx, incy, kl, ku, lda, m, n
       character*1,intent(in)              :: trans
       !dec$ attributes no_arg_check       :: a, x
       complex,intent(in)                  :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       complex,intent(inout)               :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cgbmv_omp_offload_ilp64

     subroutine mkl_blas_dgemv_omp_offload_ilp64 ( trans, m, n, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: alpha, beta
       integer,intent(in)                  :: incx, incy, lda, m, n
       character*1,intent(in)              :: trans
       !dec$ attributes no_arg_check       :: a, x
       double precision,intent(in)         :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       double precision,intent(inout)      :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dgemv_omp_offload_ilp64

     subroutine mkl_blas_sgemv_omp_offload_ilp64 ( trans, m, n, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: alpha, beta
       integer,intent(in)                  :: incx, incy, lda, m, n
       character*1,intent(in)              :: trans
       !dec$ attributes no_arg_check       :: a, x
       real,intent(in)                     :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       real,intent(inout)                  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sgemv_omp_offload_ilp64

     subroutine mkl_blas_zgemv_omp_offload_ilp64 ( trans, m, n, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)               :: alpha, beta
       integer,intent(in)                  :: incx, incy, lda, m, n
       character*1,intent(in)              :: trans
       !dec$ attributes no_arg_check       :: a, x
       complex*16,intent(in)               :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       complex*16,intent(inout)            :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zgemv_omp_offload_ilp64

     subroutine mkl_blas_cgemv_omp_offload_ilp64 ( trans, m, n, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                  :: alpha, beta
       integer,intent(in)                  :: incx, incy, lda, m, n
       character*1,intent(in)              :: trans
       !dec$ attributes no_arg_check       :: a, x
       complex,intent(in)                  :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       complex,intent(inout)               :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cgemv_omp_offload_ilp64

     subroutine mkl_blas_dsbmv_omp_offload_ilp64 ( uplo, n, k, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: alpha, beta
       integer,intent(in)                  :: incx, incy, k, lda, n
       character*1,intent(in)              :: uplo
       !dec$ attributes no_arg_check       :: a, x
       double precision,intent(in)         :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       double precision,intent(inout)      :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dsbmv_omp_offload_ilp64

     subroutine mkl_blas_ssbmv_omp_offload_ilp64 ( uplo, n, k, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: alpha, beta
       integer,intent(in)                  :: incx, incy, k, lda, n
       character*1,intent(in)              :: uplo
       !dec$ attributes no_arg_check       :: a, x
       real,intent(in)                     :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       real,intent(inout)                  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ssbmv_omp_offload_ilp64

     subroutine mkl_blas_zhbmv_omp_offload_ilp64 ( uplo, n, k, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)               :: alpha, beta
       integer,intent(in)                  :: incx, incy, k, lda, n
       character*1,intent(in)              :: uplo
       !dec$ attributes no_arg_check       :: a, x
       complex*16,intent(in)               :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       complex*16,intent(inout)            :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zhbmv_omp_offload_ilp64

     subroutine mkl_blas_chbmv_omp_offload_ilp64 ( uplo, n, k, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                  :: alpha, beta
       integer,intent(in)                  :: incx, incy, k, lda, n
       character*1,intent(in)              :: uplo
       !dec$ attributes no_arg_check       :: a, x
       complex,intent(in)                  :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       complex,intent(inout)               :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_chbmv_omp_offload_ilp64

     subroutine mkl_blas_dsymv_omp_offload_ilp64 ( uplo, n, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: alpha, beta
       integer,intent(in)                  :: incx, incy, lda, n
       character*1,intent(in)              :: uplo
       !dec$ attributes no_arg_check       :: a, x
       double precision,intent(in)         :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       double precision,intent(inout)      :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dsymv_omp_offload_ilp64

     subroutine mkl_blas_ssymv_omp_offload_ilp64 ( uplo, n, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: alpha, beta
       integer,intent(in)                  :: incx, incy, lda, n
       character*1,intent(in)              :: uplo
       !dec$ attributes no_arg_check       :: a, x
       real,intent(in)                     :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       real,intent(inout)                  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ssymv_omp_offload_ilp64

     subroutine mkl_blas_zhemv_omp_offload_ilp64 ( uplo, n, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)               :: alpha, beta
       integer,intent(in)                  :: incx, incy, lda, n
       character*1,intent(in)              :: uplo
       !dec$ attributes no_arg_check       :: a, x
       complex*16,intent(in)               :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       complex*16,intent(inout)            :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zhemv_omp_offload_ilp64

     subroutine mkl_blas_chemv_omp_offload_ilp64 ( uplo, n, alpha, a, lda, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                  :: alpha, beta
       integer,intent(in)                  :: incx, incy, lda, n
       character*1,intent(in)              :: uplo
       !dec$ attributes no_arg_check       :: a, x
       complex,intent(in)                  :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       complex,intent(inout)               :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_chemv_omp_offload_ilp64

     subroutine mkl_blas_dspmv_omp_offload_ilp64 ( uplo, n, alpha, a, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: alpha, beta
       integer,intent(in)                  :: incx, incy, n
       character*1,intent(in)              :: uplo
       !dec$ attributes no_arg_check       :: a, x
       double precision,intent(in)         :: a( * ), x( * )
       !dec$ attributes no_arg_check       :: y
       double precision,intent(inout)      :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dspmv_omp_offload_ilp64

     subroutine mkl_blas_sspmv_omp_offload_ilp64 ( uplo, n, alpha, a, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: alpha, beta
       integer,intent(in)                  :: incx, incy, n
       character*1,intent(in)              :: uplo
       !dec$ attributes no_arg_check       :: a, x
       real,intent(in)                     :: a( * ), x( * )
       !dec$ attributes no_arg_check       :: y
       real,intent(inout)                  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sspmv_omp_offload_ilp64

     subroutine mkl_blas_zhpmv_omp_offload_ilp64 ( uplo, n, alpha, a, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)               :: alpha, beta
       integer,intent(in)                  :: incx, incy, n
       character*1,intent(in)              :: uplo
       !dec$ attributes no_arg_check       :: a, x
       complex*16,intent(in)               :: a( * ), x( * )
       !dec$ attributes no_arg_check       :: y
       complex*16,intent(inout)            :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zhpmv_omp_offload_ilp64

     subroutine mkl_blas_chpmv_omp_offload_ilp64 ( uplo, n, alpha, a, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                  :: alpha, beta
       integer,intent(in)                  :: incx, incy, n
       character*1,intent(in)              :: uplo
       !dec$ attributes no_arg_check       :: a, x
       complex,intent(in)                  :: a( * ), x( * )
       !dec$ attributes no_arg_check       :: y
       complex,intent(inout)               :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_chpmv_omp_offload_ilp64

     subroutine mkl_blas_dger_omp_offload_ilp64 ( m, n, alpha, x, incx,   &
          &y, incy, a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: alpha
       integer,intent(in)                  :: incx, incy, lda, m, n
       !dec$ attributes no_arg_check       :: y, x
       double precision,intent(in)         :: y( * ), x( * )
       !dec$ attributes no_arg_check       :: a
       double precision,intent(inout)      :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dger_omp_offload_ilp64

     subroutine mkl_blas_sger_omp_offload_ilp64 ( m, n, alpha, x, incx,   &
          &y, incy, a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: alpha
       integer,intent(in)                  :: incx, incy, lda, m, n
       !dec$ attributes no_arg_check       :: y, x
       real,intent(in)                     :: y( * ), x( * )
       !dec$ attributes no_arg_check       :: a
       real,intent(inout)                  :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sger_omp_offload_ilp64

     subroutine mkl_blas_zgerc_omp_offload_ilp64 ( m, n, alpha, x, incx,   &
          &y, incy, a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)               :: alpha
       integer,intent(in)                  :: incx, incy, lda, m, n
       !dec$ attributes no_arg_check       :: y, x
       complex*16,intent(in)               :: y( * ), x( * )
       !dec$ attributes no_arg_check       :: a
       complex*16,intent(inout)            :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zgerc_omp_offload_ilp64

     subroutine mkl_blas_cgerc_omp_offload_ilp64 ( m, n, alpha, x, incx,   &
          &y, incy, a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                  :: alpha
       integer,intent(in)                  :: incx, incy, lda, m, n
       !dec$ attributes no_arg_check       :: y, x
       complex,intent(in)                  :: y( * ), x( * )
       !dec$ attributes no_arg_check       :: a
       complex,intent(inout)               :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cgerc_omp_offload_ilp64

     subroutine mkl_blas_zgeru_omp_offload_ilp64 ( m, n, alpha, x, incx,   &
          &y, incy, a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)               :: alpha
       integer,intent(in)                  :: incx, incy, lda, m, n
       !dec$ attributes no_arg_check       :: y, x
       complex*16,intent(in)               :: y( * ), x( * )
       !dec$ attributes no_arg_check       :: a
       complex*16,intent(inout)            :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zgeru_omp_offload_ilp64

     subroutine mkl_blas_cgeru_omp_offload_ilp64 ( m, n, alpha, x, incx,   &
          &y, incy, a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                  :: alpha
       integer,intent(in)                  :: incx, incy, lda, m, n
       !dec$ attributes no_arg_check       :: y, x
       complex,intent(in)                  :: y( * ), x( * )
       !dec$ attributes no_arg_check       :: a
       complex,intent(inout)               :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cgeru_omp_offload_ilp64

     subroutine mkl_blas_dsyr_omp_offload_ilp64 ( uplo, n, alpha, x, incx,   &
          &a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       double precision,intent(in)         :: alpha
       integer,intent(in)                  :: incx, lda, n
       !dec$ attributes no_arg_check       :: x
       double precision,intent(in)         :: x( * )
       !dec$ attributes no_arg_check       :: a
       double precision,intent(inout)      :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dsyr_omp_offload_ilp64

     subroutine mkl_blas_ssyr_omp_offload_ilp64 ( uplo, n, alpha, x, incx,   &
          &a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       real,intent(in)                     :: alpha
       integer,intent(in)                  :: incx, lda, n
       !dec$ attributes no_arg_check       :: x
       real,intent(in)                     :: x( * )
       !dec$ attributes no_arg_check       :: a
       real,intent(inout)                  :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ssyr_omp_offload_ilp64

     subroutine mkl_blas_zher_omp_offload_ilp64 ( uplo, n, alpha, x, incx,   &
          &a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       double precision,intent(in)         :: alpha
       integer,intent(in)                  :: incx, lda, n
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(in)               :: x( * )
       !dec$ attributes no_arg_check       :: a
       complex*16,intent(inout)            :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zher_omp_offload_ilp64

     subroutine mkl_blas_cher_omp_offload_ilp64 ( uplo, n, alpha, x, incx,   &
          &a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       real,intent(in)                     :: alpha
       integer,intent(in)                  :: incx, lda, n
       !dec$ attributes no_arg_check       :: x
       complex,intent(in)                  :: x( * )
       !dec$ attributes no_arg_check       :: a
       complex,intent(inout)               :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cher_omp_offload_ilp64

     subroutine mkl_blas_dspr_omp_offload_ilp64 ( uplo, n, alpha, x, incx,   &
          &a, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       double precision,intent(in)         :: alpha
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       double precision,intent(in)         :: x( * )
       !dec$ attributes no_arg_check       :: a
       double precision,intent(inout)      :: a( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dspr_omp_offload_ilp64

     subroutine mkl_blas_sspr_omp_offload_ilp64 ( uplo, n, alpha, x, incx,   &
          &a, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       real,intent(in)                     :: alpha
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       real,intent(in)                     :: x( * )
       !dec$ attributes no_arg_check       :: a
       real,intent(inout)                  :: a( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sspr_omp_offload_ilp64

     subroutine mkl_blas_zhpr_omp_offload_ilp64 ( uplo, n, alpha, x, incx,   &
          &a, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       double precision,intent(in)         :: alpha
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(in)               :: x( * )
       !dec$ attributes no_arg_check       :: a
       complex*16,intent(inout)            :: a( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zhpr_omp_offload_ilp64

     subroutine mkl_blas_chpr_omp_offload_ilp64 ( uplo, n, alpha, x, incx,   &
          &a, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       real,intent(in)                     :: alpha
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex,intent(in)                  :: x( * )
       !dec$ attributes no_arg_check       :: a
       complex,intent(inout)               :: a( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_chpr_omp_offload_ilp64

     subroutine mkl_blas_dsyr2_omp_offload_ilp64 ( uplo, n, alpha, x, incx, y, incy,   &
          &a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       double precision,intent(in)         :: alpha
       integer,intent(in)                  :: incx, incy, lda, n
       !dec$ attributes no_arg_check       :: x, y
       double precision,intent(in)         :: x( * ), y( * )
       !dec$ attributes no_arg_check       :: a
       double precision,intent(inout)      :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dsyr2_omp_offload_ilp64

     subroutine mkl_blas_ssyr2_omp_offload_ilp64 ( uplo, n, alpha, x, incx, y, incy,   &
          &a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       real,intent(in)                     :: alpha
       integer,intent(in)                  :: incx, incy, lda, n
       !dec$ attributes no_arg_check       :: x, y
       real,intent(in)                     :: x( * ), y( * )
       !dec$ attributes no_arg_check       :: a
       real,intent(inout)                  :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ssyr2_omp_offload_ilp64

     subroutine mkl_blas_zher2_omp_offload_ilp64 ( uplo, n, alpha, x, incx, y, incy,   &
          &a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       complex*16,intent(in)               :: alpha
       integer,intent(in)                  :: incx, incy, lda, n
       !dec$ attributes no_arg_check       :: x, y
       complex*16,intent(in)               :: x( * ), y( * )
       !dec$ attributes no_arg_check       :: a
       complex*16,intent(inout)            :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zher2_omp_offload_ilp64

     subroutine mkl_blas_cher2_omp_offload_ilp64 ( uplo, n, alpha, x, incx, y, incy,   &
          &a, lda, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       complex,intent(in)                  :: alpha
       integer,intent(in)                  :: incx, incy, lda, n
       !dec$ attributes no_arg_check       :: x, y
       complex,intent(in)                  :: x( * ), y( * )
       !dec$ attributes no_arg_check       :: a
       complex,intent(inout)               :: a( lda, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cher2_omp_offload_ilp64

     subroutine mkl_blas_dspr2_omp_offload_ilp64 ( uplo, n, alpha, x, incx, y, incy,   &
          &a, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       double precision,intent(in)         :: alpha
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       double precision,intent(in)         :: x( * ), y( * )
       !dec$ attributes no_arg_check       :: a
       double precision,intent(inout)      :: a( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dspr2_omp_offload_ilp64

     subroutine mkl_blas_sspr2_omp_offload_ilp64 ( uplo, n, alpha, x, incx, y, incy,   &
          &a, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       real,intent(in)                     :: alpha
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       real,intent(in)                     :: x( * ), y( * )
       !dec$ attributes no_arg_check       :: a
       real,intent(inout)                  :: a( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sspr2_omp_offload_ilp64

     subroutine mkl_blas_zhpr2_omp_offload_ilp64 ( uplo, n, alpha, x, incx, y, incy,   &
          &a, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       complex*16,intent(in)               :: alpha
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       complex*16,intent(in)               :: x( * ), y( * )
       !dec$ attributes no_arg_check       :: a
       complex*16,intent(inout)            :: a( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zhpr2_omp_offload_ilp64

     subroutine mkl_blas_chpr2_omp_offload_ilp64 ( uplo, n, alpha, x, incx, y, incy,   &
          &a, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo
       complex,intent(in)                  :: alpha
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       complex,intent(in)                  :: x( * ), y( * )
       !dec$ attributes no_arg_check       :: a
       complex,intent(inout)               :: a( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_chpr2_omp_offload_ilp64

     subroutine mkl_blas_dtbmv_omp_offload_ilp64 ( uplo, trans, diag, n, k, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, k, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       double precision,intent(in)         :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       double precision,intent(inout)      :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dtbmv_omp_offload_ilp64

     subroutine mkl_blas_stbmv_omp_offload_ilp64 ( uplo, trans, diag, n, k, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, k, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       real,intent(in)                     :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       real,intent(inout)                  :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_stbmv_omp_offload_ilp64

     subroutine mkl_blas_ztbmv_omp_offload_ilp64 ( uplo, trans, diag, n, k, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, k, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       complex*16,intent(in)               :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(inout)            :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ztbmv_omp_offload_ilp64

     subroutine mkl_blas_ctbmv_omp_offload_ilp64 ( uplo, trans, diag, n, k, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, k, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       complex,intent(in)                  :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       complex,intent(inout)               :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ctbmv_omp_offload_ilp64

     subroutine mkl_blas_dtrmv_omp_offload_ilp64 ( uplo, trans, diag, n, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       double precision,intent(in)         :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       double precision,intent(inout)      :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dtrmv_omp_offload_ilp64

     subroutine mkl_blas_strmv_omp_offload_ilp64 ( uplo, trans, diag, n, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       real,intent(in)                     :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       real,intent(inout)                  :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_strmv_omp_offload_ilp64

     subroutine mkl_blas_ztrmv_omp_offload_ilp64 ( uplo, trans, diag, n, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       complex*16,intent(in)               :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(inout)            :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ztrmv_omp_offload_ilp64

     subroutine mkl_blas_ctrmv_omp_offload_ilp64 ( uplo, trans, diag, n, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       complex,intent(in)                  :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       complex,intent(inout)               :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ctrmv_omp_offload_ilp64

     subroutine mkl_blas_dtpmv_omp_offload_ilp64 ( uplo, trans, diag, n, a, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       double precision,intent(in)         :: a( * )
       !dec$ attributes no_arg_check       :: x
       double precision,intent(inout)      :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dtpmv_omp_offload_ilp64

     subroutine mkl_blas_stpmv_omp_offload_ilp64 ( uplo, trans, diag, n, a, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       real,intent(in)                     :: a( * )
       !dec$ attributes no_arg_check       :: x
       real,intent(inout)                  :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_stpmv_omp_offload_ilp64

     subroutine mkl_blas_ztpmv_omp_offload_ilp64 ( uplo, trans, diag, n, a, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       complex*16,intent(in)               :: a( * )
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(inout)            :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ztpmv_omp_offload_ilp64

     subroutine mkl_blas_ctpmv_omp_offload_ilp64 ( uplo, trans, diag, n, a, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       complex,intent(in)                  :: a( * )
       !dec$ attributes no_arg_check       :: x
       complex,intent(inout)               :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ctpmv_omp_offload_ilp64

     subroutine mkl_blas_dtbsv_omp_offload_ilp64 ( uplo, trans, diag, n, k, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, k, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       double precision,intent(in)         :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       double precision,intent(inout)      :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dtbsv_omp_offload_ilp64

     subroutine mkl_blas_stbsv_omp_offload_ilp64 ( uplo, trans, diag, n, k, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, k, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       real,intent(in)                     :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       real,intent(inout)                  :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_stbsv_omp_offload_ilp64

     subroutine mkl_blas_ztbsv_omp_offload_ilp64 ( uplo, trans, diag, n, k, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, k, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       complex*16,intent(in)               :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(inout)            :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ztbsv_omp_offload_ilp64

     subroutine mkl_blas_ctbsv_omp_offload_ilp64 ( uplo, trans, diag, n, k, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, k, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       complex,intent(in)                  :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       complex,intent(inout)               :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ctbsv_omp_offload_ilp64

     subroutine mkl_blas_dtrsv_omp_offload_ilp64 ( uplo, trans, diag, n, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       double precision,intent(in)         :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       double precision,intent(inout)      :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dtrsv_omp_offload_ilp64

     subroutine mkl_blas_strsv_omp_offload_ilp64 ( uplo, trans, diag, n, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       real,intent(in)                     :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       real,intent(inout)                  :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_strsv_omp_offload_ilp64

     subroutine mkl_blas_ztrsv_omp_offload_ilp64 ( uplo, trans, diag, n, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       complex*16,intent(in)               :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(inout)            :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ztrsv_omp_offload_ilp64

     subroutine mkl_blas_ctrsv_omp_offload_ilp64 ( uplo, trans, diag, n, a, lda, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, lda, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       complex,intent(in)                  :: a( lda, * )
       !dec$ attributes no_arg_check       :: x
       complex,intent(inout)               :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ctrsv_omp_offload_ilp64

     subroutine mkl_blas_dtpsv_omp_offload_ilp64 ( uplo, trans, diag, n, a, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       double precision,intent(in)         :: a( * )
       !dec$ attributes no_arg_check       :: x
       double precision,intent(inout)      :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dtpsv_omp_offload_ilp64

     subroutine mkl_blas_stpsv_omp_offload_ilp64 ( uplo, trans, diag, n, a, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       real,intent(in)                     :: a( * )
       !dec$ attributes no_arg_check       :: x
       real,intent(inout)                  :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_stpsv_omp_offload_ilp64

     subroutine mkl_blas_ztpsv_omp_offload_ilp64 ( uplo, trans, diag, n, a, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       complex*16,intent(in)               :: a( * )
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(inout)            :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ztpsv_omp_offload_ilp64

     subroutine mkl_blas_ctpsv_omp_offload_ilp64 ( uplo, trans, diag, n, a, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, n
       character*1,intent(in)              :: uplo, trans, diag
       !dec$ attributes no_arg_check       :: a
       complex,intent(in)                  :: a( * )
       !dec$ attributes no_arg_check       :: x
       complex,intent(inout)               :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ctpsv_omp_offload_ilp64

     ! BLAS Level1

     function mkl_blas_dasum_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision                    :: mkl_blas_dasum_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       double precision,intent(in)         :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_dasum_omp_offload_ilp64

     function mkl_blas_sasum_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real                                :: mkl_blas_sasum_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       real,intent(in)                     :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_sasum_omp_offload_ilp64

     function mkl_blas_dzasum_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision                    :: mkl_blas_dzasum_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(in)               :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_dzasum_omp_offload_ilp64

     function mkl_blas_scasum_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real                                :: mkl_blas_scasum_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex,intent(in)                  :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_scasum_omp_offload_ilp64

     function mkl_blas_dnrm2_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision                    :: mkl_blas_dnrm2_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       double precision,intent(in)         :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_dnrm2_omp_offload_ilp64

     function mkl_blas_snrm2_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real                                :: mkl_blas_snrm2_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       real,intent(in)                     :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_snrm2_omp_offload_ilp64

     function mkl_blas_dznrm2_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision                    :: mkl_blas_dznrm2_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(in)               :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_dznrm2_omp_offload_ilp64

     function mkl_blas_scnrm2_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real                                :: mkl_blas_scnrm2_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex,intent(in)                  :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_scnrm2_omp_offload_ilp64

     subroutine mkl_blas_daxpy_omp_offload_ilp64 ( n, alpha, x, incx,   &
          &y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: alpha
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x
       double precision,intent(in)         :: x( * )
       !dec$ attributes no_arg_check       :: y
       double precision,intent(inout)      :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_daxpy_omp_offload_ilp64

     subroutine mkl_blas_saxpy_omp_offload_ilp64 ( n, alpha, x, incx,   &
          &y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: alpha
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x
       real,intent(in)                     :: x( * )
       !dec$ attributes no_arg_check       :: y
       real,intent(inout)                  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_saxpy_omp_offload_ilp64

     subroutine mkl_blas_zaxpy_omp_offload_ilp64 ( n, alpha, x, incx,   &
          &y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)               :: alpha
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(in)               :: x( * )
       !dec$ attributes no_arg_check       :: y
       complex*16,intent(inout)            :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zaxpy_omp_offload_ilp64

     subroutine mkl_blas_caxpy_omp_offload_ilp64 ( n, alpha, x, incx,   &
          &y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                  :: alpha
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x
       complex,intent(in)                  :: x( * )
       !dec$ attributes no_arg_check       :: y
       complex,intent(inout)               :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_caxpy_omp_offload_ilp64

     subroutine mkl_blas_daxpby_omp_offload_ilp64 ( n, alpha, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: alpha, beta
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x
       double precision,intent(in)         :: x( * )
       !dec$ attributes no_arg_check       :: y
       double precision,intent(inout)      :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_daxpby_omp_offload_ilp64

     subroutine mkl_blas_saxpby_omp_offload_ilp64 ( n, alpha, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: alpha, beta
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x
       real,intent(in)                     :: x( * )
       !dec$ attributes no_arg_check       :: y
       real,intent(inout)                  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_saxpby_omp_offload_ilp64

     subroutine mkl_blas_zaxpby_omp_offload_ilp64 ( n, alpha, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)               :: alpha, beta
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(in)               :: x( * )
       !dec$ attributes no_arg_check       :: y
       complex*16,intent(inout)            :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zaxpby_omp_offload_ilp64

     subroutine mkl_blas_caxpby_omp_offload_ilp64 ( n, alpha, x, incx,   &
          &beta, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                  :: alpha, beta
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x
       complex,intent(in)                  :: x( * )
       !dec$ attributes no_arg_check       :: y
       complex,intent(inout)               :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_caxpby_omp_offload_ilp64

     subroutine mkl_blas_dcopy_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x
       double precision,intent(in)         :: x( * )
       !dec$ attributes no_arg_check       :: y
       double precision,intent(inout)      :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dcopy_omp_offload_ilp64

     subroutine mkl_blas_scopy_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x
       real,intent(in)                     :: x( * )
       !dec$ attributes no_arg_check       :: y
       real,intent(inout)                  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_scopy_omp_offload_ilp64

     subroutine mkl_blas_zcopy_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(in)               :: x( * )
       !dec$ attributes no_arg_check       :: y
       complex*16,intent(inout)            :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zcopy_omp_offload_ilp64

     subroutine mkl_blas_ccopy_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x
       complex,intent(in)                  :: x( * )
       !dec$ attributes no_arg_check       :: y
       complex,intent(inout)               :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ccopy_omp_offload_ilp64

     function mkl_blas_ddot_omp_offload_ilp64 ( n, x, incx, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision                    :: mkl_blas_ddot_omp_offload_ilp64
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       double precision,intent(in)         :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_ddot_omp_offload_ilp64

     function mkl_blas_sdot_omp_offload_ilp64 ( n, x, incx, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real                                :: mkl_blas_sdot_omp_offload_ilp64
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       real,intent(in)                     :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_sdot_omp_offload_ilp64

     function mkl_blas_dsdot_omp_offload_ilp64 ( n, x, incx, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision                    :: mkl_blas_dsdot_omp_offload_ilp64
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       real,intent(in)                     :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_dsdot_omp_offload_ilp64

     function mkl_blas_sdsdot_omp_offload_ilp64 ( n, sb, x, incx, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real                                :: mkl_blas_sdsdot_omp_offload_ilp64
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       real,intent(in)                     :: sb, x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_sdsdot_omp_offload_ilp64

     function mkl_blas_zdotc_omp_offload_ilp64 ( n, x, incx, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16                          :: mkl_blas_zdotc_omp_offload_ilp64
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       complex*16,intent(in)               :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_zdotc_omp_offload_ilp64

     function mkl_blas_cdotc_omp_offload_ilp64 ( n, x, incx, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex                             :: mkl_blas_cdotc_omp_offload_ilp64
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       complex,intent(in)                  :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_cdotc_omp_offload_ilp64

     function mkl_blas_zdotu_omp_offload_ilp64 ( n, x, incx, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16                          :: mkl_blas_zdotu_omp_offload_ilp64
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       complex*16,intent(in)               :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_zdotu_omp_offload_ilp64

     function mkl_blas_cdotu_omp_offload_ilp64 ( n, x, incx, y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex                             :: mkl_blas_cdotu_omp_offload_ilp64
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       complex,intent(in)                  :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_cdotu_omp_offload_ilp64

     subroutine mkl_blas_drot_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, c, s, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: c, s
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       double precision,intent(inout)      :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_drot_omp_offload_ilp64

     subroutine mkl_blas_srot_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, c, s, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: c, s
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       real,intent(inout)                  :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_srot_omp_offload_ilp64

     subroutine mkl_blas_zdrot_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, c, s, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: c, s
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       complex*16,intent(inout)            :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zdrot_omp_offload_ilp64

     subroutine mkl_blas_zrot_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, c, s, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: c
       complex*16, intent(in)              :: s
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       complex*16,intent(inout)            :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zrot_omp_offload_ilp64

     subroutine mkl_blas_csrot_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, c, s, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: c, s
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       complex,intent(inout)               :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_csrot_omp_offload_ilp64

     subroutine mkl_blas_crot_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, c, s, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: c
       complex, intent(in)                 :: s
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       complex,intent(inout)               :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_crot_omp_offload_ilp64

     subroutine mkl_blas_drotg_omp_offload_ilp64 ( a, b, c, s, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       !dec$ attributes no_arg_check       :: a, b, c, s
       double precision,intent(inout)      :: a( * ), b( * ), c( * ), s( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_drotg_omp_offload_ilp64

     subroutine mkl_blas_srotg_omp_offload_ilp64 ( a, b, c, s, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       !dec$ attributes no_arg_check       :: a, b, c, s
       real,intent(inout)                  :: a( * ), b( * ), c( * ), s( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_srotg_omp_offload_ilp64

     subroutine mkl_blas_zrotg_omp_offload_ilp64 ( a, b, c, s, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       !dec$ attributes no_arg_check       :: a, b, s
       complex*16,intent(inout)            :: a( * ), b( * ), s( * )
       !dec$ attributes no_arg_check       :: c
       double precision,intent(inout)      :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zrotg_omp_offload_ilp64

     subroutine mkl_blas_crotg_omp_offload_ilp64 ( a, b, c, s, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       !dec$ attributes no_arg_check       :: a, b, s
       complex,intent(inout)               :: a( * ), b( * ), s( * )
       !dec$ attributes no_arg_check       :: c
       real,intent(inout)                  :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_crotg_omp_offload_ilp64

     subroutine mkl_blas_drotm_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, param, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: param
       double precision,intent(in)         :: param( * )
       !dec$ attributes no_arg_check       :: x, y
       double precision,intent(inout)      :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_drotm_omp_offload_ilp64

     subroutine mkl_blas_srotm_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, param, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: param
       real,intent(in)                     :: param( * )
       !dec$ attributes no_arg_check       :: x, y
       real,intent(inout)                  :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_srotm_omp_offload_ilp64

     subroutine mkl_blas_drotmg_omp_offload_ilp64 ( d1, d2, x1, x2, param, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       !dec$ attributes no_arg_check       :: x2
       double precision,intent(in)         :: x2
       !dec$ attributes no_arg_check       :: d1, d2, x1, param
       double precision,intent(inout)      :: d1( * ), d2( * ), x1( * ), param( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_drotmg_omp_offload_ilp64

     subroutine mkl_blas_srotmg_omp_offload_ilp64 ( d1, d2, x1, x2, param, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       !dec$ attributes no_arg_check       :: x2
       real,intent(in)                     :: x2
       !dec$ attributes no_arg_check       :: d1, d2, x1, param
       real,intent(inout)                  :: d1( * ), d2( * ), x1( * ), param( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_srotmg_omp_offload_ilp64

     subroutine mkl_blas_dscal_omp_offload_ilp64 ( n, alpha, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: alpha
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       double precision,intent(inout)      :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dscal_omp_offload_ilp64

     subroutine mkl_blas_sscal_omp_offload_ilp64 ( n, alpha, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: alpha
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       real,intent(inout)                  :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sscal_omp_offload_ilp64

     subroutine mkl_blas_zscal_omp_offload_ilp64 ( n, alpha, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)               :: alpha
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(inout)            :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zscal_omp_offload_ilp64

     subroutine mkl_blas_cscal_omp_offload_ilp64 ( n, alpha, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                  :: alpha
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex,intent(inout)               :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cscal_omp_offload_ilp64

     subroutine mkl_blas_zdscal_omp_offload_ilp64 ( n, alpha, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: alpha
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(inout)            :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zdscal_omp_offload_ilp64

     subroutine mkl_blas_csscal_omp_offload_ilp64 ( n, alpha, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: alpha
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex,intent(inout)               :: x( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_csscal_omp_offload_ilp64

     subroutine mkl_blas_dswap_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       double precision,intent(inout)      :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dswap_omp_offload_ilp64

     subroutine mkl_blas_sswap_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       real,intent(inout)                  :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sswap_omp_offload_ilp64

     subroutine mkl_blas_zswap_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       complex*16,intent(inout)            :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zswap_omp_offload_ilp64

     subroutine mkl_blas_cswap_omp_offload_ilp64 ( n, x, incx,   &
          &y, incy, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, incy, n
       !dec$ attributes no_arg_check       :: x, y
       complex,intent(inout)               :: x( * ), y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cswap_omp_offload_ilp64

     function mkl_blas_idamax_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer                             :: mkl_blas_idamax_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       double precision,intent(in)         :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_idamax_omp_offload_ilp64

     function mkl_blas_isamax_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer                             :: mkl_blas_isamax_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       real,intent(in)                     :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_isamax_omp_offload_ilp64

     function mkl_blas_izamax_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer                             :: mkl_blas_izamax_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(in)               :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_izamax_omp_offload_ilp64

     function mkl_blas_icamax_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer                             :: mkl_blas_icamax_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex,intent(in)                  :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_icamax_omp_offload_ilp64

     function mkl_blas_idamin_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer                             :: mkl_blas_idamin_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       double precision,intent(in)         :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_idamin_omp_offload_ilp64

     function mkl_blas_isamin_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer                             :: mkl_blas_isamin_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       real,intent(in)                     :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_isamin_omp_offload_ilp64

     function mkl_blas_izamin_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer                             :: mkl_blas_izamin_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(in)               :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_izamin_omp_offload_ilp64

     function mkl_blas_icamin_omp_offload_ilp64 ( n, x, incx, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer                             :: mkl_blas_icamin_omp_offload_ilp64
       integer,intent(in)                  :: incx, n
       !dec$ attributes no_arg_check       :: x
       complex,intent(in)                  :: x( * )
       type(c_ptr), intent(in), value :: interop
     end function mkl_blas_icamin_omp_offload_ilp64

     ! BLAS batch

     subroutine mkl_blas_dtrsm_batch_strided_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda, stridea,        &
          &b, ldb, strideb, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, stridea, ldb, strideb, batch_size
       double precision,intent(in)        :: alpha
       !dec$ attributes no_arg_check      :: a
       double precision,intent(in)        :: a( lda, * )
       !dec$ attributes no_arg_check      :: b
       double precision,intent(inout)     :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dtrsm_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_strsm_batch_strided_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda, stridea,        &
          &b, ldb, strideb, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: side, uplo, trans, diag
       integer,intent(in)                  :: m, n, lda, stridea, ldb, strideb, batch_size
       real,intent(in)                     :: alpha
       !dec$ attributes no_arg_check       :: a
       real,intent(in)                     :: a( lda, * )
       !dec$ attributes no_arg_check       :: b
       real,intent(inout)                  :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_strsm_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_ztrsm_batch_strided_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda, stridea,        &
          &b, ldb, strideb, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: side, uplo, trans, diag
       integer,intent(in)                 :: m, n, lda, stridea, ldb, strideb, batch_size
       complex*16,intent(in)              :: alpha
       !dec$ attributes no_arg_check      :: a
       complex*16,intent(in)              :: a( lda, * )
       !dec$ attributes no_arg_check      :: b
       complex*16,intent(inout)           :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ztrsm_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_ctrsm_batch_strided_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda, stridea,        &
          &b, ldb, strideb, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: side, uplo, trans, diag
       integer,intent(in)                  :: m, n, lda, stridea, ldb, strideb, batch_size
       complex,intent(in)                  :: alpha
       !dec$ attributes no_arg_check       :: a
       complex,intent(in)                  :: a( lda, * )
       !dec$ attributes no_arg_check       :: b
       complex,intent(inout)               :: b( ldb, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ctrsm_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_dgemm_batch_strided_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda, stridea,        &
          &b, ldb, strideb, beta, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: transa, transb
       integer,intent(in)                 :: m, n, k, lda, stridea, ldb, strideb, ldc, stridec, batch_size
       double precision,intent(in)        :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       double precision,intent(in)        :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       double precision,intent(inout)     :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dgemm_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_sgemm_batch_strided_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda, stridea,        &
          &b, ldb, strideb, beta, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: transa, transb
       integer,intent(in)                  :: m, n, k, lda, stridea, ldb, strideb, ldc, stridec, batch_size
       real,intent(in)                     :: alpha, beta
       !dec$ attributes no_arg_check       :: a, b
       real,intent(in)                     :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       real,intent(inout)                  :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sgemm_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_zgemm_batch_strided_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda, stridea,        &
          &b, ldb, strideb, beta, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: transa, transb
       integer,intent(in)                 :: m, n, k, lda, stridea, ldb, strideb, ldc, stridec, batch_size
       complex*16,intent(in)              :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       complex*16,intent(in)              :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       complex*16,intent(inout)           :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zgemm_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_cgemm_batch_strided_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda, stridea,        &
          &b, ldb, strideb, beta, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: transa, transb
       integer,intent(in)                  :: m, n, k, lda, stridea, ldb, strideb, ldc, stridec, batch_size
       complex,intent(in)                  :: alpha, beta
       !dec$ attributes no_arg_check       :: a, b
       complex,intent(in)                  :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       complex,intent(inout)               :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cgemm_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_zgemm3m_batch_strided_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda, stridea,        &
          &b, ldb, strideb, beta, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)             :: transa, transb
       integer,intent(in)                 :: m, n, k, lda, stridea, ldb, strideb, ldc, stridec, batch_size
       complex*16,intent(in)              :: alpha, beta
       !dec$ attributes no_arg_check      :: a, b
       complex*16,intent(in)              :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check      :: c
       complex*16,intent(inout)           :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zgemm3m_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_cgemm3m_batch_strided_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda, stridea,        &
          &b, ldb, strideb, beta, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: transa, transb
       integer,intent(in)                  :: m, n, k, lda, stridea, ldb, strideb, ldc, stridec, batch_size
       complex,intent(in)                  :: alpha, beta
       !dec$ attributes no_arg_check       :: a, b
       complex,intent(in)                  :: a( lda, * ), b( ldb, * )
       !dec$ attributes no_arg_check       :: c
       complex,intent(inout)               :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cgemm3m_batch_strided_omp_offload_ilp64
     
     subroutine mkl_blas_ssyrk_batch_strided_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda, stridea,        &
           beta, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo, trans
       integer,intent(in)                  :: n, k, lda, stridea, ldc, stridec, batch_size
       real,intent(in)                     :: alpha, beta
       !dec$ attributes no_arg_check       :: a
       real,intent(in)                     :: a( lda, * )
       !dec$ attributes no_arg_check       :: c
       real,intent(inout)                  :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ssyrk_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_dsyrk_batch_strided_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda, stridea,        &
           beta, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo, trans
       integer,intent(in)                  :: n, k, lda, stridea, ldc, stridec, batch_size
       double precision,intent(in)                     :: alpha, beta
       !dec$ attributes no_arg_check                   :: a
       double precision,intent(in)                     :: a( lda, * )
       !dec$ attributes no_arg_check                   :: c
       double precision,intent(inout)                  :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dsyrk_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_csyrk_batch_strided_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda, stridea,         &
           beta, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo, trans
       integer,intent(in)                  :: n, k, lda, stridea, ldc, stridec, batch_size
       complex,intent(in)                     :: alpha, beta
       !dec$ attributes no_arg_check          :: a
       complex,intent(in)                     :: a( lda, * )
       !dec$ attributes no_arg_check          :: c
       complex,intent(inout)                  :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_csyrk_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_zsyrk_batch_strided_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda, stridea,         &
           beta, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)              :: uplo, trans
       integer,intent(in)                  :: n, k, lda, stridea, ldc, stridec, batch_size
       complex*16,intent(in)                     :: alpha, beta
       !dec$ attributes no_arg_check             :: a
       complex*16,intent(in)                     :: a( lda, * )
       !dec$ attributes no_arg_check             :: c
       complex*16,intent(inout)                  :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zsyrk_batch_strided_omp_offload_ilp64

    subroutine mkl_blas_daxpy_batch_strided_omp_offload_ilp64 ( n, alpha, x, incx, stridex,   &
          &y, incy, stridey, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: alpha
       integer,intent(in)                  :: incx, stridex, incy, stridey, batch_size, n
       !dec$ attributes no_arg_check       :: x
       double precision,intent(in)         :: x( * )
       !dec$ attributes no_arg_check       :: y
       double precision,intent(inout)      :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_daxpy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_saxpy_batch_strided_omp_offload_ilp64 ( n, alpha, x, incx, stridex,   &
          &y, incy, stridey, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: alpha
       integer,intent(in)                  :: incx, stridex, incy, stridey, batch_size, n
       !dec$ attributes no_arg_check       :: x
       real,intent(in)                     :: x( * )
       !dec$ attributes no_arg_check       :: y
       real,intent(inout)                  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_saxpy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_zaxpy_batch_strided_omp_offload_ilp64 ( n, alpha, x, incx, stridex,   &
          &y, incy, stridey, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)               :: alpha
       integer,intent(in)                  :: incx, stridex, incy, stridey, batch_size, n
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(in)               :: x( * )
       !dec$ attributes no_arg_check       :: y
       complex*16,intent(inout)            :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zaxpy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_caxpy_batch_strided_omp_offload_ilp64 ( n, alpha, x, incx, stridex,   &
          &y, incy, stridey, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                  :: alpha
       integer,intent(in)                  :: incx, stridex, incy, stridey, batch_size, n
       !dec$ attributes no_arg_check       :: x
       complex,intent(in)                  :: x( * )
       !dec$ attributes no_arg_check       :: y
       complex,intent(inout)               :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_caxpy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_scopy_batch_strided_omp_offload_ilp64 ( n, x, incx, stridex,   &
          &y, incy, stridey, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, stridex, incy, stridey, batch_size, n
       !dec$ attributes no_arg_check       :: x
       real,intent(in)                     :: x( * )
       !dec$ attributes no_arg_check       :: y
       real,intent(inout)                  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_scopy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_dcopy_batch_strided_omp_offload_ilp64 ( n, x, incx, stridex,   &
          &y, incy, stridey, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, stridex, incy, stridey, batch_size, n
       !dec$ attributes no_arg_check       :: x
       double precision,intent(in)         :: x( * )
       !dec$ attributes no_arg_check       :: y
       double precision,intent(inout)      :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dcopy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_ccopy_batch_strided_omp_offload_ilp64 ( n, x, incx, stridex,   &
          &y, incy, stridey, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, stridex, incy, stridey, batch_size, n
       !dec$ attributes no_arg_check       :: x
       complex,intent(in)                  :: x( * )
       !dec$ attributes no_arg_check       :: y
       complex,intent(inout)               :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ccopy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_zcopy_batch_strided_omp_offload_ilp64 ( n, x, incx, stridex,   &
          &y, incy, stridey, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, stridex, incy, stridey, batch_size, n
       !dec$ attributes no_arg_check       :: x
       complex*16,intent(in)               :: x( * )
       !dec$ attributes no_arg_check       :: y
       complex*16,intent(inout)            :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zcopy_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_dgemv_batch_strided_omp_offload_ilp64 ( trans, m, n, alpha, a, lda, stridea, x, incx,   &
          &stridex, beta, y, incy, stridey, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)         :: alpha, beta
       integer,intent(in)                  :: incx, incy, lda, m, n
       integer,intent(in)                  :: stridea, stridex, stridey, batch_size
       character*1,intent(in)              :: trans
       !dec$ attributes no_arg_check       :: a, x
       double precision,intent(in)         :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       double precision,intent(inout)      :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dgemv_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_sgemv_batch_strided_omp_offload_ilp64 ( trans, m, n, alpha, a, lda, stridea, x, incx,   &
          &stridex, beta, y, incy, stridey, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                     :: alpha, beta
       integer,intent(in)                  :: incx, incy, lda, m, n
       integer,intent(in)                  :: stridea, stridex, stridey, batch_size
       character*1,intent(in)              :: trans
       !dec$ attributes no_arg_check       :: a, x
       real,intent(in)                     :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       real,intent(inout)                  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sgemv_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_zgemv_batch_strided_omp_offload_ilp64 ( trans, m, n, alpha, a, lda, stridea, x, incx,   &
          &stridex, beta, y, incy, stridey, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)               :: alpha, beta
       integer,intent(in)                  :: incx, incy, lda, m, n
       integer,intent(in)                  :: stridea, stridex, stridey, batch_size
       character*1,intent(in)              :: trans
       !dec$ attributes no_arg_check       :: a, x
       complex*16,intent(in)               :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       complex*16,intent(inout)            :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zgemv_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_cgemv_batch_strided_omp_offload_ilp64 ( trans, m, n, alpha, a, lda, stridea, x, incx,   &
          &stridex, beta, y, incy, stridey, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                  :: alpha, beta
       integer,intent(in)                  :: incx, incy, lda, m, n
       integer,intent(in)                  :: stridea, stridex, stridey, batch_size
       character*1,intent(in)              :: trans
       !dec$ attributes no_arg_check       :: a, x
       complex,intent(in)                  :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: y
       complex,intent(inout)               :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cgemv_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_ddgmm_batch_strided_omp_offload_ilp64 ( side, m, n, a, lda, stridea, x, incx,   &
          &stridex, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, ldc, lda, m, n
       integer,intent(in)                  :: stridea, stridex, stridec, batch_size
       character*1,intent(in)              :: side
       !dec$ attributes no_arg_check       :: a, x
       double precision,intent(in)         :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: c
       double precision,intent(inout)      :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ddgmm_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_sdgmm_batch_strided_omp_offload_ilp64 ( side, m, n, a, lda, stridea, x, incx,   &
          &stridex, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, ldc, lda, m, n
       integer,intent(in)                  :: stridea, stridex, stridec, batch_size
       character*1,intent(in)              :: side
       !dec$ attributes no_arg_check       :: a, x
       real,intent(in)                     :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: c
       real,intent(inout)                  :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sdgmm_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_zdgmm_batch_strided_omp_offload_ilp64 ( side, m, n, a, lda, stridea, x, incx,   &
          &stridex, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, ldc, lda, m, n
       integer,intent(in)                  :: stridea, stridex, stridec, batch_size
       character*1,intent(in)              :: side
       !dec$ attributes no_arg_check       :: a, x
       complex*16,intent(in)               :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: c
       complex*16,intent(inout)            :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zdgmm_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_cdgmm_batch_strided_omp_offload_ilp64 ( side, m, n, a, lda, stridea, x, incx,   &
          &stridex, c, ldc, stridec, batch_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                  :: incx, ldc, lda, m, n
       integer,intent(in)                  :: stridea, stridex, stridec, batch_size
       character*1,intent(in)              :: side
       !dec$ attributes no_arg_check       :: a, x
       complex,intent(in)                  :: a( lda, * ), x( * )
       !dec$ attributes no_arg_check       :: c
       complex,intent(inout)               :: c( ldc, * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cdgmm_batch_strided_omp_offload_ilp64

     subroutine mkl_blas_daxpy_batch_omp_offload_ilp64(n_array,alpha_array,x_array,incx_array,    &
          &y_array,incy_array,group_count,group_size_array,interop) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)             :: alpha_array(*)
       integer,intent(in)                      :: incx_array(*),incy_array(*),n_array(*)
       integer,intent(in)                      :: group_count,group_size_array(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: x_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  :: y_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_daxpy_batch_omp_offload_ilp64

     subroutine mkl_blas_saxpy_batch_omp_offload_ilp64(n_array,alpha_array,x_array,incx_array,    &
          &y_array,incy_array,group_count,group_size_array,interop) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                         :: alpha_array(*)
       integer,intent(in)                      :: incx_array(*),incy_array(*),n_array(*)
       integer,intent(in)                      :: group_count,group_size_array(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: x_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  :: y_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_saxpy_batch_omp_offload_ilp64

     subroutine mkl_blas_zaxpy_batch_omp_offload_ilp64(n_array,alpha_array,x_array,incx_array,    &
          &y_array,incy_array,group_count,group_size_array,interop) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)                   :: alpha_array(*)
       integer,intent(in)                      :: incx_array(*),incy_array(*),n_array(*)
       integer,intent(in)                      :: group_count,group_size_array(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: x_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  :: y_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zaxpy_batch_omp_offload_ilp64

     subroutine mkl_blas_caxpy_batch_omp_offload_ilp64(n_array,alpha_array,x_array,incx_array,    &
          &y_array,incy_array,group_count,group_size_array,interop) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                      :: alpha_array(*)
       integer,intent(in)                      :: incx_array(*),incy_array(*),n_array(*)
       integer,intent(in)                      :: group_count,group_size_array(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: x_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  :: y_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_caxpy_batch_omp_offload_ilp64

     subroutine mkl_blas_scopy_batch_omp_offload_ilp64(n_array,x_array,incx_array,    &
          &y_array,incy_array,group_count,group_size_array,interop) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                      :: incx_array(*),incy_array(*),n_array(*)
       integer,intent(in)                      :: group_count,group_size_array(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: x_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  :: y_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_scopy_batch_omp_offload_ilp64

     subroutine mkl_blas_dcopy_batch_omp_offload_ilp64(n_array,x_array,incx_array,    &
          &y_array,incy_array,group_count,group_size_array,interop) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                      :: incx_array(*),incy_array(*),n_array(*)
       integer,intent(in)                      :: group_count,group_size_array(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: x_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  :: y_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dcopy_batch_omp_offload_ilp64

     subroutine mkl_blas_ccopy_batch_omp_offload_ilp64(n_array,x_array,incx_array,    &
          &y_array,incy_array,group_count,group_size_array,interop) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                      :: incx_array(*),incy_array(*),n_array(*)
       integer,intent(in)                      :: group_count,group_size_array(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: x_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  :: y_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ccopy_batch_omp_offload_ilp64

     subroutine mkl_blas_zcopy_batch_omp_offload_ilp64(n_array,x_array,incx_array,    &
          &y_array,incy_array,group_count,group_size_array,interop) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                      :: incx_array(*),incy_array(*),n_array(*)
       integer,intent(in)                      :: group_count,group_size_array(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: x_array(*)
       integer(KIND=C_INTPTR_T),intent(inout)  :: y_array(*)
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zcopy_batch_omp_offload_ilp64

     subroutine mkl_blas_dtrsm_batch_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda,        &
          &b, ldb, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: side(*), uplo(*), trans(*), diag(*)
       integer,intent(in)                     :: m(*), n(*), lda(*), ldb(*)
       integer,intent(in)                     :: group_size(*), group_count
       double precision,intent(in)            :: alpha(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: b( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dtrsm_batch_omp_offload_ilp64

     subroutine mkl_blas_strsm_batch_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda,        &
          &b, ldb, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: side(*), uplo(*), trans(*), diag(*)
       integer,intent(in)                     :: m(*), n(*), lda(*), ldb(*)
       integer,intent(in)                     :: group_size(*), group_count
       real,intent(in)                        :: alpha(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: b( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_strsm_batch_omp_offload_ilp64

     subroutine mkl_blas_ztrsm_batch_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda,        &
          &b, ldb, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: side(*), uplo(*), trans(*), diag(*)
       integer,intent(in)                     :: m(*), n(*), lda(*), ldb(*)
       integer,intent(in)                     :: group_size(*), group_count
       complex*16,intent(in)                  :: alpha(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: b( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ztrsm_batch_omp_offload_ilp64

     subroutine mkl_blas_ctrsm_batch_omp_offload_ilp64 ( side, uplo, trans, diag, m, n, alpha, a, lda,        &
          &b, ldb, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: side(*), uplo(*), trans(*), diag(*)
       integer,intent(in)                     :: m(*), n(*), lda(*), ldb(*)
       integer,intent(in)                     :: group_size(*), group_count
       complex,intent(in)                     :: alpha(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: b( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ctrsm_batch_omp_offload_ilp64

     subroutine mkl_blas_dsyrk_batch_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          beta, c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: uplo(*), trans(*)
       integer,intent(in)                     :: n(*), k(*), lda(*), ldc(*)
       integer,intent(in)                     :: group_size(*), group_count
       double precision,intent(in)            :: alpha(*), beta(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dsyrk_batch_omp_offload_ilp64

     subroutine mkl_blas_ssyrk_batch_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          beta, c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: uplo(*), trans(*)
       integer,intent(in)                     :: n(*), k(*), lda(*), ldc(*)
       integer,intent(in)                     :: group_size(*), group_count
       real,intent(in)                        :: alpha(*), beta(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ssyrk_batch_omp_offload_ilp64

     subroutine mkl_blas_zsyrk_batch_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          beta, c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: uplo(*), trans(*)
       integer,intent(in)                     :: n(*), k(*), lda(*), ldc(*)
       integer,intent(in)                     :: group_size(*), group_count
       complex*16,intent(in)                  :: alpha(*), beta(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zsyrk_batch_omp_offload_ilp64

     subroutine mkl_blas_csyrk_batch_omp_offload_ilp64 ( uplo, trans, n, k, alpha, a, lda,        &
          beta, c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: uplo(*), trans(*)
       integer,intent(in)                     :: n(*), k(*), lda(*), ldc(*)
       integer,intent(in)                     :: group_size(*), group_count
       complex,intent(in)                     :: alpha(*), beta(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_csyrk_batch_omp_offload_ilp64

     subroutine mkl_blas_dgemm_batch_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: transa(*), transb(*)
       integer,intent(in)                     :: m(*), n(*), k(*), lda(*), ldb(*), ldc(*)
       integer,intent(in)                     :: group_size(*), group_count
       double precision,intent(in)            :: alpha(*), beta(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * ), b( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dgemm_batch_omp_offload_ilp64

     subroutine mkl_blas_sgemm_batch_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: transa(*), transb(*)
       integer,intent(in)                     :: m(*), n(*), k(*), lda(*), ldb(*), ldc(*)
       integer,intent(in)                     :: group_size(*), group_count
       real,intent(in)                        :: alpha(*), beta(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * ), b( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sgemm_batch_omp_offload_ilp64

     subroutine mkl_blas_zgemm_batch_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: transa(*), transb(*)
       integer,intent(in)                     :: m(*), n(*), k(*), lda(*), ldb(*), ldc(*)
       integer,intent(in)                     :: group_size(*), group_count
       complex*16,intent(in)                  :: alpha(*), beta(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * ), b( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zgemm_batch_omp_offload_ilp64

     subroutine mkl_blas_cgemm_batch_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: transa(*), transb(*)
       integer,intent(in)                     :: m(*), n(*), k(*), lda(*), ldb(*), ldc(*)
       integer,intent(in)                     :: group_size(*), group_count
       complex,intent(in)                     :: alpha(*), beta(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * ), b( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cgemm_batch_omp_offload_ilp64

     subroutine mkl_blas_zgemm3m_batch_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: transa(*), transb(*)
       integer,intent(in)                     :: m(*), n(*), k(*), lda(*), ldb(*), ldc(*)
       integer,intent(in)                     :: group_size(*), group_count
       complex*16,intent(in)                  :: alpha(*), beta(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * ), b( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zgemm3m_batch_omp_offload_ilp64

     subroutine mkl_blas_cgemm3m_batch_omp_offload_ilp64 ( transa, transb, m, n, k, alpha, a, lda,        &
          &b, ldb, beta, c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       character*1,intent(in)                 :: transa(*), transb(*)
       integer,intent(in)                     :: m(*), n(*), k(*), lda(*), ldb(*), ldc(*)
       integer,intent(in)                     :: group_size(*), group_count
       complex,intent(in)                     :: alpha(*), beta(*)
       integer(KIND=C_INTPTR_T),intent(in)    :: a( * ), b( * )
       integer(KIND=C_INTPTR_T),intent(inout) :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cgemm3m_batch_omp_offload_ilp64
     
     subroutine mkl_blas_dgemv_batch_omp_offload_ilp64 ( trans, m, n, alpha, a, lda, x, incx,   &
          &beta, y, incy, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       double precision,intent(in)             :: alpha(*), beta(*)
       integer,intent(in)                      :: incx(*), incy(*), lda(*), m(*), n(*)
       integer,intent(in)                      :: group_size(*), group_count
       character*1,intent(in)                  :: trans(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: a( * ), x( * )
       integer(KIND=C_INTPTR_T),intent(inout)  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_dgemv_batch_omp_offload_ilp64

     subroutine mkl_blas_sgemv_batch_omp_offload_ilp64 ( trans, m, n, alpha, a, lda, x, incx,   &
          &beta, y, incy, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       real,intent(in)                         :: alpha(*), beta(*)
       integer,intent(in)                      :: incx(*), incy(*), lda(*), m(*), n(*)
       integer,intent(in)                      :: group_size(*), group_count
       character*1,intent(in)                  :: trans(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: a( * ), x( * )
       integer(KIND=C_INTPTR_T),intent(inout)  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sgemv_batch_omp_offload_ilp64

     subroutine mkl_blas_zgemv_batch_omp_offload_ilp64 ( trans, m, n, alpha, a, lda, x, incx,   &
          &beta, y, incy, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex*16,intent(in)                   :: alpha(*), beta(*)
       integer,intent(in)                      :: incx(*), incy(*), lda(*), m(*), n(*)
       integer,intent(in)                      :: group_size(*), group_count
       character*1,intent(in)                  :: trans(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: a( * ), x( * )
       integer(KIND=C_INTPTR_T),intent(inout)  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zgemv_batch_omp_offload_ilp64

     subroutine mkl_blas_cgemv_batch_omp_offload_ilp64 ( trans, m, n, alpha, a, lda, x, incx,   &
          &beta, y, incy, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       complex,intent(in)                      :: alpha(*), beta(*)
       integer,intent(in)                      :: incx(*), incy(*), lda(*), m(*), n(*)
       integer,intent(in)                      :: group_size(*), group_count
       character*1,intent(in)                  :: trans(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: a( * ), x( * )
       integer(KIND=C_INTPTR_T),intent(inout)  :: y( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cgemv_batch_omp_offload_ilp64

     subroutine mkl_blas_ddgmm_batch_omp_offload_ilp64 ( side, m, n, a, lda, x, incx,   &
          &c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                      :: incx(*), ldc(*), lda(*), m(*), n(*)
       integer,intent(in)                      :: group_size(*), group_count
       character*1,intent(in)                  :: side(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: a( * ), x( * )
       integer(KIND=C_INTPTR_T),intent(inout)  :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_ddgmm_batch_omp_offload_ilp64

     subroutine mkl_blas_sdgmm_batch_omp_offload_ilp64 ( side, m, n, a, lda, x, incx,   &
          &c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                      :: incx(*), ldc(*), lda(*), m(*), n(*)
       integer,intent(in)                      :: group_size(*), group_count
       character*1,intent(in)                  :: side(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: a( * ), x( * )
       integer(KIND=C_INTPTR_T),intent(inout)  :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_sdgmm_batch_omp_offload_ilp64

     subroutine mkl_blas_zdgmm_batch_omp_offload_ilp64 ( side, m, n, a, lda, x, incx,   &
          &c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                      :: incx(*), ldc(*), lda(*), m(*), n(*)
       integer,intent(in)                      :: group_size(*), group_count
       character*1,intent(in)                  :: side(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: a( * ), x( * )
       integer(KIND=C_INTPTR_T),intent(inout)  :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_zdgmm_batch_omp_offload_ilp64

     subroutine mkl_blas_cdgmm_batch_omp_offload_ilp64 ( side, m, n, a, lda, x, incx,   &
          &c, ldc, group_count, group_size, interop ) BIND(C)
       use, intrinsic :: iso_c_binding
       integer,intent(in)                      :: incx(*), ldc(*), lda(*), m(*), n(*)
       integer,intent(in)                      :: group_size(*), group_count
       character*1,intent(in)                  :: side(*)
       integer(KIND=C_INTPTR_T),intent(in)     :: a( * ), x( * )
       integer(KIND=C_INTPTR_T),intent(inout)  :: c( * )
       type(c_ptr), intent(in), value :: interop
     end subroutine mkl_blas_cdgmm_batch_omp_offload_ilp64

  end interface
