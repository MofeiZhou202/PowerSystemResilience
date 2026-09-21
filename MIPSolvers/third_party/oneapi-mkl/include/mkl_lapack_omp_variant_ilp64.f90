!===============================================================================
! Copyright (C) 2020 Intel Corporation
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
!      Intel(R) oneAPI Math Kernel Library (oneMKL) FORTRAN interface for
!      OpenMP offload for LAPACK
!*******************************************************************************

   interface

      subroutine mkl_lapack_cgebrd_omp_offload_ilp64(m, n, a, lda, d,  &
                                                     e, tauq, taup,    &
                                                     work, lwork,      &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*8, intent(inout) :: a(lda,*)
      real, intent(out) :: d(*)
      real, intent(out) :: e(*)
      complex*8, intent(out) :: tauq(*)
      complex*8, intent(out) :: taup(*)
      complex*8, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgebrd_omp_offload_ilp64

      subroutine mkl_lapack_dgebrd_omp_offload_ilp64(m, n, a, lda, d,  &
                                                     e, tauq, taup,    &
                                                     work, lwork,      &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      double precision, intent(inout) :: a(lda,*)
      double precision, intent(out) :: d(*)
      double precision, intent(out) :: e(*)
      double precision, intent(out) :: tauq(*)
      double precision, intent(out) :: taup(*)
      double precision, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgebrd_omp_offload_ilp64

      subroutine mkl_lapack_sgebrd_omp_offload_ilp64(m, n, a, lda, d,  &
                                                     e, tauq, taup,    &
                                                     work, lwork,      &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      real, intent(inout) :: a(lda,*)
      real, intent(out) :: d(*)
      real, intent(out) :: e(*)
      real, intent(out) :: tauq(*)
      real, intent(out) :: taup(*)
      real, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgebrd_omp_offload_ilp64

      subroutine mkl_lapack_zgebrd_omp_offload_ilp64(m, n, a, lda, d,  &
                                                     e, tauq, taup,    &
                                                     work, lwork,      &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*16, intent(inout) :: a(lda,*)
      double precision, intent(out) :: d(*)
      double precision, intent(out) :: e(*)
      complex*16, intent(out) :: tauq(*)
      complex*16, intent(out) :: taup(*)
      complex*16, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgebrd_omp_offload_ilp64

      subroutine mkl_lapack_cgeqrf_omp_offload_ilp64(m, n, a, lda, tau,&
                                                     work, lwork,      &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      complex*8, intent(inout) :: a(lda,*)
      integer, intent(in) :: lda
      complex*8, intent(out) :: tau(*)
      complex*8, intent(out) :: work(*)
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgeqrf_omp_offload_ilp64

      subroutine mkl_lapack_dgeqrf_omp_offload_ilp64(m, n, a, lda, tau,&
                                                     work, lwork,      &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      double precision, intent(inout) :: a(lda,*)
      integer, intent(in) :: lda
      double precision, intent(out) :: tau(*)
      double precision, intent(out) :: work(*)
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgeqrf_omp_offload_ilp64

      subroutine mkl_lapack_sgeqrf_omp_offload_ilp64(m, n, a, lda, tau,&
                                                     work, lwork,      &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      real, intent(inout) :: a(lda,*)
      integer, intent(in) :: lda
      real, intent(out) :: tau(*)
      real, intent(out) :: work(*)
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgeqrf_omp_offload_ilp64

      subroutine mkl_lapack_zgeqrf_omp_offload_ilp64(m, n, a, lda, tau,&
                                                     work, lwork,      &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      complex*16, intent(inout) :: a(lda,*)
      integer, intent(in) :: lda
      complex*16, intent(out) :: tau(*)
      complex*16, intent(out) :: work(*)
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgeqrf_omp_offload_ilp64

      subroutine mkl_lapack_cgeqrf_batch_strided_omp_offload_ilp64(m,  &
                                                      n, a, lda,       &
                                                      stride_a, tau,   &
                                                      stride_tau, work,&
                                                      lwork,           &
                                                      batch_size, info,&
                                                      interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_tau
      integer, intent(in) :: lwork
      integer, intent(in) :: batch_size
      complex*8, intent(inout) :: a(stride_a,*)
      complex*8, intent(out) :: tau(stride_tau,*)
      complex*8, intent(out) :: work(*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgeqrf_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_dgeqrf_batch_strided_omp_offload_ilp64(m,  &
                                                      n, a, lda,       &
                                                      stride_a, tau,   &
                                                      stride_tau, work,&
                                                      lwork,           &
                                                      batch_size, info,&
                                                      interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_tau
      integer, intent(in) :: lwork
      integer, intent(in) :: batch_size
      double precision, intent(inout) :: a(stride_a,*)
      double precision, intent(out) :: tau(stride_tau,*)
      double precision, intent(out) :: work(*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgeqrf_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_sgeqrf_batch_strided_omp_offload_ilp64(m,  &
                                                      n, a, lda,       &
                                                      stride_a, tau,   &
                                                      stride_tau, work,&
                                                      lwork,           &
                                                      batch_size, info,&
                                                      interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_tau
      integer, intent(in) :: lwork
      integer, intent(in) :: batch_size
      real, intent(inout) :: a(stride_a,*)
      real, intent(out) :: tau(stride_tau,*)
      real, intent(out) :: work(*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgeqrf_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_zgeqrf_batch_strided_omp_offload_ilp64(m,  &
                                                      n, a, lda,       &
                                                      stride_a, tau,   &
                                                      stride_tau, work,&
                                                      lwork,           &
                                                      batch_size, info,&
                                                      interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_tau
      integer, intent(in) :: lwork
      integer, intent(in) :: batch_size
      complex*16, intent(inout) :: a(stride_a,*)
      complex*16, intent(out) :: tau(stride_tau,*)
      complex*16, intent(out) :: work(*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgeqrf_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_cgesvd_omp_offload_ilp64(jobu, jobvt, m, n,&
                                                     a, lda, s, u, ldu,&
                                                     vt, ldvt, work,   &
                                                     lwork, rwork,     &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobu
      character*1, intent(in) :: jobvt
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldu
      integer, intent(in) :: ldvt
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*8, intent(inout) :: a(lda,*)
      real, intent(out) :: s(*)
      complex*8, intent(out) :: u(ldu,*)
      complex*8, intent(out) :: vt(ldvt,*)
      complex*8, intent(out) :: work(*)
      real, intent(out) :: rwork(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgesvd_omp_offload_ilp64

      subroutine mkl_lapack_zgesvd_omp_offload_ilp64(jobu, jobvt, m, n,&
                                                     a, lda, s, u, ldu,&
                                                     vt, ldvt, work,   &
                                                     lwork, rwork,     &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobu
      character*1, intent(in) :: jobvt
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldu
      integer, intent(in) :: ldvt
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*16, intent(inout) :: a(lda,*)
      double precision, intent(out) :: s(*)
      complex*16, intent(out) :: u(ldu,*)
      complex*16, intent(out) :: vt(ldvt,*)
      complex*16, intent(out) :: work(*)
      double precision, intent(out) :: rwork(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgesvd_omp_offload_ilp64

      subroutine mkl_lapack_dgesvd_omp_offload_ilp64(jobu, jobvt, m, n,&
                                                     a, lda, s, u, ldu,&
                                                     vt, ldvt, work,   &
                                                     lwork,            &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobu
      character*1, intent(in) :: jobvt
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldu
      integer, intent(in) :: ldvt
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      double precision, intent(inout) :: a(lda,*)
      double precision, intent(out) :: s(*)
      double precision, intent(out) :: u(ldu,*)
      double precision, intent(out) :: vt(ldvt,*)
      double precision, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgesvd_omp_offload_ilp64

      subroutine mkl_lapack_sgesvd_omp_offload_ilp64(jobu, jobvt, m, n,&
                                                     a, lda, s, u, ldu,&
                                                     vt, ldvt, work,   &
                                                     lwork,            &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobu
      character*1, intent(in) :: jobvt
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldu
      integer, intent(in) :: ldvt
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      real, intent(inout) :: a(lda,*)
      real, intent(out) :: s(*)
      real, intent(out) :: u(ldu,*)
      real, intent(out) :: vt(ldvt,*)
      real, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgesvd_omp_offload_ilp64

      subroutine mkl_lapack_cgesvda_batch_omp_offload_ilp64(iparm,     &
                                                         irank, m, n,  &
                                                         a, lda,       &
                                                         stride_a, s,  &
                                                         stride_s, u,  &
                                                         ldu, stride_u,&
                                                         vt, ldvt,     &
                                                         stride_vt,    &
                                                         tolerance,    &
                                                         residual,     &
                                                         work, lwork,  &
                                                         batch_size,   &
                                                         info) bind(c)
      integer, intent(in) :: iparm
      integer, intent(inout) :: irank
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_s
      integer, intent(in) :: ldu
      integer, intent(in) :: stride_u
      integer, intent(in) :: ldvt
      integer, intent(in) :: stride_vt
      real, intent(in) :: tolerance
      integer, intent(in) :: lwork
      integer, intent(in) :: batch_size
      complex*8, intent(inout) :: a(stride_a,*)
      real, intent(out) :: s(stride_s,*)
      complex*8, intent(out) :: u(stride_u,*)
      complex*8, intent(out) :: vt(stride_u,*)
      real, intent(out) :: residual(*)
      complex*8, intent(out) :: work(*)
      integer, intent(out) :: info(*)
      end subroutine mkl_lapack_cgesvda_batch_omp_offload_ilp64

      subroutine mkl_lapack_cgetrf_omp_offload_ilp64(m, n, a, lda,     &
                                                     ipiv,             &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(out) :: info
      complex*8, intent(inout) :: a(lda,*)
      integer, intent(out) :: ipiv(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgetrf_omp_offload_ilp64

      subroutine mkl_lapack_dgesvda_batch_omp_offload_ilp64(iparm,     &
                                                         irank, m, n,  &
                                                         a, lda,       &
                                                         stride_a, s,  &
                                                         stride_s, u,  &
                                                         ldu, stride_u,&
                                                         vt, ldvt,     &
                                                         stride_vt,    &
                                                         tolerance,    &
                                                         residual,     &
                                                         work, lwork,  &
                                                         batch_size,   &
                                                         info) bind(c)
      integer, intent(in) :: iparm
      integer, intent(inout) :: irank
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_s
      integer, intent(in) :: ldu
      integer, intent(in) :: stride_u
      integer, intent(in) :: ldvt
      integer, intent(in) :: stride_vt
      double precision, intent(in) :: tolerance
      integer, intent(in) :: lwork
      integer, intent(in) :: batch_size
      double precision, intent(inout) :: a(stride_a,*)
      double precision, intent(out) :: s(stride_s,*)
      double precision, intent(out) :: u(stride_u,*)
      double precision, intent(out) :: vt(stride_u,*)
      double precision, intent(out) :: residual(*)
      double precision, intent(out) :: work(*)
      integer, intent(out) :: info(*)
      end subroutine mkl_lapack_dgesvda_batch_omp_offload_ilp64

      subroutine mkl_lapack_dgetrf_omp_offload_ilp64(m, n, a, lda,     &
                                                     ipiv,             &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(out) :: info
      double precision, intent(inout) :: a(lda,*)
      integer, intent(out) :: ipiv(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgetrf_omp_offload_ilp64

      subroutine mkl_lapack_sgesvda_batch_omp_offload_ilp64(iparm,     &
                                                         irank, m, n,  &
                                                         a, lda,       &
                                                         stride_a, s,  &
                                                         stride_s, u,  &
                                                         ldu, stride_u,&
                                                         vt, ldvt,     &
                                                         stride_vt,    &
                                                         tolerance,    &
                                                         residual,     &
                                                         work, lwork,  &
                                                         batch_size,   &
                                                         info) bind(c)
      integer, intent(in) :: iparm
      integer, intent(inout) :: irank
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_s
      integer, intent(in) :: ldu
      integer, intent(in) :: stride_u
      integer, intent(in) :: ldvt
      integer, intent(in) :: stride_vt
      real, intent(in) :: tolerance
      integer, intent(in) :: lwork
      integer, intent(in) :: batch_size
      real, intent(inout) :: a(stride_a,*)
      real, intent(out) :: s(stride_s,*)
      real, intent(out) :: u(stride_u,*)
      real, intent(out) :: vt(stride_u,*)
      real, intent(out) :: residual(*)
      real, intent(out) :: work(*)
      integer, intent(out) :: info(*)
      end subroutine mkl_lapack_sgesvda_batch_omp_offload_ilp64

      subroutine mkl_lapack_sgetrf_omp_offload_ilp64(m, n, a, lda,     &
                                                     ipiv,             &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(out) :: info
      real, intent(inout) :: a(lda,*)
      integer, intent(out) :: ipiv(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgetrf_omp_offload_ilp64

      subroutine mkl_lapack_zgesvda_batch_omp_offload_ilp64(iparm,     &
                                                         irank, m, n,  &
                                                         a, lda,       &
                                                         stride_a, s,  &
                                                         stride_s, u,  &
                                                         ldu, stride_u,&
                                                         vt, ldvt,     &
                                                         stride_vt,    &
                                                         tolerance,    &
                                                         residual,     &
                                                         work, lwork,  &
                                                         batch_size,   &
                                                         info) bind(c)
      integer, intent(in) :: iparm
      integer, intent(inout) :: irank
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_s
      integer, intent(in) :: ldu
      integer, intent(in) :: stride_u
      integer, intent(in) :: ldvt
      integer, intent(in) :: stride_vt
      double precision, intent(in) :: tolerance
      integer, intent(in) :: lwork
      integer, intent(in) :: batch_size
      complex*16, intent(inout) :: a(stride_a,*)
      double precision, intent(out) :: s(stride_s,*)
      complex*16, intent(out) :: u(stride_u,*)
      complex*16, intent(out) :: vt(stride_u,*)
      double precision, intent(out) :: residual(*)
      complex*16, intent(out) :: work(*)
      integer, intent(out) :: info(*)
      end subroutine mkl_lapack_zgesvda_batch_omp_offload_ilp64

      subroutine mkl_lapack_zgetrf_omp_offload_ilp64(m, n, a, lda,     &
                                                     ipiv,             &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(out) :: info
      complex*16, intent(inout) :: a(lda,*)
      integer, intent(out) :: ipiv(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgetrf_omp_offload_ilp64

      subroutine mkl_lapack_cgetrf_batch_omp_offload_ilp64(m, n, a, lda, ipiv, &
                                                           group_count, group_size, &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: group_count
      integer, intent(in) :: group_size(*)
      integer, intent(out) :: info(*)
      integer(KIND=C_INTPTR_T), intent(inout) :: a(*)
      integer(KIND=C_INTPTR_T), intent(inout) :: ipiv(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgetrf_batch_omp_offload_ilp64

      subroutine mkl_lapack_dgetrf_batch_omp_offload_ilp64(m, n, a, lda, ipiv, &
                                                           group_count, group_size, &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: group_count
      integer, intent(in) :: group_size(*)
      integer, intent(out) :: info(*)
      integer(KIND=C_INTPTR_T), intent(inout) :: a(*)
      integer(KIND=C_INTPTR_T), intent(inout) :: ipiv(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgetrf_batch_omp_offload_ilp64

      subroutine mkl_lapack_sgetrf_batch_omp_offload_ilp64(m, n, a, lda, ipiv, &
                                                           group_count, group_size, &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: group_count
      integer, intent(in) :: group_size(*)
      integer, intent(out) :: info(*)
      integer(KIND=C_INTPTR_T), intent(inout) :: a(*)
      integer(KIND=C_INTPTR_T), intent(inout) :: ipiv(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgetrf_batch_omp_offload_ilp64

      subroutine mkl_lapack_zgetrf_batch_omp_offload_ilp64(m, n, a, lda, ipiv, &
                                                           group_count, group_size, &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: group_count
      integer, intent(in) :: group_size(*)
      integer, intent(out) :: info(*)
      integer(KIND=C_INTPTR_T), intent(inout) :: a(*)
      integer(KIND=C_INTPTR_T), intent(inout) :: ipiv(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgetrf_batch_omp_offload_ilp64

      subroutine mkl_lapack_cgetrfnp_batch_omp_offload_ilp64(m, n, a, lda,          &
                                                           group_count, group_size, &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: group_count
      integer, intent(in) :: group_size(*)
      integer, intent(out) :: info(*)
      integer(KIND=C_INTPTR_T), intent(inout) :: a(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgetrfnp_batch_omp_offload_ilp64

      subroutine mkl_lapack_dgetrfnp_batch_omp_offload_ilp64(m, n, a, lda,          &
                                                           group_count, group_size, &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: group_count
      integer, intent(in) :: group_size(*)
      integer, intent(out) :: info(*)
      integer(KIND=C_INTPTR_T), intent(inout) :: a(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgetrfnp_batch_omp_offload_ilp64

      subroutine mkl_lapack_sgetrfnp_batch_omp_offload_ilp64(m, n, a, lda,          &
                                                           group_count, group_size, &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: group_count
      integer, intent(in) :: group_size(*)
      integer, intent(out) :: info(*)
      integer(KIND=C_INTPTR_T), intent(inout) :: a(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgetrfnp_batch_omp_offload_ilp64

      subroutine mkl_lapack_zgetrfnp_batch_omp_offload_ilp64(m, n, a, lda,          &
                                                           group_count, group_size, &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: group_count
      integer, intent(in) :: group_size(*)
      integer, intent(out) :: info(*)
      integer(KIND=C_INTPTR_T), intent(inout) :: a(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgetrfnp_batch_omp_offload_ilp64

      subroutine mkl_lapack_cgetri_oop_batch_omp_offload_ilp64(n, a, lda, ipiv, ainv, ldainv, &
                                                           group_count, group_size, &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldainv
      integer, intent(in) :: group_count
      integer, intent(in) :: group_size(*)
      integer, intent(out) :: info(*)
      integer(KIND=C_INTPTR_T), intent(in) :: a(*), ipiv(*)
      integer(KIND=C_INTPTR_T), intent(out) :: ainv(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgetri_oop_batch_omp_offload_ilp64

      subroutine mkl_lapack_dgetri_oop_batch_omp_offload_ilp64(n, a, lda, ipiv, ainv, ldainv, &
                                                           group_count, group_size, &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldainv
      integer, intent(in) :: group_count
      integer, intent(in) :: group_size(*)
      integer, intent(out) :: info(*)
      integer(KIND=C_INTPTR_T), intent(in) :: a(*), ipiv(*)
      integer(KIND=C_INTPTR_T), intent(out) :: ainv(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgetri_oop_batch_omp_offload_ilp64

      subroutine mkl_lapack_sgetri_oop_batch_omp_offload_ilp64(n, a, lda, ipiv, ainv, ldainv, &
                                                           group_count, group_size, &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldainv
      integer, intent(in) :: group_count
      integer, intent(in) :: group_size(*)
      integer, intent(out) :: info(*)
      integer(KIND=C_INTPTR_T), intent(in) :: a(*), ipiv(*)
      integer(KIND=C_INTPTR_T), intent(out) :: ainv(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgetri_oop_batch_omp_offload_ilp64

      subroutine mkl_lapack_zgetri_oop_batch_omp_offload_ilp64(n, a, lda, ipiv, ainv, ldainv, &
                                                           group_count, group_size, &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldainv
      integer, intent(in) :: group_count
      integer, intent(in) :: group_size(*)
      integer, intent(out) :: info(*)
      integer(KIND=C_INTPTR_T), intent(in) :: a(*), ipiv(*)
      integer(KIND=C_INTPTR_T), intent(out) :: ainv(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgetri_oop_batch_omp_offload_ilp64

      subroutine mkl_lapack_cgetrf_batch_strided_omp_offload_ilp64(m,  &
                                                      n, a, lda,       &
                                                      stride_a, ipiv,  &
                                                      stride_ipiv,     &
                                                      batch_size, info,&
                                                      interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_ipiv
      integer, intent(in) :: batch_size
      complex*8, intent(inout) :: a(stride_a,*)
      integer, intent(out) :: ipiv(stride_ipiv,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgetrf_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_dgetrf_batch_strided_omp_offload_ilp64(m,  &
                                                      n, a, lda,       &
                                                      stride_a, ipiv,  &
                                                      stride_ipiv,     &
                                                      batch_size, info,&
                                                      interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_ipiv
      integer, intent(in) :: batch_size
      double precision, intent(inout) :: a(stride_a,*)
      integer, intent(out) :: ipiv(stride_ipiv,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgetrf_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_sgetrf_batch_strided_omp_offload_ilp64(m,  &
                                                      n, a, lda,       &
                                                      stride_a, ipiv,  &
                                                      stride_ipiv,     &
                                                      batch_size, info,&
                                                      interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_ipiv
      integer, intent(in) :: batch_size
      real, intent(inout) :: a(stride_a,*)
      integer, intent(out) :: ipiv(stride_ipiv,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgetrf_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_zgetrf_batch_strided_omp_offload_ilp64(m,  &
                                                      n, a, lda,       &
                                                      stride_a, ipiv,  &
                                                      stride_ipiv,     &
                                                      batch_size, info,&
                                                      interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_ipiv
      integer, intent(in) :: batch_size
      complex*16, intent(inout) :: a(stride_a,*)
      integer, intent(out) :: ipiv(stride_ipiv,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgetrf_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_cgetri_omp_offload_ilp64(n, a, lda, ipiv,  &
                                                     work, lwork, info,&
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*8, intent(inout) :: a(lda,*)
      integer, intent(in) :: ipiv(*)
      complex*8, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgetri_omp_offload_ilp64

      subroutine mkl_lapack_dgetri_omp_offload_ilp64(n, a, lda, ipiv,  &
                                                     work, lwork, info,&
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      double precision, intent(inout) :: a(lda,*)
      integer, intent(in) :: ipiv(*)
      double precision, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgetri_omp_offload_ilp64

      subroutine mkl_lapack_sgetri_omp_offload_ilp64(n, a, lda, ipiv,  &
                                                     work, lwork, info,&
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      real, intent(inout) :: a(lda,*)
      integer, intent(in) :: ipiv(*)
      real, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgetri_omp_offload_ilp64

      subroutine mkl_lapack_zgetri_omp_offload_ilp64(n, a, lda, ipiv,  &
                                                     work, lwork, info,&
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*16, intent(inout) :: a(lda,*)
      integer, intent(in) :: ipiv(*)
      complex*16, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgetri_omp_offload_ilp64

      subroutine mkl_lapack_cgetri_oop_batch_strided_omp_offload_ilp64(n,&
                                                           a, lda,       &
                                                           stride_a,     &
                                                           ipiv,         &
                                                           stride_ipiv,  &
                                                           ainv, ldainv, &
                                                           stride_ainv,  &
                                                           batch_size,   &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_ipiv
      integer, intent(in) :: ldainv
      integer, intent(in) :: stride_ainv
      integer, intent(in) :: batch_size
      complex*8, intent(in) :: a(stride_a,*)
      integer, intent(in) :: ipiv(stride_ipiv,*)
      complex*8, intent(inout) :: ainv(stride_ainv,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgetri_oop_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_dgetri_oop_batch_strided_omp_offload_ilp64(n,&
                                                           a, lda,       &
                                                           stride_a,     &
                                                           ipiv,         &
                                                           stride_ipiv,  &
                                                           ainv, ldainv, &
                                                           stride_ainv,  &
                                                           batch_size,   &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_ipiv
      integer, intent(in) :: ldainv
      integer, intent(in) :: stride_ainv
      integer, intent(in) :: batch_size
      double precision, intent(in) :: a(stride_a,*)
      integer, intent(in) :: ipiv(stride_ipiv,*)
      double precision, intent(inout) :: ainv(stride_ainv,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgetri_oop_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_sgetri_oop_batch_strided_omp_offload_ilp64(n,&
                                                           a, lda,       &
                                                           stride_a,     &
                                                           ipiv,         &
                                                           stride_ipiv,  &
                                                           ainv, ldainv, &
                                                           stride_ainv,  &
                                                           batch_size,   &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_ipiv
      integer, intent(in) :: ldainv
      integer, intent(in) :: stride_ainv
      integer, intent(in) :: batch_size
      real, intent(in) :: a(stride_a,*)
      integer, intent(in) :: ipiv(stride_ipiv,*)
      real, intent(inout) :: ainv(stride_ainv,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgetri_oop_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_zgetri_oop_batch_strided_omp_offload_ilp64(n,&
                                                           a, lda,       &
                                                           stride_a,     &
                                                           ipiv,         &
                                                           stride_ipiv,  &
                                                           ainv, ldainv, &
                                                           stride_ainv,  &
                                                           batch_size,   &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_ipiv
      integer, intent(in) :: ldainv
      integer, intent(in) :: stride_ainv
      integer, intent(in) :: batch_size
      complex*16, intent(in) :: a(stride_a,*)
      integer, intent(in) :: ipiv(stride_ipiv,*)
      complex*16, intent(inout) :: ainv(stride_ainv,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgetri_oop_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_cgetrs_omp_offload_ilp64(trans, n, nrhs, a,&
                                                     lda, ipiv, b, ldb,&
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: trans
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      integer, intent(out) :: info
      complex*8, intent(in) :: a(lda,*)
      integer, intent(in) :: ipiv(*)
      complex*8, intent(inout) :: b(ldb,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgetrs_omp_offload_ilp64

      subroutine mkl_lapack_dgetrs_omp_offload_ilp64(trans, n, nrhs, a,&
                                                     lda, ipiv, b, ldb,&
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: trans
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      integer, intent(out) :: info
      double precision, intent(in) :: a(lda,*)
      integer, intent(in) :: ipiv(*)
      double precision, intent(inout) :: b(ldb,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgetrs_omp_offload_ilp64

      subroutine mkl_lapack_sgetrs_omp_offload_ilp64(trans, n, nrhs, a,&
                                                     lda, ipiv, b, ldb,&
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: trans
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      integer, intent(out) :: info
      real, intent(in) :: a(lda,*)
      integer, intent(in) :: ipiv(*)
      real, intent(inout) :: b(ldb,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgetrs_omp_offload_ilp64

      subroutine mkl_lapack_zgetrs_omp_offload_ilp64(trans, n, nrhs, a,&
                                                     lda, ipiv, b, ldb,&
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: trans
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      integer, intent(out) :: info
      complex*16, intent(in) :: a(lda,*)
      integer, intent(in) :: ipiv(*)
      complex*16, intent(inout) :: b(ldb,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgetrs_omp_offload_ilp64

      subroutine mkl_lapack_cheev_omp_offload_ilp64(jobz, uplo, n, a,  &
                                                    lda, w, work,      &
                                                    lwork, rwork,      &
                                                    info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobz
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*8, intent(inout) :: a(lda,*)
      real, intent(out) :: w(*)
      complex*8, intent(out) :: work(*)
      real, intent(out) :: rwork(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cheev_omp_offload_ilp64

      subroutine mkl_lapack_zheev_omp_offload_ilp64(jobz, uplo, n, a,  &
                                                    lda, w, work,      &
                                                    lwork, rwork,      &
                                                    info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobz
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*16, intent(inout) :: a(lda,*)
      double precision, intent(out) :: w(*)
      complex*16, intent(out) :: work(*)
      double precision, intent(out) :: rwork(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zheev_omp_offload_ilp64

      subroutine mkl_lapack_cheevd_omp_offload_ilp64(jobz, uplo, n, a, &
                                                     lda, w, work,     &
                                                     lwork, rwork,     &
                                                     lrwork, iwork,    &
                                                     liwork, info,     &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobz
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(in) :: lrwork
      integer, intent(in) :: liwork
      integer, intent(out) :: info
      complex*8, intent(inout) :: a(lda,*)
      real, intent(out) :: w(*)
      complex*8, intent(out) :: work(*)
      real, intent(out) :: rwork(*)
      integer, intent(out) :: iwork(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cheevd_omp_offload_ilp64

      subroutine mkl_lapack_zheevd_omp_offload_ilp64(jobz, uplo, n, a, &
                                                     lda, w, work,     &
                                                     lwork, rwork,     &
                                                     lrwork, iwork,    &
                                                     liwork, info,     &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobz
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(in) :: lrwork
      integer, intent(in) :: liwork
      integer, intent(out) :: info
      complex*16, intent(inout) :: a(lda,*)
      double precision, intent(out) :: w(*)
      complex*16, intent(out) :: work(*)
      double precision, intent(out) :: rwork(*)
      integer, intent(out) :: iwork(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zheevd_omp_offload_ilp64

      subroutine mkl_lapack_cheevx_omp_offload_ilp64(jobz, range, uplo,&
                                                     n, a, lda, vl, vu,&
                                                     il, iu, abstol, m,&
                                                     w, z, ldz, work,  &
                                                     lwork, rwork,     &
                                                     iwork, ifail,     &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobz
      character*1, intent(in) :: range
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      real, intent(in) :: vl
      real, intent(in) :: vu
      integer, intent(in) :: il
      integer, intent(in) :: iu
      real, intent(in) :: abstol
      integer, intent(out) :: m
      integer, intent(in) :: ldz
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*8, intent(inout) :: a(lda,*)
      real, intent(out) :: w(*)
      complex*8, intent(out) :: z(ldz,*)
      complex*8, intent(out) :: work(*)
      real, intent(out) :: rwork(*)
      integer, intent(out) :: iwork(*)
      integer, intent(out) :: ifail(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cheevx_omp_offload_ilp64

      subroutine mkl_lapack_zheevx_omp_offload_ilp64(jobz, range, uplo,&
                                                     n, a, lda, vl, vu,&
                                                     il, iu, abstol, m,&
                                                     w, z, ldz, work,  &
                                                     lwork, rwork,     &
                                                     iwork, ifail,     &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobz
      character*1, intent(in) :: range
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      double precision, intent(in) :: vl
      double precision, intent(in) :: vu
      integer, intent(in) :: il
      integer, intent(in) :: iu
      double precision, intent(in) :: abstol
      integer, intent(out) :: m
      integer, intent(in) :: ldz
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*16, intent(inout) :: a(lda,*)
      double precision, intent(out) :: w(*)
      complex*16, intent(out) :: z(ldz,*)
      complex*16, intent(out) :: work(*)
      double precision, intent(out) :: rwork(*)
      integer, intent(out) :: iwork(*)
      integer, intent(out) :: ifail(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zheevx_omp_offload_ilp64

      subroutine mkl_lapack_chegvd_omp_offload_ilp64(itype, jobz, uplo,&
                                                     n, a, lda, b, ldb,&
                                                     w, work, lwork,   &
                                                     rwork, lrwork,    &
                                                     iwork, liwork,    &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: itype
      character*1, intent(in) :: jobz
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      integer, intent(in) :: lwork
      integer, intent(in) :: lrwork
      integer, intent(in) :: liwork
      integer, intent(out) :: info
      complex*8, intent(inout) :: a(lda,*)
      complex*8, intent(inout) :: b(ldb,*)
      real, intent(out) :: w(*)
      complex*8, intent(out) :: work(*)
      real, intent(out) :: rwork(*)
      integer, intent(out) :: iwork(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_chegvd_omp_offload_ilp64

      subroutine mkl_lapack_zhegvd_omp_offload_ilp64(itype, jobz, uplo,&
                                                     n, a, lda, b, ldb,&
                                                     w, work, lwork,   &
                                                     rwork, lrwork,    &
                                                     iwork, liwork,    &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: itype
      character*1, intent(in) :: jobz
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      integer, intent(in) :: lwork
      integer, intent(in) :: lrwork
      integer, intent(in) :: liwork
      integer, intent(out) :: info
      complex*16, intent(inout) :: a(lda,*)
      complex*16, intent(inout) :: b(ldb,*)
      double precision, intent(out) :: w(*)
      complex*16, intent(out) :: work(*)
      double precision, intent(out) :: rwork(*)
      integer, intent(out) :: iwork(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zhegvd_omp_offload_ilp64

      subroutine mkl_lapack_chegvx_omp_offload_ilp64(itype, jobz,      &
                                                     range, uplo, n, a,&
                                                     lda, b, ldb, vl,  &
                                                     vu, il, iu,       &
                                                     abstol, m, w, z,  &
                                                     ldz, work, lwork, &
                                                     rwork, iwork,     &
                                                     ifail, info,      &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: itype
      character*1, intent(in) :: jobz
      character*1, intent(in) :: range
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      real, intent(in) :: vl
      real, intent(in) :: vu
      integer, intent(in) :: il
      integer, intent(in) :: iu
      real, intent(in) :: abstol
      integer, intent(out) :: m
      integer, intent(in) :: ldz
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*8, intent(inout) :: a(lda,*)
      complex*8, intent(inout) :: b(ldb,*)
      real, intent(out) :: w(*)
      complex*8, intent(out) :: z(ldz,*)
      complex*8, intent(out) :: work(*)
      real, intent(out) :: rwork(*)
      integer, intent(out) :: iwork(*)
      integer, intent(out) :: ifail(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_chegvx_omp_offload_ilp64

      subroutine mkl_lapack_zhegvx_omp_offload_ilp64(itype, jobz,      &
                                                     range, uplo, n, a,&
                                                     lda, b, ldb, vl,  &
                                                     vu, il, iu,       &
                                                     abstol, m, w, z,  &
                                                     ldz, work, lwork, &
                                                     rwork, iwork,     &
                                                     ifail, info,      &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: itype
      character*1, intent(in) :: jobz
      character*1, intent(in) :: range
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      double precision, intent(in) :: vl
      double precision, intent(in) :: vu
      integer, intent(in) :: il
      integer, intent(in) :: iu
      double precision, intent(in) :: abstol
      integer, intent(out) :: m
      integer, intent(in) :: ldz
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*16, intent(inout) :: a(lda,*)
      complex*16, intent(inout) :: b(ldb,*)
      double precision, intent(out) :: w(*)
      complex*16, intent(out) :: z(ldz,*)
      complex*16, intent(out) :: work(*)
      double precision, intent(out) :: rwork(*)
      integer, intent(out) :: iwork(*)
      integer, intent(out) :: ifail(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zhegvx_omp_offload_ilp64

      subroutine mkl_lapack_chetrd_omp_offload_ilp64(uplo, n, a, lda,  &
                                                     d, e, tau, work,  &
                                                     lwork,            &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*8, intent(inout) :: a(lda,*)
      real, intent(out) :: d(*)
      real, intent(out) :: e(*)
      complex*8, intent(out) :: tau(*)
      complex*8, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_chetrd_omp_offload_ilp64

      subroutine mkl_lapack_zhetrd_omp_offload_ilp64(uplo, n, a, lda,  &
                                                     d, e, tau, work,  &
                                                     lwork,            &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*16, intent(inout) :: a(lda,*)
      double precision, intent(out) :: d(*)
      double precision, intent(out) :: e(*)
      complex*16, intent(out) :: tau(*)
      complex*16, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zhetrd_omp_offload_ilp64

      subroutine mkl_lapack_dorgqr_omp_offload_ilp64(m, n, k, a, lda,  &
                                                     tau, work, lwork, &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: k
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      double precision, intent(inout) :: a(lda,*)
      double precision, intent(in) :: tau(*)
      double precision, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dorgqr_omp_offload_ilp64

      subroutine mkl_lapack_sorgqr_omp_offload_ilp64(m, n, k, a, lda,  &
                                                     tau, work, lwork, &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: k
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      real, intent(inout) :: a(lda,*)
      real, intent(in) :: tau(*)
      real, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sorgqr_omp_offload_ilp64

      subroutine mkl_lapack_dormqr_omp_offload_ilp64(side, trans, m, n,&
                                                     k, a, lda, tau, c,&
                                                     ldc, work, lwork, &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: side
      character*1, intent(in) :: trans
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: k
      integer, intent(in) :: lda
      integer, intent(in) :: ldc
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      double precision, intent(in) :: a(lda,*)
      double precision, intent(in) :: tau(*)
      double precision, intent(inout) :: c(ldc,*)
      double precision, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dormqr_omp_offload_ilp64

      subroutine mkl_lapack_sormqr_omp_offload_ilp64(side, trans, m, n,&
                                                     k, a, lda, tau, c,&
                                                     ldc, work, lwork, &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: side
      character*1, intent(in) :: trans
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: k
      integer, intent(in) :: lda
      integer, intent(in) :: ldc
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      real, intent(in) :: a(lda,*)
      real, intent(in) :: tau(*)
      real, intent(inout) :: c(ldc,*)
      real, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sormqr_omp_offload_ilp64

      subroutine mkl_lapack_csteqr_omp_offload_ilp64(compz, n, d, e, z,&
                                                     ldz, work,        &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: compz
      integer, intent(in) :: n
      integer, intent(in) :: ldz
      integer, intent(out) :: info
      real, intent(inout) :: d(*)
      real, intent(inout) :: e(*)
      complex*8, intent(inout) :: z(ldz,*)
      real, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_csteqr_omp_offload_ilp64

      subroutine mkl_lapack_dsteqr_omp_offload_ilp64(compz, n, d, e, z,&
                                                     ldz, work,        &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: compz
      integer, intent(in) :: n
      integer, intent(in) :: ldz
      integer, intent(out) :: info
      double precision, intent(inout) :: d(*)
      double precision, intent(inout) :: e(*)
      double precision, intent(inout) :: z(ldz,*)
      double precision, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dsteqr_omp_offload_ilp64

      subroutine mkl_lapack_ssteqr_omp_offload_ilp64(compz, n, d, e, z,&
                                                     ldz, work,        &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: compz
      integer, intent(in) :: n
      integer, intent(in) :: ldz
      integer, intent(out) :: info
      real, intent(inout) :: d(*)
      real, intent(inout) :: e(*)
      real, intent(inout) :: z(ldz,*)
      real, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_ssteqr_omp_offload_ilp64

      subroutine mkl_lapack_zsteqr_omp_offload_ilp64(compz, n, d, e, z,&
                                                     ldz, work,        &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: compz
      integer, intent(in) :: n
      integer, intent(in) :: ldz
      integer, intent(out) :: info
      double precision, intent(inout) :: d(*)
      double precision, intent(inout) :: e(*)
      complex*16, intent(inout) :: z(ldz,*)
      double precision, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zsteqr_omp_offload_ilp64

      subroutine mkl_lapack_dsyev_omp_offload_ilp64(jobz, uplo, n, a,  &
                                                    lda, w, work,      &
                                                    lwork,             &
                                                    info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobz
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      double precision, intent(inout) :: a(lda,*)
      double precision, intent(out) :: w(*)
      double precision, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dsyev_omp_offload_ilp64

      subroutine mkl_lapack_ssyev_omp_offload_ilp64(jobz, uplo, n, a,  &
                                                    lda, w, work,      &
                                                    lwork,             &
                                                    info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobz
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      real, intent(inout) :: a(lda,*)
      real, intent(out) :: w(*)
      real, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_ssyev_omp_offload_ilp64

      subroutine mkl_lapack_dsyevd_omp_offload_ilp64(jobz, uplo, n, a, &
                                                     lda, w, work,     &
                                                     lwork, iwork,     &
                                                     liwork, info,     &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobz
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(in) :: liwork
      integer, intent(out) :: info
      double precision, intent(inout) :: a(lda,*)
      double precision, intent(out) :: w(*)
      double precision, intent(out) :: work(*)
      integer, intent(out) :: iwork(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dsyevd_omp_offload_ilp64

      subroutine mkl_lapack_ssyevd_omp_offload_ilp64(jobz, uplo, n, a, &
                                                     lda, w, work,     &
                                                     lwork, iwork,     &
                                                     liwork, info,     &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobz
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(in) :: liwork
      integer, intent(out) :: info
      real, intent(inout) :: a(lda,*)
      real, intent(out) :: w(*)
      real, intent(out) :: work(*)
      integer, intent(out) :: iwork(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_ssyevd_omp_offload_ilp64

      subroutine mkl_lapack_dsyevx_omp_offload_ilp64(jobz, range, uplo,&
                                                     n, a, lda, vl, vu,&
                                                     il, iu, abstol, m,&
                                                     w, z, ldz, work,  &
                                                     lwork, iwork,     &
                                                     ifail, info,      &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobz
      character*1, intent(in) :: range
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      double precision, intent(in) :: vl
      double precision, intent(in) :: vu
      integer, intent(in) :: il
      integer, intent(in) :: iu
      double precision, intent(in) :: abstol
      integer, intent(out) :: m
      integer, intent(in) :: ldz
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      double precision, intent(inout) :: a(lda,*)
      double precision, intent(out) :: w(*)
      double precision, intent(out) :: z(ldz,*)
      double precision, intent(out) :: work(*)
      integer, intent(out) :: iwork(*)
      integer, intent(out) :: ifail(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dsyevx_omp_offload_ilp64

      subroutine mkl_lapack_ssyevx_omp_offload_ilp64(jobz, range, uplo,&
                                                     n, a, lda, vl, vu,&
                                                     il, iu, abstol, m,&
                                                     w, z, ldz, work,  &
                                                     lwork, iwork,     &
                                                     ifail, info,      &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: jobz
      character*1, intent(in) :: range
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      real, intent(in) :: vl
      real, intent(in) :: vu
      integer, intent(in) :: il
      integer, intent(in) :: iu
      real, intent(in) :: abstol
      integer, intent(out) :: m
      integer, intent(in) :: ldz
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      real, intent(inout) :: a(lda,*)
      real, intent(out) :: w(*)
      real, intent(out) :: z(ldz,*)
      real, intent(out) :: work(*)
      integer, intent(out) :: iwork(*)
      integer, intent(out) :: ifail(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_ssyevx_omp_offload_ilp64

      subroutine mkl_lapack_dsygvd_omp_offload_ilp64(itype, jobz, uplo,&
                                                     n, a, lda, b, ldb,&
                                                     w, work, lwork,   &
                                                     iwork, liwork,    &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: itype
      character*1, intent(in) :: jobz
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      integer, intent(in) :: lwork
      integer, intent(in) :: liwork
      integer, intent(out) :: info
      double precision, intent(inout) :: a(lda,*)
      double precision, intent(inout) :: b(ldb,*)
      double precision, intent(out) :: w(*)
      double precision, intent(out) :: work(*)
      integer, intent(out) :: iwork(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dsygvd_omp_offload_ilp64

      subroutine mkl_lapack_ssygvd_omp_offload_ilp64(itype, jobz, uplo,&
                                                     n, a, lda, b, ldb,&
                                                     w, work, lwork,   &
                                                     iwork, liwork,    &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: itype
      character*1, intent(in) :: jobz
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      integer, intent(in) :: lwork
      integer, intent(in) :: liwork
      integer, intent(out) :: info
      real, intent(inout) :: a(lda,*)
      real, intent(inout) :: b(ldb,*)
      real, intent(out) :: w(*)
      real, intent(out) :: work(*)
      integer, intent(out) :: iwork(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_ssygvd_omp_offload_ilp64

      subroutine mkl_lapack_dsygvx_omp_offload_ilp64(itype, jobz,      &
                                                     range, uplo, n, a,&
                                                     lda, b, ldb, vl,  &
                                                     vu, il, iu,       &
                                                     abstol, m, w, z,  &
                                                     ldz, work, lwork, &
                                                     iwork, ifail,     &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: itype
      character*1, intent(in) :: jobz
      character*1, intent(in) :: range
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      double precision, intent(in) :: vl
      double precision, intent(in) :: vu
      integer, intent(in) :: il
      integer, intent(in) :: iu
      double precision, intent(in) :: abstol
      integer, intent(out) :: m
      integer, intent(in) :: ldz
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      double precision, intent(inout) :: a(lda,*)
      double precision, intent(inout) :: b(ldb,*)
      double precision, intent(out) :: w(*)
      double precision, intent(out) :: z(ldz,*)
      double precision, intent(out) :: work(*)
      integer, intent(out) :: iwork(*)
      integer, intent(out) :: ifail(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dsygvx_omp_offload_ilp64

      subroutine mkl_lapack_ssygvx_omp_offload_ilp64(itype, jobz,      &
                                                     range, uplo, n, a,&
                                                     lda, b, ldb, vl,  &
                                                     vu, il, iu,       &
                                                     abstol, m, w, z,  &
                                                     ldz, work, lwork, &
                                                     iwork, ifail,     &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: itype
      character*1, intent(in) :: jobz
      character*1, intent(in) :: range
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      real, intent(in) :: vl
      real, intent(in) :: vu
      integer, intent(in) :: il
      integer, intent(in) :: iu
      real, intent(in) :: abstol
      integer, intent(out) :: m
      integer, intent(in) :: ldz
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      real, intent(inout) :: a(lda,*)
      real, intent(inout) :: b(ldb,*)
      real, intent(out) :: w(*)
      real, intent(out) :: z(ldz,*)
      real, intent(out) :: work(*)
      integer, intent(out) :: iwork(*)
      integer, intent(out) :: ifail(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_ssygvx_omp_offload_ilp64

      subroutine mkl_lapack_dsytrd_omp_offload_ilp64(uplo, n, a, lda,  &
                                                     d, e, tau, work,  &
                                                     lwork,            &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      double precision, intent(inout) :: a(lda,*)
      double precision, intent(out) :: d(*)
      double precision, intent(out) :: e(*)
      double precision, intent(out) :: tau(*)
      double precision, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dsytrd_omp_offload_ilp64

      subroutine mkl_lapack_ssytrd_omp_offload_ilp64(uplo, n, a, lda,  &
                                                     d, e, tau, work,  &
                                                     lwork,            &
                                                     info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      real, intent(inout) :: a(lda,*)
      real, intent(out) :: d(*)
      real, intent(out) :: e(*)
      real, intent(out) :: tau(*)
      real, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_ssytrd_omp_offload_ilp64

      subroutine mkl_lapack_ctrtri_omp_offload_ilp64(uplo, diag, n, a, &
                                                     lda, info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      character*1, intent(in) :: diag
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(out) :: info
      complex*8, intent(inout) :: a(lda,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_ctrtri_omp_offload_ilp64

      subroutine mkl_lapack_dtrtri_omp_offload_ilp64(uplo, diag, n, a, &
                                                     lda, info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      character*1, intent(in) :: diag
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(out) :: info
      double precision, intent(inout) :: a(lda,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dtrtri_omp_offload_ilp64

      subroutine mkl_lapack_strtri_omp_offload_ilp64(uplo, diag, n, a, &
                                                     lda, info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      character*1, intent(in) :: diag
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(out) :: info
      real, intent(inout) :: a(lda,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_strtri_omp_offload_ilp64

      subroutine mkl_lapack_ztrtri_omp_offload_ilp64(uplo, diag, n, a, &
                                                     lda, info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      character*1, intent(in) :: diag
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(out) :: info
      complex*16, intent(inout) :: a(lda,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_ztrtri_omp_offload_ilp64

      subroutine mkl_lapack_ctrtrs_omp_offload_ilp64(uplo, trans, diag,&
                                                     n, nrhs, a, lda,  &
                                                     b, ldb, info,     &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      character*1, intent(in) :: trans
      character*1, intent(in) :: diag
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      integer, intent(out) :: info
      complex*8, intent(in) :: a(lda,*)
      complex*8, intent(inout) :: b(ldb,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_ctrtrs_omp_offload_ilp64

      subroutine mkl_lapack_dtrtrs_omp_offload_ilp64(uplo, trans, diag,&
                                                     n, nrhs, a, lda,  &
                                                     b, ldb, info,     &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      character*1, intent(in) :: trans
      character*1, intent(in) :: diag
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      integer, intent(out) :: info
      double precision, intent(in) :: a(lda,*)
      double precision, intent(inout) :: b(ldb,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dtrtrs_omp_offload_ilp64

      subroutine mkl_lapack_strtrs_omp_offload_ilp64(uplo, trans, diag,&
                                                     n, nrhs, a, lda,  &
                                                     b, ldb, info,     &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      character*1, intent(in) :: trans
      character*1, intent(in) :: diag
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      integer, intent(out) :: info
      real, intent(in) :: a(lda,*)
      real, intent(inout) :: b(ldb,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_strtrs_omp_offload_ilp64

      subroutine mkl_lapack_ztrtrs_omp_offload_ilp64(uplo, trans, diag,&
                                                     n, nrhs, a, lda,  &
                                                     b, ldb, info,     &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      character*1, intent(in) :: trans
      character*1, intent(in) :: diag
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: ldb
      integer, intent(out) :: info
      complex*16, intent(in) :: a(lda,*)
      complex*16, intent(inout) :: b(ldb,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_ztrtrs_omp_offload_ilp64

      subroutine mkl_lapack_cungqr_omp_offload_ilp64(m, n, k, a, lda,  &
                                                     tau, work, lwork, &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: k
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*8, intent(inout) :: a(lda,*)
      complex*8, intent(in) :: tau(*)
      complex*8, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cungqr_omp_offload_ilp64

      subroutine mkl_lapack_zungqr_omp_offload_ilp64(m, n, k, a, lda,  &
                                                     tau, work, lwork, &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: k
      integer, intent(in) :: lda
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*16, intent(inout) :: a(lda,*)
      complex*16, intent(in) :: tau(*)
      complex*16, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zungqr_omp_offload_ilp64

      subroutine mkl_lapack_cunmqr_omp_offload_ilp64(side, trans, m, n,&
                                                     k, a, lda, tau, c,&
                                                     ldc, work, lwork, &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: side
      character*1, intent(in) :: trans
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: k
      integer, intent(in) :: lda
      integer, intent(in) :: ldc
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*8, intent(in) :: a(lda,*)
      complex*8, intent(in) :: tau(*)
      complex*8, intent(inout) :: c(ldc,*)
      complex*8, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cunmqr_omp_offload_ilp64

      subroutine mkl_lapack_zunmqr_omp_offload_ilp64(side, trans, m, n,&
                                                     k, a, lda, tau, c,&
                                                     ldc, work, lwork, &
                                                     info,             &
                                                     interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: side
      character*1, intent(in) :: trans
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: k
      integer, intent(in) :: lda
      integer, intent(in) :: ldc
      integer, intent(in) :: lwork
      integer, intent(out) :: info
      complex*16, intent(in) :: a(lda,*)
      complex*16, intent(in) :: tau(*)
      complex*16, intent(inout) :: c(ldc,*)
      complex*16, intent(out) :: work(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zunmqr_omp_offload_ilp64

      subroutine mkl_lapack_cgetrs_batch_strided_omp_offload_ilp64(trans,&
                                                           n, nrhs, a,   &
                                                           lda, stride_a,&
                                                           ipiv,         &
                                                           stride_ipiv,  &
                                                           b, ldb,       &
                                                           stride_b,     &
                                                           batch_size,   &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: trans
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_ipiv
      integer, intent(in) :: ldb
      integer, intent(in) :: stride_b
      integer, intent(in) :: batch_size
      complex*8, intent(in) :: a(stride_a,*)
      integer, intent(in) :: ipiv(stride_ipiv,*)
      complex*8, intent(inout) :: b(stride_b,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgetrs_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_dgetrs_batch_strided_omp_offload_ilp64(trans,&
                                                           n, nrhs, a,   &
                                                           lda, stride_a,&
                                                           ipiv,         &
                                                           stride_ipiv,  &
                                                           b, ldb,       &
                                                           stride_b,     &
                                                           batch_size,   &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: trans
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_ipiv
      integer, intent(in) :: ldb
      integer, intent(in) :: stride_b
      integer, intent(in) :: batch_size
      double precision, intent(in) :: a(stride_a,*)
      integer, intent(in) :: ipiv(stride_ipiv,*)
      double precision, intent(inout) :: b(stride_b,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgetrs_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_sgetrs_batch_strided_omp_offload_ilp64(trans,&
                                                           n, nrhs, a,   &
                                                           lda, stride_a,&
                                                           ipiv,         &
                                                           stride_ipiv,  &
                                                           b, ldb,       &
                                                           stride_b,     &
                                                           batch_size,   &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: trans
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_ipiv
      integer, intent(in) :: ldb
      integer, intent(in) :: stride_b
      integer, intent(in) :: batch_size
      real, intent(in) :: a(stride_a,*)
      integer, intent(in) :: ipiv(stride_ipiv,*)
      real, intent(inout) :: b(stride_b,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgetrs_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_zgetrs_batch_strided_omp_offload_ilp64(trans,&
                                                           n, nrhs, a,   &
                                                           lda, stride_a,&
                                                           ipiv,         &
                                                           stride_ipiv,  &
                                                           b, ldb,       &
                                                           stride_b,     &
                                                           batch_size,   &
                                                           info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: trans
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: stride_ipiv
      integer, intent(in) :: ldb
      integer, intent(in) :: stride_b
      integer, intent(in) :: batch_size
      complex*16, intent(in) :: a(stride_a,*)
      integer, intent(in) :: ipiv(stride_ipiv,*)
      complex*16, intent(inout) :: b(stride_b,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgetrs_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_cgetrfnp_omp_offload_ilp64(m, n, a, lda,   &
                                                       info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(out) :: info
      complex*8, intent(inout) :: a(lda,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgetrfnp_omp_offload_ilp64

      subroutine mkl_lapack_cgetrfnp_batch_strided_omp_offload_ilp64(m,&
                                                      n, a, lda,       &
                                                      stride_a,        &
                                                      batch_size, info,&
                                                      interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: batch_size
      complex*8, intent(inout) :: a(stride_a,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgetrfnp_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_dgetrfnp_omp_offload_ilp64(m, n, a, lda,   &
                                                       info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(out) :: info
      double precision, intent(inout) :: a(lda,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgetrfnp_omp_offload_ilp64

      subroutine mkl_lapack_dgetrfnp_batch_strided_omp_offload_ilp64(m,&
                                                      n, a, lda,       &
                                                      stride_a,        &
                                                      batch_size, info,&
                                                      interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: batch_size
      double precision, intent(inout) :: a(stride_a,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgetrfnp_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_sgetrfnp_omp_offload_ilp64(m, n, a, lda,   &
                                                       info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(out) :: info
      real, intent(inout) :: a(lda,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgetrfnp_omp_offload_ilp64

      subroutine mkl_lapack_sgetrfnp_batch_strided_omp_offload_ilp64(m,&
                                                      n, a, lda,       &
                                                      stride_a,        &
                                                      batch_size, info,&
                                                      interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: batch_size
      real, intent(inout) :: a(stride_a,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgetrfnp_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_zgetrfnp_omp_offload_ilp64(m, n, a, lda,   &
                                                       info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(out) :: info
      complex*16, intent(inout) :: a(lda,*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgetrfnp_omp_offload_ilp64

      subroutine mkl_lapack_zgetrfnp_batch_strided_omp_offload_ilp64(m,&
                                                      n, a, lda,       &
                                                      stride_a,        &
                                                      batch_size, info,&
                                                      interop) bind(c)
      use, intrinsic :: iso_c_binding
      integer, intent(in) :: m
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: batch_size
      complex*16, intent(inout) :: a(stride_a,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgetrfnp_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_cgetrsnp_batch_strided_omp_offload_ilp64(trans,&
                                                             n, nrhs, a,   &
                                                             lda, stride_a,&
                                                             b, ldb,       &
                                                             stride_b,     &
                                                             batch_size,   &
                                                             info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: trans
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: ldb
      integer, intent(in) :: stride_b
      integer, intent(in) :: batch_size
      complex*8, intent(in) :: a(stride_a,*)
      complex*8, intent(inout) :: b(stride_b,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgetrsnp_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_dgetrsnp_batch_strided_omp_offload_ilp64(trans,&
                                                             n, nrhs, a,   &
                                                             lda, stride_a,&
                                                             b, ldb,       &
                                                             stride_b,     &
                                                             batch_size,   &
                                                             info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: trans
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: ldb
      integer, intent(in) :: stride_b
      integer, intent(in) :: batch_size
      double precision, intent(in) :: a(stride_a,*)
      double precision, intent(inout) :: b(stride_b,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgetrsnp_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_sgetrsnp_batch_strided_omp_offload_ilp64(trans,&
                                                             n, nrhs, a,   &
                                                             lda, stride_a,&
                                                             b, ldb,       &
                                                             stride_b,     &
                                                             batch_size,   &
                                                             info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: trans
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: ldb
      integer, intent(in) :: stride_b
      integer, intent(in) :: batch_size
      real, intent(in) :: a(stride_a,*)
      real, intent(inout) :: b(stride_b,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgetrsnp_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_zgetrsnp_batch_strided_omp_offload_ilp64(trans,&
                                                             n, nrhs, a,   &
                                                             lda, stride_a,&
                                                             b, ldb,       &
                                                             stride_b,     &
                                                             batch_size,   &
                                                             info, interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: trans
      integer, intent(in) :: n
      integer, intent(in) :: nrhs
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: ldb
      integer, intent(in) :: stride_b
      integer, intent(in) :: batch_size
      complex*16, intent(in) :: a(stride_a,*)
      complex*16, intent(inout) :: b(stride_b,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgetrsnp_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_dpotrf_omp_offload_ilp64(uplo, n, a, lda, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: uplo
        integer, intent(in) :: n
        double precision, intent(inout) :: a(lda,*)
        integer, intent(in) :: lda
        integer, intent(out) :: info
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dpotrf_omp_offload_ilp64

      subroutine mkl_lapack_spotrf_omp_offload_ilp64(uplo, n, a, lda, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: uplo
        integer, intent(in) :: n
        real, intent(inout) :: a(lda,*)
        integer, intent(in) :: lda
        integer, intent(out) :: info
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_spotrf_omp_offload_ilp64

      subroutine mkl_lapack_cpotrf_omp_offload_ilp64(uplo, n, a, lda, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: uplo
        integer, intent(in) :: n
        complex*8, intent(inout) :: a(lda,*)
        integer, intent(in) :: lda
        integer, intent(out) :: info
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cpotrf_omp_offload_ilp64

      subroutine mkl_lapack_zpotrf_omp_offload_ilp64(uplo, n, a, lda, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: uplo
        integer, intent(in) :: n
        complex*16, intent(inout) :: a(lda,*)
        integer, intent(in) :: lda
        integer, intent(out) :: info
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zpotrf_omp_offload_ilp64


      subroutine mkl_lapack_dpotrf_batch_strided_omp_offload_ilp64(uplo,&
                                                       n, a, lda,       &
                                                       stride_a,        &
                                                       batch_size, info,&
                                                       interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: batch_size
      double precision, intent(inout) :: a(stride_a,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dpotrf_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_spotrf_batch_strided_omp_offload_ilp64(uplo,&
                                                       n, a, lda,       &
                                                       stride_a,        &
                                                       batch_size, info,&
                                                       interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: batch_size
      real, intent(inout) :: a(stride_a,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_spotrf_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_cpotrf_batch_strided_omp_offload_ilp64(uplo,&
                                                       n, a, lda,       &
                                                       stride_a,        &
                                                       batch_size, info,&
                                                       interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: batch_size
      complex*8, intent(inout) :: a(stride_a,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cpotrf_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_zpotrf_batch_strided_omp_offload_ilp64(uplo,&
                                                       n, a, lda,       &
                                                       stride_a,        &
                                                       batch_size, info,&
                                                       interop) bind(c)
      use, intrinsic :: iso_c_binding
      character*1, intent(in) :: uplo
      integer, intent(in) :: n
      integer, intent(in) :: lda
      integer, intent(in) :: stride_a
      integer, intent(in) :: batch_size
      complex*16, intent(inout) :: a(stride_a,*)
      integer, intent(out) :: info(*)
      type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zpotrf_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_dpotri_omp_offload_ilp64(uplo, n, a, lda, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: uplo
        integer, intent(in) :: n
        double precision, intent(inout) :: a(lda,*)
        integer, intent(in) :: lda
        integer, intent(out) :: info
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dpotri_omp_offload_ilp64

      subroutine mkl_lapack_spotri_omp_offload_ilp64(uplo, n, a, lda, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: uplo
        integer, intent(in) :: n
        real, intent(inout) :: a(lda,*)
        integer, intent(in) :: lda
        integer, intent(out) :: info
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_spotri_omp_offload_ilp64

      subroutine mkl_lapack_cpotri_omp_offload_ilp64(uplo, n, a, lda, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: uplo
        integer, intent(in) :: n
        complex*8, intent(inout) :: a(lda,*)
        integer, intent(in) :: lda
        integer, intent(out) :: info
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cpotri_omp_offload_ilp64

      subroutine mkl_lapack_zpotri_omp_offload_ilp64(uplo, n, a, lda, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: uplo
        integer, intent(in) :: n
        complex*16, intent(inout) :: a(lda,*)
        integer, intent(in) :: lda
        integer, intent(out) :: info
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zpotri_omp_offload_ilp64


      subroutine mkl_lapack_dpotrs_omp_offload_ilp64(uplo, n, nrhs, a, lda, b, ldb, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: uplo
        integer, intent(in) :: n, nrhs, lda, ldb
        double precision, intent(in) :: a(lda,*)
        double precision, intent(inout) :: b(ldb,*)
        integer, intent(out) :: info
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dpotrs_omp_offload_ilp64

      subroutine mkl_lapack_spotrs_omp_offload_ilp64(uplo, n, nrhs, a, lda, b, ldb, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: uplo
        integer, intent(in) :: n, nrhs, lda, ldb
        real, intent(in) :: a(lda,*)
        real, intent(inout) :: b(ldb,*)
        integer, intent(out) :: info
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_spotrs_omp_offload_ilp64

      subroutine mkl_lapack_cpotrs_omp_offload_ilp64(uplo, n, nrhs, a, lda, b, ldb, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: uplo
        integer, intent(in) :: n, nrhs, lda, ldb
        complex*8, intent(in) :: a(lda,*)
        complex*8, intent(inout) :: b(ldb,*)
        integer, intent(out) :: info
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cpotrs_omp_offload_ilp64

      subroutine mkl_lapack_zpotrs_omp_offload_ilp64(uplo, n, nrhs, a, lda, b, ldb, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: uplo
        integer, intent(in) :: n, nrhs, lda, ldb
        complex*16, intent(in) :: a(lda,*)
        complex*16, intent(inout) :: b(ldb,*)
        integer, intent(out) :: info
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zpotrs_omp_offload_ilp64

      subroutine mkl_lapack_sgels_batch_strided_omp_offload_ilp64(trans, m, n, nrhs, a, lda, stride_a, b, ldb, stride_b, batch_size, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: trans
        integer, intent(in) :: m
        integer, intent(in) :: n
        integer, intent(in) :: nrhs
        integer, intent(in) :: lda
        integer, intent(in) :: stride_a
        integer, intent(in) :: ldb
        integer, intent(in) :: stride_b
        integer, intent(in) :: batch_size
        real,    intent(inout) :: a(stride_a,*)
        real,    intent(inout) :: b(stride_b,*)
        integer, intent(out) :: info(*)
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_sgels_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_dgels_batch_strided_omp_offload_ilp64(trans, m, n, nrhs, a, lda, stride_a, b, ldb, stride_b, batch_size, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: trans
        integer, intent(in) :: m
        integer, intent(in) :: n
        integer, intent(in) :: nrhs
        integer, intent(in) :: lda
        integer, intent(in) :: stride_a
        integer, intent(in) :: ldb
        integer, intent(in) :: stride_b
        integer, intent(in) :: batch_size
        double precision, intent(inout) :: a(stride_a,*)
        double precision, intent(inout) :: b(stride_b,*)
        integer, intent(out) :: info(*)
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_dgels_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_cgels_batch_strided_omp_offload_ilp64(trans, m, n, nrhs, a, lda, stride_a, b, ldb, stride_b, batch_size, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: trans
        integer, intent(in) :: m
        integer, intent(in) :: n
        integer, intent(in) :: nrhs
        integer, intent(in) :: lda
        integer, intent(in) :: stride_a
        integer, intent(in) :: ldb
        integer, intent(in) :: stride_b
        integer, intent(in) :: batch_size
        complex*8, intent(inout) :: a(stride_a,*)
        complex*8, intent(inout) :: b(stride_b,*)
        integer, intent(out) :: info(*)
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_cgels_batch_strided_omp_offload_ilp64

      subroutine mkl_lapack_zgels_batch_strided_omp_offload_ilp64(trans, m, n, nrhs, a, lda, stride_a, b, ldb, stride_b, batch_size, info, interop) bind(c)
        use, intrinsic :: iso_c_binding
        character*1, intent(in) :: trans
        integer, intent(in) :: m
        integer, intent(in) :: n
        integer, intent(in) :: nrhs
        integer, intent(in) :: lda
        integer, intent(in) :: stride_a
        integer, intent(in) :: ldb
        integer, intent(in) :: stride_b
        integer, intent(in) :: batch_size
        complex*16, intent(inout) :: a(stride_a,*)
        complex*16, intent(inout) :: b(stride_b,*)
        integer, intent(out) :: info(*)
        type(c_ptr), intent(in), value :: interop
      end subroutine mkl_lapack_zgels_batch_strided_omp_offload_ilp64

   end interface
