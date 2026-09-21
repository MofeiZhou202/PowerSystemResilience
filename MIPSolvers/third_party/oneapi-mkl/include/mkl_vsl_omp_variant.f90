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
!      oneMKL Library FORTRAN interface for VSL OpenMP offload
!*******************************************************************************

include "mkl_vsl.f90"

module onemkl_vsl_omp_variant
  use MKL_VSL_TYPE

  !++
  !  VSL CONTINUOUS DISTRIBUTION GENERATOR FUNCTION INTERFACES.
  !--

  ! Uniform distribution
  interface
     integer function mkl_vsl_vsrnguniform_omp_offload (method, stream, n, r, a, b, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: b
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnguniform_omp_offload
  end interface

  interface
     integer function mkl_vsl_vdrnguniform_omp_offload (method, stream, n, r, a, b, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: b
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnguniform_omp_offload
  end interface

  ! Gaussian distribution
  interface
     integer function mkl_vsl_vsrnggaussian_omp_offload (method, stream, n, r, a, sigma, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: sigma
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnggaussian_omp_offload
  end interface

  interface
     integer function mkl_vsl_vdrnggaussian_omp_offload (method, stream, n, r, a, sigma, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: sigma
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnggaussian_omp_offload
  end interface

  ! GaussianMV distribution
  interface
     integer function mkl_vsl_vsrnggaussianmv_omp_offload (method, stream, n, r, dimen, mstorage, a, t, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       integer, intent(in)      :: dimen
       real(kind=4),intent(out) :: r(dimen,n)
       integer, intent(in)      :: mstorage
       real(kind=4),intent(in)  :: a(dimen)
       real(kind=4),intent(in)  :: t(dimen,dimen)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnggaussianmv_omp_offload
  end interface

  interface
     integer function mkl_vsl_vdrnggaussianmv_omp_offload (method, stream, n, r, dimen, mstorage, a, t, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       integer, intent(in)      :: dimen
       real(kind=8),intent(out) :: r(dimen,n)
       integer, intent(in)      :: mstorage
       real(kind=8),intent(in)  :: a(dimen)
       real(kind=8),intent(in)  :: t(dimen,dimen)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnggaussianmv_omp_offload
  end interface

  ! Multinomial distribution
  interface
     integer function mkl_vsl_virngmultinomial_omp_offload (method, stream, n, r, ntrial, k, p, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer, intent(in)         :: ntrial
       integer, intent(in)         :: k
       real(kind=8),intent(in)     :: p(k)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virngmultinomial_omp_offload
  end interface

  ! PoissonV distribution
  interface
     integer function mkl_vsl_virngpoissonv_omp_offload (method, stream, n, r, lambda, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: lambda(n)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virngpoissonv_omp_offload
  end interface

  ! Exponential distribution
  interface
     integer function mkl_vsl_vsrngexponential_omp_offload (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrngexponential_omp_offload
  end interface

  interface
     integer function mkl_vsl_vdrngexponential_omp_offload (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrngexponential_omp_offload
  end interface

  ! Laplace distribution
  interface
     integer function mkl_vsl_vsrnglaplace_omp_offload (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnglaplace_omp_offload
  end interface

  interface
     integer function mkl_vsl_vdrnglaplace_omp_offload (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnglaplace_omp_offload
  end interface

  ! Weibull distribution
  interface
     integer function mkl_vsl_vsrngweibull_omp_offload (method, stream, n, r, alpha, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: alpha
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrngweibull_omp_offload
  end interface

  interface
     integer function mkl_vsl_vdrngweibull_omp_offload (method, stream, n, r, alpha, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: alpha
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrngweibull_omp_offload
  end interface

  ! Cauchy distribution
  interface
     integer function mkl_vsl_vsrngcauchy_omp_offload (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrngcauchy_omp_offload
  end interface

  interface
     integer function mkl_vsl_vdrngcauchy_omp_offload (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrngcauchy_omp_offload
  end interface

  ! Rayleigh distribution
  interface
     integer function mkl_vsl_vsrngrayleigh_omp_offload (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrngrayleigh_omp_offload
  end interface

  interface
     integer function mkl_vsl_vdrngrayleigh_omp_offload (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrngrayleigh_omp_offload
  end interface

  ! Lognormal distribution
  interface
     integer function mkl_vsl_vsrnglognormal_omp_offload (method, stream, n, r, a, sigma, b, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: sigma
       real(kind=4),intent(in)  :: b
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnglognormal_omp_offload
  end interface

  interface
     integer function mkl_vsl_vdrnglognormal_omp_offload (method, stream, n, r, a, sigma, b, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: sigma
       real(kind=8),intent(in)  :: b
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnglognormal_omp_offload
  end interface

  ! Gumbel distribution
  interface
     integer function mkl_vsl_vsrnggumbel_omp_offload (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnggumbel_omp_offload
  end interface

  interface
     integer function mkl_vsl_vdrnggumbel_omp_offload (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnggumbel_omp_offload
  end interface

  ! Beta distribution
  interface
     integer function mkl_vsl_vsrngbeta_omp_offload (method, stream, n, r, p, q, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: p
       real(kind=4),intent(in)  :: q
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrngbeta_omp_offload
  end interface

  interface
     integer function mkl_vsl_vdrngbeta_omp_offload (method, stream, n, r, p, q, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: p
       real(kind=8),intent(in)  :: q
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrngbeta_omp_offload
  end interface

  ! Gamma distribution
  interface
     integer function mkl_vsl_vsrnggamma_omp_offload (method, stream, n, r, alpha, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: alpha
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnggamma_omp_offload
  end interface

  interface
     integer function mkl_vsl_vdrnggamma_omp_offload (method, stream, n, r, alpha, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: alpha
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnggamma_omp_offload
  end interface

  ! ChiSquare distribution
  interface
     integer function mkl_vsl_vsrngchisquare_omp_offload (method, stream, n, r, v, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       integer,intent(in)       :: v
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrngchisquare_omp_offload
  end interface

  interface
     integer function mkl_vsl_vdrngchisquare_omp_offload (method, stream, n, r, v, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       integer,intent(in)       :: v
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrngchisquare_omp_offload
  end interface

!++
!  VSL DISCRETE DISTRIBUTION GENERATOR FUNCTION INTERFACES.
!--
  ! Uniform distribution
  interface
     integer function mkl_vsl_virnguniform_omp_offload (method, stream, n, r, a, b, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer(kind=4),intent(in)  :: a
       integer(kind=4),intent(in)  :: b
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virnguniform_omp_offload
  end interface

  ! UniformBits distribution
  interface
     integer function mkl_vsl_virnguniformbits_omp_offload (method, stream, n, r, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virnguniformbits_omp_offload
  end interface

  ! UniformBits32 distribution
  interface
     integer function mkl_vsl_virnguniformbits32_omp_offload (method, stream, n, r, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virnguniformbits32_omp_offload
  end interface

  ! UniformBits64 distribution
  interface
     integer function mkl_vsl_virnguniformbits64_omp_offload (method, stream, n, r, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=8),intent(out) :: r(n)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virnguniformbits64_omp_offload
  end interface

  ! Bernoulli distribution
  interface
     integer function mkl_vsl_virngbernoulli_omp_offload (method, stream, n, r, p, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: p
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virngbernoulli_omp_offload
  end interface

  ! Geometric distribution
  interface
     integer function mkl_vsl_virnggeometric_omp_offload (method, stream, n, r, p, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: p
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virnggeometric_omp_offload
  end interface

  ! Hypergeometric distribution
  interface
     integer function mkl_vsl_virnghypergeometric_omp_offload (method, stream, n, r, l, s, m, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer(kind=4),intent(in)  :: l
       integer(kind=4),intent(in)  :: s
       integer(kind=4),intent(in)  :: m
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virnghypergeometric_omp_offload
  end interface

  ! Binomial distribution
  interface
     integer function mkl_vsl_virngbinomial_omp_offload (method, stream, n, r, ntrial, p, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer(kind=4),intent(in)  :: ntrial
       real(kind=8),intent(in)     :: p
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virngbinomial_omp_offload
  end interface

  ! Negbinomial distribution
  interface
     integer function mkl_vsl_virngnegbinomial_omp_offload (method, stream, n, r, a, p, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: a
       real(kind=8),intent(in)     :: p
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virngnegbinomial_omp_offload
  end interface

  ! Poisson distribution
  interface
     integer function mkl_vsl_virngpoisson_omp_offload (method, stream, n, r, lambda, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: lambda
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virngpoisson_omp_offload
  end interface


  !++
  !  SUMMARY STATISTICS LIBRARY ROUTINES
  !--

  !  Compute routines
  interface
    integer function mkl_vsl_dsscompute_omp_offload(task, estimates, method, interop) BIND(C)
       use,intrinsic :: iso_c_binding
          import :: VSL_SS_TASK
          type(VSL_SS_TASK)            :: task
          integer(kind=8),intent(in)   :: estimates
          integer,intent(in)           :: method
          type(c_ptr),intent(in),value :: interop
    end function mkl_vsl_dsscompute_omp_offload
  end interface

  interface
    integer function mkl_vsl_ssscompute_omp_offload(task, estimates, method, interop) BIND(C)
       use,intrinsic :: iso_c_binding
          import :: VSL_SS_TASK
          type(VSL_SS_TASK)            :: task
          integer(kind=8),intent(in)   :: estimates
          integer,intent(in)           :: method
          type(c_ptr),intent(in),value :: interop
    end function mkl_vsl_ssscompute_omp_offload
  end interface

end module onemkl_vsl_omp_variant


module onemkl_vsl_omp_variant_lp64
  use MKL_VSL_TYPE

  !++
  !  VSL CONTINUOUS DISTRIBUTION GENERATOR FUNCTION INTERFACES.
  !--

  ! Uniform distribution
  interface
     integer function mkl_vsl_vsrnguniform_omp_offload_lp64 (method, stream, n, r, a, b, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: b
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnguniform_omp_offload_lp64
  end interface

  interface
     integer function mkl_vsl_vdrnguniform_omp_offload_lp64 (method, stream, n, r, a, b, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: b
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnguniform_omp_offload_lp64
  end interface

  ! Gaussian distribution
  interface
     integer function mkl_vsl_vsrnggaussian_omp_offload_lp64 (method, stream, n, r, a, sigma, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: sigma
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnggaussian_omp_offload_lp64
  end interface

  interface
     integer function mkl_vsl_vdrnggaussian_omp_offload_lp64 (method, stream, n, r, a, sigma, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: sigma
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnggaussian_omp_offload_lp64
  end interface

  ! GaussianMV distribution
  interface
     integer function mkl_vsl_vsrnggaussianmv_omp_offload_lp64 (method, stream, n, r, dimen, mstorage, a, t, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       integer, intent(in)      :: dimen
       real(kind=4),intent(out) :: r(dimen,n)
       integer, intent(in)      :: mstorage
       real(kind=4),intent(in)  :: a(dimen)
       real(kind=4),intent(in)  :: t(dimen,dimen)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnggaussianmv_omp_offload_lp64
  end interface

  interface
     integer function mkl_vsl_vdrnggaussianmv_omp_offload_lp64 (method, stream, n, r, dimen, mstorage, a, t, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       integer, intent(in)      :: dimen
       real(kind=8),intent(out) :: r(dimen,n)
       integer, intent(in)      :: mstorage
       real(kind=8),intent(in)  :: a(dimen)
       real(kind=8),intent(in)  :: t(dimen,dimen)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnggaussianmv_omp_offload_lp64
  end interface

  ! Multinomial distribution
  interface
     integer function mkl_vsl_virngmultinomial_omp_offload_lp64 (method, stream, n, r, ntrial, k, p, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer, intent(in)         :: ntrial
       integer, intent(in)         :: k
       real(kind=8),intent(in)     :: p(k)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virngmultinomial_omp_offload_lp64
  end interface

  ! PoissonV distribution
  interface
     integer function mkl_vsl_virngpoissonv_omp_offload_lp64 (method, stream, n, r, lambda, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: lambda(n)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virngpoissonv_omp_offload_lp64
  end interface

  ! Exponential distribution
  interface
     integer function mkl_vsl_vsrngexponential_omp_offload_lp64 (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrngexponential_omp_offload_lp64
  end interface

  interface
     integer function mkl_vsl_vdrngexponential_omp_offload_lp64 (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrngexponential_omp_offload_lp64
  end interface

  ! Laplace distribution
  interface
     integer function mkl_vsl_vsrnglaplace_omp_offload_lp64 (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnglaplace_omp_offload_lp64
  end interface

  interface
     integer function mkl_vsl_vdrnglaplace_omp_offload_lp64 (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnglaplace_omp_offload_lp64
  end interface

  ! Weibull distribution
  interface
     integer function mkl_vsl_vsrngweibull_omp_offload_lp64 (method, stream, n, r, alpha, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: alpha
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrngweibull_omp_offload_lp64
  end interface

  interface
     integer function mkl_vsl_vdrngweibull_omp_offload_lp64 (method, stream, n, r, alpha, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: alpha
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrngweibull_omp_offload_lp64
  end interface

  ! Cauchy distribution
  interface
     integer function mkl_vsl_vsrngcauchy_omp_offload_lp64 (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrngcauchy_omp_offload_lp64
  end interface

  interface
     integer function mkl_vsl_vdrngcauchy_omp_offload_lp64 (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrngcauchy_omp_offload_lp64
  end interface

  ! Rayleigh distribution
  interface
     integer function mkl_vsl_vsrngrayleigh_omp_offload_lp64 (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrngrayleigh_omp_offload_lp64
  end interface

  interface
     integer function mkl_vsl_vdrngrayleigh_omp_offload_lp64 (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrngrayleigh_omp_offload_lp64
  end interface

  ! Lognormal distribution
  interface
     integer function mkl_vsl_vsrnglognormal_omp_offload_lp64 (method, stream, n, r, a, sigma, b, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: sigma
       real(kind=4),intent(in)  :: b
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnglognormal_omp_offload_lp64
  end interface

  interface
     integer function mkl_vsl_vdrnglognormal_omp_offload_lp64 (method, stream, n, r, a, sigma, b, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: sigma
       real(kind=8),intent(in)  :: b
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnglognormal_omp_offload_lp64
  end interface

  ! Gumbel distribution
  interface
     integer function mkl_vsl_vsrnggumbel_omp_offload_lp64 (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnggumbel_omp_offload_lp64
  end interface

  interface
     integer function mkl_vsl_vdrnggumbel_omp_offload_lp64 (method, stream, n, r, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnggumbel_omp_offload_lp64
  end interface

  ! Beta distribution
  interface
     integer function mkl_vsl_vsrngbeta_omp_offload_lp64 (method, stream, n, r, p, q, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: p
       real(kind=4),intent(in)  :: q
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrngbeta_omp_offload_lp64
  end interface

  interface
     integer function mkl_vsl_vdrngbeta_omp_offload_lp64 (method, stream, n, r, p, q, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: p
       real(kind=8),intent(in)  :: q
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrngbeta_omp_offload_lp64
  end interface

  ! Gamma distribution
  interface
     integer function mkl_vsl_vsrnggamma_omp_offload_lp64 (method, stream, n, r, alpha, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: alpha
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrnggamma_omp_offload_lp64
  end interface

  interface
     integer function mkl_vsl_vdrnggamma_omp_offload_lp64 (method, stream, n, r, alpha, a, beta, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: alpha
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrnggamma_omp_offload_lp64
  end interface

  ! ChiSquare distribution
  interface
     integer function mkl_vsl_vsrngchisquare_omp_offload_lp64 (method, stream, n, r, v, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       integer,intent(in)       :: v
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vsrngchisquare_omp_offload_lp64
  end interface

  interface
     integer function mkl_vsl_vdrngchisquare_omp_offload_lp64 (method, stream, n, r, v, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       integer,intent(in)       :: v
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_vdrngchisquare_omp_offload_lp64
  end interface

!++
!  VSL DISCRETE DISTRIBUTION GENERATOR FUNCTION INTERFACES.
!--
  ! Uniform distribution
  interface
     integer function mkl_vsl_virnguniform_omp_offload_lp64 (method, stream, n, r, a, b, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer(kind=4),intent(in)  :: a
       integer(kind=4),intent(in)  :: b
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virnguniform_omp_offload_lp64
  end interface

  ! UniformBits distribution
  interface
     integer function mkl_vsl_virnguniformbits_omp_offload_lp64 (method, stream, n, r, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virnguniformbits_omp_offload_lp64
  end interface

  ! UniformBits32 distribution
  interface
     integer function mkl_vsl_virnguniformbits32_omp_offload_lp64 (method, stream, n, r, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virnguniformbits32_omp_offload_lp64
  end interface

  ! UniformBits64 distribution
  interface
     integer function mkl_vsl_virnguniformbits64_omp_offload_lp64 (method, stream, n, r, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=8),intent(out) :: r(n)
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virnguniformbits64_omp_offload_lp64
  end interface

  ! Bernoulli distribution
  interface
     integer function mkl_vsl_virngbernoulli_omp_offload_lp64 (method, stream, n, r, p, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: p
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virngbernoulli_omp_offload_lp64
  end interface

  ! Geometric distribution
  interface
     integer function mkl_vsl_virnggeometric_omp_offload_lp64 (method, stream, n, r, p, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: p
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virnggeometric_omp_offload_lp64
  end interface

  ! Hypergeometric distribution
  interface
     integer function mkl_vsl_virnghypergeometric_omp_offload_lp64 (method, stream, n, r, l, s, m, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer(kind=4),intent(in)  :: l
       integer(kind=4),intent(in)  :: s
       integer(kind=4),intent(in)  :: m
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virnghypergeometric_omp_offload_lp64
  end interface

  ! Binomial distribution
  interface
     integer function mkl_vsl_virngbinomial_omp_offload_lp64 (method, stream, n, r, ntrial, p, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer(kind=4),intent(in)  :: ntrial
       real(kind=8),intent(in)     :: p
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virngbinomial_omp_offload_lp64
  end interface

  ! Negbinomial distribution
  interface
     integer function mkl_vsl_virngnegbinomial_omp_offload_lp64 (method, stream, n, r, a, p, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: a
       real(kind=8),intent(in)     :: p
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virngnegbinomial_omp_offload_lp64
  end interface

  ! Poisson distribution
  interface
     integer function mkl_vsl_virngpoisson_omp_offload_lp64 (method, stream, n, r, lambda, interop) BIND(C)
       use,intrinsic :: iso_c_binding
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: lambda
       type(c_ptr),intent(in),value :: interop
     end function mkl_vsl_virngpoisson_omp_offload_lp64
  end interface

  !++
  !  SUMMARY STATISTICS LIBRARY ROUTINES
  !--

  !  Compute routines
  interface
    integer function mkl_vsl_dsscompute_omp_offload_lp64(task, estimates, method, interop) BIND(C)
       use,intrinsic :: iso_c_binding
          import :: VSL_SS_TASK
          type(VSL_SS_TASK)            :: task
          integer(kind=8),intent(in)   :: estimates
          integer,intent(in)           :: method
          type(c_ptr),intent(in),value :: interop
    end function mkl_vsl_dsscompute_omp_offload_lp64
  end interface

  interface
    integer function mkl_vsl_ssscompute_omp_offload_lp64(task, estimates, method, interop) BIND(C)
       use,intrinsic :: iso_c_binding
          import :: VSL_SS_TASK
          type(VSL_SS_TASK)            :: task
          integer(kind=8),intent(in)   :: estimates
          integer,intent(in)           :: method
          type(c_ptr),intent(in),value :: interop
    end function mkl_vsl_ssscompute_omp_offload_lp64
  end interface
end module onemkl_vsl_omp_variant_lp64
