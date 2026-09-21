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

include "mkl_vsl_omp_variant.f90"

module onemkl_vsl_omp_offload
  use onemkl_vsl_omp_variant

  !++
  !  VSL CONTINUOUS DISTRIBUTION GENERATOR FUNCTION INTERFACES.
  !--

  ! Uniform distribution
  interface
     integer function vsrnguniform(method, stream, n, r, a, b)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: b
       !$omp  declare variant( vsrnguniform:mkl_vsl_vsrnguniform_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnguniform
  end interface

  interface
     integer function vdrnguniform (method, stream, n, r, a, b)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: b
       !$omp  declare variant( vdrnguniform:mkl_vsl_vdrnguniform_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnguniform
  end interface
  
  ! Gaussian distribution
  interface
     integer function vsrnggaussian (method, stream, n, r, a, sigma)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: sigma
       !$omp  declare variant( vsrnggaussian:mkl_vsl_vsrnggaussian_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnggaussian
  end interface

  interface
     integer function vdrnggaussian (method, stream, n, r, a, sigma)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: sigma
       !$omp  declare variant( vdrnggaussian:mkl_vsl_vdrnggaussian_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnggaussian
  end interface

  ! GaussianMV distribution
  interface
     integer function vsrnggaussianmv (method, stream, n, r, dimen, mstorage, a, t)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       integer, intent(in)      :: dimen
       real(kind=4),intent(out) :: r(dimen,n)
       integer, intent(in)      :: mstorage
       real(kind=4),intent(in)  :: a(dimen)
       real(kind=4),intent(in)  :: t(dimen,dimen)
       !$omp  declare variant( vsrnggaussianmv:mkl_vsl_vsrnggaussianmv_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnggaussianmv
  end interface

  interface
     integer function vdrnggaussianmv (method, stream, n, r, dimen, mstorage, a, t)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       integer, intent(in)      :: dimen
       real(kind=8),intent(out) :: r(dimen,n)
       integer, intent(in)      :: mstorage
       real(kind=8),intent(in)  :: a(dimen)
       real(kind=8),intent(in)  :: t(dimen,dimen)
       !$omp  declare variant( vdrnggaussianmv:mkl_vsl_vdrnggaussianmv_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnggaussianmv
  end interface

  ! Multinomial distribution
  interface
     integer function virngmultinomial (method, stream, n, r, ntrial, k, p)
       import :: VSL_STREAM_STATE
       integer,intent(in)           :: method
       type(VSL_STREAM_STATE)       :: stream
       integer,intent(in)           :: n
       integer(kind=4),intent(out) :: r(n)
       integer, intent(in)          :: ntrial
       integer, intent(in)          :: k
       real(kind=8),intent(in)      :: p(k)
       !$omp  declare variant( virngmultinomial:mkl_vsl_virngmultinomial_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virngmultinomial
  end interface

  ! PoissonV distribution
  interface
     integer function virngpoissonv (method, stream, n, r, ntrial, k, lambda)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: lambda(n)
       !$omp  declare variant( virngpoissonv:mkl_vsl_virngpoissonv_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virngpoissonv
  end interface

  ! Exponential distribution
  interface
     integer function vsrngexponential (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrngexponential:mkl_vsl_vsrngexponential_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrngexponential
  end interface

  interface
     integer function vdrngexponential (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrngexponential:mkl_vsl_vdrngexponential_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrngexponential
  end interface

  ! Laplace distribution
  interface
     integer function vsrnglaplace (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrnglaplace:mkl_vsl_vsrnglaplace_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnglaplace
  end interface

  interface
     integer function vdrnglaplace (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrnglaplace:mkl_vsl_vdrnglaplace_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnglaplace
  end interface

  ! Weibull distribution
  interface
     integer function vsrngweibull (method, stream, n, r, alpha, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: alpha
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrngweibull:mkl_vsl_vsrngweibull_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrngweibull
  end interface

  interface
     integer function vdrngweibull (method, stream, n, r, alpha, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: alpha
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrngweibull:mkl_vsl_vdrngweibull_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrngweibull
  end interface

  ! Cauchy distribution
  interface
     integer function vsrngcauchy (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrngcauchy:mkl_vsl_vsrngcauchy_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrngcauchy
  end interface

  interface
     integer function vdrngcauchy (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrngcauchy:mkl_vsl_vdrngcauchy_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrngcauchy
  end interface

  ! Rayleigh distribution
  interface
     integer function vsrngrayleigh (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrngrayleigh:mkl_vsl_vsrngrayleigh_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrngrayleigh
  end interface

  interface
     integer function vdrngrayleigh (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrngrayleigh:mkl_vsl_vdrngrayleigh_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrngrayleigh
  end interface

  ! Lognormal distribution
  interface
     integer function vsrnglognormal (method, stream, n, r, a, sigma, b, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: sigma
       real(kind=4),intent(in)  :: b
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrnglognormal:mkl_vsl_vsrnglognormal_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnglognormal
  end interface

  interface
     integer function vdrnglognormal (method, stream, n, r, a, sigma, b, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: sigma
       real(kind=8),intent(in)  :: b
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrnglognormal:mkl_vsl_vdrnglognormal_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnglognormal
  end interface

  ! Gumbel distribution
  interface
     integer function vsrnggumbel (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrnggumbel:mkl_vsl_vsrnggumbel_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnggumbel
  end interface

  interface
     integer function vdrnggumbel (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrnggumbel:mkl_vsl_vdrnggumbel_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnggumbel
  end interface

  ! Beta distribution
  interface
     integer function vsrngbeta (method, stream, n, r, p, q, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: p
       real(kind=4),intent(in)  :: q
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrngbeta:mkl_vsl_vsrngbeta_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrngbeta
  end interface

  interface
     integer function vdrngbeta (method, stream, n, r, p, q, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: p
       real(kind=8),intent(in)  :: q
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrngbeta:mkl_vsl_vdrngbeta_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrngbeta
    end interface

  ! Gamma distribution
  interface
     integer function vsrnggamma (method, stream, n, r, alpha, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: alpha
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrnggamma:mkl_vsl_vsrnggamma_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnggamma
  end interface

  interface
     integer function vdrnggamma (method, stream, n, r, alpha, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: alpha
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrnggamma:mkl_vsl_vdrnggamma_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnggamma
    end interface

  ! ChiSquare distribution
  interface
     integer function vsrngchisquare (method, stream, n, r,v)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       integer,intent(in)       :: v
       !$omp  declare variant( vsrngchisquare:mkl_vsl_vsrngchisquare_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrngchisquare
  end interface

  interface
     integer function vdrngchisquare (method, stream, n, r, v)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       integer,intent(in)       :: v
       !$omp  declare variant( vdrngchisquare:mkl_vsl_vdrngchisquare_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrngchisquare
    end interface

!++
!  VSL DISCRETE DISTRIBUTION GENERATOR FUNCTION INTERFACES.
!--
  ! Uniform distribution
  interface
     integer function virnguniform (method, stream, n, r, a, b)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer(kind=4),intent(in)  :: a
       integer(kind=4),intent(in)  :: b
       !$omp  declare variant( virnguniform:mkl_vsl_virnguniform_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virnguniform
  end interface

  ! UniformBits distribution
  interface
     integer function virnguniformbits (method, stream, n, r)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       !$omp  declare variant( virnguniformbits:mkl_vsl_virnguniformbits_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virnguniformbits
  end interface

  ! UniformBits32 distribution
  interface
     integer function virnguniformbits32 (method, stream, n, r)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       !$omp  declare variant( virnguniformbits32:mkl_vsl_virnguniformbits32_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virnguniformbits32
  end interface

  ! UniformBits64 distribution
  interface
     integer function virnguniformbits64 (method, stream, n, r)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=8),intent(out) :: r(n)
       !$omp  declare variant( virnguniformbits64:mkl_vsl_virnguniformbits64_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virnguniformbits64
  end interface

  ! Bernoulli distribution
  interface
     integer function virngbernoulli (method, stream, n, r, p)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: p
       !$omp  declare variant( virngbernoulli:mkl_vsl_virngbernoulli_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virngbernoulli
  end interface

  ! Geometric distribution
  interface
     integer function virnggeometric (method, stream, n, r, p)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: p
       !$omp  declare variant( virnggeometric:mkl_vsl_virnggeometric_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virnggeometric
  end interface

  ! Hypergeometric distribution
  interface
     integer function virnghypergeometric (method, stream, n, r, l, s, m)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer(kind=4),intent(in)  :: l
       integer(kind=4),intent(in)  :: s
       integer(kind=4),intent(in)  :: m
       !$omp  declare variant( virnghypergeometric:mkl_vsl_virnghypergeometric_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virnghypergeometric
  end interface

  ! Binomial distribution
  interface
     integer function virngbinomial (method, stream, n, r, ntrial, p)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer(kind=4),intent(in)  :: ntrial
       real(kind=8),intent(in)     :: p
       !$omp  declare variant( virngbinomial:mkl_vsl_virngbinomial_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virngbinomial
  end interface

  ! Negbinomial distribution
  interface
     integer function virngnegbinomial (method, stream, n, r, a, p)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: a
       real(kind=8),intent(in)     :: p
       !$omp  declare variant( virngnegbinomial:mkl_vsl_virngnegbinomial_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virngnegbinomial
  end interface

  ! Poisson distribution
  interface
     integer function virngpoisson (method, stream, n, r, lambda)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: lambda
       !$omp  declare variant( virngpoisson:mkl_vsl_virngpoisson_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virngpoisson
  end interface

  !++
  !  VSL SERVICE FUNCTION INTERFACES.
  !--

  ! NewStream - stream creation/initialization
        interface
          integer function vslnewstream( stream, brng, seed )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE) :: stream
            integer,intent(in)     :: brng
            integer,intent(in)     :: seed
     end function
        end interface

  ! NewStreamEx - advanced stream creation/initialization
        interface
          integer function vslnewstreamex( stream, brng, n, params )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE)     :: stream
            integer,intent(in)         :: brng
            integer,intent(in)         :: n
            integer(kind=4),intent(in) :: params(n)
     end function
        end interface

  ! DeleteStream - delete stream
        interface
          integer function vsldeletestream( stream )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE) :: stream
     end function
        end interface

  ! CopyStream - copy all stream information
        interface
          integer function vslcopystream( newstream, srcstream )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE) :: newstream
            type(VSL_STREAM_STATE) :: srcstream
     end function
        end interface

  ! CopyStreamState - copy stream state only
        interface
          integer function vslcopystreamstate( deststream, srcstream )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE) :: deststream
            type(VSL_STREAM_STATE) :: srcstream
     end function
        end interface

  ! LeapfrogStream - leapfrog method
        interface
          integer function vslleapfrogstream( stream, k, nstreams )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE) :: stream
            integer,intent(in)     :: k
            integer,intent(in)     :: nstreams
     end function
        end interface

  ! SkipAheadStream - skip-ahead method
        interface
          integer function vslskipaheadstream( stream, nskip )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE)     :: stream
            integer(kind=8),intent(in) :: nskip
     end function
        end interface

  ! SkipAheadStreamEx - skip-ahead extended method
        interface
          integer function vslskipaheadstreamex( stream, n, params )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE)     :: stream
            integer,intent(in)         :: n
            integer(kind=8),intent(in) :: params(n)
     end function
        end interface

  ! GetBrngProperties - get BRNG properties
        interface
          integer function vslgetbrngproperties( brng, properties )
              import :: VSL_BRNG_PROPERTIES
            integer(kind=4),intent(in)            :: brng
            type(VSL_BRNG_PROPERTIES),intent(out) :: properties
     end function
        end interface

  ! GetStreamStateBrng - get BRNG associated with given stream
        interface
          integer function vslgetstreamstatebrng( stream )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE) :: stream
     end function
        end interface

  ! SaveStreamF - save stream to file
        interface
          integer function vslsavestreamf( stream, fname )
              import :: VSL_STREAM_STATE
            character(*)           :: fname
            type(VSL_STREAM_STATE) :: stream
     end function
        end interface

  ! LoadStreamF - save stream to file
        interface
          integer function vslloadstreamf( stream, fname )
              import :: VSL_STREAM_STATE
            character(*)           :: fname
            type(VSL_STREAM_STATE) :: stream
     end function
        end interface

  ! SaveStreamM - save stream to memory
        interface
          integer function vslsavestreamm( stream, memptr )
              import :: VSL_STREAM_STATE
            integer(kind=1),dimension(*),intent(out)::memptr
            type(VSL_STREAM_STATE),intent(in)       :: stream
     end function
        end interface

  ! LoadStreamM - load stream from memory
        interface
          integer function vslloadstreamm( stream, memptr )
              import :: VSL_STREAM_STATE
            integer(kind=1),dimension(*),intent(in)::memptr
            type(VSL_STREAM_STATE),intent(out)      ::stream
     end function
        end interface

  ! GetStreamSize - get size of random stream
        interface
          integer function vslgetstreamsize( stream )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE),intent(in) :: stream
     end function
        end interface

  !++
  !  SUMMARY STATISTICS LIBRARY ROUTINES
  !--

  !  Compute routines
      interface
        integer function vsldsscompute(task, estimates, method)
              import :: VSL_SS_TASK
              type(VSL_SS_TASK)            :: task
              integer(kind=8),intent(in)   :: estimates
              integer,intent(in)           :: method
              !$omp  declare variant( vsldsscompute:mkl_vsl_dsscompute_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync))
     end function
      end interface

      interface
        integer function vslssscompute(task, estimates, method)
              import :: VSL_SS_TASK
              type(VSL_SS_TASK)            :: task
              integer(kind=8),intent(in)   :: estimates
              integer,intent(in)           :: method
              !$omp  declare variant( vslssscompute:mkl_vsl_ssscompute_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync))
     end function
      end interface

    !  Task constructors
      INTERFACE
       INTEGER FUNCTION vsldssnewtask(task,p,n,x_storage,x,w,indices)
             IMPORT :: VSL_SS_TASK
             TYPE(VSL_SS_TASK)       :: task
             INTEGER,INTENT(IN)      :: p
             INTEGER,INTENT(IN)      :: n
             INTEGER,INTENT(IN)      :: x_storage
             REAL(KIND=8),INTENT(IN) :: x(n,p)
             REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL:: w
             INTEGER,DIMENSION(*),INTENT(IN),OPTIONAL:: indices
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslsssnewtask(task,p,n,x_storage,x,w,indices)
             IMPORT :: VSL_SS_TASK
             TYPE(VSL_SS_TASK)       :: task
             INTEGER,INTENT(IN)      :: p
             INTEGER,INTENT(IN)      :: n
             INTEGER,INTENT(IN)      :: x_storage
             REAL(KIND=4),INTENT(IN) :: x(n,p)
             REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL:: w
             INTEGER,DIMENSION(*),INTENT(IN),OPTIONAL:: indices
     end function
      END INTERFACE


!  Task editors

!  Editor to modify a task parameter
      INTERFACE
       INTEGER FUNCTION vsldssedittask(task,parameter,par_addr)
             IMPORT :: VSL_SS_TASK
             TYPE(VSL_SS_TASK)                    :: task
             INTEGER,INTENT(IN)                   :: parameter
             REAL(KIND=8),DIMENSION(*),INTENT(IN) :: par_addr
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslsssedittask(task,parameter,par_addr)
             IMPORT :: VSL_SS_TASK
             TYPE(VSL_SS_TASK)                    :: task
             INTEGER,INTENT(IN)                   :: parameter
             REAL(KIND=4),DIMENSION(*),INTENT(IN) :: par_addr
     end function
       END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslissedittask(task,parameter,par_addr)
             IMPORT :: VSL_SS_TASK
             TYPE(VSL_SS_TASK)               :: task
             INTEGER,INTENT(IN)              :: parameter
             INTEGER,INTENT(IN)              :: par_addr
     end function
      END INTERFACE


!  Task specific editors

!  Editors to modify moments related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditmoments(task, mean, r2m, r3m, r4m,     &
     &                                                c2m, c3m, c4m)
           IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                             :: task
              REAL(KIND=8),DIMENSION(*),INTENT(IN)          :: mean
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: r2m
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: r3m
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: r4m
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: c2m
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: c3m
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: c4m
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditmoments(task, mean, r2m, r3m, r4m,     &
     &                                                c2m, c3m, c4m)
           IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                             :: task
              REAL(KIND=4),DIMENSION(*),INTENT(IN)          :: mean
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: r2m
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: r3m
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: r4m
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: c2m
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: c3m
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: c4m
     end function
      END INTERFACE

!  Editors to modify sums related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditsums(task, sum, r2s, r3s, r4s,         &
     &                                                c2s, c3s, c4s)
           IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                             :: task
              REAL(KIND=8),DIMENSION(*),INTENT(IN)          :: sum
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: r2s
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: r3s
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: r4s
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: c2s
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: c3s
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: c4s
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditsums(task, sum, r2s, r3s, r4s,         &
     &                                                c2s, c3s, c4s)
           IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                             :: task
              REAL(KIND=4),DIMENSION(*),INTENT(IN)          :: sum
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: r2s
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: r3s
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: r4s
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: c2s
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: c3s
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: c4s
     end function
      END INTERFACE

!  Editors to modify variance-covariance/correlation matrix
!  related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditcovcor(task, mean,cov, cov_storage,    &
     &                                              cor, cor_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                              :: task
              REAL(KIND=8),DIMENSION(*),INTENT(IN)           :: mean
              REAL(KIND=8),DIMENSION(*),INTENT(IN), OPTIONAL :: cov
              INTEGER,INTENT(IN), OPTIONAL                :: cov_storage
              REAL(KIND=8),DIMENSION(*),INTENT(IN), OPTIONAL :: cor
              INTEGER,INTENT(IN), OPTIONAL                :: cor_storage
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditcovcor(task, mean,cov, cov_storage,    &
     &                                              cor, cor_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                              :: task
              REAL(KIND=4),DIMENSION(*),INTENT(IN)           :: mean
              REAL(KIND=4),DIMENSION(*),INTENT(IN), OPTIONAL :: cov
              INTEGER,INTENT(IN), OPTIONAL                :: cov_storage
              REAL(KIND=4),DIMENSION(*),INTENT(IN), OPTIONAL :: cor
              INTEGER,INTENT(IN), OPTIONAL                :: cor_storage
     end function
      END INTERFACE

!  Editors to modify cross-product matrix
!  related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditcp(task, mean, sum,                    &
     &                                              cp, cp_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                              :: task
              REAL(KIND=8),DIMENSION(*),INTENT(IN)           :: mean
              REAL(KIND=8),DIMENSION(*),INTENT(IN), OPTIONAL :: sum
              REAL(KIND=8),DIMENSION(*),INTENT(IN)           :: cp
              INTEGER,INTENT(IN)                          :: cp_storage
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditcp(task, mean, sum,                    &
     &                                              cp, cp_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                              :: task
              REAL(KIND=4),DIMENSION(*),INTENT(IN)           :: mean
              REAL(KIND=4),DIMENSION(*),INTENT(IN), OPTIONAL :: sum
              REAL(KIND=4),DIMENSION(*),INTENT(IN)           :: cp
              INTEGER,INTENT(IN)                          :: cp_storage
     end function
      END INTERFACE

!  Editors to modify partial variance-covariance matrix
!  related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditpartialcovcor(task, p_idx_array,       &
     &                       cov, cov_storage, cor, cor_storage,         &
     &                       p_cov, p_cov_storage, p_cor, p_cor_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)       :: task
              INTEGER,DIMENSION(*),INTENT(IN)      :: p_idx_array
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: cov
              INTEGER,INTENT(IN),OPTIONAL              :: cov_storage
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: cor
              INTEGER,INTENT(IN),OPTIONAL              :: cor_storage
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: p_cov
              INTEGER,INTENT(IN),OPTIONAL      :: p_cov_storage
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: p_cor
              INTEGER,INTENT(IN),OPTIONAL      :: p_cor_storage
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditpartialcovcor(task, p_idx_array,       &
     &                       cov, cov_storage, cor, cor_storage,         &
     &                       p_cov, p_cov_storage, p_cor, p_cor_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)       :: task
              INTEGER,DIMENSION(*),INTENT(IN)      :: p_idx_array
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: cov
              INTEGER,INTENT(IN),OPTIONAL              :: cov_storage
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: cor
              INTEGER,INTENT(IN),OPTIONAL              :: cor_storage
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: p_cov
              INTEGER,INTENT(IN),OPTIONAL      :: p_cov_storage
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: p_cor
              INTEGER,INTENT(IN),OPTIONAL      :: p_cor_storage
     end function
      END INTERFACE

!  Editors to modify quantiles related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditquantiles(task, quant_order_n,         &
     &                                 quant_order,quants,               &
     &                                 order_stats, order_stats_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                         :: task
              INTEGER,INTENT(IN),OPTIONAL               :: quant_order_n
              REAL(KIND=8),INTENT(IN),dimension(*),OPTIONAL::quant_order
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL::quants
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL ::           &
     &                                                     order_stats
              INTEGER,INTENT(IN),OPTIONAL  ::      order_stats_storage
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditquantiles(task, quant_order_n,         &
     &                                 quant_order,quants,               &
     &                                 order_stats, order_stats_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                         :: task
              INTEGER,INTENT(IN),OPTIONAL               :: quant_order_n
              REAL(KIND=4),INTENT(IN),dimension(*),OPTIONAL::quant_order
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL::quants
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL ::           &
     &                                                     order_stats
              INTEGER,INTENT(IN),OPTIONAL  ::      order_stats_storage
     end function
      END INTERFACE


!  Editors to modify stream data quantiles related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditstreamquantiles(task,                  &
     &        quant_order_n, quant_order, quants, nparams, params)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                     :: task
              INTEGER,INTENT(IN)                    :: quant_order_n
              REAL(KIND=8),INTENT(IN),dimension(*)  :: quant_order
              REAL(KIND=8),DIMENSION(*),INTENT(IN)  :: quants
              INTEGER,INTENT(IN)                    :: nparams
              REAL(KIND=8),INTENT(IN), DIMENSION(*) :: params
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditstreamquantiles(task,                  &
     &        quant_order_n, quant_order, quants, nparams, params)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                     :: task
              INTEGER,INTENT(IN)                    :: quant_order_n
              REAL(KIND=4),INTENT(IN),dimension(*)  :: quant_order
              REAL(KIND=4),DIMENSION(*),INTENT(IN)  :: quants
              INTEGER,INTENT(IN)                    :: nparams
              REAL(KIND=4),INTENT(IN), DIMENSION(*) :: params
     end function
      END INTERFACE


!  Editors to modify pooled/group variance-covariance matrix
!  related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditpooledcovariance(task, grp_indices,    &
     &           pld_mean, pld_cov, grp_cov_indices, grp_means, grp_cov)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                    :: task
              INTEGER,DIMENSION(*),INTENT(IN)      :: grp_indices
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: pld_mean
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: pld_cov
              INTEGER,DIMENSION(*),INTENT(IN),OPTIONAL ::grp_cov_indices
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: grp_means
              REAL(KIND=8),DIMENSION(*),INTENT(IN), OPTIONAL :: grp_cov
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditpooledcovariance(task, grp_indices,    &
     &           pld_mean, pld_cov, grp_cov_indices, grp_means, grp_cov)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                    :: task
              INTEGER,DIMENSION(*),INTENT(IN)      :: grp_indices
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: pld_mean
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: pld_cov
              INTEGER,DIMENSION(*),INTENT(IN),OPTIONAL ::grp_cov_indices
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: grp_means
              REAL(KIND=4),DIMENSION(*),INTENT(IN), OPTIONAL :: grp_cov
     end function
      END INTERFACE


!  Editors to modify robust variance-covariance matrix
!  related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditrobustcovariance(task,rcov_storage,    &
     &                                      nparams, params,rmean, rcov)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                    :: task
              INTEGER,INTENT(IN)                   :: rcov_storage
              INTEGER,INTENT(IN)                   :: nparams
              REAL(KIND=8),INTENT(IN)              :: params(nparams)
              REAL(KIND=8),DIMENSION(*),INTENT(IN) :: rmean
              REAL(KIND=8),DIMENSION(*),INTENT(IN) :: rcov
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditrobustcovariance(task,rcov_storage,    &
     &                                      nparams, params,rmean, rcov)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                    :: task
              INTEGER,INTENT(IN)                   :: rcov_storage
              INTEGER,INTENT(IN)                   :: nparams
              REAL(KIND=4),INTENT(IN)              :: params(nparams)
              REAL(KIND=4),DIMENSION(*),INTENT(IN) :: rmean
              REAL(KIND=4),DIMENSION(*),INTENT(IN) :: rcov
     end function
      END INTERFACE


!  Editors to modify outliers detection parameters
      INTERFACE
        INTEGER FUNCTION vsldsseditoutliersdetection(task,               &
     &                                               nparams, params, w)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)       :: task
              INTEGER,INTENT(IN)      :: nparams
              REAL(KIND=8),DIMENSION(*),INTENT(IN) :: params(nparams)
              REAL(KIND=8),DIMENSION(*),INTENT(IN) :: w
     end function
      END INTERFACE

      INTERFACE
        INTEGER FUNCTION vslssseditoutliersdetection(task,               &
     &                                               nparams, params, w)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)       :: task
              INTEGER,INTENT(IN)      :: nparams
              REAL(KIND=4),DIMENSION(*),INTENT(IN) :: params(nparams)
              REAL(KIND=4),DIMENSION(*),INTENT(IN) :: w
     end function
      END INTERFACE

!  Editors to modify missing values parameters
      INTERFACE
        INTEGER FUNCTION vsldsseditmissingvalues(task, nparams, params,  &
     &        init_estimates_n, init_estimates,  prior_n, prior,         &
     &        simul_missing_vals_n,simul_missing_vals,                   &
     &        estimates_n, estimates)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)           :: task
              INTEGER,INTENT(IN)          :: nparams
              REAL(KIND=8),INTENT(IN)     :: params(nparams)
              INTEGER,INTENT(IN),OPTIONAL :: init_estimates_n
              REAL(KIND=8),INTENT(IN), DIMENSION(*),OPTIONAL::           &
     &                                       init_estimates
              INTEGER,INTENT(IN),OPTIONAL :: prior_n
              REAL(KIND=8),INTENT(IN),DIMENSION(*),OPTIONAL :: prior
              INTEGER,INTENT(IN),OPTIONAL :: simul_missing_vals_n
              REAL(KIND=8),INTENT(IN),DIMENSION(*),OPTIONAL ::           &
     &                                 simul_missing_vals
              INTEGER,INTENT(IN),OPTIONAL :: estimates_n
              REAL(KIND=8),INTENT(IN), DIMENSION(*), OPTIONAL ::         &
     &                                 estimates
     end function
      END INTERFACE

      INTERFACE
        INTEGER FUNCTION vslssseditmissingvalues(task, nparams, params,  &
     &        init_estimates_n, init_estimates,  prior_n, prior,         &
     &        simul_missing_vals_n,simul_missing_vals,                   &
     &        estimates_n, estimates)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)           :: task
              INTEGER,INTENT(IN)          :: nparams
              REAL(KIND=4),INTENT(IN)     :: params(nparams)
              INTEGER,INTENT(IN),OPTIONAL :: init_estimates_n
              REAL(KIND=4),INTENT(IN), DIMENSION(*),OPTIONAL::           &
     &                                       init_estimates
              INTEGER,INTENT(IN),OPTIONAL :: prior_n
              REAL(KIND=4),INTENT(IN),DIMENSION(*),OPTIONAL :: prior
              INTEGER,INTENT(IN),OPTIONAL :: simul_missing_vals_n
              REAL(KIND=4),INTENT(IN),DIMENSION(*),OPTIONAL ::           &
     &                                 simul_missing_vals
              INTEGER,INTENT(IN),OPTIONAL :: estimates_n
              REAL(KIND=4),INTENT(IN), DIMENSION(*), OPTIONAL ::         &
     &                                 estimates
     end function
      END INTERFACE

    !     Editors to modify matrix parameterization
    !     related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditcorparameterization (task,             &
     &                             cor, cor_storage, pcor, pcor_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                    :: task
              REAL(KIND=8),DIMENSION(*),INTENT(IN) :: cor
              INTEGER,INTENT(IN)                   :: cor_storage
              REAL(KIND=8),DIMENSION(*),INTENT(IN) :: pcor
              INTEGER,INTENT(IN)                   :: pcor_storage
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditcorparameterization (task,             &
     &                             cor, cor_storage, pcor, pcor_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                    :: task
              REAL(KIND=4),DIMENSION(*),INTENT(IN) :: cor
              INTEGER,INTENT(IN)                   :: cor_storage
              REAL(KIND=4),DIMENSION(*),INTENT(IN) :: pcor
              INTEGER,INTENT(IN)                   :: pcor_storage
     end function
      END INTERFACE
  
    !  Task destructor
    INTERFACE
      INTEGER FUNCTION vslssdeletetask(task)
          IMPORT :: VSL_SS_TASK
            TYPE(VSL_SS_TASK)      :: task
     end function
    END INTERFACE
end module onemkl_vsl_omp_offload


module onemkl_vsl_omp_offload_ilp64
  use onemkl_vsl_omp_offload
end module onemkl_vsl_omp_offload_ilp64


module onemkl_vsl_omp_offload_lp64
  use onemkl_vsl_omp_variant_lp64

  !++
  !  VSL CONTINUOUS DISTRIBUTION GENERATOR FUNCTION INTERFACES.
  !--

  ! Uniform distribution
  interface
     integer function vsrnguniform(method, stream, n, r, a, b)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: b
       !$omp  declare variant( vsrnguniform:mkl_vsl_vsrnguniform_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnguniform
  end interface

  interface
     integer function vdrnguniform (method, stream, n, r, a, b)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: b
       !$omp  declare variant( vdrnguniform:mkl_vsl_vdrnguniform_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnguniform
  end interface
  
  ! Gaussian distribution
  interface
     integer function vsrnggaussian (method, stream, n, r, a, sigma)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: sigma
       !$omp  declare variant( vsrnggaussian:mkl_vsl_vsrnggaussian_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnggaussian
  end interface

  interface
     integer function vdrnggaussian (method, stream, n, r, a, sigma)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: sigma
       !$omp  declare variant( vdrnggaussian:mkl_vsl_vdrnggaussian_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnggaussian
  end interface

  ! GaussianMV distribution
  interface
     integer function vsrnggaussianmv (method, stream, n, r, dimen, mstorage, a, t)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       integer, intent(in)      :: dimen
       real(kind=4),intent(out) :: r(dimen,n)
       integer, intent(in)      :: mstorage
       real(kind=4),intent(in)  :: a(dimen)
       real(kind=4),intent(in)  :: t(dimen,dimen)
       !$omp  declare variant( vsrnggaussianmv:mkl_vsl_vsrnggaussianmv_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnggaussianmv
  end interface

  interface
     integer function vdrnggaussianmv (method, stream, n, r, dimen, mstorage, a, t)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       integer, intent(in)      :: dimen
       real(kind=8),intent(out) :: r(dimen,n)
       integer, intent(in)      :: mstorage
       real(kind=8),intent(in)  :: a(dimen)
       real(kind=8),intent(in)  :: t(dimen,dimen)
       !$omp  declare variant( vdrnggaussianmv:mkl_vsl_vdrnggaussianmv_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnggaussianmv
  end interface

  ! Multinomial distribution
  interface
     integer function virngmultinomial (method, stream, n, r, ntrial, k, p)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer, intent(in)         :: ntrial
       integer, intent(in)         :: k
       real(kind=8),intent(in)     :: p(k)
       !$omp  declare variant( virngmultinomial:mkl_vsl_virngmultinomial_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virngmultinomial
  end interface

  ! PoissonV distribution
  interface
     integer function virngpoissonv (method, stream, n, r, lambda)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: lambda(n)
       !$omp  declare variant( virngpoissonv:mkl_vsl_virngpoissonv_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virngpoissonv
  end interface

  ! Exponential distribution
  interface
     integer function vsrngexponential (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrngexponential:mkl_vsl_vsrngexponential_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrngexponential
  end interface

  interface
     integer function vdrngexponential (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrngexponential:mkl_vsl_vdrngexponential_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrngexponential
  end interface

  ! Laplace distribution
  interface
     integer function vsrnglaplace (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrnglaplace:mkl_vsl_vsrnglaplace_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnglaplace
  end interface

  interface
     integer function vdrnglaplace (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrnglaplace:mkl_vsl_vdrnglaplace_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnglaplace
  end interface

  ! Weibull distribution
  interface
     integer function vsrngweibull (method, stream, n, r, alpha, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: alpha
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrngweibull:mkl_vsl_vsrngweibull_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrngweibull
  end interface

  interface
     integer function vdrngweibull (method, stream, n, r, alpha, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: alpha
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrngweibull:mkl_vsl_vdrngweibull_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrngweibull
  end interface

  ! Cauchy distribution
  interface
     integer function vsrngcauchy (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrngcauchy:mkl_vsl_vsrngcauchy_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrngcauchy
  end interface

  interface
     integer function vdrngcauchy (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrngcauchy:mkl_vsl_vdrngcauchy_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrngcauchy
  end interface

  ! Rayleigh distribution
  interface
     integer function vsrngrayleigh (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrngrayleigh:mkl_vsl_vsrngrayleigh_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrngrayleigh
  end interface

  interface
     integer function vdrngrayleigh (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrngrayleigh:mkl_vsl_vdrngrayleigh_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrngrayleigh
  end interface

  ! Lognormal distribution
  interface
     integer function vsrnglognormal (method, stream, n, r, a, sigma, b, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: sigma
       real(kind=4),intent(in)  :: b
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrnglognormal:mkl_vsl_vsrnglognormal_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnglognormal
  end interface

  interface
     integer function vdrnglognormal (method, stream, n, r, a, sigma, b, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: sigma
       real(kind=8),intent(in)  :: b
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrnglognormal:mkl_vsl_vdrnglognormal_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnglognormal
  end interface

  ! Gumbel distribution
  interface
     integer function vsrnggumbel (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrnggumbel:mkl_vsl_vsrnggumbel_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnggumbel
  end interface

  interface
     integer function vdrnggumbel (method, stream, n, r, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrnggumbel:mkl_vsl_vdrnggumbel_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnggumbel
  end interface

  ! Beta distribution
  interface
     integer function vsrngbeta (method, stream, n, r, p, q, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: p
       real(kind=4),intent(in)  :: q
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrngbeta:mkl_vsl_vsrngbeta_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrngbeta
  end interface

  interface
     integer function vdrngbeta (method, stream, n, r, p, q, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: p
       real(kind=8),intent(in)  :: q
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrngbeta:mkl_vsl_vdrngbeta_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrngbeta
    end interface

  ! Gamma distribution
  interface
     integer function vsrnggamma (method, stream, n, r, alpha, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       real(kind=4),intent(in)  :: alpha
       real(kind=4),intent(in)  :: a
       real(kind=4),intent(in)  :: beta
       !$omp  declare variant( vsrnggamma:mkl_vsl_vsrnggamma_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrnggamma
  end interface

  interface
     integer function vdrnggamma (method, stream, n, r, alpha, a, beta)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       real(kind=8),intent(in)  :: alpha
       real(kind=8),intent(in)  :: a
       real(kind=8),intent(in)  :: beta
       !$omp  declare variant( vdrnggamma:mkl_vsl_vdrnggamma_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrnggamma
    end interface

  ! ChiSquare distribution
  interface
     integer function vsrngchisquare (method, stream, n, r, v)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=4),intent(out) :: r(n)
       integer,intent(in)       :: v
       !$omp  declare variant( vsrngchisquare:mkl_vsl_vsrngchisquare_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vsrngchisquare
  end interface

  interface
     integer function vdrngchisquare (method, stream, n, r, v)
       import :: VSL_STREAM_STATE
       integer,intent(in)       :: method
       type(VSL_STREAM_STATE)   :: stream
       integer,intent(in)       :: n
       real(kind=8),intent(out) :: r(n)
       integer,intent(in)       :: v
       !$omp  declare variant( vdrngchisquare:mkl_vsl_vdrngchisquare_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function vdrngchisquare
    end interface

!++
!  VSL DISCRETE DISTRIBUTION GENERATOR FUNCTION INTERFACES.
!--
  ! Uniform distribution
  interface
     integer function virnguniform (method, stream, n, r, a, b)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer(kind=4),intent(in)  :: a
       integer(kind=4),intent(in)  :: b
       !$omp  declare variant( virnguniform:mkl_vsl_virnguniform_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virnguniform
  end interface

  ! UniformBits distribution
  interface
     integer function virnguniformbits (method, stream, n, r)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       !$omp  declare variant( virnguniformbits:mkl_vsl_virnguniformbits_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virnguniformbits
  end interface

  ! UniformBits32 distribution
  interface
     integer function virnguniformbits32 (method, stream, n, r)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       !$omp  declare variant( virnguniformbits32:mkl_vsl_virnguniformbits32_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virnguniformbits32
  end interface

  ! UniformBits64 distribution
  interface
     integer function virnguniformbits64 (method, stream, n, r)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=8),intent(out) :: r(n)
       !$omp  declare variant( virnguniformbits64:mkl_vsl_virnguniformbits64_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virnguniformbits64
  end interface

  ! Bernoulli distribution
  interface
     integer function virngbernoulli (method, stream, n, r, p)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: p
       !$omp  declare variant( virngbernoulli:mkl_vsl_virngbernoulli_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virngbernoulli
  end interface

  ! Geometric distribution
  interface
     integer function virnggeometric (method, stream, n, r, p)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: p
       !$omp  declare variant( virnggeometric:mkl_vsl_virnggeometric_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virnggeometric
  end interface

  ! Hypergeometric distribution
  interface
     integer function virnghypergeometric (method, stream, n, r, l, s, m)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer(kind=4),intent(in)  :: l
       integer(kind=4),intent(in)  :: s
       integer(kind=4),intent(in)  :: m
       !$omp  declare variant( virnghypergeometric:mkl_vsl_virnghypergeometric_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virnghypergeometric
  end interface

  ! Binomial distribution
  interface
     integer function virngbinomial (method, stream, n, r, ntrial, p)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       integer(kind=4),intent(in)  :: ntrial
       real(kind=8),intent(in)     :: p
       !$omp  declare variant( virngbinomial:mkl_vsl_virngbinomial_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virngbinomial
  end interface

  ! Negbinomial distribution
  interface
     integer function virngnegbinomial (method, stream, n, r, a, p)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: a
       real(kind=8),intent(in)     :: p
       !$omp  declare variant( virngnegbinomial:mkl_vsl_virngnegbinomial_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virngnegbinomial
  end interface

  ! Poisson distribution
  interface
     integer function virngpoisson (method, stream, n, r, lambda)
       import :: VSL_STREAM_STATE
       integer,intent(in)          :: method
       type(VSL_STREAM_STATE)      :: stream
       integer,intent(in)          :: n
       integer(kind=4),intent(out) :: r(n)
       real(kind=8),intent(in)     :: lambda
       !$omp  declare variant( virngpoisson:mkl_vsl_virngpoisson_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync)) adjust_args(need_device_addr:r)
     end function virngpoisson
  end interface

  !++
  !  VSL SERVICE FUNCTION INTERFACES.
  !--

  ! NewStream - stream creation/initialization
        interface
          integer function vslnewstream( stream, brng, seed )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE) :: stream
            integer,intent(in)     :: brng
            integer,intent(in)     :: seed
     end function
        end interface

  ! NewStreamEx - advanced stream creation/initialization
        interface
          integer function vslnewstreamex( stream, brng, n, params )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE)     :: stream
            integer,intent(in)         :: brng
            integer,intent(in)         :: n
            integer(kind=4),intent(in) :: params(n)
     end function
        end interface

  ! DeleteStream - delete stream
        interface
          integer function vsldeletestream( stream )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE) :: stream
     end function
        end interface

  ! CopyStream - copy all stream information
        interface
          integer function vslcopystream( newstream, srcstream )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE) :: newstream
            type(VSL_STREAM_STATE) :: srcstream
     end function
        end interface

  ! CopyStreamState - copy stream state only
        interface
          integer function vslcopystreamstate( deststream, srcstream )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE) :: deststream
            type(VSL_STREAM_STATE) :: srcstream
     end function
        end interface

  ! LeapfrogStream - leapfrog method
        interface
          integer function vslleapfrogstream( stream, k, nstreams )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE) :: stream
            integer,intent(in)     :: k
            integer,intent(in)     :: nstreams
     end function
        end interface

  ! SkipAheadStream - skip-ahead method
        interface
          integer function vslskipaheadstream( stream, nskip )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE)     :: stream
            integer(kind=8),intent(in) :: nskip
     end function
        end interface

  ! SkipAheadStreamEx - skip-ahead extended method
        interface
          integer function vslskipaheadstreamex( stream, n, params )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE)     :: stream
            integer,intent(in)         :: n
            integer(kind=8),intent(in) :: params(n)
     end function
        end interface

  ! GetBrngProperties - get BRNG properties
        interface
          integer function vslgetbrngproperties( brng, properties )
              import :: VSL_BRNG_PROPERTIES
            integer(kind=4),intent(in)            :: brng
            type(VSL_BRNG_PROPERTIES),intent(out) :: properties
     end function
        end interface

  ! GetStreamStateBrng - get BRNG associated with given stream
        interface
          integer function vslgetstreamstatebrng( stream )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE) :: stream
     end function
        end interface

  ! SaveStreamF - save stream to file
        interface
          integer function vslsavestreamf( stream, fname )
              import :: VSL_STREAM_STATE
            character(*)           :: fname
            type(VSL_STREAM_STATE) :: stream
     end function
        end interface

  ! LoadStreamF - save stream to file
        interface
          integer function vslloadstreamf( stream, fname )
              import :: VSL_STREAM_STATE
            character(*)           :: fname
            type(VSL_STREAM_STATE) :: stream
     end function
        end interface

  ! SaveStreamM - save stream to memory
        interface
          integer function vslsavestreamm( stream, memptr )
              import :: VSL_STREAM_STATE
            integer(kind=1),dimension(*),intent(out)::memptr
            type(VSL_STREAM_STATE),intent(in)       :: stream
     end function
        end interface

  ! LoadStreamM - load stream from memory
        interface
          integer function vslloadstreamm( stream, memptr )
              import :: VSL_STREAM_STATE
            integer(kind=1),dimension(*),intent(in)::memptr
            type(VSL_STREAM_STATE),intent(out)      ::stream
     end function
        end interface

  ! GetStreamSize - get size of random stream
        interface
          integer function vslgetstreamsize( stream )
              import :: VSL_STREAM_STATE
            type(VSL_STREAM_STATE),intent(in) :: stream
     end function
        end interface

  !++
  !  SUMMARY STATISTICS LIBRARY ROUTINES
  !--

  !  Compute routines
      interface
        integer function vsldsscompute(task, estimates, method)
              import :: VSL_SS_TASK
              type(VSL_SS_TASK)            :: task
              integer(kind=8),intent(in)   :: estimates
              integer,intent(in)           :: method
              !$omp  declare variant( vsldsscompute:mkl_vsl_dsscompute_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync))
     end function
      end interface

      interface
        integer function vslssscompute(task, estimates, method)
              import :: VSL_SS_TASK
              type(VSL_SS_TASK)            :: task
              integer(kind=8),intent(in)   :: estimates
              integer,intent(in)           :: method
              !$omp  declare variant( vslssscompute:mkl_vsl_ssscompute_omp_offload_lp64 ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(targetsync))
     end function
      end interface

  !  Task constructors
      INTERFACE
       INTEGER FUNCTION vsldssnewtask(task,p,n,x_storage,x,w,indices)
             IMPORT :: VSL_SS_TASK
             TYPE(VSL_SS_TASK)       :: task
             INTEGER,INTENT(IN)      :: p
             INTEGER,INTENT(IN)      :: n
             INTEGER,INTENT(IN)      :: x_storage
             REAL(KIND=8),INTENT(IN) :: x(n,p)
             REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL:: w
             INTEGER,DIMENSION(*),INTENT(IN),OPTIONAL:: indices
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslsssnewtask(task,p,n,x_storage,x,w,indices)
             IMPORT :: VSL_SS_TASK
             TYPE(VSL_SS_TASK)       :: task
             INTEGER,INTENT(IN)      :: p
             INTEGER,INTENT(IN)      :: n
             INTEGER,INTENT(IN)      :: x_storage
             REAL(KIND=4),INTENT(IN) :: x(n,p)
             REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL:: w
             INTEGER,DIMENSION(*),INTENT(IN),OPTIONAL:: indices
     end function
      END INTERFACE


!  Task editors

!  Editor to modify a task parameter
      INTERFACE
       INTEGER FUNCTION vsldssedittask(task,parameter,par_addr)
             IMPORT :: VSL_SS_TASK
             TYPE(VSL_SS_TASK)                    :: task
             INTEGER,INTENT(IN)                   :: parameter
             REAL(KIND=8),DIMENSION(*),INTENT(IN) :: par_addr
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslsssedittask(task,parameter,par_addr)
             IMPORT :: VSL_SS_TASK
             TYPE(VSL_SS_TASK)                    :: task
             INTEGER,INTENT(IN)                   :: parameter
             REAL(KIND=4),DIMENSION(*),INTENT(IN) :: par_addr
     end function
       END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslissedittask(task,parameter,par_addr)
             IMPORT :: VSL_SS_TASK
             TYPE(VSL_SS_TASK)               :: task
             INTEGER,INTENT(IN)              :: parameter
             INTEGER,INTENT(IN)              :: par_addr
     end function
      END INTERFACE


!  Task specific editors

!  Editors to modify moments related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditmoments(task, mean, r2m, r3m, r4m,     &
     &                                                c2m, c3m, c4m)
           IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                             :: task
              REAL(KIND=8),DIMENSION(*),INTENT(IN)          :: mean
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: r2m
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: r3m
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: r4m
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: c2m
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: c3m
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: c4m
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditmoments(task, mean, r2m, r3m, r4m,     &
     &                                                c2m, c3m, c4m)
           IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                             :: task
              REAL(KIND=4),DIMENSION(*),INTENT(IN)          :: mean
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: r2m
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: r3m
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: r4m
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: c2m
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: c3m
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: c4m
     end function
      END INTERFACE

!  Editors to modify sums related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditsums(task, sum, r2s, r3s, r4s,         &
     &                                                c2s, c3s, c4s)
           IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                             :: task
              REAL(KIND=8),DIMENSION(*),INTENT(IN)          :: sum
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: r2s
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: r3s
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: r4s
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: c2s
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: c3s
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: c4s
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditsums(task, sum, r2s, r3s, r4s,         &
     &                                                c2s, c3s, c4s)
           IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                             :: task
              REAL(KIND=4),DIMENSION(*),INTENT(IN)          :: sum
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: r2s
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: r3s
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: r4s
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: c2s
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: c3s
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: c4s
     end function
      END INTERFACE

!  Editors to modify variance-covariance/correlation matrix
!  related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditcovcor(task, mean,cov, cov_storage,    &
     &                                              cor, cor_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                              :: task
              REAL(KIND=8),DIMENSION(*),INTENT(IN)           :: mean
              REAL(KIND=8),DIMENSION(*),INTENT(IN), OPTIONAL :: cov
              INTEGER,INTENT(IN), OPTIONAL                :: cov_storage
              REAL(KIND=8),DIMENSION(*),INTENT(IN), OPTIONAL :: cor
              INTEGER,INTENT(IN), OPTIONAL                :: cor_storage
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditcovcor(task, mean,cov, cov_storage,    &
     &                                              cor, cor_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                              :: task
              REAL(KIND=4),DIMENSION(*),INTENT(IN)           :: mean
              REAL(KIND=4),DIMENSION(*),INTENT(IN), OPTIONAL :: cov
              INTEGER,INTENT(IN), OPTIONAL                :: cov_storage
              REAL(KIND=4),DIMENSION(*),INTENT(IN), OPTIONAL :: cor
              INTEGER,INTENT(IN), OPTIONAL                :: cor_storage
     end function
      END INTERFACE

!  Editors to modify cross-product matrix
!  related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditcp(task, mean, sum,                    &
     &                                              cp, cp_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                              :: task
              REAL(KIND=8),DIMENSION(*),INTENT(IN)           :: mean
              REAL(KIND=8),DIMENSION(*),INTENT(IN), OPTIONAL :: sum
              REAL(KIND=8),DIMENSION(*),INTENT(IN)           :: cp
              INTEGER,INTENT(IN)                          :: cp_storage
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditcp(task, mean, sum,                    &
     &                                              cp, cp_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                              :: task
              REAL(KIND=4),DIMENSION(*),INTENT(IN)           :: mean
              REAL(KIND=4),DIMENSION(*),INTENT(IN), OPTIONAL :: sum
              REAL(KIND=4),DIMENSION(*),INTENT(IN)           :: cp
              INTEGER,INTENT(IN)                          :: cp_storage
     end function
      END INTERFACE

!  Editors to modify partial variance-covariance matrix
!  related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditpartialcovcor(task, p_idx_array,       &
     &                       cov, cov_storage, cor, cor_storage,         &
     &                       p_cov, p_cov_storage, p_cor, p_cor_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)       :: task
              INTEGER,DIMENSION(*),INTENT(IN)      :: p_idx_array
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: cov
              INTEGER,INTENT(IN),OPTIONAL              :: cov_storage
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: cor
              INTEGER,INTENT(IN),OPTIONAL              :: cor_storage
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: p_cov
              INTEGER,INTENT(IN),OPTIONAL      :: p_cov_storage
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: p_cor
              INTEGER,INTENT(IN),OPTIONAL      :: p_cor_storage
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditpartialcovcor(task, p_idx_array,       &
     &                       cov, cov_storage, cor, cor_storage,         &
     &                       p_cov, p_cov_storage, p_cor, p_cor_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)       :: task
              INTEGER,DIMENSION(*),INTENT(IN)      :: p_idx_array
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: cov
              INTEGER,INTENT(IN),OPTIONAL              :: cov_storage
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: cor
              INTEGER,INTENT(IN),OPTIONAL              :: cor_storage
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: p_cov
              INTEGER,INTENT(IN),OPTIONAL      :: p_cov_storage
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: p_cor
              INTEGER,INTENT(IN),OPTIONAL      :: p_cor_storage
     end function
      END INTERFACE

!  Editors to modify quantiles related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditquantiles(task, quant_order_n,         &
     &                                 quant_order,quants,               &
     &                                 order_stats, order_stats_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                         :: task
              INTEGER,INTENT(IN),OPTIONAL               :: quant_order_n
              REAL(KIND=8),INTENT(IN),dimension(*),OPTIONAL::quant_order
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL::quants
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL ::           &
     &                                                     order_stats
              INTEGER,INTENT(IN),OPTIONAL  ::      order_stats_storage
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditquantiles(task, quant_order_n,         &
     &                                 quant_order,quants,               &
     &                                 order_stats, order_stats_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                         :: task
              INTEGER,INTENT(IN),OPTIONAL               :: quant_order_n
              REAL(KIND=4),INTENT(IN),dimension(*),OPTIONAL::quant_order
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL::quants
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL ::           &
     &                                                     order_stats
              INTEGER,INTENT(IN),OPTIONAL  ::      order_stats_storage
     end function
      END INTERFACE


!  Editors to modify stream data quantiles related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditstreamquantiles(task,                  &
     &        quant_order_n, quant_order, quants, nparams, params)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                     :: task
              INTEGER,INTENT(IN)                    :: quant_order_n
              REAL(KIND=8),INTENT(IN),dimension(*)  :: quant_order
              REAL(KIND=8),DIMENSION(*),INTENT(IN)  :: quants
              INTEGER,INTENT(IN)                    :: nparams
              REAL(KIND=8),INTENT(IN), DIMENSION(*) :: params
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditstreamquantiles(task,                  &
     &        quant_order_n, quant_order, quants, nparams, params)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                     :: task
              INTEGER,INTENT(IN)                    :: quant_order_n
              REAL(KIND=4),INTENT(IN),dimension(*)  :: quant_order
              REAL(KIND=4),DIMENSION(*),INTENT(IN)  :: quants
              INTEGER,INTENT(IN)                    :: nparams
              REAL(KIND=4),INTENT(IN), DIMENSION(*) :: params
     end function
      END INTERFACE


!  Editors to modify pooled/group variance-covariance matrix
!  related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditpooledcovariance(task, grp_indices,    &
     &           pld_mean, pld_cov, grp_cov_indices, grp_means, grp_cov)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                    :: task
              INTEGER,DIMENSION(*),INTENT(IN)      :: grp_indices
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: pld_mean
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: pld_cov
              INTEGER,DIMENSION(*),INTENT(IN),OPTIONAL ::grp_cov_indices
              REAL(KIND=8),DIMENSION(*),INTENT(IN),OPTIONAL :: grp_means
              REAL(KIND=8),DIMENSION(*),INTENT(IN), OPTIONAL :: grp_cov
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditpooledcovariance(task, grp_indices,    &
     &           pld_mean, pld_cov, grp_cov_indices, grp_means, grp_cov)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                    :: task
              INTEGER,DIMENSION(*),INTENT(IN)      :: grp_indices
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: pld_mean
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: pld_cov
              INTEGER,DIMENSION(*),INTENT(IN),OPTIONAL ::grp_cov_indices
              REAL(KIND=4),DIMENSION(*),INTENT(IN),OPTIONAL :: grp_means
              REAL(KIND=4),DIMENSION(*),INTENT(IN), OPTIONAL :: grp_cov
     end function
      END INTERFACE


!  Editors to modify robust variance-covariance matrix
!  related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditrobustcovariance(task,rcov_storage,    &
     &                                      nparams, params,rmean, rcov)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                    :: task
              INTEGER,INTENT(IN)                   :: rcov_storage
              INTEGER,INTENT(IN)                   :: nparams
              REAL(KIND=8),INTENT(IN)              :: params(nparams)
              REAL(KIND=8),DIMENSION(*),INTENT(IN) :: rmean
              REAL(KIND=8),DIMENSION(*),INTENT(IN) :: rcov
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditrobustcovariance(task,rcov_storage,    &
     &                                      nparams, params,rmean, rcov)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                    :: task
              INTEGER,INTENT(IN)                   :: rcov_storage
              INTEGER,INTENT(IN)                   :: nparams
              REAL(KIND=4),INTENT(IN)              :: params(nparams)
              REAL(KIND=4),DIMENSION(*),INTENT(IN) :: rmean
              REAL(KIND=4),DIMENSION(*),INTENT(IN) :: rcov
     end function
      END INTERFACE


!  Editors to modify outliers detection parameters
      INTERFACE
        INTEGER FUNCTION vsldsseditoutliersdetection(task,               &
     &                                               nparams, params, w)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)       :: task
              INTEGER,INTENT(IN)      :: nparams
              REAL(KIND=8),DIMENSION(*),INTENT(IN) :: params(nparams)
              REAL(KIND=8),DIMENSION(*),INTENT(IN) :: w
     end function
      END INTERFACE

      INTERFACE
        INTEGER FUNCTION vslssseditoutliersdetection(task,               &
     &                                               nparams, params, w)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)       :: task
              INTEGER,INTENT(IN)      :: nparams
              REAL(KIND=4),DIMENSION(*),INTENT(IN) :: params(nparams)
              REAL(KIND=4),DIMENSION(*),INTENT(IN) :: w
     end function
      END INTERFACE

!  Editors to modify missing values parameters
      INTERFACE
        INTEGER FUNCTION vsldsseditmissingvalues(task, nparams, params,  &
     &        init_estimates_n, init_estimates,  prior_n, prior,         &
     &        simul_missing_vals_n,simul_missing_vals,                   &
     &        estimates_n, estimates)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)           :: task
              INTEGER,INTENT(IN)          :: nparams
              REAL(KIND=8),INTENT(IN)     :: params(nparams)
              INTEGER,INTENT(IN),OPTIONAL :: init_estimates_n
              REAL(KIND=8),INTENT(IN), DIMENSION(*),OPTIONAL::           &
     &                                       init_estimates
              INTEGER,INTENT(IN),OPTIONAL :: prior_n
              REAL(KIND=8),INTENT(IN),DIMENSION(*),OPTIONAL :: prior
              INTEGER,INTENT(IN),OPTIONAL :: simul_missing_vals_n
              REAL(KIND=8),INTENT(IN),DIMENSION(*),OPTIONAL ::           &
     &                                 simul_missing_vals
              INTEGER,INTENT(IN),OPTIONAL :: estimates_n
              REAL(KIND=8),INTENT(IN), DIMENSION(*), OPTIONAL ::         &
     &                                 estimates
     end function
      END INTERFACE

      INTERFACE
        INTEGER FUNCTION vslssseditmissingvalues(task, nparams, params,  &
     &        init_estimates_n, init_estimates,  prior_n, prior,         &
     &        simul_missing_vals_n,simul_missing_vals,                   &
     &        estimates_n, estimates)
            IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)           :: task
              INTEGER,INTENT(IN)          :: nparams
              REAL(KIND=4),INTENT(IN)     :: params(nparams)
              INTEGER,INTENT(IN),OPTIONAL :: init_estimates_n
              REAL(KIND=4),INTENT(IN), DIMENSION(*),OPTIONAL::           &
     &                                       init_estimates
              INTEGER,INTENT(IN),OPTIONAL :: prior_n
              REAL(KIND=4),INTENT(IN),DIMENSION(*),OPTIONAL :: prior
              INTEGER,INTENT(IN),OPTIONAL :: simul_missing_vals_n
              REAL(KIND=4),INTENT(IN),DIMENSION(*),OPTIONAL ::           &
     &                                 simul_missing_vals
              INTEGER,INTENT(IN),OPTIONAL :: estimates_n
              REAL(KIND=4),INTENT(IN), DIMENSION(*), OPTIONAL ::         &
     &                                 estimates
     end function
      END INTERFACE

    !     Editors to modify matrix parameterization
    !     related parameters
      INTERFACE
       INTEGER FUNCTION vsldsseditcorparameterization (task,             &
     &                             cor, cor_storage, pcor, pcor_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                    :: task
              REAL(KIND=8),DIMENSION(*),INTENT(IN) :: cor
              INTEGER,INTENT(IN)                   :: cor_storage
              REAL(KIND=8),DIMENSION(*),INTENT(IN) :: pcor
              INTEGER,INTENT(IN)                   :: pcor_storage
     end function
      END INTERFACE

      INTERFACE
       INTEGER FUNCTION vslssseditcorparameterization (task,             &
     &                             cor, cor_storage, pcor, pcor_storage)
              IMPORT :: VSL_SS_TASK
              TYPE(VSL_SS_TASK)                    :: task
              REAL(KIND=4),DIMENSION(*),INTENT(IN) :: cor
              INTEGER,INTENT(IN)                   :: cor_storage
              REAL(KIND=4),DIMENSION(*),INTENT(IN) :: pcor
              INTEGER,INTENT(IN)                   :: pcor_storage
     end function
      END INTERFACE
  
    !  Task destructor
    INTERFACE
      INTEGER FUNCTION vslssdeletetask(task)
          IMPORT :: VSL_SS_TASK
            TYPE(VSL_SS_TASK)      :: task
     end function
    END INTERFACE
end module onemkl_vsl_omp_offload_lp64
