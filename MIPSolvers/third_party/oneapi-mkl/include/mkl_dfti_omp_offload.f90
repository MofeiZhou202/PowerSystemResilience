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

! Content:
!    Intel(R) oneAPI Math Kernel Library (oneMKL)
!    Discrete Fourier Transform Interface (DFTI)
!*****************************************************************************

MODULE MKL_DFT_TYPE

  TYPE, PUBLIC :: DFTI_DESCRIPTOR
     PRIVATE
     INTEGER :: dontuse
     ! Structure of this type is not used in Fortran code
     ! the pointer to this type is used only
  END TYPE DFTI_DESCRIPTOR

  !======================================================================
  ! These real type kind parameters are not for direct use
  !======================================================================

  INTEGER, PARAMETER :: DFTI_SPKP = SELECTED_REAL_KIND(6,37)
  INTEGER, PARAMETER :: DFTI_DPKP = SELECTED_REAL_KIND(15,307)

  !======================================================================
  ! Descriptor configuration parameters [default values in brackets]
  !======================================================================

  ! Domain for forward transform. No default value
  INTEGER, PARAMETER :: DFTI_FORWARD_DOMAIN = 0

  ! Dimensionality, or rank. No default value
  INTEGER, PARAMETER :: DFTI_DIMENSION = 1

  ! Length(s) of transform. No default value
  INTEGER, PARAMETER :: DFTI_LENGTHS = 2

  ! Floating point precision. No default value
  INTEGER, PARAMETER :: DFTI_PRECISION = 3

  ! Scale factor for forward transform [1.0]
  INTEGER, PARAMETER :: DFTI_FORWARD_SCALE = 4

  ! Scale factor for backward transform [1.0]
  INTEGER, PARAMETER :: DFTI_BACKWARD_SCALE = 5

  ! Exponent sign for forward transform [DFTI_NEGATIVE]
  ! INTEGER, PARAMETER :: DFTI_FORWARD_SIGN = 6 ! NOT IMPLEMENTED

  ! Number of data sets to be transformed [1]
  INTEGER, PARAMETER :: DFTI_NUMBER_OF_TRANSFORMS = 7

  ! Storage of finite complex-valued sequences in complex domain
  ! [DFTI_COMPLEX_COMPLEX]
  INTEGER, PARAMETER :: DFTI_COMPLEX_STORAGE = 8

  ! Storage of finite real-valued sequences in real domain
  ! [DFTI_REAL_REAL]
  INTEGER, PARAMETER :: DFTI_REAL_STORAGE = 9

  ! Storage of finite complex-valued sequences in conjugate-even
  ! domain [DFTI_COMPLEX_REAL]
  INTEGER, PARAMETER :: DFTI_CONJUGATE_EVEN_STORAGE = 10

  ! Placement of result [DFTI_INPLACE]
  INTEGER, PARAMETER :: DFTI_PLACEMENT = 11

  ! Generalized strides for input data layout
  ! [tight, col-major for Fortran]
  INTEGER, PARAMETER :: DFTI_INPUT_STRIDES = 12

  ! Generalized strides for output data layout
  ! [tight, col-major for Fortran]
  INTEGER, PARAMETER :: DFTI_OUTPUT_STRIDES = 13

  ! Distance between first input elements for multiple transforms [0]
  INTEGER, PARAMETER :: DFTI_INPUT_DISTANCE = 14

  ! Distance between first output elements for multiple transforms [0]
  INTEGER, PARAMETER :: DFTI_OUTPUT_DISTANCE = 15

  ! Effort spent in initialization [DFTI_MEDIUM]
  ! INTEGER, PARAMETER :: DFTI_INITIALIZATION_EFFORT = 16 ! NOT IMPLEMENTED

  ! Use of workspace during computation [DFTI_ALLOW]
  INTEGER, PARAMETER :: DFTI_WORKSPACE = 17

  ! Ordering of the result [DFTI_ORDERED]
  INTEGER, PARAMETER :: DFTI_ORDERING = 18

  ! Possible transposition of result [DFTI_NONE]
  INTEGER, PARAMETER :: DFTI_TRANSPOSE = 19

  ! User-settable descriptor name [""]
  INTEGER, PARAMETER :: DFTI_DESCRIPTOR_NAME = 20

  ! Packing format for DFTI_COMPLEX_REAL storage of finite
  ! conjugate-even sequences [DFTI_CCS_FORMAT]
  INTEGER, PARAMETER :: DFTI_PACKED_FORMAT = 21

  ! Commit status of the descriptor. Read-only parameter
  INTEGER, PARAMETER :: DFTI_COMMIT_STATUS = 22

  ! Version string for this DFTI implementation. Read-only parameter
  INTEGER, PARAMETER :: DFTI_VERSION = 23

  ! Ordering of the forward transform. Read-only parameter
  ! INTEGER, PARAMETER :: DFTI_FORWARD_ORDERING = 24 ! NOT IMPLEMENTED

  ! Ordering of the backward transform. Read-only parameter
  ! INTEGER, PARAMETER :: DFTI_BACKWARD_ORDERING = 25 ! NOT IMPLEMENTED

  ! Number of user threads that share the descriptor [1]
  INTEGER, PARAMETER :: DFTI_NUMBER_OF_USER_THREADS = 26

  ! Limit the number of threads used by this descriptor [0 = don't care]
  INTEGER, PARAMETER :: DFTI_THREAD_LIMIT = 27

  !======================================================================
  ! Values of the descriptor configuration parameters
  !======================================================================

  ! DFTI_COMMIT_STATUS
  INTEGER, PARAMETER :: DFTI_COMMITTED = 30
  INTEGER, PARAMETER :: DFTI_UNCOMMITTED = 31

  ! DFTI_FORWARD_DOMAIN
  INTEGER, PARAMETER :: DFTI_COMPLEX = 32
  INTEGER, PARAMETER :: DFTI_REAL = 33
  ! INTEGER, PARAMETER :: DFTI_CONJUGATE_EVEN = 34 ! NOT IMPLEMENTED

  ! DFTI_PRECISION
  INTEGER, PARAMETER :: DFTI_SINGLE = 35
  INTEGER, PARAMETER :: DFTI_DOUBLE = 36

  ! DFTI_PRECISION for reduced size of statically linked application.
  ! Recommended use: modify statement 'USE MKL_DFTI' in your program,
  ! so that it reads as either of:
  ! USE MKL_DFTI, FORGET=>DFTI_SINGLE, DFTI_SINGLE=>DFTI_SINGLE_R
  ! USE MKL_DFTI, FORGET=>DFTI_DOUBLE, DFTI_DOUBLE=>DFTI_DOUBLE_R
  ! where word 'FORGET' can be any name not used in the program.
  REAL(DFTI_SPKP), PARAMETER :: DFTI_SINGLE_R = REAL(35)
  REAL(DFTI_DPKP), PARAMETER :: DFTI_DOUBLE_R = REAL(36)

  ! DFTI_FORWARD_SIGN
  ! INTEGER, PARAMETER :: DFTI_NEGATIVE = 37 ! NOT IMPLEMENTED
  ! INTEGER, PARAMETER :: DFTI_POSITIVE = 38 ! NOT IMPLEMENTED

  ! DFTI_COMPLEX_STORAGE and DFTI_CONJUGATE_EVEN_STORAGE
  INTEGER, PARAMETER :: DFTI_COMPLEX_COMPLEX = 39
  INTEGER, PARAMETER :: DFTI_COMPLEX_REAL = 40

  ! DFTI_REAL_STORAGE
  INTEGER, PARAMETER :: DFTI_REAL_COMPLEX = 41
  INTEGER, PARAMETER :: DFTI_REAL_REAL = 42

  ! DFTI_PLACEMENT
  INTEGER, PARAMETER :: DFTI_INPLACE = 43 ! Result overwrites input
  INTEGER, PARAMETER :: DFTI_NOT_INPLACE  = 44 ! Have another place for result

  ! DFTI_INITIALIZATION_EFFORT
  ! INTEGER, PARAMETER :: DFTI_LOW = 45 ! NOT IMPLEMENTED
  ! INTEGER, PARAMETER :: DFTI_MEDIUM = 46 ! NOT IMPLEMENTED
  ! INTEGER, PARAMETER :: DFTI_HIGH = 47 ! NOT IMPLEMENTED

  ! DFTI_ORDERING
  INTEGER, PARAMETER :: DFTI_ORDERED = 48
  INTEGER, PARAMETER :: DFTI_BACKWARD_SCRAMBLED = 49
  ! INTEGER, PARAMETER :: DFTI_FORWARD_SCRAMBLED  = 50 ! NOT IMPLEMENTED

  ! Allow/avoid certain usages
  INTEGER, PARAMETER :: DFTI_ALLOW = 51 ! Allow transposition or workspace
  INTEGER, PARAMETER :: DFTI_AVOID = 52 ! Avoid auxiliary storage
  INTEGER, PARAMETER :: DFTI_NONE = 53

  ! DFTI_PACKED_FORMAT
  ! (for storing congugate-even finite sequence in real array)
  INTEGER, PARAMETER :: DFTI_CCS_FORMAT = 54  ! Complex conjugate-symmetric
  INTEGER, PARAMETER :: DFTI_PACK_FORMAT = 55 ! Pack format for real DFT
  INTEGER, PARAMETER :: DFTI_PERM_FORMAT = 56 ! Perm format for real DFT
  INTEGER, PARAMETER :: DFTI_CCE_FORMAT = 57  ! Complex conjugate-even

  !======================================================================
  ! Error classes
  !======================================================================
  INTEGER, PARAMETER :: DFTI_NO_ERROR = 0
  INTEGER, PARAMETER :: DFTI_MEMORY_ERROR = 1
  INTEGER, PARAMETER :: DFTI_INVALID_CONFIGURATION = 2
  INTEGER, PARAMETER :: DFTI_INCONSISTENT_CONFIGURATION = 3
  INTEGER, PARAMETER :: DFTI_MULTITHREADED_ERROR = 4
  INTEGER, PARAMETER :: DFTI_BAD_DESCRIPTOR = 5
  INTEGER, PARAMETER :: DFTI_UNIMPLEMENTED = 6
  INTEGER, PARAMETER :: DFTI_MKL_INTERNAL_ERROR = 7
  INTEGER, PARAMETER :: DFTI_NUMBER_OF_THREADS_ERROR = 8
  INTEGER, PARAMETER :: DFTI_1D_LENGTH_EXCEEDS_INT32 = 9
  INTEGER, PARAMETER :: DFTI_1D_MEMORY_EXCEEDS_INT32 = 9

  ! Maximum length of error string
  INTEGER, PARAMETER :: DFTI_MAX_MESSAGE_LENGTH = 80

  ! Maximum length of user-settable descriptor name
  INTEGER, PARAMETER :: DFTI_MAX_NAME_LENGTH = 10

  ! Maximum length of oneMKL version string
  INTEGER, PARAMETER :: DFTI_VERSION_LENGTH = 198

END MODULE MKL_DFT_TYPE

MODULE MKL_DFTI_OMP_OFFLOAD

  USE MKL_DFT_TYPE

  INTERFACE DftiCreateDescriptor

     ! overloading of DftiCreateDescriptor for 1D DFT
     FUNCTION dfti_create_descriptor_1d(desc, precision, domain, dim, length)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_create_descriptor_1d
       !DEC$ ATTRIBUTES REFERENCE :: dfti_create_descriptor_1d
       INTEGER dfti_create_descriptor_1d
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       INTEGER, INTENT(IN) :: precision
       INTEGER, INTENT(IN) :: domain
       INTEGER, INTENT(IN) :: dim, length
     END FUNCTION dfti_create_descriptor_1d

     ! overloading of DftiCreateDescriptor for nD DFT
     FUNCTION dfti_create_descriptor_highd(desc, precision, domain, dim,length)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_create_descriptor_highd
       !DEC$ ATTRIBUTES REFERENCE :: dfti_create_descriptor_highd
       INTEGER dfti_create_descriptor_highd
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       INTEGER, INTENT(IN) :: precision
       INTEGER, INTENT(IN) :: domain
       INTEGER, INTENT(IN) :: dim
       INTEGER, INTENT(IN), DIMENSION(*) :: length
     END FUNCTION dfti_create_descriptor_highd

     ! overloading of DftiCreateDescriptor for SP 1D DFT
     ! second parameter (precision) should be any REAL*4 value
     ! for dispatching during compile time
     FUNCTION dfti_create_descriptor_s_1d(desc, s, dom, one, dim)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_create_descriptor_s_1d
       !DEC$ ATTRIBUTES REFERENCE :: dfti_create_descriptor_s_1d
       INTEGER dfti_create_descriptor_s_1d
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), INTENT(IN) :: s
       INTEGER, INTENT(IN) :: dom
       INTEGER, INTENT(IN) :: one
       INTEGER, INTENT(IN) :: dim
     END FUNCTION dfti_create_descriptor_s_1d

     ! overloading of DftiCreateDescriptor for SP nD DFT
     ! second parameter (precision) should be any REAL*4 value
     ! for dispatching during compile time
     FUNCTION dfti_create_descriptor_s_md(desc, s, dom, many, dims)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_create_descriptor_s_md
       !DEC$ ATTRIBUTES REFERENCE :: dfti_create_descriptor_s_md
       INTEGER dfti_create_descriptor_s_md
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), INTENT(IN) :: s
       INTEGER, INTENT(IN) :: dom
       INTEGER, INTENT(IN) :: many
       INTEGER, INTENT(IN), DIMENSION(*) :: dims
     END FUNCTION dfti_create_descriptor_s_md

     ! overloading of DftiCreateDescriptor for DP 1D DFT
     ! second parameter (precision) should be any REAL*8 value
     ! for dispatching during compile time
     FUNCTION dfti_create_descriptor_d_1d(desc, d, dom, one, dim)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_create_descriptor_d_1d
       !DEC$ ATTRIBUTES REFERENCE :: dfti_create_descriptor_d_1d
       INTEGER dfti_create_descriptor_d_1d
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), INTENT(IN) :: d
       INTEGER, INTENT(IN) :: dom
       INTEGER, INTENT(IN) :: one
       INTEGER, INTENT(IN) :: dim
     END FUNCTION dfti_create_descriptor_d_1d

     ! overloading of DftiCreateDescriptor for DP nD DFT
     ! second parameter (precision) should be any REAL*8 value
     ! for dispatching during compile time
     FUNCTION dfti_create_descriptor_d_md(desc, d, dom, many, dims)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_create_descriptor_d_md
       !DEC$ ATTRIBUTES REFERENCE :: dfti_create_descriptor_d_md
       INTEGER dfti_create_descriptor_d_md
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), INTENT(IN) :: d
       INTEGER, INTENT(IN) :: dom
       INTEGER, INTENT(IN) :: many
       INTEGER, INTENT(IN), DIMENSION(*) :: dims
     END FUNCTION dfti_create_descriptor_d_md

  END INTERFACE

  INTERFACE DftiCopyDescriptor

     FUNCTION dfti_copy_descriptor_external(desc, new_desc)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_copy_descriptor_external
       !DEC$ ATTRIBUTES REFERENCE :: dfti_copy_descriptor_external
       INTEGER dfti_copy_descriptor_external
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       TYPE(DFTI_DESCRIPTOR), POINTER :: new_desc
     END FUNCTION dfti_copy_descriptor_external

  END INTERFACE

  INTERFACE DftiSetValue

     ! overloading of DftiSetValue for integer value
     FUNCTION dfti_set_value_intval(desc, OptName, IntVal)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_set_value_intval
       !DEC$ ATTRIBUTES REFERENCE :: dfti_set_value_intval
       INTEGER dfti_set_value_intval
       INTEGER, INTENT(IN) :: OptName
       INTEGER, INTENT(IN) :: IntVal
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
     END FUNCTION dfti_set_value_intval

     ! overloading of DftiSetValue for SP value
     FUNCTION dfti_set_value_sglval(desc, OptName, sglval)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_set_value_sglval
       !DEC$ ATTRIBUTES REFERENCE :: dfti_set_value_sglval
       INTEGER dfti_set_value_sglval
       INTEGER, INTENT(IN) :: OptName
       REAL(DFTI_SPKP), INTENT(IN) :: sglval
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
     END FUNCTION dfti_set_value_sglval

     ! overloading of DftiSetValue for DP value
     FUNCTION dfti_set_value_dblval(desc, OptName, DblVal)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_set_value_dblval
       !DEC$ ATTRIBUTES REFERENCE :: dfti_set_value_dblval
       INTEGER dfti_set_value_dblval
       INTEGER, INTENT(IN) :: OptName
       REAL(DFTI_DPKP), INTENT(IN) :: DblVal
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
     END FUNCTION dfti_set_value_dblval

     ! overloading of DftiSetValue for integer vector
     FUNCTION dfti_set_value_intvec(desc, OptName, IntVec)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_set_value_intvec
       !DEC$ ATTRIBUTES REFERENCE :: dfti_set_value_intvec
       INTEGER dfti_set_value_intvec
       INTEGER, INTENT(IN) :: OptName
       INTEGER, INTENT(IN), DIMENSION(*) :: IntVec
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
     END FUNCTION dfti_set_value_intvec

     ! overloading of DftiSetValue for char vector
     FUNCTION dfti_set_value_chars(desc, OptName, Chars)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_set_value_chars
       !DEC$ ATTRIBUTES REFERENCE :: dfti_set_value_chars
       INTEGER dfti_set_value_chars
       INTEGER, INTENT(IN) :: OptName
       CHARACTER(*), INTENT(IN) :: Chars
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
     END FUNCTION dfti_set_value_chars

  END INTERFACE

  INTERFACE DftiGetValue

     ! overloading of DftiGetValue for integer value
     FUNCTION dfti_get_value_intval(desc, OptName, IntVal)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_get_value_intval
       !DEC$ ATTRIBUTES REFERENCE :: dfti_get_value_intval
       INTEGER dfti_get_value_intval
       INTEGER, INTENT(IN) :: OptName
       INTEGER, INTENT(OUT) :: IntVal
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
     END FUNCTION dfti_get_value_intval

     ! overloading of DftiGetValue for SP value
     FUNCTION dfti_get_value_sglval(desc, OptName, sglval)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_get_value_sglval
       !DEC$ ATTRIBUTES REFERENCE :: dfti_get_value_sglval
       INTEGER dfti_get_value_sglval
       INTEGER, INTENT(IN) :: OptName
       REAL(DFTI_SPKP), INTENT(OUT) :: sglval
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
     END FUNCTION dfti_get_value_sglval

     ! overloading of DftiGetValue for DP value
     FUNCTION dfti_get_value_dblval(desc, OptName, DblVal)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_get_value_dblval
       !DEC$ ATTRIBUTES REFERENCE :: dfti_get_value_dblval
       INTEGER dfti_get_value_dblval
       INTEGER, INTENT(IN) :: OptName
       REAL(DFTI_DPKP), INTENT(OUT) :: DblVal
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
     END FUNCTION dfti_get_value_dblval

     ! overloading of DftiGetValue for integer vector
     FUNCTION dfti_get_value_intvec(desc, OptName, IntVec)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_get_value_intvec
       !DEC$ ATTRIBUTES REFERENCE :: dfti_get_value_intvec
       INTEGER dfti_get_value_intvec
       INTEGER, INTENT(IN) :: OptName
       INTEGER, INTENT(OUT), DIMENSION(*) :: IntVec
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
     END FUNCTION dfti_get_value_intvec

     ! overloading of DftiGetValue for char vector
     FUNCTION dfti_get_value_chars(desc, OptName, Chars)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_get_value_chars
       !DEC$ ATTRIBUTES REFERENCE :: dfti_get_value_chars
       INTEGER dfti_get_value_chars
       INTEGER, INTENT(IN) :: OptName
       CHARACTER(*), INTENT(OUT) :: Chars
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
     END FUNCTION dfti_get_value_chars

  END INTERFACE

  INTERFACE
     FUNCTION mkl_dfti_commit_descriptor_external_omp_offload(desc,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_commit_descriptor_external_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_commit_descriptor_external_omp_offload
  END INTERFACE

  INTERFACE DftiCommitDescriptor

     FUNCTION dfti_commit_descriptor_external(desc)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_commit_descriptor_external
       !DEC$ ATTRIBUTES REFERENCE :: dfti_commit_descriptor_external
       INTEGER dfti_commit_descriptor_external
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       !$omp declare variant( dfti_commit_descriptor_external:mkl_dfti_commit_descriptor_external_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args(interop(prefer_type("sycl","level_zero"),targetsync))
     END FUNCTION dfti_commit_descriptor_external

  END INTERFACE DftiCommitDescriptor

  INTERFACE

     ! overloading of DftiComputeForward for SP R2C DFT (inplace)
     FUNCTION mkl_dfti_compute_forward_s_omp_offload(desc,sSrcDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_forward_s_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), INTENT(INOUT), DIMENSION(..) :: sSrcDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_forward_s_omp_offload

     ! overloading of DftiComputeForward for SP C2C DFT (inplace)
     FUNCTION mkl_dfti_compute_forward_c_omp_offload(desc,cSrcDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_forward_c_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_SPKP), INTENT(INOUT), DIMENSION(..) :: cSrcDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_forward_c_omp_offload

     ! overloading of DftiComputeForward for SP C2C DFT (inplace, split complex)
     FUNCTION mkl_dfti_compute_forward_ss_omp_offload(desc,sSrcDstRe,sSrcDstIm,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_forward_ss_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), DIMENSION(..) :: sSrcDstRe
       REAL(DFTI_SPKP), DIMENSION(..) :: sSrcDstIm
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_forward_ss_omp_offload

     ! overloading of DftiComputeForward for SP R2C DFT (out-of-place)
     FUNCTION mkl_dfti_compute_forward_sc_omp_offload(desc,sSrc,cDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_forward_sc_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: sSrc
       COMPLEX(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: cDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_forward_sc_omp_offload

     ! overloading of DftiComputeForward for SP C2C DFT (out-of-place)
     FUNCTION mkl_dfti_compute_forward_cc_omp_offload(desc,cSrc,cDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_forward_cc_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: cSrc
       COMPLEX(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: cDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_forward_cc_omp_offload

     ! overloading of DftiComputeForward for SP C2C DFT (out-of-place, split
     ! complex)
     FUNCTION mkl_dfti_compute_forward_ssss_omp_offload(desc,sSrcRe,sSrcIm,sDstRe,sDstIm,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_forward_ssss_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: sSrcRe
       REAL(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: sSrcIm
       REAL(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: sDstRe
       REAL(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: sDstIm
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_forward_ssss_omp_offload

     ! overloading of DftiComputeForward for DP R2C DFT (inplace)
     FUNCTION mkl_dfti_compute_forward_d_omp_offload(desc,dSrcDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_forward_d_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), INTENT(INOUT), DIMENSION(..) :: dSrcDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_forward_d_omp_offload

     ! overloading of DftiComputeForward for DP C2C DFT (inplace)
     FUNCTION mkl_dfti_compute_forward_z_omp_offload(desc,zSrcDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_forward_z_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_DPKP), INTENT(INOUT), DIMENSION(..) :: zSrcDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_forward_z_omp_offload

     ! overloading of DftiComputeForward for DP C2C DFT (inplace, split complex)
     FUNCTION mkl_dfti_compute_forward_dd_omp_offload(desc,dSrcDstRe,dSrcDstIm,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_forward_dd_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), DIMENSION(..) :: dSrcDstRe
       REAL(DFTI_DPKP), DIMENSION(..) :: dSrcDstIm
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_forward_dd_omp_offload

     ! overloading of DftiComputeForward for DP R2C DFT (out-of-place)
     FUNCTION mkl_dfti_compute_forward_dz_omp_offload(desc,dSrc,zDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_forward_dz_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: dSrc
       COMPLEX(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: zDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_forward_dz_omp_offload

     ! overloading of DftiComputeForward for DP C2C DFT (out-of-place)
     FUNCTION mkl_dfti_compute_forward_zz_omp_offload(desc,zSrc,zDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_forward_zz_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: zSrc
       COMPLEX(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: zDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_forward_zz_omp_offload

     ! overloading of DftiComputeForward for DP C2C DFT (out-of-place, split
     ! complex)
     FUNCTION mkl_dfti_compute_forward_dddd_omp_offload(desc,dSrcRe,dSrcIm,dDstRe,dDstIm,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_forward_dddd_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: dSrcRe
       REAL(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: dSrcIm
       REAL(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: dDstRe
       REAL(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: dDstIm
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_forward_dddd_omp_offload

  END INTERFACE

  INTERFACE DftiComputeForward

     ! overloading of DftiComputeForward for SP R2C DFT (inplace)
     FUNCTION dfti_compute_forward_s_cpu(desc,sSrcDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_forward_s_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_forward_s_cpu
       INTEGER dfti_compute_forward_s_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), INTENT(INOUT), DIMENSION(..) :: sSrcDst
       !$omp  declare variant( dfti_compute_forward_s_cpu:mkl_dfti_compute_forward_s_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:sSrcDst ) 
     END FUNCTION dfti_compute_forward_s_cpu

     ! overloading of DftiComputeForward for SP C2C DFT (inplace)
     FUNCTION dfti_compute_forward_c_cpu(desc,cSrcDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_forward_c_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_forward_c_cpu
       INTEGER dfti_compute_forward_c_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_SPKP), INTENT(INOUT), DIMENSION(..) :: cSrcDst
       !$omp  declare variant( dfti_compute_forward_c_cpu:mkl_dfti_compute_forward_c_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:cSrcDst )
     END FUNCTION dfti_compute_forward_c_cpu

     ! overloading of DftiComputeForward for SP C2C DFT (inplace, split complex)
     FUNCTION dfti_compute_forward_ss_cpu(desc,sSrcDstRe,sSrcDstIm)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_forward_ss_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_forward_ss_cpu
       INTEGER dfti_compute_forward_ss_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), DIMENSION(..) :: sSrcDstRe
       REAL(DFTI_SPKP), DIMENSION(..) :: sSrcDstIm
       !$omp  declare variant(dfti_compute_forward_ss_cpu:mkl_dfti_compute_forward_ss_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:sSrcDstRe,sSrcDstIm )
     END FUNCTION dfti_compute_forward_ss_cpu

     ! overloading of DftiComputeForward for SP R2C DFT (out-of-place)
     FUNCTION dfti_compute_forward_sc_cpu(desc,sSrc,cDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_forward_sc_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_forward_sc_cpu
       INTEGER dfti_compute_forward_sc_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: sSrc
       COMPLEX(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: cDst
       !$omp  declare variant( dfti_compute_forward_sc_cpu:mkl_dfti_compute_forward_sc_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:sSrc,cDst )
     END FUNCTION dfti_compute_forward_sc_cpu

     ! overloading of DftiComputeForward for SP C2C DFT (out-of-place)
     FUNCTION dfti_compute_forward_cc_cpu(desc,cSrc,cDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_forward_cc_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_forward_cc_cpu
       INTEGER dfti_compute_forward_cc_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: cSrc
       COMPLEX(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: cDst
       !$omp  declare variant( dfti_compute_forward_cc_cpu:mkl_dfti_compute_forward_cc_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:cSrc,cDst )
     END FUNCTION dfti_compute_forward_cc_cpu

     ! overloading of DftiComputeForward for SP C2C DFT (out-of-place, split
     ! complex)
     FUNCTION dfti_compute_forward_ssss_cpu(desc,sSrcRe,sSrcIm,sDstRe,sDstIm)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_forward_ssss_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_forward_ssss_cpu
       INTEGER dfti_compute_forward_ssss_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: sSrcRe
       REAL(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: sSrcIm
       REAL(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: sDstRe
       REAL(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: sDstIm
       !$omp  declare variant( dfti_compute_forward_ssss_cpu:mkl_dfti_compute_forward_ssss_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:sSrcRe,sSrcIm,sDstRe,sDstIm )
     END FUNCTION dfti_compute_forward_ssss_cpu

     ! overloading of DftiComputeForward for DP R2C DFT (inplace)
     FUNCTION dfti_compute_forward_d_cpu(desc,dSrcDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_forward_d_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_forward_d_cpu
       INTEGER dfti_compute_forward_d_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), INTENT(INOUT), DIMENSION(..) :: dSrcDst
       !$omp  declare variant( dfti_compute_forward_d_cpu:mkl_dfti_compute_forward_d_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:dSrcDst )
     END FUNCTION dfti_compute_forward_d_cpu

     ! overloading of DftiComputeForward for DP C2C DFT (inplace)
     FUNCTION dfti_compute_forward_z_cpu(desc,zSrcDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_forward_z_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_forward_z_cpu
       INTEGER dfti_compute_forward_z_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_DPKP), INTENT(INOUT), DIMENSION(..) :: zSrcDst
       !$omp  declare variant( dfti_compute_forward_z_cpu:mkl_dfti_compute_forward_z_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:zSrcDst )
     END FUNCTION dfti_compute_forward_z_cpu

     ! overloading of DftiComputeForward for DP C2C DFT (inplace, split complex)
     FUNCTION dfti_compute_forward_dd_cpu(desc,dSrcDstRe,dSrcDstIm)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_forward_dd_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_forward_dd_cpu
       INTEGER dfti_compute_forward_dd_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), DIMENSION(..) :: dSrcDstRe
       REAL(DFTI_DPKP), DIMENSION(..) :: dSrcDstIm
       !$omp  declare variant( dfti_compute_forward_dd_cpu:mkl_dfti_compute_forward_dd_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:dSrcDstRe,dSrcDstIm )
     END FUNCTION dfti_compute_forward_dd_cpu

     ! overloading of DftiComputeForward for DP R2C DFT (out-of-place)
     FUNCTION dfti_compute_forward_dz_cpu(desc,dSrc,zDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_forward_dz_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_forward_dz_cpu
       INTEGER dfti_compute_forward_dz_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: dSrc
       COMPLEX(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: zDst
       !$omp  declare variant( dfti_compute_forward_dz_cpu:mkl_dfti_compute_forward_dz_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:dSrc,zDst )
     END FUNCTION dfti_compute_forward_dz_cpu

     ! overloading of DftiComputeForward for DP C2C DFT (out-of-place)
     FUNCTION dfti_compute_forward_zz_cpu(desc,zSrc,zDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_forward_zz_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_forward_zz_cpu
       INTEGER dfti_compute_forward_zz_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: zSrc
       COMPLEX(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: zDst
       !$omp  declare variant( dfti_compute_forward_zz_cpu:mkl_dfti_compute_forward_zz_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:zSrc,zDst )
     END FUNCTION dfti_compute_forward_zz_cpu

     ! overloading of DftiComputeForward for DP C2C DFT (out-of-place, split
     ! complex)
     FUNCTION dfti_compute_forward_dddd_cpu(desc,dSrcRe,dSrcIm,dDstRe,dDstIm)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_forward_dddd_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_forward_dddd_cpu
       INTEGER dfti_compute_forward_dddd_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: dSrcRe
       REAL(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: dSrcIm
       REAL(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: dDstRe
       REAL(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: dDstIm
       !$omp  declare variant( dfti_compute_forward_dddd_cpu:mkl_dfti_compute_forward_dddd_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:dSrcRe,dSrcIm,dDstRe,dDstIm )
     END FUNCTION dfti_compute_forward_dddd_cpu

  END INTERFACE DftiComputeForward

  INTERFACE


     ! overloading of DftiComputeBackward for SP C2R DFT (inplace)
     FUNCTION mkl_dfti_compute_backward_s_omp_offload(desc,sSrcDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_backward_s_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), INTENT(INOUT), DIMENSION(..) :: sSrcDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_backward_s_omp_offload

     ! overloading of DftiComputeBackward for SP C2C DFT (inplace)
     FUNCTION mkl_dfti_compute_backward_c_omp_offload(desc,cSrcDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_backward_c_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_SPKP), INTENT(INOUT), DIMENSION(..) :: cSrcDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_backward_c_omp_offload

     ! overloading of DftiComputeBackward for SP C2C DFT (inplace, split complex)
     FUNCTION mkl_dfti_compute_backward_ss_omp_offload(desc,sSrcDstRe,sSrcDstIm,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_backward_ss_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), DIMENSION(..) :: sSrcDstRe
       REAL(DFTI_SPKP), DIMENSION(..) :: sSrcDstIm
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_backward_ss_omp_offload

     ! overloading of DftiComputeBackward for SP C2R DFT (out-of-place)
     FUNCTION mkl_dfti_compute_backward_cs_omp_offload(desc,cSrc,sDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_backward_cs_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: cSrc
       REAL(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: sDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_backward_cs_omp_offload

     ! overloading of DftiComputeBackward for SP C2C DFT (out-of-place)
     FUNCTION mkl_dfti_compute_backward_cc_omp_offload(desc,cSrc,cDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_backward_cc_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: cSrc
       COMPLEX(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: cDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_backward_cc_omp_offload

     ! overloading of DftiComputeBackward for SP C2C DFT (out-of-place, split
     ! complex)
     FUNCTION mkl_dfti_compute_backward_ssss_omp_offload(desc,sSrcRe,sSrcIm,sDstRe,sDstIm,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_backward_ssss_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: sSrcRe
       REAL(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: sSrcIm
       REAL(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: sDstRe
       REAL(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: sDstIm
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_backward_ssss_omp_offload

     ! overloading of DftiComputeBackward for DP C2R DFT (inplace)
     FUNCTION mkl_dfti_compute_backward_d_omp_offload(desc,dSrcDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_backward_d_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), INTENT(INOUT), DIMENSION(..) :: dSrcDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_backward_d_omp_offload

     ! overloading of DftiComputeBackward for DP C2C DFT (inplace)
     FUNCTION mkl_dfti_compute_backward_z_omp_offload(desc,zSrcDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_backward_z_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_DPKP), INTENT(INOUT), DIMENSION(..) :: zSrcDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_backward_z_omp_offload

     ! overloading of DftiComputeBackward for DP C2C DFT (inplace, split complex)
     FUNCTION mkl_dfti_compute_backward_dd_omp_offload(desc,dSrcDstRe,dSrcDstIm,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_backward_dd_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), DIMENSION(..) :: dSrcDstRe
       REAL(DFTI_DPKP), DIMENSION(..) :: dSrcDstIm
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_backward_dd_omp_offload

     ! overloading of DftiComputeBackward for DP C2R DFT (out-of-place)
     FUNCTION mkl_dfti_compute_backward_zd_omp_offload(desc,zSrc,dDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_backward_zd_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: zSrc
       REAL(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: dDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_backward_zd_omp_offload

     ! overloading of DftiComputeBackward for DP C2C DFT (out-of-place)
     FUNCTION mkl_dfti_compute_backward_zz_omp_offload(desc,zSrc,zDst,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_backward_zz_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: zSrc
       COMPLEX(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: zDst
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_backward_zz_omp_offload

     ! overloading of DftiComputeBackward for DP C2C DFT (out-of-place, split
     ! complex)
     FUNCTION mkl_dfti_compute_backward_dddd_omp_offload(desc,dSrcRe,dSrcIm,dDstRe,dDstIm,interop) BIND(C)
       USE MKL_DFT_TYPE
       USE, INTRINSIC :: ISO_C_BINDING
       INTEGER mkl_dfti_compute_backward_dddd_omp_offload
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: dSrcRe
       REAL(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: dSrcIm
       REAL(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: dDstRe
       REAL(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: dDstIm
       TYPE(C_PTR), INTENT(IN), VALUE :: interop
     END FUNCTION mkl_dfti_compute_backward_dddd_omp_offload

  END INTERFACE

  INTERFACE DftiComputeBackward


     ! overloading of DftiComputeBackward for SP C2R DFT (inplace)
     FUNCTION dfti_compute_backward_s_cpu(desc,sSrcDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_backward_s_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_backward_s_cpu
       INTEGER dfti_compute_backward_s_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), INTENT(INOUT), DIMENSION(..) :: sSrcDst
       !$omp  declare variant( dfti_compute_backward_s_cpu:mkl_dfti_compute_backward_s_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:sSrcDst )
     END FUNCTION dfti_compute_backward_s_cpu

     ! overloading of DftiComputeBackward for SP C2C DFT (inplace)
     FUNCTION dfti_compute_backward_c_cpu(desc,cSrcDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_backward_c_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_backward_c_cpu
       INTEGER dfti_compute_backward_c_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_SPKP), INTENT(INOUT), DIMENSION(..) :: cSrcDst
       !$omp  declare variant( dfti_compute_backward_c_cpu:mkl_dfti_compute_backward_c_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:cSrcDst )
     END FUNCTION dfti_compute_backward_c_cpu

     ! overloading of DftiComputeBackward for SP C2C DFT (inplace, split complex)
     FUNCTION dfti_compute_backward_ss_cpu(desc,sSrcDstRe,sSrcDstIm)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_backward_ss_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_backward_ss_cpu
       INTEGER dfti_compute_backward_ss_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), DIMENSION(..) :: sSrcDstRe
       REAL(DFTI_SPKP), DIMENSION(..) :: sSrcDstIm
       !$omp  declare variant( dfti_compute_backward_ss_cpu:mkl_dfti_compute_backward_ss_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:sSrcDstRe,sSrcDstIm )
     END FUNCTION dfti_compute_backward_ss_cpu

     ! overloading of DftiComputeBackward for SP C2R DFT (out-of-place)
     FUNCTION dfti_compute_backward_cs_cpu(desc,cSrc,sDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_backward_cs_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_backward_cs_cpu
       INTEGER dfti_compute_backward_cs_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: cSrc
       REAL(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: sDst
       !$omp  declare variant( dfti_compute_backward_cs_cpu:mkl_dfti_compute_backward_cs_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:cSrc,sDst )
     END FUNCTION dfti_compute_backward_cs_cpu

     ! overloading of DftiComputeBackward for SP C2C DFT (out-of-place)
     FUNCTION dfti_compute_backward_cc_cpu(desc,cSrc,cDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_backward_cc_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_backward_cc_cpu
       INTEGER dfti_compute_backward_cc_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: cSrc
       COMPLEX(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: cDst
       !$omp  declare variant( dfti_compute_backward_cc_cpu:mkl_dfti_compute_backward_cc_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:cSrc,cDst )
     END FUNCTION dfti_compute_backward_cc_cpu

     ! overloading of DftiComputeBackward for SP C2C DFT (out-of-place, split
     ! complex)
     FUNCTION dfti_compute_backward_ssss_cpu(desc,sSrcRe,sSrcIm,sDstRe,sDstIm)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_backward_ssss_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_backward_ssss_cpu
       INTEGER dfti_compute_backward_ssss_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: sSrcRe
       REAL(DFTI_SPKP), INTENT(IN), DIMENSION(..) :: sSrcIm
       REAL(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: sDstRe
       REAL(DFTI_SPKP), INTENT(OUT), DIMENSION(..) :: sDstIm
       !$omp  declare variant( dfti_compute_backward_ssss_cpu:mkl_dfti_compute_backward_ssss_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:sSrcRe,sSrcIm,sDstRe,sDstIm )
     END FUNCTION dfti_compute_backward_ssss_cpu

     ! overloading of DftiComputeBackward for DP C2R DFT (inplace)
     FUNCTION dfti_compute_backward_d_cpu(desc,dSrcDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_backward_d_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_backward_d_cpu
       INTEGER dfti_compute_backward_d_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), INTENT(INOUT), DIMENSION(..) :: dSrcDst
       !$omp  declare variant( dfti_compute_backward_d_cpu:mkl_dfti_compute_backward_d_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:dSrcDst )
     END FUNCTION dfti_compute_backward_d_cpu

     ! overloading of DftiComputeBackward for DP C2C DFT (inplace)
     FUNCTION dfti_compute_backward_z_cpu(desc,zSrcDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_backward_z_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_backward_z_cpu
       INTEGER dfti_compute_backward_z_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_DPKP), INTENT(INOUT), DIMENSION(..) :: zSrcDst
       !$omp  declare variant( dfti_compute_backward_z_cpu:mkl_dfti_compute_backward_z_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:zSrcDst )
     END FUNCTION dfti_compute_backward_z_cpu

     ! overloading of DftiComputeBackward for DP C2C DFT (inplace, split complex)
     FUNCTION dfti_compute_backward_dd_cpu(desc,dSrcDstRe,dSrcDstIm)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_backward_dd_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_backward_dd_cpu
       INTEGER dfti_compute_backward_dd_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), DIMENSION(..) :: dSrcDstRe
       REAL(DFTI_DPKP), DIMENSION(..) :: dSrcDstIm
       !$omp  declare variant( dfti_compute_backward_dd_cpu:mkl_dfti_compute_backward_dd_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:dSrcDstRe,dSrcDstIm )
     END FUNCTION dfti_compute_backward_dd_cpu

     ! overloading of DftiComputeBackward for DP C2R DFT (out-of-place)
     FUNCTION dfti_compute_backward_zd_cpu(desc,zSrc,dDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_backward_zd_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_backward_zd_cpu
       INTEGER dfti_compute_backward_zd_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: zSrc
       REAL(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: dDst
       !$omp  declare variant( dfti_compute_backward_zd_cpu:mkl_dfti_compute_backward_zd_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:zSrc,dDst )
     END FUNCTION dfti_compute_backward_zd_cpu

     ! overloading of DftiComputeBackward for DP C2C DFT (out-of-place)
     FUNCTION dfti_compute_backward_zz_cpu(desc,zSrc,zDst)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_backward_zz_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_backward_zz_cpu
       INTEGER dfti_compute_backward_zz_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       COMPLEX(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: zSrc
       COMPLEX(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: zDst
       !$omp  declare variant( dfti_compute_backward_zz_cpu:mkl_dfti_compute_backward_zz_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:zSrc,zDst )
     END FUNCTION dfti_compute_backward_zz_cpu

     ! overloading of DftiComputeBackward for DP C2C DFT (out-of-place, split
     ! complex)
     FUNCTION dfti_compute_backward_dddd_cpu(desc,dSrcRe,dSrcIm,dDstRe,dDstIm)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_compute_backward_dddd_cpu
       !DEC$ ATTRIBUTES REFERENCE :: dfti_compute_backward_dddd_cpu
       INTEGER dfti_compute_backward_dddd_cpu
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
       REAL(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: dSrcRe
       REAL(DFTI_DPKP), INTENT(IN), DIMENSION(..) :: dSrcIm
       REAL(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: dDstRe
       REAL(DFTI_DPKP), INTENT(OUT), DIMENSION(..) :: dDstIm
       !$omp  declare variant( dfti_compute_backward_dddd_cpu:mkl_dfti_compute_backward_dddd_omp_offload ) match( construct={dispatch}, device={arch(gen)} ) append_args( interop(prefer_type("sycl","level_zero"),targetsync) ) adjust_args( need_device_addr:dSrcRe,dSrcIm,dDstRe,dDstIm )
     END FUNCTION dfti_compute_backward_dddd_cpu

  END INTERFACE DftiComputeBackward

  INTERFACE DftiFreeDescriptor

     FUNCTION dfti_free_descriptor_external(desc)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_free_descriptor_external
       !DEC$ ATTRIBUTES REFERENCE :: dfti_free_descriptor_external
       INTEGER dfti_free_descriptor_external
       TYPE(DFTI_DESCRIPTOR), POINTER :: desc
     END FUNCTION dfti_free_descriptor_external

  END INTERFACE

  INTERFACE DftiErrorClass

     FUNCTION dfti_error_class_external(Status, ErrorClass)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_error_class_external
       !DEC$ ATTRIBUTES REFERENCE :: dfti_error_class_external
       LOGICAL dfti_error_class_external
       INTEGER, INTENT(IN) :: Status
       INTEGER, INTENT(IN) :: ErrorClass
     END FUNCTION dfti_error_class_external

  END INTERFACE

  INTERFACE DftiErrorMessage

     FUNCTION dfti_error_message_external(Status)
       USE MKL_DFT_TYPE
       !DEC$ ATTRIBUTES C :: dfti_error_message_external
       !DEC$ ATTRIBUTES REFERENCE :: dfti_error_message_external
       CHARACTER(LEN=DFTI_MAX_MESSAGE_LENGTH) :: dfti_error_message_external
       INTEGER, INTENT(IN) :: Status
     END FUNCTION dfti_error_message_external

  END INTERFACE

END MODULE MKL_DFTI_OMP_OFFLOAD
