! -== file: mkl_vml_omp_offload.f90 ==-
!===============================================================================
! Copyright (C) 2006 Intel Corporation
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

!*******************************************************************************"
!  Content:"
!      Intel(R) oneAPI Math Kernel Library (oneMKL) FORTRAN interface for"
!      OpenMP offload for VM"
!*******************************************************************************"

    module onemkl_vml_omp_offload
        integer(kind=8) ::vml_la
        integer(kind=8) ::vml_ha
        integer(kind=8) ::vml_ep
        parameter (vml_la = INT( Z"00000001" ))
        parameter (vml_ha = INT( Z"00000002" ))
        parameter (vml_ep = INT( Z"00000003" ))


    include "mkl_vml_omp_variant.f90"

    interface
        integer(kind=4) function vmlsetmode(n)
        integer(kind=8), intent(in) :: n
!$omp declare variant ( vmlsetmode:mkl_vm_vmlsetmode_omp_offload_ilp64_ ) match( construct={dispatch}, device={arch(gen)}) append_args(interop(prefer_type("sycl","level_zero"),targetsync))
        end function

        integer(kind=4) function vmlgetmode()
!$omp declare variant ( vmlgetmode:mkl_vm_vmlgetmode_omp_offload_ilp64_ ) match( construct={dispatch}, device={arch(gen)}) append_args(interop(prefer_type("sycl","level_zero"),targetsync))
        end function

        integer(kind=4) function vmlseterrstatus(n)
        integer(kind=8), intent(in) :: n
!$omp declare variant ( vmlseterrstatus:mkl_vm_vmlseterrstatus_omp_offload_ilp64_ ) match( construct={dispatch}, device={arch(gen)}) append_args(interop(prefer_type("sycl","level_zero"),targetsync))
        end function

        integer(kind=4) function vmlgeterrstatus()
!$omp declare variant ( vmlgeterrstatus:mkl_vm_vmlgeterrstatus_omp_offload_ilp64_ ) match( construct={dispatch}, device={arch(gen)}) append_args(interop(prefer_type("sycl","level_zero"),targetsync))
        end function

        integer(kind=4) function vmlclearerrstatus()
!$omp declare variant ( vmlclearerrstatus:mkl_vm_vmlclearerrstatus_omp_offload_ilp64_ ) match(construct={dispatch}, device={arch(gen)}) append_args(interop(prefer_type("sycl","level_zero"),targetsync))
        end function





! function: Abs, indexing: linear
        subroutine vmsabs(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsabs:mkl_vm_vmsabs_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsabs

        subroutine vsabs(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsabs:mkl_vm_vsabs_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsabs

        subroutine vmdabs(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdabs:mkl_vm_vmdabs_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdabs

        subroutine vdabs(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdabs:mkl_vm_vdabs_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdabs

        subroutine vmcabs(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcabs:mkl_vm_vmcabs_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcabs

        subroutine vcabs(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcabs:mkl_vm_vcabs_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcabs

        subroutine vmzabs(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzabs:mkl_vm_vmzabs_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzabs

        subroutine vzabs(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzabs:mkl_vm_vzabs_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzabs


! function: Abs, indexing: strided
        subroutine vmsabsi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsabsi:mkl_vm_vmsabsi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsabsi

        subroutine vsabsi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsabsi:mkl_vm_vsabsi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsabsi

        subroutine vmdabsi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdabsi:mkl_vm_vmdabsi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdabsi

        subroutine vdabsi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdabsi:mkl_vm_vdabsi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdabsi

        subroutine vmcabsi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcabsi:mkl_vm_vmcabsi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcabsi

        subroutine vcabsi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcabsi:mkl_vm_vcabsi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcabsi

        subroutine vmzabsi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzabsi:mkl_vm_vmzabsi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzabsi

        subroutine vzabsi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzabsi:mkl_vm_vzabsi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzabsi




! function: Acos, indexing: linear
        subroutine vmsacos(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsacos:mkl_vm_vmsacos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsacos

        subroutine vsacos(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsacos:mkl_vm_vsacos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsacos

        subroutine vmdacos(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdacos:mkl_vm_vmdacos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdacos

        subroutine vdacos(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdacos:mkl_vm_vdacos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdacos

        subroutine vmcacos(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcacos:mkl_vm_vmcacos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcacos

        subroutine vcacos(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcacos:mkl_vm_vcacos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcacos

        subroutine vmzacos(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzacos:mkl_vm_vmzacos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzacos

        subroutine vzacos(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzacos:mkl_vm_vzacos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzacos


! function: Acos, indexing: strided
        subroutine vmsacosi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsacosi:mkl_vm_vmsacosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsacosi

        subroutine vsacosi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsacosi:mkl_vm_vsacosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsacosi

        subroutine vmdacosi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdacosi:mkl_vm_vmdacosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdacosi

        subroutine vdacosi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdacosi:mkl_vm_vdacosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdacosi

        subroutine vmcacosi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcacosi:mkl_vm_vmcacosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcacosi

        subroutine vcacosi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcacosi:mkl_vm_vcacosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcacosi

        subroutine vmzacosi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzacosi:mkl_vm_vmzacosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzacosi

        subroutine vzacosi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzacosi:mkl_vm_vzacosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzacosi




! function: Acosh, indexing: linear
        subroutine vmsacosh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsacosh:mkl_vm_vmsacosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsacosh

        subroutine vsacosh(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsacosh:mkl_vm_vsacosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsacosh

        subroutine vmdacosh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdacosh:mkl_vm_vmdacosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdacosh

        subroutine vdacosh(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdacosh:mkl_vm_vdacosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdacosh

        subroutine vmcacosh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcacosh:mkl_vm_vmcacosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcacosh

        subroutine vcacosh(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcacosh:mkl_vm_vcacosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcacosh

        subroutine vmzacosh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzacosh:mkl_vm_vmzacosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzacosh

        subroutine vzacosh(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzacosh:mkl_vm_vzacosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzacosh


! function: Acosh, indexing: strided
        subroutine vmsacoshi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsacoshi:mkl_vm_vmsacoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsacoshi

        subroutine vsacoshi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsacoshi:mkl_vm_vsacoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsacoshi

        subroutine vmdacoshi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdacoshi:mkl_vm_vmdacoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdacoshi

        subroutine vdacoshi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdacoshi:mkl_vm_vdacoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdacoshi

        subroutine vmcacoshi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcacoshi:mkl_vm_vmcacoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcacoshi

        subroutine vcacoshi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcacoshi:mkl_vm_vcacoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcacoshi

        subroutine vmzacoshi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzacoshi:mkl_vm_vmzacoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzacoshi

        subroutine vzacoshi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzacoshi:mkl_vm_vzacoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzacoshi




! function: Acospi, indexing: linear
        subroutine vmsacospi(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsacospi:mkl_vm_vmsacospi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsacospi

        subroutine vsacospi(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsacospi:mkl_vm_vsacospi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsacospi

        subroutine vmdacospi(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdacospi:mkl_vm_vmdacospi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdacospi

        subroutine vdacospi(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdacospi:mkl_vm_vdacospi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdacospi


! function: Acospi, indexing: strided
        subroutine vmsacospii(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsacospii:mkl_vm_vmsacospii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsacospii

        subroutine vsacospii(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsacospii:mkl_vm_vsacospii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsacospii

        subroutine vmdacospii(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdacospii:mkl_vm_vmdacospii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdacospii

        subroutine vdacospii(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdacospii:mkl_vm_vdacospii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdacospii




! function: Add, indexing: linear
        subroutine vmsadd(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsadd:mkl_vm_vmsadd_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsadd

        subroutine vsadd(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsadd:mkl_vm_vsadd_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsadd

        subroutine vmdadd(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdadd:mkl_vm_vmdadd_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdadd

        subroutine vdadd(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdadd:mkl_vm_vdadd_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdadd

        subroutine vmcadd(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcadd:mkl_vm_vmcadd_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmcadd

        subroutine vcadd(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcadd:mkl_vm_vcadd_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vcadd

        subroutine vmzadd(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzadd:mkl_vm_vmzadd_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmzadd

        subroutine vzadd(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzadd:mkl_vm_vzadd_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vzadd


! function: Add, indexing: strided
        subroutine vmsaddi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsaddi:mkl_vm_vmsaddi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsaddi

        subroutine vsaddi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsaddi:mkl_vm_vsaddi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsaddi

        subroutine vmdaddi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdaddi:mkl_vm_vmdaddi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdaddi

        subroutine vdaddi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdaddi:mkl_vm_vdaddi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdaddi

        subroutine vmcaddi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcaddi:mkl_vm_vmcaddi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmcaddi

        subroutine vcaddi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcaddi:mkl_vm_vcaddi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vcaddi

        subroutine vmzaddi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzaddi:mkl_vm_vmzaddi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmzaddi

        subroutine vzaddi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzaddi:mkl_vm_vzaddi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vzaddi




! function: Arg, indexing: linear
        subroutine vmcarg(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcarg:mkl_vm_vmcarg_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcarg

        subroutine vcarg(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcarg:mkl_vm_vcarg_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcarg

        subroutine vmzarg(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzarg:mkl_vm_vmzarg_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzarg

        subroutine vzarg(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzarg:mkl_vm_vzarg_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzarg


! function: Arg, indexing: strided
        subroutine vmcargi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcargi:mkl_vm_vmcargi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcargi

        subroutine vcargi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcargi:mkl_vm_vcargi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcargi

        subroutine vmzargi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzargi:mkl_vm_vmzargi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzargi

        subroutine vzargi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzargi:mkl_vm_vzargi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzargi




! function: Asin, indexing: linear
        subroutine vmsasin(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsasin:mkl_vm_vmsasin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsasin

        subroutine vsasin(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsasin:mkl_vm_vsasin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsasin

        subroutine vmdasin(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdasin:mkl_vm_vmdasin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdasin

        subroutine vdasin(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdasin:mkl_vm_vdasin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdasin

        subroutine vmcasin(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcasin:mkl_vm_vmcasin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcasin

        subroutine vcasin(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcasin:mkl_vm_vcasin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcasin

        subroutine vmzasin(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzasin:mkl_vm_vmzasin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzasin

        subroutine vzasin(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzasin:mkl_vm_vzasin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzasin


! function: Asin, indexing: strided
        subroutine vmsasini(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsasini:mkl_vm_vmsasini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsasini

        subroutine vsasini(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsasini:mkl_vm_vsasini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsasini

        subroutine vmdasini(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdasini:mkl_vm_vmdasini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdasini

        subroutine vdasini(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdasini:mkl_vm_vdasini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdasini

        subroutine vmcasini(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcasini:mkl_vm_vmcasini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcasini

        subroutine vcasini(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcasini:mkl_vm_vcasini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcasini

        subroutine vmzasini(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzasini:mkl_vm_vmzasini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzasini

        subroutine vzasini(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzasini:mkl_vm_vzasini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzasini




! function: Asinh, indexing: linear
        subroutine vmsasinh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsasinh:mkl_vm_vmsasinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsasinh

        subroutine vsasinh(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsasinh:mkl_vm_vsasinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsasinh

        subroutine vmdasinh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdasinh:mkl_vm_vmdasinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdasinh

        subroutine vdasinh(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdasinh:mkl_vm_vdasinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdasinh

        subroutine vmcasinh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcasinh:mkl_vm_vmcasinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcasinh

        subroutine vcasinh(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcasinh:mkl_vm_vcasinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcasinh

        subroutine vmzasinh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzasinh:mkl_vm_vmzasinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzasinh

        subroutine vzasinh(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzasinh:mkl_vm_vzasinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzasinh


! function: Asinh, indexing: strided
        subroutine vmsasinhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsasinhi:mkl_vm_vmsasinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsasinhi

        subroutine vsasinhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsasinhi:mkl_vm_vsasinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsasinhi

        subroutine vmdasinhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdasinhi:mkl_vm_vmdasinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdasinhi

        subroutine vdasinhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdasinhi:mkl_vm_vdasinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdasinhi

        subroutine vmcasinhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcasinhi:mkl_vm_vmcasinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcasinhi

        subroutine vcasinhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcasinhi:mkl_vm_vcasinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcasinhi

        subroutine vmzasinhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzasinhi:mkl_vm_vmzasinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzasinhi

        subroutine vzasinhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzasinhi:mkl_vm_vzasinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzasinhi




! function: Asinpi, indexing: linear
        subroutine vmsasinpi(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsasinpi:mkl_vm_vmsasinpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsasinpi

        subroutine vsasinpi(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsasinpi:mkl_vm_vsasinpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsasinpi

        subroutine vmdasinpi(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdasinpi:mkl_vm_vmdasinpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdasinpi

        subroutine vdasinpi(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdasinpi:mkl_vm_vdasinpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdasinpi


! function: Asinpi, indexing: strided
        subroutine vmsasinpii(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsasinpii:mkl_vm_vmsasinpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsasinpii

        subroutine vsasinpii(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsasinpii:mkl_vm_vsasinpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsasinpii

        subroutine vmdasinpii(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdasinpii:mkl_vm_vmdasinpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdasinpii

        subroutine vdasinpii(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdasinpii:mkl_vm_vdasinpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdasinpii




! function: Atan, indexing: linear
        subroutine vmsatan(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsatan:mkl_vm_vmsatan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsatan

        subroutine vsatan(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsatan:mkl_vm_vsatan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsatan

        subroutine vmdatan(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdatan:mkl_vm_vmdatan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdatan

        subroutine vdatan(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdatan:mkl_vm_vdatan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdatan

        subroutine vmcatan(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcatan:mkl_vm_vmcatan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcatan

        subroutine vcatan(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcatan:mkl_vm_vcatan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcatan

        subroutine vmzatan(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzatan:mkl_vm_vmzatan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzatan

        subroutine vzatan(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzatan:mkl_vm_vzatan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzatan


! function: Atan, indexing: strided
        subroutine vmsatani(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsatani:mkl_vm_vmsatani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsatani

        subroutine vsatani(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsatani:mkl_vm_vsatani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsatani

        subroutine vmdatani(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdatani:mkl_vm_vmdatani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdatani

        subroutine vdatani(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdatani:mkl_vm_vdatani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdatani

        subroutine vmcatani(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcatani:mkl_vm_vmcatani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcatani

        subroutine vcatani(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcatani:mkl_vm_vcatani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcatani

        subroutine vmzatani(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzatani:mkl_vm_vmzatani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzatani

        subroutine vzatani(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzatani:mkl_vm_vzatani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzatani




! function: Atan2, indexing: linear
        subroutine vmsatan2(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsatan2:mkl_vm_vmsatan2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsatan2

        subroutine vsatan2(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsatan2:mkl_vm_vsatan2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsatan2

        subroutine vmdatan2(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdatan2:mkl_vm_vmdatan2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdatan2

        subroutine vdatan2(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdatan2:mkl_vm_vdatan2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdatan2


! function: Atan2, indexing: strided
        subroutine vmsatan2i(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsatan2i:mkl_vm_vmsatan2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsatan2i

        subroutine vsatan2i(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsatan2i:mkl_vm_vsatan2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsatan2i

        subroutine vmdatan2i(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdatan2i:mkl_vm_vmdatan2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdatan2i

        subroutine vdatan2i(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdatan2i:mkl_vm_vdatan2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdatan2i




! function: Atan2pi, indexing: linear
        subroutine vmsatan2pi(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsatan2pi:mkl_vm_vmsatan2pi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsatan2pi

        subroutine vsatan2pi(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsatan2pi:mkl_vm_vsatan2pi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsatan2pi

        subroutine vmdatan2pi(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdatan2pi:mkl_vm_vmdatan2pi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdatan2pi

        subroutine vdatan2pi(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdatan2pi:mkl_vm_vdatan2pi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdatan2pi


! function: Atan2pi, indexing: strided
        subroutine vmsatan2pii(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsatan2pii:mkl_vm_vmsatan2pii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsatan2pii

        subroutine vsatan2pii(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsatan2pii:mkl_vm_vsatan2pii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsatan2pii

        subroutine vmdatan2pii(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdatan2pii:mkl_vm_vmdatan2pii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdatan2pii

        subroutine vdatan2pii(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdatan2pii:mkl_vm_vdatan2pii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdatan2pii




! function: Atanh, indexing: linear
        subroutine vmsatanh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsatanh:mkl_vm_vmsatanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsatanh

        subroutine vsatanh(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsatanh:mkl_vm_vsatanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsatanh

        subroutine vmdatanh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdatanh:mkl_vm_vmdatanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdatanh

        subroutine vdatanh(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdatanh:mkl_vm_vdatanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdatanh

        subroutine vmcatanh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcatanh:mkl_vm_vmcatanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcatanh

        subroutine vcatanh(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcatanh:mkl_vm_vcatanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcatanh

        subroutine vmzatanh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzatanh:mkl_vm_vmzatanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzatanh

        subroutine vzatanh(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzatanh:mkl_vm_vzatanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzatanh


! function: Atanh, indexing: strided
        subroutine vmsatanhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsatanhi:mkl_vm_vmsatanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsatanhi

        subroutine vsatanhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsatanhi:mkl_vm_vsatanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsatanhi

        subroutine vmdatanhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdatanhi:mkl_vm_vmdatanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdatanhi

        subroutine vdatanhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdatanhi:mkl_vm_vdatanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdatanhi

        subroutine vmcatanhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcatanhi:mkl_vm_vmcatanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcatanhi

        subroutine vcatanhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcatanhi:mkl_vm_vcatanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcatanhi

        subroutine vmzatanhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzatanhi:mkl_vm_vmzatanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzatanhi

        subroutine vzatanhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzatanhi:mkl_vm_vzatanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzatanhi




! function: Atanpi, indexing: linear
        subroutine vmsatanpi(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsatanpi:mkl_vm_vmsatanpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsatanpi

        subroutine vsatanpi(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsatanpi:mkl_vm_vsatanpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsatanpi

        subroutine vmdatanpi(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdatanpi:mkl_vm_vmdatanpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdatanpi

        subroutine vdatanpi(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdatanpi:mkl_vm_vdatanpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdatanpi


! function: Atanpi, indexing: strided
        subroutine vmsatanpii(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsatanpii:mkl_vm_vmsatanpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsatanpii

        subroutine vsatanpii(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsatanpii:mkl_vm_vsatanpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsatanpii

        subroutine vmdatanpii(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdatanpii:mkl_vm_vmdatanpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdatanpii

        subroutine vdatanpii(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdatanpii:mkl_vm_vdatanpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdatanpii




! function: Cbrt, indexing: linear
        subroutine vmscbrt(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscbrt:mkl_vm_vmscbrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscbrt

        subroutine vscbrt(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vscbrt:mkl_vm_vscbrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscbrt

        subroutine vmdcbrt(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcbrt:mkl_vm_vmdcbrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcbrt

        subroutine vdcbrt(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdcbrt:mkl_vm_vdcbrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcbrt


! function: Cbrt, indexing: strided
        subroutine vmscbrti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscbrti:mkl_vm_vmscbrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscbrti

        subroutine vscbrti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vscbrti:mkl_vm_vscbrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscbrti

        subroutine vmdcbrti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcbrti:mkl_vm_vmdcbrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcbrti

        subroutine vdcbrti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdcbrti:mkl_vm_vdcbrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcbrti




! function: CdfNorm, indexing: linear
        subroutine vmscdfnorm(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscdfnorm:mkl_vm_vmscdfnorm_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscdfnorm

        subroutine vscdfnorm(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vscdfnorm:mkl_vm_vscdfnorm_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscdfnorm

        subroutine vmdcdfnorm(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcdfnorm:mkl_vm_vmdcdfnorm_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcdfnorm

        subroutine vdcdfnorm(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdcdfnorm:mkl_vm_vdcdfnorm_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcdfnorm


! function: CdfNorm, indexing: strided
        subroutine vmscdfnormi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscdfnormi:mkl_vm_vmscdfnormi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscdfnormi

        subroutine vscdfnormi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vscdfnormi:mkl_vm_vscdfnormi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscdfnormi

        subroutine vmdcdfnormi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcdfnormi:mkl_vm_vmdcdfnormi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcdfnormi

        subroutine vdcdfnormi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdcdfnormi:mkl_vm_vdcdfnormi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcdfnormi




! function: CdfNormInv, indexing: linear
        subroutine vmscdfnorminv(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscdfnorminv:mkl_vm_vmscdfnorminv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscdfnorminv

        subroutine vscdfnorminv(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vscdfnorminv:mkl_vm_vscdfnorminv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscdfnorminv

        subroutine vmdcdfnorminv(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcdfnorminv:mkl_vm_vmdcdfnorminv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcdfnorminv

        subroutine vdcdfnorminv(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdcdfnorminv:mkl_vm_vdcdfnorminv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcdfnorminv


! function: CdfNormInv, indexing: strided
        subroutine vmscdfnorminvi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscdfnorminvi:mkl_vm_vmscdfnorminvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscdfnorminvi

        subroutine vscdfnorminvi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vscdfnorminvi:mkl_vm_vscdfnorminvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscdfnorminvi

        subroutine vmdcdfnorminvi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcdfnorminvi:mkl_vm_vmdcdfnorminvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcdfnorminvi

        subroutine vdcdfnorminvi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdcdfnorminvi:mkl_vm_vdcdfnorminvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcdfnorminvi




! function: Ceil, indexing: linear
        subroutine vmsceil(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsceil:mkl_vm_vmsceil_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsceil

        subroutine vsceil(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsceil:mkl_vm_vsceil_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsceil

        subroutine vmdceil(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdceil:mkl_vm_vmdceil_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdceil

        subroutine vdceil(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdceil:mkl_vm_vdceil_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdceil


! function: Ceil, indexing: strided
        subroutine vmsceili(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsceili:mkl_vm_vmsceili_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsceili

        subroutine vsceili(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsceili:mkl_vm_vsceili_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsceili

        subroutine vmdceili(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdceili:mkl_vm_vmdceili_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdceili

        subroutine vdceili(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdceili:mkl_vm_vdceili_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdceili




! function: CIS, indexing: linear
        subroutine vmccis(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmccis:mkl_vm_vmccis_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmccis

        subroutine vccis(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vccis:mkl_vm_vccis_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vccis

        subroutine vmzcis(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzcis:mkl_vm_vmzcis_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzcis

        subroutine vzcis(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzcis:mkl_vm_vzcis_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzcis


! function: CIS, indexing: strided
        subroutine vmccisi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmccisi:mkl_vm_vmccisi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmccisi

        subroutine vccisi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vccisi:mkl_vm_vccisi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vccisi

        subroutine vmzcisi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzcisi:mkl_vm_vmzcisi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzcisi

        subroutine vzcisi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzcisi:mkl_vm_vzcisi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzcisi




! function: Conj, indexing: linear
        subroutine vmcconj(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcconj:mkl_vm_vmcconj_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcconj

        subroutine vcconj(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcconj:mkl_vm_vcconj_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcconj

        subroutine vmzconj(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzconj:mkl_vm_vmzconj_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzconj

        subroutine vzconj(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzconj:mkl_vm_vzconj_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzconj


! function: Conj, indexing: strided
        subroutine vmcconji(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcconji:mkl_vm_vmcconji_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcconji

        subroutine vcconji(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcconji:mkl_vm_vcconji_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcconji

        subroutine vmzconji(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzconji:mkl_vm_vmzconji_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzconji

        subroutine vzconji(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzconji:mkl_vm_vzconji_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzconji




! function: CopySign, indexing: linear
        subroutine vmscopysign(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscopysign:mkl_vm_vmscopysign_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmscopysign

        subroutine vscopysign(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vscopysign:mkl_vm_vscopysign_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vscopysign

        subroutine vmdcopysign(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcopysign:mkl_vm_vmdcopysign_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdcopysign

        subroutine vdcopysign(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdcopysign:mkl_vm_vdcopysign_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdcopysign


! function: CopySign, indexing: strided
        subroutine vmscopysigni(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscopysigni:mkl_vm_vmscopysigni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmscopysigni

        subroutine vscopysigni(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vscopysigni:mkl_vm_vscopysigni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vscopysigni

        subroutine vmdcopysigni(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcopysigni:mkl_vm_vmdcopysigni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdcopysigni

        subroutine vdcopysigni(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdcopysigni:mkl_vm_vdcopysigni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdcopysigni




! function: Cos, indexing: linear
        subroutine vmscos(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscos:mkl_vm_vmscos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscos

        subroutine vscos(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vscos:mkl_vm_vscos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscos

        subroutine vmdcos(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcos:mkl_vm_vmdcos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcos

        subroutine vdcos(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdcos:mkl_vm_vdcos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcos

        subroutine vmccos(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmccos:mkl_vm_vmccos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmccos

        subroutine vccos(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vccos:mkl_vm_vccos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vccos

        subroutine vmzcos(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzcos:mkl_vm_vmzcos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzcos

        subroutine vzcos(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzcos:mkl_vm_vzcos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzcos


! function: Cos, indexing: strided
        subroutine vmscosi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscosi:mkl_vm_vmscosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscosi

        subroutine vscosi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vscosi:mkl_vm_vscosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscosi

        subroutine vmdcosi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcosi:mkl_vm_vmdcosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcosi

        subroutine vdcosi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdcosi:mkl_vm_vdcosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcosi

        subroutine vmccosi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmccosi:mkl_vm_vmccosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmccosi

        subroutine vccosi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vccosi:mkl_vm_vccosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vccosi

        subroutine vmzcosi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzcosi:mkl_vm_vmzcosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzcosi

        subroutine vzcosi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzcosi:mkl_vm_vzcosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzcosi




! function: Cosd, indexing: linear
        subroutine vmscosd(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscosd:mkl_vm_vmscosd_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscosd

        subroutine vscosd(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vscosd:mkl_vm_vscosd_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscosd

        subroutine vmdcosd(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcosd:mkl_vm_vmdcosd_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcosd

        subroutine vdcosd(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdcosd:mkl_vm_vdcosd_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcosd


! function: Cosd, indexing: strided
        subroutine vmscosdi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscosdi:mkl_vm_vmscosdi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscosdi

        subroutine vscosdi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vscosdi:mkl_vm_vscosdi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscosdi

        subroutine vmdcosdi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcosdi:mkl_vm_vmdcosdi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcosdi

        subroutine vdcosdi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdcosdi:mkl_vm_vdcosdi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcosdi




! function: Cosh, indexing: linear
        subroutine vmscosh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscosh:mkl_vm_vmscosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscosh

        subroutine vscosh(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vscosh:mkl_vm_vscosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscosh

        subroutine vmdcosh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcosh:mkl_vm_vmdcosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcosh

        subroutine vdcosh(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdcosh:mkl_vm_vdcosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcosh

        subroutine vmccosh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmccosh:mkl_vm_vmccosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmccosh

        subroutine vccosh(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vccosh:mkl_vm_vccosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vccosh

        subroutine vmzcosh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzcosh:mkl_vm_vmzcosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzcosh

        subroutine vzcosh(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzcosh:mkl_vm_vzcosh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzcosh


! function: Cosh, indexing: strided
        subroutine vmscoshi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscoshi:mkl_vm_vmscoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscoshi

        subroutine vscoshi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vscoshi:mkl_vm_vscoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscoshi

        subroutine vmdcoshi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcoshi:mkl_vm_vmdcoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcoshi

        subroutine vdcoshi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdcoshi:mkl_vm_vdcoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcoshi

        subroutine vmccoshi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmccoshi:mkl_vm_vmccoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmccoshi

        subroutine vccoshi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vccoshi:mkl_vm_vccoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vccoshi

        subroutine vmzcoshi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzcoshi:mkl_vm_vmzcoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzcoshi

        subroutine vzcoshi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzcoshi:mkl_vm_vzcoshi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzcoshi




! function: Cospi, indexing: linear
        subroutine vmscospi(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscospi:mkl_vm_vmscospi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscospi

        subroutine vscospi(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vscospi:mkl_vm_vscospi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscospi

        subroutine vmdcospi(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcospi:mkl_vm_vmdcospi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcospi

        subroutine vdcospi(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdcospi:mkl_vm_vdcospi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcospi


! function: Cospi, indexing: strided
        subroutine vmscospii(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmscospii:mkl_vm_vmscospii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmscospii

        subroutine vscospii(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vscospii:mkl_vm_vscospii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vscospii

        subroutine vmdcospii(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdcospii:mkl_vm_vmdcospii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdcospii

        subroutine vdcospii(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdcospii:mkl_vm_vdcospii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdcospii




! function: Div, indexing: linear
        subroutine vmsdiv(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsdiv:mkl_vm_vmsdiv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsdiv

        subroutine vsdiv(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsdiv:mkl_vm_vsdiv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsdiv

        subroutine vmddiv(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmddiv:mkl_vm_vmddiv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmddiv

        subroutine vddiv(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vddiv:mkl_vm_vddiv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vddiv

        subroutine vmcdiv(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcdiv:mkl_vm_vmcdiv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmcdiv

        subroutine vcdiv(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcdiv:mkl_vm_vcdiv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vcdiv

        subroutine vmzdiv(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzdiv:mkl_vm_vmzdiv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmzdiv

        subroutine vzdiv(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzdiv:mkl_vm_vzdiv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vzdiv


! function: Div, indexing: strided
        subroutine vmsdivi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsdivi:mkl_vm_vmsdivi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsdivi

        subroutine vsdivi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsdivi:mkl_vm_vsdivi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsdivi

        subroutine vmddivi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmddivi:mkl_vm_vmddivi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmddivi

        subroutine vddivi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vddivi:mkl_vm_vddivi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vddivi

        subroutine vmcdivi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcdivi:mkl_vm_vmcdivi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmcdivi

        subroutine vcdivi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcdivi:mkl_vm_vcdivi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vcdivi

        subroutine vmzdivi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzdivi:mkl_vm_vmzdivi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmzdivi

        subroutine vzdivi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzdivi:mkl_vm_vzdivi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vzdivi




! function: Erf, indexing: linear
        subroutine vmserf(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmserf:mkl_vm_vmserf_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmserf

        subroutine vserf(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vserf:mkl_vm_vserf_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vserf

        subroutine vmderf(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmderf:mkl_vm_vmderf_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmderf

        subroutine vderf(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vderf:mkl_vm_vderf_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vderf


! function: Erf, indexing: strided
        subroutine vmserfi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmserfi:mkl_vm_vmserfi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmserfi

        subroutine vserfi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vserfi:mkl_vm_vserfi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vserfi

        subroutine vmderfi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmderfi:mkl_vm_vmderfi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmderfi

        subroutine vderfi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vderfi:mkl_vm_vderfi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vderfi




! function: Erfc, indexing: linear
        subroutine vmserfc(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmserfc:mkl_vm_vmserfc_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmserfc

        subroutine vserfc(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vserfc:mkl_vm_vserfc_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vserfc

        subroutine vmderfc(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmderfc:mkl_vm_vmderfc_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmderfc

        subroutine vderfc(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vderfc:mkl_vm_vderfc_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vderfc


! function: Erfc, indexing: strided
        subroutine vmserfci(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmserfci:mkl_vm_vmserfci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmserfci

        subroutine vserfci(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vserfci:mkl_vm_vserfci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vserfci

        subroutine vmderfci(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmderfci:mkl_vm_vmderfci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmderfci

        subroutine vderfci(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vderfci:mkl_vm_vderfci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vderfci




! function: ErfcInv, indexing: linear
        subroutine vmserfcinv(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmserfcinv:mkl_vm_vmserfcinv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmserfcinv

        subroutine vserfcinv(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vserfcinv:mkl_vm_vserfcinv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vserfcinv

        subroutine vmderfcinv(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmderfcinv:mkl_vm_vmderfcinv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmderfcinv

        subroutine vderfcinv(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vderfcinv:mkl_vm_vderfcinv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vderfcinv


! function: ErfcInv, indexing: strided
        subroutine vmserfcinvi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmserfcinvi:mkl_vm_vmserfcinvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmserfcinvi

        subroutine vserfcinvi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vserfcinvi:mkl_vm_vserfcinvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vserfcinvi

        subroutine vmderfcinvi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmderfcinvi:mkl_vm_vmderfcinvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmderfcinvi

        subroutine vderfcinvi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vderfcinvi:mkl_vm_vderfcinvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vderfcinvi




! function: Erfcx, indexing: linear
        subroutine vmserfcx(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmserfcx:mkl_vm_vmserfcx_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmserfcx

        subroutine vserfcx(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vserfcx:mkl_vm_vserfcx_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vserfcx

        subroutine vmderfcx(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmderfcx:mkl_vm_vmderfcx_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmderfcx

        subroutine vderfcx(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vderfcx:mkl_vm_vderfcx_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vderfcx


! function: Erfcx, indexing: strided
        subroutine vmserfcxi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmserfcxi:mkl_vm_vmserfcxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmserfcxi

        subroutine vserfcxi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vserfcxi:mkl_vm_vserfcxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vserfcxi

        subroutine vmderfcxi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmderfcxi:mkl_vm_vmderfcxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmderfcxi

        subroutine vderfcxi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vderfcxi:mkl_vm_vderfcxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vderfcxi




! function: ErfInv, indexing: linear
        subroutine vmserfinv(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmserfinv:mkl_vm_vmserfinv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmserfinv

        subroutine vserfinv(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vserfinv:mkl_vm_vserfinv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vserfinv

        subroutine vmderfinv(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmderfinv:mkl_vm_vmderfinv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmderfinv

        subroutine vderfinv(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vderfinv:mkl_vm_vderfinv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vderfinv


! function: ErfInv, indexing: strided
        subroutine vmserfinvi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmserfinvi:mkl_vm_vmserfinvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmserfinvi

        subroutine vserfinvi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vserfinvi:mkl_vm_vserfinvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vserfinvi

        subroutine vmderfinvi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmderfinvi:mkl_vm_vmderfinvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmderfinvi

        subroutine vderfinvi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vderfinvi:mkl_vm_vderfinvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vderfinvi




! function: Exp, indexing: linear
        subroutine vmsexp(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsexp:mkl_vm_vmsexp_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsexp

        subroutine vsexp(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsexp:mkl_vm_vsexp_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsexp

        subroutine vmdexp(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdexp:mkl_vm_vmdexp_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdexp

        subroutine vdexp(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdexp:mkl_vm_vdexp_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdexp

        subroutine vmcexp(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcexp:mkl_vm_vmcexp_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcexp

        subroutine vcexp(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcexp:mkl_vm_vcexp_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcexp

        subroutine vmzexp(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzexp:mkl_vm_vmzexp_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzexp

        subroutine vzexp(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzexp:mkl_vm_vzexp_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzexp


! function: Exp, indexing: strided
        subroutine vmsexpi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsexpi:mkl_vm_vmsexpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsexpi

        subroutine vsexpi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsexpi:mkl_vm_vsexpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsexpi

        subroutine vmdexpi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdexpi:mkl_vm_vmdexpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdexpi

        subroutine vdexpi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdexpi:mkl_vm_vdexpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdexpi

        subroutine vmcexpi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcexpi:mkl_vm_vmcexpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcexpi

        subroutine vcexpi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcexpi:mkl_vm_vcexpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcexpi

        subroutine vmzexpi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzexpi:mkl_vm_vmzexpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzexpi

        subroutine vzexpi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzexpi:mkl_vm_vzexpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzexpi




! function: Exp10, indexing: linear
        subroutine vmsexp10(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsexp10:mkl_vm_vmsexp10_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsexp10

        subroutine vsexp10(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsexp10:mkl_vm_vsexp10_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsexp10

        subroutine vmdexp10(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdexp10:mkl_vm_vmdexp10_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdexp10

        subroutine vdexp10(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdexp10:mkl_vm_vdexp10_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdexp10


! function: Exp10, indexing: strided
        subroutine vmsexp10i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsexp10i:mkl_vm_vmsexp10i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsexp10i

        subroutine vsexp10i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsexp10i:mkl_vm_vsexp10i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsexp10i

        subroutine vmdexp10i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdexp10i:mkl_vm_vmdexp10i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdexp10i

        subroutine vdexp10i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdexp10i:mkl_vm_vdexp10i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdexp10i




! function: Exp2, indexing: linear
        subroutine vmsexp2(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsexp2:mkl_vm_vmsexp2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsexp2

        subroutine vsexp2(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsexp2:mkl_vm_vsexp2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsexp2

        subroutine vmdexp2(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdexp2:mkl_vm_vmdexp2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdexp2

        subroutine vdexp2(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdexp2:mkl_vm_vdexp2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdexp2


! function: Exp2, indexing: strided
        subroutine vmsexp2i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsexp2i:mkl_vm_vmsexp2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsexp2i

        subroutine vsexp2i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsexp2i:mkl_vm_vsexp2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsexp2i

        subroutine vmdexp2i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdexp2i:mkl_vm_vmdexp2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdexp2i

        subroutine vdexp2i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdexp2i:mkl_vm_vdexp2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdexp2i




! function: ExpInt1, indexing: linear
        subroutine vmsexpint1(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsexpint1:mkl_vm_vmsexpint1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsexpint1

        subroutine vsexpint1(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsexpint1:mkl_vm_vsexpint1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsexpint1

        subroutine vmdexpint1(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdexpint1:mkl_vm_vmdexpint1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdexpint1

        subroutine vdexpint1(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdexpint1:mkl_vm_vdexpint1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdexpint1


! function: ExpInt1, indexing: strided
        subroutine vmsexpint1i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsexpint1i:mkl_vm_vmsexpint1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsexpint1i

        subroutine vsexpint1i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsexpint1i:mkl_vm_vsexpint1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsexpint1i

        subroutine vmdexpint1i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdexpint1i:mkl_vm_vmdexpint1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdexpint1i

        subroutine vdexpint1i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdexpint1i:mkl_vm_vdexpint1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdexpint1i




! function: Expm1, indexing: linear
        subroutine vmsexpm1(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsexpm1:mkl_vm_vmsexpm1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsexpm1

        subroutine vsexpm1(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsexpm1:mkl_vm_vsexpm1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsexpm1

        subroutine vmdexpm1(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdexpm1:mkl_vm_vmdexpm1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdexpm1

        subroutine vdexpm1(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdexpm1:mkl_vm_vdexpm1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdexpm1


! function: Expm1, indexing: strided
        subroutine vmsexpm1i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsexpm1i:mkl_vm_vmsexpm1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsexpm1i

        subroutine vsexpm1i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsexpm1i:mkl_vm_vsexpm1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsexpm1i

        subroutine vmdexpm1i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdexpm1i:mkl_vm_vmdexpm1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdexpm1i

        subroutine vdexpm1i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdexpm1i:mkl_vm_vdexpm1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdexpm1i




! function: Fdim, indexing: linear
        subroutine vmsfdim(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsfdim:mkl_vm_vmsfdim_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsfdim

        subroutine vsfdim(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsfdim:mkl_vm_vsfdim_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsfdim

        subroutine vmdfdim(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdfdim:mkl_vm_vmdfdim_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdfdim

        subroutine vdfdim(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdfdim:mkl_vm_vdfdim_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdfdim


! function: Fdim, indexing: strided
        subroutine vmsfdimi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsfdimi:mkl_vm_vmsfdimi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsfdimi

        subroutine vsfdimi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsfdimi:mkl_vm_vsfdimi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsfdimi

        subroutine vmdfdimi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdfdimi:mkl_vm_vmdfdimi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdfdimi

        subroutine vdfdimi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdfdimi:mkl_vm_vdfdimi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdfdimi




! function: Floor, indexing: linear
        subroutine vmsfloor(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsfloor:mkl_vm_vmsfloor_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsfloor

        subroutine vsfloor(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsfloor:mkl_vm_vsfloor_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsfloor

        subroutine vmdfloor(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdfloor:mkl_vm_vmdfloor_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdfloor

        subroutine vdfloor(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdfloor:mkl_vm_vdfloor_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdfloor


! function: Floor, indexing: strided
        subroutine vmsfloori(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsfloori:mkl_vm_vmsfloori_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsfloori

        subroutine vsfloori(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsfloori:mkl_vm_vsfloori_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsfloori

        subroutine vmdfloori(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdfloori:mkl_vm_vmdfloori_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdfloori

        subroutine vdfloori(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdfloori:mkl_vm_vdfloori_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdfloori




! function: Fmax, indexing: linear
        subroutine vmsfmax(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsfmax:mkl_vm_vmsfmax_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsfmax

        subroutine vsfmax(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsfmax:mkl_vm_vsfmax_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsfmax

        subroutine vmdfmax(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdfmax:mkl_vm_vmdfmax_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdfmax

        subroutine vdfmax(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdfmax:mkl_vm_vdfmax_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdfmax


! function: Fmax, indexing: strided
        subroutine vmsfmaxi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsfmaxi:mkl_vm_vmsfmaxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsfmaxi

        subroutine vsfmaxi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsfmaxi:mkl_vm_vsfmaxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsfmaxi

        subroutine vmdfmaxi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdfmaxi:mkl_vm_vmdfmaxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdfmaxi

        subroutine vdfmaxi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdfmaxi:mkl_vm_vdfmaxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdfmaxi




! function: Fmin, indexing: linear
        subroutine vmsfmin(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsfmin:mkl_vm_vmsfmin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsfmin

        subroutine vsfmin(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsfmin:mkl_vm_vsfmin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsfmin

        subroutine vmdfmin(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdfmin:mkl_vm_vmdfmin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdfmin

        subroutine vdfmin(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdfmin:mkl_vm_vdfmin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdfmin


! function: Fmin, indexing: strided
        subroutine vmsfmini(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsfmini:mkl_vm_vmsfmini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsfmini

        subroutine vsfmini(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsfmini:mkl_vm_vsfmini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsfmini

        subroutine vmdfmini(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdfmini:mkl_vm_vmdfmini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdfmini

        subroutine vdfmini(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdfmini:mkl_vm_vdfmini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdfmini




! function: Fmod, indexing: linear
        subroutine vmsfmod(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsfmod:mkl_vm_vmsfmod_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsfmod

        subroutine vsfmod(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsfmod:mkl_vm_vsfmod_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsfmod

        subroutine vmdfmod(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdfmod:mkl_vm_vmdfmod_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdfmod

        subroutine vdfmod(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdfmod:mkl_vm_vdfmod_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdfmod


! function: Fmod, indexing: strided
        subroutine vmsfmodi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsfmodi:mkl_vm_vmsfmodi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsfmodi

        subroutine vsfmodi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsfmodi:mkl_vm_vsfmodi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsfmodi

        subroutine vmdfmodi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdfmodi:mkl_vm_vmdfmodi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdfmodi

        subroutine vdfmodi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdfmodi:mkl_vm_vdfmodi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdfmodi




! function: Frac, indexing: linear
        subroutine vmsfrac(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsfrac:mkl_vm_vmsfrac_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsfrac

        subroutine vsfrac(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsfrac:mkl_vm_vsfrac_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsfrac

        subroutine vmdfrac(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdfrac:mkl_vm_vmdfrac_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdfrac

        subroutine vdfrac(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdfrac:mkl_vm_vdfrac_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdfrac


! function: Frac, indexing: strided
        subroutine vmsfraci(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsfraci:mkl_vm_vmsfraci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsfraci

        subroutine vsfraci(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsfraci:mkl_vm_vsfraci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsfraci

        subroutine vmdfraci(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdfraci:mkl_vm_vmdfraci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdfraci

        subroutine vdfraci(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdfraci:mkl_vm_vdfraci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdfraci




! function: Hypot, indexing: linear
        subroutine vmshypot(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmshypot:mkl_vm_vmshypot_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmshypot

        subroutine vshypot(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vshypot:mkl_vm_vshypot_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vshypot

        subroutine vmdhypot(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdhypot:mkl_vm_vmdhypot_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdhypot

        subroutine vdhypot(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdhypot:mkl_vm_vdhypot_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdhypot


! function: Hypot, indexing: strided
        subroutine vmshypoti(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmshypoti:mkl_vm_vmshypoti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmshypoti

        subroutine vshypoti(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vshypoti:mkl_vm_vshypoti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vshypoti

        subroutine vmdhypoti(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdhypoti:mkl_vm_vmdhypoti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdhypoti

        subroutine vdhypoti(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdhypoti:mkl_vm_vdhypoti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdhypoti




! function: I0, indexing: linear
        subroutine vmsi0(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsi0:mkl_vm_vmsi0_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsi0

        subroutine vsi0(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsi0:mkl_vm_vsi0_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsi0

        subroutine vmdi0(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdi0:mkl_vm_vmdi0_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdi0

        subroutine vdi0(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdi0:mkl_vm_vdi0_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdi0


! function: I0, indexing: strided
        subroutine vmsi0i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsi0i:mkl_vm_vmsi0i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsi0i

        subroutine vsi0i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsi0i:mkl_vm_vsi0i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsi0i

        subroutine vmdi0i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdi0i:mkl_vm_vmdi0i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdi0i

        subroutine vdi0i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdi0i:mkl_vm_vdi0i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdi0i




! function: I1, indexing: linear
        subroutine vmsi1(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsi1:mkl_vm_vmsi1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsi1

        subroutine vsi1(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsi1:mkl_vm_vsi1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsi1

        subroutine vmdi1(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdi1:mkl_vm_vmdi1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdi1

        subroutine vdi1(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdi1:mkl_vm_vdi1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdi1


! function: I1, indexing: strided
        subroutine vmsi1i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsi1i:mkl_vm_vmsi1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsi1i

        subroutine vsi1i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsi1i:mkl_vm_vsi1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsi1i

        subroutine vmdi1i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdi1i:mkl_vm_vmdi1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdi1i

        subroutine vdi1i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdi1i:mkl_vm_vdi1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdi1i




! function: Inv, indexing: linear
        subroutine vmsinv(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsinv:mkl_vm_vmsinv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsinv

        subroutine vsinv(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsinv:mkl_vm_vsinv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsinv

        subroutine vmdinv(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdinv:mkl_vm_vmdinv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdinv

        subroutine vdinv(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdinv:mkl_vm_vdinv_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdinv


! function: Inv, indexing: strided
        subroutine vmsinvi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsinvi:mkl_vm_vmsinvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsinvi

        subroutine vsinvi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsinvi:mkl_vm_vsinvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsinvi

        subroutine vmdinvi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdinvi:mkl_vm_vmdinvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdinvi

        subroutine vdinvi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdinvi:mkl_vm_vdinvi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdinvi




! function: InvCbrt, indexing: linear
        subroutine vmsinvcbrt(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsinvcbrt:mkl_vm_vmsinvcbrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsinvcbrt

        subroutine vsinvcbrt(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsinvcbrt:mkl_vm_vsinvcbrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsinvcbrt

        subroutine vmdinvcbrt(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdinvcbrt:mkl_vm_vmdinvcbrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdinvcbrt

        subroutine vdinvcbrt(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdinvcbrt:mkl_vm_vdinvcbrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdinvcbrt


! function: InvCbrt, indexing: strided
        subroutine vmsinvcbrti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsinvcbrti:mkl_vm_vmsinvcbrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsinvcbrti

        subroutine vsinvcbrti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsinvcbrti:mkl_vm_vsinvcbrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsinvcbrti

        subroutine vmdinvcbrti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdinvcbrti:mkl_vm_vmdinvcbrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdinvcbrti

        subroutine vdinvcbrti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdinvcbrti:mkl_vm_vdinvcbrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdinvcbrti




! function: InvSqrt, indexing: linear
        subroutine vmsinvsqrt(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsinvsqrt:mkl_vm_vmsinvsqrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsinvsqrt

        subroutine vsinvsqrt(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsinvsqrt:mkl_vm_vsinvsqrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsinvsqrt

        subroutine vmdinvsqrt(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdinvsqrt:mkl_vm_vmdinvsqrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdinvsqrt

        subroutine vdinvsqrt(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdinvsqrt:mkl_vm_vdinvsqrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdinvsqrt


! function: InvSqrt, indexing: strided
        subroutine vmsinvsqrti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsinvsqrti:mkl_vm_vmsinvsqrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsinvsqrti

        subroutine vsinvsqrti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsinvsqrti:mkl_vm_vsinvsqrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsinvsqrti

        subroutine vmdinvsqrti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdinvsqrti:mkl_vm_vmdinvsqrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdinvsqrti

        subroutine vdinvsqrti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdinvsqrti:mkl_vm_vdinvsqrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdinvsqrti




! function: J0, indexing: linear
        subroutine vmsj0(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsj0:mkl_vm_vmsj0_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsj0

        subroutine vsj0(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsj0:mkl_vm_vsj0_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsj0

        subroutine vmdj0(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdj0:mkl_vm_vmdj0_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdj0

        subroutine vdj0(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdj0:mkl_vm_vdj0_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdj0


! function: J0, indexing: strided
        subroutine vmsj0i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsj0i:mkl_vm_vmsj0i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsj0i

        subroutine vsj0i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsj0i:mkl_vm_vsj0i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsj0i

        subroutine vmdj0i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdj0i:mkl_vm_vmdj0i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdj0i

        subroutine vdj0i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdj0i:mkl_vm_vdj0i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdj0i




! function: J1, indexing: linear
        subroutine vmsj1(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsj1:mkl_vm_vmsj1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsj1

        subroutine vsj1(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsj1:mkl_vm_vsj1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsj1

        subroutine vmdj1(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdj1:mkl_vm_vmdj1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdj1

        subroutine vdj1(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdj1:mkl_vm_vdj1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdj1


! function: J1, indexing: strided
        subroutine vmsj1i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsj1i:mkl_vm_vmsj1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsj1i

        subroutine vsj1i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsj1i:mkl_vm_vsj1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsj1i

        subroutine vmdj1i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdj1i:mkl_vm_vmdj1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdj1i

        subroutine vdj1i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdj1i:mkl_vm_vdj1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdj1i




! function: Jn, indexing: linear
        subroutine vmsjn(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsjn:mkl_vm_vmsjn_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsjn

        subroutine vsjn(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsjn:mkl_vm_vsjn_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsjn

        subroutine vmdjn(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdjn:mkl_vm_vmdjn_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdjn

        subroutine vdjn(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdjn:mkl_vm_vdjn_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdjn


! function: Jn, indexing: strided
        subroutine vmsjni(n, a, inca, b, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsjni:mkl_vm_vmsjni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsjni

        subroutine vsjni(n, a, inca, b, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsjni:mkl_vm_vsjni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsjni

        subroutine vmdjni(n, a, inca, b, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdjni:mkl_vm_vmdjni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdjni

        subroutine vdjni(n, a, inca, b, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdjni:mkl_vm_vdjni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdjni




! function: LGamma, indexing: linear
        subroutine vmslgamma(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmslgamma:mkl_vm_vmslgamma_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmslgamma

        subroutine vslgamma(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vslgamma:mkl_vm_vslgamma_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vslgamma

        subroutine vmdlgamma(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdlgamma:mkl_vm_vmdlgamma_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdlgamma

        subroutine vdlgamma(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdlgamma:mkl_vm_vdlgamma_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdlgamma


! function: LGamma, indexing: strided
        subroutine vmslgammai(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmslgammai:mkl_vm_vmslgammai_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmslgammai

        subroutine vslgammai(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vslgammai:mkl_vm_vslgammai_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vslgammai

        subroutine vmdlgammai(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdlgammai:mkl_vm_vmdlgammai_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdlgammai

        subroutine vdlgammai(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdlgammai:mkl_vm_vdlgammai_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdlgammai




! function: LinearFrac, indexing: linear
        subroutine vmslinearfrac(n, a, b, c, d, e, f, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(in) :: c
        real(kind=4), intent(in) :: d
        real(kind=4), intent(in) :: e
        real(kind=4), intent(in) :: f
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmslinearfrac:mkl_vm_vmslinearfrac_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmslinearfrac

        subroutine vslinearfrac(n, a, b, c, d, e, f, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(in) :: c
        real(kind=4), intent(in) :: d
        real(kind=4), intent(in) :: e
        real(kind=4), intent(in) :: f
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vslinearfrac:mkl_vm_vslinearfrac_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vslinearfrac

        subroutine vmdlinearfrac(n, a, b, c, d, e, f, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(in) :: c
        real(kind=8), intent(in) :: d
        real(kind=8), intent(in) :: e
        real(kind=8), intent(in) :: f
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdlinearfrac:mkl_vm_vmdlinearfrac_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdlinearfrac

        subroutine vdlinearfrac(n, a, b, c, d, e, f, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(in) :: c
        real(kind=8), intent(in) :: d
        real(kind=8), intent(in) :: e
        real(kind=8), intent(in) :: f
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdlinearfrac:mkl_vm_vdlinearfrac_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdlinearfrac


! function: LinearFrac, indexing: strided
        subroutine vmslinearfraci(n, a, inca, b, incb, c, d, e, f, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(in) :: c
        real(kind=4), intent(in) :: d
        real(kind=4), intent(in) :: e
        real(kind=4), intent(in) :: f
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmslinearfraci:mkl_vm_vmslinearfraci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmslinearfraci

        subroutine vslinearfraci(n, a, inca, b, incb, c, d, e, f, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(in) :: c
        real(kind=4), intent(in) :: d
        real(kind=4), intent(in) :: e
        real(kind=4), intent(in) :: f
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vslinearfraci:mkl_vm_vslinearfraci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vslinearfraci

        subroutine vmdlinearfraci(n, a, inca, b, incb, c, d, e, f, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(in) :: c
        real(kind=8), intent(in) :: d
        real(kind=8), intent(in) :: e
        real(kind=8), intent(in) :: f
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdlinearfraci:mkl_vm_vmdlinearfraci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdlinearfraci

        subroutine vdlinearfraci(n, a, inca, b, incb, c, d, e, f, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(in) :: c
        real(kind=8), intent(in) :: d
        real(kind=8), intent(in) :: e
        real(kind=8), intent(in) :: f
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdlinearfraci:mkl_vm_vdlinearfraci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdlinearfraci




! function: Ln, indexing: linear
        subroutine vmsln(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsln:mkl_vm_vmsln_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsln

        subroutine vsln(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsln:mkl_vm_vsln_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsln

        subroutine vmdln(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdln:mkl_vm_vmdln_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdln

        subroutine vdln(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdln:mkl_vm_vdln_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdln

        subroutine vmcln(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcln:mkl_vm_vmcln_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcln

        subroutine vcln(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcln:mkl_vm_vcln_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcln

        subroutine vmzln(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzln:mkl_vm_vmzln_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzln

        subroutine vzln(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzln:mkl_vm_vzln_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzln


! function: Ln, indexing: strided
        subroutine vmslni(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmslni:mkl_vm_vmslni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmslni

        subroutine vslni(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vslni:mkl_vm_vslni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vslni

        subroutine vmdlni(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdlni:mkl_vm_vmdlni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdlni

        subroutine vdlni(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdlni:mkl_vm_vdlni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdlni

        subroutine vmclni(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmclni:mkl_vm_vmclni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmclni

        subroutine vclni(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vclni:mkl_vm_vclni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vclni

        subroutine vmzlni(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzlni:mkl_vm_vmzlni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzlni

        subroutine vzlni(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzlni:mkl_vm_vzlni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzlni




! function: Log10, indexing: linear
        subroutine vmslog10(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmslog10:mkl_vm_vmslog10_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmslog10

        subroutine vslog10(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vslog10:mkl_vm_vslog10_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vslog10

        subroutine vmdlog10(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdlog10:mkl_vm_vmdlog10_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdlog10

        subroutine vdlog10(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdlog10:mkl_vm_vdlog10_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdlog10

        subroutine vmclog10(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmclog10:mkl_vm_vmclog10_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmclog10

        subroutine vclog10(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vclog10:mkl_vm_vclog10_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vclog10

        subroutine vmzlog10(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzlog10:mkl_vm_vmzlog10_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzlog10

        subroutine vzlog10(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzlog10:mkl_vm_vzlog10_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzlog10


! function: Log10, indexing: strided
        subroutine vmslog10i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmslog10i:mkl_vm_vmslog10i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmslog10i

        subroutine vslog10i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vslog10i:mkl_vm_vslog10i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vslog10i

        subroutine vmdlog10i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdlog10i:mkl_vm_vmdlog10i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdlog10i

        subroutine vdlog10i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdlog10i:mkl_vm_vdlog10i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdlog10i

        subroutine vmclog10i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmclog10i:mkl_vm_vmclog10i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmclog10i

        subroutine vclog10i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vclog10i:mkl_vm_vclog10i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vclog10i

        subroutine vmzlog10i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzlog10i:mkl_vm_vmzlog10i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzlog10i

        subroutine vzlog10i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzlog10i:mkl_vm_vzlog10i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzlog10i




! function: Log1p, indexing: linear
        subroutine vmslog1p(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmslog1p:mkl_vm_vmslog1p_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmslog1p

        subroutine vslog1p(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vslog1p:mkl_vm_vslog1p_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vslog1p

        subroutine vmdlog1p(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdlog1p:mkl_vm_vmdlog1p_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdlog1p

        subroutine vdlog1p(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdlog1p:mkl_vm_vdlog1p_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdlog1p


! function: Log1p, indexing: strided
        subroutine vmslog1pi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmslog1pi:mkl_vm_vmslog1pi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmslog1pi

        subroutine vslog1pi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vslog1pi:mkl_vm_vslog1pi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vslog1pi

        subroutine vmdlog1pi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdlog1pi:mkl_vm_vmdlog1pi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdlog1pi

        subroutine vdlog1pi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdlog1pi:mkl_vm_vdlog1pi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdlog1pi




! function: Log2, indexing: linear
        subroutine vmslog2(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmslog2:mkl_vm_vmslog2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmslog2

        subroutine vslog2(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vslog2:mkl_vm_vslog2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vslog2

        subroutine vmdlog2(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdlog2:mkl_vm_vmdlog2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdlog2

        subroutine vdlog2(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdlog2:mkl_vm_vdlog2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdlog2


! function: Log2, indexing: strided
        subroutine vmslog2i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmslog2i:mkl_vm_vmslog2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmslog2i

        subroutine vslog2i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vslog2i:mkl_vm_vslog2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vslog2i

        subroutine vmdlog2i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdlog2i:mkl_vm_vmdlog2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdlog2i

        subroutine vdlog2i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdlog2i:mkl_vm_vdlog2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdlog2i




! function: Logb, indexing: linear
        subroutine vmslogb(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmslogb:mkl_vm_vmslogb_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmslogb

        subroutine vslogb(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vslogb:mkl_vm_vslogb_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vslogb

        subroutine vmdlogb(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdlogb:mkl_vm_vmdlogb_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdlogb

        subroutine vdlogb(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdlogb:mkl_vm_vdlogb_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdlogb


! function: Logb, indexing: strided
        subroutine vmslogbi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmslogbi:mkl_vm_vmslogbi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmslogbi

        subroutine vslogbi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vslogbi:mkl_vm_vslogbi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vslogbi

        subroutine vmdlogbi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdlogbi:mkl_vm_vmdlogbi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdlogbi

        subroutine vdlogbi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdlogbi:mkl_vm_vdlogbi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdlogbi




! function: MaxMag, indexing: linear
        subroutine vmsmaxmag(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsmaxmag:mkl_vm_vmsmaxmag_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsmaxmag

        subroutine vsmaxmag(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsmaxmag:mkl_vm_vsmaxmag_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsmaxmag

        subroutine vmdmaxmag(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdmaxmag:mkl_vm_vmdmaxmag_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdmaxmag

        subroutine vdmaxmag(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdmaxmag:mkl_vm_vdmaxmag_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdmaxmag


! function: MaxMag, indexing: strided
        subroutine vmsmaxmagi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsmaxmagi:mkl_vm_vmsmaxmagi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsmaxmagi

        subroutine vsmaxmagi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsmaxmagi:mkl_vm_vsmaxmagi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsmaxmagi

        subroutine vmdmaxmagi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdmaxmagi:mkl_vm_vmdmaxmagi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdmaxmagi

        subroutine vdmaxmagi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdmaxmagi:mkl_vm_vdmaxmagi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdmaxmagi




! function: MinMag, indexing: linear
        subroutine vmsminmag(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsminmag:mkl_vm_vmsminmag_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsminmag

        subroutine vsminmag(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsminmag:mkl_vm_vsminmag_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsminmag

        subroutine vmdminmag(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdminmag:mkl_vm_vmdminmag_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdminmag

        subroutine vdminmag(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdminmag:mkl_vm_vdminmag_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdminmag


! function: MinMag, indexing: strided
        subroutine vmsminmagi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsminmagi:mkl_vm_vmsminmagi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsminmagi

        subroutine vsminmagi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsminmagi:mkl_vm_vsminmagi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsminmagi

        subroutine vmdminmagi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdminmagi:mkl_vm_vmdminmagi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdminmagi

        subroutine vdminmagi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdminmagi:mkl_vm_vdminmagi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdminmagi




! function: Modf, indexing: linear
        subroutine vmsmodf(n, a, y, z, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        real(kind=4), intent(inout) :: z(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsmodf:mkl_vm_vmsmodf_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vmsmodf

        subroutine vsmodf(n, a, y, z)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        real(kind=4), intent(inout) :: z(*)
!$omp declare variant (vsmodf:mkl_vm_vsmodf_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vsmodf

        subroutine vmdmodf(n, a, y, z, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        real(kind=8), intent(inout) :: z(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdmodf:mkl_vm_vmdmodf_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vmdmodf

        subroutine vdmodf(n, a, y, z)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        real(kind=8), intent(inout) :: z(*)
!$omp declare variant (vdmodf:mkl_vm_vdmodf_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vdmodf


! function: Modf, indexing: strided
        subroutine vmsmodfi(n, a, inca, y, incy, z, incz, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        real(kind=4), intent(inout) :: z(*)
        integer(kind=8), intent(in) :: incz
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsmodfi:mkl_vm_vmsmodfi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vmsmodfi

        subroutine vsmodfi(n, a, inca, y, incy, z, incz)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        real(kind=4), intent(inout) :: z(*)
        integer(kind=8), intent(in) :: incz
!$omp declare variant (vsmodfi:mkl_vm_vsmodfi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vsmodfi

        subroutine vmdmodfi(n, a, inca, y, incy, z, incz, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        real(kind=8), intent(inout) :: z(*)
        integer(kind=8), intent(in) :: incz
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdmodfi:mkl_vm_vmdmodfi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vmdmodfi

        subroutine vdmodfi(n, a, inca, y, incy, z, incz)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        real(kind=8), intent(inout) :: z(*)
        integer(kind=8), intent(in) :: incz
!$omp declare variant (vdmodfi:mkl_vm_vdmodfi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vdmodfi




! function: Mul, indexing: linear
        subroutine vmsmul(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsmul:mkl_vm_vmsmul_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsmul

        subroutine vsmul(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsmul:mkl_vm_vsmul_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsmul

        subroutine vmdmul(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdmul:mkl_vm_vmdmul_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdmul

        subroutine vdmul(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdmul:mkl_vm_vdmul_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdmul

        subroutine vmcmul(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcmul:mkl_vm_vmcmul_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmcmul

        subroutine vcmul(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcmul:mkl_vm_vcmul_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vcmul

        subroutine vmzmul(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzmul:mkl_vm_vmzmul_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmzmul

        subroutine vzmul(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzmul:mkl_vm_vzmul_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vzmul


! function: Mul, indexing: strided
        subroutine vmsmuli(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsmuli:mkl_vm_vmsmuli_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsmuli

        subroutine vsmuli(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsmuli:mkl_vm_vsmuli_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsmuli

        subroutine vmdmuli(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdmuli:mkl_vm_vmdmuli_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdmuli

        subroutine vdmuli(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdmuli:mkl_vm_vdmuli_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdmuli

        subroutine vmcmuli(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcmuli:mkl_vm_vmcmuli_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmcmuli

        subroutine vcmuli(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcmuli:mkl_vm_vcmuli_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vcmuli

        subroutine vmzmuli(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzmuli:mkl_vm_vmzmuli_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmzmuli

        subroutine vzmuli(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzmuli:mkl_vm_vzmuli_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vzmuli




! function: MulByConj, indexing: linear
        subroutine vmcmulbyconj(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcmulbyconj:mkl_vm_vmcmulbyconj_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmcmulbyconj

        subroutine vcmulbyconj(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcmulbyconj:mkl_vm_vcmulbyconj_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vcmulbyconj

        subroutine vmzmulbyconj(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzmulbyconj:mkl_vm_vmzmulbyconj_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmzmulbyconj

        subroutine vzmulbyconj(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzmulbyconj:mkl_vm_vzmulbyconj_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vzmulbyconj


! function: MulByConj, indexing: strided
        subroutine vmcmulbyconji(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcmulbyconji:mkl_vm_vmcmulbyconji_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmcmulbyconji

        subroutine vcmulbyconji(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcmulbyconji:mkl_vm_vcmulbyconji_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vcmulbyconji

        subroutine vmzmulbyconji(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzmulbyconji:mkl_vm_vmzmulbyconji_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmzmulbyconji

        subroutine vzmulbyconji(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzmulbyconji:mkl_vm_vzmulbyconji_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vzmulbyconji




! function: NearbyInt, indexing: linear
        subroutine vmsnearbyint(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsnearbyint:mkl_vm_vmsnearbyint_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsnearbyint

        subroutine vsnearbyint(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsnearbyint:mkl_vm_vsnearbyint_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsnearbyint

        subroutine vmdnearbyint(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdnearbyint:mkl_vm_vmdnearbyint_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdnearbyint

        subroutine vdnearbyint(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdnearbyint:mkl_vm_vdnearbyint_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdnearbyint


! function: NearbyInt, indexing: strided
        subroutine vmsnearbyinti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsnearbyinti:mkl_vm_vmsnearbyinti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsnearbyinti

        subroutine vsnearbyinti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsnearbyinti:mkl_vm_vsnearbyinti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsnearbyinti

        subroutine vmdnearbyinti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdnearbyinti:mkl_vm_vmdnearbyinti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdnearbyinti

        subroutine vdnearbyinti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdnearbyinti:mkl_vm_vdnearbyinti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdnearbyinti




! function: NextAfter, indexing: linear
        subroutine vmsnextafter(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsnextafter:mkl_vm_vmsnextafter_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsnextafter

        subroutine vsnextafter(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsnextafter:mkl_vm_vsnextafter_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsnextafter

        subroutine vmdnextafter(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdnextafter:mkl_vm_vmdnextafter_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdnextafter

        subroutine vdnextafter(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdnextafter:mkl_vm_vdnextafter_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdnextafter


! function: NextAfter, indexing: strided
        subroutine vmsnextafteri(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsnextafteri:mkl_vm_vmsnextafteri_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsnextafteri

        subroutine vsnextafteri(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsnextafteri:mkl_vm_vsnextafteri_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsnextafteri

        subroutine vmdnextafteri(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdnextafteri:mkl_vm_vmdnextafteri_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdnextafteri

        subroutine vdnextafteri(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdnextafteri:mkl_vm_vdnextafteri_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdnextafteri




! function: Pow, indexing: linear
        subroutine vmspow(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmspow:mkl_vm_vmspow_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmspow

        subroutine vspow(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vspow:mkl_vm_vspow_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vspow

        subroutine vmdpow(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdpow:mkl_vm_vmdpow_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdpow

        subroutine vdpow(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdpow:mkl_vm_vdpow_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdpow

        subroutine vmcpow(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcpow:mkl_vm_vmcpow_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmcpow

        subroutine vcpow(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcpow:mkl_vm_vcpow_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vcpow

        subroutine vmzpow(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzpow:mkl_vm_vmzpow_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmzpow

        subroutine vzpow(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzpow:mkl_vm_vzpow_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vzpow


! function: Pow, indexing: strided
        subroutine vmspowi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmspowi:mkl_vm_vmspowi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmspowi

        subroutine vspowi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vspowi:mkl_vm_vspowi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vspowi

        subroutine vmdpowi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdpowi:mkl_vm_vmdpowi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdpowi

        subroutine vdpowi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdpowi:mkl_vm_vdpowi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdpowi

        subroutine vmcpowi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcpowi:mkl_vm_vmcpowi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmcpowi

        subroutine vcpowi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcpowi:mkl_vm_vcpowi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vcpowi

        subroutine vmzpowi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzpowi:mkl_vm_vmzpowi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmzpowi

        subroutine vzpowi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzpowi:mkl_vm_vzpowi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vzpowi




! function: Pow2o3, indexing: linear
        subroutine vmspow2o3(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmspow2o3:mkl_vm_vmspow2o3_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmspow2o3

        subroutine vspow2o3(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vspow2o3:mkl_vm_vspow2o3_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vspow2o3

        subroutine vmdpow2o3(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdpow2o3:mkl_vm_vmdpow2o3_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdpow2o3

        subroutine vdpow2o3(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdpow2o3:mkl_vm_vdpow2o3_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdpow2o3


! function: Pow2o3, indexing: strided
        subroutine vmspow2o3i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmspow2o3i:mkl_vm_vmspow2o3i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmspow2o3i

        subroutine vspow2o3i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vspow2o3i:mkl_vm_vspow2o3i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vspow2o3i

        subroutine vmdpow2o3i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdpow2o3i:mkl_vm_vmdpow2o3i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdpow2o3i

        subroutine vdpow2o3i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdpow2o3i:mkl_vm_vdpow2o3i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdpow2o3i




! function: Pow3o2, indexing: linear
        subroutine vmspow3o2(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmspow3o2:mkl_vm_vmspow3o2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmspow3o2

        subroutine vspow3o2(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vspow3o2:mkl_vm_vspow3o2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vspow3o2

        subroutine vmdpow3o2(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdpow3o2:mkl_vm_vmdpow3o2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdpow3o2

        subroutine vdpow3o2(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdpow3o2:mkl_vm_vdpow3o2_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdpow3o2


! function: Pow3o2, indexing: strided
        subroutine vmspow3o2i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmspow3o2i:mkl_vm_vmspow3o2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmspow3o2i

        subroutine vspow3o2i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vspow3o2i:mkl_vm_vspow3o2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vspow3o2i

        subroutine vmdpow3o2i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdpow3o2i:mkl_vm_vmdpow3o2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdpow3o2i

        subroutine vdpow3o2i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdpow3o2i:mkl_vm_vdpow3o2i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdpow3o2i




! function: Powr, indexing: linear
        subroutine vmspowr(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmspowr:mkl_vm_vmspowr_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmspowr

        subroutine vspowr(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vspowr:mkl_vm_vspowr_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vspowr

        subroutine vmdpowr(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdpowr:mkl_vm_vmdpowr_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdpowr

        subroutine vdpowr(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdpowr:mkl_vm_vdpowr_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdpowr


! function: Powr, indexing: strided
        subroutine vmspowri(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmspowri:mkl_vm_vmspowri_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmspowri

        subroutine vspowri(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vspowri:mkl_vm_vspowri_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vspowri

        subroutine vmdpowri(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdpowri:mkl_vm_vmdpowri_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdpowri

        subroutine vdpowri(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdpowri:mkl_vm_vdpowri_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdpowri




! function: Powx, indexing: linear
        subroutine vmspowx(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmspowx:mkl_vm_vmspowx_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmspowx

        subroutine vspowx(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vspowx:mkl_vm_vspowx_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vspowx

        subroutine vmdpowx(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdpowx:mkl_vm_vmdpowx_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdpowx

        subroutine vdpowx(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdpowx:mkl_vm_vdpowx_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdpowx

        subroutine vmcpowx(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcpowx:mkl_vm_vmcpowx_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcpowx

        subroutine vcpowx(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcpowx:mkl_vm_vcpowx_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcpowx

        subroutine vmzpowx(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzpowx:mkl_vm_vmzpowx_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzpowx

        subroutine vzpowx(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzpowx:mkl_vm_vzpowx_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzpowx


! function: Powx, indexing: strided
        subroutine vmspowxi(n, a, inca, b, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmspowxi:mkl_vm_vmspowxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmspowxi

        subroutine vspowxi(n, a, inca, b, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vspowxi:mkl_vm_vspowxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vspowxi

        subroutine vmdpowxi(n, a, inca, b, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdpowxi:mkl_vm_vmdpowxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdpowxi

        subroutine vdpowxi(n, a, inca, b, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdpowxi:mkl_vm_vdpowxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdpowxi

        subroutine vmcpowxi(n, a, inca, b, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcpowxi:mkl_vm_vmcpowxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcpowxi

        subroutine vcpowxi(n, a, inca, b, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcpowxi:mkl_vm_vcpowxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcpowxi

        subroutine vmzpowxi(n, a, inca, b, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzpowxi:mkl_vm_vmzpowxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzpowxi

        subroutine vzpowxi(n, a, inca, b, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzpowxi:mkl_vm_vzpowxi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzpowxi




! function: Remainder, indexing: linear
        subroutine vmsremainder(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsremainder:mkl_vm_vmsremainder_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsremainder

        subroutine vsremainder(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsremainder:mkl_vm_vsremainder_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsremainder

        subroutine vmdremainder(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdremainder:mkl_vm_vmdremainder_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdremainder

        subroutine vdremainder(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdremainder:mkl_vm_vdremainder_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdremainder


! function: Remainder, indexing: strided
        subroutine vmsremainderi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsremainderi:mkl_vm_vmsremainderi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmsremainderi

        subroutine vsremainderi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsremainderi:mkl_vm_vsremainderi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vsremainderi

        subroutine vmdremainderi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdremainderi:mkl_vm_vmdremainderi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdremainderi

        subroutine vdremainderi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdremainderi:mkl_vm_vdremainderi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdremainderi




! function: Rint, indexing: linear
        subroutine vmsrint(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsrint:mkl_vm_vmsrint_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsrint

        subroutine vsrint(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsrint:mkl_vm_vsrint_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsrint

        subroutine vmdrint(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdrint:mkl_vm_vmdrint_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdrint

        subroutine vdrint(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdrint:mkl_vm_vdrint_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdrint


! function: Rint, indexing: strided
        subroutine vmsrinti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsrinti:mkl_vm_vmsrinti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsrinti

        subroutine vsrinti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsrinti:mkl_vm_vsrinti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsrinti

        subroutine vmdrinti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdrinti:mkl_vm_vmdrinti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdrinti

        subroutine vdrinti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdrinti:mkl_vm_vdrinti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdrinti




! function: Round, indexing: linear
        subroutine vmsround(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsround:mkl_vm_vmsround_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsround

        subroutine vsround(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsround:mkl_vm_vsround_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsround

        subroutine vmdround(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdround:mkl_vm_vmdround_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdround

        subroutine vdround(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdround:mkl_vm_vdround_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdround


! function: Round, indexing: strided
        subroutine vmsroundi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsroundi:mkl_vm_vmsroundi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsroundi

        subroutine vsroundi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsroundi:mkl_vm_vsroundi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsroundi

        subroutine vmdroundi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdroundi:mkl_vm_vmdroundi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdroundi

        subroutine vdroundi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdroundi:mkl_vm_vdroundi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdroundi




! function: Sin, indexing: linear
        subroutine vmssin(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssin:mkl_vm_vmssin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmssin

        subroutine vssin(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vssin:mkl_vm_vssin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vssin

        subroutine vmdsin(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsin:mkl_vm_vmdsin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdsin

        subroutine vdsin(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdsin:mkl_vm_vdsin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdsin

        subroutine vmcsin(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcsin:mkl_vm_vmcsin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcsin

        subroutine vcsin(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcsin:mkl_vm_vcsin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcsin

        subroutine vmzsin(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzsin:mkl_vm_vmzsin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzsin

        subroutine vzsin(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzsin:mkl_vm_vzsin_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzsin


! function: Sin, indexing: strided
        subroutine vmssini(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssini:mkl_vm_vmssini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmssini

        subroutine vssini(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vssini:mkl_vm_vssini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vssini

        subroutine vmdsini(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsini:mkl_vm_vmdsini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdsini

        subroutine vdsini(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdsini:mkl_vm_vdsini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdsini

        subroutine vmcsini(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcsini:mkl_vm_vmcsini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcsini

        subroutine vcsini(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcsini:mkl_vm_vcsini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcsini

        subroutine vmzsini(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzsini:mkl_vm_vmzsini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzsini

        subroutine vzsini(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzsini:mkl_vm_vzsini_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzsini




! function: SinCos, indexing: linear
        subroutine vmssincos(n, a, y, z, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        real(kind=4), intent(inout) :: z(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssincos:mkl_vm_vmssincos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vmssincos

        subroutine vssincos(n, a, y, z)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        real(kind=4), intent(inout) :: z(*)
!$omp declare variant (vssincos:mkl_vm_vssincos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vssincos

        subroutine vmdsincos(n, a, y, z, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        real(kind=8), intent(inout) :: z(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsincos:mkl_vm_vmdsincos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vmdsincos

        subroutine vdsincos(n, a, y, z)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        real(kind=8), intent(inout) :: z(*)
!$omp declare variant (vdsincos:mkl_vm_vdsincos_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vdsincos


! function: SinCos, indexing: strided
        subroutine vmssincosi(n, a, inca, y, incy, z, incz, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        real(kind=4), intent(inout) :: z(*)
        integer(kind=8), intent(in) :: incz
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssincosi:mkl_vm_vmssincosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vmssincosi

        subroutine vssincosi(n, a, inca, y, incy, z, incz)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        real(kind=4), intent(inout) :: z(*)
        integer(kind=8), intent(in) :: incz
!$omp declare variant (vssincosi:mkl_vm_vssincosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vssincosi

        subroutine vmdsincosi(n, a, inca, y, incy, z, incz, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        real(kind=8), intent(inout) :: z(*)
        integer(kind=8), intent(in) :: incz
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsincosi:mkl_vm_vmdsincosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vmdsincosi

        subroutine vdsincosi(n, a, inca, y, incy, z, incz)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        real(kind=8), intent(inout) :: z(*)
        integer(kind=8), intent(in) :: incz
!$omp declare variant (vdsincosi:mkl_vm_vdsincosi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y,z)
        end subroutine vdsincosi




! function: Sind, indexing: linear
        subroutine vmssind(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssind:mkl_vm_vmssind_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmssind

        subroutine vssind(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vssind:mkl_vm_vssind_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vssind

        subroutine vmdsind(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsind:mkl_vm_vmdsind_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdsind

        subroutine vdsind(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdsind:mkl_vm_vdsind_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdsind


! function: Sind, indexing: strided
        subroutine vmssindi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssindi:mkl_vm_vmssindi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmssindi

        subroutine vssindi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vssindi:mkl_vm_vssindi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vssindi

        subroutine vmdsindi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsindi:mkl_vm_vmdsindi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdsindi

        subroutine vdsindi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdsindi:mkl_vm_vdsindi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdsindi




! function: Sinh, indexing: linear
        subroutine vmssinh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssinh:mkl_vm_vmssinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmssinh

        subroutine vssinh(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vssinh:mkl_vm_vssinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vssinh

        subroutine vmdsinh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsinh:mkl_vm_vmdsinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdsinh

        subroutine vdsinh(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdsinh:mkl_vm_vdsinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdsinh

        subroutine vmcsinh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcsinh:mkl_vm_vmcsinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcsinh

        subroutine vcsinh(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcsinh:mkl_vm_vcsinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcsinh

        subroutine vmzsinh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzsinh:mkl_vm_vmzsinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzsinh

        subroutine vzsinh(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzsinh:mkl_vm_vzsinh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzsinh


! function: Sinh, indexing: strided
        subroutine vmssinhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssinhi:mkl_vm_vmssinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmssinhi

        subroutine vssinhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vssinhi:mkl_vm_vssinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vssinhi

        subroutine vmdsinhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsinhi:mkl_vm_vmdsinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdsinhi

        subroutine vdsinhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdsinhi:mkl_vm_vdsinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdsinhi

        subroutine vmcsinhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcsinhi:mkl_vm_vmcsinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcsinhi

        subroutine vcsinhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcsinhi:mkl_vm_vcsinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcsinhi

        subroutine vmzsinhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzsinhi:mkl_vm_vmzsinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzsinhi

        subroutine vzsinhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzsinhi:mkl_vm_vzsinhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzsinhi




! function: Sinpi, indexing: linear
        subroutine vmssinpi(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssinpi:mkl_vm_vmssinpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmssinpi

        subroutine vssinpi(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vssinpi:mkl_vm_vssinpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vssinpi

        subroutine vmdsinpi(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsinpi:mkl_vm_vmdsinpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdsinpi

        subroutine vdsinpi(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdsinpi:mkl_vm_vdsinpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdsinpi


! function: Sinpi, indexing: strided
        subroutine vmssinpii(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssinpii:mkl_vm_vmssinpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmssinpii

        subroutine vssinpii(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vssinpii:mkl_vm_vssinpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vssinpii

        subroutine vmdsinpii(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsinpii:mkl_vm_vmdsinpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdsinpii

        subroutine vdsinpii(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdsinpii:mkl_vm_vdsinpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdsinpii




! function: Sqr, indexing: linear
        subroutine vmssqr(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssqr:mkl_vm_vmssqr_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmssqr

        subroutine vssqr(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vssqr:mkl_vm_vssqr_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vssqr

        subroutine vmdsqr(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsqr:mkl_vm_vmdsqr_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdsqr

        subroutine vdsqr(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdsqr:mkl_vm_vdsqr_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdsqr


! function: Sqr, indexing: strided
        subroutine vmssqri(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssqri:mkl_vm_vmssqri_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmssqri

        subroutine vssqri(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vssqri:mkl_vm_vssqri_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vssqri

        subroutine vmdsqri(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsqri:mkl_vm_vmdsqri_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdsqri

        subroutine vdsqri(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdsqri:mkl_vm_vdsqri_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdsqri




! function: Sqrt, indexing: linear
        subroutine vmssqrt(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssqrt:mkl_vm_vmssqrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmssqrt

        subroutine vssqrt(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vssqrt:mkl_vm_vssqrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vssqrt

        subroutine vmdsqrt(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsqrt:mkl_vm_vmdsqrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdsqrt

        subroutine vdsqrt(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdsqrt:mkl_vm_vdsqrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdsqrt

        subroutine vmcsqrt(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcsqrt:mkl_vm_vmcsqrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcsqrt

        subroutine vcsqrt(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcsqrt:mkl_vm_vcsqrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcsqrt

        subroutine vmzsqrt(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzsqrt:mkl_vm_vmzsqrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzsqrt

        subroutine vzsqrt(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzsqrt:mkl_vm_vzsqrt_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzsqrt


! function: Sqrt, indexing: strided
        subroutine vmssqrti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssqrti:mkl_vm_vmssqrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmssqrti

        subroutine vssqrti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vssqrti:mkl_vm_vssqrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vssqrti

        subroutine vmdsqrti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsqrti:mkl_vm_vmdsqrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdsqrti

        subroutine vdsqrti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdsqrti:mkl_vm_vdsqrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdsqrti

        subroutine vmcsqrti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcsqrti:mkl_vm_vmcsqrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmcsqrti

        subroutine vcsqrti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcsqrti:mkl_vm_vcsqrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vcsqrti

        subroutine vmzsqrti(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzsqrti:mkl_vm_vmzsqrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmzsqrti

        subroutine vzsqrti(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzsqrti:mkl_vm_vzsqrti_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vzsqrti




! function: Sub, indexing: linear
        subroutine vmssub(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssub:mkl_vm_vmssub_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmssub

        subroutine vssub(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vssub:mkl_vm_vssub_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vssub

        subroutine vmdsub(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsub:mkl_vm_vmdsub_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdsub

        subroutine vdsub(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdsub:mkl_vm_vdsub_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdsub

        subroutine vmcsub(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcsub:mkl_vm_vmcsub_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmcsub

        subroutine vcsub(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(in) :: b(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vcsub:mkl_vm_vcsub_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vcsub

        subroutine vmzsub(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzsub:mkl_vm_vmzsub_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmzsub

        subroutine vzsub(n, a, b, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(in) :: b(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vzsub:mkl_vm_vzsub_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vzsub


! function: Sub, indexing: strided
        subroutine vmssubi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmssubi:mkl_vm_vmssubi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmssubi

        subroutine vssubi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vssubi:mkl_vm_vssubi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vssubi

        subroutine vmdsubi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdsubi:mkl_vm_vmdsubi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmdsubi

        subroutine vdsubi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdsubi:mkl_vm_vdsubi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vdsubi

        subroutine vmcsubi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmcsubi:mkl_vm_vmcsubi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmcsubi

        subroutine vcsubi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vcsubi:mkl_vm_vcsubi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vcsubi

        subroutine vmzsubi(n, a, inca, b, incb, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmzsubi:mkl_vm_vmzsubi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vmzsubi

        subroutine vzsubi(n, a, inca, b, incb, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(in) :: b(*)
        integer(kind=8), intent(in) :: incb
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vzsubi:mkl_vm_vzsubi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,b,y)
        end subroutine vzsubi




! function: Tan, indexing: linear
        subroutine vmstan(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmstan:mkl_vm_vmstan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmstan

        subroutine vstan(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vstan:mkl_vm_vstan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vstan

        subroutine vmdtan(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdtan:mkl_vm_vmdtan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdtan

        subroutine vdtan(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdtan:mkl_vm_vdtan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdtan

        subroutine vmctan(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmctan:mkl_vm_vmctan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmctan

        subroutine vctan(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vctan:mkl_vm_vctan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vctan

        subroutine vmztan(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmztan:mkl_vm_vmztan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmztan

        subroutine vztan(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vztan:mkl_vm_vztan_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vztan


! function: Tan, indexing: strided
        subroutine vmstani(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmstani:mkl_vm_vmstani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmstani

        subroutine vstani(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vstani:mkl_vm_vstani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vstani

        subroutine vmdtani(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdtani:mkl_vm_vmdtani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdtani

        subroutine vdtani(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdtani:mkl_vm_vdtani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdtani

        subroutine vmctani(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmctani:mkl_vm_vmctani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmctani

        subroutine vctani(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vctani:mkl_vm_vctani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vctani

        subroutine vmztani(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmztani:mkl_vm_vmztani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmztani

        subroutine vztani(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vztani:mkl_vm_vztani_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vztani




! function: Tand, indexing: linear
        subroutine vmstand(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmstand:mkl_vm_vmstand_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmstand

        subroutine vstand(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vstand:mkl_vm_vstand_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vstand

        subroutine vmdtand(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdtand:mkl_vm_vmdtand_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdtand

        subroutine vdtand(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdtand:mkl_vm_vdtand_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdtand


! function: Tand, indexing: strided
        subroutine vmstandi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmstandi:mkl_vm_vmstandi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmstandi

        subroutine vstandi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vstandi:mkl_vm_vstandi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vstandi

        subroutine vmdtandi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdtandi:mkl_vm_vmdtandi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdtandi

        subroutine vdtandi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdtandi:mkl_vm_vdtandi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdtandi




! function: Tanh, indexing: linear
        subroutine vmstanh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmstanh:mkl_vm_vmstanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmstanh

        subroutine vstanh(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vstanh:mkl_vm_vstanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vstanh

        subroutine vmdtanh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdtanh:mkl_vm_vmdtanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdtanh

        subroutine vdtanh(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdtanh:mkl_vm_vdtanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdtanh

        subroutine vmctanh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmctanh:mkl_vm_vmctanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmctanh

        subroutine vctanh(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        complex(kind=4), intent(inout) :: y(*)
!$omp declare variant (vctanh:mkl_vm_vctanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vctanh

        subroutine vmztanh(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmztanh:mkl_vm_vmztanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmztanh

        subroutine vztanh(n, a, y)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        complex(kind=8), intent(inout) :: y(*)
!$omp declare variant (vztanh:mkl_vm_vztanh_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vztanh


! function: Tanh, indexing: strided
        subroutine vmstanhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmstanhi:mkl_vm_vmstanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmstanhi

        subroutine vstanhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vstanhi:mkl_vm_vstanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vstanhi

        subroutine vmdtanhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdtanhi:mkl_vm_vmdtanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdtanhi

        subroutine vdtanhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdtanhi:mkl_vm_vdtanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdtanhi

        subroutine vmctanhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmctanhi:mkl_vm_vmctanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmctanhi

        subroutine vctanhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vctanhi:mkl_vm_vctanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vctanhi

        subroutine vmztanhi(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmztanhi:mkl_vm_vmztanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmztanhi

        subroutine vztanhi(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        complex(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        complex(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vztanhi:mkl_vm_vztanhi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vztanhi




! function: Tanpi, indexing: linear
        subroutine vmstanpi(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmstanpi:mkl_vm_vmstanpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmstanpi

        subroutine vstanpi(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vstanpi:mkl_vm_vstanpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vstanpi

        subroutine vmdtanpi(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdtanpi:mkl_vm_vmdtanpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdtanpi

        subroutine vdtanpi(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdtanpi:mkl_vm_vdtanpi_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdtanpi


! function: Tanpi, indexing: strided
        subroutine vmstanpii(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmstanpii:mkl_vm_vmstanpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmstanpii

        subroutine vstanpii(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vstanpii:mkl_vm_vstanpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vstanpii

        subroutine vmdtanpii(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdtanpii:mkl_vm_vmdtanpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdtanpii

        subroutine vdtanpii(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdtanpii:mkl_vm_vdtanpii_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdtanpii




! function: TGamma, indexing: linear
        subroutine vmstgamma(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmstgamma:mkl_vm_vmstgamma_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmstgamma

        subroutine vstgamma(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vstgamma:mkl_vm_vstgamma_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vstgamma

        subroutine vmdtgamma(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdtgamma:mkl_vm_vmdtgamma_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdtgamma

        subroutine vdtgamma(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdtgamma:mkl_vm_vdtgamma_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdtgamma


! function: TGamma, indexing: strided
        subroutine vmstgammai(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmstgammai:mkl_vm_vmstgammai_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmstgammai

        subroutine vstgammai(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vstgammai:mkl_vm_vstgammai_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vstgammai

        subroutine vmdtgammai(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdtgammai:mkl_vm_vmdtgammai_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdtgammai

        subroutine vdtgammai(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdtgammai:mkl_vm_vdtgammai_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdtgammai




! function: Trunc, indexing: linear
        subroutine vmstrunc(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmstrunc:mkl_vm_vmstrunc_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmstrunc

        subroutine vstrunc(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vstrunc:mkl_vm_vstrunc_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vstrunc

        subroutine vmdtrunc(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdtrunc:mkl_vm_vmdtrunc_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdtrunc

        subroutine vdtrunc(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdtrunc:mkl_vm_vdtrunc_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdtrunc


! function: Trunc, indexing: strided
        subroutine vmstrunci(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmstrunci:mkl_vm_vmstrunci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmstrunci

        subroutine vstrunci(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vstrunci:mkl_vm_vstrunci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vstrunci

        subroutine vmdtrunci(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdtrunci:mkl_vm_vmdtrunci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdtrunci

        subroutine vdtrunci(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdtrunci:mkl_vm_vdtrunci_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdtrunci




! function: Y0, indexing: linear
        subroutine vmsy0(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsy0:mkl_vm_vmsy0_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsy0

        subroutine vsy0(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsy0:mkl_vm_vsy0_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsy0

        subroutine vmdy0(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdy0:mkl_vm_vmdy0_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdy0

        subroutine vdy0(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdy0:mkl_vm_vdy0_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdy0


! function: Y0, indexing: strided
        subroutine vmsy0i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsy0i:mkl_vm_vmsy0i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsy0i

        subroutine vsy0i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsy0i:mkl_vm_vsy0i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsy0i

        subroutine vmdy0i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdy0i:mkl_vm_vmdy0i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdy0i

        subroutine vdy0i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdy0i:mkl_vm_vdy0i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdy0i




! function: Y1, indexing: linear
        subroutine vmsy1(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsy1:mkl_vm_vmsy1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsy1

        subroutine vsy1(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsy1:mkl_vm_vsy1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsy1

        subroutine vmdy1(n, a, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdy1:mkl_vm_vmdy1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdy1

        subroutine vdy1(n, a, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdy1:mkl_vm_vdy1_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdy1


! function: Y1, indexing: strided
        subroutine vmsy1i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsy1i:mkl_vm_vmsy1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsy1i

        subroutine vsy1i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsy1i:mkl_vm_vsy1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsy1i

        subroutine vmdy1i(n, a, inca, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdy1i:mkl_vm_vmdy1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdy1i

        subroutine vdy1i(n, a, inca, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdy1i:mkl_vm_vdy1i_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdy1i




! function: Yn, indexing: linear
        subroutine vmsyn(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsyn:mkl_vm_vmsyn_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsyn

        subroutine vsyn(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        real(kind=4), intent(in) :: b
        real(kind=4), intent(inout) :: y(*)
!$omp declare variant (vsyn:mkl_vm_vsyn_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsyn

        subroutine vmdyn(n, a, b, y, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdyn:mkl_vm_vmdyn_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdyn

        subroutine vdyn(n, a, b, y)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        real(kind=8), intent(in) :: b
        real(kind=8), intent(inout) :: y(*)
!$omp declare variant (vdyn:mkl_vm_vdyn_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdyn


! function: Yn, indexing: strided
        subroutine vmsyni(n, a, inca, b, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmsyni:mkl_vm_vmsyni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmsyni

        subroutine vsyni(n, a, inca, b, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=4), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=4), intent(in) :: b
        real(kind=4), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vsyni:mkl_vm_vsyni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vsyni

        subroutine vmdyni(n, a, inca, b, y, incy, mode)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
        integer(kind=8), intent(in) :: mode
!$omp declare variant (vmdyni:mkl_vm_vmdyni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vmdyni

        subroutine vdyni(n, a, inca, b, y, incy)
        integer(kind=8), intent(in) :: n
        real(kind=8), intent(in) :: a(*)
        integer(kind=8), intent(in) :: inca
        real(kind=8), intent(in) :: b
        real(kind=8), intent(inout) :: y(*)
        integer(kind=8), intent(in) :: incy
!$omp declare variant (vdyni:mkl_vm_vdyni_omp_offload_ilp64_) match( construct={dispatch}, device={arch(gen)} )  append_args(interop(prefer_type("sycl","level_zero"), targetsync)) adjust_args(need_device_addr:a,y)
        end subroutine vdyni


    end interface

end module
