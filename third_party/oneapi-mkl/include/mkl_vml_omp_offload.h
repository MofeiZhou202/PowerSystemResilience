/* -== file: mkl_vml_omp_offload.h ==- */
/*******************************************************************************
* Copyright (C) 2006 Intel Corporation
*
* This software and the related documents are Intel copyrighted  materials,  and
* your use of  them is  governed by the  express license  under which  they were
* provided to you (License).  Unless the License provides otherwise, you may not
* use, modify, copy, publish, distribute,  disclose or transmit this software or
* the related documents without Intel's prior written permission.
*
* This software and the related documents  are provided as  is,  with no express
* or implied  warranties,  other  than those  that are  expressly stated  in the
* License.
*******************************************************************************/

#ifndef MKL_VML_OMP_OFFLOAD
#define MKL_VML_OMP_OFFLOAD 1

#include <omp.h>

#include "mkl_types.h"
#include "mkl_vml_omp_variant.h"

#ifdef __cplusplus
extern "C" {
#endif

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmlsetmode)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync))
unsigned int vmlSetMode(const MKL_UINT mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmlgetmode)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync))
unsigned int vmlGetMode() NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmlseterrstatus)) match(construct = {dispatch}, device = {arch(gen)})             \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync))
int vmlSetErrStatus(MKL_INT new_status) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmlgeterrstatus)) match(construct = {dispatch}, device = {arch(gen)})             \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync))
int vmlGetErrStatus() NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmlclearerrstatus)) match(construct = {dispatch}, device = {arch(gen)})           \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync))
int vmlClearErrStatus() NOTHROW;

// function abs
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsabs)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAbs(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsabsi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAbsI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsabs)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAbs(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsabsi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAbsI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdabs)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAbs(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdabsi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAbsI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdabs)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAbs(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdabsi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAbsI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcabs)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAbs(const MKL_INT n, const MKL_Complex8* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcabsi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAbsI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcabs)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAbs(const MKL_INT n, const MKL_Complex8* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcabsi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAbsI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzabs)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAbs(const MKL_INT n, const MKL_Complex16* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzabsi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAbsI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzabs)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAbs(const MKL_INT n, const MKL_Complex16* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzabsi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAbsI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function acos
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsacos)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAcos(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsacosi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAcosI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsacos)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAcos(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsacosi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAcosI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdacos)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAcos(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdacosi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAcosI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdacos)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAcos(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdacosi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAcosI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcacos)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAcos(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcacosi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAcosI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcacos)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAcos(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcacosi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAcosI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzacos)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAcos(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzacosi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAcosI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzacos)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAcos(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzacosi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAcosI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function acosh
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsacosh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAcosh(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsacoshi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAcoshI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsacosh)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAcosh(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsacoshi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAcoshI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdacosh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAcosh(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdacoshi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAcoshI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdacosh)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAcosh(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdacoshi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAcoshI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcacosh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAcosh(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcacoshi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAcoshI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcacosh)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAcosh(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcacoshi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAcoshI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzacosh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAcosh(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzacoshi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAcoshI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzacosh)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAcosh(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzacoshi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAcoshI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function acospi
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsacospi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAcospi(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsacospii)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAcospiI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsacospi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAcospi(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsacospii)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAcospiI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdacospi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAcospi(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdacospii)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAcospiI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdacospi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAcospi(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdacospii)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAcospiI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function add
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsadd)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsAdd(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsaddi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsAddI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsadd)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsAdd(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsaddi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsAddI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
             MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdadd)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdAdd(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdaddi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdAddI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdadd)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdAdd(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdaddi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdAddI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
             MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcadd)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vcAdd(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8* b, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcaddi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vcAddI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8* b, MKL_INT incb, MKL_Complex8* y,
            MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcadd)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmcAdd(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8* b, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcaddi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmcAddI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8* b, MKL_INT incb, MKL_Complex8* y,
             MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzadd)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vzAdd(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16* b, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzaddi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vzAddI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16* b, MKL_INT incb, MKL_Complex16* y,
            MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzadd)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmzAdd(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16* b, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzaddi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmzAddI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16* b, MKL_INT incb, MKL_Complex16* y,
             MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function arg
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcarg)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcArg(const MKL_INT n, const MKL_Complex8* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcargi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcArgI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcarg)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcArg(const MKL_INT n, const MKL_Complex8* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcargi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcArgI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzarg)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzArg(const MKL_INT n, const MKL_Complex16* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzargi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzArgI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzarg)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzArg(const MKL_INT n, const MKL_Complex16* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzargi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzArgI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function asin
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsasin)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAsin(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsasini)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAsinI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsasin)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAsin(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsasini)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAsinI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdasin)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAsin(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdasini)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAsinI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdasin)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAsin(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdasini)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAsinI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcasin)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAsin(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcasini)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAsinI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcasin)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAsin(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcasini)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAsinI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzasin)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAsin(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzasini)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAsinI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzasin)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAsin(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzasini)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAsinI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function asinh
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsasinh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAsinh(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsasinhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAsinhI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsasinh)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAsinh(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsasinhi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAsinhI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdasinh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAsinh(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdasinhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAsinhI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdasinh)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAsinh(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdasinhi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAsinhI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcasinh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAsinh(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcasinhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAsinhI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcasinh)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAsinh(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcasinhi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAsinhI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzasinh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAsinh(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzasinhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAsinhI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzasinh)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAsinh(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzasinhi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAsinhI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function asinpi
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsasinpi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAsinpi(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsasinpii)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAsinpiI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsasinpi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAsinpi(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsasinpii)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAsinpiI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdasinpi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAsinpi(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdasinpii)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAsinpiI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdasinpi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAsinpi(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdasinpii)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAsinpiI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function atan
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsatan)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAtan(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsatani)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAtanI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsatan)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAtan(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsatani)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAtanI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdatan)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAtan(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdatani)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAtanI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdatan)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAtan(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdatani)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAtanI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcatan)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAtan(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcatani)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAtanI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcatan)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAtan(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcatani)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAtanI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzatan)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAtan(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzatani)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAtanI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzatan)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAtan(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzatani)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAtanI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function atan2
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsatan2)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsAtan2(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsatan2i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsAtan2I(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsatan2)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsAtan2(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsatan2i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsAtan2I(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
               MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdatan2)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdAtan2(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdatan2i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdAtan2I(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdatan2)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdAtan2(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdatan2i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdAtan2I(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
               MKL_INT64 mode) NOTHROW;

// function atan2pi
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsatan2pi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsAtan2pi(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsatan2pii)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsAtan2piI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsatan2pi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsAtan2pi(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsatan2pii)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsAtan2piI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
                 MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdatan2pi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdAtan2pi(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdatan2pii)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdAtan2piI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdatan2pi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdAtan2pi(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdatan2pii)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdAtan2piI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
                 MKL_INT64 mode) NOTHROW;

// function atanh
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsatanh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAtanh(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsatanhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAtanhI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsatanh)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAtanh(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsatanhi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAtanhI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdatanh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAtanh(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdatanhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAtanhI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdatanh)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAtanh(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdatanhi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAtanhI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcatanh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAtanh(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcatanhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcAtanhI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcatanh)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAtanh(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcatanhi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcAtanhI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzatanh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAtanh(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzatanhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzAtanhI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzatanh)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAtanh(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzatanhi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzAtanhI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function atanpi
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsatanpi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAtanpi(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsatanpii)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsAtanpiI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsatanpi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAtanpi(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsatanpii)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsAtanpiI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdatanpi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAtanpi(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdatanpii)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdAtanpiI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdatanpi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAtanpi(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdatanpii)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdAtanpiI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function cbrt
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscbrt)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCbrt(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscbrti)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCbrtI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscbrt)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCbrt(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscbrti)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCbrtI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcbrt)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCbrt(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcbrti)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCbrtI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcbrt)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCbrt(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcbrti)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCbrtI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function cdfnorm
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscdfnorm)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCdfNorm(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscdfnormi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCdfNormI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscdfnorm)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCdfNorm(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscdfnormi)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCdfNormI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcdfnorm)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCdfNorm(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcdfnormi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCdfNormI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcdfnorm)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCdfNorm(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcdfnormi)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCdfNormI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function cdfnorminv
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscdfnorminv)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCdfNormInv(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscdfnorminvi)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCdfNormInvI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscdfnorminv)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCdfNormInv(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscdfnorminvi)) match(construct = {dispatch}, device = {arch(gen)})              \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCdfNormInvI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcdfnorminv)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCdfNormInv(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcdfnorminvi)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCdfNormInvI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcdfnorminv)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCdfNormInv(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcdfnorminvi)) match(construct = {dispatch}, device = {arch(gen)})              \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCdfNormInvI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function ceil
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsceil)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCeil(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsceili)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCeilI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsceil)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCeil(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsceili)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCeilI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdceil)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCeil(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdceili)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCeilI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdceil)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCeil(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdceili)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCeilI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function cis
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vccis)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcCIS(const MKL_INT n, const float* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vccisi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcCISI(const MKL_INT n, const float* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmccis)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcCIS(const MKL_INT n, const float* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmccisi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcCISI(const MKL_INT n, const float* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzcis)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzCIS(const MKL_INT n, const double* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzcisi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzCISI(const MKL_INT n, const double* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzcis)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzCIS(const MKL_INT n, const double* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzcisi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzCISI(const MKL_INT n, const double* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function conj
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcconj)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcConj(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcconji)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcConjI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcconj)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcConj(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcconji)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcConjI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzconj)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzConj(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzconji)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzConjI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzconj)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzConj(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzconji)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzConjI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function copysign
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscopysign)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsCopySign(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscopysigni)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsCopySignI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscopysign)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsCopySign(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscopysigni)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsCopySignI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
                  MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcopysign)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdCopySign(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcopysigni)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdCopySignI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcopysign)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdCopySign(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcopysigni)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdCopySignI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
                  MKL_INT64 mode) NOTHROW;

// function cos
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscos)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCos(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscosi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCosI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscos)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCos(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscosi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCosI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcos)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCos(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcosi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCosI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcos)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCos(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcosi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCosI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vccos)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcCos(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vccosi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcCosI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmccos)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcCos(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmccosi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcCosI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzcos)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzCos(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzcosi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzCosI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzcos)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzCos(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzcosi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzCosI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function cosd
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscosd)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCosd(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscosdi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCosdI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscosd)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCosd(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscosdi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCosdI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcosd)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCosd(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcosdi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCosdI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcosd)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCosd(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcosdi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCosdI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function cosh
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscosh)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCosh(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscoshi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCoshI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscosh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCosh(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscoshi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCoshI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcosh)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCosh(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcoshi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCoshI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcosh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCosh(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcoshi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCoshI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vccosh)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcCosh(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vccoshi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcCoshI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmccosh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcCosh(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmccoshi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcCoshI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzcosh)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzCosh(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzcoshi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzCoshI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzcosh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzCosh(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzcoshi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzCoshI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function cospi
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscospi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCospi(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vscospii)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsCospiI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscospi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCospi(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmscospii)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsCospiI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcospi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCospi(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdcospii)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdCospiI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcospi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCospi(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdcospii)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdCospiI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function div
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsdiv)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsDiv(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsdivi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsDivI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsdiv)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsDiv(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsdivi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsDivI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
             MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vddiv)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdDiv(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vddivi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdDivI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmddiv)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdDiv(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmddivi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdDivI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
             MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcdiv)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vcDiv(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8* b, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcdivi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vcDivI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8* b, MKL_INT incb, MKL_Complex8* y,
            MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcdiv)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmcDiv(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8* b, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcdivi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmcDivI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8* b, MKL_INT incb, MKL_Complex8* y,
             MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzdiv)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vzDiv(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16* b, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzdivi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vzDivI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16* b, MKL_INT incb, MKL_Complex16* y,
            MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzdiv)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmzDiv(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16* b, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzdivi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmzDivI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16* b, MKL_INT incb, MKL_Complex16* y,
             MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function erf
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vserf)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsErf(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vserfi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsErfI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmserf)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsErf(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmserfi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsErfI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vderf)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdErf(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vderfi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdErfI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmderf)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdErf(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmderfi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdErfI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function erfc
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vserfc)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsErfc(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vserfci)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsErfcI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmserfc)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsErfc(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmserfci)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsErfcI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vderfc)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdErfc(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vderfci)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdErfcI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmderfc)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdErfc(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmderfci)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdErfcI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function erfcinv
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vserfcinv)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsErfcInv(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vserfcinvi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsErfcInvI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmserfcinv)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsErfcInv(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmserfcinvi)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsErfcInvI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vderfcinv)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdErfcInv(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vderfcinvi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdErfcInvI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmderfcinv)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdErfcInv(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmderfcinvi)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdErfcInvI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function erfcx
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vserfcx)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsErfcx(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vserfcxi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsErfcxI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmserfcx)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsErfcx(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmserfcxi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsErfcxI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vderfcx)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdErfcx(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vderfcxi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdErfcxI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmderfcx)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdErfcx(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmderfcxi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdErfcxI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function erfinv
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vserfinv)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsErfInv(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vserfinvi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsErfInvI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmserfinv)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsErfInv(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmserfinvi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsErfInvI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vderfinv)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdErfInv(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vderfinvi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdErfInvI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmderfinv)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdErfInv(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmderfinvi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdErfInvI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function exp
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsexp)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsExp(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsexpi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsExpI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsexp)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsExp(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsexpi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsExpI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdexp)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdExp(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdexpi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdExpI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdexp)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdExp(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdexpi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdExpI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcexp)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcExp(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcexpi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcExpI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcexp)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcExp(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcexpi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcExpI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzexp)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzExp(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzexpi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzExpI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzexp)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzExp(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzexpi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzExpI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function exp10
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsexp10)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsExp10(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsexp10i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsExp10I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsexp10)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsExp10(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsexp10i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsExp10I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdexp10)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdExp10(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdexp10i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdExp10I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdexp10)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdExp10(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdexp10i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdExp10I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function exp2
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsexp2)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsExp2(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsexp2i)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsExp2I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsexp2)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsExp2(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsexp2i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsExp2I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdexp2)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdExp2(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdexp2i)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdExp2I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdexp2)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdExp2(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdexp2i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdExp2I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function expint1
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsexpint1)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsExpInt1(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsexpint1i)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsExpInt1I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsexpint1)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsExpInt1(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsexpint1i)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsExpInt1I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdexpint1)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdExpInt1(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdexpint1i)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdExpInt1I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdexpint1)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdExpInt1(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdexpint1i)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdExpInt1I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function expm1
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsexpm1)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsExpm1(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsexpm1i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsExpm1I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsexpm1)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsExpm1(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsexpm1i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsExpm1I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdexpm1)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdExpm1(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdexpm1i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdExpm1I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdexpm1)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdExpm1(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdexpm1i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdExpm1I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function fdim
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsfdim)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsFdim(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsfdimi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsFdimI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsfdim)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsFdim(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsfdimi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsFdimI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
              MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdfdim)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdFdim(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdfdimi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdFdimI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdfdim)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdFdim(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdfdimi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdFdimI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
              MKL_INT64 mode) NOTHROW;

// function floor
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsfloor)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsFloor(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsfloori)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsFloorI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsfloor)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsFloor(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsfloori)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsFloorI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdfloor)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdFloor(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdfloori)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdFloorI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdfloor)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdFloor(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdfloori)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdFloorI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function fmax
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsfmax)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsFmax(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsfmaxi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsFmaxI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsfmax)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsFmax(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsfmaxi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsFmaxI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
              MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdfmax)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdFmax(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdfmaxi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdFmaxI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdfmax)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdFmax(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdfmaxi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdFmaxI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
              MKL_INT64 mode) NOTHROW;

// function fmin
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsfmin)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsFmin(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsfmini)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsFminI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsfmin)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsFmin(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsfmini)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsFminI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
              MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdfmin)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdFmin(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdfmini)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdFminI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdfmin)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdFmin(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdfmini)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdFminI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
              MKL_INT64 mode) NOTHROW;

// function fmod
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsfmod)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsFmod(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsfmodi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsFmodI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsfmod)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsFmod(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsfmodi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsFmodI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
              MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdfmod)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdFmod(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdfmodi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdFmodI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdfmod)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdFmod(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdfmodi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdFmodI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
              MKL_INT64 mode) NOTHROW;

// function frac
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsfrac)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsFrac(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsfraci)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsFracI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsfrac)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsFrac(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsfraci)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsFracI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdfrac)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdFrac(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdfraci)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdFracI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdfrac)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdFrac(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdfraci)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdFracI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function hypot
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vshypot)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsHypot(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vshypoti)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsHypotI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmshypot)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsHypot(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmshypoti)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsHypotI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
               MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdhypot)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdHypot(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdhypoti)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdHypotI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdhypot)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdHypot(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdhypoti)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdHypotI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
               MKL_INT64 mode) NOTHROW;

// function i0
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsi0)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsI0(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsi0i)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsI0I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsi0)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsI0(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsi0i)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsI0I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdi0)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdI0(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdi0i)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdI0I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdi0)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdI0(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdi0i)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdI0I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function i1
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsi1)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsI1(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsi1i)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsI1I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsi1)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsI1(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsi1i)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsI1I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdi1)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdI1(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdi1i)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdI1I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdi1)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdI1(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdi1i)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdI1I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function inv
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsinv)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsInv(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsinvi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsInvI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsinv)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsInv(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsinvi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsInvI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdinv)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdInv(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdinvi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdInvI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdinv)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdInv(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdinvi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdInvI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function invcbrt
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsinvcbrt)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsInvCbrt(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsinvcbrti)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsInvCbrtI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsinvcbrt)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsInvCbrt(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsinvcbrti)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsInvCbrtI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdinvcbrt)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdInvCbrt(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdinvcbrti)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdInvCbrtI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdinvcbrt)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdInvCbrt(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdinvcbrti)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdInvCbrtI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function invsqrt
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsinvsqrt)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsInvSqrt(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsinvsqrti)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsInvSqrtI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsinvsqrt)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsInvSqrt(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsinvsqrti)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsInvSqrtI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdinvsqrt)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdInvSqrt(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdinvsqrti)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdInvSqrtI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdinvsqrt)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdInvSqrt(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdinvsqrti)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdInvSqrtI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function j0
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsj0)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsJ0(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsj0i)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsJ0I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsj0)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsJ0(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsj0i)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsJ0I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdj0)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdJ0(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdj0i)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdJ0I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdj0)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdJ0(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdj0i)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdJ0I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function j1
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsj1)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsJ1(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsj1i)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsJ1I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsj1)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsJ1(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsj1i)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsJ1I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdj1)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdJ1(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdj1i)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdJ1I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdj1)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdJ1(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdj1i)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdJ1I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function jn
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsjn)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsJn(const MKL_INT n, const float* a, const float b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsjni)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsJnI(const MKL_INT n, const float* a, MKL_INT inca, const float b, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsjn)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsJn(const MKL_INT n, const float* a, const float b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsjni)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsJnI(const MKL_INT n, const float* a, MKL_INT inca, const float b, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdjn)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdJn(const MKL_INT n, const double* a, const double b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdjni)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdJnI(const MKL_INT n, const double* a, MKL_INT inca, const double b, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdjn)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdJn(const MKL_INT n, const double* a, const double b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdjni)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdJnI(const MKL_INT n, const double* a, MKL_INT inca, const double b, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function lgamma
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vslgamma)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsLGamma(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vslgammai)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsLGammaI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmslgamma)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsLGamma(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmslgammai)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsLGammaI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdlgamma)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdLGamma(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdlgammai)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdLGammaI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdlgamma)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdLGamma(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdlgammai)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdLGammaI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function linearfrac
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vslinearfrac)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsLinearFrac(const MKL_INT n, const float* a, const float* b, const float c, const float d, const float e, const float f,
                  float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vslinearfraci)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsLinearFracI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, const float c, const float d,
                   const float e, const float f, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmslinearfrac)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsLinearFrac(const MKL_INT n, const float* a, const float* b, const float c, const float d, const float e, const float f,
                   float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmslinearfraci)) match(construct = {dispatch}, device = {arch(gen)})              \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsLinearFracI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, const float c, const float d,
                    const float e, const float f, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdlinearfrac)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdLinearFrac(const MKL_INT n, const double* a, const double* b, const double c, const double d, const double e, const double f,
                  double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdlinearfraci)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdLinearFracI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, const double c, const double d,
                   const double e, const double f, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdlinearfrac)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdLinearFrac(const MKL_INT n, const double* a, const double* b, const double c, const double d, const double e,
                   const double f, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdlinearfraci)) match(construct = {dispatch}, device = {arch(gen)})              \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdLinearFracI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, const double c, const double d,
                    const double e, const double f, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function ln
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsln)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsLn(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vslni)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsLnI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsln)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsLn(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmslni)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsLnI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdln)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdLn(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdlni)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdLnI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdln)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdLn(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdlni)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdLnI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcln)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcLn(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vclni)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcLnI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcln)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcLn(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmclni)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcLnI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzln)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzLn(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzlni)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzLnI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzln)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzLn(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzlni)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzLnI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function log10
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vslog10)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsLog10(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vslog10i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsLog10I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmslog10)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsLog10(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmslog10i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsLog10I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdlog10)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdLog10(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdlog10i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdLog10I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdlog10)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdLog10(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdlog10i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdLog10I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vclog10)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcLog10(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vclog10i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcLog10I(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmclog10)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcLog10(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmclog10i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcLog10I(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzlog10)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzLog10(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzlog10i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzLog10I(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzlog10)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzLog10(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzlog10i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzLog10I(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function log1p
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vslog1p)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsLog1p(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vslog1pi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsLog1pI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmslog1p)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsLog1p(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmslog1pi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsLog1pI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdlog1p)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdLog1p(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdlog1pi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdLog1pI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdlog1p)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdLog1p(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdlog1pi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdLog1pI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function log2
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vslog2)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsLog2(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vslog2i)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsLog2I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmslog2)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsLog2(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmslog2i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsLog2I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdlog2)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdLog2(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdlog2i)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdLog2I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdlog2)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdLog2(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdlog2i)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdLog2I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function logb
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vslogb)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsLogb(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vslogbi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsLogbI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmslogb)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsLogb(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmslogbi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsLogbI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdlogb)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdLogb(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdlogbi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdLogbI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdlogb)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdLogb(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdlogbi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdLogbI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function maxmag
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsmaxmag)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsMaxMag(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsmaxmagi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsMaxMagI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsmaxmag)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsMaxMag(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsmaxmagi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsMaxMagI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
                MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdmaxmag)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdMaxMag(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdmaxmagi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdMaxMagI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdmaxmag)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdMaxMag(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdmaxmagi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdMaxMagI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
                MKL_INT64 mode) NOTHROW;

// function minmag
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsminmag)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsMinMag(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsminmagi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsMinMagI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsminmag)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsMinMag(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsminmagi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsMinMagI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
                MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdminmag)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdMinMag(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdminmagi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdMinMagI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdminmag)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdMinMag(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdminmagi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdMinMagI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
                MKL_INT64 mode) NOTHROW;

// function modf
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsmodf)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vsModf(const MKL_INT n, const float* a, float* y, float* z) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsmodfi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vsModfI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, float* z, MKL_INT incz) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsmodf)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vmsModf(const MKL_INT n, const float* a, float* y, float* z, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsmodfi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vmsModfI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, float* z, MKL_INT incz,
              MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdmodf)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vdModf(const MKL_INT n, const double* a, double* y, double* z) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdmodfi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vdModfI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, double* z, MKL_INT incz) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdmodf)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vmdModf(const MKL_INT n, const double* a, double* y, double* z, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdmodfi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vmdModfI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, double* z, MKL_INT incz,
              MKL_INT64 mode) NOTHROW;

// function mul
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsmul)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsMul(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsmuli)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsMulI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsmul)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsMul(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsmuli)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsMulI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
             MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdmul)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdMul(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdmuli)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdMulI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdmul)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdMul(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdmuli)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdMulI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
             MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcmul)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vcMul(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8* b, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcmuli)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vcMulI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8* b, MKL_INT incb, MKL_Complex8* y,
            MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcmul)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmcMul(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8* b, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcmuli)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmcMulI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8* b, MKL_INT incb, MKL_Complex8* y,
             MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzmul)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vzMul(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16* b, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzmuli)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vzMulI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16* b, MKL_INT incb, MKL_Complex16* y,
            MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzmul)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmzMul(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16* b, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzmuli)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmzMulI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16* b, MKL_INT incb, MKL_Complex16* y,
             MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function mulbyconj
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcmulbyconj)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vcMulByConj(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8* b, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcmulbyconji)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vcMulByConjI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8* b, MKL_INT incb, MKL_Complex8* y,
                  MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcmulbyconj)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmcMulByConj(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8* b, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcmulbyconji)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmcMulByConjI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8* b, MKL_INT incb, MKL_Complex8* y,
                   MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzmulbyconj)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vzMulByConj(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16* b, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzmulbyconji)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vzMulByConjI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16* b, MKL_INT incb, MKL_Complex16* y,
                  MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzmulbyconj)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmzMulByConj(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16* b, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzmulbyconji)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmzMulByConjI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16* b, MKL_INT incb, MKL_Complex16* y,
                   MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function nearbyint
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsnearbyint)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsNearbyInt(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsnearbyinti)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsNearbyIntI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsnearbyint)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsNearbyInt(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsnearbyinti)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsNearbyIntI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdnearbyint)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdNearbyInt(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdnearbyinti)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdNearbyIntI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdnearbyint)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdNearbyInt(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdnearbyinti)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdNearbyIntI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function nextafter
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsnextafter)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsNextAfter(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsnextafteri)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsNextAfterI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsnextafter)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsNextAfter(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsnextafteri)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsNextAfterI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
                   MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdnextafter)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdNextAfter(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdnextafteri)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdNextAfterI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdnextafter)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdNextAfter(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdnextafteri)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdNextAfterI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
                   MKL_INT64 mode) NOTHROW;

// function pow
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vspow)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsPow(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vspowi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsPowI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmspow)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsPow(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmspowi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsPowI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
             MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdpow)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdPow(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdpowi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdPowI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdpow)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdPow(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdpowi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdPowI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
             MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcpow)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vcPow(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8* b, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcpowi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vcPowI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8* b, MKL_INT incb, MKL_Complex8* y,
            MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcpow)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmcPow(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8* b, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcpowi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmcPowI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8* b, MKL_INT incb, MKL_Complex8* y,
             MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzpow)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vzPow(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16* b, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzpowi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vzPowI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16* b, MKL_INT incb, MKL_Complex16* y,
            MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzpow)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmzPow(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16* b, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzpowi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmzPowI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16* b, MKL_INT incb, MKL_Complex16* y,
             MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function pow2o3
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vspow2o3)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsPow2o3(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vspow2o3i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsPow2o3I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmspow2o3)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsPow2o3(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmspow2o3i)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsPow2o3I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdpow2o3)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdPow2o3(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdpow2o3i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdPow2o3I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdpow2o3)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdPow2o3(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdpow2o3i)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdPow2o3I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function pow3o2
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vspow3o2)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsPow3o2(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vspow3o2i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsPow3o2I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmspow3o2)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsPow3o2(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmspow3o2i)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsPow3o2I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdpow3o2)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdPow3o2(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdpow3o2i)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdPow3o2I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdpow3o2)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdPow3o2(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdpow3o2i)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdPow3o2I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function powr
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vspowr)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsPowr(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vspowri)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsPowrI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmspowr)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsPowr(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmspowri)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsPowrI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
              MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdpowr)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdPowr(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdpowri)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdPowrI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdpowr)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdPowr(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdpowri)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdPowrI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
              MKL_INT64 mode) NOTHROW;

// function powx
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vspowx)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsPowx(const MKL_INT n, const float* a, const float b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vspowxi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsPowxI(const MKL_INT n, const float* a, MKL_INT inca, const float b, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmspowx)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsPowx(const MKL_INT n, const float* a, const float b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmspowxi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsPowxI(const MKL_INT n, const float* a, MKL_INT inca, const float b, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdpowx)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdPowx(const MKL_INT n, const double* a, const double b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdpowxi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdPowxI(const MKL_INT n, const double* a, MKL_INT inca, const double b, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdpowx)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdPowx(const MKL_INT n, const double* a, const double b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdpowxi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdPowxI(const MKL_INT n, const double* a, MKL_INT inca, const double b, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcpowx)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcPowx(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8 b, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcpowxi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcPowxI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8 b, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcpowx)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcPowx(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8 b, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcpowxi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcPowxI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8 b, MKL_Complex8* y, MKL_INT incy,
              MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzpowx)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzPowx(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16 b, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzpowxi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzPowxI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16 b, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzpowx)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzPowx(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16 b, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzpowxi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzPowxI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16 b, MKL_Complex16* y, MKL_INT incy,
              MKL_INT64 mode) NOTHROW;

// function remainder
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsremainder)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsRemainder(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsremainderi)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsRemainderI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsremainder)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsRemainder(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsremainderi)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsRemainderI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
                   MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdremainder)) match(construct = {dispatch}, device = {arch(gen)})                 \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdRemainder(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdremainderi)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdRemainderI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdremainder)) match(construct = {dispatch}, device = {arch(gen)})                \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdRemainder(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdremainderi)) match(construct = {dispatch}, device = {arch(gen)})               \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdRemainderI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
                   MKL_INT64 mode) NOTHROW;

// function rint
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsrint)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsRint(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsrinti)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsRintI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsrint)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsRint(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsrinti)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsRintI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdrint)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdRint(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdrinti)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdRintI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdrint)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdRint(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdrinti)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdRintI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function round
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsround)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsRound(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsroundi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsRoundI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsround)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsRound(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsroundi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsRoundI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdround)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdRound(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdroundi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdRoundI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdround)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdRound(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdroundi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdRoundI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function sin
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssin)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsSin(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssini)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsSinI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssin)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsSin(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssini)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsSinI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsin)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdSin(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsini)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdSinI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsin)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdSin(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsini)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdSinI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcsin)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcSin(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcsini)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcSinI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcsin)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcSin(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcsini)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcSinI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzsin)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzSin(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzsini)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzSinI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzsin)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzSin(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzsini)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzSinI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function sincos
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssincos)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vsSinCos(const MKL_INT n, const float* a, float* y, float* z) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssincosi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vsSinCosI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, float* z, MKL_INT incz) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssincos)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vmsSinCos(const MKL_INT n, const float* a, float* y, float* z, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssincosi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vmsSinCosI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, float* z, MKL_INT incz,
                MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsincos)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vdSinCos(const MKL_INT n, const double* a, double* y, double* z) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsincosi)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vdSinCosI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, double* z, MKL_INT incz) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsincos)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vmdSinCos(const MKL_INT n, const double* a, double* y, double* z, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsincosi)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : z)
void vmdSinCosI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, double* z, MKL_INT incz,
                MKL_INT64 mode) NOTHROW;

// function sincospi

// function sind
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssind)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsSind(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssindi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsSindI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssind)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsSind(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssindi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsSindI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsind)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdSind(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsindi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdSindI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsind)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdSind(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsindi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdSindI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function sinh
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssinh)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsSinh(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssinhi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsSinhI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssinh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsSinh(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssinhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsSinhI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsinh)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdSinh(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsinhi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdSinhI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsinh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdSinh(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsinhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdSinhI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcsinh)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcSinh(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcsinhi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcSinhI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcsinh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcSinh(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcsinhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcSinhI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzsinh)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzSinh(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzsinhi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzSinhI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzsinh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzSinh(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzsinhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzSinhI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function sinpi
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssinpi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsSinpi(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssinpii)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsSinpiI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssinpi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsSinpi(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssinpii)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsSinpiI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsinpi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdSinpi(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsinpii)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdSinpiI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsinpi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdSinpi(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsinpii)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdSinpiI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function sqr
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssqr)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsSqr(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssqri)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsSqrI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssqr)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsSqr(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssqri)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsSqrI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsqr)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdSqr(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsqri)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdSqrI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsqr)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdSqr(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsqri)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdSqrI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function sqrt
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssqrt)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsSqrt(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssqrti)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsSqrtI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssqrt)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsSqrt(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssqrti)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsSqrtI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsqrt)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdSqrt(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsqrti)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdSqrtI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsqrt)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdSqrt(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsqrti)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdSqrtI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcsqrt)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcSqrt(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcsqrti)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcSqrtI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcsqrt)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcSqrt(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcsqrti)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcSqrtI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzsqrt)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzSqrt(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzsqrti)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzSqrtI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzsqrt)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzSqrt(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzsqrti)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzSqrtI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function sub
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssub)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsSub(const MKL_INT n, const float* a, const float* b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vssubi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vsSubI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssub)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsSub(const MKL_INT n, const float* a, const float* b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmssubi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmsSubI(const MKL_INT n, const float* a, MKL_INT inca, const float* b, MKL_INT incb, float* y, MKL_INT incy,
             MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsub)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdSub(const MKL_INT n, const double* a, const double* b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdsubi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vdSubI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsub)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdSub(const MKL_INT n, const double* a, const double* b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdsubi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmdSubI(const MKL_INT n, const double* a, MKL_INT inca, const double* b, MKL_INT incb, double* y, MKL_INT incy,
             MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcsub)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vcSub(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8* b, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vcsubi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vcSubI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8* b, MKL_INT incb, MKL_Complex8* y,
            MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcsub)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmcSub(const MKL_INT n, const MKL_Complex8* a, const MKL_Complex8* b, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmcsubi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmcSubI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, const MKL_Complex8* b, MKL_INT incb, MKL_Complex8* y,
             MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzsub)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vzSub(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16* b, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vzsubi)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vzSubI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16* b, MKL_INT incb, MKL_Complex16* y,
            MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzsub)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmzSub(const MKL_INT n, const MKL_Complex16* a, const MKL_Complex16* b, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmzsubi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : b)                          \
        adjust_args(need_device_ptr(fb_preserve)                                                                                   \
                    : y)
void vmzSubI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, const MKL_Complex16* b, MKL_INT incb, MKL_Complex16* y,
             MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function tan
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vstan)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsTan(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vstani)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsTanI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmstan)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsTan(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmstani)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsTanI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdtan)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdTan(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdtani)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdTanI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdtan)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdTan(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdtani)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdTanI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vctan)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcTan(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vctani)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcTanI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmctan)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcTan(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmctani)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcTanI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vztan)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzTan(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vztani)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzTanI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmztan)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzTan(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmztani)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzTanI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function tand
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vstand)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsTand(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vstandi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsTandI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmstand)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsTand(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmstandi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsTandI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdtand)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdTand(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdtandi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdTandI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdtand)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdTand(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdtandi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdTandI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function tanh
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vstanh)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsTanh(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vstanhi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsTanhI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmstanh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsTanh(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmstanhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsTanhI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdtanh)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdTanh(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdtanhi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdTanhI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdtanh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdTanh(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdtanhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdTanhI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vctanh)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcTanh(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vctanhi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vcTanhI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmctanh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcTanh(const MKL_INT n, const MKL_Complex8* a, MKL_Complex8* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmctanhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmcTanhI(const MKL_INT n, const MKL_Complex8* a, MKL_INT inca, MKL_Complex8* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vztanh)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzTanh(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vztanhi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vzTanhI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmztanh)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzTanh(const MKL_INT n, const MKL_Complex16* a, MKL_Complex16* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmztanhi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmzTanhI(const MKL_INT n, const MKL_Complex16* a, MKL_INT inca, MKL_Complex16* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function tanpi
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vstanpi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsTanpi(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vstanpii)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsTanpiI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmstanpi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsTanpi(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmstanpii)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsTanpiI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdtanpi)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdTanpi(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdtanpii)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdTanpiI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdtanpi)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdTanpi(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdtanpii)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdTanpiI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function tgamma
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vstgamma)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsTGamma(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vstgammai)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsTGammaI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmstgamma)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsTGamma(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmstgammai)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsTGammaI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdtgamma)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdTGamma(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdtgammai)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdTGammaI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdtgamma)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdTGamma(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdtgammai)) match(construct = {dispatch}, device = {arch(gen)})                  \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdTGammaI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function trunc
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vstrunc)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsTrunc(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vstrunci)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsTruncI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmstrunc)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsTrunc(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmstrunci)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsTruncI(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdtrunc)) match(construct = {dispatch}, device = {arch(gen)})                     \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdTrunc(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdtrunci)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdTruncI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdtrunc)) match(construct = {dispatch}, device = {arch(gen)})                    \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdTrunc(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdtrunci)) match(construct = {dispatch}, device = {arch(gen)})                   \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdTruncI(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function y0
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsy0)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsY0(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsy0i)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsY0I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsy0)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsY0(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsy0i)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsY0I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdy0)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdY0(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdy0i)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdY0I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdy0)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdY0(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdy0i)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdY0I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function y1
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsy1)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsY1(const MKL_INT n, const float* a, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsy1i)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsY1I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsy1)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsY1(const MKL_INT n, const float* a, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsy1i)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsY1I(const MKL_INT n, const float* a, MKL_INT inca, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdy1)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdY1(const MKL_INT n, const double* a, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdy1i)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdY1I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdy1)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdY1(const MKL_INT n, const double* a, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdy1i)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdY1I(const MKL_INT n, const double* a, MKL_INT inca, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

// function yn
#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsyn)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsYn(const MKL_INT n, const float* a, const float b, float* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vsyni)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vsYnI(const MKL_INT n, const float* a, MKL_INT inca, const float b, float* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsyn)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsYn(const MKL_INT n, const float* a, const float b, float* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmsyni)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmsYnI(const MKL_INT n, const float* a, MKL_INT inca, const float b, float* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdyn)) match(construct = {dispatch}, device = {arch(gen)})                        \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdYn(const MKL_INT n, const double* a, const double b, double* y) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vdyni)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vdYnI(const MKL_INT n, const double* a, MKL_INT inca, const double b, double* y, MKL_INT incy) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdyn)) match(construct = {dispatch}, device = {arch(gen)})                       \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdYn(const MKL_INT n, const double* a, const double b, double* y, MKL_INT64 mode) NOTHROW;

#pragma omp declare variant(MKL_VARIANT_NAME(vm, vmdyni)) match(construct = {dispatch}, device = {arch(gen)})                      \
    append_args(interop(prefer_type("sycl", "level_zero"), targetsync)) adjust_args(need_device_ptr(fb_preserve)                   \
                                                                                    : a) adjust_args(need_device_ptr(fb_preserve)  \
                                                                                                     : y)
void vmdYnI(const MKL_INT n, const double* a, MKL_INT inca, const double b, double* y, MKL_INT incy, MKL_INT64 mode) NOTHROW;

#ifdef __cplusplus
}
#endif

#endif
