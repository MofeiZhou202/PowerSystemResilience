/*******************************************************************************
* Copyright (C) 2020 Intel Corporation
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

/*
!  Content:
!      Intel(R) oneAPI Math Kernel Library (oneMKL) for OpenMP compiler offload
!      interface
!******************************************************************************/

#ifndef _MKL_VSL_OMP_OFFLOAD_H_
#define _MKL_VSL_OMP_OFFLOAD_H_

#include <omp.h>

#include "mkl_vsl_types.h"
#include "mkl_vsl_omp_variant.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vsRngUniform)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vsRngUniform(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, float r[], const float a, const float b) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vdRngUniform)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vdRngUniform(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, double r[], const double a, const double b) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, viRngUniform)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int viRngUniform(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, int r[], const int a, const int b) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vsRngGaussian)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vsRngGaussian(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, float r[], const float mean, const float stddev) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vdRngGaussian)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vdRngGaussian(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, double r[], const double mean, const double stddev) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vsRngGaussianMV)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vsRngGaussianMV(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, float r[], const MKL_INT dimen, const MKL_INT mstorage, const float* a, const float* t) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vdRngGaussianMV)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vdRngGaussianMV(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, double r[], const MKL_INT dimen, const MKL_INT mstorage, const double* a, const double* t) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vsRngLognormal)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vsRngLognormal(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, float r[], const float mean, const float stddev, const float displ, const float scale) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vdRngLognormal)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vdRngLognormal(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, double r[], const double mean, const double stddev, const double displ, const double scale) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vsRngCauchy)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vsRngCauchy(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, float r[], const float a, const float b) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vdRngCauchy)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vdRngCauchy(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, double r[], const double a, const double b) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vsRngExponential)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vsRngExponential(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, float r[], const float a, const float b) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vdRngExponential)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vdRngExponential(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, double r[], const double a, const double b) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vsRngGumbel)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vsRngGumbel(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, float r[], const float a, const float b) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vdRngGumbel)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vdRngGumbel(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, double r[], const double a, const double b) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vsRngLaplace)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vsRngLaplace(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, float r[], const float a, const float b) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vdRngLaplace)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vdRngLaplace(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, double r[], const double a, const double b) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vsRngRayleigh)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vsRngRayleigh(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, float r[], const float a, const float b) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vdRngRayleigh)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vdRngRayleigh(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, double r[], const double a, const double b) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vsRngWeibull)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vsRngWeibull(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, float r[], const float alpha, const float a, const float beta) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vdRngWeibull)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vdRngWeibull(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, double r[], const double alpha, const double a, const double beta) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vsRngBeta)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vsRngBeta(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, float r[], const float p, const float q, const float a, const float beta) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vdRngBeta)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vdRngBeta(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, double r[], const double p, const double q, const double a, const double beta) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vsRngGamma)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vsRngGamma(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, float r[], const float alpha, const float a, const float beta) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vdRngGamma)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vdRngGamma(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, double r[], const double alpha, const double a, const double beta) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vsRngChiSquare)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vsRngChiSquare(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, float r[], const int v) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, vdRngChiSquare)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int vdRngChiSquare(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, double r[], const int v) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, viRngHypergeometric)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int viRngHypergeometric(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, int r[], const int l, const int s, const int m) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, viRngBinomial)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int viRngBinomial(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, int r[], const int ntrial, const double p) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, viRngMultinomial)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int viRngMultinomial(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, int r[], const int ntrial, const int k, const double* p) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, viRngPoissonV)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int viRngPoissonV(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, int r[], const double* lambda) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, viRngNegbinomial)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int viRngNegbinomial(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, int r[], const double a, const double p) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, viRngBernoulli)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int viRngBernoulli(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, int r[], const double p) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, viRngGeometric)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int viRngGeometric(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, int r[], const double p) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, viRngPoisson)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int viRngPoisson(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, int r[], const double lambda) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, viRngUniformBits)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int viRngUniformBits(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, unsigned int r[]) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, viRngUniformBits32)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int viRngUniformBits32(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, unsigned int r[]) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, viRngUniformBits64)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync)) \
    adjust_args(need_device_ptr(fb_preserve):r)
int viRngUniformBits64(const MKL_INT method, VSLStreamStatePtr stream, const MKL_INT n, unsigned MKL_INT64 r[]) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, sSSCompute)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync))
int vslsSSCompute(VSLSSTaskPtr, const unsigned MKL_INT64 estimate, const MKL_INT method) NOTHROW;

#pragma omp declare variant (MKL_VARIANT_NAME(vsl, dSSCompute)) match(construct={dispatch}, device={arch(gen)}) \
    append_args(interop(prefer_type("sycl","level_zero"),targetsync))
int vsldSSCompute(VSLSSTaskPtr, const unsigned MKL_INT64 estimate, const MKL_INT method) NOTHROW;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // _MKL_VSL_OMP_OFFLOAD_H_
