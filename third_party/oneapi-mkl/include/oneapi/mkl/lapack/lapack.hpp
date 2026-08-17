/*******************************************************************************
* Copyright (C) 2024 Intel Corporation
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

#ifndef _ONEAPI_MKL_LAPACK_LAPACK_HPP__
#define _ONEAPI_MKL_LAPACK_LAPACK_HPP__

#include <complex>
#include <cstdint>
#include <sycl/sycl.hpp>

#include "oneapi/mkl/types.hpp"
#include "oneapi/mkl/export.hpp"

namespace oneapi {
namespace mkl {
namespace lapack {

// potrf
DLL_EXPORT sycl::event potrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, float  *a, std::int64_t lda, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, double *a, std::int64_t lda, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::complex<float>  *a, std::int64_t lda, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::complex<double> *a, std::int64_t lda, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void potrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<float>  &a, std::int64_t lda, sycl::buffer<float>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<double> &a, std::int64_t lda, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<std::complex<double>> &a, std::int64_t lda, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

// potrs
DLL_EXPORT sycl::event potrs(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, const float  *a, std::int64_t lda, float  *b, std::int64_t ldb, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrs(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, const double *a, std::int64_t lda, double *b, std::int64_t ldb, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrs(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, const std::complex<float>  *a, std::int64_t lda, std::complex<float>  *b, std::int64_t ldb, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrs(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, const std::complex<double> *a, std::int64_t lda, std::complex<double> *b, std::int64_t ldb, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void potrs(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, sycl::buffer<float>  &a, std::int64_t lda, sycl::buffer<float>  &b, std::int64_t ldb, sycl::buffer<float>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potrs(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, sycl::buffer<double> &a, std::int64_t lda, sycl::buffer<double> &b, std::int64_t ldb, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potrs(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, sycl::buffer<std::complex<float>>  &b, std::int64_t ldb, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potrs(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, sycl::buffer<std::complex<double>> &a, std::int64_t lda, sycl::buffer<std::complex<double>> &b, std::int64_t ldb, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

// potri
DLL_EXPORT sycl::event potri(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, float  *a, std::int64_t lda, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potri(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, double *a, std::int64_t lda, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potri(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::complex<float>  *a, std::int64_t lda, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potri(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::complex<double> *a, std::int64_t lda, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void potri(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<float>  &a, std::int64_t lda, sycl::buffer<float>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potri(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<double> &a, std::int64_t lda, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potri(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potri(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<std::complex<double>> &a, std::int64_t lda, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

// trtri
DLL_EXPORT sycl::event trtri(sycl::queue &queue, oneapi::mkl::uplo uplo, oneapi::mkl::diag diag, std::int64_t n, float  *a, std::int64_t lda, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event trtri(sycl::queue &queue, oneapi::mkl::uplo uplo, oneapi::mkl::diag diag, std::int64_t n, double *a, std::int64_t lda, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event trtri(sycl::queue &queue, oneapi::mkl::uplo uplo, oneapi::mkl::diag diag, std::int64_t n, std::complex<float>  *a, std::int64_t lda, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event trtri(sycl::queue &queue, oneapi::mkl::uplo uplo, oneapi::mkl::diag diag, std::int64_t n, std::complex<double> *a, std::int64_t lda, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

// gesv
DLL_EXPORT sycl::event gesv(sycl::queue &queue, std::int64_t n, std::int64_t nrhs, float *a, std::int64_t lda, std::int64_t *ipiv, float *b, std::int64_t ldb, float *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gesv(sycl::queue &queue, std::int64_t n, std::int64_t nrhs, double *a, std::int64_t lda, std::int64_t *ipiv, double *b, std::int64_t ldb, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gesv(sycl::queue &queue, std::int64_t n, std::int64_t nrhs, std::complex<float> *a, std::int64_t lda, std::int64_t *ipiv, std::complex<float> *b, std::int64_t ldb, std::complex<float> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gesv(sycl::queue &queue, std::int64_t n, std::int64_t nrhs, std::complex<double> *a, std::int64_t lda, std::int64_t *ipiv, std::complex<double> *b, std::int64_t ldb, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

// gebrd: USM API
DLL_EXPORT sycl::event gebrd(sycl::queue &queue, int64_t m, int64_t n, std::complex<float> *a, int64_t lda, float *d,
                             float *e, std::complex<float> *tauq, std::complex<float> *taup,
                             std::complex<float> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event gebrd(sycl::queue &queue, int64_t m, int64_t n, double *a, int64_t lda, double *d, double *e,
                             double *tauq, double *taup, double *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event gebrd(sycl::queue &queue, int64_t m, int64_t n, float *a, int64_t lda, float *d, float *e,
                             float *tauq, float *taup, float *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event gebrd(sycl::queue &queue, int64_t m, int64_t n, std::complex<double> *a, int64_t lda, double *d,
                             double *e, std::complex<double> *tauq, std::complex<double> *taup,
                             std::complex<double> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// gebrd: Buffer API
DLL_EXPORT void gebrd(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<float>> &a, int64_t lda,
                      sycl::buffer<float> &d, sycl::buffer<float> &e, sycl::buffer<std::complex<float>> &tauq,
                      sycl::buffer<std::complex<float>> &taup, sycl::buffer<std::complex<float>> &scratchpad,
                      int64_t scratchpad_size);
DLL_EXPORT void gebrd(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<double> &a, int64_t lda,
                      sycl::buffer<double> &d, sycl::buffer<double> &e, sycl::buffer<double> &tauq,
                      sycl::buffer<double> &taup, sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void gebrd(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<float> &a, int64_t lda,
                      sycl::buffer<float> &d, sycl::buffer<float> &e, sycl::buffer<float> &tauq,
                      sycl::buffer<float> &taup, sycl::buffer<float> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void gebrd(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<double>> &a, int64_t lda,
                      sycl::buffer<double> &d, sycl::buffer<double> &e, sycl::buffer<std::complex<double>> &tauq,
                      sycl::buffer<std::complex<double>> &taup, sycl::buffer<std::complex<double>> &scratchpad,
                      int64_t scratchpad_size);

// geqrf: USM API
DLL_EXPORT sycl::event geqrf(sycl::queue &queue, int64_t m, int64_t n, std::complex<float> *a, int64_t lda,
                                 std::complex<float> *tau, std::complex<float> *scratchpad, int64_t scratchpad_size,
                                 const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event geqrf(sycl::queue &queue, int64_t m, int64_t n, double *a, int64_t lda, double *tau,
                                 double *scratchpad, int64_t scratchpad_size,
                                 const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event geqrf(sycl::queue &queue, int64_t m, int64_t n, float *a, int64_t lda, float *tau,
                                 float *scratchpad, int64_t scratchpad_size,
                                 const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event geqrf(sycl::queue &queue, int64_t m, int64_t n, std::complex<double> *a, int64_t lda,
                                 std::complex<double> *tau, std::complex<double> *scratchpad, int64_t scratchpad_size,
                                 const std::vector<sycl::event> &event_list = {});

// geqrf: Buffer API
DLL_EXPORT void geqrf(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<float>> &a,
                      int64_t lda, sycl::buffer<std::complex<float>> &tau,
                      sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void geqrf(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<double> &a, int64_t lda,
                      sycl::buffer<double> &tau, sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void geqrf(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<float> &a, int64_t lda,
                      sycl::buffer<float> &tau, sycl::buffer<float> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void geqrf(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<double>> &a,
                      int64_t lda, sycl::buffer<std::complex<double>> &tau,
                      sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// geqrf_batch_strided: USM API
DLL_EXPORT sycl::event geqrf_batch(sycl::queue &queue, int64_t m, int64_t n, std::complex<float> *a, int64_t lda,
                                   int64_t stride_a, std::complex<float> *tau, int64_t stride_tau, int64_t batch_size,
                                   std::complex<float> *scratchpad, int64_t scratchpad_size,
                                   const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event geqrf_batch(sycl::queue &queue, int64_t m, int64_t n, double *a, int64_t lda, int64_t stride_a,
                                   double *tau, int64_t stride_tau, int64_t batch_size, double *scratchpad,
                                   int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event geqrf_batch(sycl::queue &queue, int64_t m, int64_t n, float *a, int64_t lda, int64_t stride_a,
                                   float *tau, int64_t stride_tau, int64_t batch_size, float *scratchpad,
                                   int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event geqrf_batch(sycl::queue &queue, int64_t m, int64_t n, std::complex<double> *a, int64_t lda,
                                   int64_t stride_a, std::complex<double> *tau, int64_t stride_tau, int64_t batch_size,
                                   std::complex<double> *scratchpad, int64_t scratchpad_size,
                                   const std::vector<sycl::event> &event_list = {});

// geqrf_batch_strided: Buffer API
DLL_EXPORT void geqrf_batch(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<float>> &a, int64_t lda,
                            int64_t stride_a, sycl::buffer<std::complex<float>> &tau, int64_t stride_tau,
                            int64_t batch_size, sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void geqrf_batch(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<double> &a, int64_t lda,
                            int64_t stride_a, sycl::buffer<double> &tau, int64_t stride_tau, int64_t batch_size,
                            sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void geqrf_batch(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<float> &a, int64_t lda,
                            int64_t stride_a, sycl::buffer<float> &tau, int64_t stride_tau, int64_t batch_size,
                            sycl::buffer<float> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void geqrf_batch(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<double>> &a,
                            int64_t lda, int64_t stride_a, sycl::buffer<std::complex<double>> &tau, int64_t stride_tau,
                            int64_t batch_size, sycl::buffer<std::complex<double>> &scratchpad,
                            int64_t scratchpad_size);

// gesvd_cmplx: USM API
DLL_EXPORT sycl::event gesvd(sycl::queue &queue, oneapi::mkl::jobsvd jobu, oneapi::mkl::jobsvd jobvt, int64_t m,
                             int64_t n, std::complex<float> *a, int64_t lda, float *s, std::complex<float> *u,
                             int64_t ldu, std::complex<float> *vt, int64_t ldvt, std::complex<float> *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event gesvd(sycl::queue &queue, oneapi::mkl::jobsvd jobu, oneapi::mkl::jobsvd jobvt, int64_t m,
                             int64_t n, std::complex<double> *a, int64_t lda, double *s, std::complex<double> *u,
                             int64_t ldu, std::complex<double> *vt, int64_t ldvt, std::complex<double> *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});

// gesvd_cmplx: Buffer API
DLL_EXPORT void gesvd(sycl::queue &queue, oneapi::mkl::jobsvd jobu, oneapi::mkl::jobsvd jobvt, int64_t m, int64_t n,
                      sycl::buffer<std::complex<float>> &a, int64_t lda, sycl::buffer<float> &s,
                      sycl::buffer<std::complex<float>> &u, int64_t ldu, sycl::buffer<std::complex<float>> &vt,
                      int64_t ldvt, sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void gesvd(sycl::queue &queue, oneapi::mkl::jobsvd jobu, oneapi::mkl::jobsvd jobvt, int64_t m, int64_t n,
                      sycl::buffer<std::complex<double>> &a, int64_t lda, sycl::buffer<double> &s,
                      sycl::buffer<std::complex<double>> &u, int64_t ldu, sycl::buffer<std::complex<double>> &vt,
                      int64_t ldvt, sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// gesvd_real: USM API
DLL_EXPORT sycl::event gesvd(sycl::queue &queue, oneapi::mkl::jobsvd jobu, oneapi::mkl::jobsvd jobvt, int64_t m,
                             int64_t n, double *a, int64_t lda, double *s, double *u, int64_t ldu, double *vt,
                             int64_t ldvt, double *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event gesvd(sycl::queue &queue, oneapi::mkl::jobsvd jobu, oneapi::mkl::jobsvd jobvt, int64_t m,
                             int64_t n, float *a, int64_t lda, float *s, float *u, int64_t ldu, float *vt, int64_t ldvt,
                             float *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// gesvd_real: Buffer API
DLL_EXPORT void gesvd(sycl::queue &queue, oneapi::mkl::jobsvd jobu, oneapi::mkl::jobsvd jobvt, int64_t m, int64_t n,
                      sycl::buffer<double> &a, int64_t lda, sycl::buffer<double> &s, sycl::buffer<double> &u,
                      int64_t ldu, sycl::buffer<double> &vt, int64_t ldvt, sycl::buffer<double> &scratchpad,
                      int64_t scratchpad_size);
DLL_EXPORT void gesvd(sycl::queue &queue, oneapi::mkl::jobsvd jobu, oneapi::mkl::jobsvd jobvt, int64_t m, int64_t n,
                      sycl::buffer<float> &a, int64_t lda, sycl::buffer<float> &s, sycl::buffer<float> &u, int64_t ldu,
                      sycl::buffer<float> &vt, int64_t ldvt, sycl::buffer<float> &scratchpad, int64_t scratchpad_size);


// gesvda_batch_strided: USM API
DLL_EXPORT sycl::event gesvda_batch(sycl::queue &queue, int64_t *iparm, int64_t *irank, int64_t m, int64_t n,
                                    std::complex<float> *a, int64_t lda, int64_t stride_a, float *s, int64_t stride_s,
                                    std::complex<float> *u, int64_t ldu, int64_t stride_u, std::complex<float> *vt,
                                    int64_t ldvt, int64_t stride_vt, float tolerance, float *residual,
                                    int64_t batch_size, std::complex<float> *scratchpad, int64_t scratchpad_size,
                                    const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event gesvda_batch(sycl::queue &queue, int64_t *iparm, int64_t *irank, int64_t m, int64_t n, double *a,
                                    int64_t lda, int64_t stride_a, double *s, int64_t stride_s, double *u, int64_t ldu,
                                    int64_t stride_u, double *vt, int64_t ldvt, int64_t stride_vt, double tolerance,
                                    double *residual, int64_t batch_size, double *scratchpad, int64_t scratchpad_size,
                                    const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event gesvda_batch(sycl::queue &queue, int64_t *iparm, int64_t *irank, int64_t m, int64_t n, float *a,
                                    int64_t lda, int64_t stride_a, float *s, int64_t stride_s, float *u, int64_t ldu,
                                    int64_t stride_u, float *vt, int64_t ldvt, int64_t stride_vt, float tolerance,
                                    float *residual, int64_t batch_size, float *scratchpad, int64_t scratchpad_size,
                                    const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event gesvda_batch(sycl::queue &queue, int64_t *iparm, int64_t *irank, int64_t m, int64_t n,
                                    std::complex<double> *a, int64_t lda, int64_t stride_a, double *s, int64_t stride_s,
                                    std::complex<double> *u, int64_t ldu, int64_t stride_u, std::complex<double> *vt,
                                    int64_t ldvt, int64_t stride_vt, double tolerance, double *residual,
                                    int64_t batch_size, std::complex<double> *scratchpad, int64_t scratchpad_size,
                                    const std::vector<sycl::event> &event_list = {});

// gesvda_batch_strided: Buffer API
DLL_EXPORT void gesvda_batch(sycl::queue &queue, sycl::buffer<int64_t> &iparm, sycl::buffer<int64_t> &irank, int64_t m,
                             int64_t n, sycl::buffer<std::complex<float>> &a, int64_t lda, int64_t stride_a,
                             sycl::buffer<float> &s, int64_t stride_s, sycl::buffer<std::complex<float>> &u,
                             int64_t ldu, int64_t stride_u, sycl::buffer<std::complex<float>> &vt, int64_t ldvt,
                             int64_t stride_vt, float tolerance, sycl::buffer<float> &residual, int64_t batch_size,
                             sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void gesvda_batch(sycl::queue &queue, sycl::buffer<int64_t> &iparm, sycl::buffer<int64_t> &irank, int64_t m,
                             int64_t n, sycl::buffer<double> &a, int64_t lda, int64_t stride_a, sycl::buffer<double> &s,
                             int64_t stride_s, sycl::buffer<double> &u, int64_t ldu, int64_t stride_u,
                             sycl::buffer<double> &vt, int64_t ldvt, int64_t stride_vt, double tolerance,
                             sycl::buffer<double> &residual, int64_t batch_size, sycl::buffer<double> &scratchpad,
                             int64_t scratchpad_size);
DLL_EXPORT void gesvda_batch(sycl::queue &queue, sycl::buffer<int64_t> &iparm, sycl::buffer<int64_t> &irank, int64_t m,
                             int64_t n, sycl::buffer<float> &a, int64_t lda, int64_t stride_a, sycl::buffer<float> &s,
                             int64_t stride_s, sycl::buffer<float> &u, int64_t ldu, int64_t stride_u,
                             sycl::buffer<float> &vt, int64_t ldvt, int64_t stride_vt, float tolerance,
                             sycl::buffer<float> &residual, int64_t batch_size, sycl::buffer<float> &scratchpad,
                             int64_t scratchpad_size);
DLL_EXPORT void gesvda_batch(sycl::queue &queue, sycl::buffer<int64_t> &iparm, sycl::buffer<int64_t> &irank, int64_t m,
                             int64_t n, sycl::buffer<std::complex<double>> &a, int64_t lda, int64_t stride_a,
                             sycl::buffer<double> &s, int64_t stride_s, sycl::buffer<std::complex<double>> &u,
                             int64_t ldu, int64_t stride_u, sycl::buffer<std::complex<double>> &vt, int64_t ldvt,
                             int64_t stride_vt, double tolerance, sycl::buffer<double> &residual, int64_t batch_size,
                             sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// getrf: USM API
DLL_EXPORT sycl::event getrf(sycl::queue &queue, int64_t m, int64_t n, std::complex<float> *a, int64_t lda,
                             int64_t *ipiv, std::complex<float> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrf(sycl::queue &queue, int64_t m, int64_t n, double *a, int64_t lda, int64_t *ipiv,
                             double *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrf(sycl::queue &queue, int64_t m, int64_t n, float *a, int64_t lda, int64_t *ipiv,
                             float *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrf(sycl::queue &queue, int64_t m, int64_t n, std::complex<double> *a, int64_t lda,
                             int64_t *ipiv, std::complex<double> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// getrf: Buffer API
DLL_EXPORT void getrf(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<float>> &a, int64_t lda,
                      sycl::buffer<int64_t> &ipiv, sycl::buffer<std::complex<float>> &scratchpad,
                      int64_t scratchpad_size);
DLL_EXPORT void getrf(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<double> &a, int64_t lda,
                      sycl::buffer<int64_t> &ipiv, sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrf(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<float> &a, int64_t lda,
                      sycl::buffer<int64_t> &ipiv, sycl::buffer<float> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrf(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<double>> &a, int64_t lda,
                      sycl::buffer<int64_t> &ipiv, sycl::buffer<std::complex<double>> &scratchpad,
                      int64_t scratchpad_size);

// getrf_batch_group: USM API
DLL_EXPORT sycl::event getrf_batch(sycl::queue &queue, int64_t *m, int64_t *n, std::complex<float> **a, int64_t *lda,
                                   int64_t **ipiv, int64_t group_count, int64_t *group_sizes,
                                   std::complex<float> *scratchpad, int64_t scratchpad_size,
                                   const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrf_batch(sycl::queue &queue, int64_t *m, int64_t *n, double **a, int64_t *lda, int64_t **ipiv,
                                   int64_t group_count, int64_t *group_sizes, double *scratchpad,
                                   int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrf_batch(sycl::queue &queue, int64_t *m, int64_t *n, float **a, int64_t *lda, int64_t **ipiv,
                                   int64_t group_count, int64_t *group_sizes, float *scratchpad,
                                   int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrf_batch(sycl::queue &queue, int64_t *m, int64_t *n, std::complex<double> **a, int64_t *lda,
                                   int64_t **ipiv, int64_t group_count, int64_t *group_sizes,
                                   std::complex<double> *scratchpad, int64_t scratchpad_size,
                                   const std::vector<sycl::event> &event_list = {});

// getrf_batch_strided: USM API
DLL_EXPORT sycl::event getrf_batch(sycl::queue &queue, int64_t m, int64_t n, std::complex<float> *a, int64_t lda,
                                   int64_t stride_a, int64_t *ipiv, int64_t stride_ipiv, int64_t batch_size,
                                   std::complex<float> *scratchpad, int64_t scratchpad_size,
                                   const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrf_batch(sycl::queue &queue, int64_t m, int64_t n, double *a, int64_t lda, int64_t stride_a,
                                   int64_t *ipiv, int64_t stride_ipiv, int64_t batch_size, double *scratchpad,
                                   int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrf_batch(sycl::queue &queue, int64_t m, int64_t n, float *a, int64_t lda, int64_t stride_a,
                                   int64_t *ipiv, int64_t stride_ipiv, int64_t batch_size, float *scratchpad,
                                   int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrf_batch(sycl::queue &queue, int64_t m, int64_t n, std::complex<double> *a, int64_t lda,
                                   int64_t stride_a, int64_t *ipiv, int64_t stride_ipiv, int64_t batch_size,
                                   std::complex<double> *scratchpad, int64_t scratchpad_size,
                                   const std::vector<sycl::event> &event_list = {});

// getrf_batch_strided: Buffer API
DLL_EXPORT void getrf_batch(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<float>> &a, int64_t lda,
                            int64_t stride_a, sycl::buffer<int64_t> &ipiv, int64_t stride_ipiv, int64_t batch_size,
                            sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrf_batch(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<double> &a, int64_t lda,
                            int64_t stride_a, sycl::buffer<int64_t> &ipiv, int64_t stride_ipiv, int64_t batch_size,
                            sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrf_batch(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<float> &a, int64_t lda,
                            int64_t stride_a, sycl::buffer<int64_t> &ipiv, int64_t stride_ipiv, int64_t batch_size,
                            sycl::buffer<float> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrf_batch(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<double>> &a,
                            int64_t lda, int64_t stride_a, sycl::buffer<int64_t> &ipiv, int64_t stride_ipiv,
                            int64_t batch_size, sycl::buffer<std::complex<double>> &scratchpad,
                            int64_t scratchpad_size);

// getrfnp: USM API
DLL_EXPORT sycl::event getrfnp(sycl::queue &queue, int64_t m, int64_t n, std::complex<float> *a, int64_t lda,
                               std::complex<float> *scratchpad, int64_t scratchpad_size,
                               const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrfnp(sycl::queue &queue, int64_t m, int64_t n, double *a, int64_t lda, double *scratchpad,
                               int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrfnp(sycl::queue &queue, int64_t m, int64_t n, float *a, int64_t lda, float *scratchpad,
                               int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrfnp(sycl::queue &queue, int64_t m, int64_t n, std::complex<double> *a, int64_t lda,
                               std::complex<double> *scratchpad, int64_t scratchpad_size,
                               const std::vector<sycl::event> &event_list = {});

// getrfnp: Buffer API
DLL_EXPORT void getrfnp(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<float>> &a, int64_t lda,
                        sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrfnp(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<double> &a, int64_t lda,
                        sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrfnp(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<float> &a, int64_t lda,
                        sycl::buffer<float> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrfnp(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<double>> &a, int64_t lda,
                        sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);


// getrfnp_batch_group: USM API
DLL_EXPORT sycl::event getrfnp_batch(sycl::queue &queue, int64_t *m, int64_t *n, std::complex<float> **a, int64_t *lda,
                                     int64_t group_count, int64_t *group_sizes, std::complex<float> *scratchpad,
                                     int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrfnp_batch(sycl::queue &queue, int64_t *m, int64_t *n, double **a, int64_t *lda,
                                     int64_t group_count, int64_t *group_sizes, double *scratchpad,
                                     int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrfnp_batch(sycl::queue &queue, int64_t *m, int64_t *n, float **a, int64_t *lda,
                                     int64_t group_count, int64_t *group_sizes, float *scratchpad,
                                     int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrfnp_batch(sycl::queue &queue, int64_t *m, int64_t *n, std::complex<double> **a, int64_t *lda,
                                     int64_t group_count, int64_t *group_sizes, std::complex<double> *scratchpad,
                                     int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});

// getrfnp_batch_strided: USM API
DLL_EXPORT sycl::event getrfnp_batch(sycl::queue &queue, int64_t m, int64_t n, std::complex<float> *a, int64_t lda,
                                     int64_t stride_a, int64_t batch_size, std::complex<float> *scratchpad,
                                     int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrfnp_batch(sycl::queue &queue, int64_t m, int64_t n, double *a, int64_t lda, int64_t stride_a,
                                     int64_t batch_size, double *scratchpad, int64_t scratchpad_size,
                                     const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrfnp_batch(sycl::queue &queue, int64_t m, int64_t n, float *a, int64_t lda, int64_t stride_a,
                                     int64_t batch_size, float *scratchpad, int64_t scratchpad_size,
                                     const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrfnp_batch(sycl::queue &queue, int64_t m, int64_t n, std::complex<double> *a, int64_t lda,
                                     int64_t stride_a, int64_t batch_size, std::complex<double> *scratchpad,
                                     int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});

// getrfnp_batch_strided: Buffer API
DLL_EXPORT void getrfnp_batch(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<float>> &a,
                              int64_t lda, int64_t stride_a, int64_t batch_size,
                              sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrfnp_batch(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<double> &a, int64_t lda,
                              int64_t stride_a, int64_t batch_size, sycl::buffer<double> &scratchpad,
                              int64_t scratchpad_size);
DLL_EXPORT void getrfnp_batch(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<float> &a, int64_t lda,
                              int64_t stride_a, int64_t batch_size, sycl::buffer<float> &scratchpad,
                              int64_t scratchpad_size);
DLL_EXPORT void getrfnp_batch(sycl::queue &queue, int64_t m, int64_t n, sycl::buffer<std::complex<double>> &a,
                              int64_t lda, int64_t stride_a, int64_t batch_size,
                              sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// getri: USM API
DLL_EXPORT sycl::event getri(sycl::queue &queue, int64_t n, std::complex<float> *a, int64_t lda, const int64_t *ipiv,
                             std::complex<float> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getri(sycl::queue &queue, int64_t n, double *a, int64_t lda, const int64_t *ipiv, double *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getri(sycl::queue &queue, int64_t n, float *a, int64_t lda, const int64_t *ipiv, float *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getri(sycl::queue &queue, int64_t n, std::complex<double> *a, int64_t lda, const int64_t *ipiv,
                             std::complex<double> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// getri: Buffer API
DLL_EXPORT void getri(sycl::queue &queue, int64_t n, sycl::buffer<std::complex<float>> &a, int64_t lda,
                      sycl::buffer<int64_t> &ipiv, sycl::buffer<std::complex<float>> &scratchpad,
                      int64_t scratchpad_size);
DLL_EXPORT void getri(sycl::queue &queue, int64_t n, sycl::buffer<double> &a, int64_t lda, sycl::buffer<int64_t> &ipiv,
                      sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getri(sycl::queue &queue, int64_t n, sycl::buffer<float> &a, int64_t lda, sycl::buffer<int64_t> &ipiv,
                      sycl::buffer<float> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getri(sycl::queue &queue, int64_t n, sycl::buffer<std::complex<double>> &a, int64_t lda,
                      sycl::buffer<int64_t> &ipiv, sycl::buffer<std::complex<double>> &scratchpad,
                      int64_t scratchpad_size);

// getri_oop_batch_strided: USM API
DLL_EXPORT sycl::event getri_batch(sycl::queue &queue, int64_t n, std::complex<float> *a, int64_t lda,
                                       int64_t stride_a, const int64_t *ipiv, int64_t stride_ipiv, std::complex<float> *ainv,
                                       int64_t ldainv, int64_t stride_ainv, int64_t batch_size,
                                       std::complex<float> *scratchpad, int64_t scratchpad_size,
                                       const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getri_batch(sycl::queue &queue, int64_t n, double *a, int64_t lda, int64_t stride_a,
                                       const int64_t *ipiv, int64_t stride_ipiv, double *ainv, int64_t ldainv,
                                       int64_t stride_ainv, int64_t batch_size, double *scratchpad,
                                       int64_t scratchpad_size,
                                       const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getri_batch(sycl::queue &queue, int64_t n, float *a, int64_t lda, int64_t stride_a,
                                       const int64_t *ipiv, int64_t stride_ipiv, float *ainv, int64_t ldainv,
                                       int64_t stride_ainv, int64_t batch_size, float *scratchpad,
                                       int64_t scratchpad_size,
                                       const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getri_batch(sycl::queue &queue, int64_t n, std::complex<double> *a, int64_t lda,
                                       int64_t stride_a, const int64_t *ipiv, int64_t stride_ipiv, std::complex<double> *ainv,
                                       int64_t ldainv, int64_t stride_ainv, int64_t batch_size,
                                       std::complex<double> *scratchpad, int64_t scratchpad_size,
                                       const std::vector<sycl::event> &event_list = {});

// getri_oop_batch_strided: Buffer API
DLL_EXPORT void getri_batch(sycl::queue &queue, int64_t n, sycl::buffer<std::complex<float>> &a, int64_t lda,
                            int64_t stride_a, sycl::buffer<int64_t> &ipiv, int64_t stride_ipiv,
                            sycl::buffer<std::complex<float>> &ainv, int64_t ldainv, int64_t stride_ainv,
                            int64_t batch_size, sycl::buffer<std::complex<float>> &scratchpad,
                            int64_t scratchpad_size);
DLL_EXPORT void getri_batch(sycl::queue &queue, int64_t n, sycl::buffer<double> &a, int64_t lda,
                            int64_t stride_a, sycl::buffer<int64_t> &ipiv, int64_t stride_ipiv,
                            sycl::buffer<double> &ainv, int64_t ldainv, int64_t stride_ainv, int64_t batch_size,
                            sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getri_batch(sycl::queue &queue, int64_t n, sycl::buffer<float> &a, int64_t lda,
                            int64_t stride_a, sycl::buffer<int64_t> &ipiv, int64_t stride_ipiv,
                            sycl::buffer<float> &ainv, int64_t ldainv, int64_t stride_ainv, int64_t batch_size,
                            sycl::buffer<float> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getri_batch(sycl::queue &queue, int64_t n, sycl::buffer<std::complex<double>> &a, int64_t lda,
                            int64_t stride_a, sycl::buffer<int64_t> &ipiv, int64_t stride_ipiv,
                            sycl::buffer<std::complex<double>> &ainv, int64_t ldainv, int64_t stride_ainv,
                            int64_t batch_size, sycl::buffer<std::complex<double>> &scratchpad,
                            int64_t scratchpad_size);

// getrs: USM API
DLL_EXPORT sycl::event getrs(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                             const std::complex<float> *a, int64_t lda, const int64_t *ipiv, std::complex<float> *b,
                             int64_t ldb, std::complex<float> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrs(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs, const double *a,
                             int64_t lda, const int64_t *ipiv, double *b, int64_t ldb, double *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrs(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs, const float *a,
                             int64_t lda, const int64_t *ipiv, float *b, int64_t ldb, float *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrs(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                             const std::complex<double> *a, int64_t lda, const int64_t *ipiv, std::complex<double> *b,
                             int64_t ldb, std::complex<double> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// getrs: Buffer API
DLL_EXPORT void getrs(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                      sycl::buffer<std::complex<float>> &a, int64_t lda, sycl::buffer<int64_t> &ipiv,
                      sycl::buffer<std::complex<float>> &b, int64_t ldb, sycl::buffer<std::complex<float>> &scratchpad,
                      int64_t scratchpad_size);
DLL_EXPORT void getrs(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                      sycl::buffer<double> &a, int64_t lda, sycl::buffer<int64_t> &ipiv, sycl::buffer<double> &b,
                      int64_t ldb, sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrs(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs, sycl::buffer<float> &a,
                      int64_t lda, sycl::buffer<int64_t> &ipiv, sycl::buffer<float> &b, int64_t ldb,
                      sycl::buffer<float> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrs(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                      sycl::buffer<std::complex<double>> &a, int64_t lda, sycl::buffer<int64_t> &ipiv,
                      sycl::buffer<std::complex<double>> &b, int64_t ldb,
                      sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// getrs_batch_strided: USM API
DLL_EXPORT sycl::event getrs_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                                       const std::complex<float> *a, int64_t lda, int64_t stride_a, const int64_t *ipiv,
                                       int64_t stride_ipiv, std::complex<float> *b, int64_t ldb, int64_t stride_b,
                                       int64_t batch_size, std::complex<float> *scratchpad, int64_t scratchpad_size,
                                       const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrs_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                                       const double *a, int64_t lda, int64_t stride_a, const int64_t *ipiv, int64_t stride_ipiv,
                                       double *b, int64_t ldb, int64_t stride_b, int64_t batch_size, double *scratchpad,
                                       int64_t scratchpad_size,
                                       const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrs_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                                       const float *a, int64_t lda, int64_t stride_a, const int64_t *ipiv, int64_t stride_ipiv,
                                       float *b, int64_t ldb, int64_t stride_b, int64_t batch_size, float *scratchpad,
                                       int64_t scratchpad_size,
                                       const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrs_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                                       const std::complex<double> *a, int64_t lda, int64_t stride_a, const int64_t *ipiv,
                                       int64_t stride_ipiv, std::complex<double> *b, int64_t ldb, int64_t stride_b,
                                       int64_t batch_size, std::complex<double> *scratchpad, int64_t scratchpad_size,
                                       const std::vector<sycl::event> &event_list = {});

// getrs_batch_strided: Buffer API
DLL_EXPORT void getrs_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                            sycl::buffer<std::complex<float>> &a, int64_t lda, int64_t stride_a,
                            sycl::buffer<int64_t> &ipiv, int64_t stride_ipiv,
                            sycl::buffer<std::complex<float>> &b, int64_t ldb, int64_t stride_b, int64_t batch_size,
                            sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrs_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                            sycl::buffer<double> &a, int64_t lda, int64_t stride_a, sycl::buffer<int64_t> &ipiv,
                            int64_t stride_ipiv, sycl::buffer<double> &b, int64_t ldb, int64_t stride_b,
                            int64_t batch_size, sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrs_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                            sycl::buffer<float> &a, int64_t lda, int64_t stride_a, sycl::buffer<int64_t> &ipiv,
                            int64_t stride_ipiv, sycl::buffer<float> &b, int64_t ldb, int64_t stride_b,
                            int64_t batch_size, sycl::buffer<float> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrs_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                            sycl::buffer<std::complex<double>> &a, int64_t lda, int64_t stride_a,
                            sycl::buffer<int64_t> &ipiv, int64_t stride_ipiv,
                            sycl::buffer<std::complex<double>> &b, int64_t ldb, int64_t stride_b,
                            int64_t batch_size, sycl::buffer<std::complex<double>> &scratchpad,
                            int64_t scratchpad_size);

// getrsnp_batch_strided: USM API
DLL_EXPORT sycl::event getrsnp_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                                     const std::complex<float> *a, int64_t lda, int64_t stride_a, std::complex<float> *b,
                                     int64_t ldb, int64_t stride_b, int64_t batch_size, std::complex<float> *scratchpad,
                                     int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrsnp_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                                     const double *a, int64_t lda, int64_t stride_a, double *b, int64_t ldb, int64_t stride_b,
                                     int64_t batch_size, double *scratchpad, int64_t scratchpad_size,
                                     const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrsnp_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                                     const float *a, int64_t lda, int64_t stride_a, float *b, int64_t ldb, int64_t stride_b,
                                     int64_t batch_size, float *scratchpad, int64_t scratchpad_size,
                                     const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event getrsnp_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                                     const std::complex<double> *a, int64_t lda, int64_t stride_a, std::complex<double> *b,
                                     int64_t ldb, int64_t stride_b, int64_t batch_size,
                                     std::complex<double> *scratchpad, int64_t scratchpad_size,
                                     const std::vector<sycl::event> &event_list = {});

// getrsnp_batch_strided: Buffer API
DLL_EXPORT void getrsnp_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                              sycl::buffer<std::complex<float>> &a, int64_t lda, int64_t stride_a,
                              sycl::buffer<std::complex<float>> &b, int64_t ldb, int64_t stride_b, int64_t batch_size,
                              sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void getrsnp_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                              sycl::buffer<double> &a, int64_t lda, int64_t stride_a, sycl::buffer<double> &b,
                              int64_t ldb, int64_t stride_b, int64_t batch_size, sycl::buffer<double> &scratchpad,
                              int64_t scratchpad_size);
DLL_EXPORT void getrsnp_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                              sycl::buffer<float> &a, int64_t lda, int64_t stride_a, sycl::buffer<float> &b,
                              int64_t ldb, int64_t stride_b, int64_t batch_size, sycl::buffer<float> &scratchpad,
                              int64_t scratchpad_size);
DLL_EXPORT void getrsnp_batch(sycl::queue &queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs,
                              sycl::buffer<std::complex<double>> &a, int64_t lda, int64_t stride_a,
                              sycl::buffer<std::complex<double>> &b, int64_t ldb, int64_t stride_b, int64_t batch_size,
                              sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// heev: USM API
DLL_EXPORT sycl::event heev(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::uplo uplo, int64_t n,
                            std::complex<float> *a, int64_t lda, float *w, std::complex<float> *scratchpad,
                            int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event heev(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::uplo uplo, int64_t n,
                            std::complex<double> *a, int64_t lda, double *w, std::complex<double> *scratchpad,
                            int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});

// heev: Buffer API
DLL_EXPORT void heev(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::uplo uplo, int64_t n,
                     sycl::buffer<std::complex<float>> &a, int64_t lda, sycl::buffer<float> &w,
                     sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void heev(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::uplo uplo, int64_t n,
                     sycl::buffer<std::complex<double>> &a, int64_t lda, sycl::buffer<double> &w,
                     sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// heevd: USM API
DLL_EXPORT sycl::event heevd(sycl::queue &queue, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n,
                             std::complex<float> *a, int64_t lda, float *w, std::complex<float> *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event heevd(sycl::queue &queue, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n,
                             std::complex<double> *a, int64_t lda, double *w, std::complex<double> *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});

// heevd: Buffer API
DLL_EXPORT void heevd(sycl::queue &queue, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n,
                      sycl::buffer<std::complex<float>> &a, int64_t lda, sycl::buffer<float> &w,
                      sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void heevd(sycl::queue &queue, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n,
                      sycl::buffer<std::complex<double>> &a, int64_t lda, sycl::buffer<double> &w,
                      sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// heevx: USM API
DLL_EXPORT sycl::event heevx(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::rangev range,
                             oneapi::mkl::uplo uplo, int64_t n, std::complex<float> *a, int64_t lda, float vl, float vu,
                             int64_t il, int64_t iu, float abstol, int64_t *m, float *w, std::complex<float> *z,
                             int64_t ldz, std::complex<float> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event heevx(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::rangev range,
                             oneapi::mkl::uplo uplo, int64_t n, std::complex<double> *a, int64_t lda, double vl,
                             double vu, int64_t il, int64_t iu, double abstol, int64_t *m, double *w,
                             std::complex<double> *z, int64_t ldz, std::complex<double> *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});

// heevx: Buffer API
DLL_EXPORT void heevx(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::rangev range, oneapi::mkl::uplo uplo,
                      int64_t n, sycl::buffer<std::complex<float>> &a, int64_t lda, float vl, float vu, int64_t il,
                      int64_t iu, float abstol, sycl::buffer<int64_t> &m, sycl::buffer<float> &w,
                      sycl::buffer<std::complex<float>> &z, int64_t ldz, sycl::buffer<std::complex<float>> &scratchpad,
                      int64_t scratchpad_size);
DLL_EXPORT void heevx(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::rangev range, oneapi::mkl::uplo uplo,
                      int64_t n, sycl::buffer<std::complex<double>> &a, int64_t lda, double vl, double vu, int64_t il,
                      int64_t iu, double abstol, sycl::buffer<int64_t> &m, sycl::buffer<double> &w,
                      sycl::buffer<std::complex<double>> &z, int64_t ldz,
                      sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// hegvd: USM API
DLL_EXPORT sycl::event hegvd(sycl::queue &queue, int64_t itype, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo,
                             int64_t n, std::complex<float> *a, int64_t lda, std::complex<float> *b, int64_t ldb,
                             float *w, std::complex<float> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event hegvd(sycl::queue &queue, int64_t itype, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo,
                             int64_t n, std::complex<double> *a, int64_t lda, std::complex<double> *b, int64_t ldb,
                             double *w, std::complex<double> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// hegvd: Buffer API
DLL_EXPORT void hegvd(sycl::queue &queue, int64_t itype, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n,
                      sycl::buffer<std::complex<float>> &a, int64_t lda, sycl::buffer<std::complex<float>> &b,
                      int64_t ldb, sycl::buffer<float> &w, sycl::buffer<std::complex<float>> &scratchpad,
                      int64_t scratchpad_size);
DLL_EXPORT void hegvd(sycl::queue &queue, int64_t itype, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n,
                      sycl::buffer<std::complex<double>> &a, int64_t lda, sycl::buffer<std::complex<double>> &b,
                      int64_t ldb, sycl::buffer<double> &w, sycl::buffer<std::complex<double>> &scratchpad,
                      int64_t scratchpad_size);

// hegvx: USM API
DLL_EXPORT sycl::event hegvx(sycl::queue &queue, int64_t itype, oneapi::mkl::compz jobz, oneapi::mkl::rangev range,
                             oneapi::mkl::uplo uplo, int64_t n, std::complex<float> *a, int64_t lda,
                             std::complex<float> *b, int64_t ldb, float vl, float vu, int64_t il, int64_t iu,
                             float abstol, int64_t *m, float *w, std::complex<float> *z, int64_t ldz,
                             std::complex<float> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event hegvx(sycl::queue &queue, int64_t itype, oneapi::mkl::compz jobz, oneapi::mkl::rangev range,
                             oneapi::mkl::uplo uplo, int64_t n, std::complex<double> *a, int64_t lda,
                             std::complex<double> *b, int64_t ldb, double vl, double vu, int64_t il, int64_t iu,
                             double abstol, int64_t *m, double *w, std::complex<double> *z, int64_t ldz,
                             std::complex<double> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// hegvx: Buffer API
DLL_EXPORT void hegvx(sycl::queue &queue, int64_t itype, oneapi::mkl::compz jobz, oneapi::mkl::rangev range,
                      oneapi::mkl::uplo uplo, int64_t n, sycl::buffer<std::complex<float>> &a, int64_t lda,
                      sycl::buffer<std::complex<float>> &b, int64_t ldb, float vl, float vu, int64_t il, int64_t iu,
                      float abstol, sycl::buffer<int64_t> &m, sycl::buffer<float> &w,
                      sycl::buffer<std::complex<float>> &z, int64_t ldz, sycl::buffer<std::complex<float>> &scratchpad,
                      int64_t scratchpad_size);
DLL_EXPORT void hegvx(sycl::queue &queue, int64_t itype, oneapi::mkl::compz jobz, oneapi::mkl::rangev range,
                      oneapi::mkl::uplo uplo, int64_t n, sycl::buffer<std::complex<double>> &a, int64_t lda,
                      sycl::buffer<std::complex<double>> &b, int64_t ldb, double vl, double vu, int64_t il, int64_t iu,
                      double abstol, sycl::buffer<int64_t> &m, sycl::buffer<double> &w,
                      sycl::buffer<std::complex<double>> &z, int64_t ldz,
                      sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// hetrd: USM API
DLL_EXPORT sycl::event hetrd(sycl::queue &queue, oneapi::mkl::uplo uplo, int64_t n, std::complex<float> *a, int64_t lda,
                             float *d, float *e, std::complex<float> *tau, std::complex<float> *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event hetrd(sycl::queue &queue, oneapi::mkl::uplo uplo, int64_t n, std::complex<double> *a,
                             int64_t lda, double *d, double *e, std::complex<double> *tau,
                             std::complex<double> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// hetrd: Buffer API
DLL_EXPORT void hetrd(sycl::queue &queue, oneapi::mkl::uplo uplo, int64_t n, sycl::buffer<std::complex<float>> &a,
                      int64_t lda, sycl::buffer<float> &d, sycl::buffer<float> &e,
                      sycl::buffer<std::complex<float>> &tau, sycl::buffer<std::complex<float>> &scratchpad,
                      int64_t scratchpad_size);
DLL_EXPORT void hetrd(sycl::queue &queue, oneapi::mkl::uplo uplo, int64_t n, sycl::buffer<std::complex<double>> &a,
                      int64_t lda, sycl::buffer<double> &d, sycl::buffer<double> &e,
                      sycl::buffer<std::complex<double>> &tau, sycl::buffer<std::complex<double>> &scratchpad,
                      int64_t scratchpad_size);

// hetrf
DLL_EXPORT sycl::event hetrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::complex<float>  *a, std::int64_t lda, std::int64_t *ipiv, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event hetrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::complex<double> *a, std::int64_t lda, std::int64_t *ipiv, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void hetrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, sycl::buffer<std::int64_t> &ipiv, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void hetrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<std::complex<double>> &a, std::int64_t lda, sycl::buffer<std::int64_t> &ipiv, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

// orgbr
DLL_EXPORT sycl::event orgbr(sycl::queue &queue, oneapi::mkl::generate vec, std::int64_t m, std::int64_t n, std::int64_t k, float  *a, std::int64_t lda, const float  *tau, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event orgbr(sycl::queue &queue, oneapi::mkl::generate vec, std::int64_t m, std::int64_t n, std::int64_t k, double *a, std::int64_t lda, const double *tau, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void orgbr(sycl::queue &queue, oneapi::mkl::generate vec, std::int64_t m, std::int64_t n, std::int64_t k, sycl::buffer<float>  &a, std::int64_t lda, sycl::buffer<float>  &tau, sycl::buffer<float>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void orgbr(sycl::queue &queue, oneapi::mkl::generate vec, std::int64_t m, std::int64_t n, std::int64_t k, sycl::buffer<double> &a, std::int64_t lda, sycl::buffer<double> &tau, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);

// orgqr: USM API
DLL_EXPORT sycl::event orgqr(sycl::queue &queue, int64_t m, int64_t n, int64_t k, double *a, int64_t lda,
                             const double *tau, double *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event orgqr(sycl::queue &queue, int64_t m, int64_t n, int64_t k, float *a, int64_t lda,
                             const float *tau, float *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// orgqr: Buffer API
DLL_EXPORT void orgqr(sycl::queue &queue, int64_t m, int64_t n, int64_t k, sycl::buffer<double> &a, int64_t lda,
                      sycl::buffer<double> &tau, sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void orgqr(sycl::queue &queue, int64_t m, int64_t n, int64_t k, sycl::buffer<float> &a, int64_t lda,
                      sycl::buffer<float> &tau, sycl::buffer<float> &scratchpad, int64_t scratchpad_size);

// ormqr: USM API
DLL_EXPORT sycl::event ormqr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, int64_t m,
                             int64_t n, int64_t k, const double *a, int64_t lda, const double *tau, double *c, int64_t ldc,
                             double *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event ormqr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, int64_t m,
                             int64_t n, int64_t k, const float *a, int64_t lda, const float *tau, float *c, int64_t ldc,
                             float *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// ormqr: Buffer API
DLL_EXPORT void ormqr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, int64_t m, int64_t n,
                      int64_t k, sycl::buffer<double> &a, int64_t lda, sycl::buffer<double> &tau,
                      sycl::buffer<double> &c, int64_t ldc, sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void ormqr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, int64_t m, int64_t n,
                      int64_t k, sycl::buffer<float> &a, int64_t lda, sycl::buffer<float> &tau, sycl::buffer<float> &c,
                      int64_t ldc, sycl::buffer<float> &scratchpad, int64_t scratchpad_size);

// steqr: USM API
DLL_EXPORT sycl::event steqr(sycl::queue &queue, oneapi::mkl::compz compz, int64_t n, float *d, float *e,
                             std::complex<float> *z, int64_t ldz, std::complex<float> *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event steqr(sycl::queue &queue, oneapi::mkl::compz compz, int64_t n, double *d, double *e, double *z,
                             int64_t ldz, double *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event steqr(sycl::queue &queue, oneapi::mkl::compz compz, int64_t n, float *d, float *e, float *z,
                             int64_t ldz, float *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event steqr(sycl::queue &queue, oneapi::mkl::compz compz, int64_t n, double *d, double *e,
                             std::complex<double> *z, int64_t ldz, std::complex<double> *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});

// steqr: Buffer API
DLL_EXPORT void steqr(sycl::queue &queue, oneapi::mkl::compz compz, int64_t n, sycl::buffer<float> &d,
                      sycl::buffer<float> &e, sycl::buffer<std::complex<float>> &z, int64_t ldz,
                      sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void steqr(sycl::queue &queue, oneapi::mkl::compz compz, int64_t n, sycl::buffer<double> &d,
                      sycl::buffer<double> &e, sycl::buffer<double> &z, int64_t ldz, sycl::buffer<double> &scratchpad,
                      int64_t scratchpad_size);
DLL_EXPORT void steqr(sycl::queue &queue, oneapi::mkl::compz compz, int64_t n, sycl::buffer<float> &d,
                      sycl::buffer<float> &e, sycl::buffer<float> &z, int64_t ldz, sycl::buffer<float> &scratchpad,
                      int64_t scratchpad_size);
DLL_EXPORT void steqr(sycl::queue &queue, oneapi::mkl::compz compz, int64_t n, sycl::buffer<double> &d,
                      sycl::buffer<double> &e, sycl::buffer<std::complex<double>> &z, int64_t ldz,
                      sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// syev: USM API
DLL_EXPORT sycl::event syev(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::uplo uplo, int64_t n, double *a,
                            int64_t lda, double *w, double *scratchpad, int64_t scratchpad_size,
                            const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event syev(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::uplo uplo, int64_t n, float *a,
                            int64_t lda, float *w, float *scratchpad, int64_t scratchpad_size,
                            const std::vector<sycl::event> &event_list = {});

// syev: Buffer API
DLL_EXPORT void syev(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::uplo uplo, int64_t n,
                     sycl::buffer<double> &a, int64_t lda, sycl::buffer<double> &w, sycl::buffer<double> &scratchpad,
                     int64_t scratchpad_size);
DLL_EXPORT void syev(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::uplo uplo, int64_t n,
                     sycl::buffer<float> &a, int64_t lda, sycl::buffer<float> &w, sycl::buffer<float> &scratchpad,
                     int64_t scratchpad_size);

// syevd: USM API
DLL_EXPORT sycl::event syevd(sycl::queue &queue, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n, double *a,
                             int64_t lda, double *w, double *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event syevd(sycl::queue &queue, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n, float *a,
                             int64_t lda, float *w, float *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// syevd: Buffer API
DLL_EXPORT void syevd(sycl::queue &queue, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n,
                      sycl::buffer<double> &a, int64_t lda, sycl::buffer<double> &w, sycl::buffer<double> &scratchpad,
                      int64_t scratchpad_size);
DLL_EXPORT void syevd(sycl::queue &queue, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n,
                      sycl::buffer<float> &a, int64_t lda, sycl::buffer<float> &w, sycl::buffer<float> &scratchpad,
                      int64_t scratchpad_size);

// syevx: USM API
DLL_EXPORT sycl::event syevx(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::rangev range,
                             oneapi::mkl::uplo uplo, int64_t n, double *a, int64_t lda, double vl, double vu,
                             int64_t il, int64_t iu, double abstol, int64_t *m, double *w, double *z, int64_t ldz,
                             double *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event syevx(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::rangev range,
                             oneapi::mkl::uplo uplo, int64_t n, float *a, int64_t lda, float vl, float vu, int64_t il,
                             int64_t iu, float abstol, int64_t *m, float *w, float *z, int64_t ldz, float *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});

// syevx: Buffer API
DLL_EXPORT void syevx(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::rangev range, oneapi::mkl::uplo uplo,
                      int64_t n, sycl::buffer<double> &a, int64_t lda, double vl, double vu, int64_t il, int64_t iu,
                      double abstol, sycl::buffer<int64_t> &m, sycl::buffer<double> &w, sycl::buffer<double> &z,
                      int64_t ldz, sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void syevx(sycl::queue &queue, oneapi::mkl::compz jobz, oneapi::mkl::rangev range, oneapi::mkl::uplo uplo,
                      int64_t n, sycl::buffer<float> &a, int64_t lda, float vl, float vu, int64_t il, int64_t iu,
                      float abstol, sycl::buffer<int64_t> &m, sycl::buffer<float> &w, sycl::buffer<float> &z,
                      int64_t ldz, sycl::buffer<float> &scratchpad, int64_t scratchpad_size);

// sygvd: USM API
DLL_EXPORT sycl::event sygvd(sycl::queue &queue, int64_t itype, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo,
                             int64_t n, double *a, int64_t lda, double *b, int64_t ldb, double *w, double *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event sygvd(sycl::queue &queue, int64_t itype, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo,
                             int64_t n, float *a, int64_t lda, float *b, int64_t ldb, float *w, float *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});

// sygvd: Buffer API
DLL_EXPORT void sygvd(sycl::queue &queue, int64_t itype, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n,
                      sycl::buffer<double> &a, int64_t lda, sycl::buffer<double> &b, int64_t ldb,
                      sycl::buffer<double> &w, sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void sygvd(sycl::queue &queue, int64_t itype, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n,
                      sycl::buffer<float> &a, int64_t lda, sycl::buffer<float> &b, int64_t ldb, sycl::buffer<float> &w,
                      sycl::buffer<float> &scratchpad, int64_t scratchpad_size);

// sygvx: USM API
DLL_EXPORT sycl::event sygvx(sycl::queue &queue, int64_t itype, oneapi::mkl::compz jobz, oneapi::mkl::rangev range,
                             oneapi::mkl::uplo uplo, int64_t n, double *a, int64_t lda, double *b, int64_t ldb,
                             double vl, double vu, int64_t il, int64_t iu, double abstol, int64_t *m, double *w,
                             double *z, int64_t ldz, double *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event sygvx(sycl::queue &queue, int64_t itype, oneapi::mkl::compz jobz, oneapi::mkl::rangev range,
                             oneapi::mkl::uplo uplo, int64_t n, float *a, int64_t lda, float *b, int64_t ldb, float vl,
                             float vu, int64_t il, int64_t iu, float abstol, int64_t *m, float *w, float *z,
                             int64_t ldz, float *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// sygvx: Buffer API
DLL_EXPORT void sygvx(sycl::queue &queue, int64_t itype, oneapi::mkl::compz jobz, oneapi::mkl::rangev range,
                      oneapi::mkl::uplo uplo, int64_t n, sycl::buffer<double> &a, int64_t lda, sycl::buffer<double> &b,
                      int64_t ldb, double vl, double vu, int64_t il, int64_t iu, double abstol,
                      sycl::buffer<int64_t> &m, sycl::buffer<double> &w, sycl::buffer<double> &z, int64_t ldz,
                      sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void sygvx(sycl::queue &queue, int64_t itype, oneapi::mkl::compz jobz, oneapi::mkl::rangev range,
                      oneapi::mkl::uplo uplo, int64_t n, sycl::buffer<float> &a, int64_t lda, sycl::buffer<float> &b,
                      int64_t ldb, float vl, float vu, int64_t il, int64_t iu, float abstol, sycl::buffer<int64_t> &m,
                      sycl::buffer<float> &w, sycl::buffer<float> &z, int64_t ldz, sycl::buffer<float> &scratchpad,
                      int64_t scratchpad_size);

// sytrd: USM API
DLL_EXPORT sycl::event sytrd(sycl::queue &queue, oneapi::mkl::uplo uplo, int64_t n, double *a, int64_t lda, double *d,
                             double *e, double *tau, double *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event sytrd(sycl::queue &queue, oneapi::mkl::uplo uplo, int64_t n, float *a, int64_t lda, float *d,
                             float *e, float *tau, float *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// sytrd: Buffer API
DLL_EXPORT void sytrd(sycl::queue &queue, oneapi::mkl::uplo uplo, int64_t n, sycl::buffer<double> &a, int64_t lda,
                      sycl::buffer<double> &d, sycl::buffer<double> &e, sycl::buffer<double> &tau,
                      sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void sytrd(sycl::queue &queue, oneapi::mkl::uplo uplo, int64_t n, sycl::buffer<float> &a, int64_t lda,
                      sycl::buffer<float> &d, sycl::buffer<float> &e, sycl::buffer<float> &tau,
                      sycl::buffer<float> &scratchpad, int64_t scratchpad_size);

// trtrs: USM API
DLL_EXPORT sycl::event trtrs(sycl::queue &queue, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans,
                             oneapi::mkl::diag diag, int64_t n, int64_t nrhs, const std::complex<float> *a, int64_t lda,
                             std::complex<float> *b, int64_t ldb, std::complex<float> *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event trtrs(sycl::queue &queue, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans,
                             oneapi::mkl::diag diag, int64_t n, int64_t nrhs, const double *a, int64_t lda, double *b,
                             int64_t ldb, double *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event trtrs(sycl::queue &queue, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans,
                             oneapi::mkl::diag diag, int64_t n, int64_t nrhs, const float *a, int64_t lda, float *b,
                             int64_t ldb, float *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event trtrs(sycl::queue &queue, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans,
                             oneapi::mkl::diag diag, int64_t n, int64_t nrhs, const std::complex<double> *a, int64_t lda,
                             std::complex<double> *b, int64_t ldb, std::complex<double> *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});

// trtrs: Buffer API
DLL_EXPORT void trtrs(sycl::queue &queue, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, oneapi::mkl::diag diag,
                      int64_t n, int64_t nrhs, sycl::buffer<std::complex<float>> &a, int64_t lda,
                      sycl::buffer<std::complex<float>> &b, int64_t ldb, sycl::buffer<std::complex<float>> &scratchpad,
                      int64_t scratchpad_size);
DLL_EXPORT void trtrs(sycl::queue &queue, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, oneapi::mkl::diag diag,
                      int64_t n, int64_t nrhs, sycl::buffer<double> &a, int64_t lda, sycl::buffer<double> &b,
                      int64_t ldb, sycl::buffer<double> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void trtrs(sycl::queue &queue, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, oneapi::mkl::diag diag,
                      int64_t n, int64_t nrhs, sycl::buffer<float> &a, int64_t lda, sycl::buffer<float> &b, int64_t ldb,
                      sycl::buffer<float> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void trtrs(sycl::queue &queue, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, oneapi::mkl::diag diag,
                      int64_t n, int64_t nrhs, sycl::buffer<std::complex<double>> &a, int64_t lda,
                      sycl::buffer<std::complex<double>> &b, int64_t ldb,
                      sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// ungbr
DLL_EXPORT sycl::event ungbr(sycl::queue &queue, oneapi::mkl::generate vec, std::int64_t m, std::int64_t n, std::int64_t k, std::complex<float>  *a, std::int64_t lda, const std::complex<float>  *tau, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event ungbr(sycl::queue &queue, oneapi::mkl::generate vec, std::int64_t m, std::int64_t n, std::int64_t k, std::complex<double> *a, std::int64_t lda, const std::complex<double> *tau, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void ungbr(sycl::queue &queue, oneapi::mkl::generate vec, std::int64_t m, std::int64_t n, std::int64_t k, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, sycl::buffer<std::complex<float>>  &tau, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void ungbr(sycl::queue &queue, oneapi::mkl::generate vec, std::int64_t m, std::int64_t n, std::int64_t k, sycl::buffer<std::complex<double>> &a, std::int64_t lda, sycl::buffer<std::complex<double>> &tau, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

// ungqr: USM API
DLL_EXPORT sycl::event ungqr(sycl::queue &queue, int64_t m, int64_t n, int64_t k, std::complex<float> *a, int64_t lda,
                             const std::complex<float> *tau, std::complex<float> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event ungqr(sycl::queue &queue, int64_t m, int64_t n, int64_t k, std::complex<double> *a, int64_t lda,
                             const std::complex<double> *tau, std::complex<double> *scratchpad, int64_t scratchpad_size,
                             const std::vector<sycl::event> &event_list = {});

// ungqr: Buffer API
DLL_EXPORT void ungqr(sycl::queue &queue, int64_t m, int64_t n, int64_t k, sycl::buffer<std::complex<float>> &a,
                      int64_t lda, sycl::buffer<std::complex<float>> &tau,
                      sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void ungqr(sycl::queue &queue, int64_t m, int64_t n, int64_t k, sycl::buffer<std::complex<double>> &a,
                      int64_t lda, sycl::buffer<std::complex<double>> &tau,
                      sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// unmqr: USM API
DLL_EXPORT sycl::event unmqr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, int64_t m,
                             int64_t n, int64_t k, const std::complex<float> *a, int64_t lda, const std::complex<float> *tau,
                             std::complex<float> *c, int64_t ldc, std::complex<float> *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});
DLL_EXPORT sycl::event unmqr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, int64_t m,
                             int64_t n, int64_t k, const std::complex<double> *a, int64_t lda, const std::complex<double> *tau,
                             std::complex<double> *c, int64_t ldc, std::complex<double> *scratchpad,
                             int64_t scratchpad_size, const std::vector<sycl::event> &event_list = {});

// unmqr: Buffer API
DLL_EXPORT void unmqr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, int64_t m, int64_t n,
                      int64_t k, sycl::buffer<std::complex<float>> &a, int64_t lda,
                      sycl::buffer<std::complex<float>> &tau, sycl::buffer<std::complex<float>> &c, int64_t ldc,
                      sycl::buffer<std::complex<float>> &scratchpad, int64_t scratchpad_size);
DLL_EXPORT void unmqr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, int64_t m, int64_t n,
                      int64_t k, sycl::buffer<std::complex<double>> &a, int64_t lda,
                      sycl::buffer<std::complex<double>> &tau, sycl::buffer<std::complex<double>> &c, int64_t ldc,
                      sycl::buffer<std::complex<double>> &scratchpad, int64_t scratchpad_size);

// gerqf
DLL_EXPORT sycl::event gerqf(sycl::queue &queue, std::int64_t m, std::int64_t n, float  *a, std::int64_t lda, float  *tau, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gerqf(sycl::queue &queue, std::int64_t m, std::int64_t n, double *a, std::int64_t lda, double *tau, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gerqf(sycl::queue &queue, std::int64_t m, std::int64_t n, std::complex<float>  *a, std::int64_t lda, std::complex<float>  *tau, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gerqf(sycl::queue &queue, std::int64_t m, std::int64_t n, std::complex<double> *a, std::int64_t lda, std::complex<double> *tau, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void gerqf(sycl::queue &queue, std::int64_t m, std::int64_t n, sycl::buffer<float>  &a, std::int64_t lda, sycl::buffer<float>  &tau, sycl::buffer<float>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void gerqf(sycl::queue &queue, std::int64_t m, std::int64_t n, sycl::buffer<double> &a, std::int64_t lda, sycl::buffer<double> &tau, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void gerqf(sycl::queue &queue, std::int64_t m, std::int64_t n, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, sycl::buffer<std::complex<float>>  &tau, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void gerqf(sycl::queue &queue, std::int64_t m, std::int64_t n, sycl::buffer<std::complex<double>> &a, std::int64_t lda, sycl::buffer<std::complex<double>> &tau, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

// ormrq
DLL_EXPORT sycl::event ormrq(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t k, const float  *a, std::int64_t lda, const float  *tau, float  *c, std::int64_t ldc, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event ormrq(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t k, const double *a, std::int64_t lda, const double *tau, double *c, std::int64_t ldc, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void ormrq(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t k, sycl::buffer<float>  &a, std::int64_t lda, sycl::buffer<float>  &tau, sycl::buffer<float>  &c, std::int64_t ldc, sycl::buffer<float>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void ormrq(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t k, sycl::buffer<double> &a, std::int64_t lda, sycl::buffer<double> &tau, sycl::buffer<double> &c, std::int64_t ldc, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);

// unmrq
DLL_EXPORT sycl::event unmrq(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t k, const std::complex<float>  *a, std::int64_t lda, const std::complex<float>  *tau, std::complex<float>  *c, std::int64_t ldc, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event unmrq(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t k, const std::complex<double> *a, std::int64_t lda, const std::complex<double> *tau, std::complex<double> *c, std::int64_t ldc, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void unmrq(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t k, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, sycl::buffer<std::complex<float>>  &tau, sycl::buffer<std::complex<float>>  &c, std::int64_t ldc, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void unmrq(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t k, sycl::buffer<std::complex<double>> &a, std::int64_t lda, sycl::buffer<std::complex<double>> &tau, sycl::buffer<std::complex<double>> &c, std::int64_t ldc, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

// sytrf
DLL_EXPORT sycl::event sytrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, float  *a, std::int64_t lda, std::int64_t *ipiv, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event sytrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, double *a, std::int64_t lda, std::int64_t *ipiv, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event sytrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::complex<float>  *a, std::int64_t lda, std::int64_t *ipiv, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event sytrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::complex<double> *a, std::int64_t lda, std::int64_t *ipiv, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void sytrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<float>  &a, std::int64_t lda, sycl::buffer<std::int64_t> &ipiv, sycl::buffer<float>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void sytrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<double> &a, std::int64_t lda, sycl::buffer<std::int64_t> &ipiv, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void sytrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, sycl::buffer<std::int64_t> &ipiv, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void sytrf(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<std::complex<double>> &a, std::int64_t lda, sycl::buffer<std::int64_t> &ipiv, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

// orgtr
DLL_EXPORT sycl::event orgtr(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, float  *a, std::int64_t lda, const float  *tau, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event orgtr(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, double *a, std::int64_t lda, const double *tau, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void orgtr(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<float>  &a, std::int64_t lda, sycl::buffer<float>  &tau, sycl::buffer<float>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void orgtr(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<double> &a, std::int64_t lda, sycl::buffer<double> &tau, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);

// ungtr
DLL_EXPORT sycl::event ungtr(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::complex<float>  *a, std::int64_t lda, const std::complex<float>  *tau, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event ungtr(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::complex<double> *a, std::int64_t lda, const std::complex<double> *tau, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void ungtr(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, sycl::buffer<std::complex<float>>  &tau, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void ungtr(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<std::complex<double>> &a, std::int64_t lda, sycl::buffer<std::complex<double>> &tau, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

// ormtr
DLL_EXPORT sycl::event ormtr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, const float  *a, std::int64_t lda, const float  *tau, float  *c, std::int64_t ldc, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event ormtr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, const double *a, std::int64_t lda, const double *tau, double *c, std::int64_t ldc, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void ormtr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, sycl::buffer<float>  &a, std::int64_t lda, sycl::buffer<float>  &tau, sycl::buffer<float>  &c, std::int64_t ldc, sycl::buffer<float>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void ormtr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, sycl::buffer<double> &a, std::int64_t lda, sycl::buffer<double> &tau, sycl::buffer<double> &c, std::int64_t ldc, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);

// unmtr
DLL_EXPORT sycl::event unmtr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, const std::complex<float>  *a, std::int64_t lda, const std::complex<float>  *tau, std::complex<float>  *c, std::int64_t ldc, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event unmtr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, const std::complex<double> *a, std::int64_t lda, const std::complex<double> *tau, std::complex<double> *c, std::int64_t ldc, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void unmtr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, sycl::buffer<std::complex<float>>  &tau, sycl::buffer<std::complex<float>>  &c, std::int64_t ldc, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void unmtr(sycl::queue &queue, oneapi::mkl::side side, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, sycl::buffer<std::complex<double>> &a, std::int64_t lda, sycl::buffer<std::complex<double>> &tau, sycl::buffer<std::complex<double>> &c, std::int64_t ldc, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

// gels
DLL_EXPORT sycl::event gels(sycl::queue &queue, oneapi::mkl::transpose trans, std::int64_t m,  std::int64_t n, std::int64_t nrhs, float *a, std::int64_t lda, float *b, std::int64_t ldb, float *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gels(sycl::queue &queue, oneapi::mkl::transpose trans, std::int64_t m,  std::int64_t n, std::int64_t nrhs, double *a, std::int64_t lda, double *b, std::int64_t ldb, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gels(sycl::queue &queue, oneapi::mkl::transpose trans, std::int64_t m,  std::int64_t n, std::int64_t nrhs, std::complex<float> *a, std::int64_t lda, std::complex<float> *b, std::int64_t ldb, std::complex<float> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gels(sycl::queue &queue, oneapi::mkl::transpose trans, std::int64_t m,  std::int64_t n, std::int64_t nrhs, std::complex<double> *a, std::int64_t lda, std::complex<double> *b, std::int64_t ldb, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

//
// SYCL oneMKL LAPACK batch group API
//

DLL_EXPORT sycl::event potrf_batch(sycl::queue &queue, oneapi::mkl::uplo *uplo, std::int64_t *n, float  **a, std::int64_t *lda, std::int64_t group_count, std::int64_t *group_sizes, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrf_batch(sycl::queue &queue, oneapi::mkl::uplo *uplo, std::int64_t *n, double **a, std::int64_t *lda, std::int64_t group_count, std::int64_t *group_sizes, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrf_batch(sycl::queue &queue, oneapi::mkl::uplo *uplo, std::int64_t *n, std::complex<float>  **a, std::int64_t *lda, std::int64_t group_count, std::int64_t *group_sizes, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrf_batch(sycl::queue &queue, oneapi::mkl::uplo *uplo, std::int64_t *n, std::complex<double> **a, std::int64_t *lda, std::int64_t group_count, std::int64_t *group_sizes, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

DLL_EXPORT sycl::event potrs_batch(sycl::queue &queue, oneapi::mkl::uplo *uplo, std::int64_t *n, std::int64_t *nrhs, const float  * const* a, std::int64_t *lda, float  **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrs_batch(sycl::queue &queue, oneapi::mkl::uplo *uplo, std::int64_t *n, std::int64_t *nrhs, const double * const* a, std::int64_t *lda, double **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrs_batch(sycl::queue &queue, oneapi::mkl::uplo *uplo, std::int64_t *n, std::int64_t *nrhs, const std::complex<float>  * const* a, std::int64_t *lda, std::complex<float>  **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrs_batch(sycl::queue &queue, oneapi::mkl::uplo *uplo, std::int64_t *n, std::int64_t *nrhs, const std::complex<double> * const* a, std::int64_t *lda, std::complex<double> **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

DLL_EXPORT sycl::event geinv_batch(sycl::queue &queue, std::int64_t *n, float **a, std::int64_t *lda, std::int64_t group_count, std::int64_t *group_sizes, float *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event geinv_batch(sycl::queue &queue, std::int64_t *n, double **a, std::int64_t *lda, std::int64_t group_count, std::int64_t *group_sizes, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event geinv_batch(sycl::queue &queue, std::int64_t *n, std::complex<float> **a, std::int64_t *lda, std::int64_t group_count, std::int64_t *group_sizes, std::complex<float> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event geinv_batch(sycl::queue &queue, std::int64_t *n, std::complex<double> **a, std::int64_t *lda, std::int64_t group_count, std::int64_t *group_sizes, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

DLL_EXPORT sycl::event getrs_batch(sycl::queue &queue, oneapi::mkl::transpose *trans, std::int64_t *n, std::int64_t *nrhs, const float  * const* a, std::int64_t *lda, const std::int64_t* const* ipiv, float  **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event getrs_batch(sycl::queue &queue, oneapi::mkl::transpose *trans, std::int64_t *n, std::int64_t *nrhs, const double * const* a, std::int64_t *lda, const std::int64_t* const* ipiv, double **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event getrs_batch(sycl::queue &queue, oneapi::mkl::transpose *trans, std::int64_t *n, std::int64_t *nrhs, const std::complex<float>  * const* a, std::int64_t *lda, const std::int64_t* const* ipiv, std::complex<float>  **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event getrs_batch(sycl::queue &queue, oneapi::mkl::transpose *trans, std::int64_t *n, std::int64_t *nrhs, const std::complex<double> * const* a, std::int64_t *lda, const std::int64_t* const* ipiv, std::complex<double> **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

DLL_EXPORT sycl::event getri_batch(sycl::queue &queue, std::int64_t *n, float  **a, std::int64_t *lda, const std::int64_t* const* ipiv, std::int64_t group_count, std::int64_t *group_sizes, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event getri_batch(sycl::queue &queue, std::int64_t *n, double **a, std::int64_t *lda, const std::int64_t* const* ipiv, std::int64_t group_count, std::int64_t *group_sizes, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event getri_batch(sycl::queue &queue, std::int64_t *n, std::complex<float>  **a, std::int64_t *lda, const std::int64_t* const* ipiv, std::int64_t group_count, std::int64_t *group_sizes, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event getri_batch(sycl::queue &queue, std::int64_t *n, std::complex<double> **a, std::int64_t *lda, const std::int64_t* const* ipiv, std::int64_t group_count, std::int64_t *group_sizes, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

DLL_EXPORT sycl::event geqrf_batch(sycl::queue &queue, std::int64_t *m, std::int64_t *n, float  **a, std::int64_t *lda, float  **tau, std::int64_t group_count, std::int64_t *group_sizes, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event geqrf_batch(sycl::queue &queue, std::int64_t *m, std::int64_t *n, double **a, std::int64_t *lda, double **tau, std::int64_t group_count, std::int64_t *group_sizes, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event geqrf_batch(sycl::queue &queue, std::int64_t *m, std::int64_t *n, std::complex<float>  **a, std::int64_t *lda, std::complex<float>  **tau, std::int64_t group_count, std::int64_t *group_sizes, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event geqrf_batch(sycl::queue &queue, std::int64_t *m, std::int64_t *n, std::complex<double> **a, std::int64_t *lda, std::complex<double> **tau, std::int64_t group_count, std::int64_t *group_sizes, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

DLL_EXPORT sycl::event orgqr_batch(sycl::queue &queue, std::int64_t *m, std::int64_t *n, std::int64_t *k, float  **a, std::int64_t *lda, const float  * const* tau, std::int64_t group_count, std::int64_t *group_sizes, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event orgqr_batch(sycl::queue &queue, std::int64_t *m, std::int64_t *n, std::int64_t *k, double **a, std::int64_t *lda, const double * const* tau, std::int64_t group_count, std::int64_t *group_sizes, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

DLL_EXPORT sycl::event ungqr_batch(sycl::queue &queue, std::int64_t *m, std::int64_t *n, std::int64_t *k, std::complex<float>  **a, std::int64_t *lda, const std::complex<float>  * const* tau, std::int64_t group_count, std::int64_t *group_sizes, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event ungqr_batch(sycl::queue &queue, std::int64_t *m, std::int64_t *n, std::int64_t *k, std::complex<double> **a, std::int64_t *lda, const std::complex<double> * const* tau, std::int64_t group_count, std::int64_t *group_sizes, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

DLL_EXPORT sycl::event ormqr_batch(sycl::queue &queue, oneapi::mkl::side *side, oneapi::mkl::transpose *trans, std::int64_t *m, std::int64_t *n, std::int64_t *k, const float * const* a, std::int64_t *lda, const float * const* tau, float **c, std::int64_t *ldc, std::int64_t group_count, std::int64_t *group_sizes, float *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event ormqr_batch(sycl::queue &queue, oneapi::mkl::side *side, oneapi::mkl::transpose *trans, std::int64_t *m, std::int64_t *n, std::int64_t *k, const double * const* a, std::int64_t *lda, const double * const* tau, double **c, std::int64_t *ldc, std::int64_t group_count, std::int64_t *group_sizes, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

DLL_EXPORT sycl::event unmqr_batch(sycl::queue &queue, oneapi::mkl::side *side, oneapi::mkl::transpose *trans, std::int64_t *m, std::int64_t *n, std::int64_t *k, const std::complex<float> * const* a, std::int64_t *lda, const std::complex<float> * const* tau, std::complex<float> **c, std::int64_t *ldc, std::int64_t group_count, std::int64_t *group_sizes, std::complex<float> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event unmqr_batch(sycl::queue &queue, oneapi::mkl::side *side, oneapi::mkl::transpose *trans, std::int64_t *m, std::int64_t *n, std::int64_t *k, const std::complex<double> * const* a, std::int64_t *lda, const std::complex<double> * const* tau, std::complex<double> **c, std::int64_t *ldc, std::int64_t group_count, std::int64_t *group_sizes, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

DLL_EXPORT sycl::event trtrs_batch(sycl::queue &queue, oneapi::mkl::uplo *uplo, oneapi::mkl::transpose *trans, oneapi::mkl::diag *diag, std::int64_t *n, std::int64_t *nrhs, const float * const* a, std::int64_t *lda, float **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, float *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event trtrs_batch(sycl::queue &queue, oneapi::mkl::uplo *uplo, oneapi::mkl::transpose *trans, oneapi::mkl::diag *diag, std::int64_t *n, std::int64_t *nrhs, const double * const* a, std::int64_t *lda, double **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event trtrs_batch(sycl::queue &queue, oneapi::mkl::uplo *uplo, oneapi::mkl::transpose *trans, oneapi::mkl::diag *diag, std::int64_t *n, std::int64_t *nrhs, const std::complex<float> * const* a, std::int64_t *lda, std::complex<float> **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, std::complex<float> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event trtrs_batch(sycl::queue &queue, oneapi::mkl::uplo *uplo, oneapi::mkl::transpose *trans, oneapi::mkl::diag *diag, std::int64_t *n, std::int64_t *nrhs, const std::complex<double> * const* a, std::int64_t *lda, std::complex<double> **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

DLL_EXPORT sycl::event gels_batch(sycl::queue &queue, oneapi::mkl::transpose *trans, std::int64_t *m,  std::int64_t *n, std::int64_t *nrhs, float **a, std::int64_t *lda, float **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, float *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gels_batch(sycl::queue &queue, oneapi::mkl::transpose *trans, std::int64_t *m,  std::int64_t *n, std::int64_t *nrhs, double **a, std::int64_t *lda, double **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gels_batch(sycl::queue &queue, oneapi::mkl::transpose *trans, std::int64_t *m,  std::int64_t *n, std::int64_t *nrhs, std::complex<float> **a, std::int64_t *lda,std::complex<float> **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, std::complex<float> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gels_batch(sycl::queue &queue, oneapi::mkl::transpose *trans, std::int64_t *m,  std::int64_t *n, std::int64_t *nrhs, std::complex<double> **a, std::int64_t *lda, std::complex<double> **b, std::int64_t *ldb, std::int64_t group_count, std::int64_t *group_sizes, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});

//
// SYCL oneMKL LAPACK batch stride API
//

DLL_EXPORT sycl::event potrf_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, float  *a, std::int64_t lda, std::int64_t stride_a, std::int64_t batch_size, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrf_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, double *a, std::int64_t lda, std::int64_t stride_a, std::int64_t batch_size, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrf_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::complex<float>  *a, std::int64_t lda, std::int64_t stride_a, std::int64_t batch_size, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrf_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::complex<double> *a, std::int64_t lda, std::int64_t stride_a, std::int64_t batch_size, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void potrf_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<float>  &a, std::int64_t lda, std::int64_t stride_a, std::int64_t batch_size, sycl::buffer<float>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potrf_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<double> &a, std::int64_t lda, std::int64_t stride_a, std::int64_t batch_size, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potrf_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, std::int64_t stride_a, std::int64_t batch_size, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potrf_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, sycl::buffer<std::complex<double>> &a, std::int64_t lda, std::int64_t stride_a, std::int64_t batch_size, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

DLL_EXPORT sycl::event potrs_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, const float  *a, std::int64_t lda, std::int64_t stride_a, float  *b, std::int64_t ldb, std::int64_t stride_b, std::int64_t batch_size, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrs_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, const double *a, std::int64_t lda, std::int64_t stride_a, double *b, std::int64_t ldb, std::int64_t stride_b, std::int64_t batch_size, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrs_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, const std::complex<float>  *a, std::int64_t lda, std::int64_t stride_a, std::complex<float>  *b, std::int64_t ldb, std::int64_t stride_b, std::int64_t batch_size, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event potrs_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, const std::complex<double> *a, std::int64_t lda, std::int64_t stride_a, std::complex<double> *b, std::int64_t ldb, std::int64_t stride_b, std::int64_t batch_size, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void potrs_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, sycl::buffer<float>  &a, std::int64_t lda, std::int64_t stride_a, sycl::buffer<float>  &b, std::int64_t ldb, std::int64_t stride_b, std::int64_t batch_size, sycl::buffer<float>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potrs_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, sycl::buffer<double> &a, std::int64_t lda, std::int64_t stride_a, sycl::buffer<double> &b, std::int64_t ldb, std::int64_t stride_b, std::int64_t batch_size, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potrs_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, std::int64_t stride_a, sycl::buffer<std::complex<float>>  &b, std::int64_t ldb, std::int64_t stride_b, std::int64_t batch_size, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void potrs_batch(sycl::queue &queue, oneapi::mkl::uplo uplo, std::int64_t n, std::int64_t nrhs, sycl::buffer<std::complex<double>> &a, std::int64_t lda, std::int64_t stride_a, sycl::buffer<std::complex<double>> &b, std::int64_t ldb, std::int64_t stride_b, std::int64_t batch_size, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

DLL_EXPORT sycl::event orgqr_batch(sycl::queue &queue, std::int64_t m, std::int64_t n, std::int64_t k, float  *a, std::int64_t lda, std::int64_t stride_a, const float  *tau, std::int64_t stride_tau, std::int64_t batch_size, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event orgqr_batch(sycl::queue &queue, std::int64_t m, std::int64_t n, std::int64_t k, double *a, std::int64_t lda, std::int64_t stride_a, const double *tau, std::int64_t stride_tau, std::int64_t batch_size, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void orgqr_batch(sycl::queue &queue, std::int64_t m, std::int64_t n, std::int64_t k, sycl::buffer<float>  &a, std::int64_t lda, std::int64_t stride_a, sycl::buffer<float>  &tau, std::int64_t stride_tau, std::int64_t batch_size, sycl::buffer<float>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void orgqr_batch(sycl::queue &queue, std::int64_t m, std::int64_t n, std::int64_t k, sycl::buffer<double> &a, std::int64_t lda, std::int64_t stride_a, sycl::buffer<double> &tau, std::int64_t stride_tau, std::int64_t batch_size, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);

DLL_EXPORT sycl::event ungqr_batch(sycl::queue &queue, std::int64_t m, std::int64_t n, std::int64_t k, std::complex<float>  *a, std::int64_t lda, std::int64_t stride_a, const std::complex<float>  *tau, std::int64_t stride_tau, std::int64_t batch_size, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event ungqr_batch(sycl::queue &queue, std::int64_t m, std::int64_t n, std::int64_t k, std::complex<double> *a, std::int64_t lda, std::int64_t stride_a, const std::complex<double> *tau, std::int64_t stride_tau, std::int64_t batch_size, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void ungqr_batch(sycl::queue &queue, std::int64_t m, std::int64_t n, std::int64_t k, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, std::int64_t stride_a, sycl::buffer<std::complex<float>>  &tau, std::int64_t stride_tau, std::int64_t batch_size, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void ungqr_batch(sycl::queue &queue, std::int64_t m, std::int64_t n, std::int64_t k, sycl::buffer<std::complex<double>> &a, std::int64_t lda, std::int64_t stride_a, sycl::buffer<std::complex<double>> &tau, std::int64_t stride_tau, std::int64_t batch_size, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

DLL_EXPORT sycl::event getri_batch(sycl::queue &queue, std::int64_t n, float  *a, std::int64_t lda, std::int64_t stride_a, const std::int64_t *ipiv, std::int64_t stride_ipiv, std::int64_t batch_size, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event getri_batch(sycl::queue &queue, std::int64_t n, double *a, std::int64_t lda, std::int64_t stride_a, const std::int64_t *ipiv, std::int64_t stride_ipiv, std::int64_t batch_size, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event getri_batch(sycl::queue &queue, std::int64_t n, std::complex<float>  *a, std::int64_t lda, std::int64_t stride_a, const std::int64_t *ipiv, std::int64_t stride_ipiv, std::int64_t batch_size, std::complex<float>  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event getri_batch(sycl::queue &queue, std::int64_t n, std::complex<double> *a, std::int64_t lda, std::int64_t stride_a, const std::int64_t *ipiv, std::int64_t stride_ipiv, std::int64_t batch_size, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void getri_batch(sycl::queue &queue, std::int64_t n, sycl::buffer<float>  &a, std::int64_t lda, std::int64_t stride_a, sycl::buffer<std::int64_t> &ipiv, std::int64_t stride_ipiv, std::int64_t batch_size, sycl::buffer<float>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void getri_batch(sycl::queue &queue, std::int64_t n, sycl::buffer<double> &a, std::int64_t lda, std::int64_t stride_a, sycl::buffer<std::int64_t> &ipiv, std::int64_t stride_ipiv, std::int64_t batch_size, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void getri_batch(sycl::queue &queue, std::int64_t n, sycl::buffer<std::complex<float>>  &a, std::int64_t lda, std::int64_t stride_a, sycl::buffer<std::int64_t> &ipiv, std::int64_t stride_ipiv, std::int64_t batch_size, sycl::buffer<std::complex<float>>  &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void getri_batch(sycl::queue &queue, std::int64_t n, sycl::buffer<std::complex<double>> &a, std::int64_t lda, std::int64_t stride_a, sycl::buffer<std::int64_t> &ipiv, std::int64_t stride_ipiv, std::int64_t batch_size, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

DLL_EXPORT sycl::event gels_batch(sycl::queue &queue, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t nrhs, float  *_a, std::int64_t lda, std::int64_t stride_a, float  *_b, std::int64_t ldb, std::int64_t stride_b, std::int64_t batch_size, float  *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gels_batch(sycl::queue &queue, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t nrhs, double *_a, std::int64_t lda, std::int64_t stride_a, double *_b, std::int64_t ldb, std::int64_t stride_b, std::int64_t batch_size, double *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gels_batch(sycl::queue &queue, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t nrhs, std::complex<float> *_a, std::int64_t lda, std::int64_t stride_a, std::complex<float> *_b, std::int64_t ldb, std::int64_t stride_b, std::int64_t batch_size, std::complex<float> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT sycl::event gels_batch(sycl::queue &queue, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t nrhs, std::complex<double> *_a, std::int64_t lda, std::int64_t stride_a, std::complex<double> *_b, std::int64_t ldb, std::int64_t stride_b, std::int64_t batch_size, std::complex<double> *scratchpad, std::int64_t scratchpad_size, const std::vector<sycl::event> &events = {});
DLL_EXPORT void gels_batch(sycl::queue &queue, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t nrhs, sycl::buffer<float> &a, std::int64_t lda, std::int64_t stridea, sycl::buffer<float> &b, std::int64_t ldb, std::int64_t strideb, std::int64_t batchsize, sycl::buffer<float> &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void gels_batch(sycl::queue &queue, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t nrhs, sycl::buffer<double> &a, std::int64_t lda, std::int64_t stridea, sycl::buffer<double> &b, std::int64_t ldb, std::int64_t strideb, std::int64_t batchsize, sycl::buffer<double> &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void gels_batch(sycl::queue &queue, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t nrhs, sycl::buffer<std::complex<float>> &a, std::int64_t lda, std::int64_t stridea, sycl::buffer<std::complex<float>> &b, std::int64_t ldb, std::int64_t strideb, std::int64_t batchsize, sycl::buffer<std::complex<float>> &scratchpad, std::int64_t scratchpad_size);
DLL_EXPORT void gels_batch(sycl::queue &queue, oneapi::mkl::transpose trans, std::int64_t m, std::int64_t n, std::int64_t nrhs, sycl::buffer<std::complex<double>> &a, std::int64_t lda, std::int64_t stridea, sycl::buffer<std::complex<double>> &b, std::int64_t ldb, std::int64_t strideb, std::int64_t batchsize, sycl::buffer<std::complex<double>> &scratchpad, std::int64_t scratchpad_size);

} // namespace lapack
} // namespace mkl
} // namespace oneapi

#endif  // _ONEAPI_MKL_LAPACK_LAPACK_HPP__
