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

#ifndef _ONEAPI_MKL_LAPACK_SCRATCHPAD_HPP__
#define _ONEAPI_MKL_LAPACK_SCRATCHPAD_HPP__

#include <complex>
#include <cstdint>
#include <sycl/sycl.hpp>

#include "oneapi/mkl/types.hpp"
#include "oneapi/mkl/lapack/concepts.hpp"

namespace oneapi {
namespace mkl {
namespace lapack {

// nonbatch queries
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   gebrd_scratchpad_size(sycl::queue& queue, int64_t m, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t    gels_scratchpad_size(sycl::queue& queue, oneapi::mkl::transpose trans, int64_t m, int64_t n, int64_t nrhs, int64_t lda, int64_t ldb);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   geqrf_scratchpad_size(sycl::queue& queue, int64_t m, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   gerqf_scratchpad_size(sycl::queue& queue, int64_t m, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t    gesv_scratchpad_size(sycl::queue& queue, int64_t n, int64_t nrhs, int64_t lda, int64_t ldb);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   gesvd_scratchpad_size(sycl::queue& queue, oneapi::mkl::jobsvd jobu, oneapi::mkl::jobsvd jobvt, int64_t m, int64_t n, int64_t lda, int64_t ldu, int64_t ldvt);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   getrf_scratchpad_size(sycl::queue& queue, int64_t m, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t getrfnp_scratchpad_size(sycl::queue& queue, int64_t m, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   getri_scratchpad_size(sycl::queue& queue, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   getrs_scratchpad_size(sycl::queue& queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs, int64_t lda, int64_t ldb);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t    heev_scratchpad_size(sycl::queue& queue, oneapi::mkl::compz jobz, oneapi::mkl::uplo uplo, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   heevd_scratchpad_size(sycl::queue& queue, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   heevx_scratchpad_size(sycl::queue& queue, oneapi::mkl::compz jobz, oneapi::mkl::rangev range, oneapi::mkl::uplo uplo, int64_t n, int64_t lda, typename fp::value_type vl, typename fp::value_type vu, int64_t il, int64_t iu, typename fp::value_type abstol, int64_t ldz);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   hegvd_scratchpad_size(sycl::queue& queue, int64_t itype, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n, int64_t lda, int64_t ldb);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   hegvx_scratchpad_size(sycl::queue& queue, int64_t itype, oneapi::mkl::compz jobz, oneapi::mkl::rangev range, oneapi::mkl::uplo uplo, int64_t n, int64_t lda, int64_t ldb, typename fp::value_type vl, typename fp::value_type vu, int64_t il, int64_t iu, typename fp::value_type abstol, int64_t ldz);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   hetrd_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo uplo, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   hetrf_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo uplo, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   orgbr_scratchpad_size(sycl::queue& queue, oneapi::mkl::generate vect, int64_t m, int64_t n, int64_t k, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   orgqr_scratchpad_size(sycl::queue& queue, int64_t m, int64_t n, int64_t k, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   orgtr_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo uplo, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   ormqr_scratchpad_size(sycl::queue& queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, int64_t m, int64_t n, int64_t k, int64_t lda, int64_t ldc);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   ormrq_scratchpad_size(sycl::queue& queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, int64_t m, int64_t n, int64_t k, int64_t lda, int64_t ldc);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   ormtr_scratchpad_size(sycl::queue& queue, oneapi::mkl::side side, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, int64_t m, int64_t n, int64_t lda, int64_t ldc);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   potrf_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo uplo, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   potri_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo uplo, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   potrs_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo uplo, int64_t n, int64_t nrhs, int64_t lda, int64_t ldb);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   steqr_scratchpad_size(sycl::queue& queue, oneapi::mkl::compz compz, int64_t n, int64_t ldz);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t    syev_scratchpad_size(sycl::queue& queue, oneapi::mkl::compz jobz, oneapi::mkl::uplo uplo, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   syevd_scratchpad_size(sycl::queue& queue, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   syevx_scratchpad_size(sycl::queue& queue, oneapi::mkl::compz jobz, oneapi::mkl::rangev range, oneapi::mkl::uplo uplo, int64_t n, int64_t lda, fp vl, fp vu, int64_t il, int64_t iu, fp abstol, int64_t ldz);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   sygvd_scratchpad_size(sycl::queue& queue, int64_t itype, oneapi::mkl::job jobz, oneapi::mkl::uplo uplo, int64_t n, int64_t lda, int64_t ldb);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   sygvx_scratchpad_size(sycl::queue& queue, int64_t itype, oneapi::mkl::compz jobz, oneapi::mkl::rangev range, oneapi::mkl::uplo uplo, int64_t n, int64_t lda, int64_t ldb, fp vl, fp vu, int64_t il, int64_t iu, fp abstol, int64_t ldz);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   sytrd_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo uplo, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   sytrf_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo uplo, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   trtri_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo uplo, oneapi::mkl::diag diag, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   trtrs_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, oneapi::mkl::diag diag, int64_t n, int64_t nrhs, int64_t lda, int64_t ldb);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   ungbr_scratchpad_size(sycl::queue& queue, oneapi::mkl::generate vect, int64_t m, int64_t n, int64_t k, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   ungqr_scratchpad_size(sycl::queue& queue, int64_t m, int64_t n, int64_t k, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   ungtr_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo uplo, int64_t n, int64_t lda);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   unmqr_scratchpad_size(sycl::queue& queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, int64_t m, int64_t n, int64_t k, int64_t lda, int64_t ldc);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   unmrq_scratchpad_size(sycl::queue& queue, oneapi::mkl::side side, oneapi::mkl::transpose trans, int64_t m, int64_t n, int64_t k, int64_t lda, int64_t ldc);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   unmtr_scratchpad_size(sycl::queue& queue, oneapi::mkl::side side, oneapi::mkl::uplo uplo, oneapi::mkl::transpose trans, int64_t m, int64_t n, int64_t lda, int64_t ldc);

// batch queries
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   geinv_batch_scratchpad_size(sycl::queue& queue, int64_t* n, int64_t* lda, int64_t group_count, int64_t* group_sizes);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t    gels_batch_scratchpad_size(sycl::queue& queue, oneapi::mkl::transpose* trans, int64_t* m, int64_t* n, int64_t* nrhs, int64_t* lda, int64_t* ldb, int64_t group_count, int64_t* group_sizes);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t    gels_batch_scratchpad_size(sycl::queue& queue, oneapi::mkl::transpose trans, int64_t m, int64_t n, int64_t nrhs, int64_t lda, int64_t stride_a, int64_t ldb, int64_t stride_b, int64_t batch_size);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   geqrf_batch_scratchpad_size(sycl::queue& queue, int64_t* m, int64_t* n, int64_t* lda, int64_t group_count, int64_t* group_sizes);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   geqrf_batch_scratchpad_size(sycl::queue& queue, int64_t m, int64_t n, int64_t lda, int64_t stride_a, int64_t stride_tau, int64_t batch_size);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t  gesvda_batch_scratchpad_size(sycl::queue& queue, int64_t m, int64_t n, int64_t lda, int64_t stride_a, int64_t stride_s, int64_t ldu, int64_t stride_u, int64_t ldvt, int64_t stride_vt, int64_t batch_size);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   getrf_batch_scratchpad_size(sycl::queue& queue, int64_t m, int64_t n, int64_t lda, int64_t stride_a, int64_t stride_ipiv, int64_t batch_size);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   getrf_batch_scratchpad_size(sycl::queue& queue, int64_t* m, int64_t* n, int64_t* lda, int64_t group_count, int64_t* group_sizes);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t getrfnp_batch_scratchpad_size(sycl::queue& queue, int64_t m, int64_t n, int64_t lda, int64_t stride_a, int64_t batch_size);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t getrfnp_batch_scratchpad_size(sycl::queue& queue, int64_t* m, int64_t* n, int64_t* lda, int64_t group_count, int64_t* group_sizes);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   getri_batch_scratchpad_size(sycl::queue& queue, int64_t* n, int64_t* lda, int64_t group_count, int64_t* group_sizes); 
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   getri_batch_scratchpad_size(sycl::queue& queue, int64_t n, int64_t lda, int64_t stride_a, int64_t stride_ipiv, int64_t batch_size);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   getri_batch_scratchpad_size(sycl::queue& queue, int64_t n, int64_t lda, int64_t stride_a, int64_t stride_ipiv, int64_t ldainv, int64_t stride_ainv, int64_t batch_size);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   getrs_batch_scratchpad_size(sycl::queue& queue, oneapi::mkl::transpose* trans, int64_t* n, int64_t* nrhs, int64_t* lda, int64_t* ldb, int64_t group_count, int64_t* group_sizes);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   getrs_batch_scratchpad_size(sycl::queue& queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs, int64_t lda, int64_t stride_a, int64_t stride_ipiv, int64_t ldb, int64_t stride_b, int64_t batch_size);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t getrsnp_batch_scratchpad_size(sycl::queue& queue, oneapi::mkl::transpose trans, int64_t n, int64_t nrhs, int64_t lda, int64_t stride_a, int64_t ldb, int64_t stride_b, int64_t batch_size);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   orgqr_batch_scratchpad_size(sycl::queue& queue, int64_t* m, int64_t* n, int64_t* k, int64_t* lda, int64_t group_count, int64_t* group_sizes);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   orgqr_batch_scratchpad_size(sycl::queue& queue, int64_t m, int64_t n, int64_t k, int64_t lda, int64_t stride_a, int64_t stride_tau, int64_t batch_size);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_real_floating_point<fp>>    int64_t   ormqr_batch_scratchpad_size(sycl::queue& queue, oneapi::mkl::side* side, oneapi::mkl::transpose* trans, int64_t* m, int64_t* n, int64_t* k, int64_t* lda, int64_t* ldc, int64_t group_count, int64_t* group_sizes);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   potrf_batch_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo* uplo, int64_t* n, int64_t* lda, int64_t group_count, int64_t* group_sizes);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   potrf_batch_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo uplo, int64_t n, int64_t lda, int64_t stride_a, int64_t batch_size);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   potrs_batch_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo* uplo, int64_t* n, int64_t* nrhs, int64_t* lda, int64_t* ldb, int64_t group_count, int64_t* group_sizes);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   potrs_batch_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo uplo, int64_t n, int64_t nrhs, int64_t lda, int64_t stride_a, int64_t ldb, int64_t stride_b, int64_t batch_size);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_floating_point<fp>>         int64_t   trtrs_batch_scratchpad_size(sycl::queue& queue, oneapi::mkl::uplo* uplo, oneapi::mkl::transpose* trans, oneapi::mkl::diag* diag, int64_t* n, int64_t* nrhs, int64_t* lda, int64_t* ldb, int64_t group_count, int64_t* group_sizes);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   ungqr_batch_scratchpad_size(sycl::queue& queue, int64_t* m, int64_t* n, int64_t* k, int64_t* lda, int64_t group_count, int64_t* group_sizes);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   ungqr_batch_scratchpad_size(sycl::queue& queue, int64_t m, int64_t n, int64_t k, int64_t lda, int64_t stride_a, int64_t stride_tau, int64_t batch_size);
template <typename fp, typename = oneapi::mkl::lapack::internal::is_complex_floating_point<fp>> int64_t   unmqr_batch_scratchpad_size(sycl::queue& queue, oneapi::mkl::side* side, oneapi::mkl::transpose* trans, int64_t* m, int64_t* n, int64_t* k, int64_t* lda, int64_t* ldc, int64_t group_count, int64_t* group_sizes);

} // namespace lapack
} // namespace mkl
} // namespace oneapi

#endif // _ONEAPI_MKL_LAPACK_SCRATCHPAD_HPP__
