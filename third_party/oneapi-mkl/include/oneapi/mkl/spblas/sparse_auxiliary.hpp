/*******************************************************************************
* Copyright (C) 2019 Intel Corporation
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

#ifndef _ONEMKL_SPARSE_AUXILIARY_HPP_
#define _ONEMKL_SPARSE_AUXILIARY_HPP_

#include <sycl/sycl.hpp>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "oneapi/mkl/export.hpp"
#include "oneapi/mkl/types.hpp"
#include "oneapi/mkl/exceptions.hpp"

#include "oneapi/mkl/spblas/sparse_structures.hpp"

namespace oneapi {
namespace mkl {
namespace sparse {

/******************************************************************************/
DLL_EXPORT sycl::event omatcopy(sycl::queue &queue,
                                transpose transpose_val,
                                matrix_handle_t spMat_in,
                                matrix_handle_t spMat_out,
                                const std::vector<sycl::event> &dependencies = {});

/******************************************************************************/
DLL_EXPORT void omatconvert_buffer_size(
    sycl::queue &queue,
    matrix_handle_t spMat_in,
    matrix_handle_t spMat_out,
    omatconvert_alg alg,
    omatconvert_descr_t descr,
    std::int64_t &sizeTempWorkspace);

DLL_EXPORT void omatconvert_analyze(
    sycl::queue &queue,
    matrix_handle_t spMat_in,
    matrix_handle_t spMat_out,
    omatconvert_alg alg,
    omatconvert_descr_t descr,
    sycl::buffer<std::uint8_t, 1> *tempWorkspace);

DLL_EXPORT sycl::event omatconvert_analyze(
    sycl::queue &queue,
    matrix_handle_t spMat_in,
    matrix_handle_t spMat_out,
    omatconvert_alg alg,
    omatconvert_descr_t descr,
    void *tempWorkspace,
    const std::vector<sycl::event> &dependencies = {});

DLL_EXPORT void omatconvert_get_nnz(
    sycl::queue &queue,
    matrix_handle_t spMat_in,
    matrix_handle_t spMat_out,
    omatconvert_alg alg,
    omatconvert_descr_t descr,
    std::int64_t &nnzOut,
    const std::vector<sycl::event> &dependencies = {});

DLL_EXPORT sycl::event omatconvert (
    sycl::queue &queue,
    oneapi::mkl::sparse::matrix_handle_t spMat_in,
    oneapi::mkl::sparse::matrix_handle_t spMat_out,
    omatconvert_alg alg,
    omatconvert_descr_t descr,
    const std::vector<sycl::event> &dependencies = {});

/******************************************************************************/
DLL_EXPORT sycl::event sort_matrix(sycl::queue &queue,
                                   matrix_handle_t spMat,
                                   const std::vector<sycl::event> &dependencies = {});

/******************************************************************************/
#define ONEMKL_DECLARE_SPARSE_UPDATE_DIAGONAL_VALUES(FpType) \
    DLL_EXPORT void update_diagonal_values(sycl::queue &queue, \
                                           matrix_handle_t spMat, \
                                           sycl::buffer<FpType, 1> &new_diag_values); \
    DLL_EXPORT sycl::event update_diagonal_values(sycl::queue &queue, \
                                                  matrix_handle_t spMat, \
                                                  std::int64_t length, \
                                                  const FpType *new_diag_values, \
                                                  const std::vector<sycl::event> &dependencies = {})

ONEMKL_DECLARE_SPARSE_UPDATE_DIAGONAL_VALUES(float);
ONEMKL_DECLARE_SPARSE_UPDATE_DIAGONAL_VALUES(double);
ONEMKL_DECLARE_SPARSE_UPDATE_DIAGONAL_VALUES(std::complex<float>);
ONEMKL_DECLARE_SPARSE_UPDATE_DIAGONAL_VALUES(std::complex<double>);

#undef ONEMKL_DECLARE_SPARSE_UPDATE_DIAGONAL_VALUES

} /* namespace oneapi::mkl::sparse */
} /* namespace mkl */
} // namespace oneapi

#endif // #ifndef _ONEMKL_SPARSE_AUXILIARY_HPP_
