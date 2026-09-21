/*******************************************************************************
* Copyright (C) 2025 Intel Corporation
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
!      oneAPI Math Kernel Library (oneMKL) SYCL interface
!******************************************************************************/

#ifndef ONEMATH_DFT_SPEC_HPP
#define ONEMATH_DFT_SPEC_HPP

#include "oneapi/mkl/spec.hpp"

#if defined(ONEMATH_SPEC_VERSION_NON_COMPLIANT)
    #define ONEMATH_DFT_SPEC_VERSION ONEMATH_SPEC_VERSION_NON_COMPLIANT
#else
    #error "ONEMATH_SPEC_VERSION_NON_COMPLIANT should be defined"
#endif

namespace oneapi {
namespace mkl {
namespace dft {

constexpr auto spec_version =
        static_cast<SpecVersion>(ONEMATH_DFT_SPEC_VERSION);

} // namespace dft
} // namespace mkl
} // namespace oneapi

#endif /* ONEMATH_DFT_SPEC_HPP */
