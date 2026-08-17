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

#ifndef MKL_SPEC_H
#define MKL_SPEC_H

#define ONEMATH_SPEC_VERSION_NON_COMPLIANT 1
#define ONEMATH_SPEC_VERSION(MAJOR, MINOR) (MAJOR*100 + MINOR)

namespace oneapi {
namespace mkl {

// Versions of the oneMATH specification
enum class SpecVersion{
    not_compliant = ONEMATH_SPEC_VERSION_NON_COMPLIANT,
    version_1_1 = ONEMATH_SPEC_VERSION(1, 1),
    version_1_2 = ONEMATH_SPEC_VERSION(1, 2),
    version_1_3 = ONEMATH_SPEC_VERSION(1, 3),
    version_1_4 = ONEMATH_SPEC_VERSION(1, 4),
    not_released,
};

} // namespace mkl
} // namespace oneapi

#endif /* MKL_SPEC_H */
