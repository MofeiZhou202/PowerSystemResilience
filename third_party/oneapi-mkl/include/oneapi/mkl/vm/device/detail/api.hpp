/* -== file: api.hpp ==- */
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

#ifndef ONEAPI_MKL_VM_DEVICE_DETAIL_API_HPP
#define ONEAPI_MKL_VM_DEVICE_DETAIL_API_HPP 1


#include <complex>
#include <tuple>
#include <type_traits>

#include <sycl/sycl.hpp>

#include "oneapi/mkl/vm/decls.hpp"

namespace oneapi::mkl::vm::device::detail {

enum class Function : int {
    kAbs = 0,
    kAcos = 1,
    kAcosh = 2,
    kAcospi = 3,
    kAdd = 4,
    kArg = 5,
    kAsin = 6,
    kAsinh = 7,
    kAsinpi = 8,
    kAtan = 9,
    kAtan2 = 10,
    kAtan2pi = 11,
    kAtanh = 12,
    kAtanpi = 13,
    kCbrt = 14,
    kCdfnorm = 15,
    kCdfnorminv = 16,
    kCeil = 17,
    kCis = 18,
    kConj = 19,
    kCopysign = 20,
    kCos = 21,
    kCosd = 22,
    kCosh = 23,
    kCospi = 24,
    kDiv = 25,
    kErf = 26,
    kErfc = 27,
    kErfcinv = 28,
    kErfcx = 29,
    kErfinv = 30,
    kExp = 31,
    kExp10 = 32,
    kExp2 = 33,
    kExpint1 = 34,
    kExpm1 = 35,
    kFdim = 36,
    kFloor = 37,
    kFmax = 38,
    kFmin = 39,
    kFmod = 40,
    kFrac = 41,
    kHypot = 42,
    kI0 = 43,
    kI1 = 44,
    kInv = 45,
    kInvcbrt = 46,
    kInvsqrt = 47,
    kJ0 = 48,
    kJ1 = 49,
    kJn = 50,
    kLgamma = 51,
    kLinearfrac = 52,
    kLn = 53,
    kLog10 = 54,
    kLog1p = 55,
    kLog2 = 56,
    kLogb = 57,
    kMaxmag = 58,
    kMinmag = 59,
    kModf = 60,
    kMul = 61,
    kMulbyconj = 62,
    kNearbyint = 63,
    kNextafter = 64,
    kPow = 65,
    kPow2o3 = 66,
    kPow3o2 = 67,
    kPowr = 68,
    kPowx = 69,
    kRemainder = 70,
    kRint = 71,
    kRound = 72,
    kSin = 73,
    kSincos = 74,
    kSincospi = 75,
    kSind = 76,
    kSinh = 77,
    kSinpi = 78,
    kSqr = 79,
    kSqrt = 80,
    kSub = 81,
    kTan = 82,
    kTand = 83,
    kTanh = 84,
    kTanpi = 85,
    kTgamma = 86,
    kTrunc = 87,
    kY0 = 88,
    kY1 = 89,
    kYn = 90,
}; // enum class Function : in

enum class Accuracy : int {
    kNA = -1,
    kNS = 0,
    kEP = 1,
    kLA = 2,
    kHA = 3,
}; // enum class Accuracy : in

enum class Feature : int {
    kNS = -1,
    kNA = 0,
    kGE = 1,
    kGECast32 = 2,
    kTA = 3,
    kTACast32 = 4,
    kSO = 5,
    kDR = 6,
    kSOCast32 = 7,
    kDRCast32 = 8,
}; // enum class Feature : int

struct  AccuracyEP {
    static constexpr Accuracy value = Accuracy::kEP;
}; // struct  AccuracyEP

struct  AccuracyLA {
    static constexpr Accuracy value = Accuracy::kLA;
}; // struct  AccuracyLA

struct  AccuracyHA {
    static constexpr Accuracy value = Accuracy::kHA;
}; // struct  AccuracyHA

namespace mode {

constexpr AccuracyEP ep;
constexpr AccuracyLA la;
constexpr AccuracyHA ha;
constexpr AccuracyHA not_defined;

} // mode
constexpr AccuracyHA DefaultAccuracy;
template <Function func, typename Tin, typename Tout, Accuracy acc = Accuracy::kNS, Feature fea = Feature::kNS>
struct Evaluator {};

} // oneapi::mkl::vm::device::detail

namespace oneapi::mkl::vm::detail::gpu::intel::scalar {

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_d_ha_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_c_ep_gen(const std::complex<float>* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_c_la_gen(const std::complex<float>* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_c_ha_gen(const std::complex<float>* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_z_ep_gen(const std::complex<double>* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_z_la_gen(const std::complex<double>* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status abs_z_ha_gen(const std::complex<double>* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acos_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acos_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acos_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acos_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acos_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acos_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acos_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acos_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acos_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acosh_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acosh_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acosh_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acosh_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acosh_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acosh_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acosh_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acosh_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acosh_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acospi_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acospi_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acospi_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acospi_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acospi_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acospi_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acospi_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acospi_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status acospi_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_s_ha_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_d_ha_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_c_ep_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_c_la_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_c_ha_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_z_ep_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_z_la_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status add_z_ha_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status arg_c_ep_gen(const std::complex<float>* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status arg_c_la_gen(const std::complex<float>* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status arg_c_ha_gen(const std::complex<float>* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status arg_z_ep_gen(const std::complex<double>* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status arg_z_la_gen(const std::complex<double>* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status arg_z_ha_gen(const std::complex<double>* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asin_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asin_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asin_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asin_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asin_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asin_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asin_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asin_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asin_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_h_ep_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_h_la_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_h_ha_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_s_ep_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_s_la_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinh_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinpi_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinpi_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinpi_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinpi_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinpi_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinpi_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinpi_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinpi_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status asinpi_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan_d_la_nolut(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2_s_ha_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2_d_ha_gen(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2pi_h_ep_fp64only_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2pi_h_la_fp64only_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2pi_h_ha_fp64only_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2pi_s_ep_fp64only(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2pi_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2pi_s_la_nofp64(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2pi_s_ha_fp64only(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2pi_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2pi_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atan2pi_d_ha_gen(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanh_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanh_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanh_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanh_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanh_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanh_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanh_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanh_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanh_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_h_ep_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_h_la_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_h_ha_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_s_ep_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_s_la_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status atanpi_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cbrt_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cbrt_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cbrt_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cbrt_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cbrt_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cbrt_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cbrt_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cbrt_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cbrt_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorm_h_ep_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorm_h_la_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorm_h_ha_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorm_s_ep_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorm_s_la_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorm_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorm_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorm_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorm_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorminv_h_ep_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorminv_h_la_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorminv_h_ha_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorminv_s_ep_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorminv_s_la_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorminv_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorminv_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorminv_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cdfnorminv_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ceil_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ceil_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ceil_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ceil_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ceil_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ceil_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ceil_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ceil_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ceil_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cis_c_ep_gen(const float* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cis_c_la_gen(const float* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cis_c_ha_gen(const float* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cis_z_ep_gen(const double* a, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cis_z_la_gen(const double* a, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cis_z_ha_gen(const double* a, std::complex<double>* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status conj_c_ep_gen(const std::complex<float>* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status conj_c_la_gen(const std::complex<float>* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status conj_c_ha_gen(const std::complex<float>* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status conj_z_ep_gen(const std::complex<double>* a, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status conj_z_la_gen(const std::complex<double>* a, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status conj_z_ha_gen(const std::complex<double>* a, std::complex<double>* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status copysign_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status copysign_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status copysign_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status copysign_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status copysign_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status copysign_s_ha_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status copysign_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status copysign_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status copysign_d_ha_gen(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cos_h_ep_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cos_h_la_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cos_h_ha_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cos_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cos_s_ep_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cos_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cos_s_la_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cos_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cos_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cos_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cos_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosd_h_ep_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosd_h_la_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosd_h_ha_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosd_s_ep_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosd_s_la_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosd_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosd_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosd_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosd_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosh_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosh_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosh_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosh_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosh_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosh_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosh_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosh_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cosh_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_h_ep_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_h_la_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_h_ha_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_s_ep_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_s_la_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status cospi_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_h_ep_nofp64_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_h_la_nofp64_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_h_ha_nofp64_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_s_ep_nofp64(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_s_la_nofp64(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_s_ha_fp64only(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_d_ha_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_c_ep_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_c_la_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_c_ha_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_z_ep_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_z_la_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status div_z_ha_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erf_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erf_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erf_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erf_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erf_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erf_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erf_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erf_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erf_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfc_h_ep_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfc_h_la_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfc_h_ha_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfc_s_ep_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfc_s_la_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfc_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfc_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfc_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfc_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcinv_h_ep_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcinv_h_la_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcinv_h_ha_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcinv_s_ep_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcinv_s_la_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcinv_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcinv_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcinv_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcinv_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcx_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcx_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcx_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcx_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcx_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcx_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcx_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcx_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfcx_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_h_ep_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_h_ep_nolut_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_h_la_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_h_la_nolut_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_h_ha_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_h_ha_nolut_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_s_ep_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_s_ep_nolut(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_s_la_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_s_la_nolut(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_s_ha_nolut(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status erfinv_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_h_ep_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_h_la_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_h_ha_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_d_la_nolut(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_d_ha_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_c_ep_gen(const std::complex<float>* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_c_la_gen(const std::complex<float>* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_c_ha_gen(const std::complex<float>* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_z_ep_gen(const std::complex<double>* a, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_z_la_gen(const std::complex<double>* a, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp_z_ha_nolut(const std::complex<double>* a, std::complex<double>* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp10_h_ep_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp10_h_la_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp10_h_ha_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp10_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp10_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp10_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp10_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp10_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp10_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp2_h_ep_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp2_h_la_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp2_h_ha_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp2_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp2_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp2_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp2_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp2_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status exp2_d_ha_gen(const double* a, double* y) noexcept;


ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_h_ep_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_h_la_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_h_ha_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_s_ep_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_s_la_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_s_ha_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status expm1_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fdim_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fdim_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fdim_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fdim_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fdim_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fdim_s_ha_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fdim_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fdim_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fdim_d_ha_gen(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status floor_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status floor_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status floor_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status floor_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status floor_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status floor_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status floor_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status floor_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status floor_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmax_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmax_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmax_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmax_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmax_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmax_s_ha_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmax_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmax_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmax_d_ha_gen(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmin_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmin_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmin_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmin_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmin_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmin_s_ha_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmin_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmin_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmin_d_ha_gen(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmod_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmod_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmod_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmod_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmod_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmod_s_ha_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmod_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmod_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status fmod_d_ha_gen(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status frac_h_ep_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status frac_h_la_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status frac_h_ha_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status frac_s_ep_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status frac_s_la_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status frac_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status frac_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status frac_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status frac_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status hypot_h_ep_fp64only_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status hypot_h_la_fp64only_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status hypot_h_ha_fp64only_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status hypot_s_ep_fp64only(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status hypot_s_la_fp64only(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status hypot_s_ha_fp64only(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status hypot_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status hypot_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status hypot_d_ha_gen(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status i0_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status i0_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status i0_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status i0_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status i0_d_ep_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status i1_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status i1_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status i1_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status i1_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status i1_d_ep_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status inv_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status inv_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status inv_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status inv_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status inv_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status inv_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status inv_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status inv_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status inv_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invcbrt_h_ep_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invcbrt_h_la_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invcbrt_h_ha_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invcbrt_s_ep_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invcbrt_s_la_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invcbrt_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invcbrt_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invcbrt_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invcbrt_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invsqrt_h_ep_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invsqrt_h_la_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invsqrt_h_ha_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invsqrt_s_ep_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invsqrt_s_la_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invsqrt_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invsqrt_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invsqrt_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status invsqrt_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status j0_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status j0_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status j0_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status j0_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status j0_d_ep_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status j1_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status j1_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status j1_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status j1_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status j1_d_ep_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status jn_h_ep_gen_fp32cast(const sycl::half* a, sycl::half b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status jn_h_la_gen_fp32cast(const sycl::half* a, sycl::half b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status jn_h_ha_gen_fp32cast(const sycl::half* a, sycl::half b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status jn_s_ep_gen(const float* a, float b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status jn_d_ep_gen(const double* a, double b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status lgamma_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status lgamma_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status lgamma_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status lgamma_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status lgamma_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status lgamma_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status lgamma_d_ep_gen(const double* a, double* y) noexcept;


ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_h_ep_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_h_la_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_h_ha_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_s_ep_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_s_la_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_s_ha_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_d_ep_nolut(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_d_la_nolut(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_d_ha_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_c_ep_gen(const std::complex<float>* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_c_la_gen(const std::complex<float>* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_c_ha_gen(const std::complex<float>* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_z_ep_gen(const std::complex<double>* a, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_z_la_gen(const std::complex<double>* a, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status ln_z_ha_nolut(const std::complex<double>* a, std::complex<double>* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log10_h_ep_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log10_h_la_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log10_h_ha_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log10_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log10_s_ep_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log10_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log10_s_la_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log10_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log10_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log10_d_ep_nolut(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log10_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log10_d_la_nolut(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log10_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log1p_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log1p_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log1p_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log1p_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log1p_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log1p_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log1p_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log1p_d_ep_nolut(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log1p_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log1p_d_la_nolut(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log1p_d_ha_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log1p_d_ha_nolut(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log2_h_ep_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log2_h_la_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log2_h_ha_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log2_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log2_s_ep_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log2_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log2_s_la_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log2_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log2_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log2_d_ep_nolut(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log2_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log2_d_la_nolut(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status log2_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status logb_h_ep_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status logb_h_la_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status logb_h_ha_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status logb_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status logb_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status logb_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status logb_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status logb_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status logb_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status maxmag_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status maxmag_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status maxmag_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status maxmag_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status maxmag_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status maxmag_s_ha_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status maxmag_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status maxmag_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status maxmag_d_ha_gen(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status minmag_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status minmag_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status minmag_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status minmag_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status minmag_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status minmag_s_ha_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status minmag_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status minmag_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status minmag_d_ha_gen(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status modf_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y, sycl::half* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status modf_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y, sycl::half* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status modf_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y, sycl::half* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status modf_s_ep_gen(const float* a, float* y, float* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status modf_s_la_gen(const float* a, float* y, float* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status modf_s_ha_gen(const float* a, float* y, float* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status modf_d_ep_gen(const double* a, double* y, double* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status modf_d_la_gen(const double* a, double* y, double* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status modf_d_ha_gen(const double* a, double* y, double* z) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_s_ha_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_d_ha_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_c_ep_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_c_la_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_c_ha_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_c_ha_nolut(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_z_ep_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_z_la_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_z_ha_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mul_z_ha_nolut(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mulbyconj_c_ep_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mulbyconj_c_la_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mulbyconj_c_ha_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mulbyconj_z_ep_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mulbyconj_z_la_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status mulbyconj_z_ha_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nearbyint_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nearbyint_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nearbyint_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nearbyint_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nearbyint_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nearbyint_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nearbyint_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nearbyint_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nearbyint_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nextafter_h_ep_gen(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nextafter_h_la_gen(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nextafter_h_ha_gen(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nextafter_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nextafter_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nextafter_s_ha_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nextafter_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nextafter_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status nextafter_d_ha_gen(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_s_la_nolut(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_s_ha_fp64only(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_s_ha_nolut(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_d_ep_nolut(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_d_la_nolut(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_d_ha_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow_d_ha_nolut(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow2o3_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow2o3_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow2o3_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow2o3_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow2o3_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow2o3_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow2o3_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow2o3_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow2o3_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow3o2_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow3o2_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow3o2_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow3o2_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow3o2_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow3o2_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow3o2_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow3o2_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status pow3o2_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_s_la_nolut(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_s_ha_fp64only(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_s_ha_nolut(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_d_ep_nolut(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_d_la_nolut(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_d_ha_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powr_d_ha_nolut(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_h_ep_fp64only_fp32cast(const sycl::half* a, sycl::half b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_h_ep_nolut_fp32cast(const sycl::half* a, sycl::half b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_h_la_fp64only_fp32cast(const sycl::half* a, sycl::half b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_h_la_nolut_fp32cast(const sycl::half* a, sycl::half b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_h_ha_fp64only_fp32cast(const sycl::half* a, sycl::half b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_h_ha_nolut_fp32cast(const sycl::half* a, sycl::half b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_s_ep_fp64only(const float* a, float b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_s_ep_nolut(const float* a, float b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_s_la_fp64only(const float* a, float b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_s_la_nolut(const float* a, float b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_s_ha_fp64only(const float* a, float b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_s_ha_nolut(const float* a, float b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_d_ep_gen(const double* a, double b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_d_ep_nolut(const double* a, double b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_d_la_gen(const double* a, double b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_d_la_nolut(const double* a, double b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_d_ha_gen(const double* a, double b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status powx_d_ha_nolut(const double* a, double b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status remainder_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status remainder_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status remainder_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status remainder_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status remainder_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status remainder_s_ha_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status remainder_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status remainder_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status remainder_d_ha_gen(const double* a, const double* b, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status rint_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status rint_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status rint_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status rint_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status rint_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status rint_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status rint_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status rint_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status rint_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status round_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status round_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status round_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status round_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status round_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status round_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status round_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status round_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status round_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sin_h_ep_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sin_h_la_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sin_h_ha_gen(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sin_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sin_s_ep_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sin_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sin_s_la_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sin_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sin_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sin_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sin_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincos_h_ep_gen(const sycl::half* a, sycl::half* y, sycl::half* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincos_h_la_gen(const sycl::half* a, sycl::half* y, sycl::half* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincos_h_ha_gen(const sycl::half* a, sycl::half* y, sycl::half* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincos_s_ep_gen(const float* a, float* y, float* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincos_s_ep_nofp64(const float* a, float* y, float* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincos_s_la_gen(const float* a, float* y, float* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincos_s_la_nofp64(const float* a, float* y, float* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincos_s_ha_fp64only(const float* a, float* y, float* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincos_d_ep_gen(const double* a, double* y, double* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincos_d_la_gen(const double* a, double* y, double* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincos_d_ha_gen(const double* a, double* y, double* z) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincospi_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y, sycl::half* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincospi_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y, sycl::half* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincospi_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y, sycl::half* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincospi_s_ep_gen(const float* a, float* y, float* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincospi_s_la_gen(const float* a, float* y, float* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincospi_s_ha_gen(const float* a, float* y, float* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincospi_d_ep_gen(const double* a, double* y, double* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincospi_d_la_gen(const double* a, double* y, double* z) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sincospi_d_ha_gen(const double* a, double* y, double* z) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sind_h_ep_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sind_h_la_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sind_h_ha_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sind_s_ep_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sind_s_la_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sind_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sind_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sind_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sind_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinh_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinh_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinh_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinh_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinh_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinh_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinh_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinh_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinh_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_h_ep_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_h_la_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_h_ha_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_s_ep_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_s_la_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sinpi_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqr_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqr_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqr_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqr_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqr_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqr_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqr_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqr_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqr_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_d_ha_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_c_ep_gen(const std::complex<float>* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_c_la_gen(const std::complex<float>* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_c_ha_gen(const std::complex<float>* a, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_z_ep_gen(const std::complex<double>* a, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_z_la_gen(const std::complex<double>* a, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sqrt_z_ha_gen(const std::complex<double>* a, std::complex<double>* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_h_ep_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_h_la_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_h_ha_gen_fp32cast(const sycl::half* a, const sycl::half* b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_s_ep_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_s_la_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_s_ha_gen(const float* a, const float* b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_d_ep_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_d_la_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_d_ha_gen(const double* a, const double* b, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_c_ep_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_c_la_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_c_ha_gen(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_z_ep_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_z_la_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status sub_z_ha_gen(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_h_ep_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_h_la_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_h_ha_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_s_ep_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_s_la_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tan_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tand_h_ep_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tand_h_la_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tand_h_ha_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tand_s_ep_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tand_s_la_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tand_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tand_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tand_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tand_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_h_ep_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_h_la_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_h_ha_nofp64_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_s_ep_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_s_la_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_s_ha_nofp64(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanh_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanpi_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanpi_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanpi_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanpi_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanpi_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanpi_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanpi_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanpi_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tanpi_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tgamma_h_ep_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tgamma_h_la_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tgamma_h_ha_fp64only_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tgamma_s_ep_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tgamma_s_la_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tgamma_s_ha_fp64only(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tgamma_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status tgamma_d_la_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status trunc_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status trunc_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status trunc_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status trunc_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status trunc_s_la_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status trunc_s_ha_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status trunc_d_ep_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status trunc_d_la_gen(const double* a, double* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status trunc_d_ha_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status y0_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status y0_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status y0_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status y0_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status y0_d_ep_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status y1_h_ep_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status y1_h_la_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status y1_h_ha_gen_fp32cast(const sycl::half* a, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status y1_s_ep_gen(const float* a, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status y1_d_ep_gen(const double* a, double* y) noexcept;

ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status yn_h_ep_gen_fp32cast(const sycl::half* a, sycl::half b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status yn_h_la_gen_fp32cast(const sycl::half* a, sycl::half b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status yn_h_ha_gen_fp32cast(const sycl::half* a, sycl::half b, sycl::half* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status yn_s_ep_gen(const float* a, float b, float* y) noexcept;
ONEAPI_MKL_VM_EXPORT_CPP SYCL_EXTERNAL oneapi::mkl::vm::status yn_d_ep_gen(const double* a, double b, double* y) noexcept;


} // oneapi::mkl::vm::detail::gpu::intel::scalar
namespace oneapi::mkl::vm::device::detail {

template <>
struct Evaluator<Function::kAbs, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, std::complex<float>, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_c_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, std::complex<float>, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_c_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, std::complex<float>, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_c_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, std::complex<double>, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_z_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, std::complex<double>, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_z_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAbs, std::complex<double>, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::abs_z_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kAcos, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acos_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcos, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acos_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcos, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acos_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcos, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acos_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcos, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acos_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcos, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acos_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcos, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acos_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcos, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acos_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcos, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acos_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kAcosh, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acosh_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcosh, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acosh_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcosh, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acosh_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcosh, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acosh_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcosh, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acosh_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcosh, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acosh_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcosh, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acosh_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcosh, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acosh_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcosh, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acosh_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kAcospi, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acospi_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcospi, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acospi_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcospi, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acospi_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcospi, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acospi_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcospi, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acospi_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcospi, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acospi_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcospi, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acospi_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcospi, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acospi_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAcospi, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::acospi_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kAdd, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_s_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, std::complex<float>, std::complex<float>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_c_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, std::complex<float>, std::complex<float>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_c_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, std::complex<float>, std::complex<float>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_c_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, std::complex<double>, std::complex<double>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_z_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, std::complex<double>, std::complex<double>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_z_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAdd, std::complex<double>, std::complex<double>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::add_z_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kArg, std::complex<float>, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::arg_c_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kArg, std::complex<float>, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::arg_c_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kArg, std::complex<float>, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::arg_c_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kArg, std::complex<double>, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::arg_z_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kArg, std::complex<double>, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::arg_z_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kArg, std::complex<double>, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::arg_z_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kAsin, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asin_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsin, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asin_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsin, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asin_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsin, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asin_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsin, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asin_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsin, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asin_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsin, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asin_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsin, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asin_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsin, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asin_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kAsinh, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinh, sycl::half, sycl::half, Accuracy::kEP, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_h_ep_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinh, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinh, sycl::half, sycl::half, Accuracy::kLA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_h_la_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinh, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinh, sycl::half, sycl::half, Accuracy::kHA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_h_ha_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinh, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinh, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_s_ep_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinh, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinh, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_s_la_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinh, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinh, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinh, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinh, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinh_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kAsinpi, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinpi_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinpi, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinpi_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinpi, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinpi_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinpi, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinpi_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinpi, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinpi_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinpi, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinpi_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinpi, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinpi_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinpi, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinpi_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAsinpi, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::asinpi_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kAtan, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan, double, double, Accuracy::kLA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan_d_la_nolut(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kAtan2, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2_s_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kAtan2pi, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2pi_h_ep_fp64only_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2pi, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2pi_h_la_fp64only_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2pi, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2pi_h_ha_fp64only_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2pi, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2pi_s_ep_fp64only(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2pi, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2pi_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2pi, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2pi_s_la_nofp64(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2pi, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2pi_s_ha_fp64only(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2pi, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2pi_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2pi, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2pi_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtan2pi, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atan2pi_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kAtanh, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanh_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanh, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanh_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanh, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanh_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanh, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanh_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanh, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanh_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanh, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanh_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanh, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanh_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanh, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanh_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanh, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanh_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kAtanpi, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanpi, sycl::half, sycl::half, Accuracy::kEP, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_h_ep_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanpi, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanpi, sycl::half, sycl::half, Accuracy::kLA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_h_la_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanpi, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanpi, sycl::half, sycl::half, Accuracy::kHA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_h_ha_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanpi, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanpi, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_s_ep_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanpi, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanpi, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_s_la_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanpi, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanpi, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanpi, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kAtanpi, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::atanpi_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kCbrt, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cbrt_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCbrt, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cbrt_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCbrt, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cbrt_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCbrt, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cbrt_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCbrt, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cbrt_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCbrt, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cbrt_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCbrt, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cbrt_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCbrt, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cbrt_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCbrt, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cbrt_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kCdfnorm, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorm_h_ep_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorm, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorm_h_la_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorm, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorm_h_ha_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorm, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorm_s_ep_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorm, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorm_s_la_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorm, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorm_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorm, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorm_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorm, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorm_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorm, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorm_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kCdfnorminv, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorminv_h_ep_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorminv, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorminv_h_la_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorminv, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorminv_h_ha_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorminv, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorminv_s_ep_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorminv, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorminv_s_la_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorminv, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorminv_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorminv, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorminv_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorminv, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorminv_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCdfnorminv, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cdfnorminv_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kCeil, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ceil_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCeil, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ceil_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCeil, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ceil_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCeil, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ceil_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCeil, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ceil_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCeil, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ceil_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCeil, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ceil_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCeil, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ceil_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCeil, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ceil_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kCis, float, std::complex<float>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cis_c_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCis, float, std::complex<float>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cis_c_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCis, float, std::complex<float>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cis_c_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCis, double, std::complex<double>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cis_z_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCis, double, std::complex<double>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cis_z_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCis, double, std::complex<double>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cis_z_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kConj, std::complex<float>, std::complex<float>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::conj_c_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kConj, std::complex<float>, std::complex<float>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::conj_c_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kConj, std::complex<float>, std::complex<float>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::conj_c_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kConj, std::complex<double>, std::complex<double>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::conj_z_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kConj, std::complex<double>, std::complex<double>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::conj_z_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kConj, std::complex<double>, std::complex<double>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::conj_z_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kCopysign, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::copysign_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCopysign, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::copysign_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCopysign, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::copysign_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCopysign, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::copysign_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCopysign, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::copysign_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCopysign, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::copysign_s_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCopysign, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::copysign_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCopysign, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::copysign_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCopysign, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::copysign_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kCos, sycl::half, sycl::half, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cos_h_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCos, sycl::half, sycl::half, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cos_h_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCos, sycl::half, sycl::half, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cos_h_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCos, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cos_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCos, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cos_s_ep_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCos, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cos_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCos, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cos_s_la_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCos, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cos_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCos, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cos_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCos, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cos_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCos, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cos_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kCosd, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosd_h_ep_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosd, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosd_h_la_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosd, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosd_h_ha_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosd, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosd_s_ep_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosd, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosd_s_la_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosd, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosd_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosd, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosd_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosd, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosd_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosd, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosd_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kCosh, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosh_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosh, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosh_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosh, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosh_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosh, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosh_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosh, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosh_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosh, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosh_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosh, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosh_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosh, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosh_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCosh, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cosh_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kCospi, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCospi, sycl::half, sycl::half, Accuracy::kEP, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_h_ep_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCospi, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCospi, sycl::half, sycl::half, Accuracy::kLA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_h_la_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCospi, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCospi, sycl::half, sycl::half, Accuracy::kHA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_h_ha_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCospi, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCospi, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_s_ep_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCospi, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCospi, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_s_la_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCospi, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCospi, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCospi, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kCospi, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::cospi_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kDiv, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, sycl::half, sycl::half, Accuracy::kEP, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_h_ep_nofp64_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, sycl::half, sycl::half, Accuracy::kLA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_h_la_nofp64_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, sycl::half, sycl::half, Accuracy::kHA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_h_ha_nofp64_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_s_ep_nofp64(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_s_la_nofp64(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_s_ha_fp64only(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, std::complex<float>, std::complex<float>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_c_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, std::complex<float>, std::complex<float>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_c_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, std::complex<float>, std::complex<float>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_c_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, std::complex<double>, std::complex<double>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_z_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, std::complex<double>, std::complex<double>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_z_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kDiv, std::complex<double>, std::complex<double>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::div_z_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kErf, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erf_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErf, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erf_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErf, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erf_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErf, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erf_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErf, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erf_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErf, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erf_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErf, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erf_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErf, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erf_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErf, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erf_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kErfc, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfc_h_ep_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfc, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfc_h_la_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfc, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfc_h_ha_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfc, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfc_s_ep_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfc, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfc_s_la_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfc, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfc_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfc, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfc_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfc, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfc_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfc, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfc_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kErfcinv, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcinv_h_ep_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcinv, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcinv_h_la_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcinv, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcinv_h_ha_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcinv, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcinv_s_ep_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcinv, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcinv_s_la_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcinv, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcinv_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcinv, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcinv_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcinv, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcinv_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcinv, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcinv_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kErfcx, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcx_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcx, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcx_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcx, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcx_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcx, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcx_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcx, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcx_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcx, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcx_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcx, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcx_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcx, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcx_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfcx, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfcx_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kErfinv, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_h_ep_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, sycl::half, sycl::half, Accuracy::kEP, Feature::kTACast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_h_ep_nolut_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_h_la_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, sycl::half, sycl::half, Accuracy::kLA, Feature::kTACast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_h_la_nolut_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_h_ha_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, sycl::half, sycl::half, Accuracy::kHA, Feature::kTACast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_h_ha_nolut_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_s_ep_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, float, float, Accuracy::kEP, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_s_ep_nolut(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_s_la_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, float, float, Accuracy::kLA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_s_la_nolut(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, float, float, Accuracy::kHA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_s_ha_nolut(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kErfinv, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::erfinv_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kExp, sycl::half, sycl::half, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_h_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, sycl::half, sycl::half, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_h_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, sycl::half, sycl::half, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_h_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, double, double, Accuracy::kLA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_d_la_nolut(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, std::complex<float>, std::complex<float>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_c_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, std::complex<float>, std::complex<float>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_c_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, std::complex<float>, std::complex<float>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_c_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, std::complex<double>, std::complex<double>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_z_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, std::complex<double>, std::complex<double>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_z_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp, std::complex<double>, std::complex<double>, Accuracy::kHA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp_z_ha_nolut(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kExp10, sycl::half, sycl::half, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp10_h_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp10, sycl::half, sycl::half, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp10_h_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp10, sycl::half, sycl::half, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp10_h_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp10, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp10_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp10, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp10_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp10, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp10_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp10, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp10_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp10, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp10_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp10, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp10_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kExp2, sycl::half, sycl::half, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp2_h_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp2, sycl::half, sycl::half, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp2_h_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp2, sycl::half, sycl::half, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp2_h_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp2, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp2_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp2, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp2_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp2, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp2_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp2, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp2_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp2, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp2_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExp2, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::exp2_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};



template <>
struct Evaluator<Function::kExpm1, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, sycl::half, sycl::half, Accuracy::kEP, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_h_ep_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, sycl::half, sycl::half, Accuracy::kLA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_h_la_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, sycl::half, sycl::half, Accuracy::kHA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_h_ha_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_s_ep_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_s_la_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, float, float, Accuracy::kHA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_s_ha_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kExpm1, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::expm1_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kFdim, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fdim_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFdim, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fdim_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFdim, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fdim_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFdim, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fdim_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFdim, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fdim_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFdim, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fdim_s_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFdim, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fdim_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFdim, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fdim_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFdim, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fdim_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kFloor, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::floor_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFloor, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::floor_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFloor, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::floor_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFloor, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::floor_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFloor, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::floor_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFloor, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::floor_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFloor, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::floor_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFloor, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::floor_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFloor, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::floor_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kFmax, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmax_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmax, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmax_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmax, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmax_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmax, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmax_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmax, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmax_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmax, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmax_s_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmax, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmax_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmax, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmax_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmax, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmax_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kFmin, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmin_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmin, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmin_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmin, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmin_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmin, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmin_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmin, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmin_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmin, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmin_s_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmin, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmin_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmin, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmin_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmin, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmin_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kFmod, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmod_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmod, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmod_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmod, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmod_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmod, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmod_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmod, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmod_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmod, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmod_s_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmod, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmod_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmod, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmod_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFmod, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::fmod_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kFrac, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::frac_h_ep_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFrac, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::frac_h_la_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFrac, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::frac_h_ha_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFrac, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::frac_s_ep_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFrac, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::frac_s_la_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFrac, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::frac_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFrac, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::frac_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFrac, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::frac_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kFrac, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::frac_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kHypot, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::hypot_h_ep_fp64only_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kHypot, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::hypot_h_la_fp64only_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kHypot, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::hypot_h_ha_fp64only_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kHypot, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::hypot_s_ep_fp64only(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kHypot, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::hypot_s_la_fp64only(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kHypot, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::hypot_s_ha_fp64only(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kHypot, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::hypot_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kHypot, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::hypot_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kHypot, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::hypot_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kI0, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::i0_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kI0, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::i0_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kI0, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::i0_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kI0, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::i0_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kI0, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::i0_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kI1, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::i1_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kI1, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::i1_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kI1, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::i1_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kI1, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::i1_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kI1, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::i1_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kInv, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::inv_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInv, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::inv_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInv, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::inv_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInv, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::inv_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInv, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::inv_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInv, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::inv_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInv, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::inv_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInv, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::inv_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInv, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::inv_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kInvcbrt, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invcbrt_h_ep_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvcbrt, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invcbrt_h_la_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvcbrt, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invcbrt_h_ha_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvcbrt, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invcbrt_s_ep_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvcbrt, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invcbrt_s_la_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvcbrt, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invcbrt_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvcbrt, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invcbrt_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvcbrt, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invcbrt_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvcbrt, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invcbrt_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kInvsqrt, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invsqrt_h_ep_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvsqrt, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invsqrt_h_la_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvsqrt, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invsqrt_h_ha_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvsqrt, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invsqrt_s_ep_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvsqrt, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invsqrt_s_la_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvsqrt, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invsqrt_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvsqrt, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invsqrt_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvsqrt, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invsqrt_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kInvsqrt, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::invsqrt_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kJ0, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::j0_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kJ0, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::j0_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kJ0, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::j0_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kJ0, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::j0_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kJ0, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::j0_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kJ1, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::j1_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kJ1, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::j1_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kJ1, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::j1_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kJ1, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::j1_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kJ1, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::j1_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kJn, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::jn_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kJn, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::jn_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kJn, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::jn_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kJn, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::jn_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kJn, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::jn_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kLgamma, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::lgamma_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLgamma, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::lgamma_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLgamma, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::lgamma_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLgamma, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::lgamma_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLgamma, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::lgamma_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLgamma, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::lgamma_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLgamma, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::lgamma_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};



template <>
struct Evaluator<Function::kLn, sycl::half, sycl::half, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_h_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, sycl::half, sycl::half, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_h_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, sycl::half, sycl::half, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_h_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_s_ep_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_s_la_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, float, float, Accuracy::kHA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_s_ha_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, double, double, Accuracy::kEP, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_d_ep_nolut(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, double, double, Accuracy::kLA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_d_la_nolut(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, std::complex<float>, std::complex<float>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_c_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, std::complex<float>, std::complex<float>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_c_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, std::complex<float>, std::complex<float>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_c_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, std::complex<double>, std::complex<double>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_z_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, std::complex<double>, std::complex<double>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_z_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLn, std::complex<double>, std::complex<double>, Accuracy::kHA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::ln_z_ha_nolut(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kLog10, sycl::half, sycl::half, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log10_h_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog10, sycl::half, sycl::half, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log10_h_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog10, sycl::half, sycl::half, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log10_h_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog10, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log10_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog10, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log10_s_ep_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog10, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log10_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog10, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log10_s_la_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog10, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log10_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog10, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log10_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog10, double, double, Accuracy::kEP, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log10_d_ep_nolut(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog10, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log10_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog10, double, double, Accuracy::kLA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log10_d_la_nolut(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog10, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log10_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kLog1p, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log1p_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog1p, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log1p_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog1p, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log1p_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog1p, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log1p_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog1p, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log1p_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog1p, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log1p_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog1p, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log1p_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog1p, double, double, Accuracy::kEP, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log1p_d_ep_nolut(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog1p, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log1p_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog1p, double, double, Accuracy::kLA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log1p_d_la_nolut(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog1p, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log1p_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog1p, double, double, Accuracy::kHA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log1p_d_ha_nolut(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kLog2, sycl::half, sycl::half, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log2_h_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog2, sycl::half, sycl::half, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log2_h_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog2, sycl::half, sycl::half, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log2_h_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog2, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log2_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog2, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log2_s_ep_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog2, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log2_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog2, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log2_s_la_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog2, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log2_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog2, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log2_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog2, double, double, Accuracy::kEP, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log2_d_ep_nolut(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog2, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log2_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog2, double, double, Accuracy::kLA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log2_d_la_nolut(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLog2, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::log2_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kLogb, sycl::half, sycl::half, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::logb_h_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLogb, sycl::half, sycl::half, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::logb_h_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLogb, sycl::half, sycl::half, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::logb_h_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLogb, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::logb_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLogb, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::logb_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLogb, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::logb_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLogb, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::logb_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLogb, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::logb_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kLogb, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::logb_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kMaxmag, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::maxmag_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMaxmag, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::maxmag_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMaxmag, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::maxmag_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMaxmag, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::maxmag_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMaxmag, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::maxmag_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMaxmag, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::maxmag_s_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMaxmag, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::maxmag_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMaxmag, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::maxmag_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMaxmag, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::maxmag_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kMinmag, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::minmag_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMinmag, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::minmag_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMinmag, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::minmag_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMinmag, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::minmag_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMinmag, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::minmag_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMinmag, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::minmag_s_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMinmag, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::minmag_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMinmag, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::minmag_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMinmag, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::minmag_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kModf, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y, sycl::half* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::modf_h_ep_gen_fp32cast(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kModf, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y, sycl::half* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::modf_h_la_gen_fp32cast(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kModf, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y, sycl::half* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::modf_h_ha_gen_fp32cast(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kModf, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y, float* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::modf_s_ep_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kModf, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y, float* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::modf_s_la_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kModf, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y, float* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::modf_s_ha_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kModf, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y, double* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::modf_d_ep_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kModf, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y, double* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::modf_d_la_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kModf, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y, double* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::modf_d_ha_gen(a, y, z);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kMul, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_s_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, std::complex<float>, std::complex<float>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_c_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, std::complex<float>, std::complex<float>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_c_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, std::complex<float>, std::complex<float>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_c_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, std::complex<float>, std::complex<float>, Accuracy::kHA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_c_ha_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, std::complex<double>, std::complex<double>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_z_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, std::complex<double>, std::complex<double>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_z_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, std::complex<double>, std::complex<double>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_z_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMul, std::complex<double>, std::complex<double>, Accuracy::kHA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mul_z_ha_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kMulbyconj, std::complex<float>, std::complex<float>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mulbyconj_c_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMulbyconj, std::complex<float>, std::complex<float>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mulbyconj_c_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMulbyconj, std::complex<float>, std::complex<float>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mulbyconj_c_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMulbyconj, std::complex<double>, std::complex<double>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mulbyconj_z_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMulbyconj, std::complex<double>, std::complex<double>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mulbyconj_z_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kMulbyconj, std::complex<double>, std::complex<double>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::mulbyconj_z_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kNearbyint, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nearbyint_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNearbyint, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nearbyint_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNearbyint, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nearbyint_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNearbyint, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nearbyint_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNearbyint, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nearbyint_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNearbyint, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nearbyint_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNearbyint, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nearbyint_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNearbyint, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nearbyint_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNearbyint, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nearbyint_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kNextafter, sycl::half, sycl::half, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nextafter_h_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNextafter, sycl::half, sycl::half, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nextafter_h_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNextafter, sycl::half, sycl::half, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nextafter_h_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNextafter, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nextafter_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNextafter, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nextafter_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNextafter, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nextafter_s_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNextafter, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nextafter_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNextafter, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nextafter_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kNextafter, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::nextafter_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kPow, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow, float, float, Accuracy::kLA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_s_la_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_s_ha_fp64only(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow, float, float, Accuracy::kHA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_s_ha_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow, double, double, Accuracy::kEP, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_d_ep_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow, double, double, Accuracy::kLA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_d_la_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow, double, double, Accuracy::kHA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow_d_ha_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kPow2o3, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow2o3_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow2o3, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow2o3_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow2o3, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow2o3_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow2o3, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow2o3_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow2o3, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow2o3_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow2o3, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow2o3_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow2o3, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow2o3_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow2o3, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow2o3_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow2o3, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow2o3_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kPow3o2, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow3o2_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow3o2, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow3o2_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow3o2, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow3o2_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow3o2, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow3o2_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow3o2, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow3o2_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow3o2, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow3o2_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow3o2, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow3o2_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow3o2, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow3o2_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPow3o2, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::pow3o2_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kPowr, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowr, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowr, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowr, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowr, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowr, float, float, Accuracy::kLA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_s_la_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowr, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_s_ha_fp64only(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowr, float, float, Accuracy::kHA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_s_ha_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowr, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowr, double, double, Accuracy::kEP, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_d_ep_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowr, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowr, double, double, Accuracy::kLA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_d_la_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowr, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowr, double, double, Accuracy::kHA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powr_d_ha_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kPowx, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_h_ep_fp64only_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, sycl::half, sycl::half, Accuracy::kEP, Feature::kTACast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_h_ep_nolut_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_h_la_fp64only_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, sycl::half, sycl::half, Accuracy::kLA, Feature::kTACast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_h_la_nolut_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_h_ha_fp64only_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, sycl::half, sycl::half, Accuracy::kHA, Feature::kTACast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_h_ha_nolut_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_s_ep_fp64only(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, float, float, Accuracy::kEP, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_s_ep_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_s_la_fp64only(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, float, float, Accuracy::kLA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_s_la_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_s_ha_fp64only(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, float, float, Accuracy::kHA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_s_ha_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, double, double, Accuracy::kEP, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_d_ep_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, double, double, Accuracy::kLA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_d_la_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kPowx, double, double, Accuracy::kHA, Feature::kTA> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::powx_d_ha_nolut(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kRemainder, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::remainder_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRemainder, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::remainder_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRemainder, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::remainder_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRemainder, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::remainder_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRemainder, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::remainder_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRemainder, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::remainder_s_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRemainder, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::remainder_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRemainder, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::remainder_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRemainder, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::remainder_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kRint, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::rint_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRint, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::rint_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRint, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::rint_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRint, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::rint_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRint, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::rint_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRint, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::rint_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRint, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::rint_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRint, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::rint_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRint, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::rint_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kRound, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::round_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRound, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::round_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRound, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::round_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRound, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::round_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRound, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::round_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRound, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::round_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRound, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::round_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRound, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::round_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kRound, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::round_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kSin, sycl::half, sycl::half, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sin_h_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSin, sycl::half, sycl::half, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sin_h_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSin, sycl::half, sycl::half, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sin_h_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSin, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sin_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSin, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sin_s_ep_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSin, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sin_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSin, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sin_s_la_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSin, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sin_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSin, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sin_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSin, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sin_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSin, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sin_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kSincos, sycl::half, sycl::half, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y, sycl::half* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincos_h_ep_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincos, sycl::half, sycl::half, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y, sycl::half* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincos_h_la_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincos, sycl::half, sycl::half, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y, sycl::half* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincos_h_ha_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincos, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y, float* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincos_s_ep_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincos, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y, float* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincos_s_ep_nofp64(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincos, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y, float* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincos_s_la_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincos, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y, float* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincos_s_la_nofp64(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincos, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y, float* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincos_s_ha_fp64only(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincos, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y, double* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincos_d_ep_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincos, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y, double* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincos_d_la_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincos, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y, double* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincos_d_ha_gen(a, y, z);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kSincospi, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y, sycl::half* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincospi_h_ep_gen_fp32cast(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincospi, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y, sycl::half* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincospi_h_la_gen_fp32cast(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincospi, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y, sycl::half* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincospi_h_ha_gen_fp32cast(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincospi, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y, float* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincospi_s_ep_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincospi, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y, float* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincospi_s_la_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincospi, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y, float* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincospi_s_ha_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincospi, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y, double* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincospi_d_ep_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincospi, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y, double* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincospi_d_la_gen(a, y, z);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSincospi, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y, double* z) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sincospi_d_ha_gen(a, y, z);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kSind, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sind_h_ep_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSind, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sind_h_la_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSind, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sind_h_ha_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSind, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sind_s_ep_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSind, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sind_s_la_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSind, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sind_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSind, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sind_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSind, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sind_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSind, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sind_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kSinh, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinh_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinh, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinh_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinh, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinh_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinh, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinh_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinh, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinh_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinh, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinh_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinh, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinh_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinh, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinh_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinh, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinh_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kSinpi, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinpi, sycl::half, sycl::half, Accuracy::kEP, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_h_ep_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinpi, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinpi, sycl::half, sycl::half, Accuracy::kLA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_h_la_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinpi, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinpi, sycl::half, sycl::half, Accuracy::kHA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_h_ha_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinpi, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinpi, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_s_ep_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinpi, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinpi, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_s_la_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinpi, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinpi, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinpi, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSinpi, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sinpi_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kSqr, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqr_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqr, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqr_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqr, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqr_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqr, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqr_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqr, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqr_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqr, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqr_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqr, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqr_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqr, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqr_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqr, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqr_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kSqrt, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, std::complex<float>, std::complex<float>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_c_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, std::complex<float>, std::complex<float>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_c_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, std::complex<float>, std::complex<float>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_c_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, std::complex<double>, std::complex<double>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_z_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, std::complex<double>, std::complex<double>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_z_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSqrt, std::complex<double>, std::complex<double>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sqrt_z_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kSub, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, const sycl::half* b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_s_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, const float* b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_s_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_d_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, const double* b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_d_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, std::complex<float>, std::complex<float>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_c_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, std::complex<float>, std::complex<float>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_c_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, std::complex<float>, std::complex<float>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_c_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, std::complex<double>, std::complex<double>, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_z_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, std::complex<double>, std::complex<double>, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_z_la_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kSub, std::complex<double>, std::complex<double>, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::sub_z_ha_gen(a, b, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kTan, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTan, sycl::half, sycl::half, Accuracy::kEP, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_h_ep_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTan, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTan, sycl::half, sycl::half, Accuracy::kLA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_h_la_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTan, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTan, sycl::half, sycl::half, Accuracy::kHA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_h_ha_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTan, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTan, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_s_ep_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTan, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTan, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_s_la_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTan, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTan, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTan, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTan, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tan_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kTand, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tand_h_ep_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTand, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tand_h_la_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTand, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tand_h_ha_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTand, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tand_s_ep_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTand, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tand_s_la_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTand, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tand_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTand, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tand_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTand, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tand_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTand, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tand_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kTanh, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, sycl::half, sycl::half, Accuracy::kEP, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_h_ep_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, sycl::half, sycl::half, Accuracy::kLA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_h_la_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, sycl::half, sycl::half, Accuracy::kHA, Feature::kSOCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_h_ha_nofp64_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, float, float, Accuracy::kEP, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_s_ep_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, float, float, Accuracy::kLA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_s_la_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, float, float, Accuracy::kHA, Feature::kSO> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_s_ha_nofp64(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanh, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanh_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kTanpi, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanpi_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanpi, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanpi_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanpi, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanpi_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanpi, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanpi_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanpi, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanpi_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanpi, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanpi_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanpi, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanpi_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanpi, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanpi_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTanpi, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tanpi_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kTgamma, sycl::half, sycl::half, Accuracy::kEP, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tgamma_h_ep_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTgamma, sycl::half, sycl::half, Accuracy::kLA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tgamma_h_la_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTgamma, sycl::half, sycl::half, Accuracy::kHA, Feature::kDRCast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tgamma_h_ha_fp64only_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTgamma, float, float, Accuracy::kEP, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tgamma_s_ep_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTgamma, float, float, Accuracy::kLA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tgamma_s_la_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTgamma, float, float, Accuracy::kHA, Feature::kDR> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tgamma_s_ha_fp64only(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTgamma, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tgamma_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTgamma, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::tgamma_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kTrunc, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::trunc_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTrunc, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::trunc_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTrunc, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::trunc_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTrunc, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::trunc_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTrunc, float, float, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::trunc_s_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTrunc, float, float, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::trunc_s_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTrunc, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::trunc_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTrunc, double, double, Accuracy::kLA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::trunc_d_la_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kTrunc, double, double, Accuracy::kHA, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::trunc_d_ha_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kY0, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::y0_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kY0, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::y0_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kY0, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::y0_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kY0, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::y0_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kY0, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::y0_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kY1, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::y1_h_ep_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kY1, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::y1_h_la_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kY1, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::y1_h_ha_gen_fp32cast(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kY1, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::y1_s_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kY1, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::y1_d_ep_gen(a, y);
    } // oneapi::mkl::vm::status

};


template <>
struct Evaluator<Function::kYn, sycl::half, sycl::half, Accuracy::kEP, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::yn_h_ep_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kYn, sycl::half, sycl::half, Accuracy::kLA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::yn_h_la_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kYn, sycl::half, sycl::half, Accuracy::kHA, Feature::kGECast32> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const sycl::half* a, sycl::half b, sycl::half* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::yn_h_ha_gen_fp32cast(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kYn, float, float, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const float* a, float b, float* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::yn_s_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};

template <>
struct Evaluator<Function::kYn, double, double, Accuracy::kEP, Feature::kGE> {
    void operator()() {}
    oneapi::mkl::vm::status operator()(const double* a, double b, double* y) {
        return oneapi::mkl::vm::detail::gpu::intel::scalar::yn_d_ep_gen(a, b, y);
    } // oneapi::mkl::vm::status

};



} // oneapi::mkl::vm::device::detail

namespace oneapi::mkl::vm::device::detail {

template <Function f, typename Tin, typename Tout, Accuracy acc, Feature fea, typename = void>
constexpr bool Exists{false};
template <Function f, typename Tin, typename Tout, Accuracy acc, Feature fea>
constexpr bool Exists<
f, Tin, Tout, acc, fea,
std::void_t<decltype(
std::declval<Evaluator<f, Tin, Tout, acc, fea>>().operator()())>> = true;
using AccFeaT = std::pair<Accuracy, Feature>;
template <Function Func, typename Tin, typename Tout,
Accuracy Acc = Accuracy::kNS, Feature Fea = Feature::kNS>
struct CompileTimeSelector {
    static constexpr auto result = []() -> AccFeaT {
    if constexpr (Exists<Func, Tin, Tout, Acc, Fea>) {
        return AccFeaT(Acc, Fea);
    } // if constexpr (Exists<Fun

    if constexpr (std::is_same_v<sycl::half, Tin> || std::is_same_v<sycl::half, Tout>) {
        if constexpr (Acc == Accuracy::kEP) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kTA>) {
                return AccFeaT(Accuracy::kEP, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kGE>) {
                return AccFeaT(Accuracy::kEP, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kSO>) {
                return AccFeaT(Accuracy::kEP, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kDR>) {
                return AccFeaT(Accuracy::kEP, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTA>) {
                return AccFeaT(Accuracy::kLA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGE>) {
                return AccFeaT(Accuracy::kLA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSO>) {
                return AccFeaT(Accuracy::kLA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDR>) {
                return AccFeaT(Accuracy::kLA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

        if constexpr (Acc == Accuracy::kLA) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTA>) {
                return AccFeaT(Accuracy::kLA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGE>) {
                return AccFeaT(Accuracy::kLA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSO>) {
                return AccFeaT(Accuracy::kLA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDR>) {
                return AccFeaT(Accuracy::kLA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

        if constexpr (Acc == Accuracy::kHA) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

    } // if constexpr (std::is_sa

    if constexpr (std::is_same_v<float, Tin> || std::is_same_v<float, Tout>) {
        if constexpr (Acc == Accuracy::kEP) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kTA>) {
                return AccFeaT(Accuracy::kEP, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kGE>) {
                return AccFeaT(Accuracy::kEP, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kSO>) {
                return AccFeaT(Accuracy::kEP, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kDR>) {
                return AccFeaT(Accuracy::kEP, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTA>) {
                return AccFeaT(Accuracy::kLA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGE>) {
                return AccFeaT(Accuracy::kLA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSO>) {
                return AccFeaT(Accuracy::kLA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDR>) {
                return AccFeaT(Accuracy::kLA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

        if constexpr (Acc == Accuracy::kLA) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTA>) {
                return AccFeaT(Accuracy::kLA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGE>) {
                return AccFeaT(Accuracy::kLA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSO>) {
                return AccFeaT(Accuracy::kLA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDR>) {
                return AccFeaT(Accuracy::kLA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

        if constexpr (Acc == Accuracy::kHA) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

    } // if constexpr (std::is_sa

    if constexpr (std::is_same_v<double, Tin> || std::is_same_v<double, Tout>) {
        if constexpr (Acc == Accuracy::kEP) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kTA>) {
                return AccFeaT(Accuracy::kEP, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kGE>) {
                return AccFeaT(Accuracy::kEP, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kSO>) {
                return AccFeaT(Accuracy::kEP, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kDR>) {
                return AccFeaT(Accuracy::kEP, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTA>) {
                return AccFeaT(Accuracy::kLA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGE>) {
                return AccFeaT(Accuracy::kLA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSO>) {
                return AccFeaT(Accuracy::kLA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDR>) {
                return AccFeaT(Accuracy::kLA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

        if constexpr (Acc == Accuracy::kLA) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTA>) {
                return AccFeaT(Accuracy::kLA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGE>) {
                return AccFeaT(Accuracy::kLA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSO>) {
                return AccFeaT(Accuracy::kLA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDR>) {
                return AccFeaT(Accuracy::kLA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

        if constexpr (Acc == Accuracy::kHA) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

    } // if constexpr (std::is_sa

    if constexpr (std::is_same_v<std::complex<float>, Tin> || std::is_same_v<std::complex<float>, Tout>) {
        if constexpr (Acc == Accuracy::kEP) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kTA>) {
                return AccFeaT(Accuracy::kEP, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kGE>) {
                return AccFeaT(Accuracy::kEP, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kSO>) {
                return AccFeaT(Accuracy::kEP, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kDR>) {
                return AccFeaT(Accuracy::kEP, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTA>) {
                return AccFeaT(Accuracy::kLA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGE>) {
                return AccFeaT(Accuracy::kLA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSO>) {
                return AccFeaT(Accuracy::kLA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDR>) {
                return AccFeaT(Accuracy::kLA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

        if constexpr (Acc == Accuracy::kLA) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTA>) {
                return AccFeaT(Accuracy::kLA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGE>) {
                return AccFeaT(Accuracy::kLA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSO>) {
                return AccFeaT(Accuracy::kLA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDR>) {
                return AccFeaT(Accuracy::kLA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

        if constexpr (Acc == Accuracy::kHA) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

    } // if constexpr (std::is_sa

    if constexpr (std::is_same_v<std::complex<double>, Tin> || std::is_same_v<std::complex<double>, Tout>) {
        if constexpr (Acc == Accuracy::kEP) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kTA>) {
                return AccFeaT(Accuracy::kEP, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kGE>) {
                return AccFeaT(Accuracy::kEP, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kSO>) {
                return AccFeaT(Accuracy::kEP, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kDR>) {
                return AccFeaT(Accuracy::kEP, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kEP, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kEP, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTA>) {
                return AccFeaT(Accuracy::kLA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGE>) {
                return AccFeaT(Accuracy::kLA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSO>) {
                return AccFeaT(Accuracy::kLA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDR>) {
                return AccFeaT(Accuracy::kLA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

        if constexpr (Acc == Accuracy::kLA) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTA>) {
                return AccFeaT(Accuracy::kLA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGE>) {
                return AccFeaT(Accuracy::kLA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSO>) {
                return AccFeaT(Accuracy::kLA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDR>) {
                return AccFeaT(Accuracy::kLA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kLA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kLA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

        if constexpr (Acc == Accuracy::kHA) {
            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTA>) {
                return AccFeaT(Accuracy::kHA, Feature::kTA);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGE>) {
                return AccFeaT(Accuracy::kHA, Feature::kGE);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSO>) {
                return AccFeaT(Accuracy::kHA, Feature::kSO);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kTACast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kTACast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kGECast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kGECast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kSOCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kSOCast32);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDR>) {
                return AccFeaT(Accuracy::kHA, Feature::kDR);
            } // if constexpr (Exists<Fun

            if constexpr (Exists<Func, Tin, Tout, Accuracy::kHA, Feature::kDRCast32>) {
                return AccFeaT(Accuracy::kHA, Feature::kDRCast32);
            } // if constexpr (Exists<Fun

        } // if constexpr (Acc == Acc

    } // if constexpr (std::is_sa

    return AccFeaT(Accuracy::kNA, Feature::kNA);
    }();
}; // struct CompileTimeSelect




template <Function Func, typename Tin, typename Tout,
          Accuracy Acc = Accuracy::kNS, Feature Fea = Feature::kNS>
constexpr bool ExistsAtAll =
    CompileTimeSelector<Func, Tin, Tout, Acc, Fea>::result.first !=
    Accuracy::kNA && CompileTimeSelector<Func, Tin, Tout, Acc, Fea>::result
        .second != Feature::kNA;
template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kAbs, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
abs(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAbs, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kAbs, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // abs(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kAcos, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
acos(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAcos, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kAcos, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // acos(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kAcosh, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
acosh(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAcosh, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kAcosh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // acosh(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kAcospi, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
acospi(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAcospi, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kAcospi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // acospi(const Tin* a, Tou

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kAdd, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
add(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAdd, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kAdd, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // add(const Tin* a, const

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kArg, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
arg(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kArg, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kArg, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // arg(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kAsin, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
asin(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAsin, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kAsin, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // asin(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kAsinh, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
asinh(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAsinh, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kAsinh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // asinh(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kAsinpi, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
asinpi(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAsinpi, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kAsinpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // asinpi(const Tin* a, Tou

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kAtan, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
atan(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtan, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kAtan, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // atan(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kAtan2, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
atan2(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtan2, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kAtan2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // atan2(const Tin* a, cons

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kAtan2pi, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
atan2pi(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtan2pi, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kAtan2pi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // atan2pi(const Tin* a, co

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kAtanh, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
atanh(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtanh, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kAtanh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // atanh(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kAtanpi, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
atanpi(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtanpi, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kAtanpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // atanpi(const Tin* a, Tou

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kCbrt, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
cbrt(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCbrt, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kCbrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cbrt(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kCdfnorm, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
cdfnorm(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCdfnorm, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kCdfnorm, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cdfnorm(const Tin* a, To

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kCdfnorminv, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
cdfnorminv(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCdfnorminv, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kCdfnorminv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cdfnorminv(const Tin* a,

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kCeil, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
ceil(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCeil, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kCeil, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // ceil(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kCis, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
cis(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCis, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kCis, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cis(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kConj, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
conj(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kConj, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kConj, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // conj(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kCopysign, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
copysign(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCopysign, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kCopysign, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // copysign(const Tin* a, c

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kCos, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
cos(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCos, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kCos, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cos(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kCosd, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
cosd(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCosd, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kCosd, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cosd(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kCosh, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
cosh(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCosh, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kCosh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cosh(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kCospi, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
cospi(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCospi, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kCospi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cospi(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kDiv, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
div(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kDiv, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kDiv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // div(const Tin* a, const

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kErf, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
erf(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErf, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kErf, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erf(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kErfc, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
erfc(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfc, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kErfc, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfc(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kErfcinv, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
erfcinv(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfcinv, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kErfcinv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfcinv(const Tin* a, To

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kErfcx, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
erfcx(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfcx, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kErfcx, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfcx(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kErfinv, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
erfinv(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfinv, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kErfinv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfinv(const Tin* a, Tou

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kExp, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
exp(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExp, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kExp, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // exp(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kExp10, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
exp10(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExp10, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kExp10, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // exp10(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kExp2, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
exp2(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExp2, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kExp2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // exp2(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kExpint1, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
expint1(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExpint1, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kExpint1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // expint1(const Tin* a, To

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kExpm1, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
expm1(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExpm1, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kExpm1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // expm1(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kFdim, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
fdim(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFdim, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kFdim, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fdim(const Tin* a, const

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kFloor, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
floor(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFloor, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kFloor, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // floor(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kFmax, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
fmax(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFmax, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kFmax, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fmax(const Tin* a, const

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kFmin, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
fmin(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFmin, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kFmin, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fmin(const Tin* a, const

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kFmod, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
fmod(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFmod, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kFmod, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fmod(const Tin* a, const

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kFrac, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
frac(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFrac, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kFrac, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // frac(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kHypot, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
hypot(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kHypot, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kHypot, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // hypot(const Tin* a, cons

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kI0, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
i0(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kI0, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kI0, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // i0(const Tin* a, Tout* y

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kI1, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
i1(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kI1, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kI1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // i1(const Tin* a, Tout* y

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kInv, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
inv(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kInv, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kInv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // inv(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kInvcbrt, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
invcbrt(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kInvcbrt, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kInvcbrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // invcbrt(const Tin* a, To

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kInvsqrt, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
invsqrt(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kInvsqrt, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kInvsqrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // invsqrt(const Tin* a, To

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kJ0, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
j0(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kJ0, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kJ0, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // j0(const Tin* a, Tout* y

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kJ1, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
j1(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kJ1, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kJ1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // j1(const Tin* a, Tout* y

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kJn, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
jn(const Tin* a, Tin b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kJn, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kJn, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // jn(const Tin* a, Tin b,

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kLgamma, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
lgamma(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLgamma, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kLgamma, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // lgamma(const Tin* a, Tou

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kLinearfrac, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
linearfrac(const Tin* a, const Tin* b, Tin c, Tin d, Tin e, Tin f, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLinearfrac, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kLinearfrac, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, c, d, e, f, y);
} // linearfrac(const Tin* a,

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kLn, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
ln(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLn, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kLn, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // ln(const Tin* a, Tout* y

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kLog10, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
log10(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLog10, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kLog10, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // log10(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kLog1p, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
log1p(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLog1p, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kLog1p, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // log1p(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kLog2, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
log2(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLog2, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kLog2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // log2(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kLogb, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
logb(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLogb, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kLogb, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // logb(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kMaxmag, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
maxmag(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMaxmag, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kMaxmag, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // maxmag(const Tin* a, con

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kMinmag, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
minmag(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMinmag, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kMinmag, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // minmag(const Tin* a, con

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kModf, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
modf(const Tin* a, Tout* y, Tout* z, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kModf, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kModf, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y, z);
} // modf(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kMul, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
mul(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMul, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kMul, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // mul(const Tin* a, const

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kMulbyconj, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
mulbyconj(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMulbyconj, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kMulbyconj, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // mulbyconj(const Tin* a,

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kNearbyint, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
nearbyint(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kNearbyint, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kNearbyint, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // nearbyint(const Tin* a,

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kNextafter, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
nextafter(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kNextafter, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kNextafter, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // nextafter(const Tin* a,

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kPow, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
pow(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPow, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kPow, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // pow(const Tin* a, const

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kPow2o3, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
pow2o3(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPow2o3, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kPow2o3, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // pow2o3(const Tin* a, Tou

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kPow3o2, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
pow3o2(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPow3o2, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kPow3o2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // pow3o2(const Tin* a, Tou

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kPowr, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
powr(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPowr, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kPowr, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // powr(const Tin* a, const

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kPowx, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
powx(const Tin* a, Tin b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPowx, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kPowx, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // powx(const Tin* a, Tin b

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kRemainder, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
remainder(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kRemainder, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kRemainder, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // remainder(const Tin* a,

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kRint, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
rint(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kRint, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kRint, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // rint(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kRound, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
round(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kRound, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kRound, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // round(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kSin, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
sin(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSin, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kSin, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sin(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kSincos, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
sincos(const Tin* a, Tout* y, Tout* z, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSincos, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kSincos, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y, z);
} // sincos(const Tin* a, Tou

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kSincospi, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
sincospi(const Tin* a, Tout* y, Tout* z, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSincospi, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kSincospi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y, z);
} // sincospi(const Tin* a, T

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kSind, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
sind(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSind, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kSind, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sind(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kSinh, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
sinh(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSinh, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kSinh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sinh(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kSinpi, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
sinpi(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSinpi, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kSinpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sinpi(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kSqr, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
sqr(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSqr, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kSqr, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sqr(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kSqrt, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
sqrt(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSqrt, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kSqrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sqrt(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kSub, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
sub(const Tin* a, const Tin* b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSub, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kSub, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // sub(const Tin* a, const

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kTan, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
tan(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTan, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kTan, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tan(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kTand, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
tand(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTand, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kTand, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tand(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kTanh, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
tanh(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTanh, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kTanh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tanh(const Tin* a, Tout*

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kTanpi, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
tanpi(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTanpi, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kTanpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tanpi(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kTgamma, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
tgamma(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTgamma, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kTgamma, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tgamma(const Tin* a, Tou

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kTrunc, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
trunc(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTrunc, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kTrunc, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // trunc(const Tin* a, Tout

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kY0, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
y0(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kY0, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kY0, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // y0(const Tin* a, Tout* y

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kY1, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
y1(const Tin* a, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kY1, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kY1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // y1(const Tin* a, Tout* y

template <typename Tin, typename Tout, typename Acc = decltype(DefaultAccuracy)>
typename std::enable_if_t<ExistsAtAll<Function::kYn, Tin, Tout, Acc::value>, oneapi::mkl::vm::status>
yn(const Tin* a, Tin b, Tout* y, Acc const& acc = {}) {
    constexpr auto cts =
    CompileTimeSelector<Function::kYn, Tin, Tout, Acc::value>::result;
    Evaluator<Function::kYn, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // yn(const Tin* a, Tin b,


} // oneapi::mkl::vm::device::detail

namespace oneapi::mkl::vm::device::detail::ep {

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAbs, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
abs(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAbs, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kAbs, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // abs(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAcos, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
acos(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAcos, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kAcos, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // acos(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAcosh, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
acosh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAcosh, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kAcosh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // acosh(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAcospi, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
acospi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAcospi, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kAcospi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // acospi(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAdd, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
add(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAdd, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kAdd, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // add(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kArg, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
arg(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kArg, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kArg, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // arg(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAsin, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
asin(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAsin, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kAsin, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // asin(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAsinh, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
asinh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAsinh, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kAsinh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // asinh(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAsinpi, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
asinpi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAsinpi, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kAsinpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // asinpi(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtan, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
atan(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtan, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kAtan, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // atan(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtan2, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
atan2(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtan2, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kAtan2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // atan2(const Tin* a, cons

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtan2pi, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
atan2pi(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtan2pi, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kAtan2pi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // atan2pi(const Tin* a, co

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtanh, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
atanh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtanh, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kAtanh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // atanh(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtanpi, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
atanpi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtanpi, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kAtanpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // atanpi(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCbrt, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
cbrt(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCbrt, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kCbrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cbrt(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCdfnorm, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
cdfnorm(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCdfnorm, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kCdfnorm, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cdfnorm(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCdfnorminv, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
cdfnorminv(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCdfnorminv, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kCdfnorminv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cdfnorminv(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCeil, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
ceil(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCeil, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kCeil, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // ceil(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCis, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
cis(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCis, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kCis, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cis(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kConj, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
conj(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kConj, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kConj, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // conj(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCopysign, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
copysign(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCopysign, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kCopysign, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // copysign(const Tin* a, c

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCos, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
cos(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCos, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kCos, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cos(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCosd, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
cosd(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCosd, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kCosd, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cosd(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCosh, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
cosh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCosh, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kCosh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cosh(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCospi, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
cospi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCospi, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kCospi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cospi(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kDiv, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
div(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kDiv, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kDiv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // div(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErf, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
erf(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErf, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kErf, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erf(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErfc, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
erfc(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfc, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kErfc, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfc(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErfcinv, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
erfcinv(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfcinv, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kErfcinv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfcinv(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErfcx, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
erfcx(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfcx, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kErfcx, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfcx(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErfinv, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
erfinv(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfinv, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kErfinv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfinv(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExp, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
exp(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExp, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kExp, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // exp(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExp10, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
exp10(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExp10, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kExp10, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // exp10(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExp2, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
exp2(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExp2, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kExp2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // exp2(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExpint1, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
expint1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExpint1, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kExpint1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // expint1(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExpm1, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
expm1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExpm1, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kExpm1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // expm1(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFdim, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
fdim(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFdim, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kFdim, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fdim(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFloor, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
floor(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFloor, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kFloor, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // floor(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFmax, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
fmax(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFmax, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kFmax, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fmax(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFmin, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
fmin(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFmin, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kFmin, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fmin(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFmod, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
fmod(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFmod, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kFmod, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fmod(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFrac, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
frac(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFrac, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kFrac, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // frac(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kHypot, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
hypot(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kHypot, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kHypot, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // hypot(const Tin* a, cons

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kI0, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
i0(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kI0, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kI0, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // i0(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kI1, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
i1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kI1, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kI1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // i1(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kInv, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
inv(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kInv, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kInv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // inv(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kInvcbrt, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
invcbrt(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kInvcbrt, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kInvcbrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // invcbrt(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kInvsqrt, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
invsqrt(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kInvsqrt, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kInvsqrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // invsqrt(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kJ0, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
j0(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kJ0, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kJ0, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // j0(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kJ1, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
j1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kJ1, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kJ1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // j1(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kJn, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
jn(const Tin* a, Tin b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kJn, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kJn, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // jn(const Tin* a, Tin b,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLgamma, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
lgamma(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLgamma, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kLgamma, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // lgamma(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLinearfrac, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
linearfrac(const Tin* a, const Tin* b, Tin c, Tin d, Tin e, Tin f, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLinearfrac, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kLinearfrac, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, c, d, e, f, y);
} // linearfrac(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLn, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
ln(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLn, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kLn, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // ln(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLog10, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
log10(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLog10, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kLog10, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // log10(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLog1p, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
log1p(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLog1p, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kLog1p, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // log1p(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLog2, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
log2(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLog2, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kLog2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // log2(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLogb, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
logb(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLogb, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kLogb, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // logb(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kMaxmag, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
maxmag(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMaxmag, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kMaxmag, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // maxmag(const Tin* a, con

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kMinmag, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
minmag(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMinmag, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kMinmag, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // minmag(const Tin* a, con

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kModf, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
modf(const Tin* a, Tout* y, Tout* z) {
    constexpr auto cts =
    CompileTimeSelector<Function::kModf, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kModf, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y, z);
} // modf(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kMul, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
mul(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMul, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kMul, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // mul(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kMulbyconj, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
mulbyconj(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMulbyconj, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kMulbyconj, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // mulbyconj(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kNearbyint, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
nearbyint(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kNearbyint, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kNearbyint, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // nearbyint(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kNextafter, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
nextafter(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kNextafter, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kNextafter, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // nextafter(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPow, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
pow(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPow, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kPow, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // pow(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPow2o3, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
pow2o3(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPow2o3, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kPow2o3, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // pow2o3(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPow3o2, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
pow3o2(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPow3o2, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kPow3o2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // pow3o2(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPowr, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
powr(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPowr, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kPowr, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // powr(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPowx, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
powx(const Tin* a, Tin b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPowx, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kPowx, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // powx(const Tin* a, Tin b

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kRemainder, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
remainder(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kRemainder, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kRemainder, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // remainder(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kRint, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
rint(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kRint, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kRint, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // rint(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kRound, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
round(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kRound, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kRound, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // round(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSin, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
sin(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSin, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kSin, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sin(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSincos, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
sincos(const Tin* a, Tout* y, Tout* z) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSincos, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kSincos, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y, z);
} // sincos(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSincospi, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
sincospi(const Tin* a, Tout* y, Tout* z) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSincospi, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kSincospi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y, z);
} // sincospi(const Tin* a, T

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSind, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
sind(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSind, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kSind, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sind(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSinh, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
sinh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSinh, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kSinh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sinh(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSinpi, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
sinpi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSinpi, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kSinpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sinpi(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSqr, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
sqr(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSqr, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kSqr, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sqr(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSqrt, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
sqrt(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSqrt, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kSqrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sqrt(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSub, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
sub(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSub, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kSub, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // sub(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTan, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
tan(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTan, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kTan, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tan(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTand, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
tand(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTand, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kTand, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tand(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTanh, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
tanh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTanh, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kTanh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tanh(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTanpi, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
tanpi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTanpi, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kTanpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tanpi(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTgamma, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
tgamma(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTgamma, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kTgamma, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tgamma(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTrunc, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
trunc(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTrunc, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kTrunc, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // trunc(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kY0, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
y0(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kY0, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kY0, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // y0(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kY1, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
y1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kY1, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kY1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // y1(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kYn, Tin, Tout, Accuracy::kEP>, oneapi::mkl::vm::status>
yn(const Tin* a, Tin b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kYn, Tin, Tout, Accuracy::kEP>::result;
    Evaluator<Function::kYn, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // yn(const Tin* a, Tin b,


} // oneapi::mkl::vm::device::detail::ep

namespace oneapi::mkl::vm::device::detail::la {

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAbs, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
abs(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAbs, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kAbs, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // abs(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAcos, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
acos(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAcos, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kAcos, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // acos(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAcosh, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
acosh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAcosh, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kAcosh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // acosh(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAcospi, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
acospi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAcospi, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kAcospi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // acospi(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAdd, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
add(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAdd, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kAdd, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // add(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kArg, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
arg(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kArg, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kArg, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // arg(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAsin, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
asin(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAsin, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kAsin, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // asin(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAsinh, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
asinh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAsinh, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kAsinh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // asinh(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAsinpi, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
asinpi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAsinpi, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kAsinpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // asinpi(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtan, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
atan(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtan, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kAtan, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // atan(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtan2, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
atan2(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtan2, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kAtan2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // atan2(const Tin* a, cons

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtan2pi, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
atan2pi(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtan2pi, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kAtan2pi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // atan2pi(const Tin* a, co

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtanh, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
atanh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtanh, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kAtanh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // atanh(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtanpi, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
atanpi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtanpi, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kAtanpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // atanpi(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCbrt, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
cbrt(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCbrt, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kCbrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cbrt(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCdfnorm, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
cdfnorm(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCdfnorm, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kCdfnorm, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cdfnorm(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCdfnorminv, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
cdfnorminv(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCdfnorminv, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kCdfnorminv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cdfnorminv(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCeil, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
ceil(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCeil, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kCeil, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // ceil(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCis, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
cis(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCis, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kCis, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cis(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kConj, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
conj(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kConj, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kConj, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // conj(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCopysign, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
copysign(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCopysign, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kCopysign, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // copysign(const Tin* a, c

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCos, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
cos(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCos, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kCos, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cos(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCosd, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
cosd(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCosd, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kCosd, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cosd(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCosh, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
cosh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCosh, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kCosh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cosh(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCospi, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
cospi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCospi, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kCospi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cospi(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kDiv, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
div(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kDiv, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kDiv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // div(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErf, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
erf(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErf, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kErf, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erf(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErfc, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
erfc(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfc, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kErfc, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfc(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErfcinv, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
erfcinv(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfcinv, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kErfcinv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfcinv(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErfcx, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
erfcx(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfcx, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kErfcx, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfcx(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErfinv, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
erfinv(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfinv, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kErfinv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfinv(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExp, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
exp(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExp, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kExp, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // exp(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExp10, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
exp10(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExp10, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kExp10, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // exp10(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExp2, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
exp2(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExp2, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kExp2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // exp2(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExpint1, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
expint1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExpint1, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kExpint1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // expint1(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExpm1, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
expm1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExpm1, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kExpm1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // expm1(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFdim, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
fdim(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFdim, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kFdim, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fdim(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFloor, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
floor(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFloor, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kFloor, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // floor(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFmax, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
fmax(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFmax, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kFmax, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fmax(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFmin, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
fmin(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFmin, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kFmin, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fmin(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFmod, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
fmod(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFmod, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kFmod, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fmod(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFrac, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
frac(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFrac, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kFrac, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // frac(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kHypot, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
hypot(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kHypot, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kHypot, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // hypot(const Tin* a, cons

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kI0, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
i0(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kI0, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kI0, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // i0(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kI1, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
i1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kI1, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kI1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // i1(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kInv, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
inv(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kInv, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kInv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // inv(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kInvcbrt, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
invcbrt(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kInvcbrt, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kInvcbrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // invcbrt(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kInvsqrt, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
invsqrt(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kInvsqrt, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kInvsqrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // invsqrt(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kJ0, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
j0(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kJ0, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kJ0, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // j0(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kJ1, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
j1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kJ1, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kJ1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // j1(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kJn, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
jn(const Tin* a, Tin b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kJn, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kJn, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // jn(const Tin* a, Tin b,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLgamma, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
lgamma(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLgamma, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kLgamma, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // lgamma(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLinearfrac, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
linearfrac(const Tin* a, const Tin* b, Tin c, Tin d, Tin e, Tin f, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLinearfrac, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kLinearfrac, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, c, d, e, f, y);
} // linearfrac(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLn, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
ln(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLn, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kLn, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // ln(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLog10, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
log10(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLog10, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kLog10, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // log10(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLog1p, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
log1p(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLog1p, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kLog1p, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // log1p(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLog2, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
log2(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLog2, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kLog2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // log2(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLogb, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
logb(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLogb, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kLogb, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // logb(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kMaxmag, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
maxmag(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMaxmag, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kMaxmag, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // maxmag(const Tin* a, con

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kMinmag, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
minmag(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMinmag, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kMinmag, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // minmag(const Tin* a, con

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kModf, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
modf(const Tin* a, Tout* y, Tout* z) {
    constexpr auto cts =
    CompileTimeSelector<Function::kModf, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kModf, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y, z);
} // modf(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kMul, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
mul(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMul, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kMul, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // mul(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kMulbyconj, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
mulbyconj(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMulbyconj, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kMulbyconj, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // mulbyconj(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kNearbyint, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
nearbyint(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kNearbyint, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kNearbyint, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // nearbyint(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kNextafter, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
nextafter(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kNextafter, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kNextafter, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // nextafter(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPow, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
pow(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPow, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kPow, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // pow(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPow2o3, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
pow2o3(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPow2o3, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kPow2o3, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // pow2o3(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPow3o2, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
pow3o2(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPow3o2, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kPow3o2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // pow3o2(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPowr, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
powr(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPowr, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kPowr, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // powr(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPowx, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
powx(const Tin* a, Tin b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPowx, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kPowx, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // powx(const Tin* a, Tin b

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kRemainder, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
remainder(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kRemainder, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kRemainder, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // remainder(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kRint, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
rint(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kRint, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kRint, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // rint(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kRound, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
round(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kRound, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kRound, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // round(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSin, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
sin(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSin, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kSin, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sin(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSincos, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
sincos(const Tin* a, Tout* y, Tout* z) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSincos, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kSincos, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y, z);
} // sincos(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSincospi, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
sincospi(const Tin* a, Tout* y, Tout* z) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSincospi, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kSincospi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y, z);
} // sincospi(const Tin* a, T

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSind, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
sind(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSind, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kSind, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sind(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSinh, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
sinh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSinh, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kSinh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sinh(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSinpi, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
sinpi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSinpi, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kSinpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sinpi(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSqr, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
sqr(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSqr, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kSqr, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sqr(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSqrt, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
sqrt(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSqrt, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kSqrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sqrt(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSub, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
sub(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSub, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kSub, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // sub(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTan, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
tan(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTan, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kTan, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tan(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTand, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
tand(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTand, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kTand, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tand(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTanh, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
tanh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTanh, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kTanh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tanh(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTanpi, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
tanpi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTanpi, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kTanpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tanpi(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTgamma, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
tgamma(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTgamma, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kTgamma, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tgamma(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTrunc, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
trunc(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTrunc, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kTrunc, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // trunc(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kY0, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
y0(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kY0, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kY0, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // y0(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kY1, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
y1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kY1, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kY1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // y1(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kYn, Tin, Tout, Accuracy::kLA>, oneapi::mkl::vm::status>
yn(const Tin* a, Tin b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kYn, Tin, Tout, Accuracy::kLA>::result;
    Evaluator<Function::kYn, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // yn(const Tin* a, Tin b,


} // oneapi::mkl::vm::device::detail::la

namespace oneapi::mkl::vm::device::detail::ha {

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAbs, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
abs(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAbs, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kAbs, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // abs(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAcos, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
acos(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAcos, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kAcos, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // acos(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAcosh, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
acosh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAcosh, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kAcosh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // acosh(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAcospi, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
acospi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAcospi, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kAcospi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // acospi(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAdd, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
add(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAdd, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kAdd, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // add(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kArg, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
arg(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kArg, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kArg, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // arg(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAsin, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
asin(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAsin, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kAsin, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // asin(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAsinh, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
asinh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAsinh, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kAsinh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // asinh(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAsinpi, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
asinpi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAsinpi, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kAsinpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // asinpi(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtan, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
atan(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtan, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kAtan, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // atan(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtan2, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
atan2(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtan2, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kAtan2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // atan2(const Tin* a, cons

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtan2pi, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
atan2pi(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtan2pi, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kAtan2pi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // atan2pi(const Tin* a, co

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtanh, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
atanh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtanh, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kAtanh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // atanh(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kAtanpi, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
atanpi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kAtanpi, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kAtanpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // atanpi(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCbrt, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
cbrt(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCbrt, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kCbrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cbrt(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCdfnorm, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
cdfnorm(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCdfnorm, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kCdfnorm, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cdfnorm(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCdfnorminv, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
cdfnorminv(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCdfnorminv, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kCdfnorminv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cdfnorminv(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCeil, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
ceil(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCeil, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kCeil, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // ceil(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCis, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
cis(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCis, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kCis, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cis(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kConj, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
conj(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kConj, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kConj, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // conj(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCopysign, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
copysign(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCopysign, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kCopysign, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // copysign(const Tin* a, c

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCos, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
cos(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCos, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kCos, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cos(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCosd, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
cosd(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCosd, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kCosd, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cosd(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCosh, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
cosh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCosh, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kCosh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cosh(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kCospi, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
cospi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kCospi, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kCospi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // cospi(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kDiv, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
div(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kDiv, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kDiv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // div(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErf, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
erf(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErf, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kErf, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erf(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErfc, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
erfc(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfc, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kErfc, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfc(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErfcinv, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
erfcinv(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfcinv, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kErfcinv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfcinv(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErfcx, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
erfcx(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfcx, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kErfcx, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfcx(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kErfinv, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
erfinv(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kErfinv, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kErfinv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // erfinv(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExp, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
exp(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExp, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kExp, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // exp(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExp10, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
exp10(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExp10, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kExp10, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // exp10(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExp2, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
exp2(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExp2, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kExp2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // exp2(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExpint1, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
expint1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExpint1, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kExpint1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // expint1(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kExpm1, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
expm1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kExpm1, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kExpm1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // expm1(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFdim, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
fdim(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFdim, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kFdim, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fdim(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFloor, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
floor(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFloor, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kFloor, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // floor(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFmax, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
fmax(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFmax, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kFmax, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fmax(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFmin, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
fmin(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFmin, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kFmin, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fmin(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFmod, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
fmod(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFmod, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kFmod, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // fmod(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kFrac, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
frac(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kFrac, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kFrac, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // frac(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kHypot, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
hypot(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kHypot, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kHypot, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // hypot(const Tin* a, cons

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kI0, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
i0(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kI0, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kI0, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // i0(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kI1, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
i1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kI1, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kI1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // i1(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kInv, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
inv(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kInv, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kInv, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // inv(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kInvcbrt, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
invcbrt(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kInvcbrt, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kInvcbrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // invcbrt(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kInvsqrt, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
invsqrt(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kInvsqrt, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kInvsqrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // invsqrt(const Tin* a, To

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kJ0, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
j0(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kJ0, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kJ0, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // j0(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kJ1, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
j1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kJ1, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kJ1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // j1(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kJn, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
jn(const Tin* a, Tin b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kJn, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kJn, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // jn(const Tin* a, Tin b,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLgamma, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
lgamma(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLgamma, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kLgamma, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // lgamma(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLinearfrac, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
linearfrac(const Tin* a, const Tin* b, Tin c, Tin d, Tin e, Tin f, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLinearfrac, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kLinearfrac, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, c, d, e, f, y);
} // linearfrac(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLn, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
ln(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLn, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kLn, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // ln(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLog10, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
log10(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLog10, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kLog10, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // log10(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLog1p, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
log1p(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLog1p, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kLog1p, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // log1p(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLog2, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
log2(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLog2, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kLog2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // log2(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kLogb, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
logb(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kLogb, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kLogb, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // logb(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kMaxmag, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
maxmag(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMaxmag, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kMaxmag, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // maxmag(const Tin* a, con

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kMinmag, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
minmag(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMinmag, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kMinmag, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // minmag(const Tin* a, con

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kModf, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
modf(const Tin* a, Tout* y, Tout* z) {
    constexpr auto cts =
    CompileTimeSelector<Function::kModf, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kModf, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y, z);
} // modf(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kMul, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
mul(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMul, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kMul, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // mul(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kMulbyconj, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
mulbyconj(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kMulbyconj, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kMulbyconj, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // mulbyconj(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kNearbyint, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
nearbyint(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kNearbyint, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kNearbyint, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // nearbyint(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kNextafter, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
nextafter(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kNextafter, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kNextafter, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // nextafter(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPow, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
pow(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPow, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kPow, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // pow(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPow2o3, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
pow2o3(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPow2o3, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kPow2o3, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // pow2o3(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPow3o2, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
pow3o2(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPow3o2, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kPow3o2, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // pow3o2(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPowr, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
powr(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPowr, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kPowr, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // powr(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kPowx, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
powx(const Tin* a, Tin b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kPowx, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kPowx, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // powx(const Tin* a, Tin b

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kRemainder, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
remainder(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kRemainder, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kRemainder, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // remainder(const Tin* a,

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kRint, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
rint(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kRint, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kRint, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // rint(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kRound, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
round(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kRound, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kRound, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // round(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSin, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
sin(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSin, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kSin, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sin(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSincos, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
sincos(const Tin* a, Tout* y, Tout* z) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSincos, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kSincos, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y, z);
} // sincos(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSincospi, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
sincospi(const Tin* a, Tout* y, Tout* z) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSincospi, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kSincospi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y, z);
} // sincospi(const Tin* a, T

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSind, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
sind(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSind, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kSind, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sind(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSinh, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
sinh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSinh, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kSinh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sinh(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSinpi, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
sinpi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSinpi, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kSinpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sinpi(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSqr, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
sqr(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSqr, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kSqr, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sqr(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSqrt, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
sqrt(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSqrt, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kSqrt, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // sqrt(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kSub, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
sub(const Tin* a, const Tin* b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kSub, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kSub, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // sub(const Tin* a, const

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTan, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
tan(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTan, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kTan, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tan(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTand, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
tand(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTand, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kTand, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tand(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTanh, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
tanh(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTanh, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kTanh, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tanh(const Tin* a, Tout*

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTanpi, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
tanpi(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTanpi, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kTanpi, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tanpi(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTgamma, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
tgamma(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTgamma, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kTgamma, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // tgamma(const Tin* a, Tou

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kTrunc, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
trunc(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kTrunc, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kTrunc, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // trunc(const Tin* a, Tout

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kY0, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
y0(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kY0, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kY0, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // y0(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kY1, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
y1(const Tin* a, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kY1, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kY1, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, y);
} // y1(const Tin* a, Tout* y

template <typename Tin, typename Tout>
typename std::enable_if_t<ExistsAtAll<Function::kYn, Tin, Tout, Accuracy::kHA>, oneapi::mkl::vm::status>
yn(const Tin* a, Tin b, Tout* y) {
    constexpr auto cts =
    CompileTimeSelector<Function::kYn, Tin, Tout, Accuracy::kHA>::result;
    Evaluator<Function::kYn, Tin, Tout, cts.first, cts.second> EV;
    return EV(a, b, y);
} // yn(const Tin* a, Tin b,


} // oneapi::mkl::vm::device::detail::ha

namespace oneapi::mkl::vm::device::detail::rts {

static inline
oneapi::mkl::vm::status abs(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status abs(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status abs(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status abs(const std::complex<float>* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status abs(const std::complex<double>* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::abs(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status acos(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::acos(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::acos(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::acos(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::acos(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status acos(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::acos(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::acos(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::acos(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::acos(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status acos(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::acos(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::acos(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::acos(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::acos(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status acos(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status acos(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status acosh(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::acosh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::acosh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::acosh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::acosh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status acosh(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::acosh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::acosh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::acosh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::acosh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status acosh(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::acosh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::acosh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::acosh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::acosh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status acosh(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status acosh(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status acospi(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::acospi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::acospi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::acospi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::acospi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status acospi(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::acospi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::acospi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::acospi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::acospi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status acospi(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::acospi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::acospi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::acospi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::acospi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status add(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status add(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status add(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status add(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status add(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::add(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status arg(const std::complex<float>* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::arg(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::arg(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::arg(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::arg(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status arg(const std::complex<double>* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::arg(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::arg(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::arg(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::arg(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status asin(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::asin(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::asin(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::asin(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::asin(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status asin(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::asin(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::asin(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::asin(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::asin(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status asin(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::asin(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::asin(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::asin(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::asin(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status asin(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status asin(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status asinh(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::asinh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::asinh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::asinh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::asinh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status asinh(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::asinh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::asinh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::asinh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::asinh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status asinh(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::asinh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::asinh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::asinh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::asinh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status asinh(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status asinh(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status asinpi(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::asinpi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::asinpi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::asinpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::asinpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status asinpi(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::asinpi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::asinpi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::asinpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::asinpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status asinpi(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::asinpi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::asinpi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::asinpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::asinpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status atan(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atan(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atan(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atan(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atan(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atan(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atan(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atan(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atan(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atan(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atan(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atan(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atan(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atan(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atan(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atan(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atan(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status atan2(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atan2(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atan2(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atan2(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atan2(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atan2(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atan2(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atan2(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atan2(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atan2(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atan2(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atan2(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atan2(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atan2(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atan2(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status atan2pi(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atan2pi(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atan2pi(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atan2pi(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atan2pi(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atan2pi(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atan2pi(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atan2pi(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atan2pi(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atan2pi(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atan2pi(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atan2pi(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atan2pi(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atan2pi(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atan2pi(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status atanh(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atanh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atanh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atanh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atanh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atanh(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atanh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atanh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atanh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atanh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atanh(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atanh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atanh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atanh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atanh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atanh(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atanh(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status atanpi(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atanpi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atanpi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atanpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atanpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atanpi(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atanpi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atanpi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atanpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atanpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status atanpi(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::atanpi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::atanpi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::atanpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::atanpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status cbrt(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cbrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cbrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cbrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cbrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cbrt(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cbrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cbrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cbrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cbrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cbrt(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cbrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cbrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cbrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cbrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status cdfnorm(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cdfnorm(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cdfnorm(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cdfnorm(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cdfnorm(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cdfnorm(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cdfnorm(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cdfnorm(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cdfnorm(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cdfnorm(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cdfnorm(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cdfnorm(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cdfnorm(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cdfnorm(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cdfnorm(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status cdfnorminv(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cdfnorminv(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cdfnorminv(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cdfnorminv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cdfnorminv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cdfnorminv(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cdfnorminv(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cdfnorminv(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cdfnorminv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cdfnorminv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cdfnorminv(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cdfnorminv(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cdfnorminv(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cdfnorminv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cdfnorminv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status ceil(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::ceil(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::ceil(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::ceil(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::ceil(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status ceil(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::ceil(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::ceil(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::ceil(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::ceil(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status ceil(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::ceil(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::ceil(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::ceil(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::ceil(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status cis(const float* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cis(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cis(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cis(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cis(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cis(const double* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cis(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cis(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cis(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cis(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status conj(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::conj(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::conj(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::conj(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::conj(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status conj(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::conj(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::conj(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::conj(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::conj(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status copysign(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::copysign(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::copysign(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::copysign(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::copysign(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status copysign(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::copysign(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::copysign(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::copysign(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::copysign(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status copysign(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::copysign(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::copysign(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::copysign(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::copysign(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status cos(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cos(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cos(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cos(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cos(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cos(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cos(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cos(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cos(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cos(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cos(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cos(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cos(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cos(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cos(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cos(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cos(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status cosd(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cosd(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cosd(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cosd(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cosd(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cosd(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cosd(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cosd(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cosd(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cosd(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cosd(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cosd(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cosd(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cosd(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cosd(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status cosh(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cosh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cosh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cosh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cosh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cosh(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cosh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cosh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cosh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cosh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cosh(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cosh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cosh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cosh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cosh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cosh(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cosh(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status cospi(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cospi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cospi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cospi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cospi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cospi(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cospi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cospi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cospi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cospi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status cospi(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::cospi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::cospi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::cospi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::cospi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status div(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status div(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status div(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status div(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status div(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::div(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status erf(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erf(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erf(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erf(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erf(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status erf(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erf(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erf(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erf(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erf(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status erf(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erf(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erf(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erf(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erf(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status erfc(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erfc(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erfc(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erfc(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erfc(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status erfc(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erfc(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erfc(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erfc(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erfc(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status erfc(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erfc(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erfc(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erfc(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erfc(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status erfcinv(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erfcinv(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erfcinv(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erfcinv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erfcinv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status erfcinv(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erfcinv(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erfcinv(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erfcinv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erfcinv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status erfcinv(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erfcinv(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erfcinv(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erfcinv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erfcinv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status erfcx(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erfcx(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erfcx(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erfcx(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erfcx(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status erfcx(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erfcx(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erfcx(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erfcx(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erfcx(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status erfcx(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erfcx(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erfcx(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erfcx(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erfcx(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status erfinv(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erfinv(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erfinv(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erfinv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erfinv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status erfinv(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erfinv(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erfinv(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erfinv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erfinv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status erfinv(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::erfinv(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::erfinv(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::erfinv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::erfinv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status exp(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status exp(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status exp(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status exp(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status exp(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::exp(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status exp10(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::exp10(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::exp10(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::exp10(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::exp10(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status exp10(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::exp10(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::exp10(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::exp10(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::exp10(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status exp10(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::exp10(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::exp10(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::exp10(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::exp10(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status exp2(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::exp2(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::exp2(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::exp2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::exp2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status exp2(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::exp2(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::exp2(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::exp2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::exp2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status exp2(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::exp2(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::exp2(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::exp2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::exp2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status expint1(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status expint1(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status expint1(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status expm1(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::expm1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::expm1(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::expm1(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::expm1(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status expm1(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::expm1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::expm1(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::expm1(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::expm1(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status expm1(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::expm1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::expm1(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::expm1(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::expm1(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status fdim(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::fdim(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::fdim(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::fdim(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::fdim(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status fdim(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::fdim(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::fdim(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::fdim(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::fdim(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status fdim(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::fdim(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::fdim(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::fdim(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::fdim(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status floor(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::floor(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::floor(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::floor(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::floor(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status floor(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::floor(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::floor(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::floor(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::floor(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status floor(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::floor(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::floor(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::floor(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::floor(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status fmax(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::fmax(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::fmax(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::fmax(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::fmax(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status fmax(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::fmax(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::fmax(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::fmax(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::fmax(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status fmax(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::fmax(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::fmax(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::fmax(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::fmax(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status fmin(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::fmin(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::fmin(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::fmin(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::fmin(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status fmin(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::fmin(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::fmin(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::fmin(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::fmin(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status fmin(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::fmin(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::fmin(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::fmin(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::fmin(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status fmod(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::fmod(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::fmod(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::fmod(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::fmod(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status fmod(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::fmod(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::fmod(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::fmod(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::fmod(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status fmod(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::fmod(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::fmod(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::fmod(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::fmod(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status frac(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::frac(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::frac(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::frac(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::frac(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status frac(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::frac(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::frac(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::frac(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::frac(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status frac(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::frac(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::frac(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::frac(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::frac(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status hypot(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::hypot(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::hypot(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::hypot(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::hypot(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status hypot(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::hypot(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::hypot(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::hypot(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::hypot(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status hypot(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::hypot(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::hypot(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::hypot(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::hypot(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status i0(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::i0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::i0(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::i0(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::i0(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status i0(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::i0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::i0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status i0(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::i0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::i0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status i1(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::i1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::i1(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::i1(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::i1(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status i1(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::i1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::i1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status i1(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::i1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::i1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status inv(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::inv(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::inv(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::inv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::inv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status inv(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::inv(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::inv(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::inv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::inv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status inv(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::inv(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::inv(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::inv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::inv(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status invcbrt(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::invcbrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::invcbrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::invcbrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::invcbrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status invcbrt(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::invcbrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::invcbrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::invcbrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::invcbrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status invcbrt(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::invcbrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::invcbrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::invcbrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::invcbrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status invsqrt(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::invsqrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::invsqrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::invsqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::invsqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status invsqrt(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::invsqrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::invsqrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::invsqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::invsqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status invsqrt(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::invsqrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::invsqrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::invsqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::invsqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status j0(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::j0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::j0(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::j0(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::j0(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status j0(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::j0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::j0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status j0(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::j0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::j0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status j1(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::j1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::j1(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::j1(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::j1(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status j1(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::j1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::j1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status j1(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::j1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::j1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status jn(const sycl::half* a, sycl::half b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::jn(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::jn(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::jn(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::jn(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status jn(const float* a, float b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::jn(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::jn(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status jn(const double* a, double b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::jn(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::jn(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status lgamma(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::lgamma(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::lgamma(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::lgamma(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::lgamma(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status lgamma(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::lgamma(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::lgamma(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::lgamma(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::lgamma(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status lgamma(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::lgamma(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::lgamma(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status linearfrac(const sycl::half* a, const sycl::half* b, sycl::half c, sycl::half d, sycl::half e, sycl::half f, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status linearfrac(const float* a, const float* b, float c, float d, float e, float f, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status linearfrac(const double* a, const double* b, double c, double d, double e, double f, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status ln(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status ln(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status ln(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status ln(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status ln(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::ln(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status log10(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::log10(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::log10(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::log10(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::log10(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status log10(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::log10(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::log10(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::log10(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::log10(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status log10(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::log10(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::log10(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::log10(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::log10(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status log10(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status log10(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status log1p(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::log1p(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::log1p(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::log1p(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::log1p(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status log1p(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::log1p(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::log1p(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::log1p(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::log1p(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status log1p(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::log1p(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::log1p(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::log1p(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::log1p(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status log2(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::log2(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::log2(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::log2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::log2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status log2(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::log2(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::log2(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::log2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::log2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status log2(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::log2(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::log2(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::log2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::log2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status logb(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::logb(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::logb(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::logb(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::logb(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status logb(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::logb(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::logb(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::logb(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::logb(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status logb(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::logb(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::logb(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::logb(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::logb(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status maxmag(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::maxmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::maxmag(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::maxmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::maxmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status maxmag(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::maxmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::maxmag(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::maxmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::maxmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status maxmag(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::maxmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::maxmag(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::maxmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::maxmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status minmag(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::minmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::minmag(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::minmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::minmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status minmag(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::minmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::minmag(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::minmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::minmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status minmag(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::minmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::minmag(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::minmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::minmag(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status modf(const sycl::half* a, sycl::half* y, sycl::half* z, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::modf(a, y, z, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::modf(a, y, z, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::modf(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::modf(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status modf(const float* a, float* y, float* z, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::modf(a, y, z, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::modf(a, y, z, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::modf(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::modf(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status modf(const double* a, double* y, double* z, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::modf(a, y, z, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::modf(a, y, z, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::modf(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::modf(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status mul(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status mul(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status mul(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status mul(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status mul(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::mul(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status mulbyconj(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::mulbyconj(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::mulbyconj(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::mulbyconj(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::mulbyconj(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status mulbyconj(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::mulbyconj(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::mulbyconj(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::mulbyconj(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::mulbyconj(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status nearbyint(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::nearbyint(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::nearbyint(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::nearbyint(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::nearbyint(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status nearbyint(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::nearbyint(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::nearbyint(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::nearbyint(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::nearbyint(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status nearbyint(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::nearbyint(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::nearbyint(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::nearbyint(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::nearbyint(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status nextafter(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::nextafter(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::nextafter(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::nextafter(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::nextafter(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status nextafter(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::nextafter(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::nextafter(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::nextafter(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::nextafter(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status nextafter(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::nextafter(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::nextafter(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::nextafter(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::nextafter(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status pow(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::pow(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::pow(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::pow(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::pow(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status pow(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::pow(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::pow(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::pow(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::pow(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status pow(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::pow(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::pow(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::pow(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::pow(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status pow(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status pow(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status pow2o3(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::pow2o3(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::pow2o3(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::pow2o3(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::pow2o3(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status pow2o3(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::pow2o3(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::pow2o3(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::pow2o3(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::pow2o3(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status pow2o3(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::pow2o3(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::pow2o3(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::pow2o3(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::pow2o3(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status pow3o2(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::pow3o2(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::pow3o2(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::pow3o2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::pow3o2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status pow3o2(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::pow3o2(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::pow3o2(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::pow3o2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::pow3o2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status pow3o2(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::pow3o2(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::pow3o2(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::pow3o2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::pow3o2(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status powr(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::powr(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::powr(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::powr(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::powr(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status powr(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::powr(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::powr(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::powr(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::powr(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status powr(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::powr(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::powr(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::powr(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::powr(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status powx(const sycl::half* a, sycl::half b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::powx(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::powx(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::powx(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::powx(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status powx(const float* a, float b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::powx(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::powx(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::powx(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::powx(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status powx(const double* a, double b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::powx(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::powx(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::powx(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::powx(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status powx(const std::complex<float>* a, std::complex<float> b, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status powx(const std::complex<double>* a, std::complex<double> b, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status remainder(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::remainder(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::remainder(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::remainder(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::remainder(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status remainder(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::remainder(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::remainder(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::remainder(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::remainder(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status remainder(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::remainder(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::remainder(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::remainder(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::remainder(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status rint(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::rint(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::rint(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::rint(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::rint(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status rint(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::rint(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::rint(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::rint(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::rint(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status rint(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::rint(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::rint(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::rint(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::rint(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status round(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::round(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::round(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::round(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::round(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status round(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::round(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::round(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::round(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::round(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status round(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::round(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::round(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::round(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::round(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status sin(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sin(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sin(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sin(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sin(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sin(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sin(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sin(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sin(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sin(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sin(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sin(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sin(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sin(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sin(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sin(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sin(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status sincos(const sycl::half* a, sycl::half* y, sycl::half* z, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sincos(a, y, z, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sincos(a, y, z, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sincos(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sincos(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sincos(const float* a, float* y, float* z, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sincos(a, y, z, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sincos(a, y, z, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sincos(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sincos(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sincos(const double* a, double* y, double* z, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sincos(a, y, z, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sincos(a, y, z, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sincos(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sincos(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status sincospi(const sycl::half* a, sycl::half* y, sycl::half* z, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sincospi(a, y, z, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sincospi(a, y, z, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sincospi(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sincospi(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sincospi(const float* a, float* y, float* z, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sincospi(a, y, z, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sincospi(a, y, z, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sincospi(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sincospi(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sincospi(const double* a, double* y, double* z, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sincospi(a, y, z, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sincospi(a, y, z, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sincospi(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sincospi(a, y, z, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status sind(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sind(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sind(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sind(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sind(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sind(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sind(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sind(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sind(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sind(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sind(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sind(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sind(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sind(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sind(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status sinh(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sinh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sinh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sinh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sinh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sinh(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sinh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sinh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sinh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sinh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sinh(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sinh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sinh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sinh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sinh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sinh(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sinh(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status sinpi(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sinpi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sinpi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sinpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sinpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sinpi(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sinpi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sinpi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sinpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sinpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sinpi(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sinpi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sinpi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sinpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sinpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status sqr(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sqr(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sqr(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sqr(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sqr(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sqr(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sqr(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sqr(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sqr(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sqr(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sqr(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sqr(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sqr(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sqr(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sqr(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status sqrt(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sqrt(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sqrt(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sqrt(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sqrt(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sqrt(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status sub(const sycl::half* a, const sycl::half* b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sub(const float* a, const float* b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sub(const double* a, const double* b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sub(const std::complex<float>* a, const std::complex<float>* b, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status sub(const std::complex<double>* a, const std::complex<double>* b, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::sub(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status tan(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tan(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tan(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tan(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tan(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tan(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tan(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tan(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tan(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tan(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tan(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tan(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tan(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tan(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tan(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tan(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tan(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status tand(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tand(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tand(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tand(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tand(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tand(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tand(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tand(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tand(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tand(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tand(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tand(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tand(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tand(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tand(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status tanh(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tanh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tanh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tanh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tanh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tanh(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tanh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tanh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tanh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tanh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tanh(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tanh(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tanh(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tanh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tanh(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tanh(const std::complex<float>* a, std::complex<float>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tanh(const std::complex<double>* a, std::complex<double>* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: break; /// no implementation yet
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: break; /// no implementation yet
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status tanpi(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tanpi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tanpi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tanpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tanpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tanpi(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tanpi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tanpi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tanpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tanpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tanpi(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tanpi(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tanpi(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tanpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tanpi(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status tgamma(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tgamma(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tgamma(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tgamma(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tgamma(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tgamma(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tgamma(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tgamma(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::tgamma(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::tgamma(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status tgamma(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::tgamma(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::tgamma(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::tgamma(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status trunc(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::trunc(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::trunc(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::trunc(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::trunc(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status trunc(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::trunc(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::trunc(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::trunc(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::trunc(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status trunc(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::trunc(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::trunc(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::trunc(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::trunc(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status y0(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::y0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::y0(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::y0(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::y0(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status y0(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::y0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::y0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status y0(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::y0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::y0(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status y1(const sycl::half* a, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::y1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::y1(a, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::y1(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::y1(a, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status y1(const float* a, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::y1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::y1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status y1(const double* a, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::y1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::y1(a, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status


static inline
oneapi::mkl::vm::status yn(const sycl::half* a, sycl::half b, sycl::half* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::yn(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: r = detail::yn(a, b, y, oneapi::mkl::vm::device::detail::mode::la); break;
        case oneapi::mkl::vm::mode::ha: r = detail::yn(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
        default: r = detail::yn(a, b, y, oneapi::mkl::vm::device::detail::mode::ha); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status yn(const float* a, float b, float* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::yn(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::yn(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status

static inline
oneapi::mkl::vm::status yn(const double* a, double b, double* y, oneapi::mkl::vm::mode mode = oneapi::mkl::vm::mode::ha) {
    auto r = oneapi::mkl::vm::status::computation_empty;
    switch (mode & oneapi::mkl::vm::mode::accuracy_mask) {
        case oneapi::mkl::vm::mode::ep: r = detail::yn(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
        case oneapi::mkl::vm::mode::la: break; /// no implementation yet
        case oneapi::mkl::vm::mode::ha: break; /// no implementation yet
        default: r = detail::yn(a, b, y, oneapi::mkl::vm::device::detail::mode::ep); break;
    } // switch (mode & oneapi::m

    return r;
} // oneapi::mkl::vm::status



} // oneapi::mkl::vm::device::detail::rts



#endif // ONEAPI_MKL_VM_DEVICE_DETAIL_API_HPP
