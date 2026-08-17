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

#ifndef _DFT_HPP_
#define _DFT_HPP_

#include <cinttypes>    // std::int64_t
#include <vector>       // std::vector
#include <sycl/sycl.hpp>  // sycl::
#include "oneapi/mkl/dft/spec.hpp"
#include "mkl_dfti.h"   // DFTI_DESCRIPTOR_HANDLE

typedef struct SYCL_DFTI_DESCRIPTOR* SYCL_DFTI_DESCRIPTOR_HANDLE;

namespace oneapi {
namespace mkl {
namespace dft {

enum class precision {
    SINGLE = DFTI_SINGLE,
    DOUBLE = DFTI_DOUBLE
};

enum class domain {
    REAL = DFTI_REAL,
    COMPLEX = DFTI_COMPLEX
};

enum class config_param {
    FORWARD_DOMAIN,
    DIMENSION,
    LENGTHS,
    PRECISION,
    FORWARD_SCALE,
    BACKWARD_SCALE,
    NUMBER_OF_TRANSFORMS,
    COMPLEX_STORAGE,
    PLACEMENT,
    FWD_DISTANCE,
    BWD_DISTANCE,
    WORKSPACE,
    COMMIT_STATUS,
    THREAD_LIMIT,
    DESTROY_INPUT,
    WORKSPACE_ESTIMATE_BYTES,
    WORKSPACE_BYTES,
    FWD_STRIDES,
    BWD_STRIDES,
    WORKSPACE_PLACEMENT,        // alias for WORKSPACE
    WORKSPACE_EXTERNAL_BYTES    // alias for WORKSPACE_BYTES
};

enum class config_value {
    COMMITTED,
    UNCOMMITTED,
    COMPLEX_COMPLEX,
    REAL_REAL,
    INPLACE,
    NOT_INPLACE,
    WORKSPACE_AUTOMATIC,        // alias for WORKSPACE_INTERNAL
    ALLOW,
    AVOID,
    WORKSPACE_INTERNAL,
    WORKSPACE_EXTERNAL
};

// Compute functions which will be friends with the descriptor class
template<typename descriptor_type, typename data_type>
void compute_forward(
    descriptor_type &desc,
    sycl::buffer<data_type, 1> &inout);

template<typename descriptor_type, typename input_type, typename output_type>
void compute_forward(
    descriptor_type &desc,
    sycl::buffer<input_type, 1> &in,
    sycl::buffer<output_type, 1> &out);

template<typename descriptor_type, typename data_type>
void compute_backward(
    descriptor_type &desc,
    sycl::buffer<data_type, 1> &inout);

template<typename descriptor_type, typename input_type, typename output_type>
void compute_backward(
    descriptor_type &desc,
    sycl::buffer<input_type, 1> &in,
    sycl::buffer<output_type, 1> &out);

template <typename descriptor_type, typename data_type>
sycl::event compute_forward(
    descriptor_type &desc,
    data_type *inout,
    const std::vector<sycl::event> &dependencies = {});

template<typename descriptor_type, typename input_type, typename output_type>
sycl::event compute_forward(
    descriptor_type &desc,
    input_type *in,
    output_type *out,
    const std::vector<sycl::event> &dependencies = {});

template <typename descriptor_type, typename data_type>
sycl::event compute_backward(
    descriptor_type &desc,
    data_type *inout,
    const std::vector<sycl::event> &dependencies = {});

template<typename descriptor_type, typename input_type, typename output_type>
sycl::event compute_backward(
    descriptor_type &desc,
    input_type *in,
    output_type *out,
    const std::vector<sycl::event> &dependencies = {});

template <precision prec, domain dom>
class descriptor {
    using real_scalar_t = std::conditional_t<prec == precision::DOUBLE, double, float>;
 public:
    // initializes the DFT descriptor for a multi-dimensional DFT
    descriptor(std::vector<std::int64_t> dimensions);
    // initializes the DFT descriptor for a one-dimensional DFT
    descriptor(std::int64_t length);
    ~descriptor();
    descriptor(const descriptor&) = delete;
    descriptor& operator=(const descriptor&) = delete;
    descriptor(descriptor&&);
    descriptor& operator=(descriptor&&);

    void commit(sycl::queue &in);

    // in place forward computation; buffer API
    template <typename descriptor_type, typename data_type>
    friend void compute_forward(
        descriptor_type &desc,
        sycl::buffer<data_type, 1> &inout);
    // out of place forward computation; buffer API
    template<typename descriptor_type, typename input_type, typename output_type>
    friend void compute_forward(
        descriptor_type &desc,
        sycl::buffer<input_type, 1> &in,
        sycl::buffer<output_type, 1> &out);
    // in place backward computation; buffer API
    template <typename descriptor_type, typename data_type>
    friend void compute_backward(
        descriptor_type &desc,
        sycl::buffer<data_type, 1> &inout);
    // out of place backward computation; buffer API
    template<typename descriptor_type, typename input_type, typename output_type>
    friend void compute_backward(
        descriptor_type &desc,
        sycl::buffer<input_type, 1> &in,
        sycl::buffer<output_type, 1> &out);
    // in place forward computation; USM API
    template <typename descriptor_type, typename data_type>
    friend sycl::event compute_forward(
        descriptor_type &desc,
        data_type *inout,
        const std::vector<sycl::event> &dependencies);
    // out of place forward computation; USM API
    template<typename descriptor_type, typename input_type, typename output_type>
    friend sycl::event compute_forward(
        descriptor_type &desc,
        input_type *in,
        output_type *out,
        const std::vector<sycl::event> &dependencies);
    // in place backward computation; USM API
    template <typename descriptor_type, typename data_type>
    friend sycl::event compute_backward(
        descriptor_type &desc,
        data_type *inout,
        const std::vector<sycl::event> &dependencies);
    // out of place backward computation; USM API
    template<typename descriptor_type, typename input_type, typename output_type>
    friend sycl::event compute_backward(
        descriptor_type &desc,
        input_type *in,
        output_type *out,
        const std::vector<sycl::event> &dependencies);

    // configuration-setting member functions:
    void set_value(config_param, config_value);
    void set_value(config_param, std::int64_t);
    void set_value(config_param, real_scalar_t);
    void set_value(config_param, const std::vector<std::int64_t>&);
    template <typename T, std::enable_if_t<std::is_integral_v<T>, bool> = true>
    void set_value(config_param param, T value) {
        set_value(param, static_cast<std::int64_t>(value));
    }
    template <typename T, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
    void set_value(config_param param, T value) {
        set_value(param, static_cast<real_scalar_t>(value));
    }
    // configuration-querying member functions:
    void get_value(config_param, config_value*) const;
    void get_value(config_param, domain*) const;
    void get_value(config_param, precision*) const;
    void get_value(config_param, std::int64_t*) const;
    void get_value(config_param, real_scalar_t*) const;
    void get_value(config_param, std::vector<std::int64_t>*) const;

    template<typename data_type>
    void set_workspace(sycl::buffer<data_type, 1> &workspace);
    template<typename data_type>
    void set_workspace(data_type *workspace);

 private:
    DFTI_DESCRIPTOR_HANDLE handle;
    SYCL_DFTI_DESCRIPTOR_HANDLE device_handle;
    sycl::buffer<SYCL_DFTI_DESCRIPTOR_HANDLE, 1> handle_buffer;
};

}  // namespace dft
}  // namespace mkl
} // namespace oneapi

#endif  /* _DFT_HPP_ */
