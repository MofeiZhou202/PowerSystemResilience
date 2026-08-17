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

#ifndef DISTRIBUTED_DFT_HPP
#define DISTRIBUTED_DFT_HPP

#include <cinttypes>    // std::int64_t
#include <vector>       // std::vector
#include <sycl/sycl.hpp>  // sycl::
#include "../dft.hpp"

namespace oneapi::mkl::experimental::dft {

// Compute functions which will be friends of the distributed_descriptor class
template <typename descriptor_type, typename data_type>
sycl::event compute_forward(descriptor_type &desc, data_type *inout, const std::vector<sycl::event> &dependencies = {});

template<typename descriptor_type, typename input_type, typename output_type>
sycl::event compute_forward(descriptor_type &desc, input_type *in, output_type *out, const std::vector<sycl::event> &dependencies = {});

template <typename descriptor_type, typename data_type>
sycl::event compute_backward(descriptor_type &desc, data_type *inout, const std::vector<sycl::event> &dependencies = {});

template<typename descriptor_type, typename input_type, typename output_type>
sycl::event compute_backward(descriptor_type &desc, input_type *in, output_type *out, const std::vector<sycl::event> &dependencies = {});

enum class distributed_config_param {
    fwd_divided_dimension,
    bwd_divided_dimension,
    fwd_distribution,
    bwd_distribution,
    fwd_local_data_size_bytes,
    bwd_local_data_size_bytes
};

template <oneapi::mkl::dft::precision prec, oneapi::mkl::dft::domain dom>
class distributed_descriptor_impl;

template <oneapi::mkl::dft::precision prec, oneapi::mkl::dft::domain dom>
class distributed_descriptor {
    using real_scalar_t =
                 std::conditional_t<prec == oneapi::mkl::dft::precision::DOUBLE,
                                            double, float>;
  public:
    distributed_descriptor(MPI_Comm Comm, std::vector<std::int64_t> dimensions);
    ~distributed_descriptor();
    distributed_descriptor(const distributed_descriptor&) = delete;
    distributed_descriptor& operator=(const distributed_descriptor&) = delete;
    distributed_descriptor(distributed_descriptor&&) = delete;
    distributed_descriptor& operator=(distributed_descriptor&&) = delete;

    void commit(sycl::queue &q);

    void set_value(oneapi::mkl::dft::config_param, oneapi::mkl::dft::config_value);
    void set_value(oneapi::mkl::dft::config_param, std::int64_t);
    void set_value(distributed_config_param, std::int64_t);
    void set_value(oneapi::mkl::dft::config_param, const std::vector<std::int64_t>&);
    template <typename T, std::enable_if_t<std::is_integral_v<T>, bool> = true>
    void set_value(distributed_config_param param, T value) {
        set_value(param, static_cast<std::int64_t>(value));
    }
    void set_value(oneapi::mkl::dft::config_param, real_scalar_t);
    template <typename T, std::enable_if_t<std::is_integral_v<T>, bool> = true>
    void set_value(oneapi::mkl::dft::config_param param, T value) {
        set_value(param, static_cast<std::int64_t>(value));
    }
    template <typename T, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
    void set_value(oneapi::mkl::dft::config_param param, T value) {
        set_value(param, static_cast<real_scalar_t>(value));
    }
    void set_value(distributed_config_param param,
                   const std::vector<std::int64_t> &lower_bound,
                   const std::vector<std::int64_t> &upper_bound,
                   const std::vector<std::int64_t> &strides);

    void get_value(oneapi::mkl::dft::config_param, oneapi::mkl::dft::config_value*) const;
    void get_value(oneapi::mkl::dft::config_param, oneapi::mkl::dft::domain*) const;
    void get_value(oneapi::mkl::dft::config_param, oneapi::mkl::dft::precision*) const;
    void get_value(oneapi::mkl::dft::config_param, std::int64_t*) const;
    void get_value(distributed_config_param, std::int64_t*) const;
    void get_value(oneapi::mkl::dft::config_param, std::vector<std::int64_t>*) const;
    void get_value(oneapi::mkl::dft::config_param, real_scalar_t*) const;
    void get_value(distributed_config_param param,
                   std::vector<std::int64_t> *lower_bound,
                   std::vector<std::int64_t> *upper_bound,
                   std::vector<std::int64_t> *strides) const;

    // Supports only USM
    template <typename descriptor_type, typename data_type>
    friend sycl::event compute_forward(descriptor_type &desc,
                                       data_type *inout,
                                       const std::vector<sycl::event> &dependencies);

    template<typename descriptor_type, typename input_type, typename output_type>
    friend sycl::event compute_forward(descriptor_type &desc,
                                       input_type *in, output_type *out,
                                       const std::vector<sycl::event> &dependencies);

    template <typename descriptor_type, typename data_type>
    friend sycl::event compute_backward(descriptor_type &desc,
                                        data_type *inout,
                                        const std::vector<sycl::event> &dependencies);

    template<typename descriptor_type, typename input_type, typename output_type>
    friend sycl::event compute_backward(descriptor_type &desc,
                                        input_type *in, output_type *out,
                                        const std::vector<sycl::event> &dependencies);

  private:
    std::unique_ptr<distributed_descriptor_impl<prec, dom>> impl;
};

}

#endif
