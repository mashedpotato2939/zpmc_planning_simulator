#ifndef CONTROL_UTILS_INTERPOLATION_HPP_
#define CONTROL_UTILS_INTERPOLATION_HPP_

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace control_utils::interpolation{

/*三次样条插值*/
std::vector<double> splineByAkima(
    const std::vector<double> & base_keys, const std::vector<double> & base_values,
    const std::vector<double> & query_keys);

/*线性插值*/
double lerp(const double src_val, const double dst_val, const double ratio);

std::vector<double> lerp(
    const std::vector<double> & base_keys, const std::vector<double> & base_values,
    const std::vector<double> & query_keys);

double lerp(
    const std::vector<double> & base_keys, const std::vector<double> & base_values,
    const double query_key);

/*零阶保持*/
std::vector<size_t> calc_closest_segment_indices(
    const std::vector<double> & base_keys, const std::vector<double> & query_keys,
    const double overlap_threshold = 1e-3);

std::vector<double> zero_order_hold(
    const std::vector<double> & base_keys, const std::vector<double> & base_values,
    const std::vector<size_t> & closest_segment_indices);

std::vector<double> zero_order_hold(
    const std::vector<double> & base_keys, const std::vector<double> & base_values,
    const std::vector<double> & query_keys, const double overlap_threshold = 1e-3);

/*对于插值索引和值的检查*/
std::vector<double> validateKeys(
    const std::vector<double> & base_keys, const std::vector<double> & query_keys);

template <class T>
void validateKeysAndValues(
    const std::vector<double> & base_keys, const std::vector<T> & base_values);

}


#endif