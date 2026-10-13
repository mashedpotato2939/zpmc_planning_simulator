#include "interpolation.hpp"

namespace control_utils::interpolation{

/*Akima三次样条插值*/
std::vector<double> splineByAkima(
    const std::vector<double> & base_keys, const std::vector<double> & base_values,
    const std::vector<double> & query_keys)
{
    constexpr double epsilon = 1e-5;

    // calculate m
    std::vector<double> m_values;
    for (size_t i = 0; i < base_keys.size() - 1; ++i) {
        const double m_val =
        (base_values.at(i + 1) - base_values.at(i)) / (base_keys.at(i + 1) - base_keys.at(i));
        m_values.push_back(m_val);
    }

    // calculate s
    std::vector<double> s_values;
    for (size_t i = 0; i < base_keys.size(); ++i) {
        if (i == 0) {
        s_values.push_back(m_values.front());
        continue;
        } else if (i == base_keys.size() - 1) {
        s_values.push_back(m_values.back());
        continue;
        } else if (i == 1 || i == base_keys.size() - 2) {
        const double s_val = (m_values.at(i - 1) + m_values.at(i)) / 2.0;
        s_values.push_back(s_val);
        continue;
        }

        const double denom = std::abs(m_values.at(i + 1) - m_values.at(i)) +
                            std::abs(m_values.at(i - 1) - m_values.at(i - 2));
        if (std::abs(denom) < epsilon) {
        const double s_val = (m_values.at(i - 1) + m_values.at(i)) / 2.0;
        s_values.push_back(s_val);
        continue;
        }

        const double s_val = (std::abs(m_values.at(i + 1) - m_values.at(i)) * m_values.at(i - 1) +
                            std::abs(m_values.at(i - 1) - m_values.at(i - 2)) * m_values.at(i)) /
                            denom;
        s_values.push_back(s_val);
    }

    // calculate cubic coefficients
    std::vector<double> a;
    std::vector<double> b;
    std::vector<double> c;
    std::vector<double> d;
    for (size_t i = 0; i < base_keys.size() - 1; ++i) {
        a.push_back(
        (s_values.at(i) + s_values.at(i + 1) - 2.0 * m_values.at(i)) /
        std::pow(base_keys.at(i + 1) - base_keys.at(i), 2));
        b.push_back(
        (3.0 * m_values.at(i) - 2.0 * s_values.at(i) - s_values.at(i + 1)) /
        (base_keys.at(i + 1) - base_keys.at(i)));
        c.push_back(s_values.at(i));
        d.push_back(base_values.at(i));
    }

    // interpolate
    std::vector<double> res;
    size_t j = 0;
    for (const auto & query_key : query_keys) {
        while (base_keys.at(j + 1) < query_key) {
        ++j;
        }

        const double ds = query_key - base_keys.at(j);
        res.push_back(d.at(j) + (c.at(j) + (b.at(j) + a.at(j) * ds) * ds) * ds);
    }
    return res;
}

/*线性插值*/
double lerp(const double src_val, const double dst_val, const double ratio)
{
  return src_val + (dst_val - src_val) * ratio;
}

std::vector<double> lerp(
    const std::vector<double> & base_keys, const std::vector<double> & base_values,
    const std::vector<double> & query_keys)
{
    // throw exception for invalid arguments
    const auto validated_query_keys = validateKeys(base_keys, query_keys);
    validateKeysAndValues(base_keys, base_values);

    // calculate linear interpolation
    std::vector<double> query_values;
    size_t key_index = 0;
    for (const auto query_key : validated_query_keys) {
        while (base_keys.at(key_index + 1) < query_key) {
        ++key_index;
        }

        const double src_val = base_values.at(key_index);
        const double dst_val = base_values.at(key_index + 1);
        const double ratio = (query_key - base_keys.at(key_index)) /
                            (base_keys.at(key_index + 1) - base_keys.at(key_index));

        const double interpolated_val = lerp(src_val, dst_val, ratio);
        query_values.push_back(interpolated_val);
    }

    return query_values;
}

double lerp(
    const std::vector<double> & base_keys, const std::vector<double> & base_values, double query_key)
{
    return lerp(base_keys, base_values, std::vector<double>{query_key}).front();
}

/*零阶保持 ZOH*/
std::vector<size_t> calc_closest_segment_indices(
  const std::vector<double> & base_keys, const std::vector<double> & query_keys,
  const double overlap_threshold)
{
  // throw exception for invalid arguments
  const auto validated_query_keys = validateKeys(base_keys, query_keys);

  std::vector<size_t> closest_segment_indices(validated_query_keys.size());
  size_t closest_segment_idx = 0;
  for (size_t i = 0; i < validated_query_keys.size(); ++i) {
    // Check if query_key is closes to the terminal point of the base keys
    if (base_keys.back() - overlap_threshold < validated_query_keys.at(i)) {
      closest_segment_idx = base_keys.size() - 1;
    } else {
      for (size_t j = base_keys.size() - 1; j > closest_segment_idx; --j) {
        if (
          base_keys.at(j - 1) - overlap_threshold < validated_query_keys.at(i) &&
          validated_query_keys.at(i) < base_keys.at(j)) {
          // find closest segment in base keys
          closest_segment_idx = j - 1;
          break;
        }
      }
    }

    closest_segment_indices.at(i) = closest_segment_idx;
  }

  return closest_segment_indices;
}

std::vector<double> zero_order_hold(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<size_t> & closest_segment_indices)
{
  // throw exception for invalid arguments
  validateKeysAndValues(base_keys, base_values);

  std::vector<double> query_values(closest_segment_indices.size());
  for (size_t i = 0; i < closest_segment_indices.size(); ++i) {
    query_values.at(i) = base_values.at(closest_segment_indices.at(i));
  }

  return query_values;
}

std::vector<double> zero_order_hold(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys, const double overlap_threshold)
{
  return zero_order_hold(
    base_keys, base_values, calc_closest_segment_indices(base_keys, query_keys, overlap_threshold));
}

/*插值工具函数*/
bool isIncreasing(const std::vector<double> & x)
{
  if (x.empty()) {
    throw std::invalid_argument("Points is empty.");
  }

  for (size_t i = 0; i < x.size() - 1; ++i) {
    if (x.at(i) >= x.at(i + 1)) {
      return false;
    }
  }

  return true;
}

bool isNotDecreasing(const std::vector<double> & x)
{
  if (x.empty()) {
    throw std::invalid_argument("Points is empty.");
  }

  for (size_t i = 0; i < x.size() - 1; ++i) {
    if (x.at(i) > x.at(i + 1)) {
      return false;
    }
  }

  return true;
}

std::vector<double> validateKeys(
    const std::vector<double> & base_keys, const std::vector<double> & query_keys)
{
    // when vectors are empty
    if (base_keys.empty() || query_keys.empty()) {
        throw std::invalid_argument("Points is empty.");
    }

    // when size of vectors are less than 2
    if (base_keys.size() < 2) {
        throw std::invalid_argument(
        "The size of points is less than 2. base_keys.size() = " + std::to_string(base_keys.size()));
    }

    // when indices are not sorted
    if (!isIncreasing(base_keys) || !isNotDecreasing(query_keys)) {
        throw std::invalid_argument("Either base_keys or query_keys is not sorted.");
    }

    // when query_keys is out of base_keys (This function does not allow exterior division.)
    constexpr double epsilon = 1e-3;
    if (
        query_keys.front() < base_keys.front() - epsilon ||
        base_keys.back() + epsilon < query_keys.back()) {
        throw std::invalid_argument("query_keys is out of base_keys");
    }

    // NOTE: Due to calculation error of double, a query key may be slightly out of base keys.
    //       Therefore, query keys are cropped here.
    auto validated_query_keys = query_keys;
    validated_query_keys.front() = std::max(validated_query_keys.front(), base_keys.front());
    validated_query_keys.back() = std::min(validated_query_keys.back(), base_keys.back());

    return validated_query_keys;
}

template <class T>
void validateKeysAndValues(
  const std::vector<double> & base_keys, const std::vector<T> & base_values)
{
  // when vectors are empty
  if (base_keys.empty() || base_values.empty()) {
    throw std::invalid_argument("Points is empty.");
  }

  // when size of vectors are less than 2
  if (base_keys.size() < 2 || base_values.size() < 2) {
    throw std::invalid_argument(
      "The size of points is less than 2. base_keys.size() = " + std::to_string(base_keys.size()) +
      ", base_values.size() = " + std::to_string(base_values.size()));
  }

  // when sizes of indices and values are not same
  if (base_keys.size() != base_values.size()) {
    throw std::invalid_argument("The size of base_keys and base_values are not the same.");
  }
}

}