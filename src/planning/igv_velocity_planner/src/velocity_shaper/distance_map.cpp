// Copyright 2026 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "igv_velocity_planner/velocity_shaper/distance_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace igv_velocity_planner::velocity_shaper
{

namespace
{
constexpr float kBig = std::numeric_limits<float>::max() / 4.0f;

// 1-D squared Euclidean distance transform of a sampled function `f` (length n),
// writing the result into `d`. Felzenszwalb & Huttenlocher lower-envelope of
// parabolas — O(n). `v` and `z` are scratch buffers of size n and n+1.
void dt1d(const std::vector<float> & f, std::vector<float> & d, int n,
          std::vector<int> & v, std::vector<float> & z)
{
  int k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<float>::max();
  z[1] = std::numeric_limits<float>::max();
  for (int q = 1; q < n; ++q) {
    float s = ((f[q] + static_cast<float>(q) * q) -
               (f[v[k]] + static_cast<float>(v[k]) * v[k])) /
              (2.0f * static_cast<float>(q - v[k]));
    while (s <= z[k]) {
      --k;
      s = ((f[q] + static_cast<float>(q) * q) -
           (f[v[k]] + static_cast<float>(v[k]) * v[k])) /
          (2.0f * static_cast<float>(q - v[k]));
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<float>::max();
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < static_cast<float>(q)) ++k;
    const float dq = static_cast<float>(q - v[k]);
    d[q] = dq * dq + f[v[k]];
  }
}
}  // namespace

DistanceMap::DistanceMap(
  const nav_msgs::msg::OccupancyGrid & grid, int8_t occupancy_threshold,
  uint32_t cx0, uint32_t cy0, uint32_t cx1, uint32_t cy1)
{
  res_ = grid.info.resolution;
  const uint32_t w = grid.info.width;
  const uint32_t h = grid.info.height;
  if (res_ <= 0.0 || w == 0 || h == 0 || grid.data.empty()) return;

  // Clamp the ROI to the grid.
  cx0 = std::min(cx0, w);
  cy0 = std::min(cy0, h);
  cx1 = std::min(cx1, w);
  cy1 = std::min(cy1, h);
  if (cx1 <= cx0 || cy1 <= cy0) return;

  cx0_ = cx0;
  cy0_ = cy0;
  roi_w_ = cx1 - cx0;
  roi_h_ = cy1 - cy0;

  const size_t roi = static_cast<size_t>(roi_w_) * roi_h_;
  std::vector<float> f(roi, kBig);   // squared-distance seed: 0 at obstacles
  for (uint32_t r = 0; r < roi_h_; ++r) {
    const uint32_t gy = cy0_ + r;
    for (uint32_t c = 0; c < roi_w_; ++c) {
      const uint32_t gx = cx0_ + c;
      const int8_t val = grid.data[static_cast<size_t>(gy) * w + gx];
      if (val >= occupancy_threshold) f[static_cast<size_t>(r) * roi_w_ + c] = 0.0f;
    }
  }

  // Two-pass 2-D EDT: transform columns, then rows.
  const int maxdim = static_cast<int>(std::max(roi_w_, roi_h_));
  std::vector<float> col_in(maxdim), col_out(maxdim);
  std::vector<int> vbuf(maxdim);
  std::vector<float> zbuf(maxdim + 1);

  // Columns (vertical pass over rows).
  for (uint32_t c = 0; c < roi_w_; ++c) {
    for (uint32_t r = 0; r < roi_h_; ++r) col_in[r] = f[static_cast<size_t>(r) * roi_w_ + c];
    dt1d(col_in, col_out, static_cast<int>(roi_h_), vbuf, zbuf);
    for (uint32_t r = 0; r < roi_h_; ++r) f[static_cast<size_t>(r) * roi_w_ + c] = col_out[r];
  }
  // Rows (horizontal pass over columns).
  std::vector<float> row_in(maxdim), row_out(maxdim);
  for (uint32_t r = 0; r < roi_h_; ++r) {
    for (uint32_t c = 0; c < roi_w_; ++c) row_in[c] = f[static_cast<size_t>(r) * roi_w_ + c];
    dt1d(row_in, row_out, static_cast<int>(roi_w_), vbuf, zbuf);
    for (uint32_t c = 0; c < roi_w_; ++c) f[static_cast<size_t>(r) * roi_w_ + c] = row_out[c];
  }

  // Convert squared cell distances to metres.
  dist_.resize(roi);
  for (size_t i = 0; i < roi; ++i) dist_[i] = std::sqrt(f[i]) * static_cast<float>(res_);

  // Saturation distance reported outside the ROI / when no obstacle is in range:
  // the ROI diagonal in metres is an upper bound and always exceeds any clearance.
  saturation_m_ = std::hypot(static_cast<double>(roi_w_), static_cast<double>(roi_h_)) * res_;
  valid_ = true;
}

double DistanceMap::queryMetric(double gx, double gy) const
{
  if (!valid_) return saturation_m_;
  // Grid-local metres → ROI-local fractional cell coordinates.
  const double u = gx / res_ - static_cast<double>(cx0_);
  const double v = gy / res_ - static_cast<double>(cy0_);
  if (u < 0.0 || v < 0.0) return saturation_m_;

  const int c0 = static_cast<int>(std::floor(u));
  const int r0 = static_cast<int>(std::floor(v));
  if (c0 < 0 || r0 < 0 || c0 >= static_cast<int>(roi_w_) || r0 >= static_cast<int>(roi_h_)) {
    return saturation_m_;
  }
  const int c1 = std::min(c0 + 1, static_cast<int>(roi_w_) - 1);
  const int r1 = std::min(r0 + 1, static_cast<int>(roi_h_) - 1);
  const double fu = u - c0;
  const double fv = v - r0;

  const auto at = [&](int c, int r) -> double {
    return dist_[static_cast<size_t>(r) * roi_w_ + c];
  };
  const double d00 = at(c0, r0);
  const double d10 = at(c1, r0);
  const double d01 = at(c0, r1);
  const double d11 = at(c1, r1);
  const double top = d00 + fu * (d10 - d00);
  const double bot = d01 + fu * (d11 - d01);
  return top + fv * (bot - top);
}

}  // namespace igv_velocity_planner::velocity_shaper
