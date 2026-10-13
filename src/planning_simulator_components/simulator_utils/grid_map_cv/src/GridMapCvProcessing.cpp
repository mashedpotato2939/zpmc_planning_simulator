/*
 * GridMapCvProcessing.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_cv/GridMapCvProcessing.hpp"
#include "grid_map_cv/GridMapCvConverter.hpp"

namespace grid_map
{

GridMapCvProcessing::GridMapCvProcessing()
{
}

GridMapCvProcessing::~GridMapCvProcessing()
{
}

bool GridMapCvProcessing::changeResolution(
  const grid_map::GridMap & gridMapSource,
  grid_map::GridMap & gridMapResult,
  const double resolution,
  const int interpolationAlgorithm)
{
  grid_map::GridMap gridMapSourceCopy(gridMapSource);
  gridMapSourceCopy.convertToDefaultStartIndex();
  const double sizeFactor = gridMapSourceCopy.getResolution() / resolution;
  bool firstLayer = true;
  for (const auto & layer : gridMapSourceCopy.getLayers()) {
    cv::Mat imageSource, imageResult;
    
    // Helper function to find min/max of finite values (compatible with ROS Galactic)
    const grid_map::Matrix& data = gridMapSourceCopy.get(layer);
    float minValue = std::numeric_limits<float>::max();
    float maxValue = std::numeric_limits<float>::lowest();
    
    for (int i = 0; i < data.rows(); ++i) {
      for (int j = 0; j < data.cols(); ++j) {
        if (std::isfinite(data(i, j))) {
          minValue = std::min(minValue, data(i, j));
          maxValue = std::max(maxValue, data(i, j));
        }
      }
    }
    
    const bool hasNaN = gridMapSourceCopy.get(layer).hasNaN();
    bool result;
    if (hasNaN) {
      result = grid_map::GridMapCvConverter::toImage<uint16_t, 4>(
        gridMapSourceCopy, layer, CV_16UC4,
        minValue, maxValue, imageSource);
    } else {
      result = grid_map::GridMapCvConverter::toImage<uint16_t, 1>(
        gridMapSourceCopy, layer, CV_16UC1,
        minValue, maxValue, imageSource);
    }
    if (!result) {return false;}
    cv::resize(
      imageSource, imageResult, cv::Size(
        0.0,
        0.0), sizeFactor, sizeFactor,
      interpolationAlgorithm);
    if (firstLayer) {
      if (!GridMapCvConverter::initializeFromImage(
          imageResult, resolution, gridMapResult,
          gridMapSourceCopy.getPosition()))
      {
        return false;
      }
      firstLayer = false;
    }
    if (hasNaN) {
      result = grid_map::GridMapCvConverter::addLayerFromImage<uint16_t, 4>(
        imageResult, layer,
        gridMapResult, minValue,
        maxValue);
    } else {
      result = grid_map::GridMapCvConverter::addLayerFromImage<uint16_t, 1>(
        imageResult, layer,
        gridMapResult, minValue,
        maxValue);
    }
    if (!result) {return false;}
  }
  gridMapResult.setFrameId(gridMapSourceCopy.getFrameId());
  gridMapResult.setTimestamp(gridMapSourceCopy.getTimestamp());
  gridMapResult.setBasicLayers(gridMapSourceCopy.getBasicLayers());
  return true;
}

}  // namespace grid_map
