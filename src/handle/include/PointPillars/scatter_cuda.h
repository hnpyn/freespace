#pragma once

namespace lidar {

class ScatterCuda {
 private:
  const int num_pfe_output_channel_;
  const int grid_x_size_;
  const int grid_y_size_;

 public:
  /**
   * @brief Constructor
   * @param[in] num_pfe_output_channel The number of threads to launch cuda kernel,
   *                                   also the pfe output channel
   * @param[in] grid_x_size Number of pillars in x-coordinate
   * @param[in] grid_y_size Number of pillars in y-coordinate
   * @details Captital variables never change after the compile
   */
  ScatterCuda(const int num_pfe_output_channel,
              const int grid_x_size, const int grid_y_size);

  /**
   * @brief Call scatter cuda kernel
   * @param[in] pillar_count The valid number of pillars
   * @param[in] x_coors X-coordinate indexes for corresponding pillars
   * @param[in] y_coors Y-coordinate indexes for corresponding pillars
   * @param[in] pfe_output Output from Pillar Feature Extractor
   * @param[out] scattered_feature Gridmap representation for pillars' feature
   * @details Allocate pillars in gridmap based on index(coordinates)
   * information
   */
  void DoScatterCuda(const int pillar_count, int* voxel_coors,
                     float* pfe_output, float* scattered_feature);
};

}  // namespace lidar
