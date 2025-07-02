/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2025, Enrico Piazza, Università degli Studi di Milano
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <asb_voxel_grid/voxel_grid.hpp>

namespace asb_voxel_grid {
VoxelGrid::VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z) : logger(rclcpp::get_logger("voxel_grid")) {

    size_x_ = size_x;
    size_y_ = size_y;
    size_z_ = size_z;

    if (size_z_ > 16) {
        RCLCPP_INFO(logger, "Error, this implementation can only support up to 16 z values (%d)", size_z_);
        size_z_ = 16;
    }

    data_ = new uint32_t[size_x_ * size_y_];
    uint32_t unknown_col = ~((uint32_t) 0) >> 16;
    uint32_t *col = data_;
    for (unsigned int i = 0; i < size_x_ * size_y_; ++i) {
        *col = unknown_col;
        ++col;
    }
}

void VoxelGrid::resize(unsigned int size_x, unsigned int size_y, unsigned int size_z) {
    // if we're not actually changing the size, we can just reset things
    if (size_x == size_x_ && size_y == size_y_ && size_z == size_z_) {
        reset();
        return;
    }

    delete[] data_;
    size_x_ = size_x;
    size_y_ = size_y;
    size_z_ = size_z;

    if (size_z_ > 16) {
        RCLCPP_INFO(logger, "Error, this implementation can only support up to 16 z values (%d)", size_z);
        size_z_ = 16;
    }

    data_ = new uint32_t[size_x_ * size_y_];
    uint32_t unknown_col = ~((uint32_t) 0) >> 16;
    uint32_t *col = data_;
    for (unsigned int i = 0; i < size_x_ * size_y_; ++i) {
        *col = unknown_col;
        ++col;
    }
}

VoxelGrid::~VoxelGrid() {

    delete[] data_;
}

void VoxelGrid::reset() {

    uint32_t unknown_col = ~((uint32_t) 0) >> 16;
    uint32_t *col = data_;
    for (unsigned int i = 0; i < size_x_ * size_y_; ++i) {
        *col = unknown_col;
        ++col;
    }
}

void VoxelGrid::clearVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length, unsigned int min_length) {

    if (x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1 >= size_x_ || y1 >= size_y_ || z1 >= size_z_) {
        RCLCPP_DEBUG(logger, "Error, line endpoint out of bounds. "
                             "(%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)", x0, y0, z0, x1, y1, z1, size_x_, size_y_, size_z_);
        return;
    }

    ClearVoxel cv(data_);
    raytraceLine(cv, x0, y0, z0, x1, y1, z1, max_length, min_length);
}

VoxelStatus VoxelGrid::getVoxelColumn(unsigned int x, unsigned int y, unsigned int unknown_threshold, unsigned int marked_threshold) {

    if (x >= size_x_ || y >= size_y_) {
        RCLCPP_DEBUG(logger, "Error, voxel out of bounds. (%d, %d)\n", x, y);
        return UNKNOWN;
    }

    uint32_t *col = &data_[y * size_x_ + x];

    unsigned int unknown_bits = uint16_t(*col >> 16) ^ uint16_t(*col);
    unsigned int marked_bits = *col >> 16;

    // check if the number of marked bits qualifies the col as marked
    if (!bitsBelowThreshold(marked_bits, marked_threshold)) {
        return MARKED;
    }

    // check if the number of unknown bits qualifies the col as unknown
    if (!bitsBelowThreshold(unknown_bits, unknown_threshold)) {
        return UNKNOWN;
    }

    return FREE;
}

void VoxelGrid::transferToCostmap(const unsigned char & lethal_cost, const unsigned char & free_cost, const unsigned char & unknown_cost, const unsigned int & unknown_threshold, const unsigned int & marked_threshold, unsigned char * costmap__) {

    for (unsigned int y = 0; y < size_y_; y++) {
        for (unsigned int x = 0; x < size_x_; x++) {
            unsigned int offset = y * size_x_ + x;
            auto column = getVoxelColumn(x, y, unknown_threshold, marked_threshold);

            // overwrite the costmap if the voxel grid column is free or marked (
            if (column == asb_voxel_grid::MARKED) {
                costmap__[offset] = lethal_cost;
            } else if (column == asb_voxel_grid::FREE) {
                costmap__[offset] = free_cost;
            }

        }
    }
}

unsigned int VoxelGrid::sizeX() {

    return size_x_;
}

unsigned int VoxelGrid::sizeY() {

    return size_y_;
}

unsigned int VoxelGrid::sizeZ() {

    return size_z_;
}

}  // namespace asb_voxel_grid
