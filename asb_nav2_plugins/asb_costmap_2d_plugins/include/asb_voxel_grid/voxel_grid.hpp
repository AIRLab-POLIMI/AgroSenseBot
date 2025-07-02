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

#ifndef ASB_VOXEL_GRID__VOXEL_GRID_HPP_
#define ASB_VOXEL_GRID__VOXEL_GRID_HPP_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"

/**
 * @class VoxelGrid
 * @brief A 3D grid structure that stores points as an integer array.
 *        X and Y index the array and Z selects which bit of the integer
 *        is used giving a limit of 16 vertical cells.
 */
namespace asb_voxel_grid {

enum VoxelStatus {
    FREE = 0, UNKNOWN = 1, MARKED = 2,
};

class VoxelGrid {
public:
    /**
     * @brief  Constructor for a voxel grid
     * @param size_x The x size of the grid
     * @param size_y The y size of the grid
     * @param size_z The z size of the grid, only sizes <= 16 are supported
     */
    VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z);

    ~VoxelGrid();

    /**
     * @brief  Resizes a voxel grid to the desired size
     * @param size_x The x size of the grid
     * @param size_y The y size of the grid
     * @param size_z The z size of the grid, only sizes <= 16 are supported
     */
    void resize(unsigned int size_x, unsigned int size_y, unsigned int size_z);

    void reset();

    uint32_t *getData() { return data_; }

    inline bool markVoxelInMap(unsigned int x, unsigned int y, unsigned int z, unsigned int marked_threshold) {

        if (x >= size_x_ || y >= size_y_ || z >= size_z_) {
            RCLCPP_DEBUG(logger, "Error, voxel out of bounds.\n");
            return false;
        }

        uint32_t & col = data_[y * size_x_ + x];
        uint32_t full_mask = ((uint32_t) 1 << (z + 16)) | (1 << z);
        col |= full_mask;  // mark voxel, set bits 11
        unsigned int mark_bits = col >> 16;
        return !bitsBelowThreshold(mark_bits, marked_threshold);
    }

    inline bool bitsBelowThreshold(unsigned int n, unsigned int bit_threshold) {

        unsigned int bit_count;
        for (bit_count = 0; n;) {
            ++bit_count;
            if (bit_count > bit_threshold) {
                return false;
            }
            n &= n - 1;  // clear the least significant bit
        }
        return true;
    }

    inline bool isVoxelCleared(unsigned int x, unsigned int y, unsigned int z) {

        if (x >= size_x_ || y >= size_y_ || z >= size_z_) {
            return false;
        }

        uint32_t full_z_mask = (uint32_t) 1 << (z + 16) | (1 << z);
        return (data_[y * size_x_ + x] & full_z_mask) == 0;
    }

    static inline bool isVoxelMarked(unsigned int & data, unsigned int full_z_mask) {

        std::uint32_t mark_bits = full_z_mask << 16;
        return (mark_bits & data) > 0;
    }

    void clearVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length = UINT_MAX, unsigned int min_length = 0);

    VoxelStatus getVoxelColumn(unsigned int x, unsigned int y, unsigned int unknown_threshold = 0, unsigned int marked_threshold = 0);

//    void transferTo(VoxelGrid & other);

    void transferToCostmap(const unsigned char & lethal_cost, const unsigned char & free_cost, const unsigned char & unknown_cost, const unsigned int & unknown_threshold, const unsigned int & marked_threshold, unsigned char * costmap__);

    unsigned int sizeX();

    unsigned int sizeY();

    unsigned int sizeZ();

    template<class ActionType>
    inline void raytraceLine(ActionType at, double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length = UINT_MAX, unsigned int min_length = 0) {

        // we need to choose how much to scale our dominant dimension, based on the
        // maximum length of the line
        double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
        if ((unsigned int) (dist) < min_length) {
            return;
        }

        double scale, min_x0, min_y0, min_z0;
        if (dist > 0.0) {
            scale = std::min(1.0, max_length / dist);

            // Updating starting point to the point at distance min_length from the initial point
            min_x0 = x0 + (x1 - x0) / dist * min_length;
            min_y0 = y0 + (y1 - y0) / dist * min_length;
            min_z0 = z0 + (z1 - z0) / dist * min_length;
        } else {
            // dist can be 0 if [x0, y0, z0]==[x1, y1, z1].
            // In this case only this voxel should be processed.
            scale = 1.0;
            min_x0 = x0;
            min_y0 = y0;
            min_z0 = z0;
        }

        int dx = int(x1) - int(min_x0);  // NOLINT
        int dy = int(y1) - int(min_y0);  // NOLINT
        int dz = int(z1) - int(min_z0);  // NOLINT

        unsigned int abs_dx = abs(dx);
        unsigned int abs_dy = abs(dy);
        unsigned int abs_dz = abs(dz);

        int offset_dx = sign(dx);
        int offset_dy = sign(dy) * size_x_;
        int offset_dz = sign(dz);

        unsigned int full_z_mask = ((1 << 16) | 1) << (unsigned int) min_z0;
        unsigned int offset = (unsigned int) min_y0 * size_x_ + (unsigned int) min_x0;

        GridOffset grid_off(offset);
        ZOffset z_off(full_z_mask);

        // is x dominant
        if (abs_dx >= max(abs_dy, abs_dz)) {
            int error_y = abs_dx / 2;
            int error_z = abs_dx / 2;

            bresenham3D(at, grid_off, grid_off, z_off, abs_dx, abs_dy, abs_dz, error_y, error_z, offset_dx, offset_dy, offset_dz, offset, full_z_mask, (unsigned int) (scale * abs_dx));
            return;
        }

        // y is dominant
        if (abs_dy >= abs_dz) {
            int error_x = abs_dy / 2;
            int error_z = abs_dy / 2;

            bresenham3D(at, grid_off, grid_off, z_off, abs_dy, abs_dx, abs_dz, error_x, error_z, offset_dy, offset_dx, offset_dz, offset, full_z_mask, (unsigned int) (scale * abs_dy));
            return;
        }

        // otherwise, z is dominant
        int error_x = abs_dz / 2;
        int error_y = abs_dz / 2;

        bresenham3D(at, z_off, grid_off, grid_off, abs_dz, abs_dx, abs_dy, error_x, error_y, offset_dz, offset_dx, offset_dy, offset, full_z_mask, (unsigned int) (scale * abs_dz));
    }

private:
    // the real work is done here... 3D bresenham implementation
    template<class ActionType, class OffA, class OffB, class OffC>
    inline void bresenham3D(ActionType at, OffA off_a, OffB off_b, OffC off_c, unsigned int abs_da, unsigned int abs_db, unsigned int abs_dc, int error_b, int error_c, int offset_a, int offset_b, int offset_c, unsigned int &offset, unsigned int &full_z_mask, unsigned int max_length = UINT_MAX) {

        unsigned int end = std::min(max_length, abs_da);
        bool continue_tracing = true;
        for (unsigned int i = 0; i < end && continue_tracing; ++i) {
            continue_tracing = at(offset, full_z_mask);
            off_a(offset_a);
            error_b += abs_db;
            error_c += abs_dc;
            if ((unsigned int) error_b >= abs_da) {
                off_b(offset_b);
                error_b -= abs_da;
            }
            if ((unsigned int) error_c >= abs_da) {
                off_c(offset_c);
                error_c -= abs_da;
            }
        }
        at(offset, full_z_mask);
    }

    inline int sign(int i) {

        return i > 0 ? 1 : -1;
    }

    inline unsigned int max(unsigned int x, unsigned int y) {

        return x > y ? x : y;
    }

    unsigned int size_x_, size_y_, size_z_;
    uint32_t *data_;
    rclcpp::Logger logger;

    // Aren't functors so much fun... used to recreate the Bresenham macro Eric
    // wrote in the original version, but in "proper" c++

    class ClearVoxel {
    public:
        explicit ClearVoxel(uint32_t *data) : data_(data) {}

        inline bool operator()(unsigned int offset, unsigned int full_z_mask) {

            if(!isVoxelMarked(data_[offset], full_z_mask)){  // keep going (return true) as long as we encounter free or unknown cells
                data_[offset] &= ~(full_z_mask);  // clear unknown and clear cell
                return true;
            }
            return false;  // otherwise return false (so that bresenham stops the ray tracing)
        }

    private:
        uint32_t *data_;
    };

    class GridOffset {
    public:
        explicit GridOffset(unsigned int &offset) : offset_(offset) {}

        inline void operator()(int offset_val) {

            offset_ += offset_val;
        }

    private:
        unsigned int &offset_;
    };

    class ZOffset {
    public:
        explicit ZOffset(unsigned int &full_z_mask) : full_z_mask_(full_z_mask) {}

        inline void operator()(int offset_val) {

            offset_val > 0 ? full_z_mask_ <<= 1 : full_z_mask_ >>= 1;
        }

    private:
        unsigned int &full_z_mask_;
    };
};

}  // namespace asb_voxel_grid

#endif  // ASB_VOXEL_GRID__VOXEL_GRID_HPP_
