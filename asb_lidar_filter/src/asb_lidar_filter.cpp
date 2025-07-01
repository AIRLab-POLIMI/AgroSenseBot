// Copyright 2024 Enrico Piazza, Università degli Studi di Milano
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

#include "asb_lidar_filter/asb_lidar_filter.h"

using std::placeholders::_1;

ASBLidarFilter::ASBLidarFilter() : Node("asb_lidar_filter") {

    this->declare_parameter("base_frame_id", rclcpp::ParameterType::PARAMETER_STRING);
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();

    this->declare_parameter("mask_file_path", rclcpp::ParameterType::PARAMETER_STRING);
    mask_file_path_ = this->get_parameter("mask_file_path").as_string();

    this->declare_parameter("mask_filter_size", rclcpp::ParameterType::PARAMETER_INTEGER);
    mask_filter_size_ = (int) this->get_parameter("mask_filter_size").as_int();

    this->declare_parameter("mask_filter_count", rclcpp::ParameterType::PARAMETER_INTEGER);
    mask_filter_count_ = (unsigned int) this->get_parameter("mask_filter_count").as_int();

    this->declare_parameter("min_range", rclcpp::ParameterType::PARAMETER_DOUBLE);
    min_range_ = (float) this->get_parameter("min_range").as_double();
    min_range_2_ = min_range_*min_range_;

    this->declare_parameter("min_layer", rclcpp::ParameterType::PARAMETER_INTEGER);
    min_layer_from_bottom_ = (int) this->get_parameter("min_layer").as_int();

    this->declare_parameter("max_layer", rclcpp::ParameterType::PARAMETER_INTEGER);
    max_layer_from_bottom_ = (int) this->get_parameter("max_layer").as_int();

    if(min_layer_from_bottom_ > max_layer_from_bottom_) {
        RCLCPP_ERROR(this->get_logger(), "min_layer > max_layer");
    }

    if (load_mask_from_pbm(mask_file_path_, mask_, mask_width_, mask_height_)) {
        reset_mask_ = false;
    } else {
        reset_mask_ = true;
        RCLCPP_ERROR(this->get_logger(), "mask file not found, create a mask by calling the create_mask service");
    }

    // pointcloud filter params
    this->declare_parameter("x_min", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("x_max", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("y_min", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("y_max", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("z_min", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("z_max", rclcpp::ParameterType::PARAMETER_DOUBLE);
    x_min_ = this->get_parameter("x_min").as_double();
    x_max_ = this->get_parameter("x_max").as_double();
    y_min_ = this->get_parameter("y_min").as_double();
    y_max_ = this->get_parameter("y_max").as_double();
    z_min_ = this->get_parameter("z_min").as_double();
    z_max_ = this->get_parameter("z_max").as_double();

    // scan filter params
    this->declare_parameter("scan_min_height", rclcpp::ParameterType::PARAMETER_DOUBLE);
    scan_min_height_ = this->get_parameter("scan_min_height").as_double();
    this->declare_parameter("scan_max_height", rclcpp::ParameterType::PARAMETER_DOUBLE);
    scan_max_height_ = this->get_parameter("scan_max_height").as_double();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    points_in_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("points_in", rclcpp::SensorDataQoS().durability_volatile().reliable(), std::bind(&ASBLidarFilter::points_in_callback, this, _1));

    points_out_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_out", rclcpp::SensorDataQoS().durability_volatile().reliable());

    points_out_no_ground_ceiling_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_out_no_ground_ceiling", rclcpp::SensorDataQoS().durability_volatile().reliable());

    heartbeat_publisher_ = this->create_publisher<std_msgs::msg::Header>("heartbeat_out", rclcpp::SensorDataQoS().durability_volatile().reliable());

    scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_out", rclcpp::SensorDataQoS().durability_volatile().reliable());

    benchmarking_execution_duration_publisher_ = this->create_publisher<asb_msgs::msg::ExecutionDurationStamped>("~/benchmarking/execution_duration", rclcpp::SensorDataQoS().transient_local().reliable().keep_last(10));

    create_mask_service_ = this->create_service<std_srvs::srv::Empty>("~/create_mask", std::bind(&ASBLidarFilter::create_mask_service_callback, this, std::placeholders::_1, std::placeholders::_2));

}

void ASBLidarFilter::points_in_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points_in_msg) {
    auto execution_start = std::chrono::high_resolution_clock::now();

    typedef pcl::PointXYZ PointType;
    auto points_in_pcl = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::fromROSMsg(*points_in_msg, *points_in_pcl);

    int w = (int) points_in_pcl->width;
    int h = (int) points_in_pcl->height;

    // the index used in the for loops begins with the top layer (i == 0 at the top of the image/layers),
    // so the min/max layers must be switched and must start from points_in_pcl->height - 1
    int min_layer = std::clamp(h - 1 - max_layer_from_bottom_, 0, h - 1);
    int max_layer = std::clamp(h - 1 - min_layer_from_bottom_, 0, h - 1);

    // Create the LaserScan message (for later)
    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan_msg->header.stamp = points_in_msg->header.stamp;
    scan_msg->header.frame_id = base_frame_id_;
    scan_msg->angle_min = -M_PI;
    scan_msg->angle_max = M_PI;
    scan_msg->angle_increment = M_PI / (5 * 180.0);
    scan_msg->time_increment = 0.0;
    scan_msg->scan_time = 1.0 / 30.0;
    scan_msg->range_min = 0.0;
    scan_msg->range_max = 100.0;
    uint32_t ranges_size = std::ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

    if (points_in_pcl->isOrganized()) {

        if(reset_mask_) {
            RCLCPP_INFO(this->get_logger(), "resetting mask");
            mask_.resize(h * w, false);
            mask_width_ = w;
            mask_height_ = h;
            reset_mask_ = false;
        }

        if(create_mask_) {
            add_point_cloud_to_mask(points_in_pcl, points_in_msg->header);
        }

        if(mask_width_ != points_in_msg->width || mask_height_ != points_in_msg->height) {
            RCLCPP_ERROR(this->get_logger(), "loaded mask has different width or height than input pointcloud!");
            return;
        }

        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                float & x = points_in_pcl->at(j, i).x;
                float & y = points_in_pcl->at(j, i).y;
                float & z = points_in_pcl->at(j, i).z;

                // remove points (by setting them as NaN) out of min/max layer, out of range or in the mask
                if(i < min_layer || i > max_layer || (x*x + y*y + z*z < min_range_2_) || mask_[i * w + j]) {
                        x = std::numeric_limits<float>::quiet_NaN();
                        y = std::numeric_limits<float>::quiet_NaN();
                        z = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }

        sensor_msgs::msg::PointCloud2::SharedPtr points_out_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*points_in_pcl, *points_out_msg);
        points_out_publisher_->publish(*points_out_msg);

        auto points_base_frame_pcl = std::make_shared<pcl::PointCloud<PointType>>();
        if(!transform_point_cloud_to_base_frame<PointType>(points_in_pcl, points_in_msg->header, points_base_frame_pcl)){
            return;
        }

        for (int i = min_layer; i <= max_layer; i++) {
            for (int j = 0; j < w; j++) {
                float & x = points_base_frame_pcl->at(j, i).x;
                float & y = points_base_frame_pcl->at(j, i).y;
                float & z = points_base_frame_pcl->at(j, i).z;

                if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;

                if (z > scan_max_height_ || z < scan_min_height_) {
                    x = std::numeric_limits<float>::quiet_NaN();
                    y = std::numeric_limits<float>::quiet_NaN();
                    z = std::numeric_limits<float>::quiet_NaN();
                    continue;
                }

                float range = hypot(x, y);
                double angle = atan2(y, x);
                int k = (int) std::round((angle - scan_msg->angle_min) / scan_msg->angle_increment);
                if (std::isnan(scan_msg->ranges[k]) || range < scan_msg->ranges[k]) scan_msg->ranges[k] = range;
            }
        }

        sensor_msgs::msg::PointCloud2::SharedPtr points_out_no_ground_ceiling_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*points_base_frame_pcl, *points_out_no_ground_ceiling_msg);
        points_out_no_ground_ceiling_msg->header = scan_msg->header;
        points_out_no_ground_ceiling_publisher_->publish(*points_out_no_ground_ceiling_msg);
        scan_publisher_->publish(std::move(scan_msg));

    } else {

        // if the pointcloud is NOT organized (from sim sensor), filter the input pointcloud with a box modelling the space occupied by the
        // robot (remove points inside the box)
        auto points_base_frame_pcl = std::make_shared<pcl::PointCloud<PointType>>();
        if(!transform_point_cloud_to_base_frame<PointType>(points_in_pcl, points_in_msg->header, points_base_frame_pcl)){
            return;
        }
        pcl::CropBox<PointType> crop_box_filter;  // TODO ************************
        crop_box_filter.setNegative(true);
        crop_box_filter.setMin(Eigen::Vector4f((float) std::min(x_min_, x_max_), (float) std::min(y_min_, y_max_), (float) std::min(z_min_, z_max_), 1.0));
        crop_box_filter.setMax(Eigen::Vector4f((float) std::max(x_min_, x_max_), (float) std::max(y_min_, y_max_), (float) std::max(z_min_, z_max_), 1.0));
        crop_box_filter.setInputCloud(points_base_frame_pcl);
        crop_box_filter.filter(*points_base_frame_pcl);

        auto points_sensor_frame_pcl = std::make_shared<pcl::PointCloud<PointType>>();
        if(!transform_point_cloud_to_sensor_frame<PointType>(points_base_frame_pcl, points_in_msg->header, points_sensor_frame_pcl)){
            return;
        }

        sensor_msgs::msg::PointCloud2::SharedPtr points_out_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*points_sensor_frame_pcl, *points_out_msg);
        points_out_publisher_->publish(*points_out_msg);

        pcl::ConditionAnd<PointType>::Ptr no_ground_ceiling_cond(new pcl::ConditionAnd<PointType>());
        no_ground_ceiling_cond->addComparison(pcl::FieldComparison<PointType>::Ptr(new pcl::FieldComparison<PointType>("z", pcl::ComparisonOps::LT, scan_max_height_)));
        no_ground_ceiling_cond->addComparison(pcl::FieldComparison<PointType>::Ptr(new pcl::FieldComparison<PointType>("z", pcl::ComparisonOps::GT, scan_min_height_)));
        pcl::ConditionalRemoval<PointType> no_ground_ceiling_filter;
        no_ground_ceiling_filter.setCondition(no_ground_ceiling_cond);
        no_ground_ceiling_filter.setInputCloud(points_base_frame_pcl);
        no_ground_ceiling_filter.filter(*points_base_frame_pcl);

        sensor_msgs::msg::PointCloud2::SharedPtr points_out_no_ground_ceiling_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*points_base_frame_pcl, *points_out_no_ground_ceiling_msg);
        points_out_no_ground_ceiling_msg->header = scan_msg->header;
        points_out_no_ground_ceiling_publisher_->publish(*points_out_no_ground_ceiling_msg);

        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*points_out_no_ground_ceiling_msg, "x"), iter_y(*points_out_no_ground_ceiling_msg, "y"), iter_z(*points_out_no_ground_ceiling_msg, "z"); iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float range = hypot(*iter_x, *iter_y);
            double angle = atan2(*iter_y, *iter_x);
            int k = (int) std::round((angle - scan_msg->angle_min) / scan_msg->angle_increment);
            if (std::isnan(scan_msg->ranges[k]) || range < scan_msg->ranges[k]) scan_msg->ranges[k] = range;
        }

        scan_publisher_->publish(std::move(scan_msg));

    }

    // if we got to this point, publish the heartbeat
    heartbeat_publisher_->publish(points_in_msg->header);

    std::chrono::duration<double> execution_duration_s = std::chrono::high_resolution_clock::now() - execution_start;
    asb_msgs::msg::ExecutionDurationStamped execution_duration;
    execution_duration.stamp = this->get_clock()->now();
    execution_duration.execution_duration = rclcpp::Duration::from_seconds((execution_duration_s).count());
    execution_duration.label = "";
    benchmarking_execution_duration_publisher_->publish(execution_duration);
}

void ASBLidarFilter::create_mask_service_callback(const std::shared_ptr<Empty::Request> /*request*/, std::shared_ptr<Empty::Response> /*response*/) {
    RCLCPP_INFO(this->get_logger(), "create_mask service called: creating new mask...");
    reset_mask_ = true;
    create_mask_ = true;
    create_mask_count_ = mask_filter_count_;
}

template<typename PointT>
void ASBLidarFilter::add_point_cloud_to_mask(const std::shared_ptr<pcl::PointCloud<PointT>> & points_in_pcl, const std_msgs::msg::Header & points_in_header){

    int w = (int) points_in_pcl->width;
    int h = (int) points_in_pcl->height;

    // remove points with range less than min range by setting them as NaN
    auto points_masking_pcl = std::make_shared<pcl::PointCloud<PointT>>();
    *points_masking_pcl = *points_in_pcl;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            float & x = points_masking_pcl->at(j, i).x;
            float & y = points_masking_pcl->at(j, i).y;
            float & z = points_masking_pcl->at(j, i).z;

            float range_2 = x*x + y*y + z*z;
            if (range_2 < min_range_2_) {
                x = std::numeric_limits<float>::quiet_NaN();
                y = std::numeric_limits<float>::quiet_NaN();
                z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

    // transform the point cloud in base frame to filter points in the x/y/z min/max box
    auto points_masking_base_frame_pcl = std::make_shared<pcl::PointCloud<PointT>>();
    if(!transform_point_cloud_to_base_frame<PointT>(points_masking_pcl, points_in_header, points_masking_base_frame_pcl)){
        return;
    }

    // filter the input pointcloud with a box modelling the space occupied by the robot
    // (remove points inside the box) and also the points adjacent to the ones inside the
    // box, which are likely to be reflections on tangent surfaces
    auto x_min = (float) std::min(x_min_, x_max_);
    auto x_max = (float) std::max(x_min_, x_max_);
    auto y_min = (float) std::min(y_min_, y_max_);
    auto y_max = (float) std::max(y_min_, y_max_);
    auto z_min = (float) std::min(z_min_, z_max_);
    auto z_max = (float) std::max(z_min_, z_max_);
    mask_.resize(h * w, false);

    // note: the min/max layer params are ignored when creating the mask
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            float & x = points_masking_base_frame_pcl->at(j, i).x;
            float & y = points_masking_base_frame_pcl->at(j, i).y;
            float & z = points_masking_base_frame_pcl->at(j, i).z;

            bool x_in_box = x_min <= x && x <= x_max;
            bool y_in_box = y_min <= y && y <= y_max;
            bool z_in_box = z_min <= z && z <= z_max;

            if (x_in_box && y_in_box && z_in_box) {
                // insert in the mask the points adjacent to the ones in the box
                for (int d_i = -mask_filter_size_; d_i < mask_filter_size_ + 1; d_i++) {
                    for (int d_j = -mask_filter_size_; d_j < mask_filter_size_ + 1; d_j++) {
                        if (i + d_i >= 0 && i + d_i < h && j + d_j >= 0 && j + d_j < w) {
                            mask_[(i + d_i) * w + (j + d_j)] = true; // using flat index
                        }
                    }
                }
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "adding point cloud to mask (%u / %u)", mask_filter_count_ - create_mask_count_, mask_filter_count_);
    if(create_mask_count_ > 0) {
        create_mask_count_--;
    } else {
        save_mask_as_pbm(mask_file_path_, mask_, w, h);
        create_mask_ = false;
    }
}

template<typename PointT>
bool ASBLidarFilter::transform_point_cloud_to_base_frame(const std::shared_ptr<pcl::PointCloud<PointT>> & points_in_pcl, const std_msgs::msg::Header & points_in_header, std::shared_ptr<pcl::PointCloud<PointT>> & points_transformed_pcl){
    try {
        tf_buffer_->canTransform(base_frame_id_, points_in_header.frame_id, points_in_header.stamp, rclcpp::Duration::from_seconds(0.05));
        geometry_msgs::msg::TransformStamped sensor_to_base_transform_stamped = tf_buffer_->lookupTransform(base_frame_id_, points_in_header.frame_id, points_in_header.stamp, rclcpp::Duration::from_seconds(0.05));
        pcl_ros::transformPointCloud(*points_in_pcl, *points_transformed_pcl, sensor_to_base_transform_stamped);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform Exception: %s", ex.what());
        return false;
    }
    return true;
}

template<typename PointT>
bool ASBLidarFilter::transform_point_cloud_to_sensor_frame(const std::shared_ptr<pcl::PointCloud<PointT>> & points_in_pcl, const std_msgs::msg::Header & points_transformed_header, std::shared_ptr<pcl::PointCloud<PointT>> & points_transformed_pcl){
    try {
        tf_buffer_->canTransform(points_transformed_header.frame_id, base_frame_id_, points_transformed_header.stamp, rclcpp::Duration::from_seconds(0.05));
        geometry_msgs::msg::TransformStamped base_to_sensor_transform_stamped = tf_buffer_->lookupTransform(points_transformed_header.frame_id, base_frame_id_, points_transformed_header.stamp, rclcpp::Duration::from_seconds(0.05));
        pcl_ros::transformPointCloud(*points_in_pcl, *points_transformed_pcl, base_to_sensor_transform_stamped);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform Exception: %s", ex.what());
        return false;
    }
    return true;
}

void ASBLidarFilter::save_mask_as_pbm(const std::string& filename, const std::vector<bool>& mask, std::uint32_t width, std::uint32_t height) {
    std::ofstream out(filename, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Failed to open file for writing.");
    }

    // Write PBM header (P4 format)
    out << "P4\n" << width << " " << height << "\n";

    // Write bitmap data, 8 pixels per byte
    for (std::uint32_t row = 0; row < height; ++row) {
        for (std::uint32_t col_byte = 0; col_byte < (width + 7) / 8; ++col_byte) {
            unsigned char byte = 0;
            for (int bit = 0; bit < 8; ++bit) {
                std::uint32_t col = col_byte * 8 + bit;
                if (col < width) {
                    bool val = mask[row * width + col];
                    byte |= (val ? 0 : 1) << (7 - bit); // PBM: 1 = black, 0 = white
                }
            }
            out.put(byte);
        }
    }

    RCLCPP_INFO(
            this->get_logger(),
            "\n"
            "******************\n"
            "*   saved mask   *\n"
            "******************\n"
            );

}

bool ASBLidarFilter::load_mask_from_pbm(const std::string& filename, std::vector<bool>& mask, std::uint32_t& width, std::uint32_t& height) {
    std::ifstream in(filename, std::ios::binary);
    if (!in) {
        return false;
    }

    // Read PBM header (format: P4\nWIDTH HEIGHT\n)
    std::string line;
    std::getline(in, line);
    if (line != "P4") {
        throw std::runtime_error("Invalid PBM format (expected P4).");
    }

    // Skip comments (lines starting with #)
    do {
        std::getline(in, line);
    } while (!line.empty() && line[0] == '#');

    // Read width and height
    std::istringstream iss(line);
    iss >> width >> height;
    if (!iss) {
        throw std::runtime_error("Failed to read image dimensions.");
    }

    // Read pixel data
    std::size_t num_bits = width * height;
    std::size_t num_bytes_per_row = (width + 7) / 8;
    mask.resize(num_bits);

    for (std::uint32_t row = 0; row < height; ++row) {
        for (std::uint32_t col_byte = 0; col_byte < num_bytes_per_row; ++col_byte) {
            char byte;
            in.get(byte);
            if (in.eof()) {
                throw std::runtime_error("Unexpected EOF in PBM data.");
            }

            for (int bit = 0; bit < 8; ++bit) {
                std::uint32_t col = col_byte * 8 + bit;
                if (col < width) {
                    bool val = ((byte >> (7 - bit)) & 1) == 0; // PBM: 1 = black, 0 = white
                    mask[row * width + col] = val;
                }
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "loaded mask");

    return true;
}