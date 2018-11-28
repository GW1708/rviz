///*
// * Copyright (c) 2012, Willow Garage, Inc.
// * Copyright (c) 2018, TNG Technology Consulting GmbH.
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// *     * Redistributions of source code must retain the above copyright
// *       notice, this list of conditions and the following disclaimer.
// *     * Redistributions in binary form must reproduce the above copyright
// *       notice, this list of conditions and the following disclaimer in the
// *       documentation and/or other materials provided with the distribution.
// *     * Neither the name of the Willow Garage, Inc. nor the names of its
// *       contributors may be used to endorse or promote products derived from
// *       this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// * POSSIBILITY OF SUCH DAMAGE.
// */
//
//#include "rviz_default_plugins/displays/pointcloud/point_cloud_scalar_display.hpp"
//
//#include <memory>
//#include <string>
//#include <iostream>
//
//#include <OgreSceneNode.h>
//#include <OgreSceneManager.h>
//
//#include "rclcpp/clock.hpp"
//#include "rclcpp/time.hpp"
//
//#include "rviz_common/display_context.hpp"
//#include "rviz_common/frame_manager_iface.hpp"
//#include "rviz_common/properties/queue_size_property.hpp"
//#include "rviz_common/validate_floats.hpp"
//#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"
//#include "rviz_rendering/objects/point_cloud.hpp"
//
//namespace rviz_default_plugins
//{
//
//namespace displays
//{
//
//  PointCloudScalarDisplay::PointCloudScalarDisplay()
//  : queue_size_property_(new rviz_common::QueueSizeProperty(this, 10)),
//    point_cloud_common_(new PointCloudCommon(this)),
//    field_size_total_(0)
//  {}
//
//  PointCloudScalarDisplay::~PointCloudScalarDisplay() = default;
//
//  void PointCloudScalarDisplay::onInitialize()
//  {
//    RTDClass::onInitialize();
//    point_cloud_common_->initialize(context_, scene_node_);
//
//  }
//
//  void PointCloudScalarDisplay::updatePointCloudCommon(
//    const std::string header, const float scalar_value)
//  {
//    auto point_cloud2_message_for_point_cloud_common =
//            createPointCloud2Message(header, scalar_value);
//
//    point_cloud_common_->addMessage(point_cloud2_message_for_point_cloud_common);
//  }
//
//  std::shared_ptr<sensor_msgs::msg::PointCloud2>
//  PointCloudScalarDisplay::createPointCloud2Message(
//          const std::string header, const float scalar_value)
//  {
//    auto point_cloud_message = std::make_shared<sensor_msgs::msg::PointCloud2>();
//    const float coordinate_value = 0.0;
//
//    point_cloud_message->header = header;
//
//    int x_offset = addField32andReturnOffset(point_cloud_message, "x");
//    int y_offset = addField32andReturnOffset(point_cloud_message, "y");
//    int z_offset = addField32andReturnOffset(point_cloud_message, "z");
//    int scalar_offset = addField64andReturnOffset(point_cloud_message, "");
//
//    point_cloud_message->data.resize(field_size_total_);
//
//    memcpy(&point_cloud_message->data[x_offset], &coordinate_value, field_size_32_);
//    memcpy(&point_cloud_message->data[y_offset], &coordinate_value, field_size_32_);
//    memcpy(&point_cloud_message->data[z_offset], &coordinate_value, field_size_32_);
//    memcpy(&point_cloud_message->data[scalar_offset], &scalar_value, field_size_64_);
//
//    point_cloud_message->height = 1;
//    point_cloud_message->width = 1;
//    point_cloud_message->is_bigendian = false;
//    point_cloud_message->point_step = field_size_total_;
//    point_cloud_message->row_step = 1;
//
//    resetFieldSizeTotal();
//
//    return point_cloud_message;
//  }
//
//  int PointCloudScalarDisplay::addField32andReturnOffset(
//          std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud_message, const std::string field_name)
//  {
//    sensor_msgs::msg::PointField field;
//    field.name = field_name;
//    field.offset = field_size_total_;
//    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
//    field.count = 1;
//    point_cloud_message->fields.push_back(field);
//    field_size_total_ += field_size_32_;
//
//    return field.offset;
//  }
//
//  int PointCloudScalarDisplay::addField64andReturnOffset(
//          std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud_message, const std::string field_name)
//  {
//    sensor_msgs::msg::PointField field;
//    field.name = field_name;
//    field.offset = field_size_total_;
//    field.datatype = sensor_msgs::msg::PointField::FLOAT64;
//    field.count = 1;
//    point_cloud_message->fields.push_back(field);
//    field_size_total_ += field_size_64_;
//
//    return field.offset;
//  }
//
//  void PointCloudScalarDisplay::resetFieldSizeTotal()
//  {
//    field_size_total_ = 0;
//  }
//
//  void PointCloudScalarDisplay::update(float wall_dt, float ros_dt)
//  {
//    point_cloud_common_->update(wall_dt, ros_dt);
//    hideUnneededProperties();
//  }
//
////  void PointCloudScalarDisplay::hideUnneededProperties()
////  {
////    subProp("Position Transformer")->hide();
////    subProp("Color Transformer")->hide();
////    subProp("Channel Name")->hide();
////    subProp("Invert Rainbow")->hide();
////    subProp("Autocompute Intensity Bounds")->hide();
////  }
//
//  void PointCloudScalarDisplay::reset()
//  {
//    RTDClass::reset();
//    point_cloud_common_->reset();
//  }
//
//  void PointCloudScalarDisplay::onDisable()
//  {
//    RosTopicDisplay::onDisable();
//    point_cloud_common_->onDisable();
//  }
//
//}      // namespace displays
//}  // namespace rviz_default_plugins
//
//#include <pluginlib/class_list_macros.hpp> // NOLINT
//PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PointCloudScalarDisplay, rviz_common::Display)
