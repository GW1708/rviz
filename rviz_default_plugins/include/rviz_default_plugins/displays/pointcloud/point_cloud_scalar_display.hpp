/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2018, TNG Technology Consulting GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_SCALAR_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_SCALAR_DISPLAY_HPP_

#include <memory>
#include <string>

// #include "sensor_msgs/msg/illuminance.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rviz_common/ros_topic_display.hpp"
#include "rviz_common/properties/queue_size_property.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{

    class PointCloudCommon;

    namespace displays
    {

/**
 * \class PointCloudScalarDisplay
 * \brief This is the parent class for several scalar type messages that use a point cloud to
 * display their data
 *
 */

class RVIZ_DEFAULT_PLUGINS_PUBLIC PointCloudScalarDisplay
: public rviz_common::RosTopicDisplay<>
{
Q_OBJECT

public:
    PointCloudScalarDisplay();
    ~PointCloudScalarDisplay() override;

    void reset() override;
    void update(float wall_dt, float ros_dt) override;
    void onDisable() override;
    //virtual void processMessage(sensor_msgs::msg::Illuminance::ConstSharedPtr message) override;


    void PointCloudScalarDisplay::updatePointCloudCommon(
            const string header, const unsigned float scalar_value)

    std::shared_ptr<sensor_msgs::msg::PointCloud2> createPointCloud2Message(
            const sensor_msgs::msg::Illuminance::ConstSharedPtr message);

protected:
    void onInitialize() override;

private:
    void resetFieldSizeTotal();
    virtual void setInitialValues();
    virtual void hideUnneededProperties();

    int addField32andReturnOffset(
            std::shared_ptr<sensor_msgs::msg::PointCloud2>, std::string field_name);
    int addField64andReturnOffset(
            std::shared_ptr<sensor_msgs::msg::PointCloud2>, std::string field_name);

    std::unique_ptr<rviz_common::QueueSizeProperty> queue_size_property_;
    std::shared_ptr<rviz_default_plugins::PointCloudCommon> point_cloud_common_;

    const int field_size_32_ = 4;
    const int field_size_64_ = 8;
    int field_size_total_;
};

    }      // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_SCALAR_DISPLAY_HPP_
