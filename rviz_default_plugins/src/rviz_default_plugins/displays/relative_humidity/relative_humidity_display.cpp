/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "rviz_default_plugins/displays/relative_humidity/relative_humidity_display.hpp"

namespace rviz_default_plugins
{

namespace displays
{

RelativeHumidityDisplay::RelativeHumidityDisplay()
{}

RelativeHumidityDisplay::~RelativeHumidityDisplay() = default;


void RelativeHumidityDisplay::processMessage(
  const sensor_msgs::msg::RelativeHumidity::ConstSharedPtr message)
{
  auto point_cloud2_message_for_point_cloud_common =
    createPointCloud2Message(message->header, message->relative_humidity, "relative_humidity");

  point_cloud_common_->addMessage(point_cloud2_message_for_point_cloud_common);
}


void RelativeHumidityDisplay::setInitialValues()
{
  subProp("Channel Name")->setValue("illuminance");
  subProp("Autocompute Intensity Bounds")->setValue(false);
  subProp("Min Intensity")->setValue(0.0);
  subProp("Max Intensity")->setValue(1.0);
}

void RelativeHumidityDisplay::hideUnneededProperties()
{
  subProp("Position Transformer")->hide();
  subProp("Color Transformer")->hide();
  subProp("Channel Name")->hide();
  subProp("Autocompute Intensity Bounds")->hide();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::RelativeHumidityDisplay,
  rviz_common::Display)
