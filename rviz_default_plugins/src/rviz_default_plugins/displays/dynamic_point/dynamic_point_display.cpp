/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "rviz_default_plugins/displays/dynamic_point/dynamic_point_display.hpp"

#include <algorithm>
#include <cmath>
#include <vector>
#include <iostream>

#include "rviz_rendering/objects/axes.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/validate_floats.hpp"

namespace rviz_default_plugins
{
namespace displays
{

DynamicPointDisplay::DynamicPointDisplay()
: axes_(nullptr), message_old_(nullptr)
{
  delta_red_value_ = 0;
  dynamic_color_.r = 0;
  dynamic_color_.g = 255;
  dynamic_color_.b = 0;
  dynamic_color_.a = 1;

  /*
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 25, 0), "Color to draw the arrow.",
    this, SLOT(initialiseColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1, "Amount of transparency to apply to the arrow.",
    this, SLOT(initialiseColorAndAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
  */

}

DynamicPointDisplay::~DynamicPointDisplay() = default;

void DynamicPointDisplay::onDisable()
{
  RTDClass::onDisable();
}

void DynamicPointDisplay::onEnable()
{
  RTDClass::onEnable();
}

void DynamicPointDisplay::onInitialize()
{
  RTDClass::onInitialize();
  std::unique_ptr<rviz_rendering::Axes> null = nullptr;
  
  axes_ = std::make_unique<rviz_rendering::Axes>(scene_manager_, scene_node_, 0.2, 0.2);

  initialiseColorAndAlpha();
}

void DynamicPointDisplay::reset()
{
  RTDClass::reset();
  dynamic_point_valid_ = false;
  updateVisibility();
}

void DynamicPointDisplay::initialiseColorAndAlpha()
{
  axes_->setColor(dynamic_color_.r, dynamic_color_.g, dynamic_color_.b, dynamic_color_.a);
  context_->queueRender();
}


void DynamicPointDisplay::processMessage(geometry_msgs::msg::PointStamped::ConstSharedPtr message)
{
  if (!rviz_common::validateFloats(*message)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position = Ogre::Vector3(message->point.x, message->point.y, message->point.z);
  
  setTransformOk();

  dynamic_point_valid_ = true;
  
  updateVisibility();
  
  updateColorAndAlpha(message);
  
  scene_node_->setPosition(position);

  context_->queueRender();
}

void DynamicPointDisplay::updateColorAndAlpha(
    geometry_msgs::msg::PointStamped::ConstSharedPtr message)
{
  message_new_ = message;
  
  if(message_old_ == nullptr){
    delta_red_value_ = message_new_->point.x;
  }
  else{
    delta_red_value_ = 
      std::sqrt(std::pow(message_new_->point.x - message_old_->point.x, 2) 
              + std::pow(message_new_->point.y - message_old_->point.y, 2)) * 10;
  }  
  
  dynamic_color_.r = delta_red_value_;
  //dynamic_color_.g = 255 - delta_red_value_;

  axes_->setXColor(dynamic_color_);
  axes_->setYColor(dynamic_color_);
  axes_->setZColor(dynamic_color_);

  context_->queueRender();

  message_old_ = message_new_; 
}


void DynamicPointDisplay::updateVisibility()
{
  if (!dynamic_point_valid_) {
    axes_->getSceneNode()->setVisible(false);
  } else {
    axes_->getSceneNode()->setVisible(true);
  }
}

}
}

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::DynamicPointDisplay, rviz_common::Display)