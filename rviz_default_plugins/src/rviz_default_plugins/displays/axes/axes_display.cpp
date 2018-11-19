/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2018, Maximilian Kuehn
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

// #include <boost/bind.hpp>

#include "rviz_default_plugins/displays/axes/axes_display.hpp"

#include <memory>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/tf_frame_property.hpp"
#include "rviz_rendering/objects/axes.hpp"

namespace rviz_default_plugins
{

namespace displays {

AxesDisplay::AxesDisplay()
: Display(),  axes_(0)
{
  frame_property_ = new rviz_common::properties::TfFrameProperty(
      "Reference Frame",rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING,
      "The TF frame these axes will use for their origin.", this, NULL, true);

  length_property_ = new rviz_common::properties::FloatProperty(
      "Length", 1.0, "Length of each axis, in meters.", this, SLOT(updateShape()));
  length_property_->setMin(0.0001);

  radius_property_ = new rviz_common::properties::FloatProperty(
      "Radius", 0.1, "Radius of each axis, in meters.", this, SLOT(updateShape()));
  radius_property_->setMin(0.0001);
}

AxesDisplay::~AxesDisplay() {}

void AxesDisplay::onInitialize() {
  frame_property_->setFrameManager(context_->getFrameManager());

  axes_ = std::make_shared<rviz_rendering::Axes>(scene_manager_, nullptr, length_property_->getFloat(),
                                                radius_property_->getFloat());
  axes_->getSceneNode()->setVisible(isEnabled());
}

    void AxesDisplay::onEnable() {
      axes_->getSceneNode()->setVisible(true);
    }

    void AxesDisplay::onDisable() {
      axes_->getSceneNode()->setVisible(false);
    }


    void AxesDisplay::updateShape() {
      axes_->set(length_property_->getFloat(), radius_property_->getFloat());
      context_->queueRender();
    }

    void AxesDisplay::update(float dt, float ros_dt) {
      Q_UNUSED(dt);
      Q_UNUSED(ros_dt);
      QString qframe = frame_property_->getFrame();
      std::string frame = qframe.toStdString();

      Ogre::Vector3 position;
      Ogre::Quaternion orientation;

      if (context_->getFrameManager()->getTransform(frame, rclcpp::Time(), position, orientation)) {
        axes_->setPosition(position);
        axes_->setOrientation(orientation);
        setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "Transform OK");
      } else {
        std::string error;
        if (context_->getFrameManager()->transformHasProblems(frame, rclcpp::Time(), error)) {
          setStatus(rviz_common::properties::StatusProperty::Error, "Transform", QString::fromStdString(error));
        } else {
          setStatus(rviz_common::properties::StatusProperty::Error,
                    "Transform",
                    "Could not transform from [" + qframe + "] to Fixed Frame [" + fixed_frame_ +
                    "] for an unknown reason");
        }
      }
    }

}  // namespace displays

}  // namespace rviz

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::AxesDisplay, rviz_common::Display )
