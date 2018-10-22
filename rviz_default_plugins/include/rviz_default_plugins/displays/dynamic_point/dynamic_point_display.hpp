/*
 * Copyright (c) 2012, Willow Garage, Inc.
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


#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__DYNAMICPOINT__DYNAMICPOINT_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__DYNAMICPOINT__DYNAMICPOINT_DISPLAY_HPP_

#include "geometry_msgs/msg/point_stamped.hpp"

#include <vector>

#include "rviz_common/ros_topic_display.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_rendering
{
class Axes;
}  // namespace rviz_rendering

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class FloatProperty;
} // namespace properties
} // namespace rviz_common

namespace rviz_default_plugins
{
namespace displays
{

/*
 * @brief Displays a PointStamped message with color and alpha according to acceleration and speed respectively
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC DynamicPointDisplay : public
    rviz_common::RosTopicDisplay<geometry_msgs::msg::PointStamped>
{
    Q_OBJECT

public:
    DynamicPointDisplay();
    ~DynamicPointDisplay() override;
    
protected:
    void onDisable() override;
    void onEnable() override;
    void onInitialize() override;
    void reset() override;

private Q_SLOTS:
    void initialiseColorAndAlpha();
    void updateVisibility();

private:
    void processMessage(geometry_msgs::msg::PointStamped::ConstSharedPtr message) override;
    void updateColorAndAlpha(geometry_msgs::msg::PointStamped::ConstSharedPtr message);

    bool dynamic_point_valid_;
    double delta_red_value_;
    
    std::unique_ptr<rviz_rendering::Axes> axes_;

    geometry_msgs::msg::PointStamped::ConstSharedPtr message_new_;
    geometry_msgs::msg::PointStamped::ConstSharedPtr message_old_;

    rviz_common::properties::ColorProperty * color_property_;
    rviz_common::properties::FloatProperty * alpha_property_;

    Ogre::ColourValue dynamic_color_;
};
}
}

#endif // RVIZ_DEFAULT_PLUGINS__DISPLAYS__PATH__PATH_DISPLAY_HPP_
