/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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
 *     * Neither the name of the copyright holders nor the names of its
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

#include <gmock/gmock.h>

#include <memory>

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable : 4996)
#endif

#include <OgreEntity.h>
#include <OgreSceneNode.h>

#ifdef _WIN32
# pragma warning(pop)
#endif

#include "rviz_default_plugins/displays/illuminance/illuminance_display.hpp"
#include "../display_test_fixture.hpp"

using namespace ::testing;  // NOLINT

sensor_msgs::msg::Illuminance::ConstSharedPtr createIlluminanceMessage(
  const float illuminance = 0., const float variance = 1.)
{
  auto message = std::make_shared<sensor_msgs::msg::Illuminance>();
  message->header = std_msgs::msg::Header();
  message->header.frame_id = "illuminance_frame";
  message->header.stamp = rclcpp::Clock().now();

  message->illuminance = illuminance;
  message->variance = variance;

  return message;
}

class IlluminanceDisplayFixture : public DisplayTestFixture
{
public:
  IlluminanceDisplayFixture()
  : display_(new rviz_default_plugins::displays::IlluminanceDisplay())
  {}

  ~IlluminanceDisplayFixture()
  {
    display_.reset();
  }

  std::shared_ptr<rviz_default_plugins::displays::IlluminanceDisplay> display_;
};

TEST_F(IlluminanceDisplayFixture, allocate_point_cloud_common_memory_correctly)
{
  auto illuminance_message = createIlluminanceMessage();
  auto point_cloud_message =
    display_->createPointCloudMessageFromIlluminanceMessage(illuminance_message);

  ASSERT_THAT(point_cloud_message->point_step, Eq(20u));
}
