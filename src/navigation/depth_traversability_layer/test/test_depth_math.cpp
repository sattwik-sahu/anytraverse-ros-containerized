// Copyright 2026 AnyTraverse contributors.
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

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <limits>

#include "depth_traversability_layer/depth_buffer.hpp"
#include "sensor_msgs/msg/image.hpp"

using depth_traversability_layer::backProject;
using depth_traversability_layer::CostMapper;
using depth_traversability_layer::decodeDepthPixel;
using depth_traversability_layer::DepthTileMap;

TEST(DepthMath, BackProjectCenter)
{
  double x = 0.0, y = 0.0, z = 0.0;
  // Optical center pixel maps to the forward ray.
  EXPECT_TRUE(backProject(320.0, 240.0, 2.0, 500.0, 500.0, 320.0, 240.0, x, y, z));
  EXPECT_DOUBLE_EQ(x, 0.0);
  EXPECT_DOUBLE_EQ(y, 0.0);
  EXPECT_DOUBLE_EQ(z, 2.0);
}

TEST(DepthMath, BackProjectOffCenter)
{
  double x = 0.0, y = 0.0, z = 0.0;
  // One focal length right of center at depth 4 -> x = 4.
  EXPECT_TRUE(backProject(820.0, 240.0, 4.0, 500.0, 500.0, 320.0, 240.0, x, y, z));
  EXPECT_DOUBLE_EQ(x, 4.0);
  EXPECT_DOUBLE_EQ(y, 0.0);
  EXPECT_DOUBLE_EQ(z, 4.0);
}

TEST(DepthMath, BackProjectRejectsBadDepth)
{
  double x = 0.0, y = 0.0, z = 0.0;
  EXPECT_FALSE(backProject(320.0, 240.0, 0.0, 500.0, 500.0, 320.0, 240.0, x, y, z));
  EXPECT_FALSE(backProject(320.0, 240.0, -1.0, 500.0, 500.0, 320.0, 240.0, x, y, z));
}

TEST(DepthMath, Decode16UC1Millimetres)
{
  sensor_msgs::msg::Image img;
  img.width = 2;
  img.height = 1;
  img.step = 4;
  img.encoding = "16UC1";
  // 1500 mm then 0 (no return).
  img.data = {0xDC, 0x05, 0x00, 0x00};
  double d = 0.0;
  EXPECT_TRUE(decodeDepthPixel(img, 0, 0, d));
  EXPECT_DOUBLE_EQ(d, 1.5);
  EXPECT_FALSE(decodeDepthPixel(img, 1, 0, d));
}

TEST(DepthMath, Decode32FC1RejectsNaN)
{
  sensor_msgs::msg::Image img;
  img.width = 1;
  img.height = 1;
  img.step = 4;
  img.encoding = "32FC1";
  const float nan = std::numeric_limits<float>::quiet_NaN();
  img.data.resize(4);
  std::memcpy(img.data.data(), &nan, 4);
  double d = 0.0;
  EXPECT_FALSE(decodeDepthPixel(img, 0, 0, d));
}

TEST(DepthMath, DecodeRejectsUnknownEncoding)
{
  sensor_msgs::msg::Image img;
  img.width = 1;
  img.height = 1;
  img.step = 3;
  img.encoding = "rgb8";
  img.data = {1, 2, 3};
  double d = 0.0;
  EXPECT_FALSE(decodeDepthPixel(img, 0, 0, d));
}

TEST(DepthMath, DirectCostMatchesMapSemantics)
{
  CostMapper mapper;
  mapper.setDirectCostMapping(254.0, 0.0, -1, -1);
  EXPECT_EQ(mapper.resolveClassId(0), 254);    // blocked -> lethal
  EXPECT_EQ(mapper.resolveClassId(255), 0);    // free -> zero cost
  EXPECT_EQ(mapper.resolveClassId(128), 127);  // lround(254 * (127/255))
  EXPECT_EQ(mapper.getCostById(200).max_cost, 200);
}

TEST(DepthMath, DirectCostIgnoreBand)
{
  CostMapper mapper;
  mapper.setDirectCostMapping(254.0, 0.0, 157, 169);
  EXPECT_EQ(mapper.resolveClassId(163), -1);
  EXPECT_EQ(mapper.resolveClassId(100), 154);  // lround(254 * (155/255))
}

TEST(DepthMath, TileIndexFloorsNegatives)
{
  DepthTileMap map(0.05f, 3.0);
  // Off-boundary values: exact multiples are inherently ambiguous in
  // floating point (1.0/0.05f rounds to just under 20), so stay clear.
  const auto idx = map.worldToIndex(-0.01, 1.03);
  EXPECT_EQ(idx.x, -1);
  EXPECT_EQ(idx.y, 20);
  const auto world = map.indexToWorld(0, 0);
  EXPECT_FLOAT_EQ(static_cast<float>(world.x), 0.025f);
  EXPECT_FLOAT_EQ(static_cast<float>(world.y), 0.025f);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
