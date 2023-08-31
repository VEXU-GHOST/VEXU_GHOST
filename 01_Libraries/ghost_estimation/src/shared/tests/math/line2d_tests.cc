// Copyright 2019 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include <gtest/gtest.h>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "math/geometry.h"
#include "math/line2d.h"

using geometry::Line;
using geometry::Line2f;
using Eigen::Vector2f;

TEST(Line2D, Touching) {
  {
    const Line2f l1(Vector2f(1, 4), Vector2f(4, 1));
    const Line2f l2(Vector2f(6, 6), Vector2f(2.5, 2.5));
    EXPECT_TRUE(l1.Intersects(l2));
  }
}

TEST(Line2D, CollinearNoIntersection) {
  {
    const Line2f l1(Vector2f(1, 1), Vector2f(2, 2));
    const Line2f l2(Vector2f(3, 3), Vector2f(4, 4));
    EXPECT_FALSE(l1.Intersects(l2));
  }
}

TEST(Line2D, CollinearOverlap) {
  {
    const Line2f l1(Vector2f(1, 1), Vector2f(3, 3));
    const Line2f l2(Vector2f(2, 2), Vector2f(4, 4));
    EXPECT_TRUE(l1.Intersects(l2));
  }
}

TEST(Line2D, CollinearTouching) {
  {
    const Line2f l1(Vector2f(1, 1), Vector2f(3, 3));
    const Line2f l2(Vector2f(3, 3), Vector2f(4, 4));
    EXPECT_TRUE(l1.Intersects(l2));
  }
}

TEST(Line2D, JumboSuite) {
  EXPECT_TRUE(Line2f(1.1, 0.1, 1.9, -0.1).Intersects(Line2f(1, 0, 2, 0)));
  EXPECT_TRUE(Line2f(1.1, 0.1, 1.9, 0).Intersects(Line2f(1, 0, 2, 0)));
  EXPECT_FALSE(Line2f(1.1, 0.1, 1.9, 0.001).Intersects(Line2f(1, 0, 2, 0)));
  EXPECT_FALSE(Line2f(0, 0.1, 1.1, -0.1).Intersects(Line2f(1, 0, 2, 0)));
  EXPECT_FALSE(Line2f(1, 1, 2, 2).Intersects(Line2f(3, 3, 6, 6)));
  EXPECT_TRUE(Line2f(0, 0, 0, 5).Intersects(Line2f(0, 5, 6, 6)));
  EXPECT_TRUE(Line2f(0, 0, 0, 5).Intersects(Line2f(0, 4, 0, 6)));
  EXPECT_TRUE(Line2f(0, 0, 0, 5).Intersects(Line2f(0, 5, 0, 6)));
  EXPECT_FALSE(Line2f(0, 0, 2, 3).Intersects(Line2f(4, 6, 6, 9)));
  EXPECT_TRUE(Line2f(1, 1, 2, 2).Intersects(Line2f(1, 1, 2, 2)));
  EXPECT_FALSE(Line2f(1, 1, 2, 2).Intersects(Line2f(2, 1, 3, 2)));
  EXPECT_TRUE(Line2f(5, 0, 5, 10).Intersects(Line2f(0, 5, 10, 5)));
  EXPECT_FALSE(Line2f(5, 0, 5, 10).Intersects(Line2f(50, 50, 60, 60)));
  EXPECT_TRUE(Line2f(-1, -1, 1, 1).Intersects(Line2f(-2, -2, 3, 3)));
  EXPECT_FALSE(Line2f(0, 0, 0, 5).Intersects(Line2f(1, 1, 5, 5)));
  EXPECT_TRUE(Line2f(0, 0, 0, 5).Intersects(Line2f(0, 5, 0, 10)));
  EXPECT_FALSE(Line2f(10, 10, 10, 10).Intersects(Line2f(30, 4, 30, 10)));
  EXPECT_TRUE(Line2f(0, 200, 300, 200).Intersects(Line2f(100, 300, 100, 200)));
  EXPECT_TRUE(Line2f(0, 0, 0, 0).Intersects(Line2f(0, 0, 0, 0)));
  EXPECT_FALSE(Line2f(0, 0, 0, 0).Intersects(Line2f(0, 3, 0, 3)));
  EXPECT_FALSE(Line2f(0, 0, 0, 0.4).Intersects(Line2f(0, 3, 0, 3)));
  EXPECT_FALSE(Line2f(0.4, 0, 0, 0).Intersects(Line2f(3, 0, 3, 0)));
}

TEST(Line2D, ClosestApproach) {
  EXPECT_FLOAT_EQ(Line2f(1, 0, 0, 1).ClosestApproach(
      Vector2f(0, 0), Vector2f(-1, -1)), 1.0f / sqrt(2.0f));

  EXPECT_FLOAT_EQ(Line2f(1, 1, 10, 1).ClosestApproach(
      Vector2f(-100, 0), Vector2f(-1, -1)), sqrt(8.0f));

  EXPECT_FLOAT_EQ(Line2f(-2, 2, 2, 2).ClosestApproach(
    {-2.5, 0}, {-1.5, 4}), 0);

  EXPECT_FLOAT_EQ(Line2f(-2, 2, 2, 2).ClosestApproach(
    {-2.5, 0}, {-1.5, 0}), 2);

  EXPECT_FLOAT_EQ(Line2f(1.0f, 1.0f, 20.0f, 1.0f).ClosestApproach(
    Vector2f(0.9, 1), Vector2f(4.9, 5)), 0.1f / sqrt(2.0f));
}

TEST(Line2D, Distance) {
  EXPECT_FLOAT_EQ(Line2f(1, 0, 0, 1).Distance(
      Vector2f(0, 0)), 1.0f / sqrt(2.0f));

  EXPECT_FLOAT_EQ(Line2f(1, 1, 10, 1).Distance(
      Vector2f(0, 0)), sqrt(2.0f));

  EXPECT_FLOAT_EQ(Line2f(1, 1, 10, 1).Distance(
      Vector2f(11, 2)), sqrt(2.0f));

  EXPECT_FLOAT_EQ(Line2f(1, 1, 10, 1).Distance(
      Vector2f(6, 2)), 1.0);
}

TEST(Line2D, CloserThan) {
  EXPECT_TRUE(Line2f(-2, 2, 2, 2).CloserThan(
    {-2.61382, 0.744}, {-1.5, 4}, 0.3));
}
