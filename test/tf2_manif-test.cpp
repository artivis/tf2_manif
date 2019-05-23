/*
 * Copyright (c) Jeremie Deray
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

/** \author Jeremie Deray */


#include "tf2_manif/tf2_manif.h"

#include <gtest/gtest.h>

static const double EPS = 1e-3;

static const ros::Time t(5);
static const std::string f("test");

//
// Vector3
//

TEST(TfManif, ConvertVector3dStampedToVector3Stamped)
{
  const tf2::Stamped<Eigen::Vector3d> v(Eigen::Vector3d(1,2,3), t, f);

  tf2::Stamped<Eigen::Vector3d> v1;
  geometry_msgs::Vector3Stamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.frame_id_, v1.frame_id_);
  EXPECT_EQ(v.stamp_, v1.stamp_);
  EXPECT_EQ(v.x(), v1.x());
  EXPECT_EQ(v.y(), v1.y());
  EXPECT_EQ(v.z(), v1.z());
}

TEST(TfManif, ConvertVector3dToVector3)
{
  const Eigen::Vector3d v(1,2,3);

  Eigen::Vector3d v1;
  geometry_msgs::Vector3 p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.x(), v1.x());
  EXPECT_EQ(v.y(), v1.y());
  EXPECT_EQ(v.z(), v1.z());
}

TEST(TfManif, TransformVector3d)
{
  const tf2::Stamped<manif::SO3d> in(manif::SO3d::Identity(), t, f);
  const manif::SE3d se3(Eigen::Vector3d(-1, 2, -3), Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));

  geometry_msgs::TransformStamped trafo = tf2::manifToTransform(se3);
  trafo.header.stamp = ros::Time(10);
  trafo.header.frame_id = "expected";

  tf2::Stamped<manif::SO3d> out;
  tf2::doTransform(in, out, trafo);

  EXPECT_NEAR(out.x(), trafo.transform.rotation.x, EPS);
  EXPECT_NEAR(out.y(), trafo.transform.rotation.y, EPS);
  EXPECT_NEAR(out.z(), trafo.transform.rotation.z, EPS);
  EXPECT_NEAR(out.w(), trafo.transform.rotation.w, EPS);
}

//
// Point
//

// TEST(TfManif, ConvertVector3dStampedToPointStamped)
// {
//   const tf2::Stamped<Eigen::Vector3d> v(Eigen::Vector3d(1,2,3), t, f);
//
//   tf2::Stamped<Eigen::Vector3d> v1;
//   geometry_msgs::PointStamped p1;
//   tf2::convert(v, p1);
//   tf2::convert(p1, v1);
//
//   EXPECT_EQ(v.frame_id_, v1.frame_id_);
//   EXPECT_EQ(v.stamp_, v1.stamp_);
//   EXPECT_EQ(v.x(), v1.x());
//   EXPECT_EQ(v.y(), v1.y());
//   EXPECT_EQ(v.z(), v1.z());
// }
//
// TEST(TfManif, ConvertVector3dToPoint)
// {
//   const Eigen::Vector3d v(1,2,3);
//
//   Eigen::Vector3d v1;
//   geometry_msgs::Point p1;
//   tf2::convert(v, p1);
//   tf2::convert(p1, v1);
//
//   EXPECT_EQ(v.x(), v1.x());
//   EXPECT_EQ(v.y(), v1.y());
//   EXPECT_EQ(v.z(), v1.x());
// }

//
// SO3
//

TEST(TfManif, ConvertSO3dStamped)
{
  const tf2::Stamped<manif::SO3d> v(manif::SO3d(0,0,0,1), t, f);

  tf2::Stamped<manif::SO3d> v1;
  geometry_msgs::QuaternionStamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.frame_id_, v1.frame_id_);
  EXPECT_EQ(v.stamp_, v1.stamp_);
  EXPECT_EQ(v.w(), v1.w());
  EXPECT_EQ(v.x(), v1.x());
  EXPECT_EQ(v.y(), v1.y());
  EXPECT_EQ(v.z(), v1.z());
}

TEST(TfManif, ConvertSO3d)
{
  const manif::SO3d v(0,0,0,1);

  manif::SO3d v1;
  geometry_msgs::Quaternion p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.w(), v1.w());
  EXPECT_EQ(v.x(), v1.x());
  EXPECT_EQ(v.y(), v1.y());
  EXPECT_EQ(v.z(), v1.z());
}

TEST(TfManif, TransformSO3d)
{
  const tf2::Stamped<manif::SO3d> in(manif::SO3d::Identity(), t, f);
  const manif::SE3d se3(Eigen::Vector3d(-1, 2, -3), Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));

  geometry_msgs::TransformStamped trafo = tf2::manifToTransform(se3);
  trafo.header.stamp = ros::Time(10);
  trafo.header.frame_id = "expected";

  tf2::Stamped<manif::SO3d> out;
  tf2::doTransform(in, out, trafo);

  EXPECT_NEAR(out.x(), trafo.transform.rotation.x, EPS);
  EXPECT_NEAR(out.y(), trafo.transform.rotation.y, EPS);
  EXPECT_NEAR(out.z(), trafo.transform.rotation.z, EPS);
  EXPECT_NEAR(out.w(), trafo.transform.rotation.w, EPS);
}

//
// SE3
//

TEST(TfManif, ConvertSE3dStamped)
{
  const manif::SE3d v_nonstamped(Eigen::Vector3d(1,2,3), Eigen::Quaterniond(Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX())));
  const tf2::Stamped<manif::SE3d> v(v_nonstamped, ros::Time(42), "test_frame");

  tf2::Stamped<manif::SE3d> v1;
  geometry_msgs::PoseStamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.translation(), v1.translation());
  EXPECT_EQ(v.rotation(), v1.rotation());
  EXPECT_EQ(v.frame_id_, v1.frame_id_);
  EXPECT_EQ(v.stamp_, v1.stamp_);
}

TEST(TfManif, ConvertSE3d)
{
  const manif::SE3d v(Eigen::Vector3d(1,2,3), Eigen::Quaterniond(Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX())));

  manif::SE3d v1;
  geometry_msgs::Pose p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.translation(), v1.translation());
  EXPECT_EQ(v.rotation(), v1.rotation());
}

TEST(TfManif, ConvertTransform)
{
  manif::SE3d T(M_PI/4.0, M_PI/6.0, M_PI/12.0, 1, 2, 3);

  geometry_msgs::TransformStamped msg = tf2::manifToTransform(T);
  manif::SE3d Tback = tf2::transformToManif(msg);

  EXPECT_TRUE(T.isApprox(Tback, 1e-6));
}

//
// se3
//

TEST(TfManif, ConvertSE3TangentdStamped)
{
  const manif::SE3Tangentd v_nonstamped((Eigen::Matrix<double, 6, 1>() << 1,2,3,4,5,6).finished());
  const tf2::Stamped<manif::SE3Tangentd> v(v_nonstamped, ros::Time(42), "test_frame");

  tf2::Stamped<manif::SE3Tangentd> v1;
  geometry_msgs::TwistStamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.v(), v1.v());
  EXPECT_EQ(v.w(), v1.w());
  EXPECT_EQ(v.frame_id_, v1.frame_id_);
  EXPECT_EQ(v.stamp_, v1.stamp_);
}

TEST(TfManif, ConvertSE3Tangentd)
{
  const manif::SE3Tangentd v((Eigen::Matrix<double, 6, 1>() << 1,2,3,4,5,6).finished());

  manif::SE3Tangentd v1;
  geometry_msgs::Twist p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.v(), v1.v());
  EXPECT_EQ(v.w(), v1.w());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
