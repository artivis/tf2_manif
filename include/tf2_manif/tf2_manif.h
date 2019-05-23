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


#ifndef TF2_MANIF_H
#define TF2_MANIF_H

#include <manif/manif.h>

#include <tf2/convert.h>

// #include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace tf2 {

/** \brief Convert a transform to the equivalent manif data type.
 * \param t The transform to convert, as a geometry_msgs Transform message.
 * \return The transform message converted to an manif SE3d.
 */
inline
manif::SE3d transformToManif(const geometry_msgs::Transform& t)
{
  return manif::SE3d(Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z),
                     Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z));
}

/** \brief Convert a timestamped transform to the equivalent manif data type.
 * \param t The transform to convert, as a geometry_msgs TransformedStamped message.
 * \return The transform message converted to an manif SE3d.
 */
inline
manif::SE3d transformToManif(const geometry_msgs::TransformStamped& t)
{
  return transformToManif(t.transform);
}

/** \brief Convert a manif SE3d to the equivalent geometry_msgs message type.
 * \param s The transform to convert, as a manif SE3d.
 * \return The transform converted to a TransformStamped message.
 */
inline
geometry_msgs::TransformStamped manifToTransform(const manif::SE3d& s)
{
  geometry_msgs::TransformStamped t;
  t.transform.translation.x = s.translation().x();
  t.transform.translation.y = s.translation().y();
  t.transform.translation.z = s.translation().z();

  const Eigen::Quaterniond q(s.quat());
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  return t;
}

//
// Vector3
//
// `manif` only natively supports transforming points atm.
// see also : https://github.com/ros/geometry2/issues/389

/** \brief Convert an Eigen Vector3d type to a Vector3 message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The Eigen Vector3d to convert.
 * \return The vector converted to a Vector3 message.
 */
inline
geometry_msgs::Vector3 toMsg(const Eigen::Vector3d& in)
{
  geometry_msgs::Vector3 out;
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
  return out;
}

/** \brief Convert a timestamped Eigen Vector3d type to a Vector3Stamped message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The timestamped Eigen Vector3d to convert.
 * \return The vector converted to a Vector3Stamped message.
 */
inline
geometry_msgs::Vector3Stamped toMsg(const Stamped<Eigen::Vector3d>& in)
{
  geometry_msgs::Vector3Stamped out;
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  out.vector = toMsg(static_cast<const Eigen::Vector3d&>(in));
  return out;
}

/** \brief Convert a Vector3 message type to a Eigen-specific Vector3d type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * \param msg The Vector3 message to convert.
 * \param out The vector converted to a Eigen Vector3d.
 */
inline
void fromMsg(const geometry_msgs::Vector3& msg, Eigen::Vector3d& out)
{
  out.x() = msg.x;
  out.y() = msg.y;
  out.z() = msg.z;
}

/** \brief Convert a Vector3 message type to a Eigen-specific Vector3d type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * \param msg The Vector3 message to convert.
 * \param out The vector converted to a Eigen Vector3d.
 */
inline
void fromMsg(const geometry_msgs::Vector3Stamped& msg, Stamped<Eigen::Vector3d>& out)
{
  out.frame_id_ = msg.header.frame_id;
  out.stamp_ = msg.header.stamp;
  fromMsg(msg.vector, static_cast<Eigen::Vector3d&>(out));
}

/** \brief Apply a geometry_msgs TransformStamped to an Eigen-specific Vector3d type.
 * \note At the moment, the Eigen Vector3d is treated as a 3D point
 * as we cannot currently differentiate 3d points from vectors !
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as an Eigen Vector3d data type.
 * \param t_out The transformed vector, as an Eigen Vector3d data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const Eigen::Vector3d& t_in,
		             Eigen::Vector3d& t_out,
		             const geometry_msgs::TransformStamped& transform)
{
  t_out = transformToManif(transform).act(t_in);
}

/** \brief Apply a geometry_msgs TransformStamped to a timestamped Eigen-specific Vector3d type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The timestamped vector to transform, as a timestamped Eigen Vector3d data type.
 * \param t_out The transformed vector, as a timestamped Eigen Vector3d data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const tf2::Stamped<Eigen::Vector3d>& t_in,
		             tf2::Stamped<Eigen::Vector3d>& t_out,
		             const geometry_msgs::TransformStamped& transform)
{
  t_out.frame_id_ = transform.header.frame_id;
  t_out.stamp_ = transform.header.stamp;
  doTransform(static_cast<const Eigen::Vector3d&>(t_in),
              static_cast<Eigen::Vector3d&>(t_out),
              transform);
}

//
// Point
//

// /** \brief Convert an Eigen Vector3d type to a Point message.
//  * This function is a specialization of the toMsg template defined in tf2/convert.h.
//  * \param in The Eigen Vector3d to convert.
//  * \return The vector converted to a Point message.
//  */
// inline
// geometry_msgs::Point toMsg(const Eigen::Vector3d& in)
// {
//   geometry_msgs::Point msg;
//   msg.x = in.x();
//   msg.y = in.y();
//   msg.z = in.z();
//   return msg;
// }
//
// /** \brief Convert a timestamped Eigen Vector3d type to a PointStamped message.
//  * This function is a specialization of the toMsg template defined in tf2/convert.h.
//  * \param in The timestamped Eigen Vector3d to convert.
//  * \return The vector converted to a PointStamped message.
//  */
// inline
// geometry_msgs::PointStamped toMsg(const tf2::Stamped<Eigen::Vector3d>& in)
// {
//   geometry_msgs::PointStamped msg;
//   msg.header.stamp = in.stamp_;
//   msg.header.frame_id = in.frame_id_;
//   msg.point = toMsg(static_cast<const Eigen::Vector3d&>(in));
//   return msg;
// }
//
// /** \brief Convert a Point message type to an Eigen-specific Vector3d type.
//  * This function is a specialization of the fromMsg template defined in tf2/convert.h
//  * \param msg The Point message to convert.
//  * \param out The point converted to a Eigen Vector3d.
//  */
// inline
// void fromMsg(const geometry_msgs::Point& msg, Eigen::Vector3d& out)
// {
//   out.x() = msg.x;
//   out.y() = msg.y;
//   out.z() = msg.z;
// }
//
// /** \brief Convert a PointStamped message type to a timestamped Eigen-specific Vector3d type.
//  * This function is a specialization of the fromMsg template defined in tf2/convert.h
//  * \param msg The PointStamped message to convert.
//  * \param out The point converted to a timestamped Eigen Vector3d.
//  */
// inline
// void fromMsg(const geometry_msgs::PointStamped& msg, tf2::Stamped<Eigen::Vector3d>& out)
// {
//   out.frame_id_ = msg.header.frame_id;
//   out.stamp_ = msg.header.stamp;
//   fromMsg(msg.point, static_cast<Eigen::Vector3d&>(out));
// }

//
// SO3
//

/** \brief Convert a manif SO3d type to a Quaternion message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The manif SO3d to convert.
 * \return The quaternion converted to a Quaterion message.
 */
inline
geometry_msgs::Quaternion toMsg(const manif::SO3d& in)
{
  geometry_msgs::Quaternion msg;
  const Eigen::Quaterniond q(in.quat());
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

/** \brief Convert a timestamped manif SO3d type to a QuaternionStamped message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The timestamped manif SO3d to convert.
 * \return The quaternion converted to a QuaternionStamped message.
 */
inline
geometry_msgs::QuaternionStamped toMsg(const Stamped<manif::SO3d>& in)
{
  geometry_msgs::QuaternionStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.quaternion = toMsg(static_cast<const manif::SO3d&>(in));
  return msg;
}

/** \brief Convert a Quaternion message type to a manif SO3d type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * \param msg The Quaternion message to convert.
 * \param out The quaternion converted to a manif SO3d.
 */
inline
void fromMsg(const geometry_msgs::Quaternion& msg, manif::SO3d& out)
{
  out = manif::SO3d(msg.x, msg.y, msg.z, msg.w);
}

/** \brief Convert a QuaternionStamped message type to a timestamped manif SO3d.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * \param msg The QuaternionStamped message to convert.
 * \param out The quaternion converted to a timestamped manif SO3d.
 */
inline
void fromMsg(const geometry_msgs::QuaternionStamped& msg, Stamped<manif::SO3d>& out)
{
  out.frame_id_ = msg.header.frame_id;
  out.stamp_ = msg.header.stamp;
  fromMsg(msg.quaternion, static_cast<manif::SO3d&>(out));
}

/** \brief Apply a geometry_msgs TransformStamped to a manif SO3d type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * functions rely on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The vector to transform, as a manif SO3d data type.
 * \param t_out The transformed vector, as a manif SO3d data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(const manif::SO3d& t_in,
                 manif::SO3d& t_out,
                 const geometry_msgs::TransformStamped& transform)
{
  manif::SO3d t;
  fromMsg(transform.transform.rotation, t);
  t_out = t * t_in;
}

/** \brief Apply a geometry_msgs TransformStamped to a manif SO3d type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a timestamped manif SO3d data type.
 * \param t_out The transformed vector, as a timestamped manif SO3d data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const tf2::Stamped<manif::SO3d>& t_in,
                 tf2::Stamped<manif::SO3d>& t_out,
                 const geometry_msgs::TransformStamped& transform)
{
  t_out.frame_id_ = transform.header.frame_id;
  t_out.stamp_ = transform.header.stamp;
  doTransform(static_cast<const manif::SO3d&>(t_in),
              static_cast<manif::SO3d&>(t_out),
              transform);
}

//
// SE3
//

/** \brief Convert a manif SE3d type to a Pose message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The manif SE3d to convert.
 * \return The manif transform converted to a Pose message.
 */
inline
geometry_msgs::Pose toMsg(const manif::SE3d& in)
{
  geometry_msgs::Pose msg;
  msg.position.x = in.x();
  msg.position.y = in.y();
  msg.position.z = in.z();
  const Eigen::Quaterniond q(in.quat());
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  if (msg.orientation.w < 0)
  {
    msg.orientation.x *= -1;
    msg.orientation.y *= -1;
    msg.orientation.z *= -1;
    msg.orientation.w *= -1;
  }
  return msg;
}

/** \brief Convert a timestamped manif SE3d to a Pose message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The timestamped manif Affine3d to convert.
 * \return The manif transform converted to a PoseStamped message.
 */
inline
geometry_msgs::PoseStamped toMsg(const tf2::Stamped<manif::SE3d>& in)
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.pose = toMsg(static_cast<const manif::SE3d&>(in));
  return msg;
}

/** \brief Convert a Pose message transform type to a manif SE3d.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg The Pose message to convert.
 * \param out The pose converted to a manif SE3d.
 */
inline
void fromMsg(const geometry_msgs::Pose& msg, manif::SE3d& out)
{
  out = manif::SE3d(Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z),
                    Eigen::Quaterniond(msg.orientation.w,
                                       msg.orientation.x,
                                       msg.orientation.y,
                                       msg.orientation.z));
}

/** \brief Convert a Pose message transform type to a timestamped manif SE3d.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg The PoseStamped message to convert.
 * \param out The pose converted to a timestamped manif SE3d.
 */
inline
void fromMsg(const geometry_msgs::PoseStamped& msg, tf2::Stamped<manif::SE3d>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  fromMsg(msg.pose, static_cast<manif::SE3d&>(out));
}

/** \brief Apply a geometry_msgs TransformStamped to an manif SE3d transform.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * function relies on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The frame to transform, as a timestamped manif SE3d transform.
 * \param t_out The transformed frame, as a timestamped manif SE3d transform.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const manif::SE3d& t_in,
            		 manif::SE3d& t_out,
            		 const geometry_msgs::TransformStamped& transform)
{
  t_out = transformToManif(transform) * t_in;
}

/** \brief Apply a geometry_msgs TransformStamped to an manif SE3d transform.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,.
 * \param t_in The frame to transform, as a timestamped manif SE3d transform.
 * \param t_out The transformed frame, as a timestamped manif SE3d transform.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const tf2::Stamped<manif::SE3d>& t_in,
            		 tf2::Stamped<manif::SE3d>& t_out,
            		 const geometry_msgs::TransformStamped& transform)
{
  t_out.frame_id_ = transform.header.frame_id;
  t_out.stamp_ = transform.header.stamp;
  doTransform(static_cast<const manif::SE3d&>(t_in),
              static_cast<manif::SE3d&>(t_out),
              transform);
}

//
// se3
//

/** \brief Convert a manif SE3Tangentd type to a Twist message.
* This function is a specialization of the toMsg template defined in tf2/convert.h.
* \param in The manif SE3Tangentd to convert.
* \return The manif SE3Tangentd converted to a Twist message.
*/
inline
geometry_msgs::Twist toMsg(const manif::SE3Tangentd& in)
{
  const manif::SE3Tangentd::DataType data_in = in.coeffs();
  geometry_msgs::Twist msg;
  msg.linear.x  = data_in[0];
  msg.linear.y  = data_in[1];
  msg.linear.z  = data_in[2];
  msg.angular.x = data_in[3];
  msg.angular.y = data_in[4];
  msg.angular.z = data_in[5];
  return msg;
}

/** \brief Convert a timestamped manif SE3Tangentd type to a TwistStamped message.
* This function is a specialization of the toMsg template defined in tf2/convert.h.
* \param in The timestamped manif SE3Tangentd to convert.
* \return The timestamped manif SE3Tangentd converted to a TwistStamped message.
*/
inline
geometry_msgs::TwistStamped toMsg(const tf2::Stamped<manif::SE3Tangentd>& in)
{
  geometry_msgs::TwistStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.twist = toMsg(static_cast<const manif::SE3Tangentd&>(in));
  return msg;
}

/** \brief Convert a Twist message transform type to a manif SE3Tangentd.
* This function is a specialization of the toMsg template defined in tf2/convert.h.
* \param msg The Twist message to convert.
* \param out The twist converted to a manif SE3Tangentd.
*/
inline
void fromMsg(const geometry_msgs::Twist &msg, manif::SE3Tangentd& out)
{
  out.coeffs()[0] = msg.linear.x;
  out.coeffs()[1] = msg.linear.y;
  out.coeffs()[2] = msg.linear.z;
  out.coeffs()[3] = msg.angular.x;
  out.coeffs()[4] = msg.angular.y;
  out.coeffs()[5] = msg.angular.z;
}

/** \brief Convert a TwistStamped message transform type to a timestamped manif SE3Tangentd.
* This function is a specialization of the toMsg template defined in tf2/convert.h.
* \param msg The TwistStamped message to convert.
* \param out The twist converted to a timestamped manif SE3Tangentd.
*/
inline
void fromMsg(const geometry_msgs::TwistStamped &msg, tf2::Stamped<manif::SE3Tangentd>& out)
{
  out.frame_id_ = msg.header.frame_id;
  out.stamp_ = msg.header.stamp;
  fromMsg(msg.twist, static_cast<manif::SE3Tangentd&>(out));
}

// There is a lack of clarity of the twist message representation
// see http://wiki.ros.org/tf/Reviews/2010-03-12_API_Review

// /** \brief Apply a geometry_msgs TransformStamped to an manif SE3Tangentd.
// * This function is a specialization of the doTransform template defined in tf2/convert.h,
// * although it can not be used in tf2_ros::BufferInterface::transform because this
// * function relies on the existence of a time stamp and a frame id in the type which should
// * get transformed.
// * \param t_in The frame to transform, as a timestamped manif SE3Tangentd.
// * \param t_out The transformed frame, as a timestamped manif SE3Tangentd.
// * \param transform The timestamped transform to apply, as a TransformStamped message.
// */
// template <>
// inline
// void doTransform(const manif::SE3Tangentd& t_in,
//            		   manif::SE3Tangentd& t_out,
//            		   const geometry_msgs::TransformStamped& transform)
// {
//   Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();
//   R.topLeftCorner<3,3>() = transformToManif(transform).rotation();
//   t_out = R * t_in;
// }
//
// /** \brief Apply a geometry_msgs TransformStamped to an manif SE3Tangentd.
// * This function is a specialization of the doTransform template defined in tf2/convert.h,.
// * \param t_in The frame to transform, as a timestamped manif SE3Tangentd.
// * \param t_out The transformed frame, as a timestamped manif SE3Tangentd.
// * \param transform The timestamped transform to apply, as a TransformStamped message.
// */
// template <>
// inline
// void doTransform(const tf2::Stamped<manif::SE3Tangentd>& t_in,
//            		   tf2::Stamped<manif::SE3Tangentd>& t_out,
//            		   const geometry_msgs::TransformStamped& transform)
// {
//   t_out.frame_id_ = transform.header.frame_id;
//   t_out.stamp_ = transform.header.stamp;
//   doTransform(static_cast<const manif::SE3Tangentd&>(t_in),
//               static_cast<manif::SE3Tangentd&>(t_out),
//               transform);
// }

} // namespace tf2

// This is needed to make the usage of the following conversion functions usable in tf2::convert().
// According to clangs error note 'fromMsg'/'toMsg' should be declared prior to the call site or
// in an associated namespace of one of its arguments. The stamped versions of this conversion
// functions work because they have tf2::Stamped as an argument which is the same namespace as
// which 'fromMsg'/'toMsg' is defined in. The non-stamped versions have no argument which is
// defined in tf2, so it take the following definitions in Eigen namespace to make them usable in
// tf2::convert().

namespace Eigen {

inline
geometry_msgs::Vector3 toMsg(const Eigen::Vector3d& in) {
  return tf2::toMsg(in);
}

inline
void fromMsg(const geometry_msgs::Vector3& msg, Eigen::Vector3d& out) {
  tf2::fromMsg(msg, out);
}

} // namespace Eigen

namespace manif {

inline
geometry_msgs::Quaternion toMsg(const manif::SO3d& in) {
  return tf2::toMsg(in);
}

inline
void fromMsg(const geometry_msgs::Quaternion& msg, manif::SO3d& out) {
  tf2::fromMsg(msg, out);
}

inline
geometry_msgs::Pose toMsg(const manif::SE3d& in) {
  return tf2::toMsg(in);
}

inline
void fromMsg(const geometry_msgs::Pose& msg, manif::SE3d& out) {
  tf2::fromMsg(msg, out);
}

inline
geometry_msgs::Twist toMsg(const manif::SE3Tangentd& in) {
  return tf2::toMsg(in);
}

inline
void fromMsg(const geometry_msgs::Twist &msg, manif::SE3Tangentd& out) {
  tf2::fromMsg(msg, out);
}

} // namespace manif

#endif // TF2_MANIF_H
