/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/sensor_bridge.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. See 1.7 in "
                                  "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

}  // namespace

absl::optional<::cartographer::transform::Rigid3d>
    SensorBridge::ecef_to_local_frame_;

SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,
    const std::string& tracking_frame,
    const double lookup_transform_timeout_sec, tf2_ros::Buffer* const tf_buffer,
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder,
    const double nav_sat_translation_weight,
    double nav_sat_inverse_covariance_bias,
    double nav_sat_inverse_covariance_weight,
    const double position_translation_weight,
    const sensor_msgs::NavSatFix::ConstPtr& predefined_enu_frame_position)
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
      trajectory_builder_(trajectory_builder),
      nav_sat_translation_weight_(nav_sat_translation_weight),
      nav_sat_inverse_covariance_bias_(nav_sat_inverse_covariance_bias),
      nav_sat_inverse_covariance_weight_(nav_sat_inverse_covariance_weight),
      position_translation_weight_(position_translation_weight),
      predefined_enu_frame_position_(predefined_enu_frame_position) {}

std::unique_ptr<carto::sensor::OdometryData> SensorBridge::ToOdometryData(
    const nav_msgs::Odometry::ConstPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  return absl::make_unique<carto::sensor::OdometryData>(
      carto::sensor::OdometryData{
          time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
}

void SensorBridge::HandleOdometryMessage(
    const std::string& sensor_id, const nav_msgs::Odometry::ConstPtr& msg) {
  std::unique_ptr<carto::sensor::OdometryData> odometry_data =
      ToOdometryData(msg);
  if (odometry_data != nullptr) {
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
  }
}

ros::Publisher* ecef_to_local_pub;
ros::Publisher* ecef_to_local_fix_pub;

void SensorBridge::HandleTransformMessage(
    const std::string& sensor_id, const geometry_msgs::TransformStamped::ConstPtr& msg) {

  // POZYX messages don't have covariance info. Default to I.
  // POZYX z is bad so set its inverse covariance to 0. TODO: separate z translation weight
  std::array<double, 9> inverse_covariance{{1,0,0,0,1,0,0,0, 0 }};

  auto translation = Eigen::Vector3d(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);

  trajectory_builder_->AddSensorData(
      sensor_id,
      carto::sensor::LandmarkData{
          cartographer_ros::FromRos(msg->header.stamp),
          std::vector<carto::sensor::LandmarkObservation>{
              carto::sensor::LandmarkObservation{
                  "fixed",
                  Rigid3d::Translation(translation),
                  position_translation_weight_, 0. /* rotation_weight */,
                  false /* observed_from_tracking */,
                  inverse_covariance}}});
}

void SensorBridge::HandleNavSatFixMessage(
    const std::string& sensor_id, const sensor_msgs::NavSatFix::ConstPtr& msg) {
  if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    // If collation is turned off, we do not have to insert "no observations"
    // into the sensor queue.
    return;
  }

  if (ecef_to_local_pub == nullptr) {
    ecef_to_local_pub = new ros::Publisher();
    *ecef_to_local_pub = ros::NodeHandle().advertise<geometry_msgs::Transform>(
        "ecef_to_local", 1, true);
  }
  if (ecef_to_local_fix_pub == nullptr) {
    ecef_to_local_fix_pub = new ros::Publisher();
    *ecef_to_local_fix_pub =
        ros::NodeHandle().advertise<sensor_msgs::NavSatFix>(
            "ecef_to_local_navsat_fix", 1, true);
  }

  if (!ecef_to_local_frame_.has_value()) {
    if (predefined_enu_frame_position_) {
      ecef_to_local_frame_ = ComputeLocalFrameFromLatLong(
          predefined_enu_frame_position_->latitude,
          predefined_enu_frame_position_->longitude,
          predefined_enu_frame_position_->altitude);
    } else {
      ecef_to_local_frame_ = ComputeLocalFrameFromLatLong(
          msg->latitude, msg->longitude, msg->altitude);
    }
    LOG(INFO) << "Using NavSatFix. Setting ecef_to_local_frame with lat = "
              << msg->latitude << ", long = " << msg->longitude
              << ", alt = " << msg->altitude;
    ecef_to_local_pub->publish(
        ToGeometryMsgTransform(ecef_to_local_frame_.value()));
    ecef_to_local_fix_pub->publish(msg);
  }

  std::array<double, 9> inverse_covariance;
  for (std::size_t i = 0; i < inverse_covariance.size(); i++) {
    if (msg->position_covariance.at(i) < 1e-5) {
      // Avoid division by zero when calculating inverse covariance
      inverse_covariance[i] = nav_sat_inverse_covariance_bias_;
    } else {
      // Calculate weighted inverse covariance
      inverse_covariance[i] =
          nav_sat_inverse_covariance_bias_ +
          nav_sat_inverse_covariance_weight_ / msg->position_covariance.at(i);
    }
  }

  const carto::sensor::LandmarkData landmark_data{
          cartographer_ros::FromRos(msg->header.stamp),
          std::vector<carto::sensor::LandmarkObservation>{
              carto::sensor::LandmarkObservation{
                  "fixed",
                  Rigid3d::Translation(ecef_to_local_frame_.value() *
                                       LatLongAltToEcef(msg->latitude,
                                                        msg->longitude,
                                                        msg->altitude)),
                  nav_sat_translation_weight_, 0. /* rotation_weight */,
                  false /* observed_from_tracking */, inverse_covariance}}};


  if (false) {
    static std::ofstream out("/tmp/gps.log");
    const auto& tr = landmark_data.landmark_observations[0].landmark_to_tracking_transform.translation();
    out << msg->header.stamp.sec << " " << msg->header.stamp.nsec << " "
        << msg->header.stamp.toSec()
        << " " << tr.x() << " " << tr.y() << " " << tr.z() << std::endl;
  }

  trajectory_builder_->AddSensorData(sensor_id, landmark_data);
}

void SensorBridge::HandleLandmarkMessage(
    const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
  auto landmark_data = ToLandmarkData(*msg);

  auto tracking_from_landmark_sensor = tf_bridge_.LookupToTracking(
      landmark_data.time, CheckNoLeadingSlash(msg->header.frame_id));
  if (tracking_from_landmark_sensor != nullptr) {
    for (auto& observation : landmark_data.landmark_observations) {
      observation.landmark_to_tracking_transform =
          *tracking_from_landmark_sensor *
          observation.landmark_to_tracking_transform;
    }
  }
  trajectory_builder_->AddSensorData(sensor_id, landmark_data);
}

std::unique_ptr<carto::sensor::ImuData> SensorBridge::ToImuData(
    const sensor_msgs::Imu::ConstPtr& msg) {
  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  CHECK_NE(msg->angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  CHECK(sensor_to_tracking->translation().norm() < 1e-5)
      << "The IMU frame must be colocated with the tracking frame. "
         "Transforming linear acceleration into the tracking frame will "
         "otherwise be imprecise.";
  return absl::make_unique<carto::sensor::ImuData>(carto::sensor::ImuData{
      time, sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
      sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});
}

void SensorBridge::HandleImuMessage(const std::string& sensor_id,
                                    const sensor_msgs::Imu::ConstPtr& msg) {
  std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);
  if (imu_data != nullptr) {
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
                               imu_data->angular_velocity});
  }
}

void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id, const sensor_msgs::LaserScan::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandleMultiEchoLaserScanMessage(
    const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandlePointCloud2Message(
    const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleRangefinder(sensor_id, time, msg->header.frame_id, point_cloud.points);
}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }

void SensorBridge::HandleLaserScan(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id,
    const carto::sensor::PointCloudWithIntensities& points) {
  if (points.points.empty()) {
    return;
  }
  CHECK_LE(points.points.back().time, 0.f);
  // TODO(gaschler): Use per-point time instead of subdivisions.
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
    const size_t start_index =
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
    carto::sensor::TimedPointCloud subdivision(
        points.points.begin() + start_index, points.points.begin() + end_index);
    if (start_index == end_index) {
      continue;
    }
    const double time_to_subdivision_end = subdivision.back().time;
    // `subdivision_time` is the end of the measurement so sensor::Collator will
    // send all other sensor data first.
    const carto::common::Time subdivision_time =
        time + carto::common::FromSeconds(time_to_subdivision_end);
    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
    if (it != sensor_to_previous_subdivision_time_.end() &&
        it->second >= subdivision_time) {
      LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time;
      continue;
    }
    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
    for (auto& point : subdivision) {
      point.time -= time_to_subdivision_end;
    }
    CHECK_EQ(subdivision.back().time, 0.f);
    HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
  }
}

void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id, const carto::sensor::TimedPointCloud& ranges) {
  if (!ranges.empty()) {
    CHECK_LE(ranges.back().time, 0.f);
  }
  const auto sensor_to_tracking =
      tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddSensorData(
        sensor_id, carto::sensor::TimedPointCloudData{
                       time, sensor_to_tracking->translation().cast<float>(),
                       carto::sensor::TransformTimedPointCloud(
                           ranges, sensor_to_tracking->cast<float>())});
  }
}

}  // namespace cartographer_ros
