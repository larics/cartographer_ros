#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_FRONTIER_DETECTION_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_FRONTIER_DETECTION_H

#include <cartographer/io/submap_painter.h>
#include <cartographer/mapping/id.h>
#include <cartographer/mapping/internal/2d/pose_graph_2d.h>
#include <cartographer/transform/rigid_transform.h>
#include <cartographer_ros_msgs/SubmapList.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <atomic>
#include <mutex>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace frontier {

// const double kFreeProbability = 0.56;
// const double kOccupiedProbability = 0.5;

// cartographer::mapping::ProbabilityToValue(kFreeProbability) 18841
// cartographer::mapping::ProbabilityToValue(kOccupiedProbability) 16384
constexpr uint16_t kFreeProbabilityValue = 18841;
constexpr uint16_t kOccupiedProbabilityValue = 16384;

namespace bg = boost::geometry;
namespace bgi = bg::index;

inline Eigen::Isometry2d rigid3d_to_isometry2d(
    const cartographer::transform::Rigid3d& rigid) {
  return Eigen::Translation2d(rigid.translation().head<2>()) *
         Eigen::Rotation2Dd(
             rigid.rotation().toRotationMatrix().block<2, 2>(0, 0));
}

class Detector {
 public:
  Detector(cartographer::mapping::PoseGraph2D* pose_graph);

  void InitPublisher();

  // Performs local frontier edge detection.
  void HandleSubmapUpdates(
      const std::vector<cartographer::mapping::SubmapId>& submap_ids);

  void PublishAllSubmaps();
  void PublishSubmaps(
      const std::vector<cartographer::mapping::SubmapId>& submap_ids,
      const std::vector<cartographer::mapping::SubmapId>& additional_submaps);

  bool CheckForOptimizationEvent();

  std::vector<cartographer::mapping::SubmapId> GetIntersectingFinishedSubmaps(
      const cartographer::mapping::SubmapId& id_i);

 private:
  using Point = bg::model::point<double, 2, bg::cs::cartesian>;
  using Box = bg::model::box<Point>;
  using Value = std::pair<Box, cartographer::mapping::SubmapId>;
  using RTree = bgi::rtree<Value, bgi::quadratic<16, 4>>;

  RTree rt_;

  std::mutex mutex_;

  std::map<cartographer::mapping::SubmapId,
           std::pair<
               Eigen::Matrix3Xd,
               std::vector<cartographer::mapping::SubmapId> /* submap_hints */>>
      submap_frontier_cells_;

  struct BoundingBoxInfo {
    std::pair<Eigen::Vector3d, Eigen::Vector3d> local_box;
    Box last_global_box;
  };
  std::map<cartographer::mapping::SubmapId, BoundingBoxInfo> bounding_boxes_;

  std::vector<cartographer::mapping::SubmapId> active_submaps_;

  // cartographer_ros_msgs::SubmapList::ConstPtr last_submap_list_;
  ros::Publisher frontier_publisher_;
  bool publisher_initialized_;
  int last_optimizations_performed_;

  cartographer::mapping::PoseGraph2D* pose_graph_;

  struct Submap {
    Submap(const cartographer::mapping::SubmapId& id,
           const cartographer::mapping::PoseGraphInterface::SubmapData&
               submap_data)
        : id(id),
          submap(*static_cast<const cartographer::mapping::Submap2D*>(
              submap_data.submap.get())),
          pose(submap_data.pose), /* * submap.local_pose().inverse() *
              cartographer::transform::Rigid3d::Translation(
                  (Eigen::Vector3d() << limits.max(), 0.).finished())) */
          local_pose_inverse(submap.local_pose().inverse()),
          to_global_position(rigid3d_to_isometry2d(pose * local_pose_inverse)),
          to_local_submap_position(to_global_position.inverse()) {
      frontier_marker.header.frame_id = "map";
      frontier_marker.pose.orientation.w = 1.0;
      frontier_marker.type = visualization_msgs::Marker::POINTS;
      frontier_marker.scale.x = 0.075;
      frontier_marker.scale.y = 0.075;
      frontier_marker.color.r = 1.0;
      frontier_marker.color.a = 1.0;
      std::ostringstream ss;
      ss << "Trajectory " << id.trajectory_id << ", submap " << id.submap_index;
      frontier_marker.ns = ss.str();
    }

    const cartographer::mapping::SubmapId id;
    const cartographer::mapping::Submap2D& submap;
    const cartographer::transform::Rigid3d pose;
    const cartographer::transform::Rigid3d local_pose_inverse;
    const Eigen::Isometry2d to_global_position;
    const Eigen::Isometry2d to_local_submap_position;
    Eigen::Matrix2Xd cached_frontier_marker_cells_global;
    visualization_msgs::Marker frontier_marker;

    const cartographer::mapping::Grid2D& grid() const { return *submap.grid(); }
    const cartographer::mapping::MapLimits& limits() const {
      return submap.grid()->limits();
    }

    int ToFlatIndex(const Eigen::Array2i& cell_index) const {
      return limits().cell_limits().num_x_cells * cell_index.y() +
             cell_index.x();
    }

    bool is_in_limits(const Eigen::Array2i& xy_index) const {
      return limits().Contains(xy_index);
    }

    bool is_free(const Eigen::Array2i& xy_index) const {
      return is_in_limits(xy_index) &&
             grid().correspondence_cost_cells_[ToFlatIndex(xy_index)] >=
                 kFreeProbabilityValue;
    }

    bool is_unknown(const Eigen::Array2i& xy_index) const {
      if (!is_in_limits(xy_index)) return true;
      const int& value =
          grid().correspondence_cost_cells_[ToFlatIndex(xy_index)];
      return value == 0 ||
             (value > kOccupiedProbabilityValue && !is_free(xy_index));
    }

    bool is_known(const Eigen::Vector3d& local_position_in_submap) const {
      return !is_unknown(limits().GetCellIndex(
          local_position_in_submap.head<2>().cast<float>()));
    }
  };

  class SubmapCache {
   public:
    SubmapCache(const cartographer::mapping::PoseGraph2D* const pose_graph)
        : pose_graph_(pose_graph) {}
    Detector::Submap& operator()(
        const cartographer::mapping::SubmapId& id) const {
      return *IfExists(id);
    }

    Submap* IfExists(const cartographer::mapping::SubmapId& id) const {
      auto iter = submaps_.lower_bound(id);
      if (iter != submaps_.end() && iter->first == id) return &iter->second;
      auto submap_data = pose_graph_->GetSubmapData(id);
      if (submap_data.submap != nullptr) {
        // LOG(INFO) << "Cache has seen a new submap " << id;
        return &submaps_
                    .emplace_hint(
                        iter,
                        std::make_pair(id, Detector::Submap(id, submap_data)))
                    ->second;
      } else {
        return nullptr;
      }
    }

    cartographer::mapping::MapById<
        cartographer::mapping::SubmapId,
        cartographer::mapping::PoseGraphInterface::SubmapData>&
    last_all_submap_data() {
      return last_all_submap_data_;
    }

    void Invalidate() {
      // TODO update only poses instead of clearing out everything
      submaps_.clear();
      last_all_submap_data_ = pose_graph_->GetAllSubmapData();

      for (const auto& submap_data : last_all_submap_data_) {
        submaps_.emplace(
            std::make_pair(submap_data.id,
                           Detector::Submap(submap_data.id, submap_data.data)));
      }
    }

   private:
    using SubmapPair =
        std::pair<cartographer::mapping::SubmapId, Detector::Submap>;
    mutable std::map<cartographer::mapping::SubmapId, Detector::Submap>
        submaps_;
    const cartographer::mapping::PoseGraph2D* const pose_graph_;

    cartographer::mapping::MapById<
        cartographer::mapping::SubmapId,
        cartographer::mapping::PoseGraphInterface::SubmapData>
        last_all_submap_data_;
  };

  SubmapCache submaps_;

  // Goes through previously edge-detected local frontier points in submaps,
  // checks if they really are frontier points by looking in other submaps,
  // and creates a marker containing the appropriate frontier points.
  visualization_msgs::Marker& CreateMarkerForSubmap(
      const cartographer::mapping::SubmapId& id_i,
      const std::vector<cartographer::mapping::SubmapId>* updated_submaps,
      bool check_against_active);

  Box CalculateBoundingBox(const Submap& submap) {
    auto& bounding_box_info = bounding_boxes_[submap.id];
    const Eigen::Vector3d p1_global =
        submap.pose * bounding_box_info.local_box.first;
    const Eigen::Vector3d p2_global =
        submap.pose * bounding_box_info.local_box.second;

    /*visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    std::ostringstream ss;
    ss << "BBOX Trajectory " <<submap.id.trajectory_id << ", submap " <<
    submap.id.submap_index; marker.ns = ss.str();

    geometry_msgs::Point point;
    point.x = p1_global.x();
    point.y = p1_global.y();
    marker.points.push_back(point);
    point.x = p2_global.x();
    point.y = p2_global.y();
    marker.points.push_back(point);

    visualization_msgs::MarkerArray frontier_markers;
    frontier_markers.markers.push_back(marker);
    frontier_publisher_.publish(frontier_markers);*/

    return Box{Point(std::min(p1_global.x(), p2_global.x()),
                     std::min(p1_global.y(), p2_global.y())),
               Point(std::max(p1_global.x(), p2_global.x()),
                     std::max(p1_global.y(), p2_global.y()))};
  }

  void RebuildTree();
};

}  // namespace frontier

#endif