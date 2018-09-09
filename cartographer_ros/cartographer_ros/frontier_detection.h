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

namespace cartographer_ros {
class MapBuilderBridge;
}

namespace frontier {

// const double kFreeProbability = 0.56;
// const double kOccupiedProbability = 0.5;

// cartographer::mapping::ProbabilityToValue(kFreeProbability) 18841
// cartographer::mapping::ProbabilityToValue(kOccupiedProbability) 16384

constexpr uint16_t kFreeProbabilityValue = 18841;
constexpr uint16_t kOccupiedProbabilityValue = 16384;

namespace bg = boost::geometry;
namespace bgi = bg::index;

class Detector {
 public:
  Detector(cartographer::mapping::PoseGraph2D* pose_graph, bool publish = true);

  void InitPublisher();

  void HandleSubmapUpdate(const cartographer::mapping::SubmapId& id);

  void PublishAllSubmaps();
  void PublishSubmaps(
      const std::vector<cartographer::mapping::SubmapId>& submap_ids);

  bool CheckForOptimizationEvent() {
    int actual_optimizations_performed = pose_graph_->optimizations_performed();
    bool publish_all = false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (actual_optimizations_performed != last_optimizations_performed_) {
        last_optimizations_performed_ = actual_optimizations_performed;
        publish_all = true;
      }
    }
    if (publish_all) {
      PublishAllSubmaps();
      return true;
    } else
      return false;
  }

  std::vector<cartographer::mapping::SubmapId> GetIntersectingFinishedSubmaps(
      const cartographer::mapping::SubmapId& id_i);

 private:
  using Point = bg::model::point<double, 2, bg::cs::cartesian>;
  using Box = bg::model::box<Point>;
  using Value = std::pair<Box, cartographer::mapping::SubmapId>;

  bgi::rtree<Value, bgi::quadratic<16, 4>> rt_;

  std::mutex mutex_;

  std::map<
      cartographer::mapping::SubmapId,
      std::vector<std::pair<Eigen::Vector3d,
                            cartographer::mapping::SubmapId /* submap_hint */>>>
      submap_frontier_cells_;

  struct BoundingBoxInfo {
    std::pair<Eigen::Vector3d, Eigen::Vector3d> local_box;
    Box last_global_box;
  };
  std::map<cartographer::mapping::SubmapId, BoundingBoxInfo> bounding_boxes_;

  std::vector<cartographer::mapping::SubmapId> active_submaps_;

  // cartographer_ros_msgs::SubmapList::ConstPtr last_submap_list_;
  ros::Publisher frontier_publisher_;
  int last_optimizations_performed_;

  bool publish_;

  cartographer::mapping::PoseGraph2D* pose_graph_;

  struct Submap {
    Submap(cartographer::mapping::SubmapId id,
           const cartographer::mapping::PoseGraphInterface::SubmapData&
               submap_data)
        : id(id),
          submap(*static_cast<const cartographer::mapping::Submap2D*>(
              submap_data.submap.get())),
          grid(*submap.grid()),
          limits(grid.limits()),
          pose(submap_data.pose) /* * submap.local_pose().inverse() *
              cartographer::transform::Rigid3d::Translation(
                  (Eigen::Vector3d() << limits.max(), 0.).finished())) */
          {};
    const cartographer::mapping::SubmapId& id;
    const cartographer::mapping::Submap2D& submap;
    const cartographer::mapping::Grid2D& grid;
    const cartographer::mapping::MapLimits& limits;
    const cartographer::transform::Rigid3d pose;

    int ToFlatIndex(const Eigen::Array2i& cell_index) const {
      return grid.limits_.cell_limits().num_x_cells * cell_index.y() +
             cell_index.x();
    }

    bool is_in_limits(const Eigen::Array2i& xy_index) const {
      return grid.limits().Contains(xy_index);
    }
    bool is_free(const Eigen::Array2i& xy_index) const {
      return is_in_limits(xy_index) &&
             grid.correspondence_cost_cells_[ToFlatIndex(xy_index)] >=
                 kFreeProbabilityValue;
    }

    bool is_unknown(const Eigen::Array2i& xy_index) const {
      if (!is_in_limits(xy_index)) return true;
      const int& value = grid.correspondence_cost_cells_[ToFlatIndex(xy_index)];
      return value == 0 ||
             (value >= kOccupiedProbabilityValue && !is_free(xy_index));
    }

    bool is_known(const Eigen::Vector3d& global_position) const {
      const Eigen::Vector3d position_in_submap =
          submap.local_pose() * pose.inverse() * global_position;
      return !is_unknown(
          limits.GetCellIndex(position_in_submap.head<2>().cast<float>()));
    }
  };

  template <class T>
  visualization_msgs::Marker CreateMarkerForSubmap(
      const cartographer::mapping::SubmapId& id_i,
      const T& submap_data_getter) {
    Submap s_i(id_i, submap_data_getter(id_i));

    visualization_msgs::Marker frontier_marker;
    frontier_marker.header.frame_id = "map";
    frontier_marker.pose.orientation.w = 1.0;
    frontier_marker.type = visualization_msgs::Marker::POINTS;
    frontier_marker.scale.x = 0.1;
    frontier_marker.scale.y = 0.1;
    frontier_marker.color.r = 1.0;
    frontier_marker.color.a = 1.0;
    frontier_marker.ns = std::to_string(s_i.id.submap_index);

    auto& submap_frontier_cells = submap_frontier_cells_.at(s_i.id);
    auto& bounding_box = bounding_boxes_.at(s_i.id);

    std::vector<Value> intersecting_submaps;
    rt_.query(bgi::intersects(bounding_box.last_global_box),
              std::back_inserter(intersecting_submaps));

    for (auto& frontier_cell : submap_frontier_cells) {
      const auto global_position = s_i.pose * frontier_cell.first;

      bool ok = true;
      auto& submap_hint = frontier_cell.second;

      if (submap_hint != cartographer::mapping::SubmapId{-1, -1}) {
        const auto& submap_data_j = submap_data_getter(submap_hint);
        if (submap_data_j.submap == nullptr) {
          submap_hint = {-1, -1};
        } else {
          if (Submap(submap_hint, submap_data_j).is_known(global_position))
            ok = false;
        }
        if (ok) submap_hint = {-1, -1};
      }

      if (ok)
        for (const auto& active_submap : active_submaps_) {
          const cartographer::mapping::SubmapId& id_j = active_submap;
          if (id_j == s_i.id) continue;
          if (!bg::intersects(
                  bounding_box.last_global_box,
                  bounding_boxes_.at(active_submap).last_global_box))
            continue;

          const auto submap_data_j = submap_data_getter(id_j);
          if (submap_data_j.submap == nullptr) {
            continue;
          }
          const Submap s_j(id_j, submap_data_j);

          if (s_j.is_known(global_position)) {
            ok = false;
            submap_hint = s_j.id;
          }
        }

      if (ok) {
        for (const auto& intersecting_submap : intersecting_submaps) {
          const cartographer::mapping::SubmapId& id_j =
              intersecting_submap.second;
          if (id_j == s_i.id) continue;

          const auto submap_data_j = submap_data_getter(id_j);
          if (submap_data_j.submap == nullptr) {
            rt_.remove(
                std::make_pair(bounding_boxes_.at(id_j).last_global_box, id_j));
            continue;
          }
          const Submap s_j(id_j, submap_data_j);

          if (s_j.is_known(global_position)) {
            ok = false;
            submap_hint = s_j.id;
          }
        }
      }

      if (ok) {
        submap_hint = {-1, -1};
        geometry_msgs::Point frontier_point;
        frontier_point.x = global_position.x();
        frontier_point.y = global_position.y();
        frontier_point.z = global_position.z();
        frontier_marker.points.push_back(frontier_point);
      }
    }
    return frontier_marker;
  }

  Box CalculateBoundingBox(const Submap& submap) {
    auto& bounding_box_info = bounding_boxes_[submap.id];
    const Eigen::Vector3d p1_global =
        submap.pose * bounding_box_info.local_box.first;
    const Eigen::Vector3d p2_global =
        submap.pose * bounding_box_info.local_box.second;

    return Box{Point(std::min(p1_global.x(), p2_global.x()),
                     std::min(p1_global.y(), p2_global.y())),
               Point(std::max(p1_global.x(), p2_global.x()),
                     std::max(p1_global.y(), p2_global.y()))};
  }
};

}  // namespace frontier

#endif