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

class Detector {
 public:
  Detector(cartographer::mapping::PoseGraph2D* pose_graph);

  void InitPublisher();

  // Performs local frontier edge detection.
  void HandleSubmapUpdates(
      const std::vector<cartographer::mapping::SubmapId>& submap_ids,
      bool handling_deferred = false);

  void PublishAllSubmaps(const cartographer::mapping::MapById<
                         cartographer::mapping::SubmapId,
                         cartographer::mapping::PoseGraphInterface::SubmapData>&
                             all_submap_data);
  void PublishSubmaps(
      const std::vector<cartographer::mapping::SubmapId>& submap_ids);

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

  std::map<
      cartographer::mapping::SubmapId,
      std::vector<std::pair<Eigen::Vector3d,
                            cartographer::mapping::SubmapId /* submap_hint */>>>
      submap_frontier_cells_;
  std::vector<cartographer::mapping::SubmapId> deferred_updates_;

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
          grid(*submap.grid()),
          limits(grid.limits()),
          pose(submap_data.pose) /* * submap.local_pose().inverse() *
              cartographer::transform::Rigid3d::Translation(
                  (Eigen::Vector3d() << limits.max(), 0.).finished())) */
          {};

    const cartographer::mapping::SubmapId id;
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
  // Goes through previously edge-detected local frontier points in submaps,
  // checks if they really are frontier points by looking in other submaps,
  // and creates a marker containing the appropriate frontier points.
  visualization_msgs::Marker CreateMarkerForSubmap(
      const cartographer::mapping::SubmapId& id_i, const T& submap_data_getter);

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

  void RebuildTree(const cartographer::mapping::MapById<
                   cartographer::mapping::SubmapId,
                   cartographer::mapping::PoseGraphInterface::SubmapData>&
                       all_submap_data);
};

}  // namespace frontier

#endif