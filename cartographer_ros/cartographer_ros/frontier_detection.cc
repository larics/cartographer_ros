#include <absl/synchronization/mutex.h>
#include <cartographer/mapping/2d/map_limits.h>
#include <cartographer_ros/frontier_detection.h>
#include <cartographer_ros/msg_conversion.h>
#include <visualization_msgs/MarkerArray.h>

namespace frontier {

Detector::Detector(cartographer::mapping::PoseGraph2D* const pose_graph, const bool publish)
    : last_optimization_epoch_(-1), publish_(publish), pose_graph_(pose_graph) {
  if (publish_) InitPublisher();
}

void Detector::InitPublisher() {
  frontier_publisher_ =
      ros::NodeHandle().advertise<visualization_msgs::MarkerArray>(
          "frontier_marker", 3, true);
}

void Detector::publishUpdatedFrontiers() {
  /*visualization_msgs::MarkerArray frontier_markers;

  int i = 0;
  std::unique_lock<std::mutex> lock(mutex_);
  const auto all_submap_data = pose_graph_->GetAllSubmapData();
  for (const auto& submap_data_i : all_submap_data) {
    Submap s_i(submap_data_i.id, submap_data_i.data);
    //if(s_i.id != cartographer::mapping::SubmapId{0, 2}) continue;

    visualization_msgs::Marker frontier_marker;
    frontier_marker.header.frame_id = "map";
    frontier_marker.pose.orientation.w = 1.0;
    frontier_marker.type = visualization_msgs::Marker::POINTS;
    frontier_marker.scale.x = 0.1;
    frontier_marker.scale.y = 0.1;
    frontier_marker.color.r = 1.0;
    frontier_marker.color.a = 1.0;
    frontier_marker.ns = std::to_string(s_i.id.submap_index);
    i++;

    auto& submap_frontier = submap_frontiers_.at(s_i.id);

    std::vector<Value> intersecting_submaps;
    rt_.query(bgi::intersects(submap_frontier.last_global_bounding_box), std::back_inserter(intersecting_submaps));

    for (auto& frontier_cell : submap_frontier.cells) {
      const auto global_position = s_i.pose * frontier_cell.position_in_submap;

      bool ok = true;
      auto& submap_hint = frontier_cell.submap_hint;

      if (submap_hint != cartographer::mapping::SubmapId{-1, -1}) {
        if (!all_submap_data.Contains(submap_hint)) {
          submap_hint = {-1, -1};
        } else {
          if (Submap(submap_hint, all_submap_data.at(submap_hint)).is_known(global_position)) ok = false;
        }
        if (ok) submap_hint = {-1, -1};
      }

      cartographer::mapping::SubmapId previous_submap_id{s_i.id.trajectory_id, s_i.id.submap_index - 1};
      cartographer::mapping::SubmapId next_submap_id{s_i.id.trajectory_id, s_i.id.submap_index + 1};
      if (ok) {
        const auto previous_submap = all_submap_data.find(previous_submap_id);
        if (previous_submap != all_submap_data.end())
          if (Submap(previous_submap_id, previous_submap->data).is_known(global_position)) {
            ok = false;
            submap_hint = previous_submap_id;
          }
      }
      if (ok) {
        const auto next_submap = all_submap_data.find(next_submap_id);
        if (next_submap != all_submap_data.end())
          if (Submap(next_submap_id, next_submap->data).is_known(global_position)) {
            ok = false;
            submap_hint = next_submap_id;
          }
      }

      if (ok) {
        for (const auto &intersecting_submap : intersecting_submaps) {
          const cartographer::mapping::SubmapId id_j = intersecting_submap.second;
          if (id_j == s_i.id || id_j == previous_submap_id || id_j == next_submap_id) continue;

          auto submap_data_j = all_submap_data.find(id_j);
          if (submap_data_j == all_submap_data.end()) {
            rt_.remove({{}, id_j});
            continue;
          }
          const Submap s_j(id_j, submap_data_j->data);

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
    frontier_markers.markers.push_back(frontier_marker);
  }

  frontier_publisher_.publish(frontier_markers);*/
}

void Detector::handleSubmapUpdate(const cartographer::mapping::SubmapId& id_i) {

  const auto submap_data_i = pose_graph_->GetSubmapData(id_i);

  const auto s_i = Submap(id_i, submap_data_i);

  std::unique_lock<std::mutex> lock(mutex_);
  auto& submap_frontier = submap_frontiers_[id_i];
  submap_frontier.cells.clear();

  Eigen::Array2i offset;
  cartographer::mapping::CellLimits cell_limits;
  s_i.grid.ComputeCroppedLimits(&offset, &cell_limits);

  const auto local_pose_inverse = s_i.submap.local_pose().inverse();

  /*const double max_x = s_i.limits.max().x() - s_i.limits.resolution() * offset.y();
  const double max_y = s_i.limits.max().y() - s_i.limits.resolution() * offset.x();
  const double min_x = max_x - s_i.limits.resolution() * cell_limits.num_y_cells;
  const double min_y = max_y - s_i.limits.resolution() * cell_limits.num_x_cells;

  const Eigen::Vector3d p1 {max_x, max_y, 0.};
  const Eigen::Vector3d p2 {min_x, min_y, 0.};*/


  for (const Eigen::Array2i& xy_index_without_offset : cartographer::mapping::XYIndexRangeIterator(cell_limits)) {
    const Eigen::Array2i& xy_index = xy_index_without_offset + offset;
    if (s_i.is_unknown(xy_index)) {
      int free_neighbours = 0;
      int unknown_neighbours = 0;
      auto check_neighbour = [&](const Eigen::Array2i& other_xy_index) {
        if (s_i.is_unknown(other_xy_index)) unknown_neighbours++;
        if (s_i.is_free(other_xy_index)) free_neighbours++;
      };
      check_neighbour(xy_index + Eigen::Array2i{ 0,  1});
      check_neighbour(xy_index + Eigen::Array2i{ 0, -1});
      check_neighbour(xy_index + Eigen::Array2i{ 1,  0});
      check_neighbour(xy_index + Eigen::Array2i{-1,  0});
      check_neighbour(xy_index + Eigen::Array2i{ 1,  1});
      check_neighbour(xy_index + Eigen::Array2i{ 1, -1});
      check_neighbour(xy_index + Eigen::Array2i{-1,  1});
      check_neighbour(xy_index + Eigen::Array2i{-1, -1});
      if (free_neighbours >= 3 && unknown_neighbours >= 3) {
        submap_frontier_cells.push_back(
            std::make_pair(local_pose_inverse * (Eigen::Vector3d() << s_i.limits.GetCellCenter(xy_index).cast<double>(), 0).finished(),
                cartographer::mapping::SubmapId{-1, -1}));
      }
    }
  }
  /*submap_frontier.local_bounding_box = std::make_pair(p1, p2);
  rt_.remove({submap_frontier.last_global_bounding_box, s_i.id});
  submap_frontier.last_global_bounding_box = CalculateBoundingBox(s_i);
  rt_.insert(std::make_pair(submap_frontier.last_global_bounding_box, s_i.id));*/

  /*submap_frontier_cells.push_back(
      std::make_pair(local_pose_inverse * Eigen::Vector3d(max_x, max_y, 0.), cartographer::mapping::SubmapId{-1, -1}));
  submap_frontier_cells.push_back(
      std::make_pair(local_pose_inverse * Eigen::Vector3d(min_x, min_y, 0.), cartographer::mapping::SubmapId{-1, -1}));
  submap_frontier_cells.push_back(
      std::make_pair(local_pose_inverse * Eigen::Vector3d(max_x, min_y, 0.), cartographer::mapping::SubmapId{-1, -1}));
  submap_frontier_cells.push_back(
      std::make_pair(local_pose_inverse * Eigen::Vector3d(min_x, max_y, 0.), cartographer::mapping::SubmapId{-1, -1}));*/

  //if (publish_) publishUpdatedFrontiers();
}

}  // namespace frontier