#include <absl/synchronization/mutex.h>
#include <cartographer/mapping/2d/map_limits.h>
#include <cartographer_ros/frontier_detection.h>
#include <cartographer_ros/msg_conversion.h>

namespace frontier {

Detector::Detector(cartographer::mapping::PoseGraph2D* const pose_graph,
                   const bool publish)
    : last_optimizations_performed_(-1),
      publish_(publish),
      pose_graph_(pose_graph) {
  if (publish_) InitPublisher();
}

void Detector::InitPublisher() {
  frontier_publisher_ =
      ros::NodeHandle().advertise<visualization_msgs::MarkerArray>(
          "frontier_marker", 3, true);
}

void Detector::PublishAllSubmaps() {
  LOG(ERROR) << "publishing all submaps";
  visualization_msgs::MarkerArray frontier_markers;

  std::unique_lock<std::mutex> lock(mutex_);
  const auto all_submap_data = pose_graph_->GetAllSubmapData();
  cartographer::mapping::PoseGraphInterface::SubmapData null_data{nullptr};

  const auto submap_data_getter = [&](const cartographer::mapping::SubmapId& id)
      -> const cartographer::mapping::PoseGraphInterface::SubmapData& {
    if (all_submap_data.Contains(id))
      return all_submap_data.at(id);
    else
      return null_data;
  };

  for (const auto& submap_data_i : all_submap_data)
    frontier_markers.markers.push_back(
        CreateMarkerForSubmap(submap_data_i.id, submap_data_getter));

  frontier_publisher_.publish(frontier_markers);
}

void Detector::PublishSubmaps(
    const std::vector<cartographer::mapping::SubmapId>& submap_ids) {
  visualization_msgs::MarkerArray frontier_markers;
  using DataPair =
      std::pair<cartographer::mapping::SubmapId,
                cartographer::mapping::PoseGraphInterface::SubmapData>;
  std::vector<DataPair> submaps_data;

  const auto submap_data_getter = [&](const cartographer::mapping::SubmapId& id)
      -> const cartographer::mapping::PoseGraphInterface::SubmapData& {
    const auto result = std::find_if(
        submaps_data.begin(), submaps_data.end(),
        [&](const DataPair& data_pair) { return data_pair.first == id; });
    if (result != submaps_data.end())
      return result->second;
    else {
      submaps_data.push_back({id, pose_graph_->GetSubmapData(id)});
      return submaps_data.back().second;
    }
  };

  for (const auto& id_i : submap_ids) {
    LOG(ERROR) << "publishing submap " << id_i.submap_index;
    frontier_markers.markers.push_back(
        CreateMarkerForSubmap(id_i, submap_data_getter));
  }

  frontier_publisher_.publish(frontier_markers);
}

std::vector<cartographer::mapping::SubmapId>
Detector::GetIntersectingFinishedSubmaps(
    const cartographer::mapping::SubmapId& id_i) {
  std::vector<Value> intersecting_submaps;
  rt_.query(bgi::intersects(bounding_boxes_.at(id_i).last_global_box),
            std::back_inserter(intersecting_submaps));
  std::vector<cartographer::mapping::SubmapId> result;
  result.reserve(intersecting_submaps.size());
  for (const auto& intersecting_submap : intersecting_submaps)
    result.push_back(intersecting_submap.second);
  return result;
}

void Detector::HandleSubmapUpdate(const cartographer::mapping::SubmapId& id_i) {
  const auto submap_data_i = pose_graph_->GetSubmapData(id_i);

  const auto s_i = Submap(id_i, submap_data_i);

  std::unique_lock<std::mutex> lock(mutex_);
  auto& submap_frontier_cells = submap_frontier_cells_[id_i];
  submap_frontier_cells.clear();

  Eigen::Array2i offset;
  cartographer::mapping::CellLimits cell_limits;
  s_i.grid.ComputeCroppedLimits(&offset, &cell_limits);

  const auto local_pose_inverse = s_i.submap.local_pose().inverse();

  for (const Eigen::Array2i& xy_index :
       cartographer::mapping::XYIndexRangeIterator(
           offset, Eigen::Array2i(cell_limits.num_x_cells - 1,
                                  cell_limits.num_y_cells - 1) +
                       offset)) {
    if (s_i.is_unknown(xy_index)) {
      int free_neighbours = 0;
      int unknown_neighbours = 0;
      auto check_neighbour = [&](const Eigen::Array2i& other_xy_index) {
        if (s_i.is_unknown(other_xy_index)) unknown_neighbours++;
        if (s_i.is_free(other_xy_index)) free_neighbours++;
      };
      check_neighbour(xy_index + Eigen::Array2i{0, 1});
      check_neighbour(xy_index + Eigen::Array2i{0, -1});
      check_neighbour(xy_index + Eigen::Array2i{1, 0});
      check_neighbour(xy_index + Eigen::Array2i{-1, 0});
      check_neighbour(xy_index + Eigen::Array2i{1, 1});
      check_neighbour(xy_index + Eigen::Array2i{1, -1});
      check_neighbour(xy_index + Eigen::Array2i{-1, 1});
      check_neighbour(xy_index + Eigen::Array2i{-1, -1});
      if (free_neighbours >= 3 && unknown_neighbours >= 3) {
        submap_frontier_cells.push_back(std::make_pair(
            local_pose_inverse *
                (Eigen::Vector3d()
                     << s_i.limits.GetCellCenter(xy_index).cast<double>(),
                 0)
                    .finished(),
            cartographer::mapping::SubmapId{-1, -1}));
      }
    }
  }

  const double max_x =
      s_i.limits.max().x() - s_i.limits.resolution() * offset.y();
  const double max_y =
      s_i.limits.max().y() - s_i.limits.resolution() * offset.x();
  const double min_x =
      s_i.limits.max().x() -
      s_i.limits.resolution() * (offset.y() + cell_limits.num_y_cells);
  const double min_y =
      s_i.limits.max().y() -
      s_i.limits.resolution() * (offset.x() + cell_limits.num_x_cells);

  const Eigen::Vector3d p1{max_x, max_y, 0.};
  const Eigen::Vector3d p2{min_x, min_y, 0.};
  auto& bounding_box_info = bounding_boxes_[s_i.id];
  bounding_box_info.local_box = std::make_pair(p1, p2);
  bounding_box_info.last_global_box = CalculateBoundingBox(s_i);

  if (s_i.submap.insertion_finished()) {
    active_submaps_.erase(
        std::find(active_submaps_.begin(), active_submaps_.end(), s_i.id));
    rt_.insert(std::make_pair(bounding_box_info.last_global_box, s_i.id));
  } else {
    if (std::find(active_submaps_.begin(), active_submaps_.end(), s_i.id) ==
        active_submaps_.end()) {
      active_submaps_.push_back(s_i.id);
    }
  }

  // if (publish_) publishUpdatedFrontiers();
}

}  // namespace frontier