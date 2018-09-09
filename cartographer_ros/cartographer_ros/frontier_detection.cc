#include <absl/synchronization/mutex.h>
#include <cartographer/mapping/2d/map_limits.h>
#include <cartographer_ros/frontier_detection.h>
#include <cartographer_ros/msg_conversion.h>

namespace frontier {

Detector::Detector(cartographer::mapping::PoseGraph2D* const pose_graph)
    : publisher_initialized_(false),
      last_optimizations_performed_(-1),
      pose_graph_(pose_graph) {}

void Detector::InitPublisher() {
  frontier_publisher_ =
      ros::NodeHandle().advertise<visualization_msgs::MarkerArray>(
          "frontier_marker", 3, true);
  publisher_initialized_ = true;
}

class SubmapDataCache {
 public:
  SubmapDataCache(const cartographer::mapping::PoseGraph2D* const pose_graph)
      : pose_graph_(pose_graph) {}
  cartographer::mapping::PoseGraphInterface::SubmapData& operator()(
      const cartographer::mapping::SubmapId& id) const {
    const auto result = std::find_if(
        submaps_data_.begin(), submaps_data_.end(),
        [&](const DataPair& data_pair) { return data_pair.first == id; });
    if (result != submaps_data_.end())
      return result->second;
    else {
      submaps_data_.push_back({id, pose_graph_->GetSubmapData(id)});
      return submaps_data_.back().second;
    }
  }

 private:
  using DataPair =
      std::pair<cartographer::mapping::SubmapId,
                cartographer::mapping::PoseGraphInterface::SubmapData>;
  mutable std::vector<DataPair> submaps_data_;
  const cartographer::mapping::PoseGraph2D* const pose_graph_;
};

void Detector::PublishAllSubmaps(
    const cartographer::mapping::MapById<
        cartographer::mapping::SubmapId,
        cartographer::mapping::PoseGraphInterface::SubmapData>&
        all_submap_data) {
  if (!publisher_initialized_) InitPublisher();
  // LOG(ERROR) << "publishing all submaps";
  visualization_msgs::MarkerArray frontier_markers;

  std::unique_lock<std::mutex> lock(mutex_);
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
  if (!publisher_initialized_) InitPublisher();

  visualization_msgs::MarkerArray frontier_markers;

  SubmapDataCache cache(pose_graph_);

  for (const auto& id_i : submap_ids) {
    // LOG(ERROR) << "publishing submap " << id_i.submap_index;
    frontier_markers.markers.push_back(CreateMarkerForSubmap(id_i, cache));
  }

  frontier_publisher_.publish(frontier_markers);
}

bool Detector::CheckForOptimizationEvent() {
  int actual_optimizations_performed = pose_graph_->optimizations_performed();
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (actual_optimizations_performed != last_optimizations_performed_) {
      last_optimizations_performed_ = actual_optimizations_performed;
    } else
      return false;
  }
  if (deferred_updates_.size()) {
    LOG(INFO) << "deferred updates size: " << deferred_updates_.size();
    HandleSubmapUpdates(std::move(deferred_updates_),
                        true /* handling_deferred */);
  }

  const auto all_submap_data = pose_graph_->GetAllSubmapData();
  RebuildTree(all_submap_data);
  PublishAllSubmaps(all_submap_data);
  return true;
}

template <class T>
visualization_msgs::Marker Detector::CreateMarkerForSubmap(
    const cartographer::mapping::SubmapId& id_i, const T& submap_data_getter) {
  Submap s_i(id_i, submap_data_getter(id_i));

  visualization_msgs::Marker frontier_marker;
  frontier_marker.header.frame_id = "map";
  frontier_marker.pose.orientation.w = 1.0;
  frontier_marker.type = visualization_msgs::Marker::POINTS;
  frontier_marker.scale.x = 0.1;
  frontier_marker.scale.y = 0.1;
  frontier_marker.color.r = 1.0;
  frontier_marker.color.a = 1.0;
  std::ostringstream ss;
  ss << "Trajectory " << s_i.id.trajectory_id << ", submap "
     << s_i.id.submap_index;
  frontier_marker.ns = ss.str();

  auto& submap_frontier_cells = submap_frontier_cells_.at(s_i.id);
  auto& bounding_box = bounding_boxes_.at(s_i.id);

  std::vector<Value> intersecting_submaps;
  rt_.query(bgi::intersects(bounding_box.last_global_box),
            std::back_inserter(intersecting_submaps));

  /*LOG(WARNING) << "Publishing submap " << id_i;
  { std::ostringstream ss;
    ss << "Active submaps: ";
    for (const auto &active_submap : active_submaps_) {
      ss << active_submap << " ";
    }
    LOG(WARNING) << ss.str();
  }
  { std::ostringstream ss;
  ss << "Intersecting submaps: ";
    for (const auto &intersecting_submap : intersecting_submaps) {
      ss << intersecting_submap.second << " ";
    }
    LOG(WARNING) << ss.str();
  }*/

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
        if (!bg::intersects(bounding_box.last_global_box,
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

void Detector::HandleSubmapUpdates(
    const std::vector<cartographer::mapping::SubmapId>& submap_ids,
    const bool handling_deferred) {
  std::vector<cartographer::mapping::SubmapId> submaps_to_update;
  SubmapDataCache cache(pose_graph_);
  std::vector<Submap> submaps;

  std::vector<cartographer::mapping::SubmapId> submaps_to_publish;
  for (const auto& id_i : submap_ids) {
    Submap submap(id_i, cache(id_i));
    const bool more_than_two = submap_ids.size() > 1;
    const bool second = more_than_two && id_i == submap_ids.at(1);
    const bool unfinished = !submap.submap.insertion_finished();

    const auto deferred_iter =
        std::find(deferred_updates_.begin(), deferred_updates_.end(), id_i);
    const bool in_deferred = deferred_iter != deferred_updates_.end();
    const bool in_active_submaps =
        std::find(active_submaps_.begin(), active_submaps_.end(), id_i) !=
        active_submaps_.end();
    if (true || !(more_than_two && second && unfinished && in_active_submaps) ||
        handling_deferred) {
      submaps_to_update.push_back(id_i);
      submaps.push_back(std::move(submap));
      if (in_deferred) {
        deferred_updates_.erase(deferred_iter);
        // LOG(WARNING) << "Undeferring " << id_i << " handling_deferred " <<
        // handling_deferred << " more_than_two " << more_than_two << " second "
        //<< second << " unfinished " << unfinished <<  " in_active_submaps " <<
        //in_active_submaps;
      }
      if (more_than_two && !unfinished && in_active_submaps) {
        // LOG(WARNING) << "Finalizing submap " << id_i <<
        //   ", in_deferred: " << in_deferred;
      }
    } else {
      submaps_to_publish.push_back(id_i);
      if (!in_deferred) {
        // LOG(WARNING) << "deferring submap " << id_i;
        deferred_updates_.push_back(id_i);
      }
    }
  }

  int i = 0;
  for (const auto& id_i : submaps_to_update) {
    const Submap& s_i = submaps.at(i++);
    // std::unique_lock<std::mutex> lock(mutex_);
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

    // Keep only finished submaps in the tree in order to avoid lots of
    // insertions and removals while the submaps are being built due to the
    // bounding box being expanded.
    const auto iter =
        std::find(active_submaps_.begin(), active_submaps_.end(), s_i.id);
    if (s_i.submap.insertion_finished()) {
      // LOG(WARNING) << "Removing from active submaps: " << s_i.id;
      if (iter != active_submaps_.end()) active_submaps_.erase(iter);
      rt_.insert(std::make_pair(bounding_box_info.last_global_box, s_i.id));
    } else {
      if (iter == active_submaps_.end()) {
        active_submaps_.push_back(s_i.id);
      }
    }

    if (!handling_deferred) {
      submaps_to_publish.push_back(id_i);
      const auto intersecting_submaps = GetIntersectingFinishedSubmaps(id_i);
      for (const auto& intersecting_submap : intersecting_submaps) {
        if (std::find(submaps_to_publish.begin(), submaps_to_publish.end(),
                      intersecting_submap) == submaps_to_publish.end())
          submaps_to_publish.push_back(intersecting_submap);
      }
    }
  }

  if (handling_deferred) return;

  if (!CheckForOptimizationEvent()) {
    PublishSubmaps(submaps_to_publish);
  }
  // if (publish_) publishUpdatedFrontiers();
}

void Detector::RebuildTree(
    const cartographer::mapping::MapById<
        cartographer::mapping::SubmapId,
        cartographer::mapping::PoseGraphInterface::SubmapData>&
        all_submap_data) {
  std::unique_lock<std::mutex> lock(mutex_);
  std::vector<Value> rectangles;

  for (const auto& submap_data : all_submap_data) {
    const auto bounding_box_iter = bounding_boxes_.find(submap_data.id);
    if (bounding_box_iter == bounding_boxes_.end()) {
      LOG(ERROR) << "Bounding box missing, should not happen.";
      continue;
    }
    auto& bounding_box_info = bounding_box_iter->second;
    const Submap s_i(submap_data.id, submap_data.data);
    bounding_box_info.last_global_box = CalculateBoundingBox(s_i);

    if (std::find(active_submaps_.begin(), active_submaps_.end(), s_i.id) ==
        active_submaps_.end()) {
      rectangles.emplace_back(
          std::piecewise_construct,
          std::forward_as_tuple(bounding_box_info.last_global_box),
          std::forward_as_tuple(s_i.id));
    }
  }

  // Invokes rtree's packing constructor.
  rt_ = RTree{std::move(rectangles)};
}

}  // namespace frontier
