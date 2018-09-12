#include <absl/synchronization/mutex.h>
#include <cartographer/mapping/2d/map_limits.h>
#include <cartographer_ros/frontier_detection.h>
#include <cartographer_ros/msg_conversion.h>

namespace frontier {

Detector::Detector(cartographer::mapping::PoseGraph2D* const pose_graph)
    : publisher_initialized_(false),
      last_optimizations_performed_(-1),
      pose_graph_(pose_graph),
      submaps_(pose_graph_) {}

void Detector::InitPublisher() {
  frontier_publisher_ =
      ros::NodeHandle().advertise<visualization_msgs::MarkerArray>(
          "frontier_marker", 3, true);
  publisher_initialized_ = true;
}

void Detector::PublishAllSubmaps() {
  if (!publisher_initialized_) InitPublisher();
  // LOG(ERROR) << "publishing all submaps";
  visualization_msgs::MarkerArray frontier_markers;

  std::unique_lock<std::mutex> lock(mutex_);

  for (const auto& submap_data_i : submaps_.last_all_submap_data())
    frontier_markers.markers.push_back(CreateMarkerForSubmap(submap_data_i.id));

  frontier_publisher_.publish(frontier_markers);
}

void Detector::PublishSubmaps(
    const std::vector<cartographer::mapping::SubmapId>& submap_ids) {
  if (!publisher_initialized_) InitPublisher();

  visualization_msgs::MarkerArray frontier_markers;

  for (const auto& id_i : submap_ids) {
    // LOG(ERROR) << "publishing submap " << id_i.submap_index;
    frontier_markers.markers.push_back(CreateMarkerForSubmap(id_i));
  }

  frontier_publisher_.publish(frontier_markers);
}

bool Detector::CheckForOptimizationEvent() {
  int actual_optimizations_performed = pose_graph_->optimizations_performed();
  if (actual_optimizations_performed != last_optimizations_performed_) {
    last_optimizations_performed_ = actual_optimizations_performed;
  } else
    return false;
  submaps_.Invalidate();
  RebuildTree();
  PublishAllSubmaps();
  return true;
}

visualization_msgs::Marker Detector::CreateMarkerForSubmap(
    const cartographer::mapping::SubmapId& id_i) {
  Submap& s_i(submaps_(id_i));

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

  // TODO vectorize
  for (auto& frontier_cell : submap_frontier_cells) {
    const auto global_position = s_i.pose * frontier_cell.first;

    bool ok = true;

    auto& submap_hint = frontier_cell.second;

    if (submap_hint != cartographer::mapping::SubmapId{-1, -1}) {
      Submap* s_j = submaps_.IfExists(submap_hint);
      if (s_j == nullptr) {
        submap_hint = {-1, -1};
      } else {
        if (s_j->is_known(s_j->to_local_submap_position * global_position))
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

        Submap* s_j = submaps_.IfExists(id_j);
        if (s_j == nullptr) {
          continue;
        }
        if (s_j->is_known(s_j->to_local_submap_position * global_position)) {
          ok = false;
          submap_hint = s_j->id;
        }
      }

    if (ok) {
      for (const auto& intersecting_submap : intersecting_submaps) {
        const cartographer::mapping::SubmapId& id_j =
            intersecting_submap.second;
        if (id_j == s_i.id) continue;

        Submap* s_j = submaps_.IfExists(id_j);
        if (s_j == nullptr) {
          rt_.remove(
              std::make_pair(bounding_boxes_.at(id_j).last_global_box, id_j));
          continue;
        }

        if (s_j->is_known(s_j->to_local_submap_position * global_position)) {
          ok = false;
          submap_hint = id_j;
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
    const std::vector<cartographer::mapping::SubmapId>& submap_ids) {
  std::vector<cartographer::mapping::SubmapId> submaps_to_publish;

  std::vector<Submap*> previous_submaps_to_cleanup;

  const auto process_submap = [this, &previous_submaps_to_cleanup,
                               &submaps_to_publish](
                                  const cartographer::mapping::SubmapId& id_i,
                                  const bool cleanup_of_previous) -> void {
    const Submap& s_i(submaps_(id_i));
    // std::unique_lock<std::mutex> lock(mutex_);
    auto& submap_frontier_cells = submap_frontier_cells_[id_i];

    // TODO unnecessary if cleanup_of_previous
    submap_frontier_cells.clear();

    Eigen::Array2i offset;
    cartographer::mapping::CellLimits cell_limits;
    s_i.grid().ComputeCroppedLimits(&offset, &cell_limits);
    const auto local_pose_inverse = s_i.submap.local_pose().inverse();

    std::vector<Submap> neighbours;

    const cartographer::mapping::SubmapId id_prev(
        {id_i.trajectory_id, id_i.submap_index - 1});
    auto* submap_prev = submaps_.IfExists(id_prev);
    if (submap_prev != nullptr) {
      neighbours.emplace_back(*submap_prev);
    }

    if (cleanup_of_previous) {
      for (int i = 1; i <= 2; i++) {
        const cartographer::mapping::SubmapId id_next(
            {id_i.trajectory_id, id_i.submap_index + i});
        auto* submap_next = submaps_.IfExists(id_next);
        if (submap_next != nullptr) {
          neighbours.emplace_back(*submap_next);
        }
      }
    }

    const int full_x_dim = s_i.limits().cell_limits().num_x_cells;
    const int full_y_dim = s_i.limits().cell_limits().num_y_cells;

    using DynamicArray = Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic>;
    Eigen::Map<const DynamicArray> full_correspondence_costs(
        s_i.grid().correspondence_cost_cells().data(), full_x_dim, full_y_dim);

    const int x_dim = cell_limits.num_x_cells;
    const int y_dim = cell_limits.num_y_cells;

    const auto correspondence_costs =
        full_correspondence_costs.block(offset.x(), offset.y(), x_dim, y_dim);

    DynamicArray free_cells(
        (correspondence_costs >= kFreeProbabilityValue).cast<uint16_t>());
    DynamicArray unknown_cells(
        ((correspondence_costs == 0) +
         (correspondence_costs > kOccupiedProbabilityValue) *
             (correspondence_costs < kFreeProbabilityValue))
            .cast<uint16_t>());

    DynamicArray free_neighbours(DynamicArray::Zero(x_dim, y_dim));

    free_neighbours.leftCols(y_dim - 1) += free_cells.rightCols(y_dim - 1);
    free_neighbours.topLeftCorner(x_dim - 1, y_dim - 1) +=
        free_cells.bottomRightCorner(x_dim - 1, y_dim - 1);
    free_neighbours.topRows(x_dim - 1) += free_cells.bottomRows(x_dim - 1);

    free_neighbours.rightCols(y_dim - 1) += free_cells.leftCols(y_dim - 1);
    free_neighbours.bottomRightCorner(x_dim - 1, y_dim - 1) +=
        free_cells.topLeftCorner(x_dim - 1, y_dim - 1);
    free_neighbours.bottomRows(x_dim - 1) += free_cells.topRows(x_dim - 1);

    free_neighbours.bottomLeftCorner(x_dim - 1, y_dim - 1) +=
        free_cells.topRightCorner(x_dim - 1, y_dim - 1);
    free_neighbours.topRightCorner(x_dim - 1, y_dim - 1) +=
        free_cells.bottomLeftCorner(x_dim - 1, y_dim - 1);

    DynamicArray unknown_neighbours(DynamicArray::Zero(x_dim, y_dim));

    unknown_neighbours.leftCols(y_dim - 1) +=
        unknown_cells.rightCols(y_dim - 1);
    unknown_neighbours.topLeftCorner(x_dim - 1, y_dim - 1) +=
        unknown_cells.bottomRightCorner(x_dim - 1, y_dim - 1);
    unknown_neighbours.topRows(x_dim - 1) +=
        unknown_cells.bottomRows(x_dim - 1);

    unknown_neighbours.rightCols(y_dim - 1) +=
        unknown_cells.leftCols(y_dim - 1);
    unknown_neighbours.bottomRightCorner(x_dim - 1, y_dim - 1) +=
        unknown_cells.topLeftCorner(x_dim - 1, y_dim - 1);
    unknown_neighbours.bottomRows(x_dim - 1) +=
        unknown_cells.topRows(x_dim - 1);

    unknown_neighbours.bottomLeftCorner(x_dim - 1, y_dim - 1) +=
        unknown_cells.topRightCorner(x_dim - 1, y_dim - 1);
    unknown_neighbours.topRightCorner(x_dim - 1, y_dim - 1) +=
        unknown_cells.bottomLeftCorner(x_dim - 1, y_dim - 1);

    DynamicArray frontier(unknown_cells *
                          (unknown_neighbours >= 3u).cast<uint16_t>() *
                          (free_neighbours >= 3u).cast<uint16_t>());

    for (int y = 0; y < y_dim; y++)
      for (int x = 0; x < x_dim; x++) {
        if (frontier(x, y)) {
          const Eigen::Array2i xy_index(Eigen::Array2i{x, y} + offset);
          const Eigen::Vector3d position_in_local =
              (Eigen::Vector3d()
                   << s_i.limits().GetCellCenter(xy_index).cast<double>(),
               0)
                  .finished();
          bool ok = true;
          for (const auto& neighbour : neighbours) {
            if (neighbour.is_known(position_in_local)) ok = false;
          }
          if (ok)
            submap_frontier_cells.push_back(
                std::make_pair(local_pose_inverse * position_in_local,
                               cartographer::mapping::SubmapId{-1, -1}));
        }
      }

    const double max_x =
        s_i.limits().max().x() - s_i.limits().resolution() * offset.y();
    const double max_y =
        s_i.limits().max().y() - s_i.limits().resolution() * offset.x();
    const double min_x =
        s_i.limits().max().x() -
        s_i.limits().resolution() * (offset.y() + cell_limits.num_y_cells);
    const double min_y =
        s_i.limits().max().y() -
        s_i.limits().resolution() * (offset.x() + cell_limits.num_x_cells);

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
    if (s_i.submap.insertion_finished() && !cleanup_of_previous) {
      for (int i = 1; i <= 2; i++) {
        const cartographer::mapping::SubmapId id_prev(
            {id_i.trajectory_id, id_i.submap_index - i});
        auto* submap_prev = submaps_.IfExists(id_prev);
        if (submap_prev != nullptr) {
          previous_submaps_to_cleanup.push_back(submap_prev);
        }
      }
      // LOG(WARNING) << "Removing from active submaps: " << s_i.id;
      if (iter != active_submaps_.end()) active_submaps_.erase(iter);
      rt_.insert(std::make_pair(bounding_box_info.last_global_box, s_i.id));
    } else {
      if (iter == active_submaps_.end()) {
        active_submaps_.push_back(s_i.id);
      }
    }

    submaps_to_publish.push_back(id_i);
    const auto intersecting_submaps = GetIntersectingFinishedSubmaps(id_i);
    for (const auto& intersecting_submap : intersecting_submaps) {
      if (std::find(submaps_to_publish.begin(), submaps_to_publish.end(),
                    intersecting_submap) == submaps_to_publish.end())
        submaps_to_publish.push_back(intersecting_submap);
    }
  };

  for (const auto& id_i : submap_ids) {
    process_submap(id_i, false /* cleanup_of_previous */);
  }

  for (const auto& submap_to_cleanup : previous_submaps_to_cleanup) {
    process_submap(submap_to_cleanup->id, true /* cleanup_of_previous */);
  }

  if (!CheckForOptimizationEvent()) {
    PublishSubmaps(submaps_to_publish);
  }
  // if (publish_) publishUpdatedFrontiers();
}

void Detector::RebuildTree() {
  std::unique_lock<std::mutex> lock(mutex_);
  std::vector<Value> rectangles;

  for (const auto& submap_data : submaps_.last_all_submap_data()) {
    const auto bounding_box_iter = bounding_boxes_.find(submap_data.id);
    if (bounding_box_iter == bounding_boxes_.end()) {
      LOG(ERROR) << "Bounding box missing, should not happen.";
      continue;
    }
    auto& bounding_box_info = bounding_box_iter->second;
    const Submap& s_i(submaps_(submap_data.id));
    bounding_box_info.last_global_box = CalculateBoundingBox(s_i);

    if (std::find(active_submaps_.begin(), active_submaps_.end(), s_i.id) ==
        active_submaps_.end()) {
      rectangles.emplace_back(
          std::make_pair(bounding_box_info.last_global_box, s_i.id));
    }
  }

  // Invokes rtree's packing constructor.
  rt_ = RTree{std::move(rectangles)};
}

}  // namespace frontier
