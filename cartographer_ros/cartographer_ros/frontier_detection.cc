#include <absl/synchronization/mutex.h>
#include <cartographer/mapping/2d/map_limits.h>
#include <cartographer_ros/frontier_detection.h>
#include <cartographer_ros/msg_conversion.h>

namespace frontier {

Detector::Detector(const bool publish)
    : last_optimization_epoch_(-1), publish_(publish) {
  if (publish_)
    frontier_publisher_ =
        ros::NodeHandle().advertise<visualization_msgs::Marker>(
            "frontier_marker", 1, true);
}

void Detector::handleNewSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr& submap_list) {
  // last_submap_list_ = submap_list;
  std::unique_lock<std::mutex> lock(mutex_);
  submap_poses_.clear();

  for (const auto& entry : submap_list->submap) {
    const cartographer::mapping::SubmapId submap_id{entry.trajectory_id,
                                                    entry.submap_index};
    submap_poses_.emplace(
        std::piecewise_construct, std::forward_as_tuple(submap_id),
        std::forward_as_tuple(std::make_pair(
            entry.submap_version, cartographer_ros::ToRigid3d(entry.pose))));
  }
  if (submap_list->optimizations_performed != last_optimization_epoch_) {
    last_optimization_epoch_ = submap_list->optimizations_performed;
    lock.unlock();
    if (publish_) publishUpdatedFrontiers();
  }
}

namespace {

inline Eigen::Array2i GetCellIndex(
    const cartographer::transform::Rigid3d& submap_pose,
    const cartographer::transform::Rigid3d& global,
    const cartographer::io::SubmapTexture& texture) {
  // Index values are row major and the top left has Eigen::Array2i::Zero()
  // and contains (centered_max_x, centered_max_y). We need to flip and
  // rotate.
  const auto point_local =
      (texture.slice_pose.inverse() * submap_pose.inverse() * global)
          .translation();
  return Eigen::Array2i(cartographer::common::RoundToInt(
                            (-point_local.x() / texture.resolution - 0.5)),
                        cartographer::common::RoundToInt(
                            (-point_local.y() / texture.resolution - 0.5)));
}
inline cartographer::transform::Rigid3d GetGlobal(
    const cartographer::transform::Rigid3d submap_pose,
    const Eigen::Array2i& cell_index,
    const cartographer::io::SubmapTexture& texture) {
  // Index values are row major and the top left has Eigen::Array2i::Zero()
  // and contains (centered_max_x, centered_max_y). We need to flip and
  // rotate.
  return submap_pose * texture.slice_pose *
         cartographer::transform::Rigid3d::Translation(
             {-(cell_index[0] + 0.5) * texture.resolution,
              -(cell_index[1] + 0.5) * texture.resolution, 0});
}

inline Eigen::Array2i FromFlatIndex(
    const int flat_index, const cartographer::io::SubmapTexture& texture) {
  return {flat_index / texture.width, flat_index % texture.width};
}

inline int ToFlatIndex(const Eigen::Array2i& cell_index,
                       const cartographer::io::SubmapTexture& texture) {
  return cell_index[0] * texture.width + cell_index[1];
}

bool Contains(const Eigen::Array2i& cell_index,
              const cartographer::io::SubmapTexture& texture) {
  return (Eigen::Array2i(0, 0) <= cell_index).all() &&
         (cell_index < Eigen::Array2i(texture.height, texture.width)).all();
}

}  // namespace

void Detector::publishUpdatedFrontiers() {
  visualization_msgs::Marker frontier_marker;
  frontier_marker.header.frame_id = "map";
  frontier_marker.pose.orientation.w = 1.0;
  frontier_marker.type = visualization_msgs::Marker::POINTS;
  frontier_marker.scale.x = 0.05;
  frontier_marker.scale.y = 0.05;
  frontier_marker.color.r = 1.0;
  frontier_marker.color.a = 1.0;

  int i = 0;
  std::unique_lock<std::mutex> lock(mutex_);
  for (const auto& submap_i : submap_poses_) {
    i++;
    // if (i==2)
    // break;
    const auto& id_i = submap_i.first;
    const int submap_i_version = submap_i.second.first;
    if (submap_textures_.count(id_i) == 0 ||
        submap_i_version != submap_textures_.at(id_i)->version) {
      return;  // let's wait until we fetch that submap
    }
    if (!submap_textures_.count(id_i)) return;
    auto& frontier_cells = submap_frontier_cells_.at(id_i);
    const auto& submap_i_texture = submap_textures_.at(id_i)->textures.at(1);
    const auto& submap_i_pose = submap_i.second.second;

    /*
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.a = 1.0;
    geometry_msgs::Point frontier_point;
    frontier_point.x = submap_i_pose.translation().x();
    frontier_point.y = submap_i_pose.translation().y();
    frontier_point.z = submap_i_pose.translation().z();
    frontier_marker.points.push_back(frontier_point);
    frontier_marker.colors.push_back(color);

    color.g = 1.0;
    cartographer::transform::Rigid3d pose = submap_i_pose *
    submap_i_texture.slice_pose *
        cartographer::transform::Rigid3d::Translation({
                                                            -0.5 *
    submap_i_texture.resolution,
                                                            -(submap_i_texture.width
    - 1 + 0.5) * submap_i_texture.resolution, 0}); frontier_point.x =
    pose.translation().x(); frontier_point.y = pose.translation().y();
    frontier_point.z = pose.translation().z();
    frontier_marker.points.push_back(frontier_point);
    frontier_marker.colors.push_back(color);
    continue;*/

    for (auto& frontier_cell : frontier_cells) {
      const auto global =
          GetGlobal(submap_i_pose, frontier_cell.first, submap_i_texture);
      auto& submap_hint = frontier_cell.second;

      bool ok = true;

      const auto validate_submap =
          [&](const cartographer::mapping::SubmapId& id_j) {
            if (!submap_textures_.count(id_j)) return;
            const auto& submap_j_texture =
                submap_textures_.at(id_j)->textures.at(1);
            const auto& submap_j_pose = submap_poses_.at(id_j).second;

            const auto cell_index =
                GetCellIndex(submap_j_pose, global, submap_j_texture);
            const int dx[]{-1, -1, -1, 0, 0, 0, 1, 1, 1};
            const int dy[]{-1, 0, -1, -1, 0, 1, -1, 0, 1};
            for (int i = 4; i < 5; i++) {
              const Eigen::Array2i cell_index_d{cell_index[0] + dx[i],
                                                cell_index[1] + dy[i]};
              const auto flat_index = ToFlatIndex(cell_index, submap_j_texture);
              if (Contains(cell_index, submap_j_texture)) {
                unsigned char intensity = static_cast<unsigned char>(
                    submap_j_texture.pixels.intensity.at(flat_index));
                unsigned char alpha = static_cast<unsigned char>(
                    submap_j_texture.pixels.alpha.at(flat_index));
                if (intensity == 255 || alpha == 255) ok = false;
              }
            }
          };

      if (submap_hint != nullptr) {
        if (submap_poses_.count(*submap_hint))
          validate_submap(*submap_hint);
        else
          submap_hint = nullptr;
      }

      if (ok)
        for (const auto& submap_j : submap_poses_) {
          if (id_i == submap_j.first) continue;
          validate_submap(submap_j.first);
          if (!ok) {
            submap_hint = absl::make_unique<cartographer::mapping::SubmapId>(
                submap_j.first);
            break;
          }
        }
      if (ok) {
        geometry_msgs::Point frontier_point;
        frontier_point.x = global.translation().x();
        frontier_point.y = global.translation().y();
        frontier_point.z = global.translation().z();
        frontier_marker.points.push_back(frontier_point);
      }
    }
  }

  frontier_publisher_.publish(frontier_marker);
}

void Detector::handleNewSubmapTexture(
    const cartographer::mapping::SubmapId& id,
    const std::shared_ptr<cartographer::io::SubmapTextures>& new_texture) {
  auto texture_filtered =
      cartographer::io::SubmapTexture(new_texture->textures.at(0));
  auto frontier_texture =
      cartographer::io::SubmapTexture(new_texture->textures.at(0));

  for (int i = 0; i < texture_filtered.pixels.intensity.size(); i++) {
    char& intensity = texture_filtered.pixels.intensity.at(i);
    char& alpha = texture_filtered.pixels.alpha.at(i);
    // unsigned char add = std::min((unsigned int)255, (unsigned
    // int)(submap_texture.pixels.intensity[i]) + (unsigned
    // int)(submap_texture.pixels.alpha[i]));
    if (alpha > 0) {
      alpha = 255;
    }
    if (intensity > 0) {
      if (intensity < 15)
        intensity = 0;
      else
        intensity = 255;
    }
  }
  const int w = texture_filtered.width;
  const int h = texture_filtered.height;
  auto is_unknown = [&texture_filtered](int index) {
    return texture_filtered.pixels.intensity.at(index) == 0 &&
           texture_filtered.pixels.alpha.at(index) == 0;
  };
  auto is_free = [&texture_filtered](int index) {
    return texture_filtered.pixels.intensity.at(index) == (char)255;
  };
  auto is_in_limits = [&texture_filtered](int index) {
    return (index >= 0) && (index < texture_filtered.pixels.intensity.size());
  };
  std::vector<std::pair<Eigen::Array2i,
                        std::unique_ptr<cartographer::mapping::SubmapId>>>
      submap_frontier_cells;
  for (int i = 0; i < texture_filtered.pixels.intensity.size(); i++) {
    frontier_texture.pixels.intensity.at(i) = 0;
    frontier_texture.pixels.alpha.at(i) = 0;
    if (is_unknown(i)) {
      int free_neighbours = 0;
      int unknown_neighbours = 0;
      auto check_neighbour = [&](int index) {
        if (is_in_limits(index) && is_unknown(index)) unknown_neighbours++;
        if (is_in_limits(index) && is_free(index)) free_neighbours++;
      };
      check_neighbour(i + 1);
      check_neighbour(i - 1);
      check_neighbour(i + w);
      check_neighbour(i + w + 1);
      check_neighbour(i + w - 1);
      check_neighbour(i - w);
      check_neighbour(i - w + 1);
      check_neighbour(i - w - 1);
      if (free_neighbours >= 3 && unknown_neighbours >= 3) {
        frontier_texture.pixels.intensity.at(i) = 255;
        frontier_texture.pixels.alpha.at(i) = 255;
        submap_frontier_cells.push_back(
            std::make_pair(FromFlatIndex(i, frontier_texture), nullptr));
      }
    }
  }

  {
    std::unique_lock<std::mutex> lock(mutex_);
    submap_frontier_cells_[id] = std::move(submap_frontier_cells);
    submap_textures_[id] = new_texture;
    new_texture->textures.push_back(texture_filtered);
    new_texture->textures.push_back(frontier_texture);
  }

  if (publish_) publishUpdatedFrontiers();
}

}  // namespace frontier