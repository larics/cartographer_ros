#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_FRONTIER_DETECTION_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_FRONTIER_DETECTION_H

#include <cartographer/io/submap_painter.h>
#include <cartographer/mapping/id.h>
#include <cartographer/transform/rigid_transform.h>
#include <cartographer_ros_msgs/SubmapList.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <mutex>

namespace frontier {

class Detector {
 public:
  Detector(bool publish = true);

  void handleNewSubmapTexture(
      const cartographer::mapping::SubmapId& id,
      const std::shared_ptr<cartographer::io::SubmapTextures>&);

  void handleNewSubmapList(
      const cartographer_ros_msgs::SubmapList::ConstPtr& submap_list);

  void publishUpdatedFrontiers();

 private:
  std::mutex mutex_;
  std::map<cartographer::mapping::SubmapId,
           std::shared_ptr<cartographer::io::SubmapTextures>>
      submap_textures_;

  std::map<
      cartographer::mapping::SubmapId,
      std::vector<std::pair<Eigen::Array2i,
                            std::unique_ptr<cartographer::mapping::SubmapId>>>>
      submap_frontier_cells_;
  // cartographer_ros_msgs::SubmapList::ConstPtr last_submap_list_;
  std::map<cartographer::mapping::SubmapId,
           std::pair<int /* version */, cartographer::transform::Rigid3d>>
      submap_poses_;
  ros::Publisher frontier_publisher_;
  int last_optimization_epoch_;

  bool publish_;
};

}  // namespace frontier

#endif