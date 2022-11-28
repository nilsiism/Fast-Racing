#include "ros/ros.h"
#include "airsim_ros_wrapper.h"
#include <ros/spinner.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <math.h>
#include <random>
#include<geometry_msgs/Point32.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap_server/OctomapServer.h>
#define map_f 1
using namespace octomap;
using namespace octomap_msgs;
using namespace octomap_server;
ros::Publisher  airsim_map_pub;
sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;
//octree
OctomapServer* server_drone;
bool use_octree;

void saveMap(std::vector<Eigen::Vector3f> points, float resolution) {

  // Open the file for writing.
  std::ofstream file ("/home/nils/workspace_/projects_/supereight-2-srl/test/planner/forest-comp.ply");
  if (!file.is_open()) {
    std::cerr << "Unable to write file" << "\n";
    return;
  }

  std::stringstream ss_nodes_corners;
  std::stringstream ss_faces;
  int nodes_corners_count = 0;
  int faces_count  = 0;

  float g_max = 0.f;
  size_t counter(0);

  float x_min = std::numeric_limits<float>::max();
  float y_min = std::numeric_limits<float>::max();
  float z_min = std::numeric_limits<float>::max();

  for (const auto& point : points) {
    counter++;

    const float x = point.x();
    const float y = point.y();
    const float z = point.z();

    // Create octant
    const Eigen::Vector3f node_coord(x, y, z);
    const float node_size = resolution;

    if (x < x_min) {
      x_min = x;
    }

    if (y < y_min) {
      y_min = y;
    }

    if (z < z_min) {
      z_min = z;
    }

    Eigen::Vector3f node_corners[8];
    node_corners[0] = node_coord;
    node_corners[1] = node_coord + Eigen::Vector3f(node_size, 0, 0);
    node_corners[2] = node_coord + Eigen::Vector3f(0, node_size, 0);
    node_corners[3] = node_coord + Eigen::Vector3f(node_size, node_size, 0);
    node_corners[4] = node_coord + Eigen::Vector3f(0, 0, node_size);
    node_corners[5] = node_coord + Eigen::Vector3f(node_size, 0, node_size);
    node_corners[6] = node_coord + Eigen::Vector3f(0, node_size, node_size);
    node_corners[7] = node_coord + Eigen::Vector3f(node_size, node_size, node_size);

    for(int i = 0; i < 8; ++i) {
      ss_nodes_corners << node_corners[i].x() << " "
                       << node_corners[i].y() << " "
                       << node_corners[i].z() << " "
                       << 0.5f << " 0 0" << std::endl;
    }

    ss_faces << "4 " << nodes_corners_count     << " " << nodes_corners_count + 1
             << " "  << nodes_corners_count + 3 << " " << nodes_corners_count + 2 << std::endl;

    ss_faces << "4 " << nodes_corners_count + 1 << " " << nodes_corners_count + 5
             << " "  << nodes_corners_count + 7 << " " << nodes_corners_count + 3 << std::endl;

    ss_faces << "4 " << nodes_corners_count + 5 << " " << nodes_corners_count + 7
             << " "  << nodes_corners_count + 6 << " " << nodes_corners_count + 4 << std::endl;

    ss_faces << "4 " << nodes_corners_count     << " " << nodes_corners_count + 2
             << " "  << nodes_corners_count + 6 << " " << nodes_corners_count + 4 << std::endl;

    ss_faces << "4 " << nodes_corners_count     << " " << nodes_corners_count + 1
             << " "  << nodes_corners_count + 5 << " " << nodes_corners_count + 4 << std::endl;

    ss_faces << "4 " << nodes_corners_count + 2 << " " << nodes_corners_count + 3
             << " "  << nodes_corners_count + 7 << " " << nodes_corners_count + 6 << std::endl;

    nodes_corners_count += 8;
    faces_count  += 6;
  }
  std::cout << "FINISHED SAVING MAP - " << counter << std::endl;

  file << "ply" << std::endl;
  file << "format ascii 1.0" << std::endl;
  file << "comment octree structure" << std::endl;
  file << "element vertex " << nodes_corners_count <<  std::endl;
  file << "property float x" << std::endl;
  file << "property float y" << std::endl;
  file << "property float z" << std::endl;
  file << "property uchar red"   << std::endl;
  file << "property uchar green" << std::endl;
  file << "property uchar blue"  << std::endl;
  file << "element face " << faces_count << std::endl;
  file << "property list uchar int vertex_index" << std::endl;
  file << "end_header" << std::endl;
  file << ss_nodes_corners.str();
  file << ss_faces.str();

  file.close();

  // Open the file for writing.
  std::ofstream point_cloud_file ("/home/nils/workspace_/projects_/supereight-2-srl/test/planner/forest-comp.txt");
  if (!point_cloud_file.is_open()) {
    std::cerr << "Unable to write file" << "\n";
    return;
  }

  std::stringstream ss_nodes_centres;;

  std::cout << "x min = " << x_min << std::endl;
  std::cout << "y min = " << y_min << std::endl;
  std::cout << "z min = " << z_min << std::endl;

  for (const auto& point : points) {
    const float x = point.x();
    const float y = point.y();
    const float z = point.z();

    // Create octant
    const Eigen::Vector3f node_centre = Eigen::Vector3f(x, y, z) + Eigen::Vector3f::Constant(10 + 0.5f) - Eigen::Vector3f(x_min, y_min, z_min);

    ss_nodes_centres << node_centre.x() << " "
                     << node_centre.y() << " "
                     << node_centre.z() << std::endl;

  }
  std::cout << "FINISHED SAVING MAP - " << counter << std::endl;

  point_cloud_file << ss_nodes_centres.str();

  point_cloud_file.close();

  return;
}

struct Box
{
    Box(const Eigen::Vector3d position,
        const Eigen::Matrix3d world2body,
        const Eigen::Vector3d dimension) :
         position_(position),
         world2body_(world2body),
         dimension_(dimension)
    {
    }

    bool contains(const Eigen::Vector3d point_W,
                  const bool print_case = false)
    {
      const Eigen::Vector3d point_B = world2body_ * (point_W - position_);
      return (point_B.x() >= (-dimension_.x() / 2) &&
              point_B.x() <= ( dimension_.x() / 2) &&
              point_B.y() >= (-dimension_.y() / 2) &&
              point_B.y() <= ( dimension_.y() / 2) &&
              point_B.z() >= (-dimension_.z() / 2) &&
              point_B.z() <= ( dimension_.z() / 2));
    }

    void getIfBounds(Eigen::Vector3i& if_min,
                     Eigen::Vector3i& if_max,
                     const Eigen::Vector3d map_origin,
                     const double resolution)
    {
      Eigen::Matrix3d body2world = world2body_.transpose();
      Eigen::Vector3d corner_0(-dimension_.x() / 2,
                               -dimension_.y() / 2,
                               -dimension_.z() / 2);
      Eigen::Vector3d corner_1( dimension_.x() / 2,
                               -dimension_.y() / 2,
                               -dimension_.z() / 2);
      Eigen::Vector3d corner_2(-dimension_.x() / 2,
                                dimension_.y() / 2,
                               -dimension_.z() / 2);
      Eigen::Vector3d corner_3( dimension_.x() / 2,
                                dimension_.y() / 2,
                               -dimension_.z() / 2);
      Eigen::Vector3d corner_4(-dimension_.x() / 2,
                               -dimension_.y() / 2,
                                dimension_.z() / 2);
      Eigen::Vector3d corner_5( dimension_.x() / 2,
                               -dimension_.y() / 2,
                                dimension_.z() / 2);
      Eigen::Vector3d corner_6( dimension_.x() / 2,
                               -dimension_.y() / 2,
                                dimension_.z() / 2);
      Eigen::Vector3d corner_7( dimension_.x() / 2,
                                dimension_.y() / 2,
                                dimension_.z() / 2);

      Eigen::Matrix<double, 3, 8> corners;
      corners << body2world * corner_0, body2world * corner_1, body2world * corner_2, body2world * corner_3,
                 body2world * corner_4, body2world * corner_5, body2world * corner_6, body2world * corner_7;

      Eigen::Vector3d bb_min = corners.rowwise().minCoeff();
      Eigen::Vector3d bb_max = corners.rowwise().maxCoeff();

      Eigen::Vector3d if_min_f = bb_min + position_ - map_origin;
      if_min = (if_min_f / resolution).cast<int>();

      Eigen::Vector3d if_max_f = bb_max + position_ - map_origin;
      if_max = (if_max_f / resolution).cast<int>() + Eigen::Vector3i::Constant(1);
    }

    const Eigen::Vector3d position_;
    const Eigen::Matrix3d world2body_;
    const Eigen::Vector3d dimension_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "map_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle private_nh("~");
    std::string host_ip;
    double resolution;
    std::string world_frameid;
    Eigen::Matrix3d enutoned;
    enutoned << 0,1,0,
                1,0,0,
                0,0,-1;
    nh.param("host_ip", host_ip,std::string("localhost"));
    nh.param("resolution",resolution,0.1);
    nh.param("world_frame_id",world_frameid,std::string("/world_enu"));
    nh.param("use_octree",use_octree,false);
    if(use_octree)
      server_drone = new OctomapServer(private_nh, nh,world_frameid);
    airsim_map_pub   = nh.advertise<sensor_msgs::PointCloud2>("/airsim_global_map", 1);  
    msr::airlib::RpcLibClientBase airsim_client_map_(host_ip);
    airsim_client_map_.confirmConnection();

    const std::string mav_name = "drone_1";
    std::cout << "===========" << std::endl;
    std::cout << "mav_box      = " << airsim_client_map_.simGetObjectScale(mav_name) << std::endl;
    msr::airlib::Pose mav_pose = airsim_client_map_.simGetObjectPose(mav_name);
    std::cout << "mav_position = " << mav_pose.position.x() << ", " << mav_pose.position.y() << ", " << mav_pose.position.z() << std::endl;

    vector<string> objects_list;
    ros::Rate rate(map_f);
    int count=0;

    std::vector<Eigen::Vector3f> points;
    while(ros::ok()){
        ros::spinOnce();
        if(count<=10){
          cloudMap.points.clear();
          if(use_octree)
            server_drone->m_octree->clear();
          objects_list = airsim_client_map_.simListSceneObjects("Cube.*");

          Eigen::Vector3d lower_bound = Eigen::Vector3d::Zero();
          Eigen::Vector3d upper_bound = Eigen::Vector3d::Zero();
          for(int i =0;i<objects_list.size();i++) {
            msr::airlib::Pose cube_pose;
            msr::airlib::Vector3r cube_scale_airsim;
            string object_name;
            object_name = objects_list[i];
            cube_pose = airsim_client_map_.simGetObjectPose(object_name);
            cube_scale_airsim = airsim_client_map_.simGetObjectScale(object_name);
            const Eigen::Vector3d cube_scale(cube_scale_airsim.x(), cube_scale_airsim.y(), cube_scale_airsim.z());

            Eigen::Vector3d position(cube_pose.position.x(), cube_pose.position.y(), cube_pose.position.z());
            Eigen::Vector3d cube_corner = position - (cube_scale / 2);
            if (cube_corner.x() < lower_bound.x()) {
              lower_bound.x() = cube_corner.x();
            }
            if (cube_corner.y() <= lower_bound.y()) {
              lower_bound.y() = cube_corner.y();
            }
            if (cube_corner.z() <= lower_bound.z()) {
              lower_bound.z() = cube_corner.z();
            }

            if (cube_corner.x() > upper_bound.x()) {
              upper_bound.x() = cube_corner.x();
            }
            if (cube_corner.y() > upper_bound.y()) {
              upper_bound.y() = cube_corner.y();
            }
            if (cube_corner.z() > upper_bound.z()) {
              upper_bound.z() = cube_corner.z();
            }
          }

          Eigen::Vector3d map_origin = lower_bound;
          Eigen::Vector3d map_size = upper_bound - lower_bound;
          std::cout << "map_origin = " << map_origin.x() << ", " << map_origin.y() << ", " << map_origin.z() << std::endl;
          std::cout << "map_size   = " << map_size.x() << ", " << map_size.y() << ", " << map_size.z() << std::endl;

          for(int i =0;i<objects_list.size();i++) {
            msr::airlib::Pose cube_pose;
            msr::airlib::Vector3r cube_scale_airsim;
            string object_name;
            object_name = objects_list[i];
            cube_pose = airsim_client_map_.simGetObjectPose(object_name);
            cube_scale_airsim = airsim_client_map_.simGetObjectScale(object_name);
            const Eigen::Vector3d cube_scale(cube_scale_airsim.x(), cube_scale_airsim.y(), cube_scale_airsim.z());

            Eigen::Vector3d position(cube_pose.position.x(), cube_pose.position.y(), cube_pose.position.z());
            Eigen::Quaterniond q;
            Eigen::Matrix3d body2world;
            q.w() = cube_pose.orientation.w();
            q.x() = cube_pose.orientation.x();
            q.y() = cube_pose.orientation.y();
            q.z() = cube_pose.orientation.z();

            if (world_frameid == std::string("/world_enu")) {
              body2world = enutoned * q.toRotationMatrix();
            } else if (world_frameid == std::string("/world_ned")) {
              body2world = q.toRotationMatrix();
            } else {
              ROS_ERROR("wrong map frame id!");
            }

            Box object(position, q.toRotationMatrix().transpose(), cube_scale);
            Eigen::Vector3i if_min, if_max;
            object.getIfBounds(if_min, if_max, map_origin, resolution);

//            std::cout << "\n\n==========" << std::endl;
//            std::cout << "object_name   = " << object_name << std::endl;
//            std::cout << "position      = " << position.x() << ", " << position.y() << ", " << position.z() << std::endl;
//            std::cout << "q             = " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
//            std::cout << "cube_scale    = " << cube_scale.x() << "," << cube_scale.y() << ", " << cube_scale.z()
//                      << std::endl;
//            std::cout << "body2world    = " << body2world << std::endl;
//            Eigen::Vector3d lb =
//                    Eigen::Vector3d(if_min.x(), if_min.y(), if_min.z()) * resolution + map_origin + Eigen::Vector3d::Constant(resolution / 2);
//            Eigen::Vector3d ub =
//                    Eigen::Vector3d(if_max.x(), if_max.y(), if_max.z()) * resolution + map_origin + Eigen::Vector3d::Constant(resolution / 2);
//            std::cout << "cube lb       = " << lb.x() << ", " << lb.y() << ", " << lb.z() << std::endl;
//            std::cout << "cube ub       = " << ub.x() << ", " << ub.y() << ", " << ub.z() << std::endl;

            for (int x = if_min.x(); x <= if_max.x(); x++) {
              for (int y = if_min.y(); y <= if_max.y(); y++) {
                for (int z = if_min.z(); z <= if_max.z(); z++) {
                  Eigen::Vector3d point_W =
                          Eigen::Vector3d(x, y, z) * resolution + map_origin + Eigen::Vector3d::Constant(resolution / 2);
                  bool contains = false;
                  if (object.contains(point_W)) {
                    contains = true;
                    Eigen::Vector3d obs_world = enutoned * point_W;
                    pcl::PointXYZ pt;
                    pt.x = obs_world[0];
                    pt.y = obs_world[1];
                    pt.z = obs_world[2];
                    cloudMap.points.push_back(pt);
                    points.push_back(Eigen::Vector3f(pt.x, pt.y, pt.z));
                    geometry_msgs::Point32 cpt;
                    cpt.x = pt.x;
                    cpt.y = pt.y;
                    cpt.z = pt.z;
                    if (use_octree)
                      server_drone->m_octree->updateNode(point3d(pt.x + 1e-5, pt.y + 1e-5, pt.z + 1e-5), true);
                  }

//                  if (object_name == std::string("Cube2_8")) {
//                    std::cout << position.x() << ", " << position.y() << ", " << position.z() << " | " << point_W.x() << ", " << point_W.y() << ", " << point_W.z() << " - " << ((contains) ? "True" : "False") << std::endl;
//                  }
                }
              }
            }
          }

//          for(int i =0;i<objects_list.size();i++){
//              msr::airlib::Pose cube_pose;
//              msr::airlib::Vector3r cube_scale_airsim;
//              string object_name;
//              object_name = objects_list[i];
//              cube_pose = airsim_client_map_.simGetObjectPose(object_name);
//              cube_scale_airsim = airsim_client_map_.simGetObjectScale(object_name);
//              const Eigen::Vector3d cube_scale(cube_scale_airsim.x(), cube_scale_airsim.y(), cube_scale_airsim.z());
//              Eigen::Vector3d position;
//              Eigen::Quaterniond q;
//              Eigen::Matrix3d body2world;
//              q.w() = cube_pose.orientation.w();
//              q.x() = cube_pose.orientation.x();
//              q.y() = cube_pose.orientation.y();
//              q.z() = cube_pose.orientation.z();
//
//              if(world_frameid==std::string("/world_enu")){
//                  position<<cube_pose.position.y(),cube_pose.position.x(),-cube_pose.position.z();
//                  body2world = enutoned*q.toRotationMatrix();
//              }
//              else if(world_frameid==std::string("/world_ned")) {
//                  position << cube_pose.position.x(),cube_pose.position.y(),cube_pose.position.z();
//                  body2world = q.toRotationMatrix();
//              }
//              else{
//                ROS_ERROR("wrong map frame id!");
//              }
//
//              std::cout << "object_name = " << object_name << std::endl;
//              std::cout << "position    = " << position.x() << ", " << position.y() << ", " << position.z() << std::endl;
//              std::cout << "q           = " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
//              std::cout << "cube_scale  = " << cube_scale.x() << "," << cube_scale.y() << ", " << cube_scale.z() << std::endl;
//              std::cout << "body2world  = " << body2world << std::endl;
//
//
//
//
//              if(count==0)
//                ROS_INFO("We are sending global map~ Please wait~");
//              double lx,ly,lz;
//              for(lx = -cube_scale.x()/2; lx<cube_scale.x()/2+resolution;lx+=resolution / 2) {
//                  for(ly = -cube_scale.y()/2; ly<cube_scale.y()/2+resolution;ly+=resolution / 2) {
//                      for(lz = -cube_scale.z()/2; lz<cube_scale.z()/2+resolution;lz+=resolution / 2) {
//                          Eigen::Vector3d obs_body;
//                          obs_body << lx,ly,lz;
//                          Eigen::Vector3d obs_world;
//                          obs_world = body2world*obs_body+position;
//                          pcl::PointXYZ pt;
//                          pt.x = obs_world[0];
//                          pt.y = obs_world[1];
//                          pt.z = obs_world[2];
//                          cloudMap.points.push_back(pt);
//                          points.push_back(Eigen::Vector3f(pt.x, pt.y, pt.z));
//                          geometry_msgs::Point32 cpt;
//                          cpt.x = pt.x;
//                          cpt.y = pt.y;
//                          cpt.z = pt.z;
//                          if(use_octree)
//                            server_drone->m_octree->updateNode(point3d(pt.x+1e-5,pt.y+1e-5,pt.z+1e-5), true);
//                      }
//                  }
//              }
//          }
          if(use_octree)
            server_drone->publishAll();
          count++;
          cloudMap.width = cloudMap.points.size();
          cloudMap.height = 1;
          cloudMap.is_dense = true;
          pcl::toROSMsg(cloudMap, globalMap_pcd);
          globalMap_pcd.header.frame_id = world_frameid; 
          airsim_map_pub.publish(globalMap_pcd);
          ROS_INFO("send global map");

//          if (count < 5) {
//            saveMap(points, resolution);
//          }
        }

        rate.sleep();
    }
    return 0;
} 
/*
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "drone_1": {
      "VehicleType": "SimpleFlight",
      "DefaultVehicleState": "Armed",
      "Sensors": {
        "Barometer": {
          "SensorType": 1,
          "Enabled" : false
        },
        "Imu": {
          "SensorType": 2,
          "Enabled" : true
        },
        "Gps": {
          "SensorType": 3,
          "Enabled" : false
        },
        "Magnetometer": {
          "SensorType": 4,
          "Enabled" : false
        },
        "Distance": {
          "SensorType": 5,
          "Enabled" : false
        },
        "Lidar": {
          "SensorType": 6,
          "Enabled" : false,
          "NumberOfChannels": 16,
          "RotationsPerSecond": 10,
          "PointsPerSecond": 100000,
          "X": 0, "Y": 0, "Z": -1,
          "Roll": 0, "Pitch": 0, "Yaw" : 0,
          "VerticalFOVUpper": 0,
          "VerticalFOVLower": -0,
          "HorizontalFOVStart": -90,
          "HorizontalFOVEnd": 90,
          "DrawDebugPoints": true,
          "DataFrame": "SensorLocalFrame"
        }
      },
      "Cameras": {
        "front_center_custom": {
          "CaptureSettings": [
            {
              "PublishToRos": 1,
              "ImageType": 3,
              "Width": 320,
              "Height": 240,
              "FOV_Degrees": 90,
              "DepthOfFieldFstop": 2.8,
              "DepthOfFieldFocalDistance": 200.0, 
              "DepthOfFieldFocalRegion": 200.0,
              "TargetGamma": 1.5
            }
          ],
          "Pitch": 0.0,
          "Roll": 0.0,
          "X": 0.25,
          "Y": 0.0,
          "Yaw": 0.0,
          "Z": 0.3
        }
      },
      "X": 0, "Y": 0, "Z": 0,
      "Pitch": 0, "Roll": 0, "Yaw": 0
    }
  },
  "SubWindows": [
    {"WindowID": 1, "ImageType": 3, "CameraName": "front_center_custom", "Visible": false}
  ] 
}

*/