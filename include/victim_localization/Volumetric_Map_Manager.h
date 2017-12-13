#ifndef VOLUMETRIC_MAP_H
#define VOLUMETRIC_MAP_H


#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap_world/octomap_manager.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <octomap_server/OctomapServerConfig.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>


#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <costmap_2d/costmap_2d_ros.h>



//typedef geometry_msgs::Pose Pose;

class Volumetric_Map
{

   typedef octomap::OcTree OcTreeT;
   typedef octomap_msgs::GetOctomap OctomapSrv;
   typedef octomap_msgs::BoundingBoxQuery BBXSrv;

public:
   //....variables....

  std::string topic_pointcloud_  = "/front_cam/depth/points";
  ros::Subscriber pointcloud_sub_;
  costmap_2d::Costmap2DROS *costmap_ros_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  //...Main methods...
  Volumetric_Map(volumetric_mapping::OctomapManager *manager);
  int Get2DMapValue(const octomap::OcTreeKey &key_);
  void SetCostMapRos(costmap_2d::Costmap2DROS *costmap_);




protected:
  void callbackSetPointCloud(const sensor_msgs::PointCloud2::ConstPtr &input_msg);
  volumetric_mapping::OctomapManager *manager_;

  //2D occupancy Map related functions
     void  PrecheckFor2DMap (const ros::Time& rostime) ;

     void handleOccupiedNode(const OcTreeT::iterator& it);

     void handleOccupiedNodeInBBX(const OcTreeT::iterator& it);

     void handleFreeNode(const OcTreeT::iterator& it);

     void handleFreeNodeInBBX(const OcTreeT::iterator& it);

     void update2DMap(const OcTreeT::iterator& it, bool occupied);

    inline unsigned mapIdx(int i, int j) const {
      return m_gridmap.info.width * j + i;
    }

    inline bool isInUpdateBBX(const OcTreeT::iterator& it) const {
      // 2^(tree_depth-depth) voxels wide:
      unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
      octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
      return (key[0] + voxelWidth >= m_updateBBXMin[0]
              && key[1] + voxelWidth >= m_updateBBXMin[1]
              && key[0] <= m_updateBBXMax[0]
              && key[1] <= m_updateBBXMax[1]);
  }

    void adjustMapData(nav_msgs::OccupancyGrid& map,
                       const nav_msgs::MapMetaData& oldMapInfo) const;

    void Convert2DMaptoOccupancyGrid(const ros::Time &rostime);

    void Publish2DOccupancyMap(void);


    inline unsigned mapIdx(const octomap::OcTreeKey& key) const {
      return mapIdx((key[0] - m_paddedMinKey[0]) / m_multires2DScale,
                    (key[1] - m_paddedMinKey[1]) / m_multires2DScale);
  }

    inline bool mapChanged(const nav_msgs::MapMetaData& oldMapInfo, const nav_msgs::MapMetaData& newMapInfo) {
      return (    oldMapInfo.height != newMapInfo.height
                  || oldMapInfo.width != newMapInfo.width
                  || oldMapInfo.origin.position.x != newMapInfo.origin.position.x
                  || oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
  }

    int IsnodeOccupied (const OcTreeT::iterator& it);

      ros::Publisher m_mapPub;

      octomap::KeyRay m_keyRay;  // temp storage for ray casting
      octomap::OcTreeKey m_updateBBXMin;
      octomap::OcTreeKey m_updateBBXMax;

      std::string m_worldFrameId; // the map frame
      std::string m_baseFrameId; // base of the robot for ground plane filtering
      bool m_useHeightMap;
      std_msgs::ColorRGBA m_color;
      std_msgs::ColorRGBA m_colorFree;
      double m_colorFactor;

      bool m_latchedTopics;
      bool m_publishFreeSpace;

      double m_res;
      unsigned m_treeDepth;
      unsigned m_maxTreeDepth;

      double m_pointcloudMinX;
      double m_pointcloudMaxX;
      double m_pointcloudMinY;
      double m_pointcloudMaxY;
      double m_pointcloudMinZ;
      double m_pointcloudMaxZ;
      double m_occupancyMinZ;
      double m_occupancyMaxZ;
      double m_minSizeX;
      double m_minSizeY;
      bool m_filterSpeckles;

      bool m_filterGroundPlane;
      double m_groundFilterDistance;
      double m_groundFilterAngle;
      double m_groundFilterPlaneDistance;

      bool m_compressMap;

      bool m_initConfig;

      // downprojected 2D map:
      bool m_incrementalUpdate;
      bool m_publish2DMap;
      bool m_mapOriginChanged;
      octomap::OcTreeKey m_paddedMinKey;
      unsigned m_multires2DScale;
      bool m_projectCompleteMap;
      bool m_useColoredMap;

public:
      nav_msgs::OccupancyGrid m_gridmap;
      std::shared_ptr<OcTreeT> m_octree;



 //visualize

};

#endif // VOLUMETRIC_MAP_H
