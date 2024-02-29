#ifndef COMPRESSED_PCL_PUBLISHER_H
#define COMPRESSED_PCL_PUBLISHER_H

#include <ctime>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <type_traits>
#include <utility>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"

#include "open3d_conversions/open3d_conversions.h"
#include "open3d/Open3D.h"

#include "TMC3.h"

#include "experimental_pcl_transport/MinimalCompressedPointCloud2.h"

class RepublishCompressedPCL
{
    public:
        RepublishCompressedPCL(
            ros::NodeHandle& node_handler, 
            ros::NodeHandle& param_node_handler,
            double downsampling_voxel_size
        );

        void OnPointCloudReceivedCallback(
            const sensor_msgs::PointCloud2ConstPtr& ros_pc2_ptr
        );

    protected:
        void processPointCloud(
            const open3d::geometry::PointCloud &o3d_pcd, 
            string file_suffix,
            const sensor_msgs::PointCloud2ConstPtr& ros_pc2_ptr
        );

    private:
        ros::NodeHandle _node_handler;
        ros::NodeHandle _param_node_handler;
        ros::Publisher  _pub;
        ros::Subscriber _sub;
        double          _downsampling_voxel_size = 0.01;
};

template<typename PC1, typename PC2>
void copyCloudMetadata(PC1& target, const PC2& source)
{
  target.header         = source->header;
  target.height         = source->height;
  target.width          = source->width;
  target.is_bigendian   = source->is_bigendian;
  target.point_step     = source->point_step;
  target.row_step       = source->row_step;
  target.is_dense       = source->is_dense;
}

#endif /* COMPRESSED_PCL_PUBLISHER_H */

