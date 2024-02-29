#include "compressed_pcl_publisher.h"

#define D_SAVE_TIME_DATA 0
#define DEFAULT_ENCODER_ARG_COUNT 8
#define QUEUE_SIZE_PUB 5
#define QUEUE_SIZE_SUB 5

using namespace std::chrono;

int _received_count = 0;



//   void multiDownsample(const open3d::geometry::PointCloud &o3d_pcd)
//   {
//       //double dsValues[] = {
//       //    0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1,
//       //    0.09, 0.08, 0.07, 0.06, 0.05, 0.04, 0.03, 0.02, 0.01,
//       //    0.009, 0.008, 0.007, 0.006, 0.005, 0.004, 0.003, 0.002, 0.001,
//       //    0.0009, 0.0008, 0.0007, 0.0006, 0.0005, 0.0004, 0.0003, 0.0002, 0.0001
//       //};
//       //string dsFileTag[] = {
//       //    "0_9", "0_8", "0_7", "0_6", "0_5", "0_4", "0_3", "0_2", "0_1",
//       //    "0_09", "0_08", "0_07", "0_06", "0_05", "0_04", "0_03", "0_02", "0_01",
//       //    "0_009", "0_008", "0_007", "0_006", "0_005", "0_004", "0_003", "0_002", "0_001",
//       //    "0_0009", "0_0008", "0_0007", "0_0006", "0_0005", "0_0004", "0_0003", "0_0002", "0_0001"
//       //};
//       double dsValues[] = { 0.01 };
//       string dsFileTag[] = { "0_01" };
//   
//       int dsValueCount = (int)(sizeof(dsValues) / sizeof(double));
//       for (int i=0; i < dsValueCount; i++) {
//   
//       }
//   }
//   
//   void pclCallback(const sensor_msgs::PointCloud2ConstPtr& ros_pc2_ptr)
//   {
//       _received = true;
//   
//       // Load the original point cloud from the ROs  message
//       open3d::geometry::PointCloud o3d_pcd;
//       open3d_conversions::rosToOpen3d(ros_pc2_ptr, o3d_pcd);
//       ROS_INFO("Original PCL Size: %lu", o3d_pcd.points_.size());
//       ROS_INFO("%d", o3d_pcd.HasColors());
//   
//       // Save the data to a file
//       /*    Note: TODO Need to assign a unique name to the saved file */
//       const std::string original_filename("original.ply");
//       if (open3d::io::WritePointCloud(original_filename, o3d_pcd)) {
//           ROS_INFO("Succesfully wrote file");
//       } else {
//           ROS_INFO("Failed to write file");
//       }
//   
//       multiDownsample(o3d_pcd);
//   
//       return;
//   }

/**
    Note: Most of this code was based directly on `perception_open3d_examples/src/ex_downsample.cpp`
**/

RepublishCompressedPCL::RepublishCompressedPCL(
    ros::NodeHandle& node_handler, 
    ros::NodeHandle& param_node_handler,
    double downsampling_voxel_size
) : _node_handler(node_handler), _param_node_handler(param_node_handler)
{
    _pub = _node_handler.advertise<experimental_pcl_transport::MinimalCompressedPointCloud2>("tmc3_compressed_points2", QUEUE_SIZE_PUB);
    _sub = _node_handler.subscribe("points2", QUEUE_SIZE_SUB, &RepublishCompressedPCL::OnPointCloudReceivedCallback, this);

    if (downsampling_voxel_size)
    {
        _downsampling_voxel_size = downsampling_voxel_size;
    }
    else
    {
        _param_node_handler.getParam("voxel_size", _downsampling_voxel_size);
    }


    ROS_INFO("Waiting for pointclouds...");
}

void RepublishCompressedPCL::OnPointCloudReceivedCallback(
    const sensor_msgs::PointCloud2ConstPtr& ros_pc2_ptr
){
    _received_count += 1;

    // Load the original point cloud from the ROs  message
    open3d::geometry::PointCloud o3d_pcd;
    open3d_conversions::rosToOpen3d(ros_pc2_ptr, o3d_pcd);
    ROS_INFO("Original PCL Size: %lu", o3d_pcd.points_.size());
    ROS_INFO("%d", o3d_pcd.HasColors());

    // Save the data to a file
    /*    Note: TODO Need to assign a unique name to the saved file */
    const std::string original_filename("original.ply");
    if (open3d::io::WritePointCloud(original_filename, o3d_pcd)) {
        ROS_INFO("Succesfully wrote file");
    } else {
        ROS_INFO("Failed to write file");
    }

    processPointCloud(o3d_pcd, "0_01", ros_pc2_ptr);

    return;
}

void RepublishCompressedPCL::processPointCloud(
    const open3d::geometry::PointCloud &o3d_pcd, 
    string file_suffix,
    const sensor_msgs::PointCloud2ConstPtr& ros_pc2_ptr
)
{
    /* Pre-process the point cloud */
    ROS_INFO("Pre-processing Point Cloud...");
    pcc::chrono::Stopwatch<std::chrono::steady_clock> preproc_clock_wall;
    preproc_clock_wall.start();

    /*** Trim the point cloud ***/
    std::vector<size_t> indices;
    double pcd_z_cutoff = (0.20 * (o3d_pcd.GetMaxBound()[2] - o3d_pcd.GetMinBound()[2])) + o3d_pcd.GetMinBound()[2];
    ROS_INFO("Selected Cutoff Point: %f", pcd_z_cutoff);
    ROS_INFO("    Min: %f, Max: %f", o3d_pcd.GetMinBound()[2], o3d_pcd.GetMaxBound()[2]);
    int within_cutoff_count = 0;
    int outside_cutoff_count = 0;
    for (size_t i = 0; i < o3d_pcd.points_.size(); i++)
    {
        if ( o3d_pcd.points_[i][2] <= pcd_z_cutoff )
        {
            indices.push_back(i);
            within_cutoff_count++;
        }
        else
        {
            outside_cutoff_count++;
        }
    }
    auto cutoff_pcd = o3d_pcd.SelectByIndex(indices, false);
    ROS_INFO("Trimmed %d points. %d points remain", outside_cutoff_count, within_cutoff_count);

    /*** Downsample the point cloud ***/
    auto downsampled_pcd = cutoff_pcd->VoxelDownSample( _downsampling_voxel_size );
    ROS_INFO("Downsampled PCL Size: %lu", downsampled_pcd->points_.size());

    /*** Remove any outlier points ***/
    std::shared_ptr<open3d::geometry::PointCloud> filtered_pcd;
    std::vector<size_t> ind;
    std::tie(filtered_pcd, ind) = downsampled_pcd->RemoveStatisticalOutliers(20, 2.0);
    ROS_INFO("Outliers Removed PCL Size: %lu", filtered_pcd->points_.size());
    
    /*** Finally, rescale the point cloud ***/
    /***    NOTE: This is a very important step to do BEFORE running compression    ***/
    Eigen::Vector3d center(0, 0, 0);
    filtered_pcd->Scale(1000.0, center);

    /*** Save the data to a file ***/
    /*    Note: Need to assign a unique name to the saved file */
    std::string filename_xyz("preprocessed-" + file_suffix + ".ply");
    if (open3d::io::WritePointCloud(filename_xyz, *filtered_pcd)) {
        ROS_INFO("Succesfully wrote file");
    } else {
        ROS_INFO("Failed to write file");
    }

    preproc_clock_wall.stop();
    auto total_preprocessing_time = duration_cast<milliseconds>(preproc_clock_wall.count()).count();
    ROS_INFO("Point Cloud Preprocessing Done.");

    /*** Load the compression parameters from some defaults ***/
    pcc::chrono::Stopwatch<std::chrono::steady_clock> param_load_clock_wall;
    param_load_clock_wall.start();
    ROS_INFO("Parsing Parameters...");

    std::string compressedFilename("c0001-" + file_suffix + ".ply");
    std::string reconstructedFilename("r0001-" + file_suffix + ".ply");

    std::string rosWorkspacePath("/home/nict-nuc-02/Workspace/ros/point_cloud_transport_ws/");
    char* uncompressedPath  =  new char[100];
    char* compressedPath    =  new char[100];
    char* reconstructedPath =  new char[100];
    std::strcpy( uncompressedPath,  (std::string("--uncompressedDataPath=")  + rosWorkspacePath + filename_xyz).c_str() );
    std::strcpy( compressedPath,    (std::string("--compressedStreamPath=")  + rosWorkspacePath + compressedFilename).c_str() );
    std::strcpy( reconstructedPath, (std::string("--reconstructedDataPath=") + rosWorkspacePath + reconstructedFilename).c_str() );

    const char* DEFAULT_ENCODER_ARGS[ DEFAULT_ENCODER_ARG_COUNT ] = {
       "--mode=0",
       uncompressedPath,
       compressedPath,
       reconstructedPath,
       "--mergeDuplicatedPoints=1",
       "--srcUnit=metre",
       "--srcUnitLength=1",
       "--attribute=color"
    };
    
    ROS_INFO("Pre-processed Point Cloud Path: %s", uncompressedPath);
    ROS_INFO("Compressed Point Cloud Path: %s", compressedPath);
    ROS_INFO("Reconstructed Point Cloud Path: %s", reconstructedPath);

    Parameters params;
    try {
      if (!ParseParameters(DEFAULT_ENCODER_ARG_COUNT, 
                           DEFAULT_ENCODER_ARGS,
                           params, false, true))
      {
        ROS_INFO("Failed to Parse Parameters");
        return;
      }
    }
    catch (df::program_options_lite::ParseFailure& e)
    {
      std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
                << e.val << "\"." << std::endl;
      ROS_INFO("Exception Occurred while running Parse Parameters");
      return;
    }
    param_load_clock_wall.stop();
    auto total_param_loading_time = duration_cast<milliseconds>(param_load_clock_wall.count()).count();
    ROS_INFO("Parameters Loaded.");

    /*** Compress the point cloud ***/
    ROS_INFO("Compressing Point Cloud...");
    // Timers to count elapsed wall/user time
    pcc::chrono::Stopwatch<std::chrono::steady_clock> compress_wall;
    pcc::chrono::Stopwatch<pcc::chrono::utime_inc_children_clock> compress_user;

    compress_wall.start();
    int ret = SequenceEncoder(&params).compress(&compress_user);
    compress_wall.stop();

    auto total_compression_time = duration_cast<milliseconds>(compress_wall.count()).count();
    ROS_INFO("Compressing Finished.");

    // TODO Load the data from a file and republish
    std::streampos file_sz;
    char * compressed_data;

    std::ifstream compressed_ply_file (compressedFilename, ios::binary | ios::in | ios::ate );
    if (compressed_ply_file.is_open())
    {
        file_sz = compressed_ply_file.tellg();
        compressed_data = new char[file_sz];

        compressed_ply_file.seekg(0, ios::beg);
        compressed_ply_file.read(compressed_data, file_sz);

        compressed_ply_file.close();
    }

    experimental_pcl_transport::MinimalCompressedPointCloud2 compressed_msg;


    copyCloudMetadata(compressed_msg, ros_pc2_ptr);
    //compressed_msg.header = ros_pc2_ptr->header;
    compressed_msg.size = file_sz;

    /* Note: This part copied from draco_publisher.cpp */
    auto cast_buffer = reinterpret_cast<const unsigned char*>(compressed_data);
    std::vector<unsigned char> vec_data(cast_buffer, cast_buffer + file_sz);
    compressed_msg.compressed_data = vec_data;

    _pub.publish(compressed_msg);

    // sensor_msgs::PointCloud2 ros_pc2;
    // open3d_conversions::open3dToRos(*filtered_pcd, ros_pc2, cloud_data->header.frame_id);
    // pub_.publish(ros_pc2);
    // ROS_INFO("Published downsampled pointcloud with points original/downsampled: %lu/%lu", pcd.points_.size(),
    //          filtered_pcd->points_.size());


    #ifdef D_SAVE_TIME_DATA
    /*** Save diagnostics data ***/
    ROS_INFO("Saving time data for benchmark...");
    std::ofstream dataFile("data.csv", ios::app);
    if (dataFile.is_open())
    {
        dataFile << _downsampling_voxel_size << ", ";
        dataFile << total_preprocessing_time << ", ";
        dataFile << total_param_loading_time << ", ";
        dataFile << total_compression_time << std::endl;
        dataFile.close();
    }
    ROS_INFO("Time data saved...");
    #endif /* D_SAVE_TIME_DATA */
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_receiver");
    ros::NodeHandle node_handler;
    ros::NodeHandle param_node_handler("~");

    RepublishCompressedPCL republisher( node_handler, param_node_handler, 0.01);
    ros::spin();
    // while ( _received_count <= 10 )
    // {
    //     ros::spinOnce();
    // }

    return 0;
}

