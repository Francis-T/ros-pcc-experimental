#include "compressed_pcl_subscriber.h"

#define QUEUE_SIZE_PUB 5
#define QUEUE_SIZE_SUB 5

#define DEFAULT_DECODER_ARG_COUNT 5

using namespace std::chrono;

int _received_count = 0;

RepublishDecompressedPCL::RepublishDecompressedPCL(
    ros::NodeHandle& node_handler, 
    ros::NodeHandle& param_node_handler
) : _node_handler(node_handler), _param_node_handler(param_node_handler)
{
    _pub = _node_handler.advertise<sensor_msgs::PointCloud2>(
        "reconstructed_points2", 
        QUEUE_SIZE_PUB
    );
    _sub = _node_handler.subscribe(
        "tmc3_compressed_points2", 
        QUEUE_SIZE_SUB, 
        &RepublishDecompressedPCL::OnCompressedPointCloudReceivedCallback, 
        this
    );

    ROS_INFO("Waiting for compressed pointclouds...");
}

void RepublishDecompressedPCL::OnCompressedPointCloudReceivedCallback(
    const experimental_pcl_transport::MinimalCompressedPointCloud2& compressed
){
    _received_count += 1;

    uint32_t compressed_data_size = compressed.size;
    // empty buffer
    if (compressed_data_size == 0)
    {
        return;
    }
    std::vector<unsigned char> vec_data = compressed.compressed_data;

    char * compressed_data = new char[compressed_data_size];
    compressed_data = reinterpret_cast<char*>(&vec_data[0]);

    /* Save the received compressed point cloud to a file */
    const std::string compressed_filename("recv0001.ply");
    std::ofstream compressed_file( compressed_filename, ios::out | ios::binary );
    if (compressed_file.is_open())
    {
        compressed_file.write( compressed_data, compressed_data_size );
        compressed_file.close();
    }
    ROS_INFO("Wrote compressed point cloud to file!");

    /*** Load the decompression parameters from some defaults ***/
    pcc::chrono::Stopwatch<std::chrono::steady_clock> param_load_clock_wall;
    param_load_clock_wall.start();
    ROS_INFO("Parsing Parameters...");

    std::string compressedFilename("recv0001.ply");
    std::string reconstructedFilename("reconst0001.ply");
    std::string rosWorkspacePath("/home/nict-nuc-02/Workspace/ros/point_cloud_transport_ws/");

    char* compressedPath    =  new char[100];
    char* reconstructedPath =  new char[100];
    std::strcpy( compressedPath,    (std::string("--compressedStreamPath=")  + rosWorkspacePath + compressedFilename).c_str() );
    std::strcpy( reconstructedPath, (std::string("--reconstructedDataPath=") + rosWorkspacePath + reconstructedFilename).c_str() );

    const char* DEFAULT_DECODER_ARGS[ DEFAULT_DECODER_ARG_COUNT ] = {
        "--mode=1",
        compressedPath,
        reconstructedPath,
        "--srcUnit=metre",
        "--srcUnitLength=1"
    };

    ROS_INFO("Compressed Point Cloud Path: %s", compressedPath);
    ROS_INFO("Reconstructed Point Cloud Path: %s", reconstructedPath);

    Parameters params;
    try {
      if (!ParseParameters(DEFAULT_DECODER_ARG_COUNT, 
                           DEFAULT_DECODER_ARGS,
                           params, true, true))
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

    /*** Decompress the point cloud ***/
    ROS_INFO("Decompressing Point Cloud...");
    // Timers to count elapsed wall/user time
    pcc::chrono::Stopwatch<std::chrono::steady_clock> decompress_wall;
    pcc::chrono::Stopwatch<pcc::chrono::utime_inc_children_clock> decompress_user;

    decompress_wall.start();
    int ret = SequenceDecoder(&params).decompress(&decompress_user);
    decompress_wall.stop();

    auto total_compression_time = duration_cast<milliseconds>(decompress_wall.count()).count();
    ROS_INFO("Decompressing Finished.");

    /*** Load the decompressed point cloud and republish it ***/
    open3d::geometry::PointCloud o3d_pc;
    std::string decompressedFilePath = rosWorkspacePath + reconstructedFilename;

    ROS_INFO("Publishing Reconstructed Point Cloud...");
    open3d::io::ReadPointCloud(decompressedFilePath, o3d_pc);
    sensor_msgs::PointCloud2 ros_pointcloud;

    Eigen::Vector3d center(0, 0, 0);
    o3d_pc.Scale(0.001, center);

    open3d_conversions::open3dToRos(o3d_pc, ros_pointcloud, "reconstructed_points2");
    copyCloudMetadata( ros_pointcloud, compressed );
    ros_pointcloud.header.stamp = ros::Time::now();
    ros_pointcloud.width = o3d_pc.points_.size();

    _pub.publish( ros_pointcloud );
    ROS_INFO("Reconstructed Point Cloud Republished.");

    return;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "compressed_pcl_receiver");
    ros::NodeHandle node_handler;
    ros::NodeHandle param_node_handler("~");

    RepublishDecompressedPCL republisher( node_handler, param_node_handler );
    ros::spin();
    // while ( _received_count <= 10 )
    // {
    //     ros::spinOnce();
    // }

    return 0;
}

