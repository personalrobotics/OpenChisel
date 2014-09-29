// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <chisel_ros/ChiselServer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Chisel");
    ros::NodeHandle nh;
    int chunkSizeX, chunkSizeY, chunkSizeZ;
    double voxelResolution;
    double truncationDist;
    int weight;
    bool useCarving;
    bool useColor;
    double carvingDist;
    std::string depthImageTopic;
    std::string depthImageInfoTopic;
    std::string depthImageTransform;
    std::string colorImageTopic;
    std::string colorImageInfoTopic;
    std::string colorImageTransform;
    std::string baseTransform;
    std::string meshTopic;
    nh.param("chunk_size_x", chunkSizeX, 32);
    nh.param("chunk_size_y", chunkSizeY, 32);
    nh.param("chunk_size_z", chunkSizeZ, 32);
    nh.param("truncation_dist_m", truncationDist, 0.2);
    nh.param("integration_weight", weight, 1);
    nh.param("use_voxel_carving", useCarving, true);
    nh.param("use_color", useColor, true);
    nh.param("carving_dist_m", carvingDist, 0.05);
    nh.param("voxel_resolution_m", voxelResolution, 0.03);
    nh.param("depth_image_topic", depthImageTopic, std::string("/camera/depth_registered/image_rect"));
    nh.param("depth_image_info_topic", depthImageInfoTopic, std::string("/camera/depth/camera_info"));
    nh.param("depth_image_transform", depthImageTransform, std::string("/camera_depth_optical_frame"));
    nh.param("color_image_topic", colorImageTopic, std::string("/camera/rgb/image_rect_color"));
    nh.param("color_image_info_topic", colorImageInfoTopic, std::string("/camera/rgb/camera_info"));
    nh.param("color_image_transform", colorImageTransform, std::string("/camera_depth_optical_frame"));
    nh.param("base_transform", baseTransform, std::string("/camera_link"));
    nh.param("mesh_topic", meshTopic, std::string("full_mesh"));

    chisel_ros::ChiselServerPtr server(new chisel_ros::ChiselServer(nh, chunkSizeX, chunkSizeY, chunkSizeZ, voxelResolution, useColor));
    server->SetupProjectionIntegrator(truncationDist, static_cast<uint16_t>(weight), useCarving, carvingDist);
    server->SubscribeDepthImage(depthImageTopic, depthImageInfoTopic, depthImageTransform);
    if (useColor)
        server->SubscribeColorImage(colorImageTopic, colorImageInfoTopic, colorImageTransform);
    server->SetBaseTransform(baseTransform);
    server->SetupMeshPublisher(meshTopic);


    while (ros::ok())
    {
        ros::Rate loop_rate(100);
        ros::spinOnce();
    }
}





