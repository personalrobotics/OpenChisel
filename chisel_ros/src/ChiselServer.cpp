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

#include <chisel_ros/ChiselServer.h>
#include <chisel_ros/Conversions.h>
#include <chisel_ros/Serialization.h>
#include <visualization_msgs/Marker.h>
#include <open_chisel/truncation/QuadraticTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>

namespace chisel_ros
{

    ChiselServer::ChiselServer() :
            useColor(false), hasNewData(false), nearPlaneDist(0.05), farPlaneDist(5), isPaused(false), mode(FusionMode::DepthImage)
    {

    }

    ChiselServer::~ChiselServer()
    {

    }

    void ChiselServer::AdvertiseServices()
    {
        resetServer = nh.advertiseService("Reset", &ChiselServer::Reset, this);
        pauseServer = nh.advertiseService("TogglePaused", &ChiselServer::TogglePaused, this);
        saveMeshServer = nh.advertiseService("SaveMesh", &ChiselServer::SaveMesh, this);
        getAllChunksServer = nh.advertiseService("GetAllChunks", &ChiselServer::GetAllChunks, this);
    }

    void ChiselServer::SetupMeshPublisher(const std::string& topic)
    {
        meshTopic = topic;
        meshPublisher = nh.advertise<visualization_msgs::Marker>(meshTopic, 1);
    }

    void ChiselServer::PublishMeshes()
    {
        visualization_msgs::Marker marker;
        FillMarkerTopicWithMeshes(&marker);

        if(!marker.points.empty())
            meshPublisher.publish(marker);
    }


    void ChiselServer::SetupChunkBoxPublisher(const std::string& boxTopic)
    {
        chunkBoxTopic = boxTopic;
        chunkBoxPublisher = nh.advertise<visualization_msgs::Marker>(chunkBoxTopic, 1);
        latestChunkPublisher = nh.advertise<visualization_msgs::Marker>(chunkBoxTopic + "/latest", 1);
    }

    void ChiselServer::SetupDepthPosePublisher(const std::string& depthPoseTopic)
    {
        depthCamera.lastPosePublisher = nh.advertise<geometry_msgs::PoseStamped>(depthPoseTopic, 1);
    }

    void ChiselServer::SetupColorPosePublisher(const std::string& colorPoseTopic)
    {
        colorCamera.lastPosePublisher = nh.advertise<geometry_msgs::PoseStamped>(colorPoseTopic, 1);
    }

    void ChiselServer::SetupDepthFrustumPublisher(const std::string& frustumTopic)
    {
        depthCamera.frustumPublisher = nh.advertise<visualization_msgs::Marker>(frustumTopic, 1);
    }

    void ChiselServer::SetupColorFrustumPublisher(const std::string& frustumTopic)
    {
        colorCamera.frustumPublisher = nh.advertise<visualization_msgs::Marker>(frustumTopic, 1);
    }

    void ChiselServer::PublishDepthFrustum()
    {
        chisel::Frustum frustum;
        depthCamera.cameraModel.SetupFrustum(depthCamera.lastPose, &frustum);
        visualization_msgs::Marker marker = CreateFrustumMarker(frustum);
        depthCamera.frustumPublisher.publish(marker);
    }

    void ChiselServer::PublishColorFrustum()
    {
        chisel::Frustum frustum;
        colorCamera.cameraModel.SetupFrustum(colorCamera.lastPose, &frustum);
        visualization_msgs::Marker marker = CreateFrustumMarker(frustum);
        colorCamera.frustumPublisher.publish(marker);
    }

    visualization_msgs::Marker ChiselServer::CreateFrustumMarker(const chisel::Frustum& frustum)
    {
        visualization_msgs::Marker marker;
        marker.id = 0;
        marker.header.frame_id = baseTransform;
        marker.color.r = 1.;
        marker.color.g = 1.;
        marker.color.b = 1.;
        marker.color.a = 1.;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        const chisel::Vec3* lines = frustum.GetLines();
        for (int i = 0; i < 24; i++)
        {
            const chisel::Vec3& linePoint = lines[i];
            geometry_msgs::Point pt;
            pt.x = linePoint.x();
            pt.y = linePoint.y();
            pt.z = linePoint.z();
            marker.points.push_back(pt);
        }

        return marker;
    }

    void ChiselServer::PublishDepthPose()
    {
        chisel::Transform lastPose = depthCamera.lastPose;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = baseTransform;
        pose.header.stamp = depthCamera.lastImageTimestamp;
        pose.pose.position.x = lastPose.translation()(0);
        pose.pose.position.y = lastPose.translation()(1);
        pose.pose.position.z = lastPose.translation()(2);

        chisel::Quaternion quat(lastPose.rotation());
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();

        depthCamera.lastPosePublisher.publish(pose);
    }

    void ChiselServer::PublishColorPose()
    {
        chisel::Transform lastPose = colorCamera.lastPose;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = baseTransform;
        pose.header.stamp = colorCamera.lastImageTimestamp;
        pose.pose.position.x = lastPose.translation()(0);
        pose.pose.position.y = lastPose.translation()(1);
        pose.pose.position.z = lastPose.translation()(2);

        chisel::Quaternion quat(lastPose.rotation());
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();

        colorCamera.lastPosePublisher.publish(pose);
    }


    ChiselServer::ChiselServer(const ros::NodeHandle& nodeHanlde, int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color, FusionMode fusionMode) :
            nh(nodeHanlde), useColor(color), hasNewData(false), isPaused(false), mode(fusionMode), farPlaneDist(0), nearPlaneDist(0)
    {
        chiselMap.reset(new chisel::Chisel(Eigen::Vector3i(chunkSizeX, chunkSizeY, chunkSizeZ), resolution, color));
    }

    bool ChiselServer::TogglePaused(chisel_msgs::PauseService::Request& request, chisel_msgs::PauseService::Response& response)
    {
        SetPaused(!IsPaused());
        return true;
    }

    bool ChiselServer::Reset(chisel_msgs::ResetService::Request& request, chisel_msgs::ResetService::Response& response)
    {
        chiselMap->Reset();
        return true;
    }

    void ChiselServer::SubscribeDepthImage(const std::string& imageTopic, const std::string& infoTopic, const std::string& transform)
    {
        depthCamera.imageTopic = imageTopic;
        depthCamera.transform = transform;
        depthCamera.infoTopic = infoTopic;
        depthCamera.imageSubscriber = nh.subscribe(depthCamera.imageTopic, 20, &ChiselServer::DepthImageCallback, this);
        depthCamera.infoSubscriber = nh.subscribe(depthCamera.infoTopic, 20, &ChiselServer::DepthCameraInfoCallback, this);
    }


    void ChiselServer::DepthCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo)
    {
        SetDepthCameraInfo(cameraInfo);
    }

    void ChiselServer::SetDepthPose(const Eigen::Affine3f& tf)
    {
        depthCamera.lastPose = tf;
        depthCamera.gotPose = true;
    }

    void ChiselServer::SetColorImage(const sensor_msgs::ImageConstPtr& img)
    {
        if (!lastColorImage.get())
        {
            lastColorImage.reset(ROSImgToColorImg<ColorData>(img));
        }

        ROSImgToColorImg(img, lastColorImage.get());

        colorCamera.lastImageTimestamp = img->header.stamp;
        colorCamera.gotImage = true;
    }

    void ChiselServer::SetColorPose(const Eigen::Affine3f& tf)
    {
        colorCamera.lastPose = tf;
        colorCamera.gotPose  = true;
    }

    void ChiselServer::SetDepthImage(const sensor_msgs::ImageConstPtr& img)
    {
        if (!lastDepthImage.get())
        {
          lastDepthImage.reset(new chisel::DepthImage<DepthData>(img->width, img->height));
        }

        ROSImgToDepthImg(img, lastDepthImage.get());
        depthCamera.lastImageTimestamp = img->header.stamp;
        depthCamera.gotImage = true;
    }

    void ChiselServer::DepthImageCallback(sensor_msgs::ImageConstPtr depthImage)
    {
        if (IsPaused()) return;
        SetDepthImage(depthImage);

        bool gotTransform = false;
        tf::StampedTransform tf;

        int tries = 0;
        int maxTries = 10;

        while(!gotTransform && tries < maxTries)
        {
            tries++;
            try
            {
                transformListener.waitForTransform(depthCamera.transform, baseTransform, depthImage->header.stamp, ros::Duration(0.5));
                transformListener.lookupTransform(depthCamera.transform, baseTransform, depthImage->header.stamp, tf);
                depthCamera.gotPose = true;
                gotTransform = true;
            }
            catch (std::exception& e)
            {
                ros::Rate lookupRate(0.5f);
                ROS_WARN("%s\n", e.what());
            }
        }

        depthCamera.lastPose = RosTfToChiselTf(tf);

        hasNewData = true;
    }

    void ChiselServer::SetColorCameraInfo(const sensor_msgs::CameraInfoConstPtr& info)
    {
        colorCamera.cameraModel = RosCameraToChiselCamera(info);
        colorCamera.cameraModel.SetNearPlane(GetNearPlaneDist());
        colorCamera.cameraModel.SetFarPlane(GetFarPlaneDist());
        colorCamera.gotInfo = true;
    }

    void ChiselServer::SetDepthCameraInfo(const sensor_msgs::CameraInfoConstPtr& info)
    {
        depthCamera.cameraModel = RosCameraToChiselCamera(info);
        depthCamera.cameraModel.SetNearPlane(GetNearPlaneDist());
        depthCamera.cameraModel.SetFarPlane(GetFarPlaneDist());
        depthCamera.gotInfo = true;
    }

    void ChiselServer::SubscribeColorImage(const std::string& imageTopic, const std::string& infoTopic, const std::string& transform)
    {
        colorCamera.imageTopic = imageTopic;
        colorCamera.transform = transform;
        colorCamera.infoTopic = infoTopic;
        colorCamera.imageSubscriber = nh.subscribe(colorCamera.imageTopic, 20, &ChiselServer::ColorImageCallback, this);
        colorCamera.infoSubscriber = nh.subscribe(colorCamera.infoTopic, 20, &ChiselServer::ColorCameraInfoCallback, this);
    }

    void ChiselServer::ColorCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo)
    {
        if (IsPaused()) return;
        SetColorCameraInfo(cameraInfo);
    }

    void ChiselServer::ColorImageCallback(sensor_msgs::ImageConstPtr colorImage)
    {
        if (IsPaused()) return;
        SetColorImage(colorImage);

        bool gotTransform = false;
        tf::StampedTransform tf;

        int tries = 0;
        int maxTries = 10;

        while(!gotTransform && tries < maxTries)
        {
            tries++;
            try
            {
                transformListener.waitForTransform(colorCamera.transform, baseTransform, colorImage->header.stamp, ros::Duration(0.5));
                transformListener.lookupTransform(colorCamera.transform, baseTransform, colorImage->header.stamp, tf);
                colorCamera.gotPose = true;
                gotTransform = true;
            }
            catch (std::exception& e)
            {
                ros::Rate lookupRate(0.5f);
                ROS_WARN("%s\n", e.what());
            }
        }

        colorCamera.lastPose = RosTfToChiselTf(tf);
    }

    void ChiselServer::SubscribePointCloud(const std::string& topic)
    {
        pointcloudTopic.cloudTopic = topic;
        pointcloudTopic.gotCloud = false;
        pointcloudTopic.gotPose = false;
        pointcloudTopic.lastPose = chisel::Transform::Identity();
        pointcloudTopic.lastTimestamp = ros::Time::now();
        pointcloudTopic.cloudSubscriber = nh.subscribe(pointcloudTopic.cloudTopic, 20, &ChiselServer::PointCloudCallback, this);
    }

    void ChiselServer::PointCloudCallback(sensor_msgs::PointCloud2ConstPtr pointcloud)
    {
        ROS_INFO("Got point cloud...");
        if (IsPaused()) return;
        if (!lastPointCloud.get())
        {
            lastPointCloud.reset(new chisel::PointCloud());
        }

        if (useColor)
        {
            ROSPointCloudToChisel<pcl::PointXYZRGB>(pointcloud, lastPointCloud.get(), true);
        }
        else
        {
            ROSPointCloudToChisel<pcl::PointXYZ>(pointcloud, lastPointCloud.get(), false);
        }
        pointcloudTopic.transform = pointcloud->header.frame_id;
        bool gotTransform = false;
        tf::StampedTransform tf;

        int tries = 0;
        int maxTries = 10;

        ROS_INFO("Waiting for transform...");
        while(!gotTransform && tries < maxTries)
        {
            tries++;
            try
            {
                transformListener.waitForTransform(pointcloudTopic.transform, baseTransform, pointcloud->header.stamp, ros::Duration(0.5));
                transformListener.lookupTransform(pointcloudTopic.transform, baseTransform, pointcloud->header.stamp, tf);
                pointcloudTopic.gotPose = true;
                gotTransform = true;
            }
            catch (std::exception& e)
            {
                ros::Rate lookupRate(0.5f);
                ROS_WARN("%s\n", e.what());
            }
        }

        if (gotTransform)
        {
            pointcloudTopic.lastPose = RosTfToChiselTf(tf);
            pointcloudTopic.lastTimestamp = pointcloud->header.stamp;
            hasNewData = true;
        }
    }

    void ChiselServer::SetupProjectionIntegrator(const chisel::Vec4& truncation, uint16_t weight, bool useCarving, float carvingDist)
    {
        projectionIntegrator.SetCentroids(GetChiselMap()->GetChunkManager().GetCentroids());
        projectionIntegrator.SetTruncator(chisel::TruncatorPtr(new chisel::QuadraticTruncator(truncation(0), truncation(1), truncation(2), truncation(3))));
        projectionIntegrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(weight)));
        projectionIntegrator.SetCarvingDist(carvingDist);
        projectionIntegrator.SetCarvingEnabled(useCarving);
    }

    void ChiselServer::IntegrateLastDepthImage()
    {
        if (!IsPaused() && depthCamera.gotInfo && depthCamera.gotPose && lastDepthImage.get())
        {
            if(useColor)
            {
                chiselMap->IntegrateDepthScanColor<DepthData, ColorData>(projectionIntegrator,  lastDepthImage, depthCamera.lastPose, depthCamera.cameraModel, lastColorImage, colorCamera.lastPose, colorCamera.cameraModel);
            }
            else
            {
                printf("CHISEL: Integrating depth scan\n");
                chiselMap->IntegrateDepthScan<DepthData>(projectionIntegrator, lastDepthImage, depthCamera.lastPose, depthCamera.cameraModel);
            }
            printf("CHISEL: Done with scan\n");
            PublishLatestChunkBoxes();
            PublishDepthFrustum();

            chiselMap->UpdateMeshes();
            hasNewData = false;
        }
    }

    void ChiselServer::IntegrateLastPointCloud()
    {
        if (!IsPaused()  && pointcloudTopic.gotPose && lastPointCloud.get())
        {
            ROS_INFO("Integrating point cloud");
            chiselMap->IntegratePointCloud(projectionIntegrator, *lastPointCloud, pointcloudTopic.lastPose, farPlaneDist);
            PublishLatestChunkBoxes();
            chiselMap->UpdateMeshes();
            hasNewData = false;
        }
    }


    void ChiselServer::PublishLatestChunkBoxes()
    {
        if (!latestChunkPublisher) return;
        const chisel::ChunkManager& chunkManager = chiselMap->GetChunkManager();
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = baseTransform;
        marker.ns = "chunk_box";
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.scale.x = chunkManager.GetChunkSize()(0) * chunkManager.GetResolution();
        marker.scale.y = chunkManager.GetChunkSize()(1) * chunkManager.GetResolution();
        marker.scale.z = chunkManager.GetChunkSize()(2) * chunkManager.GetResolution();
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 0.3f;
        marker.color.g = 0.95f;
        marker.color.b = 0.3f;
        marker.color.a = 0.6f;
        const chisel::ChunkSet& latest = chiselMap->GetMeshesToUpdate();
        for (const std::pair<chisel::ChunkID, bool>& id : latest)
        {
            if(chunkManager.HasChunk(id.first))
            {
                chisel::AABB aabb = chunkManager.GetChunk(id.first)->ComputeBoundingBox();
                chisel::Vec3 center = aabb.GetCenter();
                geometry_msgs::Point pt;
                pt.x = center.x();
                pt.y = center.y();
                pt.z = center.z();
                marker.points.push_back(pt);
            }
        }

        latestChunkPublisher.publish(marker);
    }

    void ChiselServer::PublishChunkBoxes()
    {
        const chisel::ChunkManager& chunkManager = chiselMap->GetChunkManager();
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = baseTransform;
        marker.ns = "chunk_box";
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.scale.x = chunkManager.GetChunkSize()(0) * chunkManager.GetResolution();
        marker.scale.y = chunkManager.GetChunkSize()(1) * chunkManager.GetResolution();
        marker.scale.z = chunkManager.GetChunkSize()(2) * chunkManager.GetResolution();
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 0.95f;
        marker.color.g = 0.3f;
        marker.color.b = 0.3f;
        marker.color.a = 0.6f;
        for (const std::pair<chisel::ChunkID, chisel::ChunkPtr>& pair : chunkManager.GetChunks())
        {
            chisel::AABB aabb = pair.second->ComputeBoundingBox();
            chisel::Vec3 center = aabb.GetCenter();
            geometry_msgs::Point pt;
            pt.x = center.x();
            pt.y = center.y();
            pt.z = center.z();
            marker.points.push_back(pt);
        }

        chunkBoxPublisher.publish(marker);
    }

    chisel::Vec3 LAMBERT(const chisel::Vec3& n, const chisel::Vec3& light)
    {
        return fmax(n.dot(light), 0.0f) * chisel::Vec3(0.5, 0.5, 0.5);
    }

    void ChiselServer::FillMarkerTopicWithMeshes(visualization_msgs::Marker* marker)
    {
        assert(marker != nullptr);
        marker->header.stamp = ros::Time::now();
        marker->header.frame_id = baseTransform;
        marker->scale.x = 1;
        marker->scale.y = 1;
        marker->scale.z = 1;
        marker->pose.orientation.x = 0;
        marker->pose.orientation.y = 0;
        marker->pose.orientation.z = 0;
        marker->pose.orientation.w = 1;
        marker->type = visualization_msgs::Marker::TRIANGLE_LIST;
        const chisel::MeshMap& meshMap = chiselMap->GetChunkManager().GetAllMeshes();

        if(meshMap.size() == 0)
        {
            return;
        }

        chisel::Vec3 lightDir(0.8f, -0.2f, 0.7f);
        lightDir.normalize();
        chisel::Vec3 lightDir1(-0.5f, 0.2f, 0.2f);
        lightDir.normalize();
        const chisel::Vec3 ambient(0.2f, 0.2f, 0.2f);
        //int idx = 0;
        for (const std::pair<chisel::ChunkID, chisel::MeshPtr>& meshes : meshMap)
        {
            const chisel::MeshPtr& mesh = meshes.second;
            for (size_t i = 0; i < mesh->vertices.size(); i++)
            {
                const chisel::Vec3& vec = mesh->vertices[i];
                geometry_msgs::Point pt;
                pt.x = vec[0];
                pt.y = vec[1];
                pt.z = vec[2];
                marker->points.push_back(pt);

                if(mesh->HasColors())
                {
                    const chisel::Vec3& meshCol = mesh->colors[i];
                    std_msgs::ColorRGBA color;
                    color.r = meshCol[0];
                    color.g = meshCol[1];
                    color.b = meshCol[2];
                    color.a = 1.0;
                    marker->colors.push_back(color);
                }
                else
                {
                  if(mesh->HasNormals())
                  {
                      const chisel::Vec3 normal = mesh->normals[i];
                      std_msgs::ColorRGBA color;
                      chisel::Vec3 lambert = LAMBERT(normal, lightDir) + LAMBERT(normal, lightDir1) + ambient;
                      color.r = fmin(lambert[0], 1.0);
                      color.g = fmin(lambert[1], 1.0);
                      color.b = fmin(lambert[2], 1.0);
                      color.a = 1.0;
                      marker->colors.push_back(color);
                  }
                  else
                  {
                    std_msgs::ColorRGBA color;
                    color.r = vec[0] * 0.25 + 0.5;
                    color.g = vec[1] * 0.25 + 0.5;
                    color.b = vec[2] * 0.25 + 0.5;
                    color.a = 1.0;
                    marker->colors.push_back(color);
                  }
                }
                //marker->indicies.push_back(idx);
                //idx++;
            }
        }
    }

    bool ChiselServer::SaveMesh(chisel_msgs::SaveMeshService::Request& request, chisel_msgs::SaveMeshService::Response& response)
    {
        bool saveSuccess = chiselMap->SaveAllMeshesToPLY(request.file_name);
        return saveSuccess;
    }

    bool ChiselServer::GetAllChunks(chisel_msgs::GetAllChunksService::Request& request, chisel_msgs::GetAllChunksService::Response& response)
    {
        const chisel::ChunkMap& chunkmap = chiselMap->GetChunkManager().GetChunks();
        response.chunks.chunks.resize(chunkmap.size());
        response.chunks.header.stamp = ros::Time::now();
        size_t i = 0;
        for (const std::pair<chisel::ChunkID, chisel::ChunkPtr>& chunkPair : chiselMap->GetChunkManager().GetChunks())
        {
            chisel_msgs::ChunkMessage& msg = response.chunks.chunks.at(i);
            FillChunkMessage(chunkPair.second, &msg);
            i++;
        }

        return true;
    }

} // namespace chisel 
