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
            useColor(false), hasNewData(false), nearPlaneDist(0.05), farPlaneDist(5)
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


    ChiselServer::ChiselServer(const ros::NodeHandle& nodeHanlde, int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color) :
            nh(nodeHanlde), useColor(color), hasNewData(false)
    {
        chiselMap.reset(new chisel::Chisel(Eigen::Vector3i(chunkSizeX, chunkSizeY, chunkSizeZ), resolution, color));
    }

    bool ChiselServer::TogglePaused(chisel_ros::PauseService::Request& request, chisel_ros::PauseService::Response& response)
    {
        SetPaused(!IsPaused());
        return true;
    }

    bool ChiselServer::Reset(chisel_ros::ResetService::Request& request, chisel_ros::ResetService::Response& response)
    {
        chiselMap->Reset();
        return true;
    }

    void ChiselServer::SubscribeDepthImage(const std::string& imageTopic, const std::string& infoTopic, const std::string& transform)
    {
        depthCamera.imageTopic = imageTopic;
        depthCamera.transform = transform;
        depthCamera.infoTopic = infoTopic;
        depthCamera.imageSubscriber = nh.subscribe(depthCamera.imageTopic, 1, &ChiselServer::DepthImageCallback, this);
        depthCamera.infoSubscriber = nh.subscribe(depthCamera.infoTopic, 1, &ChiselServer::DepthCameraInfoCallback, this);
    }


    void ChiselServer::DepthCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo)
    {
        depthCamera.cameraModel = RosCameraToChiselCamera(cameraInfo);
        depthCamera.cameraModel.SetNearPlane(GetNearPlaneDist());
        depthCamera.cameraModel.SetFarPlane(GetFarPlaneDist());
        depthCamera.gotInfo = true;
    }

    void ChiselServer::DepthImageCallback(sensor_msgs::ImageConstPtr depthImage)
    {
        if (!lastDepthImage.get())
        {
            lastDepthImage.reset(new chisel::DepthImage<DepthData>(depthImage->width, depthImage->height));
        }

        ROSImgToDepthImg(depthImage, lastDepthImage.get());

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
        depthCamera.lastImageTimestamp = depthImage->header.stamp;

        hasNewData = true;
    }

    void ChiselServer::SubscribeColorImage(const std::string& imageTopic, const std::string& infoTopic, const std::string& transform)
    {
        colorCamera.imageTopic = imageTopic;
        colorCamera.transform = transform;
        colorCamera.infoTopic = infoTopic;
        colorCamera.imageSubscriber = nh.subscribe(colorCamera.imageTopic, 1, &ChiselServer::ColorImageCallback, this);
        colorCamera.infoSubscriber = nh.subscribe(colorCamera.infoTopic, 1, &ChiselServer::ColorCameraInfoCallback, this);
    }

    void ChiselServer::ColorCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo)
    {
        colorCamera.cameraModel = RosCameraToChiselCamera(cameraInfo);
        colorCamera.cameraModel.SetNearPlane(GetNearPlaneDist());
        colorCamera.cameraModel.SetFarPlane(GetFarPlaneDist());
        colorCamera.gotInfo = true;
    }

    void ChiselServer::ColorImageCallback(sensor_msgs::ImageConstPtr colorImage)
    {
        if (!lastColorImage.get())
        {
            lastColorImage.reset(new chisel::ColorImage<ColorData>(colorImage->width, colorImage->height));
        }

        ROSImgToColorImg(colorImage, lastColorImage.get());

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
        colorCamera.lastImageTimestamp = colorImage->header.stamp;
    }

    void ChiselServer::SetupProjectionIntegrator(const chisel::Vec4& truncation, uint16_t weight, bool useCarving, float carvingDist)
    {
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
                printf("Integrating depth scan with color\n");
                chiselMap->IntegrateDepthScanColor<DepthData, ColorData>(projectionIntegrator,  lastDepthImage, depthCamera.lastPose, depthCamera.cameraModel, lastColorImage, colorCamera.lastPose, colorCamera.cameraModel);
            }
            else
            {
                printf("Integrating depth scan\n");
                chiselMap->IntegrateDepthScan<DepthData>(projectionIntegrator, lastDepthImage, depthCamera.lastPose, depthCamera.cameraModel);
            }
            chiselMap->UpdateMeshes();
            hasNewData = false;
        }
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

    void ChiselServer::FillMarkerTopicWithMeshes(visualization_msgs::Marker* marker)
    {
        assert(marker != nullptr);
        marker->header.stamp = ros::Time::now();
        marker->header.frame_id = baseTransform;
        marker->ns = "mesh";
        marker->type = visualization_msgs::Marker::TRIANGLE_LIST;
        marker->id = 0;
        marker->scale.x = 1;
        marker->scale.y = 1;
        marker->scale.z = 1;
        marker->pose.position.x = 0;
        marker->pose.position.y = 0;
        marker->pose.position.z = 0;
        marker->pose.orientation.x = 0.0;
        marker->pose.orientation.y = 0.0;
        marker->pose.orientation.z = 0.0;
        marker->pose.orientation.w = 1.0;
        marker->color.r = 1.0f;
        marker->color.g = 1.0f;
        marker->color.b = 1.0f;
        marker->color.a = 1.0f;


        const chisel::MeshMap& meshMap = chiselMap->GetChunkManager().GetAllMeshes();

        if(meshMap.size() == 0)
        {
            return;
        }

        chisel::Vec3 lightDir(0.2, 0.4, -1.0f);
        lightDir.normalize();
        const chisel::Vec3 ambient(0.4f, 0.4f, 0.5f);
        for (const std::pair<chisel::ChunkID, chisel::MeshPtr>& meshes : meshMap)
        {
            const chisel::MeshPtr& mesh = meshes.second;

            geometry_msgs::Point pt;
            std_msgs::ColorRGBA color;
            for (size_t i = 0; i < mesh->vertices.size(); i++)
            {
                const chisel::Vec3& vec = mesh->vertices[i];
                pt.x = vec.x();
                pt.y = vec.y();
                pt.z = vec.z();
                marker->points.push_back(pt);

                if(mesh->HasColors())
                {
                    const chisel::Vec3& meshCol = mesh->colors[i];
                    color.r = meshCol.x();
                    color.g = meshCol.y();
                    color.b = meshCol.z();
                    color.a = 1.0f;
                    marker->colors.push_back(color);
                }

                else if(mesh->HasNormals())
                {
                    chisel::Vec3 lambert(1.0f, 1.0f, 1.0f);
                    const chisel::Vec3 normal = mesh->normals[i];

                    lambert = fmax(normal.dot(lightDir), 0) * lambert + ambient;

                    color.r = fmax(fmin(0.5f * (normal.x() + 1.0f), 1.0f), 0);
                    color.g = fmax(fmin(0.5f * (normal.y() + 1.0f), 1.0f), 0);
                    color.b = fmax(fmin(0.5f * (normal.z() + 1.0f), 1.0f), 0);
                    color.a = 1.0f;
                    marker->colors.push_back(color);
                }

                else
                {
                    float zval = vec.z() - 5;
                    color.r = (zval) / 10.0f;
                    color.g = 10.0f / (fabs(zval) + 0.01f);
                    color.b = (zval * zval) / 10.0f;
                    color.a = 1.0f;
                    marker->colors.push_back(color);
                }
            }

        }

    }

    bool ChiselServer::SaveMesh(chisel_ros::SaveMeshService::Request& request, chisel_ros::SaveMeshService::Response& response)
    {
        bool saveSuccess = chiselMap->SaveAllMeshesToPLY(request.file_name);
        return saveSuccess;
    }

    bool ChiselServer::GetAllChunks(chisel_ros::GetAllChunksService::Request& request, chisel_ros::GetAllChunksService::Response& response)
    {
        const chisel::ChunkMap& chunkmap = chiselMap->GetChunkManager().GetChunks();
        response.chunks.chunks.resize(chunkmap.size());
        response.chunks.header.stamp = ros::Time::now();
        size_t i = 0;
        for (const std::pair<chisel::ChunkID, chisel::ChunkPtr>& chunkPair : chiselMap->GetChunkManager().GetChunks())
        {
            ChunkMessage& msg = response.chunks.chunks.at(i);
            FillChunkMessage(chunkPair.second, &msg);
            i++;
        }

        return true;
    }

} // namespace chisel 
