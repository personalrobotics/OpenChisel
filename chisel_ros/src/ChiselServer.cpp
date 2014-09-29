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
#include <visualization_msgs/Marker.h>
#include <open_chisel/truncation/ConstantTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>

namespace chisel_ros
{

    ChiselServer::ChiselServer() :
            useColor(false)
    {
        // TODO Auto-generated constructor stub

    }

    ChiselServer::~ChiselServer()
    {
        // TODO Auto-generated destructor stub
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

    ChiselServer::ChiselServer(const ros::NodeHandle& nodeHanlde, int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color) :
            nh(nodeHanlde), useColor(color)
    {
        chiselMap.reset(new chisel::Chisel(Eigen::Vector3i(chunkSizeX, chunkSizeY, chunkSizeZ), resolution, color));
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
        depthCamera.cameraModel.SetNearPlane(0.05f);
        depthCamera.cameraModel.SetFarPlane(5);
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

        IntegrateLastDepthImage();
        PublishMeshes();
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
        colorCamera.cameraModel.SetNearPlane(0.05f);
        colorCamera.cameraModel.SetFarPlane(5);
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

    void ChiselServer::SetupProjectionIntegrator(float truncationDist, uint16_t weight, bool useCarving, float carvingDist)
    {
        projectionIntegrator.SetTruncator(chisel::TruncatorPtr(new chisel::ConstantTruncator(truncationDist)));
        projectionIntegrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(weight)));
        projectionIntegrator.SetCarvingDist(carvingDist);
        projectionIntegrator.SetCarvingEnabled(useCarving);
    }

    void ChiselServer::IntegrateLastDepthImage()
    {
        if (depthCamera.gotInfo && depthCamera.gotPose && lastDepthImage.get())
        {
            if(useColor)
            {
                chiselMap->IntegrateDepthScanColor<DepthData, ColorData>(projectionIntegrator,  lastDepthImage, depthCamera.lastPose, depthCamera.cameraModel, lastColorImage, colorCamera.lastPose, colorCamera.cameraModel);
            }
            else
            {
                chiselMap->IntegrateDepthScan<DepthData>(projectionIntegrator, lastDepthImage, depthCamera.lastPose, depthCamera.cameraModel);
            }
            chiselMap->UpdateMeshes();
        }
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
                //printf("%f %f %f\n", vec.x(), vec.y(), vec.z());
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

        if(!marker->points.empty())
            printf("Mesh has %lu vertices\n", marker->points.size());

    }

} // namespace chisel 
