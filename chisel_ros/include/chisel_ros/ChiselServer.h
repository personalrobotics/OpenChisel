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

#ifndef CHISELSERVER_H_
#define CHISELSERVER_H_

#include <chisel_ros/ResetService.h>
#include <chisel_ros/PauseService.h>
#include <chisel_ros/SaveMeshService.h>
#include <chisel_ros/GetAllChunksService.h>

#include <memory>
#include <open_chisel/Chisel.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/camera/ColorImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace chisel_ros
{

    typedef float DepthData;
    typedef uint8_t ColorData;

    class ChiselServer
    {
        public:
            struct RosCameraTopic
            {
                std::string imageTopic;
                std::string infoTopic;
                std::string transform;
                chisel::PinholeCamera cameraModel;
                ros::Subscriber imageSubscriber;
                ros::Subscriber infoSubscriber;
                ros::Publisher lastPosePublisher;
                ros::Publisher frustumPublisher;
                chisel::Transform lastPose;
                ros::Time lastImageTimestamp;
                bool gotPose;
                bool gotInfo;
                bool gotImage;
            };

            ChiselServer();
            ChiselServer(const ros::NodeHandle& nodeHanlde, int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color);
            virtual ~ChiselServer();

            inline chisel::ChiselPtr GetChiselMap() { return chiselMap; }
            inline void SetChiselMap(const chisel::ChiselPtr value) { chiselMap = value; }

            inline const std::string& GetBaseTransform() const { return baseTransform; }
            inline const std::string& GetMeshTopic() const { return meshTopic; }

            void SetupProjectionIntegrator(const chisel::Vec4& truncation, uint16_t weight, bool useCarving, float carvingDist);
            void SetupMeshPublisher(const std::string& meshTopic);
            void SetupChunkBoxPublisher(const std::string& boxTopic);
            void SetupDepthPosePublisher(const std::string& depthPoseTopic);
            void SetupColorPosePublisher(const std::string& colorPoseTopic);
            void SetupDepthFrustumPublisher(const std::string& frustumTopic);
            void SetupColorFrustumPublisher(const std::string& frustumTopic);

            void PublishMeshes();
            void PublishChunkBoxes();
            void PublishDepthPose();
            void PublishColorPose();
            void PublishDepthFrustum();
            void PublishColorFrustum();

            void SubscribeDepthImage(const std::string& depthImageTopic, const std::string& cameraInfoTopic, const std::string& transform);
            void DepthCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo);
            void DepthImageCallback(sensor_msgs::ImageConstPtr depthImage);

            void SubscribeColorImage(const std::string& colorImageTopic, const std::string& cameraInfoTopic, const std::string& transform);
            void ColorCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo);
            void ColorImageCallback(sensor_msgs::ImageConstPtr colorImage);

            void IntegrateLastDepthImage();
            void FillMarkerTopicWithMeshes(visualization_msgs::Marker* marker);
            inline void SetBaseTransform(const std::string& frameName) { baseTransform = frameName; }

            inline bool HasNewData() { return hasNewData; }

            inline float GetNearPlaneDist() const { return nearPlaneDist; }
            inline float GetFarPlaneDist() const { return farPlaneDist; }
            inline void SetNearPlaneDist(float dist) { nearPlaneDist = dist; }
            inline void SetFarPlaneDist(float dist) { farPlaneDist = dist; }

            bool Reset(chisel_ros::ResetService::Request& request, chisel_ros::ResetService::Response& response);
            bool TogglePaused(chisel_ros::PauseService::Request& request, chisel_ros::PauseService::Response& response);
            bool SaveMesh(chisel_ros::SaveMeshService::Request& request, chisel_ros::SaveMeshService::Response& response);
            bool GetAllChunks(chisel_ros::GetAllChunksService::Request& request, chisel_ros::GetAllChunksService::Response& response);

            inline bool IsPaused() { return isPaused; }
            inline void SetPaused(bool paused) { isPaused = paused; }

            void AdvertiseServices();

        protected:
            visualization_msgs::Marker CreateFrustumMarker(const chisel::Frustum& frustum);

            ros::NodeHandle nh;
            chisel::ChiselPtr chiselMap;
            tf::TransformListener transformListener;
            std::shared_ptr<chisel::DepthImage<DepthData> > lastDepthImage;
            std::shared_ptr<chisel::ColorImage<ColorData> > lastColorImage;
            chisel::ProjectionIntegrator projectionIntegrator;
            std::string baseTransform;
            std::string meshTopic;
            std::string chunkBoxTopic;
            ros::Publisher meshPublisher;
            ros::Publisher chunkBoxPublisher;
            ros::ServiceServer resetServer;
            ros::ServiceServer pauseServer;
            ros::ServiceServer saveMeshServer;
            ros::ServiceServer getAllChunksServer;
            RosCameraTopic depthCamera;
            RosCameraTopic colorCamera;
            bool useColor;
            bool hasNewData;
            float nearPlaneDist;
            float farPlaneDist;
            bool isPaused;
    };
    typedef std::shared_ptr<ChiselServer> ChiselServerPtr;
    typedef std::shared_ptr<const ChiselServer> ChiselServerConstPtr;

} // namespace chisel 

#endif // CHISELSERVER_H_ 
