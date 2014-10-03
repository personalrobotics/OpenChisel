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


#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <Eigen/Geometry>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/camera/DepthImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

namespace chisel_ros
{
    template <class DataType> void ROSImgToDepthImg(sensor_msgs::ImageConstPtr image, chisel::DepthImage<DataType>* depthImage)
    {
            size_t dataSize =image->step / image->width;
            assert(depthImage->GetHeight() == static_cast<int>(image->height) && depthImage->GetWidth() == static_cast<int>(image->width));
            assert(dataSize == sizeof(DataType));
            const DataType* imageData = reinterpret_cast<const DataType*>(image->data.data());
            DataType* depthImageData = depthImage->GetMutableData();
            int totalPixels = image->width * image->height;
            for (int i = 0; i < totalPixels; i++)
            {
                depthImageData[i] =  imageData[i];
            }
    }

    template <class DataType> void ROSImgToColorImg(sensor_msgs::ImageConstPtr image, chisel::ColorImage<DataType>* colorImage)
    {
            size_t dataSize =image->step / image->width;
            assert(colorImage->GetHeight() == static_cast<int>(image->height) && colorImage->GetWidth() == static_cast<int>(image->width));
            assert(dataSize == 3 * sizeof(DataType));
            const DataType* imageData = reinterpret_cast<const DataType*>(image->data.data());
            DataType* colorImageData = colorImage->GetMutableData();
            int totalPixels = image->width * image->height * 3;
            for (int i = 0; i < totalPixels; i++)
            {
                colorImageData[i] =  imageData[i];
            }
    }

    inline chisel::Transform RosTfToChiselTf(const tf::StampedTransform& tf)
    {
        chisel::Transform transform;
        transform.translation()(0) = tf.getOrigin().x();
        transform.translation()(1) = tf.getOrigin().y();
        transform.translation()(2) = tf.getOrigin().z();


        chisel::Quaternion quat;
        quat.x() = tf.getRotation().x();
        quat.y() = tf.getRotation().y();
        quat.z() = tf.getRotation().z();
        quat.w() = tf.getRotation().w();
        transform.linear() = quat.toRotationMatrix();

        return transform.inverse();
    }

    inline chisel::PinholeCamera RosCameraToChiselCamera(const sensor_msgs::CameraInfoConstPtr& camera)
    {
        chisel::PinholeCamera cameraToReturn;
        chisel::Intrinsics intrinsics;
        intrinsics.SetFx(camera->P[0]);
        intrinsics.SetFy(camera->P[5]);
        intrinsics.SetCx(camera->P[2]);
        intrinsics.SetCy(camera->P[6]);
        cameraToReturn.SetIntrinsics(intrinsics);
        cameraToReturn.SetWidth(camera->width);
        cameraToReturn.SetHeight(camera->height);
        return cameraToReturn;
    }
}


#endif // CONVERSIONS_H_ 
