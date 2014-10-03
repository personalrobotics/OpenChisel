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

#ifndef PROJECTIONINTEGRATOR_H_
#define PROJECTIONINTEGRATOR_H_

#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/geometry/AABB.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/camera/ColorImage.h>
#include <open_chisel/Chunk.h>

#include <open_chisel/truncation/Truncator.h>
#include <open_chisel/weighting/Weighter.h>

namespace chisel
{

    class ProjectionIntegrator
    {
        public:
            ProjectionIntegrator();
            ProjectionIntegrator(const TruncatorPtr& t, const WeighterPtr& w, float carvingDist, bool enableCarving);
            virtual ~ProjectionIntegrator();

            template<class DataType> bool Integrate(const std::shared_ptr<const DepthImage<DataType> >& depthImage, const PinholeCamera& camera, const Transform& cameraPose, Chunk* chunk) const
            {
                assert(chunk != nullptr);

                Eigen::Vector3i numVoxels = chunk->GetNumVoxels();
                float resolution = chunk->GetVoxelResolutionMeters();
                Vec3 origin = chunk->GetOrigin();
                float halfResolution = 0.5f * resolution;
                Vec3 voxelCenter;
                bool updated = false;
                for (int x = 0; x < numVoxels(0); x++)
                {
                    voxelCenter(0) = x * resolution + halfResolution + origin.x();
                    for (int y = 0; y < numVoxels(1); y++)
                    {
                        voxelCenter(1) = y * resolution + halfResolution + origin.y();
                        for (int z = 0; z < numVoxels(2); z++)
                        {
                            voxelCenter(2) = z * resolution + halfResolution + origin.z();
                            Vec3 voxelCenterInCamera = cameraPose.linear().transpose() * (voxelCenter - cameraPose.translation());
                            Vec3 cameraPos = camera.ProjectPoint(voxelCenterInCamera);

                            if (!camera.IsPointOnImage(cameraPos) || voxelCenterInCamera.z() < 0)
                                continue;

                            float voxelDist = fabs(voxelCenterInCamera.z());
                            float depth = depthImage->BilinearInterpolateDepth(cameraPos(0), cameraPos(1));

                            if(std::isnan(depth))
                            {
                                continue;
                            }

                            float truncation = truncator->GetTruncationDistance(depth);
                            float surfaceDist = depth - voxelDist;

                            if (std::abs(surfaceDist) < truncation)
                            {
                                DistVoxel& voxel = chunk->GetDistVoxelMutable(x, y, z);
                                voxel.Integrate(surfaceDist, weighter->GetWeight(surfaceDist));
                                updated = true;
                            }
                            else if (enableVoxelCarving && surfaceDist > truncation + carvingDist)
                            {
                                DistVoxel& voxel = chunk->GetDistVoxelMutable(x, y, z);
                                if (voxel.GetWeight() > 0)
                                {
                                    float sdf = voxel.GetSDF();

                                    if(sdf <= 0)
                                    {
                                        voxel.Integrate(0.001f, 1);
                                        updated = true;
                                    }
                                }
                            }
                        }
                    }
                }
                return updated;
            }
           template<class DataType, class ColorType> bool IntegrateColor(const std::shared_ptr<const DepthImage<DataType> >& depthImage, const PinholeCamera& depthCamera, const Transform& depthCameraPose, const std::shared_ptr<const ColorImage<ColorType, 3> >& colorImage, const PinholeCamera& colorCamera, const Transform& colorCameraPose, Chunk* chunk) const
           {
               assert(chunk != nullptr);

               Eigen::Vector3i numVoxels = chunk->GetNumVoxels();
               float resolution = chunk->GetVoxelResolutionMeters();
               Vec3 origin = chunk->GetOrigin();
               float halfResolution = 0.5f * resolution;
               Vec3 voxelCenter;
               bool updated = false;
               for (int x = 0; x < numVoxels(0); x++)
               {
                   voxelCenter(0) = x * resolution + halfResolution + origin.x();
                   for (int y = 0; y < numVoxels(1); y++)
                   {
                       voxelCenter(1) = y * resolution + halfResolution + origin.y();
                       for (int z = 0; z < numVoxels(2); z++)
                       {
                           voxelCenter(2) = z * resolution + halfResolution + origin.z();
                           Vec3 voxelCenterInCamera = depthCameraPose.linear().transpose() * (voxelCenter - depthCameraPose.translation());
                           Vec3 cameraPos = depthCamera.ProjectPoint(voxelCenterInCamera);

                           if (!depthCamera.IsPointOnImage(cameraPos) || voxelCenterInCamera.z() < 0)
                           {
                               //printf("%f %f is not on image...\n", cameraPos(0), cameraPos(1));
                               continue;
                           }

                           float voxelDist = fabs(voxelCenterInCamera.z());
                           float depth = depthImage->BilinearInterpolateDepth(cameraPos(0), cameraPos(1));

                           if(std::isnan(depth))
                           {
                               continue;
                           }

                           float truncation = truncator->GetTruncationDistance(depth);
                           float surfaceDist = depth - voxelDist;

                           if (std::abs(surfaceDist) < truncation)
                           {
                               Vec3 voxelCenterInColorCamera = colorCameraPose.linear().transpose() * (voxelCenter - colorCameraPose.translation());
                               Vec3 colorCameraPos = colorCamera.ProjectPoint(voxelCenterInColorCamera);
                               if(colorCamera.IsPointOnImage(colorCameraPos))
                               {
                                   ColorVoxel& colorVoxel = chunk->GetColorVoxelMutable(x, y, z);
                                   int r = static_cast<int>(colorCameraPos(1));
                                   int c = static_cast<int>(colorCameraPos(0));
                                   uint8_t red = colorImage->At(r, c, 2);
                                   uint8_t green = colorImage->At(r, c, 1);
                                   uint8_t blue = colorImage->At(r, c, 0);
                                   colorVoxel.Integrate(red, green, blue, static_cast<uint8_t>(weighter->GetWeight(surfaceDist)));
                               }

                               DistVoxel& voxel = chunk->GetDistVoxelMutable(x, y, z);
                               voxel.Integrate(surfaceDist, weighter->GetWeight(surfaceDist));

                               updated = true;
                           }
                           else if (enableVoxelCarving && surfaceDist > truncation + carvingDist)
                           {
                               DistVoxel& voxel = chunk->GetDistVoxelMutable(x, y, z);
                               if (voxel.GetWeight() > 0)
                               {
                                   float sdf = voxel.GetSDF();
                                   if(sdf <= 0)
                                   {
                                       voxel.Integrate(0.001f, 1);
                                       updated = true;
                                   }
                               }
                           }
                       }
                   }
               }
               return updated;
           }

            inline const TruncatorPtr& GetTruncator() { return truncator; }
            inline void SetTruncator(const TruncatorPtr& value) { truncator = value; }
            inline const WeighterPtr& GetWeighter() { return weighter; }
            inline void SetWeighter(const WeighterPtr& value) { weighter = value; }

            inline float GetCarvingDist() { return carvingDist; }
            inline bool IsCarvingEnabled() { return enableVoxelCarving; }
            inline void SetCarvingDist(float dist) { carvingDist = dist; }
            inline void SetCarvingEnabled(bool enabled) { enableVoxelCarving = enabled; }

        protected:
            TruncatorPtr truncator;
            WeighterPtr weighter;
            float carvingDist;
            bool enableVoxelCarving;
    };

} // namespace chisel 

#endif // PROJECTIONINTEGRATOR_H_ 
