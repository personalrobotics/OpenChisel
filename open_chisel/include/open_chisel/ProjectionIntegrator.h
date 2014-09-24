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
#include <open_chisel/Chunk.h>

#include <open_chisel/truncation/Truncator.h>
#include <open_chisel/weighting/Weighter.h>

namespace chisel
{

    class ProjectionIntegrator
    {
        public:
            ProjectionIntegrator();
            ProjectionIntegrator(const Truncator& t, const Weighter& w, float carvingDist, bool enableCarving);
            virtual ~ProjectionIntegrator();

            template <class DataType> bool Integrate(const std::shared_ptr<const DepthImage<DataType> >& depthImage, const PinholeCamera& camera, const Transform& cameraPose, Chunk* chunk)
            {
                    assert(chunk != nullptr);

                    Eigen::Vector3i numVoxels = chunk->numVoxels;
                    float resolution = chunk->voxelResolutionMeters;
                    float halfResolution = 0.5f * resolution;
                    Transform inversePose = cameraPose.inverse();
                    Vec3 voxelCenter;
                    bool updated = false;
                    for (int x = 0; x < numVoxels(0); x++)
                    {
                        voxelCenter(0) = x * resolution + halfResolution;
                        for (int y = 0; y < numVoxels(1); y++)
                        {
                            voxelCenter(1) = y * resolution + halfResolution;
                            for (int z = 0; z < numVoxels(2); z++)
                            {
                                voxelCenter(2) = z * resolution + halfResolution;
                                Vec3 voxelCenterInCamera = inversePose * voxelCenter;
                                Vec3 cameraPos = camera.ProjectPoint(voxelCenterInCamera);

                                if(!camera.IsPointOnImage(cameraPos)) continue;

                                float voxelDist = cameraPos.z();
                                float depth = depthImage->DepthAt((int)cameraPos(0), (int)cameraPos(1));
                                float truncation = truncator.GetTruncationDistance(depth);
                                float surfaceDist = depth - voxelDist;


                                if (std::abs(surfaceDist) < truncation)
                                {
                                    DistVoxel& voxel = chunk->GetDistVoxel(x, y, z);
                                    voxel.Integrate(surfaceDist, weighter.GetWeight(surfaceDist));
                                    updated = true;
                                }
                                else if(enableVoxelCarving && surfaceDist > truncation + carvingDist)
                                {
                                    DistVoxel& voxel = chunk->GetDistVoxel(x, y, z);
                                    if (voxel.GetWeight() > 0)
                                    {
                                        voxel.Reset();
                                        updated = true;
                                    }
                                }
                            }
                        }
                    }
            }

            inline const Truncator& GetTruncator() { return truncator; }
            inline void SetTruncator(const Truncator& value) { truncator = value; }
            inline const Weighter& GetWeighter() { return weighter; }
            inline void SetWeighter(const Weighter& value) { weighter = value; }

        protected:
            Truncator truncator;
            Weighter weighter;
            float carvingDist;
            bool enableVoxelCarving;
    };

} // namespace chisel 

#endif // PROJECTIONINTEGRATOR_H_ 
