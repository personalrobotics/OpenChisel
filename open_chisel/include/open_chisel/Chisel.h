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

#ifndef CHISEL_H_
#define CHISEL_H_

#include <open_chisel/ChunkManager.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/geometry/Frustum.h>

namespace chisel
{

    class Chisel
    {
        public:
            Chisel();
            Chisel(const Eigen::Vector3i& chunkSize, float voxelResolution);
            virtual ~Chisel();

            inline const ChunkManager& GetChunkManager() const { return chunkManager; }
            inline ChunkManager& GetMutableChunkManager() { return chunkManager; }
            inline void SetChunkManager(const ChunkManager& manager) { chunkManager = manager; }

            template <class DataType> void IntegrateDepthScan(const ProjectionIntegrator& integrator, const std::shared_ptr<const DepthImage<DataType> >& depthImage, const Transform& extrinsic, const PinholeCamera& camera)
            {
                    Frustum frustum;
                    camera.SetupFrustum(extrinsic, &frustum);

                    ChunkIDList chunksIntersecting;
                    chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);

                    ChunkIDList garbageChunks;
                    for (const ChunkID& chunkID : chunksIntersecting)
                    {
                        if (!chunkManager.HasChunk(chunkID))
                        {
                           chunkManager.CreateChunk(chunkID);
                        }

                        ChunkPtr chunk = chunkManager.GetChunk(chunkID);
                        bool needsUpdate = integrator.Integrate(depthImage, camera, extrinsic, chunk.get());

                        if (needsUpdate)
                        {
                            meshesToUpdate.push_back(chunkID);
                        }
                        else
                        {
                            garbageChunks.push_back(chunkID);
                        }
                    }

                    GarbageCollect(garbageChunks);
            }

            void GarbageCollect(const ChunkIDList& chunks);
            void UpdateMeshes();

        protected:
            ChunkManager chunkManager;
            ChunkIDList meshesToUpdate;

    };

} // namespace chisel 

#endif // CHISEL_H_ 
