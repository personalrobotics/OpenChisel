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

#include <open_chisel/threading/Threading.h>
#include <open_chisel/ChunkManager.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/pointcloud/PointCloud.h>

namespace chisel
{

    class Chisel
    {
        public:
            Chisel();
            Chisel(const Eigen::Vector3i& chunkSize, float voxelResolution, bool useColor);
            virtual ~Chisel();

            inline const ChunkManager& GetChunkManager() const { return chunkManager; }
            inline ChunkManager& GetMutableChunkManager() { return chunkManager; }
            inline void SetChunkManager(const ChunkManager& manager) { chunkManager = manager; }

            void IntegratePointCloud(const ProjectionIntegrator& integrator, const PointCloud& cloud, const Transform& extrinsic, float truncation, float maxDist);
            void IntegrateChunks(const ProjectionIntegrator& integrator, ChunkManager &sourceChunkManager);

            template <class DataType> void IntegrateDepthScan(const ProjectionIntegrator& integrator, const std::shared_ptr<const DepthImage<DataType> >& depthImage, const Transform& extrinsic, const PinholeCamera& camera)
            {
                clock_t begin = clock();
                printf("CHISEL: Integrating a scan\n");

                    Frustum frustum;
                    camera.SetupFrustum(extrinsic, &frustum);

                    ChunkIDList chunksIntersecting;
                    chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);

                    std::mutex mutex;
                    ChunkIDList garbageChunks;
                    //for(const ChunkID& chunkID : chunksIntersecting)
                    parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
                    {
                        bool chunkNew = false;

                        mutex.lock();
                        if (!chunkManager.HasChunk(chunkID))
                        {
                           chunkNew = true;
                           chunkManager.CreateChunk(chunkID);
                        }

                        ChunkPtr chunk = chunkManager.GetChunk(chunkID);
                        mutex.unlock();

                        bool needsUpdate = integrator.Integrate(depthImage, camera, extrinsic, chunk.get());

                        mutex.lock();
                        if (needsUpdate)
                        {
                            for (int dx = -1; dx <= 1; dx++)
                            {
                                for (int dy = -1; dy <= 1; dy++)
                                {
                                    for (int dz = -1; dz <= 1; dz++)
                                    {
                                        meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
                                    }
                                }
                            }
                        }
                        else if(chunkNew)
                        {
                            garbageChunks.push_back(chunkID);
                        }
                        mutex.unlock();
                    }
                    );
                    printf("CHISEL: Done with scan\n");
                    GarbageCollect(garbageChunks);
                    //chunkManager.PrintMemoryStatistics();

                    clock_t end = clock();
                    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
                    printf("\n \n  ~ %f HZ     \n \n", 1/elapsed_secs);
            }

            template <class DataType, class ColorType> void IntegrateDepthScanColor(const ProjectionIntegrator& integrator, const std::shared_ptr<const DepthImage<DataType> >& depthImage,  const Transform& depthExtrinsic, const PinholeCamera& depthCamera, const std::shared_ptr<const ColorImage<ColorType> >& colorImage, const Transform& colorExtrinsic, const PinholeCamera& colorCamera)
            {
                    clock_t begin = clock();

                    Frustum frustum;
                    depthCamera.SetupFrustum(depthExtrinsic, &frustum);

                    ChunkIDList chunksIntersecting;
                    chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);

                    std::mutex mutex;
                    ChunkIDList garbageChunks;
                    //for ( const ChunkID& chunkID : chunksIntersecting)
                    parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
                    {

                        mutex.lock();
                        bool chunkNew = false;
                        if (!chunkManager.HasChunk(chunkID))
                        {
                           chunkNew = true;
                           chunkManager.CreateChunk(chunkID);
                        }

                        ChunkPtr chunk = chunkManager.GetChunk(chunkID);
                        mutex.unlock();


                        bool needsUpdate = integrator.IntegrateColor(depthImage, depthCamera, depthExtrinsic, colorImage, colorCamera, colorExtrinsic, chunk.get());

                        mutex.lock();
                        if (needsUpdate)
                        {
                            for (int dx = -1; dx <= 1; dx++)
                            {
                                for (int dy = -1; dy <= 1; dy++)
                                {
                                    for (int dz = -1; dz <= 1; dz++)
                                    {
                                        meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
                                    }
                                }
                            }
                        }
                        else if(chunkNew)
                        {
                            garbageChunks.push_back(chunkID);
                        }
                        mutex.unlock();
                    }
                    );

                    GarbageCollect(garbageChunks);
                    clock_t end = clock();
                    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
                    printf("\n \n  ~ %f HZ     \n \n", 1/elapsed_secs);
                    //chunkManager.PrintMemoryStatistics();
            }

            void GarbageCollect(const ChunkIDList& chunks);
            void UpdateMeshes();

            bool SaveAllMeshesToPLY(const std::string& filename);
            void Reset();

            const ChunkSet& GetMeshesToUpdate() const { return meshesToUpdate; }
            const ChunkSet& GetLatestChunks() const {return latestChunks;}

        protected:
            ChunkManager chunkManager;
            ChunkSet meshesToUpdate;
            ChunkSet latestChunks;
        private:
            Point3 getVoxelCoordinates(VoxelID id, Eigen::Vector3i chunkSize);
            bool interpolateDistVoxel(const Vec3& voxelPos, ChunkManager& sourceChunkManager, DistVoxel* voxel);


    };
    typedef std::shared_ptr<Chisel> ChiselPtr;
    typedef std::shared_ptr<const Chisel> ChiselConstPtr;

} // namespace chisel 

#endif // CHISEL_H_ 
