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

#ifndef CHUNKMANAGER_H_
#define CHUNKMANAGER_H_

#include <memory>
#include <unordered_map>
#include <mutex>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/mesh/Mesh.h>
#include <open_chisel/ColorVoxel.h>
#include <open_chisel/DistVoxel.h>
#include <open_chisel/pointcloud/PointCloud.h>

#include "Chunk.h"

namespace chisel
{
    // Spatial hashing function from Matthias Teschner
    // Optimized Spatial Hashing for Collision Detection of Deformable Objects
    struct ChunkHasher
    {
            // Three large primes are used for spatial hashing.
            static constexpr size_t p1 = 73856093;
            static constexpr size_t p2 = 19349663;
            static constexpr size_t p3 = 8349279;

            std::size_t operator()(const ChunkID& key) const
            {
                return ( key(0) * p1 ^ key(1) * p2 ^ key(2) * p3);
            }
    };

    typedef std::unordered_map<ChunkID, ChunkPtr, ChunkHasher> ChunkMap;
    typedef std::unordered_map<ChunkID, bool, ChunkHasher> ChunkSet;
    typedef std::unordered_map<ChunkID, MeshPtr, ChunkHasher> MeshMap;
    class Frustum;
    class AABB;
    class ChunkManager
    {
        public:
            ChunkManager();
            ChunkManager(const Eigen::Vector3i& chunkSize, float voxelResolution, bool color);
            virtual ~ChunkManager();

            inline const ChunkMap& GetChunks() const { return chunks; }
            inline ChunkMap& GetMutableChunks() { return chunks; }

            inline bool HasChunk(const ChunkID& chunk) const
            {
                return chunks.find(chunk) != chunks.end();
            }

            inline ChunkPtr GetChunk(const ChunkID& chunk) const
            {
                return chunks.at(chunk);
            }

            inline void AddChunk(const ChunkPtr& chunk)
            {
                chunks.insert(std::make_pair(chunk->GetID(), chunk));
            }

            inline bool RemoveChunk(const ChunkID& chunk)
            {
                if(HasChunk(chunk))
                {
                    chunks.erase(chunk);
                    return true;
                }

                return false;
            }

            inline bool RemoveChunk(const ChunkPtr& chunk)
            {
                return RemoveChunk(chunk->GetID());
            }

            inline bool HasChunk(int x, int y, int z) const { return HasChunk(ChunkID(x, y, z)); }
            inline ChunkPtr GetChunk(int x, int y, int z) const { return GetChunk(ChunkID(x, y, z)); }

            inline ChunkPtr GetChunkAt(const Vec3& pos)
            {
                ChunkID id = GetIDAt(pos);

                if (HasChunk(id))
                {
                    return GetChunk(id);
                }

                return ChunkPtr();
            }

            inline ChunkID GetIDAt(const Vec3& pos) const
            {
                static const float roundingFactorX = 1.0f / (chunkSize(0) * voxelResolutionMeters);
                static const float roundingFactorY = 1.0f / (chunkSize(1) * voxelResolutionMeters);
                static const float roundingFactorZ = 1.0f / (chunkSize(2) * voxelResolutionMeters);

                return ChunkID(static_cast<int>(std::floor(pos(0) * roundingFactorX)),
                               static_cast<int>(std::floor(pos(1) * roundingFactorY)),
                               static_cast<int>(std::floor(pos(2) * roundingFactorZ)));
            }

            const DistVoxel* GetDistanceVoxel(const Vec3& pos);
            DistVoxel* GetDistanceVoxelMutable(const Vec3& pos);
            const ColorVoxel* GetColorVoxel(const Vec3& pos);

            void GetChunkIDsIntersecting(const AABB& box, ChunkIDList* chunkList);
            void GetChunkIDsIntersecting(const Frustum& frustum, ChunkIDList* chunkList);
            void GetChunkIDsIntersecting(const PointCloud& cloud, const Transform& cameraTransform, float truncation, float maxDist, ChunkIDList* chunkList);
            void CreateChunk(const ChunkID& id);

            void GenerateMesh(const ChunkPtr& chunk, Mesh* mesh);
            void ColorizeMesh(Mesh* mesh);
            Vec3 InterpolateColor(const Vec3& colorPos);

            void CacheCentroids();
            void ExtractBorderVoxelMesh(const ChunkPtr& chunk, const Eigen::Vector3i& index, const Eigen::Vector3f& coordinates, VertIndex* nextMeshIndex, Mesh* mesh);
            void ExtractInsideVoxelMesh(const ChunkPtr& chunk, const Eigen::Vector3i& index, const Vec3& coords, VertIndex* nextMeshIndex, Mesh* mesh);

            inline const MeshMap& GetAllMeshes() const { return allMeshes; }
            inline MeshMap& GetAllMutableMeshes() { return allMeshes; }
            inline const MeshPtr& GetMesh(const ChunkID& chunkID) const { return allMeshes.at(chunkID); }
            inline MeshPtr& GetMutableMesh(const ChunkID& chunkID) { return allMeshes.at(chunkID); }
            inline bool HasMesh(const ChunkID& chunkID) const { return allMeshes.find(chunkID) != allMeshes.end(); }

            inline bool GetUseColor() { return useColor; }

            void RecomptueMesh(const ChunkID& chunkID, std::mutex& mutex);
            void RecomputeMeshes(const ChunkSet& chunks);
            void ComputeNormalsFromGradients(Mesh* mesh);

            inline const Eigen::Vector3i& GetChunkSize() const { return chunkSize; }
            inline float GetResolution() const { return voxelResolutionMeters; }

            inline const Vec3List& GetCentroids() const { return centroids; }

            void PrintMemoryStatistics();

            void Reset();

            bool GetSDFAndGradient(const Eigen::Vector3f& pos, double* dist, Eigen::Vector3f* grad);
            bool GetSDF(const Eigen::Vector3f& pos, double* dist);

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        protected:
            ChunkMap chunks;
            Eigen::Vector3i chunkSize;
            float voxelResolutionMeters;
            Vec3List centroids;
            Eigen::Matrix<int, 3, 8> cubeIndexOffsets;
            MeshMap allMeshes;
            bool useColor;
    };

    typedef std::shared_ptr<ChunkManager> ChunkManagerPtr;
    typedef std::shared_ptr<const ChunkManager> ChunkManagerConstPtr;


} // namespace chisel 

#endif // CHUNKMANAGER_H_ 
