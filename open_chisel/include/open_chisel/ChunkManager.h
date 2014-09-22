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
    class Frustum;
    class AABB;
    class ChunkManager
    {
        public:
            ChunkManager();
            ChunkManager(const Eigen::Vector3i& chunkSize, float voxelResolution);
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

            inline ChunkID GetIDAt(const Vec3& pos)
            {
                return ChunkID(static_cast<int>(std::floor(pos(0) / chunkSize(0))),
                               static_cast<int>(std::floor(pos(1) / chunkSize(1))),
                               static_cast<int>(std::floor(pos(2) / chunkSize(2))));
            }

            void GetChunkIDsIntersecting(const AABB& box, ChunkIDList* chunkList);
            void GetChunkIDsIntersecting(const Frustum& frustum, ChunkIDList* chunkList);


            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        protected:
            ChunkMap chunks;
            Eigen::Vector3i chunkSize;
            float voxelResolutionMeters;
    };

    typedef std::shared_ptr<ChunkManager> ChunkManagerPtr;
    typedef std::shared_ptr<const ChunkManager> ChunkManagerConstPtr;


} // namespace chisel 

#endif // CHUNKMANAGER_H_ 
