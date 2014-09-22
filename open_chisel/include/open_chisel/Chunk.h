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

#ifndef CHUNK_H_
#define CHUNK_H_

#include <memory>
#include <vector>
#include <Eigen/Core>

#include <open_chisel/geometry/AABB.h>
#include "DistVoxel.h"

namespace chisel
{

    typedef Eigen::Vector3i ChunkID;
    typedef int VoxelID;

    class Chunk
    {
        public:
            Chunk();
            virtual ~Chunk();

            void AllocateDistVoxels();

            inline const ChunkID& GetID() const { return ID; }
            inline ChunkID& GetIDMutable() { return ID; }
            inline void SetID(const ChunkID& id) { ID = id; }

            inline bool HasVoxels() const { return !voxels.empty(); }
            inline const std::vector<DistVoxel>& GetVoxels() const { return voxels; }

            inline const Eigen::Vector3i& GetNumVoxels() const { return numVoxels; }
            inline float GetVoxelResolutionMeters() const { return voxelResolutionMeters; }

            inline const DistVoxel& GetDistVoxel(const VoxelID& voxelID) const { return voxels[voxelID]; }
            inline DistVoxel& GetDistVoxelMutable(const VoxelID& voxelID) { return voxels[voxelID]; }

            inline VoxelID GetVoxelID(int x, int y, int z) const
            {
                return (z * numVoxels(2) + y) * numVoxels(0) + x;
            }

            inline const DistVoxel& GetDistVoxel(int x, int y, int z) const
            {
                return GetDistVoxel(GetVoxelID(x, y, z));
            }

            inline DistVoxel& GetDistVoxelMutable(int x, int y, int z)
            {
                return GetDistVoxelMutable(GetVoxelID(x, y, z));
            }

            inline bool IsCoordValid(int x, int y, int z) const
            {
                return (x >= 0 && x < numVoxels(0) && y >= 0 && y < numVoxels(1) && z >= 0 && z < numVoxels(2));
            }

            inline size_t GetTotalNumVoxels()
            {
                return numVoxels(0) * numVoxels(1) * numVoxels(2);
            }

            AABB ComputeBoundingBox();
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        protected:
            ChunkID ID;
            Eigen::Vector3i numVoxels;
            float voxelResolutionMeters;
            std::vector<DistVoxel> voxels;

    };

    typedef std::shared_ptr<Chunk> ChunkPtr;
    typedef std::shared_ptr<const Chunk> ChunkConstPtr;

} // namespace chisel 

#endif // CHUNK_H_ 
