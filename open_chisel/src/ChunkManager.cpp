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

#include <assert.h>
#include <open_chisel/ChunkManager.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/geometry/AABB.h>
#include <open_chisel/marching_cubes/MarchingCubes.h>


namespace chisel
{

    ChunkManager::ChunkManager() :
            chunkSize(16, 16, 16), voxelResolutionMeters(0.03)
    {
        CacheCentroids();
    }

    ChunkManager::~ChunkManager()
    {

    }

    ChunkManager::ChunkManager(const Eigen::Vector3i& size, float res, bool color) :
            chunkSize(size), voxelResolutionMeters(res), useColor(color)
    {
        CacheCentroids();
    }

    void ChunkManager::CacheCentroids()
    {
        Vec3 halfResolution = Vec3(voxelResolutionMeters, voxelResolutionMeters, voxelResolutionMeters) * 0.5f;
        centroids.resize(static_cast<size_t>(chunkSize(0) * chunkSize(1) * chunkSize(2)));
        int i = 0;
        for (int z = 0; z < chunkSize(2); z++)
        {
            for(int y = 0; y < chunkSize(1); y++)
            {
                for(int x = 0; x < chunkSize(0); x++)
                {
                    centroids[i] = Vec3(x, y, z) * voxelResolutionMeters + halfResolution;
                    i++;
                }
            }
        }

        cubeIndexOffsets << 0, 1, 1, 0, 0, 1, 1, 0,
                            0, 0, 1, 1, 0, 0, 1, 1,
                            0, 0, 0, 0, 1, 1, 1, 1;
    }

    void ChunkManager::GetChunkIDsIntersecting(const AABB& box, ChunkIDList* chunkList)
    {
        assert(chunkList != nullptr);

        ChunkID minID = GetIDAt(box.min);
        ChunkID maxID = GetIDAt(box.max) + Eigen::Vector3i(1, 1, 1);

        for (int x = minID(0); x < maxID(0); x++)
        {
            for (int y = minID(1); y < maxID(1); y++)
            {
                for (int z = minID(2); z < maxID(2); z++)
                {
                    chunkList->push_back(ChunkID(x, y, z));
                }
            }
        }
    }


    void ChunkManager::RecomptueMesh(const ChunkID& chunkID)
    {
        if (!HasChunk(chunkID))
        {
            return;
        }

        MeshPtr mesh;
        if (!HasMesh(chunkID))
        {
            mesh = std::allocate_shared<Mesh>(Eigen::aligned_allocator<Mesh>());
        }
        else
        {
            mesh = GetMesh(chunkID);
        }


        ChunkPtr chunk = GetChunk(chunkID);
        GenerateMesh(chunk, mesh.get());

        if(useColor)
        {
            ColorizeMesh(mesh.get());
        }

        if(!mesh->vertices.empty())
            allMeshes[chunkID] = mesh;
    }

    void ChunkManager::RecomputeMeshes(const ChunkIDList& chunks)
    {
        for (const ChunkID& chunkID : chunks)
        {
            RecomptueMesh(chunkID);
        }
    }

    void ChunkManager::CreateChunk(const ChunkID& id)
    {
        AddChunk(std::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), id, chunkSize, voxelResolutionMeters, useColor));
    }

    void ChunkManager::Reset()
    {
        allMeshes.clear();
        chunks.clear();
    }

    void ChunkManager::GetChunkIDsIntersecting(const Frustum& frustum, ChunkIDList* chunkList)
    {
        assert(chunkList != nullptr);

        AABB frustumAABB;
        frustum.ComputeBoundingBox(&frustumAABB);

        ChunkID minID = GetIDAt(frustumAABB.min);
        ChunkID maxID = GetIDAt(frustumAABB.max) + Eigen::Vector3i(1, 1, 1);

        //printf("FrustumAABB: %f %f %f %f %f %f\n", frustumAABB.min.x(), frustumAABB.min.y(), frustumAABB.min.z(), frustumAABB.max.x(), frustumAABB.max.y(), frustumAABB.max.z());
        //printf("Frustum min: %d %d %d max: %d %d %d\n", minID.x(), minID.y(), minID.z(), maxID.x(), maxID.y(), maxID.z());
        for (int x = minID(0) - 1; x <= maxID(0); x++)
        {
            for (int y = minID(1) - 1; y <= maxID(1); y++)
            {
                for (int z = minID(2) - 1; z <= maxID(2); z++)
                {
                    Vec3 min = Vec3(x * chunkSize(0), y * chunkSize(1), z * chunkSize(2)) * voxelResolutionMeters;
                    Vec3 max = min + chunkSize.cast<float>() * voxelResolutionMeters;
                    AABB chunkBox(min, max);
                    if(frustum.Intersects(chunkBox))
                    {
                        chunkList->push_back(ChunkID(x, y, z));
                    }
                }
            }
        }

        //printf("%lu chunks intersect frustum\n", chunkList->size());
    }

    void ChunkManager::ExtractInsideVoxelMesh(const ChunkPtr& chunk, const Eigen::Vector3i& index, const Vec3& coords, VertIndex* nextMeshIndex, Mesh* mesh)
    {
        assert(mesh != nullptr);
        Eigen::Matrix<float, 3, 8> cubeCoordOffsets = cubeIndexOffsets.cast<float>() * voxelResolutionMeters;
        Eigen::Matrix<float, 3, 8> cornerCoords;
        Eigen::Matrix<float, 8, 1> cornerSDF;
        bool allNeighborsObserved = true;
        for (int i = 0; i < 8; ++i)
        {
            Eigen::Vector3i corner_index = index + cubeIndexOffsets.col(i);
            const DistVoxel& thisVoxel = chunk->GetDistVoxel(corner_index.x(), corner_index.y(), corner_index.z());

            // Do not extract a mesh here if one of the corners is unobserved and
            // outside the truncation region.
            if (thisVoxel.GetWeight() == 0)
            {
                allNeighborsObserved = false;
                break;
            }
            cornerCoords.col(i) = coords + cubeCoordOffsets.col(i);
            cornerSDF(i) = thisVoxel.GetSDF();
        }

        if (allNeighborsObserved)
        {
            MarchingCubes::MeshCube(cornerCoords, cornerSDF, nextMeshIndex, mesh);
        }
    }

    void ChunkManager::ExtractBorderVoxelMesh(const ChunkPtr& chunk, const Eigen::Vector3i& index, const Eigen::Vector3f& coordinates, VertIndex* nextMeshIndex, Mesh* mesh)
    {
        const Eigen::Matrix<float, 3, 8> cubeCoordOffsets = cubeIndexOffsets.cast<float>() * voxelResolutionMeters;
        Eigen::Matrix<float, 3, 8> cornerCoords;
        Eigen::Matrix<float, 8, 1> cornerSDF;
        bool allNeighborsObserved = true;
        for (int i = 0; i < 8; ++i)
        {
            Eigen::Vector3i cornerIDX = index + cubeIndexOffsets.col(i);

            if (chunk->IsCoordValid(cornerIDX.x(), cornerIDX.y(), cornerIDX.z()))
            {
                const DistVoxel& thisVoxel = chunk->GetDistVoxel(cornerIDX.x(), cornerIDX.y(), cornerIDX.z());
                // Do not extract a mesh here if one of the corners is unobserved
                // and outside the truncation region.
                if (thisVoxel.GetWeight() == 0)
                {
                    allNeighborsObserved = false;
                    break;
                }
                cornerCoords.col(i) = coordinates + cubeCoordOffsets.col(i);
                cornerSDF(i) = thisVoxel.GetSDF();
            }
            else
            {
                Eigen::Vector3i chunkOffset = Eigen::Vector3i::Zero();


                for (int j = 0; j < 3; j++)
                {
                    if (cornerIDX(j) < 0)
                    {
                        chunkOffset(j) = -1;
                        cornerIDX(j) = chunkSize(j) - 1;
                    }
                    else if(cornerIDX(j) >= chunkSize(j))
                    {
                        chunkOffset(j) = 1;
                        cornerIDX(j) = 0;
                    }
                }

                ChunkID neighborID = chunkOffset + chunk->GetID();

                if (HasChunk(neighborID))
                {
                    const ChunkPtr& neighborChunk = GetChunk(neighborID);

                    const DistVoxel& thisVoxel = neighborChunk->GetDistVoxel(cornerIDX.x(), cornerIDX.y(), cornerIDX.z());
                    // Do not extract a mesh here if one of the corners is unobserved
                    // and outside the truncation region.
                    if (thisVoxel.GetWeight() == 0)
                    {
                        allNeighborsObserved = false;
                        break;
                    }
                    cornerCoords.col(i) = coordinates + cubeCoordOffsets.col(i);
                    cornerSDF(i) = thisVoxel.GetSDF();
                }
                else
                {
                    allNeighborsObserved = false;
                    break;
                }

            }

        }

        if (allNeighborsObserved)
        {
            MarchingCubes::MeshCube(cornerCoords, cornerSDF, nextMeshIndex, mesh);
        }
    }


    void ChunkManager::GenerateMesh(const ChunkPtr& chunk, Mesh* mesh)
    {
        assert(mesh != nullptr);

        mesh->Clear();
        const int maxX = chunkSize(0);
        const int maxY = chunkSize(1);
        const int maxZ = chunkSize(2);


        Eigen::Vector3i index;
        VoxelID i = 0;
        VertIndex nextIndex = 0;

        // For voxels not bordering the outside, we can use a more efficient function.
        for (index.z() = 0; index.z() < maxZ - 1; index.z()++)
        {
            for (index.y() = 0; index.y() < maxY - 1; index.y()++)
            {
                for (index.x() = 0; index.x() < maxX - 1; index.x()++)
                {
                    i = chunk->GetVoxelID(index.x(), index.y(), index.z());
                    ExtractInsideVoxelMesh(chunk, index, centroids[i] + chunk->GetOrigin(), &nextIndex, mesh);
                }
            }
        }

        // Max X plane (takes care of max-Y corner as well).
        i = 0;
        index.x() = maxX - 1;
        for (index.z() = 0; index.z() < maxZ - 1; index.z()++)
        {
            for (index.y() = 0; index.y() < maxY; index.y()++)
            {
                i = chunk->GetVoxelID(index.x(), index.y(), index.z());
                ExtractBorderVoxelMesh(chunk, index, centroids[i] + chunk->GetOrigin(), &nextIndex, mesh);
            }
        }

        // Max Y plane.
        i = 0;
        index.y() = maxY - 1;
        for (index.z() = 0; index.z() < maxZ - 1; index.z()++)
        {
            for (index.x() = 0; index.x() < maxX - 1; index.x()++)
            {
                i = chunk->GetVoxelID(index.x(), index.y(), index.z());
                ExtractBorderVoxelMesh(chunk, index, centroids[i] + chunk->GetOrigin(), &nextIndex, mesh);
            }
        }

        // Max Z plane (also takes care of corners).
        i = 0;
        index.z() = maxZ - 1;
        for (index.y() = 0; index.y() < maxY; index.y()++)
        {
            for (index.x() = 0; index.x() < maxX; index.x()++)
            {
                i = chunk->GetVoxelID(index.x(), index.y(), index.z());
                ExtractBorderVoxelMesh(chunk, index, centroids[i] + chunk->GetOrigin(), &nextIndex, mesh);
            }
        }

        //printf("Generated a new mesh with %lu verts, %lu norm, and %lu idx\n", mesh->vertices.size(), mesh->normals.size(), mesh->indices.size());

        assert(mesh->vertices.size() == mesh->normals.size());
        assert(mesh->vertices.size() == mesh->indices.size());
    }

    Vec3 ChunkManager::InterpolateColor(const Vec3& colorPos)
    {
        const float& x = colorPos(0);
        const float& y = colorPos(1);
        const float& z = colorPos(2);
        const int x_0 = static_cast<int>(std::floor(x / voxelResolutionMeters));
        const int y_0 = static_cast<int>(std::floor(y / voxelResolutionMeters));
        const int z_0 = static_cast<int>(std::floor(z / voxelResolutionMeters));
        const int x_1 = x_0 + 1;
        const int y_1 = y_0 + 1;
        const int z_1 = z_0 + 1;


        const ColorVoxel* v_000 = GetColorVoxel(Vec3(x_0, y_0, z_0));
        const ColorVoxel* v_001 = GetColorVoxel(Vec3(x_0, y_0, z_1));
        const ColorVoxel* v_011 = GetColorVoxel(Vec3(x_0, y_1, z_1));
        const ColorVoxel* v_111 = GetColorVoxel(Vec3(x_1, y_1, z_1));
        const ColorVoxel* v_110 = GetColorVoxel(Vec3(x_1, y_1, z_0));
        const ColorVoxel* v_100 = GetColorVoxel(Vec3(x_1, y_0, z_0));
        const ColorVoxel* v_010 = GetColorVoxel(Vec3(x_0, y_1, z_0));
        const ColorVoxel* v_101 = GetColorVoxel(Vec3(x_1, y_0, z_1));

        if(!v_000 || !v_001 || !v_011 || !v_111 || !v_110 || !v_100 || v_010 || v_101)
        {
            const ChunkID& chunkID = GetIDAt(colorPos);

            if(!HasChunk(chunkID))
            {
                return Vec3(0, 0, 0);
            }
            else
            {
                const ChunkPtr& chunk = GetChunk(chunkID);
                return chunk->GetColorAt(colorPos);
            }
        }

        float xd = (x - x_0) / (x_1 - x_0);
        float yd = (y - y_0) / (y_1 - y_0);
        float zd = (z - z_0) / (z_1 - z_0);
        float red, green, blue = 0.0f;
        {
            float c_00 = v_000->GetRed() * (1 - xd) + v_100->GetRed() * xd;
            float c_10 = v_010->GetRed() * (1 - xd) + v_110->GetRed() * xd;
            float c_01 = v_001->GetRed() * (1 - xd) + v_101->GetRed() * xd;
            float c_11 = v_011->GetRed() * (1 - xd) + v_111->GetRed() * xd;
            float c_0 = c_00 * (1 - yd) + c_10 * yd;
            float c_1 = c_01 * (1 - yd) + c_11 * yd;
            float c = c_0 * (1 - zd) + c_1 * zd;
            red = c / 255.0f;
        }
        {
            float c_00 = v_000->GetGreen() * (1 - xd) + v_100->GetGreen() * xd;
            float c_10 = v_010->GetGreen() * (1 - xd) + v_110->GetGreen() * xd;
            float c_01 = v_001->GetGreen() * (1 - xd) + v_101->GetGreen() * xd;
            float c_11 = v_011->GetGreen() * (1 - xd) + v_111->GetGreen() * xd;
            float c_0 = c_00 * (1 - yd) + c_10 * yd;
            float c_1 = c_01 * (1 - yd) + c_11 * yd;
            float c = c_0 * (1 - zd) + c_1 * zd;
            green = c / 255.0f;
        }
        {
            float c_00 = v_000->GetBlue() * (1 - xd) + v_100->GetBlue()  * xd;
            float c_10 = v_010->GetBlue() * (1 - xd) + v_110->GetBlue()  * xd;
            float c_01 = v_001->GetBlue() * (1 - xd) + v_101->GetBlue()  * xd;
            float c_11 = v_011->GetBlue() * (1 - xd) + v_111->GetBlue()  * xd;
            float c_0 = c_00 * (1 - yd) + c_10 * yd;
            float c_1 = c_01 * (1 - yd) + c_11 * yd;
            float c = c_0 * (1 - zd) + c_1 * zd;
            blue = c / 255.0f;
         }

        return Vec3(red, green, blue);
    }

    const DistVoxel* ChunkManager::GetDistanceVoxel(const Vec3& pos)
    {
        ChunkPtr chunk = GetChunkAt(pos);

        if(chunk.get())
        {
            return &(chunk->GetDistVoxel(chunk->GetVoxelID(pos)));
        }
        else return nullptr;
    }

    const ColorVoxel* ChunkManager::GetColorVoxel(const Vec3& pos)
    {
        ChunkPtr chunk = GetChunkAt(pos);

        if(chunk.get())
        {
            chisel::AABB chunkAABB = chunk->ComputeBoundingBox();

            if(chunkAABB.Contains(pos))
            {
                return &(chunk->GetColorVoxel(chunk->GetVoxelID(pos)));
            }
            else
            {
                return nullptr;
            }
        }
        else return nullptr;
    }


    void ChunkManager::ColorizeMesh(Mesh* mesh)
    {
        assert(mesh != nullptr);

        mesh->colors.clear();
        for (size_t i = 0; i < mesh->vertices.size(); i++)
        {
            const Vec3& vertex = mesh->vertices.at(i);
            mesh->colors.push_back(InterpolateColor(vertex));
        }
    }

} // namespace chisel 
