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

#include <open_chisel/Chisel.h>

#include <open_chisel/io/PLY.h>

#include <open_chisel/geometry/Raycast.h>

#include <iostream>
namespace chisel
{

    Chisel::Chisel()
    {
        // TODO Auto-generated constructor stub
    }

    Chisel::Chisel(const Eigen::Vector3i& chunkSize, float voxelResolution, bool useColor) :
            chunkManager(chunkSize, voxelResolution, useColor)
    {

    }

    Chisel::~Chisel()
    {
        // TODO Auto-generated destructor stub
    }

    void Chisel::Reset()
    {
        chunkManager.Reset();
        meshesToUpdate.clear();
    }

    void Chisel::UpdateMeshes()
    {
        chunkManager.RecomputeMeshes(meshesToUpdate);
        meshesToUpdate.clear();
    }

    void Chisel::GarbageCollect(const ChunkIDList& chunks)
    {
        std::cout << chunkManager.GetChunks().size() << " chunks " << chunkManager.GetAllMeshes().size() << "meshes before collect.";
       for (const ChunkID& chunkID : chunks)
       {
           chunkManager.RemoveChunk(chunkID);
           meshesToUpdate.erase(chunkID);
       }
        std::cout << chunkManager.GetChunks().size() << " chunks " << chunkManager.GetAllMeshes().size() << "meshes after collect.";
    }

    bool Chisel::SaveAllMeshesToPLY(const std::string& filename)
    {
        printf("Saving all meshes to PLY file...\n");

        chisel::MeshPtr fullMesh(new chisel::Mesh());

        size_t v = 0;
        for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
        {
            for (const Vec3& vert : it.second->vertices)
            {
                fullMesh->vertices.push_back(vert);
                fullMesh->indices.push_back(v);
                v++;
            }

            for (const Vec3& color : it.second->colors)
            {
                fullMesh->colors.push_back(color);
            }

            for (const Vec3& normal : it.second->normals)
            {
                fullMesh->normals.push_back(normal);
            }
        }

        printf("Full mesh has %lu verts\n", v);
        bool success = SaveMeshPLYASCII(filename, fullMesh);

        if (!success)
        {
            printf("Saving failed!\n");
        }

        return success;
    }

    void Chisel::IntegratePointCloud(const ProjectionIntegrator& integrator,
                                     const PointCloud& cloud,
                                     const Transform& cameraPose,
                                     float maxDist)
    {

          const float roundToVoxel = 1.0f / chunkManager.GetResolution();
          const Vec3 halfVoxel = Vec3(0.5, 0.5, 0.5) * chunkManager.GetResolution();
          std::unordered_map<ChunkID, bool, ChunkHasher> updated;
          std::unordered_map<ChunkID, bool, ChunkHasher> newChunks;
          const Vec3 startCamera = cameraPose.translation();
          const Transform inversePose = cameraPose.inverse();
          const Point3 minVal(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());
          const Point3 maxVal(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
          const size_t numPoints = cloud.GetPoints().size();
          const bool carving = integrator.IsCarvingEnabled();
          const float carvingDist = integrator.GetCarvingDist();

          for (size_t i = 0; i < numPoints; i++)
          {
              Point3List raycastVoxels;
              const Vec3& point = cloud.GetPoints().at(i);
              Vec3 worldPoint = cameraPose * point;
              float depth = point.z();

              if (depth < 0.01f) continue;

              Vec3 dir = (worldPoint - startCamera).normalized();
              float truncation = integrator.GetTruncator()->GetTruncationDistance(depth);
              Vec3 start = carving ? static_cast<Vec3>(startCamera * roundToVoxel) : static_cast<Vec3>((worldPoint - dir * truncation) * roundToVoxel);
              Vec3 end = (worldPoint + dir * truncation) * roundToVoxel;

              raycastVoxels.clear();
              Raycast(start, end, minVal, maxVal, &raycastVoxels);
              if (raycastVoxels.size() > 500)
              {
                  std::cerr << "Error, too many racyast voxels." << std::endl;
                  throw std::out_of_range("Too many raycast voxels");
              }

              //std::cout << "chunk: " << chunk->GetID().transpose() << " from "
              //        << start.transpose() << " to " << end.transpose() << " : " << raycastVoxels.size() << std::endl;
              for (const Point3& voxelCoords : raycastVoxels)
              {
                  Vec3 center = chunkManager.GetCentroid(voxelCoords);
                  bool wasNew = false;
                  ChunkPtr chunk = chunkManager.GetOrCreateChunkAt(center, &wasNew);
                  if (wasNew)
                  {
                      newChunks[chunk->GetID()] = true;
                      updated[chunk->GetID()] = false;
                  }

                  VoxelID id = chunk->GetLocalVoxelIDFromGlobal(voxelCoords);
                  if (!chunk->IsCoordValid(id))
                  {
                      continue;
                  }

                  DistVoxel& distVoxel = chunk->GetDistVoxelMutable(id);
                  float u = depth - (inversePose * (center)).z();
                  float weight = integrator.GetWeighter()->GetWeight(u, truncation);

                  if (fabs(u) < truncation)
                  {
                      distVoxel.Integrate(u, weight);
                      updated[chunk->GetID()] = true;
                  }
                  else if (!wasNew && carving && u > truncation + carvingDist)
                  {
                      if (distVoxel.GetWeight() > 0 && distVoxel.GetSDF() < 0.0f)
                      {
                          distVoxel.Carve();
                          updated[chunk->GetID()] = true;
                      }
                  }
              }
          }

          for (const std::pair<ChunkID, bool>& updatedChunk : updated)
          {
              if (updatedChunk.second)
              {
                  for (int dx = -1; dx <= 1; dx++)
                  {
                      for (int dy = -1; dy <= 1; dy++)
                      {
                          for (int dz = -1; dz <= 1; dz++)
                          {
                              meshesToUpdate[updatedChunk.first + ChunkID(dx, dy, dz)] = true;
                          }
                      }
                  }
              }
          }
          ChunkIDList garbageChunks;
          for (const std::pair<ChunkID, bool>& newChunk : newChunks)
          {
              if (!updated[newChunk.first])
              {
                  garbageChunks.push_back(newChunk.first);
              }
          }

          GarbageCollect(garbageChunks);
    }


} // namespace chisel 
