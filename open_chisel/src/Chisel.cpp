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
    for (const ChunkID& chunkID : chunks)
      {
        chunkManager.RemoveChunk(chunkID);
      }
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

  void Chisel::IntegratePointCloud(const ProjectionIntegrator& integrator, const PointCloud& cloud, const Transform& extrinsic, float truncation, float maxDist)
  {
    clock_t begin = clock();
    ChunkIDList chunksIntersecting;
    chunkManager.GetChunkIDsIntersecting(cloud, extrinsic, truncation, maxDist, &chunksIntersecting);
    printf("There are %lu chunks intersecting\n", chunksIntersecting.size());
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

        bool needsUpdate = integrator.Integrate(cloud, extrinsic, chunk.get());

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
      });
    GarbageCollect(garbageChunks);
    //  chunkManager.PrintMemoryStatistics();

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    printf("\n \n  ~ %f HZ     \n \n", 1/elapsed_secs);
  }

  void Chisel::IntegrateChunks(const ProjectionIntegrator& integrator, std::vector<chisel::ChunkPtr>& chunks)
  {
    clock_t begin = clock();
    printf("CHISEL: Integrating chunks");

    ChunkIDList chunksIntersecting;
    std::vector<chisel::ChunkPtr> chunksToIntegrate;

    float mapResolution = GetChunkManager().GetResolution();
    float chunkResolution = chunks[0]->GetVoxelResolutionMeters();
    Eigen::Vector3i mapChunksize = GetChunkManager().GetChunkSize();
    Eigen::Vector3i chunkChunksize = chunks[0]->GetNumVoxels();

    ChunkManager tempChunkManager(mapChunksize,mapResolution, chunks[0]->HasColors());

    if(std::fabs(mapResolution - chunkResolution) < 0.0001 && mapChunksize == chunkChunksize)
    {
      printf("Chunk Size and chunk Resolution fit for source and target chunks! \n");
      for (ChunkPtr c : chunks)
      {
        chunksIntersecting.push_back(c->GetID());
        chunksToIntegrate.push_back(c);
      }
    }
    else
    {
      for (ChunkPtr c : chunks)
        {

          ChunkPtr chunk;

          for (int i = 0; i <c->GetTotalNumVoxels(); i++)
          {
            DistVoxel distVoxel = c->GetDistVoxelMutable(i);
            Vec3 worldPosition = c->GetOrigin() + chunkResolution * getVoxelCoordinates(i, chunkChunksize).cast<float>();
            ChunkID chunkID = tempChunkManager.GetIDAt(worldPosition);

            if (tempChunkManager.HasChunk(chunkID))
            {
              chunk = tempChunkManager.GetChunk(chunkID);
            }
            else
            {
              tempChunkManager.CreateChunk(chunkID);
              chunk = tempChunkManager.GetChunk(chunkID);
            }

            chisel::Vec3 relPosition = (worldPosition - chunk->GetOrigin());
            VoxelID voxelID = chunk->GetVoxelID(relPosition);
            //Apply the old data to the fitting voxel
            chunk->GetDistVoxelMutable(voxelID).SetSDF(distVoxel.GetSDF());
            chunk->GetDistVoxelMutable(voxelID).SetWeight(distVoxel.GetWeight());
          }
        }

      ChunkMap tempChunks = tempChunkManager.GetChunks();

        for (auto it : tempChunks)
        {
          chunksIntersecting.push_back(it.first);
          chunksToIntegrate.push_back(it.second);
        }

        printf("Finished converting");
      }

    std::mutex mutex;
    ChunkIDList garbageChunks;

    for(int i = 0; i< chunksIntersecting.size();i++ )
      //    parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
      {
        ChunkID chunkID = chunksIntersecting.at(i);
        bool chunkNew = false;

        mutex.lock();
        if (!chunkManager.HasChunk(chunkID))
          {
            chunkNew = true;
            chunkManager.CreateChunk(chunkID);
          }

        ChunkPtr chunk = chunkManager.GetChunk(chunkID);
        mutex.unlock();

        bool needsUpdate = integrator.IntegrateChunk(chunksToIntegrate.at(i).get(), chunk.get());

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
    //);
    printf("CHISEL: Done with chunks");
    //GarbageCollect(garbageChunks);
    //chunkManager.PrintMemoryStatistics();

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    printf("\n \n  Time needed: %f  ms  \n \n", elapsed_secs*1000);
  }

 /* Chunk Chisel::downSampleChunk(Chunk& originalChunk, Eigen::Vector3i targetChunkSize , Eigen::Vector3i samplingFactor)
  {
    Chunk chunk(originalChunk.GetID(), targetChunkSize, originalChunk.GetVoxelResolutionMeters(), originalChunk.HasColors());
    int voxelIndex = 0;

    for (int z = 0 ; z < targetChunkSize[2]; z += samplingFactor[2])
    {
      for (int y = 0 ; y < targetChunkSize[1]; y += samplingFactor[1])
      {
        for (int x = 0 ; x < targetChunkSize[0]; x += samplingFactor[0])
        {
          chunk.GetDistVoxelMutable(voxelIndex) = originalChunk.GetDistVoxel(x,y,z);
          voxelIndex++;
        }
      }
    }

    return chunk;

  }*/

  Point3 Chisel::getVoxelCoordinates(VoxelID id, Eigen::Vector3i chunkSize)
  {
    int x = id % chunkSize(0);

    int fraction = ((id-x)/chunkSize(0));

    int y = fraction % chunkSize(2);
    int z = (fraction-y)/chunkSize(2);

   /* if (x<0 || x>=chunkSize(0) ||y<0 || y>=chunkSize(1) || z<0 || z>=chunkSize(2)){
        printf("X: %d Y: %d Z: %d \n", x, y, z);
        printf("VoxelID %d \n", id);
        printf("Chunksize X: %d Chunksize Y: %d Chunksize Z: %d \n", chunkSize(0), chunkSize(1), chunkSize(2));

      }*/

    return Point3(x,y,z);
  }

  Chunk Chisel::upSampleChunk(Chunk& originalChunk, Eigen::Vector3i targetChunkSize , Eigen::Vector3i samplingFactor)
  {
    Chunk chunk(originalChunk.GetID(), targetChunkSize, originalChunk.GetVoxelResolutionMeters(), originalChunk.HasColors());

    return chunk;

  }

} // namespace chisel 
