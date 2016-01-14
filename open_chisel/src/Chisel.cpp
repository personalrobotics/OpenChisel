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
              };

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

  void Chisel::IntegrateChunks(const ProjectionIntegrator& integrator, const ChunkMap& chunks)
  {
    clock_t begin = clock();
    printf("CHISEL: Integrating chunks \n");

    ChunkIDList chunksIntersecting;
    std::vector<chisel::ChunkPtr> chunksToIntegrate;

    float mapResolution = GetChunkManager().GetResolution();
    float chunkResolution = chunks.cbegin()->second->GetVoxelResolutionMeters();
    Eigen::Vector3i mapChunksize = GetChunkManager().GetChunkSize();
    Eigen::Vector3i chunkChunksize = chunks.cbegin()->second->GetNumVoxels();

    ChunkManager tempTargetChunkManager(mapChunksize,mapResolution, chunks.cbegin()->second->HasColors());

    if(std::fabs(mapResolution - chunkResolution) < 0.0001 && mapChunksize == chunkChunksize)
    {
      printf("Chunk Size and chunk Resolution fit for source and target chunks! \n");
      for (const std::pair<chisel::ChunkID, ChunkPtr>& c : chunks)
      {
        chunksIntersecting.push_back(c.first);  // c->GetID());
        chunksToIntegrate.push_back(c.second);
      }
    }
    else
    {
      if (mapResolution != chunkResolution)
      {
        printf("Interpolating chunks! \n");
        int counter = 0;

      /*}
      else if (mapResolution < chunkResolution)
      {*/
        for (const std::pair<chisel::ChunkID, ChunkPtr>& c : chunks)
        {

          Vec3 origin = c.second->GetOrigin();
          Vec3 maxChunkPos = origin + chunkResolution * chunkChunksize.cast<float>();

          //Get all to the chunk belonging grid vertices as cuboid
          Vec3 minPosition(origin(0) - std::fmod(origin(0), mapResolution), origin(1) - std::fmod(origin(1), mapResolution), origin(2) - std::fmod(origin(2), mapResolution));
          Vec3 maxPosition(maxChunkPos(0) - std::fmod(maxChunkPos(0), mapResolution), maxChunkPos(1) - std::fmod(maxChunkPos(1), mapResolution), maxChunkPos(2) - std::fmod(maxChunkPos(2), mapResolution));

          ChunkPtr chunk;
          ChunkManager tempSourceChunkManager(chunkChunksize,chunkResolution, chunks.cbegin()->second->HasColors());

          Vec3 range = (maxPosition-minPosition)/mapResolution;
         // printf("Visiting %f voxels for this chunk!", range(0)*range(1)*range(2));

          std::vector<std::pair<ChunkID, VoxelID>> cornerVoxels(8);
          ChunkID sourceChunkID;
          ChunkID targetChunkID;

          for (float x = minPosition(0); x < maxPosition(0); x+=mapResolution)
          {
            for (float y = minPosition(1); y < maxPosition(1); y+=mapResolution)
            {
              for (float z = minPosition(2); z < maxPosition(2); z+=mapResolution)
              {
                Vec3 voxelPosition(x,y,z);
                //printf("Get IDs \n");
                sourceChunkID = tempSourceChunkManager.GetIDAt(voxelPosition);
                targetChunkID = tempTargetChunkManager.GetIDAt(voxelPosition);

                //  printf("Get voxel coords \n");
                const Point3 voxelCoords = c.second->GetVoxelCoords(voxelPosition-origin);
                // printf("Get voxel corners \n");
                getVoxelCornerIDs(&cornerVoxels, sourceChunkID, voxelCoords, chunkChunksize);

               // printf("Get target chunk \n");
                if (tempTargetChunkManager.HasChunk(targetChunkID))
                {
                  chunk = tempTargetChunkManager.GetChunk(targetChunkID);
                }
                else
                {
                  tempTargetChunkManager.CreateChunk(targetChunkID);
                  chunk = tempTargetChunkManager.GetChunk(targetChunkID);
                }

                //Vec3 targetOrigin =  chunk->GetOrigin();
               // printf("Got origin of target \n");
                //printf("Origin: %f %f %f \n", targetOrigin(0),targetOrigin(1),targetOrigin(2));
               // printf("Get dist voxel \n");

                DistVoxel* d = tempTargetChunkManager.GetDistanceVoxelMutable(voxelPosition);

               // printf("Got dist voxel \n");

              //  printf("interpolate voxel");
                interpolateDistVoxel(voxelPosition, cornerVoxels, chunks, chunkResolution, chunkChunksize, d );
              }
            }
          }

          counter++;
          printf("Finished chunk number %d \n", counter);
        }

      }
      else
      {
        //Resulution is fitting, but because of different chunksizes, all data goes into other chunks
        for (const std::pair<chisel::ChunkID, ChunkPtr>& c : chunks)
        {
          ChunkPtr chunk;

          for (int i = 0; i < c.second->GetTotalNumVoxels(); i++)
          {
            DistVoxel distVoxel = c.second->GetDistVoxelMutable(i);
            Vec3 worldPosition = c.second->GetOrigin() + chunkResolution * getVoxelCoordinates(i, chunkChunksize).cast<float>();
            ChunkID chunkID = tempTargetChunkManager.GetIDAt(worldPosition);

            if (tempTargetChunkManager.HasChunk(chunkID))
            {
              chunk = tempTargetChunkManager.GetChunk(chunkID);
            }
            else
            {
              tempTargetChunkManager.CreateChunk(chunkID);
              chunk = tempTargetChunkManager.GetChunk(chunkID);
            }

            chisel::Vec3 relPosition = (worldPosition - chunk->GetOrigin());

            //Prevent getting -0.0 and wrong voxelIDs with abs(pos)
            relPosition(0) = std::abs(relPosition(0));
            relPosition(1) = std::abs(relPosition(1));
            relPosition(2) = std::abs(relPosition(2));

            VoxelID voxelID = chunk->GetVoxelID(relPosition);
            if (voxelID >= 0 && voxelID < chunk->GetTotalNumVoxels())
            {
              //Apply the old data to the fitting voxel
              chunk->GetDistVoxelMutable(voxelID).SetSDF(distVoxel.GetSDF());
              chunk->GetDistVoxelMutable(voxelID).SetWeight(distVoxel.GetWeight());
            }
            else
            {
              fprintf(stderr," \n Number of voxel: %d from %d total voxels in this chunk \n", i, (int) c.second->GetTotalNumVoxels());
              fprintf(stderr,"ChunkID %d %d %d and Worldposition %.16f %.16f %.16f \n", chunkID(0),chunkID(1),chunkID(2), worldPosition(0), worldPosition(1), worldPosition(2) );
              fprintf(stderr,"VoxelID %d", voxelID);
              Point3 p = chunk->GetVoxelCoords(relPosition);
              fprintf(stderr,"Coordinates %d %d %d \n", p(0), p(1), p(2));

              //exit(0);
            }
          }
        }
      }

      ChunkMap tempChunks = tempTargetChunkManager.GetChunks();

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
    //GarbageCollect(garbageChunks);DistVoxel
    //chunkManager.PrintMemoryStatistics();

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    printf("\n \n  Time needed: %f  ms  \n \n", elapsed_secs*1000);
  }

  bool Chisel::interpolateDistVoxel(const Vec3& voxelPos, const std::vector<std::pair<ChunkID, VoxelID>>& cornerVoxels, const ChunkMap& chunks, const float& resolution, const Point3& chunkSize, DistVoxel* voxel)
  {
    if(voxel == nullptr)
      return false;

      const float x = voxelPos(0);
      const float y = voxelPos(1);
      const float z = voxelPos(2);

      Vec3 startPos = resolution * (cornerVoxels[0].first.cwiseProduct(chunkSize)).cast<float>() + resolution * getVoxelCoordinates(cornerVoxels[0].second, chunkSize).cast<float>();
      Vec3 endPos = resolution * (cornerVoxels[7].first.cwiseProduct(chunkSize)).cast<float>() + resolution * getVoxelCoordinates(cornerVoxels[7].second, chunkSize).cast<float>();

     /* printf("startPos: %f %f %f \n",  startPos(0), startPos(1), startPos(2));
      printf("Pos: %f %f %f \n",  voxelPos(0), voxelPos(1), voxelPos(2));
      printf("endPos: %f %f %f \n \n",  endPos(0), endPos(1), endPos(2));*/

      const int x_0 = startPos(0);
      const int y_0 = startPos(1);
      const int z_0 = startPos(2);
      const int x_1 = endPos(0);
      const int y_1 = endPos(1);
      const int z_1 = endPos(2);

     // printf("Getting distvoxels \n");
     // clock_t begin = clock();


      const DistVoxel v_000 = getDistVoxelFromMap(cornerVoxels[0].first, cornerVoxels[0].second, chunks);
      const DistVoxel v_001 = getDistVoxelFromMap(cornerVoxels[1].first, cornerVoxels[1].second, chunks);
      const DistVoxel v_011 = getDistVoxelFromMap(cornerVoxels[2].first, cornerVoxels[2].second, chunks);
      const DistVoxel v_111 = getDistVoxelFromMap(cornerVoxels[3].first, cornerVoxels[3].second, chunks);
      const DistVoxel v_110 = getDistVoxelFromMap(cornerVoxels[4].first, cornerVoxels[4].second, chunks);
      const DistVoxel v_100 = getDistVoxelFromMap(cornerVoxels[5].first, cornerVoxels[5].second, chunks);
      const DistVoxel v_010 = getDistVoxelFromMap(cornerVoxels[6].first, cornerVoxels[6].second, chunks);
      const DistVoxel v_101 = getDistVoxelFromMap(cornerVoxels[7].first, cornerVoxels[7].second, chunks);

     /* clock_t end = clock();
      double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
      printf("\n \n  Time needed for searching corner voxels: %f  ms  \n \n", elapsed_secs*1000);*/

      float xd = (x - x_0) / (x_1 - x_0);
      float yd = (y - y_0) / (y_1 - y_0);
      float zd = (z - z_0) / (z_1 - z_0);

      {
          float sdf_00 = v_000.GetSDF() * (1 - xd) + v_100.GetSDF() * xd;
          float sdf_10 = v_010.GetSDF() * (1 - xd) + v_110.GetSDF() * xd;
          float sdf_01 = v_001.GetSDF() * (1 - xd) + v_101.GetSDF() * xd;
          float sdf_11 = v_011.GetSDF() * (1 - xd) + v_111.GetSDF() * xd;
          float sdf_0 = sdf_00 * (1 - yd) + sdf_10 * yd;
          float sdf_1 = sdf_01 * (1 - yd) + sdf_11 * yd;
          voxel->SetSDF(sdf_0 * (1 - zd) + sdf_1 * zd);
      }

      {
          float weight_00 = v_000.GetWeight() * (1 - xd) + v_100.GetWeight() * xd;
          float weight_10 = v_010.GetWeight() * (1 - xd) + v_110.GetWeight() * xd;
          float weight_01 = v_001.GetWeight() * (1 - xd) + v_101.GetWeight() * xd;
          float weight_11 = v_011.GetWeight() * (1 - xd) + v_111.GetWeight() * xd;
          float weight_0 = weight_00 * (1 - yd) + weight_10 * yd;
          float weight_1 = weight_01 * (1 - yd) + weight_11 * yd;
          voxel->SetWeight(weight_0 * (1 - zd) + weight_1 * zd);
      }

      return true;
  }


  Point3 Chisel::getVoxelCoordinates(VoxelID id, Eigen::Vector3i chunkSize)
  {
    int x = id % chunkSize(0);

    int fraction = ((id-x)/chunkSize(0));

    int y = fraction % chunkSize(2);
    int z = (fraction-y)/chunkSize(2);

    return Point3(x,y,z);
  }


  void Chisel::getVoxelCornerIDs(std::vector<std::pair<ChunkID, VoxelID>>* cornerVoxel, const ChunkID& originalID, const Point3& originalVoxelCoords, const Eigen::Vector3i& chunkSize)
  {
    cornerVoxel->at(0) = fixVoxelIDs(originalID, originalVoxelCoords, chunkSize);
    cornerVoxel->at(1) = fixVoxelIDs(originalID, originalVoxelCoords+Point3(0,0,1), chunkSize);
    cornerVoxel->at(2) = fixVoxelIDs(originalID, originalVoxelCoords+Point3(0,1,1), chunkSize);
    cornerVoxel->at(3) = fixVoxelIDs(originalID, originalVoxelCoords+Point3(1,1,1), chunkSize);
    cornerVoxel->at(4) = fixVoxelIDs(originalID, originalVoxelCoords+Point3(1,1,0), chunkSize);
    cornerVoxel->at(5) = fixVoxelIDs(originalID, originalVoxelCoords+Point3(1,0,0), chunkSize);
    cornerVoxel->at(6) = fixVoxelIDs(originalID, originalVoxelCoords+Point3(0,1,0), chunkSize);
    cornerVoxel->at(7) = fixVoxelIDs(originalID, originalVoxelCoords+Point3(1,0,1), chunkSize);
  }


  std::pair<ChunkID, VoxelID> Chisel::fixVoxelIDs(const ChunkID& originalID, const Point3& originalVoxelCoords, const Eigen::Vector3i& chunkSize)
  {
    Point3 voxelCoords(originalVoxelCoords);
    ChunkID chunkID(originalID);

    //check x chunk id and voxelCoordinate
    if(originalVoxelCoords(0)<0)
    {
      voxelCoords(0) = chunkSize(0) + originalVoxelCoords(0);
      chunkID(0) -= 1;
    }
    else if (originalVoxelCoords(0) >= chunkSize(0))
    {
      voxelCoords(0) = originalVoxelCoords(0) - chunkSize(0);
      chunkID(0) += 1;
    }

    //check y chunk id and voxelCoordinate
    if(originalVoxelCoords(1)<0)
    {
      voxelCoords(1) = chunkSize(1) + originalVoxelCoords(1);
      chunkID(1) -= 1;
    }
    else if (originalVoxelCoords(1) >= chunkSize(1))
    {
      voxelCoords(1) = originalVoxelCoords(1) - chunkSize(1);
      chunkID(1) += 1;
    }

    //check z chunk id and voxelCoordinate
    if(originalVoxelCoords(2)<0)
    {
      voxelCoords(2) = chunkSize(2) + originalVoxelCoords(2);
      chunkID(2) -= 1;
    }
    else if (originalVoxelCoords(2) >= chunkSize(2))
    {
      voxelCoords(2) = originalVoxelCoords(2) - chunkSize(2);
      chunkID(2) += 1;
    }

    VoxelID voxelID = voxelCoords(0) * voxelCoords(1) * voxelCoords(2);

    if (voxelID <0 || voxelID > chunkSize(0)*chunkSize(1)*chunkSize(2))
    {
      printf("Chunk Size: %d %d %d \n", chunkSize(0), chunkSize(1), chunkSize(2)) ;
      printf("original coords: %d %d %d \n", originalVoxelCoords(0), originalVoxelCoords(1), originalVoxelCoords(2));
      printf("invalid coords: %d %d %d \n", voxelCoords(0), voxelCoords(1), voxelCoords(2));

      printf("original chunk ID: %d %d %d \n", originalID(0), originalID(1), originalID(2));
      printf("invalid chunkID: %d %d %d \n", chunkID(0), chunkID(1), chunkID(2));

    }

    return std::pair<ChunkID, VoxelID>(chunkID, voxelID);
  }

  DistVoxel Chisel::getDistVoxelFromMap(const ChunkID chunkID, const VoxelID voxelID, const ChunkMap chunks)
  {
     ChunkMap::const_iterator chunk_it = chunks.find (chunkID);

     if ( chunk_it != chunks.end() )
     {
       return chunk_it->second->GetDistVoxel(voxelID);
     }
     else
     {
      DistVoxel d;
      return d;
     }

  }

} // namespace chisel 
