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

#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/geometry/Raycast.h>

namespace chisel
{

  ProjectionIntegrator::ProjectionIntegrator()
  {
    // TODO Auto-generated constructor stub

  }

  ProjectionIntegrator::ProjectionIntegrator(const TruncatorPtr& t, const WeighterPtr& w, float crvDist, bool enableCrv, const Vec3List& centers) :
    truncator(t), weighter(w), carvingDist(crvDist), enableVoxelCarving(enableCrv), centroids(centers)
  {

  }

  bool ProjectionIntegrator::Integrate(const PointCloud& cloud, const Transform& cameraPose, Chunk* chunk) const
  {
    assert(!!chunk);

    if(cloud.HasColor() && chunk->HasColors())
      {
        return IntegrateColorPointCloud(cloud, cameraPose, chunk);
      }
    else
      {
        return IntegratePointCloud(cloud, cameraPose, chunk);
      }
  }


  bool ProjectionIntegrator::IntegratePointCloud(const PointCloud& cloud, const Transform& cameraPose, Chunk* chunk) const
  {
    const float roundX = 1.0f / chunk->GetVoxelResolutionMeters();
    const float roundY = 1.0f / chunk->GetVoxelResolutionMeters();
    const float roundZ = 1.0f / chunk->GetVoxelResolutionMeters();

    Point3List raycastVoxels;
    Point3 chunkMin = Point3::Zero();
    Point3 chunkMax = chunk->GetNumVoxels();
    bool updated = false;
    size_t i = 0;
    Vec3 startCamera = cameraPose.translation();
    Transform inversePose = cameraPose.inverse();
    for (const Vec3& point : cloud.GetPoints())
      {
        const Vec3& color = cloud.GetColors()[i];
        Vec3 worldPoint = cameraPose * point;
        float depth = point.z();
        Vec3 dir = (worldPoint - startCamera).normalized();
        float truncation = truncator->GetTruncationDistance(depth);
        Vec3 start = worldPoint - dir * truncation - chunk->GetOrigin();
        Vec3 end = worldPoint + dir * truncation - chunk->GetOrigin();
        start.x() *= roundX;
        start.y() *= roundY;
        start.z() *= roundZ;
        end.x() *= roundX;
        end.y() *= roundY;
        end.z() *= roundZ;
        raycastVoxels.clear();
        Raycast(start, end, chunkMin, chunkMax, &raycastVoxels);

        for (const Point3& voxelCoords : raycastVoxels)
          {
            VoxelID id = chunk->GetVoxelID(voxelCoords);
            DistVoxel& distVoxel = chunk->GetDistVoxelMutable(id);
            const Vec3& centroid = centroids[id] + chunk->GetOrigin();
            float u = depth - (inversePose * centroid - startCamera).z();
            float weight = weighter->GetWeight(u, truncation);
            if (fabs(u) < truncation)
              {
                distVoxel.Integrate(u, weight);
                updated = true;
              }
            else if (enableVoxelCarving && u > truncation + carvingDist)
              {
                if (distVoxel.GetWeight() > 0)
                  {
                    distVoxel.Integrate(1.0e-5, 5.0f);
                    updated = true;
                  }
              }
          }
        i++;

      }
    return updated;
  }

  bool ProjectionIntegrator::IntegrateColorPointCloud(const PointCloud& cloud, const Transform& cameraPose, Chunk* chunk) const
  {
    const float roundX = 1.0f / chunk->GetVoxelResolutionMeters();
    const float roundY = 1.0f / chunk->GetVoxelResolutionMeters();
    const float roundZ = 1.0f / chunk->GetVoxelResolutionMeters();

    Point3List raycastVoxels;
    Point3 chunkMin = Point3::Zero();
    Point3 chunkMax = chunk->GetNumVoxels();
    bool updated = false;
    size_t i = 0;
    Vec3 startCamera = cameraPose.translation();
    Transform inversePose = cameraPose.inverse();
    for (const Vec3& point : cloud.GetPoints())
      {
        //TODO const Vec3& color = cloud.GetColors()[i];
        Vec3 worldPoint = cameraPose * point;
        float depth = point.z();
        Vec3 dir = (worldPoint - startCamera).normalized();
        float truncation = truncator->GetTruncationDistance(depth);
        Vec3 start = worldPoint - dir * truncation - chunk->GetOrigin();
        Vec3 end = worldPoint + dir * truncation - chunk->GetOrigin();
        start.x() *= roundX;
        start.y() *= roundY;
        start.z() *= roundZ;
        end.x() *= roundX;
        end.y() *= roundY;
        end.z() *= roundZ;
        raycastVoxels.clear();
        Raycast(start, end, chunkMin, chunkMax, &raycastVoxels);

        for (const Point3& voxelCoords : raycastVoxels)
          {
            VoxelID id = chunk->GetVoxelID(voxelCoords);
            //TODO ColorVoxel& voxel = chunk->GetColorVoxelMutable(id);
            DistVoxel& distVoxel = chunk->GetDistVoxelMutable(id);
            const Vec3& centroid = centroids[id] + chunk->GetOrigin();
            float u = depth - (inversePose * centroid - startCamera).z();
            float weight = weighter->GetWeight(u, truncation);
            if (fabs(u) < truncation)
              {
                distVoxel.Integrate(u, weight);
                //TODO voxel.Integrate((uint8_t)(color.x() * 255.0f), (uint8_t)(color.y() * 255.0f), (uint8_t)(color.z() * 255.0f), 1);
                updated = true;
              }
            else if (enableVoxelCarving && u > truncation + carvingDist)
              {
                if (distVoxel.GetWeight() > 0)
                  {
                    distVoxel.Integrate(1.0e-5, 5.0f);
                    updated = true;
                  }
              }
          }
        i++;

      }
    return updated;
  }

  bool ProjectionIntegrator::IntegrateChunk(Chunk& chunkToIntegrate, Chunk* chunk) const{

    assert(chunk != nullptr);

    Eigen::Vector3i numVoxels = chunk->GetNumVoxels();
    float resolution = chunk->GetVoxelResolutionMeters();
    Vec3 origin = chunk->GetOrigin();
    float diag = 2.0 * sqrt(3.0f) * resolution;
    Vec3 voxelCenter;
    bool updated = false;
    for (size_t i = 0; i < centroids.size(); i++)
      {
        /* voxelCenter = centroids[i] + origin;
            Vec3 voxelCenterInCamera = cameraPose.linear().transpose() * (voxelCenter - cameraPose.translation());
            Vec3 cameraPos = camera.ProjectPoint(voxelCenterInCamera);

            if (!camera.IsPointOnImage(cameraPos) || voxelCenterInCamera.z() < 0)
                continue;

            float voxelDist = voxelCenterInCamera.z();
            float depth = depthImage->DepthAt((int)cameraPos(1), (int)cameraPos(0)); //depthImage->BilinearInterpolateDepth(cameraPos(0), cameraPos(1));
            */
        /*if(std::isnan(depth))
            {
                continue;
            }
*/
        /* float truncation = truncator->GetTruncationDistance(depth);
            float surfaceDist = depth - voxelDist;

            if (fabs(surfaceDist) < truncation + diag)
            {*/
        DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
        DistVoxel voxelToIntegrate = chunkToIntegrate.GetDistVoxelMutable(i);

        /* if (voxelToIntegrate.GetWeight() > 0 && voxelToIntegrate.GetSDF() > 1e-4){*/
        voxel.Integrate(voxelToIntegrate.GetSDF(), voxelToIntegrate.GetWeight());
        updated = true;
        //  }

        /* if (voxelToIntegrate.GetWeight() > 0 && voxelToIntegrate.GetSDF() < 1e-5)
                {
                    voxel.Carve();
                    updated = true;
                }*/
        /* }
            else if (enableVoxelCarving && surfaceDist > truncation + carvingDist)
            {
                DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
                if (voxel.GetWeight() > 0 && voxel.GetSDF() < 1e-5)
                {
                    voxel.Carve();
                    updated = true;
                }
            }*/


      }
    return updated;


    return true;
  }


  ProjectionIntegrator::~ProjectionIntegrator()
  {
    // TODO Auto-generated destructor stub
  }

} // namespace chisel 
