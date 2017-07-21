/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "device.hpp"
//#include <boost/graph/buffer_concepts.hpp>
//~ #include "pcl/gpu/utils/device/block.hpp"
//~ #include "pcl/gpu/utils/device/warp.hpp"
//#include "pcl/gpu/utils/device/vector_math.hpp"

#include "thrust/device_ptr.h"
#include "thrust/scan.h"

#include "templated_extract.cuh"

namespace pcl
{
  namespace device
  {
    namespace kinfuLS
    {
      //texture<int, 1, cudaReadModeElementType> edgeTex;
      texture<int, 1, cudaReadModeElementType> triTex;
      texture<int, 1, cudaReadModeElementType> numVertsTex;

      void
      bindTextures (const int */*edgeBuf*/, const int *triBuf, const int *numVertsBuf)
      {
        cudaChannelFormatDesc desc = cudaCreateChannelDesc<int>();
        //cudaSafeCall(cudaBindTexture(0, edgeTex, edgeBuf, desc) );
        cudaSafeCall (cudaBindTexture (0, triTex, triBuf, desc) );
        cudaSafeCall (cudaBindTexture (0, numVertsTex, numVertsBuf, desc) );
      }
      void
      unbindTextures ()
      {
        //cudaSafeCall( cudaUnbindTexture(edgeTex) );
        cudaSafeCall ( cudaUnbindTexture (numVertsTex) );
        cudaSafeCall ( cudaUnbindTexture (triTex) );
      }
    }
  }
}

namespace pcl
{
  namespace device
  {
    namespace kinfuLS
    {

      struct TrianglesExtractor
      {
        enum
        {
          MAX_LOCAL_POINTS = 15,
          MIN_X_MARGIN = 0,
          MIN_Y_MARGIN = 0,
          MIN_Z_MARGIN = 0,
        };
        __device__ TrianglesExtractor() {}

        float tranc_dist;

          // returns the number of points extracted
        __device__ __forceinline__ int filter(const FullScan6& parent,
          const pcl::gpu::kinfuLS::tsdf_buffer& buffer, int x, int y, int z)
        {
          if (x >= (buffer.voxels_size.x - 1) || y >= (buffer.voxels_size.y - 1) || z >= (buffer.voxels_size.z - 1))
            return 0;

          float f[8];
          cube_index = computeCubeIndex (parent, x, y, z, f);

          // output triangle vertices
          const int numVerts = tex1Dfetch (numVertsTex, cube_index);

          if (numVerts != 0)
          {
            // calculate cell vertex positions
            float3 v[8];
            v[0] = getNodeCoo (parent, x, y, z);
            v[1] = getNodeCoo (parent, x + 1, y, z);
            v[2] = getNodeCoo (parent, x + 1, y + 1, z);
            v[3] = getNodeCoo (parent, x, y + 1, z);
            v[4] = getNodeCoo (parent, x, y, z + 1);
            v[5] = getNodeCoo (parent, x + 1, y, z + 1);
            v[6] = getNodeCoo (parent, x + 1, y + 1, z + 1);
            v[7] = getNodeCoo (parent, x, y + 1, z + 1);

            // find the vertices where the surface intersects the cube
            // use shared memory to avoid using local
            points[0] = vertex_interp (v[0], v[1], f[0], f[1]);
            points[1] = vertex_interp (v[1], v[2], f[1], f[2]);
            points[2] = vertex_interp (v[2], v[3], f[2], f[3]);
            points[3] = vertex_interp (v[3], v[0], f[3], f[0]);
            points[4] = vertex_interp (v[4], v[5], f[4], f[5]);
            points[5] = vertex_interp (v[5], v[6], f[5], f[6]);
            points[6] = vertex_interp (v[6], v[7], f[6], f[7]);
            points[7] = vertex_interp (v[7], v[4], f[7], f[4]);
            points[8] = vertex_interp (v[0], v[4], f[0], f[4]);
            points[9] = vertex_interp (v[1], v[5], f[1], f[5]);
            points[10] = vertex_interp (v[2], v[6], f[2], f[6]);
            points[11] = vertex_interp (v[3], v[7], f[3], f[7]);

            normal.z = -(f[0] + f[1] + f[2] + f[3] - (f[4] + f[5] + f[6] + f[7]));
            normal.y = -(f[0] + f[1] - (f[2] + f[3]) + f[4] + f[5] - (f[6] + f[7]));
            normal.x = f[1] - f[0] + f[2] - f[3] + f[5] - f[4] + f[6] - f[7];
            normal = normalized(normal);

            curvature = 0.0;
            /*
            // curvature computation code
            if (x > 0 && y > 0 && z > 0)
            {
              int weight;
              float lz,ly;
              const float lx = parent.fetch (x - 1, y    , z    , weight);
              if (weight > 0)
                ly = parent.fetch (x    , y - 1, z    , weight);
              if (weight > 0)
                lz = parent.fetch (x    , y    , z - 1, weight);

              if (weight > 0)
              {
                // in a common SDF volume, |∇F| = 1
                // but here, |∇F| = cell_size / tranc_dist
                // so normalize as |∇F * tranc_dist / cell_size| = 1
                // divide by cell_size once more, because double derivative

                float3 ddf;
                ddf.x = ((f[1] - f[0]) - (f[0] - lx)) / square_float(parent.cell_size.x);
                ddf.y = ((f[3] - f[0]) - (f[0] - ly)) / square_float(parent.cell_size.y);
                ddf.z = ((f[4] - f[0]) - (f[0] - lz)) / square_float(parent.cell_size.z);

                // compute mean curvature
                curvature = (fabs(ddf.x + ddf.y + ddf.z) / 3.0) * tranc_dist;
              }
            }
            */
          }

          return numVerts;
        }

        __device__ __forceinline__ float square_float(const float f)
        {
          return f * f;
        }

        __device__ __forceinline__ bool isFull(const FullScan6& parent, unsigned int i)
        {
          return (i >= parent.output_xyz.size);
        }

        __device__ void store(const FullScan6& parent, int offset_storage, int l)
        {
          int v = tex1Dfetch (triTex, (cube_index * 16) + l);
          float x = points[v].x;
          float y = points[v].y;
          float z = points[v].z;
          float nx = normal.x;
          float ny = normal.y;
          float nz = normal.z;
          float c = curvature;
          parent.store_point_normals_curvature (x, y, z, nx, ny, nz, c,
            parent.output_xyz.data, parent.output_normals.data, offset_storage);
        }

        __device__ __forceinline__ int
        computeCubeIndex (const FullScan6& parent,int x, int y, int z, float f[8]) const
        {
          int weight;
          f[0] = parent.fetch (x,     y,     z, weight); if (weight == 0) return 0;
          f[1] = parent.fetch (x + 1, y,     z, weight); if (weight == 0) return 0;
          f[2] = parent.fetch (x + 1, y + 1, z, weight); if (weight == 0) return 0;
          f[3] = parent.fetch (x,     y + 1, z, weight); if (weight == 0) return 0;
          f[4] = parent.fetch (x,     y,     z + 1, weight); if (weight == 0) return 0;
          f[5] = parent.fetch (x + 1, y,     z + 1, weight); if (weight == 0) return 0;
          f[6] = parent.fetch (x + 1, y + 1, z + 1, weight); if (weight == 0) return 0;
          f[7] = parent.fetch (x,     y + 1, z + 1, weight); if (weight == 0) return 0;

          // calculate flag indicating if each vertex is inside or outside isosurface
          int cubeindex;
          cubeindex = int(f[0] < isoValue());
          cubeindex += int(f[1] < isoValue()) * 2;
          cubeindex += int(f[2] < isoValue()) * 4;
          cubeindex += int(f[3] < isoValue()) * 8;
          cubeindex += int(f[4] < isoValue()) * 16;
          cubeindex += int(f[5] < isoValue()) * 32;
          cubeindex += int(f[6] < isoValue()) * 64;
          cubeindex += int(f[7] < isoValue()) * 128;

          return cubeindex;
        }

        __device__ __forceinline__ float3
        getNodeCoo (const FullScan6& parent, int x, int y, int z) const
        {
          float3 coo = make_float3 (x, y, z);
          coo += 0.5f;                 //shift to volume cell center;

          coo.x *= parent.cell_size.x;
          coo.y *= parent.cell_size.y;
          coo.z *= parent.cell_size.z;

          return coo;
        }

        __device__ __forceinline__ float3
        vertex_interp (float3 p0, float3 p1, float f0, float f1) const
        {
          float t = (isoValue() - f0) / (f1 - f0 + 1e-15f);
          float x = p0.x + t * (p1.x - p0.x);
          float y = p0.y + t * (p1.y - p0.y);
          float z = p0.z + t * (p1.z - p0.z);
          return make_float3 (x, y, z);
        }

        static __device__ __forceinline__ float isoValue() { return 0.f; }

        float3 points[12];
        float3 normal;
        float curvature;
        int cube_index;
        private:
      };

      __global__ void
      trianglesGeneratorWithNormalsKernel (const FullScan6 tg,float tranc_dist)
      {
        TrianglesExtractor extractor;
        extractor.tranc_dist = tranc_dist;
        tg.templatedExtract(extractor);
      }

      int
      generateTrianglesWithNormals (const PtrStep<short2>& volume,
        const pcl::gpu::kinfuLS::tsdf_buffer & buffer, float tranc_dist,
        DeviceArray<PointType>& output, DeviceArray<PointType>& normals,
        PtrStep<int> last_data_transfer_matrix, int & data_transfer_finished)
      {
        FullScan6 tg;

        tg.volume = volume;
        tg.cell_size.x = buffer.volume_size.x / buffer.voxels_size.x;
        tg.cell_size.y = buffer.volume_size.y / buffer.voxels_size.y;
        tg.cell_size.z = buffer.volume_size.z / buffer.voxels_size.z;
        tg.output_xyz = output;
        tg.output_normals = normals;
        tg.data_transfer_completion_matrix = last_data_transfer_matrix;
        tg.rolling_buffer = buffer;

        dim3 block (FullScan6::CTA_SIZE_X,FullScan6::CTA_SIZE_Y);
        dim3 grid (divUp (buffer.voxels_size.x, block.x), divUp (buffer.voxels_size.y, block.y));

        tg.init_globals();

        trianglesGeneratorWithNormalsKernel<<<grid, block>>>(tg,tranc_dist);
        cudaSafeCall ( cudaGetLastError () );
        cudaSafeCall (cudaDeviceSynchronize ());

        int size = tg.get_result_size(data_transfer_finished);
        return min ((int)size, int(output.size()));
      }
    }
  }
}
