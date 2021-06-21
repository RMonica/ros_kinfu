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


namespace pcl
{
  namespace device
  {
    namespace kinfuLS
    {

    /* ************************** */
      __device__ __forceinline__ float3
      float3_mul_elements(const float3 & a,const float3 & b)
      {
        return make_float3(a.x * b.x,a.y * b.y,a.z * b.z);
      }

      __device__ bool evaluateRaycastFilter(RaycastFilter filter,float3 pt)
      {
        if (filter.has_bbox)
        {
          if (pt.x < filter.bbox_min.x || pt.y < filter.bbox_min.y || pt.z < filter.bbox_min.z ||
              pt.x >= filter.bbox_max.x || pt.y >= filter.bbox_max.y || pt.z >= filter.bbox_max.z)
            return false;
        }

        if (filter.has_sphere)
        {
          float3 diff = pt - filter.sphere_center;
          float sqnorm = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
          if (sqnorm > filter.sphere_radius * filter.sphere_radius)
            return false;
        }

        return true;
      }

      template <class StoreCondition,class StoreAction,class SearchCondition>
      struct RayCaster
      {
        enum { CTA_SIZE_X = 32, CTA_SIZE_Y = 8 };

        Mat33 Rcurr;
        float3 tcurr;

        float time_step;
        float3 volume_size;
        int3 voxels_size;
        int3 voxels_volume_padding;

        float3 cell_size;
        float3 cell_size_inv;
        int cols, rows;

        mutable SearchCondition search_condition;
        mutable StoreCondition store_condition;
        mutable StoreAction store_action;

        PtrStep<short2> volume;

        float min_range;

        Intr intr;

        mutable PtrStep<float> vmap;

        __device__ __forceinline__ float3
        get_ray_next (int x, int y) const
        {
          float3 ray_next;
          ray_next.x = (x - intr.cx) / intr.fx;
          ray_next.y = (y - intr.cy) / intr.fy;
          ray_next.z = 1;
          return ray_next;
        }

        __device__ __forceinline__ bool
        checkInds (const int3& g) const
        {
          return (g.x >= 0 && g.y >= 0 && g.z >= 0 && g.x < voxels_size.x && g.y < voxels_size.y && g.z < voxels_size.z);
        }

        __device__ __forceinline__ bool
        checkSafeInds (const int3& g) const
        {
          return (g.x >= voxels_volume_padding.x && g.y >= voxels_volume_padding.y && g.z >= voxels_volume_padding.z &&
                  g.x < voxels_size.x - voxels_volume_padding.x &&
                  g.y < voxels_size.y - voxels_volume_padding.y &&
                  g.z < voxels_size.z - voxels_volume_padding.z);
        }

        __device__ __forceinline__ void
        shift_coords (int & x, int & y, int & z, const pcl::gpu::kinfuLS::tsdf_buffer & buffer) const
        {
          x += buffer.origin_GRID.x;
          y += buffer.origin_GRID.y;
          z += buffer.origin_GRID.z;
          if (x >= buffer.voxels_size.x)
            x -= buffer.voxels_size.x;
          if (y >= buffer.voxels_size.y)
            y -= buffer.voxels_size.y;
          if (z >= buffer.voxels_size.z)
            z -= buffer.voxels_size.z;
        }

        __device__ __forceinline__ float
        readTsdf (int x, int y, int z, const pcl::gpu::kinfuLS::tsdf_buffer & buffer) const
        {
          shift_coords(x,y,z,buffer);
          const short2* pos = &(volume.ptr (buffer.voxels_size.y * z + y)[x]);
          return unpack_tsdf (*pos);
        }

        __device__ __forceinline__ void
        readTsdf (int x, int y, int z, const pcl::gpu::kinfuLS::tsdf_buffer & buffer,float& tsdf, int& weight) const
        {
          shift_coords(x,y,z,buffer);
          const short2* pos = &(volume.ptr (buffer.voxels_size.y * z + y)[x]);
          unpack_tsdf (*pos,tsdf,weight);
        }

        __device__ __forceinline__ float3
        fromMetersToCells (const float3 & point) const
        {
          return float3_mul_elements(point,cell_size_inv);
        }

        __device__ __forceinline__ float3
        fromCellsToMeters (const float3 & point) const
        {
          return float3_mul_elements(point,cell_size);
        }

        __device__ __forceinline__ int3
        getVoxelFromPoint (float3 point) const
        {
          return getVoxelFromCell(fromMetersToCells(point));
        }

        __device__ __forceinline__ int3
        getVoxelFromCell (const float3 & cell) const
        {
          int vx = __float2int_rd (cell.x);        // round to negative infinity
          int vy = __float2int_rd (cell.y);
          int vz = __float2int_rd (cell.z);

          return make_int3 (vx, vy, vz);
        }

        __device__ __forceinline__ float
        interpolateTrilineary (const float3& point, const pcl::gpu::kinfuLS::tsdf_buffer & buffer) const
        {
          const float3 cell = fromMetersToCells (point);
          return interpolateTrilinearyFromCell (cell,buffer);
        }

        __device__ __forceinline__ float
        interpolateTrilinearyFromCell (const float3& cell, const pcl::gpu::kinfuLS::tsdf_buffer & buffer) const
        {
          int3 g = getVoxelFromCell (cell);

          if (g.x <= 0 || g.x >= buffer.voxels_size.x - 1)
            return numeric_limits<float>::quiet_NaN ();

          if (g.y <= 0 || g.y >= buffer.voxels_size.y - 1)
            return numeric_limits<float>::quiet_NaN ();

          if (g.z <= 0 || g.z >= buffer.voxels_size.z - 1)
            return numeric_limits<float>::quiet_NaN ();

  /*      //OLD CODE
          float vx = (g.x + 0.5f) * cell_size.x;
          float vy = (g.y + 0.5f) * cell_size.y;
          float vz = (g.z + 0.5f) * cell_size.z;

          g.x = (point.x < vx) ? (g.x - 1) : g.x;
          g.y = (point.y < vy) ? (g.y - 1) : g.y;
          g.z = (point.z < vz) ? (g.z - 1) : g.z;

          float a = (point.x - (g.x + 0.5f) * cell_size.x) / cell_size.x;
          float b = (point.y - (g.y + 0.5f) * cell_size.y) / cell_size.y;
          float c = (point.z - (g.z + 0.5f) * cell_size.z) / cell_size.z;

          float res = readTsdf (g.x + 0, g.y + 0, g.z + 0, buffer) * (1 - a) * (1 - b) * (1 - c) +
                      readTsdf (g.x + 0, g.y + 0, g.z + 1, buffer) * (1 - a) * (1 - b) * c +
                      readTsdf (g.x + 0, g.y + 1, g.z + 0, buffer) * (1 - a) * b * (1 - c) +
                      readTsdf (g.x + 0, g.y + 1, g.z + 1, buffer) * (1 - a) * b * c +
                      readTsdf (g.x + 1, g.y + 0, g.z + 0, buffer) * a * (1 - b) * (1 - c) +
                      readTsdf (g.x + 1, g.y + 0, g.z + 1, buffer) * a * (1 - b) * c +
                      readTsdf (g.x + 1, g.y + 1, g.z + 0, buffer) * a * b * (1 - c) +
                      readTsdf (g.x + 1, g.y + 1, g.z + 1, buffer) * a * b * c;
  */
          //NEW CODE
          float a = cell.x - (g.x + 0.5f); if (a<0) { g.x--; a+=1.0f; };
          float b = cell.y - (g.y + 0.5f); if (b<0) { g.y--; b+=1.0f; };
          float c = cell.z - (g.z + 0.5f); if (c<0) { g.z--; c+=1.0f; };

          float res = (1 - a) * (
                                  (1 - b) * (
                                          readTsdf (g.x + 0, g.y + 0, g.z + 0, buffer) * (1 - c) +
                                          readTsdf (g.x + 0, g.y + 0, g.z + 1, buffer) *      c 
                                          )
                                  + b * (
                                          readTsdf (g.x + 0, g.y + 1, g.z + 0, buffer) * (1 - c) +
                                          readTsdf (g.x + 0, g.y + 1, g.z + 1, buffer) *      c  
                                          )
                                  )
                          + a * (
                                  (1 - b) * (
                                          readTsdf (g.x + 1, g.y + 0, g.z + 0, buffer) * (1 - c) +
                                          readTsdf (g.x + 1, g.y + 0, g.z + 1, buffer) *      c 
                                          )
                                  + b * (
                                          readTsdf (g.x + 1, g.y + 1, g.z + 0, buffer) * (1 - c) +
                                          readTsdf (g.x + 1, g.y + 1, g.z + 1, buffer) *      c 
                                          )
                                  )
                                          ;
          return res;
        }

        __device__ void find_min_max_time(float3 ray_org, float3 ray_dir, float3 box_max, float &tnear, float &tfar) const
        {
          const float3 box_min = make_float3(0.f, 0.f, 0.f);

          // compute intersection of ray with all six bbox planes
          float3 invR = make_float3(1.f/ray_dir.x, 1.f/ray_dir.y, 1.f/ray_dir.z);
          float3 tbot = float3_mul_elements(invR,box_min - ray_org);
          float3 ttop = float3_mul_elements(invR,box_max - ray_org);

          // re-order intersections to find smallest and largest on each axis
          float3 tmin = make_float3(fminf(ttop.x, tbot.x), fminf(ttop.y, tbot.y), fminf(ttop.z, tbot.z));
          float3 tmax = make_float3(fmaxf(ttop.x, tbot.x), fmaxf(ttop.y, tbot.y), fmaxf(ttop.z, tbot.z));

          // find the largest tmin and the smallest tmax
          tnear = fmaxf(fmaxf(tmin.x, tmin.y), fmaxf(tmin.x, tmin.z));
          tfar  = fminf(fminf(tmax.x, tmax.y), fminf(tmax.x, tmax.z));
        }

        __device__
        bool computeInterpolatedVertexAndNormal(const float3 & world_pt_prev,const float3 & world_pt,
                                                float time_curr,float time_step,
                                                const float3 & ray_start,const float3 & ray_dir,
                                                pcl::gpu::kinfuLS::tsdf_buffer & buffer,
                                                float3 & position_out, float3 & normal_out) const
        {
          float step_correction = 0.5;

          float Ftdt = interpolateTrilineary (world_pt, buffer);
          if (isnan (Ftdt))
            return false;

          float Ft = interpolateTrilineary (world_pt_prev, buffer);
          if (isnan (Ft))
            return false;

          if (abs(Ftdt - Ft) > 0.1)
            step_correction = __fdividef(Ft,Ftdt - Ft);

          float Ts = time_curr - time_step * step_correction;

          float3 vetex_found = ray_start + ray_dir * Ts;

          position_out = vetex_found;

          float3 t;
          float3 n;

          t = vetex_found;
          t.x += cell_size.x;
          float Fx1 = interpolateTrilineary (t, buffer);

          t = vetex_found;
          t.x -= cell_size.x;
          float Fx2 = interpolateTrilineary (t, buffer);

          n.x = (Fx1 - Fx2);

          t = vetex_found;
          t.y += cell_size.y;
          float Fy1 = interpolateTrilineary (t, buffer);

          t = vetex_found;
          t.y -= cell_size.y;
          float Fy2 = interpolateTrilineary (t, buffer);

          n.y = (Fy1 - Fy2);

          t = vetex_found;
          t.z += cell_size.z;
          float Fz1 = interpolateTrilineary (t, buffer);

          t = vetex_found;
          t.z -= cell_size.z;
          float Fz2 = interpolateTrilineary (t, buffer);

          n.z = (Fz1 - Fz2);

          n = normalized (n);

          normal_out = n;
          return true;
        }


        __device__ __forceinline__ void
        operator () (pcl::gpu::kinfuLS::tsdf_buffer buffer) const
        {
          int x = threadIdx.x + blockIdx.x * CTA_SIZE_X;
          int y = threadIdx.y + blockIdx.y * CTA_SIZE_Y;

          if (x >= cols || y >= rows)
            return;

          store_action.Init(*this,x,y);

          const float3 ray_start = tcurr;
          float3 norm_ray_next = normalized (get_ray_next (x, y));
          float3 ray_dir = normalized (Rcurr * get_ray_next (x, y));

          //ensure that it isn't a degenerate case
          ray_dir.x = (ray_dir.x == 0.f) ? 1e-15 : ray_dir.x;
          ray_dir.y = (ray_dir.y == 0.f) ? 1e-15 : ray_dir.y;
          ray_dir.z = (ray_dir.z == 0.f) ? 1e-15 : ray_dir.z;

          // computer time when entry and exit volume
          float time_start_volume;
          float time_exit_volume;
          find_min_max_time(ray_start,ray_dir,volume_size,time_start_volume,time_exit_volume);

          const float min_dist = 0.f;         //in meters
          time_start_volume = fmax (time_start_volume, min_dist);
          if (time_start_volume >= time_exit_volume)
            return;
          time_exit_volume -= time_step;

          float time_curr = time_start_volume;
          int3 g = getVoxelFromPoint (ray_start + ray_dir * time_curr);
          g.x = max (0, min (g.x, buffer.voxels_size.x - 1));
          g.y = max (0, min (g.y, buffer.voxels_size.y - 1));
          g.z = max (0, min (g.z, buffer.voxels_size.z - 1));

          float tsdf;
          int weight;
          readTsdf (g.x, g.y, g.z, buffer, tsdf, weight);

          //infinite loop guard
          const float max_time = min(time_exit_volume,3.0 * (volume_size.x + volume_size.y + volume_size.z));
          const float min_time_step = min(cell_size.x,min(cell_size.y,cell_size.z));

          float curr_time_step = time_step;
          float time_step_prev;

          float tsdf_prev = tsdf;
          int weight_prev = weight;

          bool zero_crossing = false;

          for (; time_curr < max_time; time_curr += curr_time_step)
          {
            tsdf_prev = tsdf;
            weight_prev = weight;
            time_step_prev = curr_time_step;

            const float3 world_pt = ray_start + ray_dir * (time_curr + curr_time_step);
            if (!search_condition.Evaluate(world_pt))
              continue;

            g = getVoxelFromPoint (world_pt);
            if (!checkInds (g))
              return;

            if (!checkSafeInds(g))
              continue;

            readTsdf (g.x, g.y, g.z, buffer, tsdf, weight);

            {
              float new_time_step = curr_time_step;
              bool rewind = false;
              if (store_condition.ChangeTimeStep(tsdf_prev,tsdf,weight_prev,weight,time_step,min_time_step,
                                                 curr_time_step,new_time_step,rewind))
              {
                if (new_time_step > min_time_step)
                {
                  if (rewind)
                  {
                    tsdf = tsdf_prev;
                    weight = weight_prev;

                    time_curr -= curr_time_step;
                  }
                  curr_time_step = new_time_step;
                  if (rewind)
                    continue;
                }
              }
            }

            if (tsdf_prev < 0.f && tsdf > 0.f)
              return;

            bool below_min_range = (time_curr * norm_ray_next.z) < min_range;
            zero_crossing = store_condition.Evaluate(tsdf_prev,tsdf,weight_prev,weight,world_pt,below_min_range);
            if (zero_crossing && below_min_range)
              return;

            if (zero_crossing)
              break; // break out of the cycle here, so Stores will be executed in sync by all threads
          }

          if (zero_crossing)
          {
            const float3 world_pt_prev = ray_start + ray_dir * (time_curr);
            const float3 world_pt = ray_start + ray_dir * (time_curr + time_step_prev);

            store_action.Store(world_pt_prev,world_pt,g,tsdf_prev,tsdf,weight_prev,weight,time_curr,time_step,
                               ray_start,ray_dir,*this,x,y,buffer);
          }
        }
      };

      struct SphereSearchCondition
      {
        SphereSearchCondition () {}
        SphereSearchCondition (const float3 & c,const float & r): sphere_center(c), sphere_radius(r) {}

        __device__ __forceinline__ bool Evaluate (const float3 & pt) const
          {
          const float xx = (sphere_center.x - pt.x);
          const float yy = (sphere_center.y - pt.y);
          const float zz = (sphere_center.z - pt.z);
          return xx * xx + yy * yy + zz * zz < sphere_radius * sphere_radius;
          }

        float3 sphere_center;
        float sphere_radius;
      };

      struct BBoxSearchCondition
      {
        BBoxSearchCondition () {}
        BBoxSearchCondition (const float3 & m,const float3 & M): bbox_min(m), bbox_max(M) {}

        __device__ __forceinline__ bool Evaluate (const float3 & pt) const
          {
          return (pt.x >= bbox_min.x && pt.y >= bbox_min.y && pt.z >= bbox_min.z &&
            pt.x < bbox_max.x && pt.y < bbox_max.y && pt.z < bbox_max.z);
          }

        float3 bbox_min;
        float3 bbox_max;
      };

      struct TrueSearchCondition
      {
        __device__ __forceinline__ bool Evaluate (const float3 & /*pt*/) const {return true; }
      };

      struct ZeroCrossingStoreCondition
      {
        __device__ __forceinline__ bool Evaluate(float tsdf_prev,float tsdf_curr,int /*weight_prev*/,int /*weight_curr*/,
                                                 float3 /*world_pt*/,bool /*below_min_range*/)
        {
          return (tsdf_prev > 0.0f && tsdf_curr < 0.0f) || (tsdf_prev < 0.f && tsdf_curr > 0.f);
        }

        __device__ __forceinline__ bool ChangeTimeStep(float /*tsdf_prev*/,float /*tsdf*/,int /*weight_prev*/,int /*weight_curr*/,
          float /*orig_time_step*/,float /*min_time_step*/,float /*time_step*/,float & /*new_time_step*/,bool & /*rewind*/)
        {
          return false;
        }
      };

      struct ZeroCrossingOrUnknownStoreCondition
      {
        __device__ __forceinline__ bool Evaluate(float tsdf_prev,float tsdf_curr,int /*weight_prev*/,int weight_curr,
                                                 float3 /*world_pt*/,bool /*below_min_range*/)
        {
          return (tsdf_prev > 0.0f && tsdf_curr < 0.0f) || (tsdf_prev < 0.f && tsdf_curr > 0.f) || (weight_curr == 0);
        }

        __device__ __forceinline__ bool ChangeTimeStep(float /*tsdf_prev*/,float /*tsdf*/,int /*weight_prev*/,int /*weight_curr*/,
          float /*orig_time_step*/,float /*min_time_step*/,float /*time_step*/,float & /*new_time_step*/,bool & /*rewind*/)
        {
          return false;
        }
      };

      struct NotEmptyStoreCondition
      {
        __device__ __forceinline__ bool Evaluate(float /*tsdf_prev*/,float tsdf_curr,int /*weight_prev*/,int weight_curr,
                                                 float3 /*world_pt*/,bool /*below_min_range*/)
        {
          return tsdf_curr < 0.0f || weight_curr == 0;
        }

        __device__ __forceinline__ bool ChangeTimeStep(float /*tsdf_prev*/,float /*tsdf*/,int weight_prev,int weight_curr,
          float /*orig_time_step*/,float min_time_step,float time_step,float & new_time_step,bool & rewind)
        {
          if (weight_curr == 0 && weight_prev != 0 && (time_step / 2.0f > min_time_step))
          {
            new_time_step = time_step / 2.0f;
            rewind = true;
            return true;
          }
          return false;
        }
      };

      struct NotEmptyStoreConditionUnkFiltered
      {
        __device__ __forceinline__ bool Evaluate(float /*tsdf_prev*/,float tsdf_curr,int /*weight_prev*/,int weight_curr,
                                                 float3 world_pt,bool below_min_range)
        {
          if (tsdf_curr < 0.0f)
            return true;
          if (weight_curr == 0 && below_min_range)
            return true;
          if (weight_curr == 0 && evaluateRaycastFilter(filter, world_pt))
            return true;
          return false;
        }

        __device__ __forceinline__ bool ChangeTimeStep(float /*tsdf_prev*/,float /*tsdf*/,int weight_prev,int weight_curr,
          float orig_time_step,float min_time_step,float time_step,float & new_time_step,bool & rewind)
        {
          if (weight_curr == 0 && weight_prev != 0 && (time_step / 2.0f > min_time_step))
          {
            new_time_step = time_step / 2.0f;
            rewind = true;
            return true;
          }
          if (weight_curr == 0 && weight_prev != 0 && (time_step / 2.0f <= min_time_step))
          {
            new_time_step = orig_time_step;
            return true; // do not rewind
          }
          return false;
        }

        RaycastFilter filter;
      };

      struct InterpolatePointAndNormalStoreAction
      {
        template <class _RayCaster>
        __device__ __forceinline__ void Init(_RayCaster & parent,int x,int y)
        {
          parent.vmap.ptr (y)[x] = numeric_limits<float>::quiet_NaN ();
          nmap.ptr (y)[x] = numeric_limits<float>::quiet_NaN ();
        }

        template <class _RayCaster>
        __device__ __forceinline__ void Store(const float3 & world_pt_prev,const float3 & world_pt,
          const int3 & /*voxel_id*/,float tsdf_prev,float tsdf,float /*weight_prev*/,float weight,
          float time_curr,float time_step,const float3 & ray_start,const float3 & ray_dir,
          const _RayCaster & parent,int x,int y,pcl::gpu::kinfuLS::tsdf_buffer & buffer)
        {
          if (weight == 0)
            return;

          if (tsdf_prev < 0.f && tsdf > 0.f)
            return; // crossing surface from wrong side

          float3 vetex_found;
          float3 n;

          bool ok = parent.computeInterpolatedVertexAndNormal(world_pt_prev, world_pt, time_curr, time_step,
                                                              ray_start, ray_dir, buffer, vetex_found, n);
          if (!ok)
            return;

          parent.vmap.ptr (y       )[x] = vetex_found.x;
          parent.vmap.ptr (y + parent.rows)[x] = vetex_found.y;
          parent.vmap.ptr (y + 2 * parent.rows)[x] = vetex_found.z;

          nmap.ptr (y       )[x] = n.x;
          nmap.ptr (y + parent.rows)[x] = n.y;
          nmap.ptr (y + 2 * parent.rows)[x] = n.z;
        }

        PtrStep<float> nmap;
      };

      struct InterpolatePointAndNormalWithUnknownStoreAction
      {
        template <class _RayCaster>
        __device__ __forceinline__ void Init(_RayCaster & parent,int x,int y)
        {
          parent.vmap.ptr (y)[x] = numeric_limits<float>::quiet_NaN ();
          nmap.ptr (y)[x] = numeric_limits<float>::quiet_NaN ();
        }

        template <class _RayCaster>
        __device__ __forceinline__ void Store(const float3 & world_pt_prev,const float3 & world_pt,
          const int3 & /*voxel_id*/,float tsdf_prev,float tsdf,float /*weight_prev*/,float weight,
          float time_curr,float time_step,const float3 & ray_start,const float3 & ray_dir,
          const _RayCaster & parent,int x,int y,pcl::gpu::kinfuLS::tsdf_buffer & buffer)
        {
          float3 vetex_found;
          float3 n;
          float u;
          if (weight == 0 || (tsdf_prev < 0.f && tsdf > 0.f))
          {
            u = -time_curr; // unknown

            vetex_found = world_pt;
            n.x = n.y = n.z = 0.0f;
          }
          else
          {
            u = time_curr; // occupied

            bool ok = parent.computeInterpolatedVertexAndNormal(world_pt_prev, world_pt, time_curr, time_step,
                                                                ray_start, ray_dir, buffer, vetex_found, n);
            if (!ok)
              return;
          }

          umap.ptr(y)[x] = u;

          parent.vmap.ptr (y       )[x] = vetex_found.x;
          parent.vmap.ptr (y + parent.rows)[x] = vetex_found.y;
          parent.vmap.ptr (y + 2 * parent.rows)[x] = vetex_found.z;

          nmap.ptr (y       )[x] = n.x;
          nmap.ptr (y + parent.rows)[x] = n.y;
          nmap.ptr (y + 2 * parent.rows)[x] = n.z;
        }

        PtrStep<float> nmap;
        PtrStep<float> umap;
      };

      enum // bitmask for STORE_POSE
      {
        STORE_POSE_NONE = 0,
        STORE_POSE_WORLD = 1,
        STORE_POSE_VOXEL = 2,
      };

      template <int STORE_POSE>
      struct SignedSensorDistanceStoreAction
      {
        template <class _RayCaster>
        __device__ __forceinline__ void Init(_RayCaster & parent,int x,int y)
        {
          if (STORE_POSE & STORE_POSE_WORLD)
            parent.vmap.ptr (y)[x] = numeric_limits<float>::quiet_NaN ();
          if (STORE_POSE & STORE_POSE_VOXEL)
            voxel_map.ptr (y + parent.rows)[x] = -1;
          umap.ptr (y)[x] = 0.0; // empty
        }

        template <class _RayCaster>
        __device__ __forceinline__ void Store(const float3 & /*world_pt_prev*/,const float3 & world_pt,
          const int3 & voxel_id,float /*tsdf_prev*/,float /*tsdf*/,float /*weight_prev*/,float weight,
          float time_curr,float /*time_step*/,const float3 & /*ray_start*/,const float3 & /*ray_dir*/,
          const _RayCaster & parent,int x,int y,pcl::gpu::kinfuLS::tsdf_buffer & buffer)
        {
          if (!evaluateRaycastFilter(filter, world_pt) && weight == 0)
            return;

          if (STORE_POSE & STORE_POSE_WORLD)
          {
            parent.vmap.ptr (y       )[x] = world_pt.x;
            parent.vmap.ptr (y + parent.rows)[x] = world_pt.y;
            parent.vmap.ptr (y + 2 * parent.rows)[x] = world_pt.z;
          }

          if (STORE_POSE & STORE_POSE_VOXEL)
          {
            voxel_map.ptr (y       )[x] = voxel_id.x;
            voxel_map.ptr (y + parent.rows)[x] = voxel_id.y;
            voxel_map.ptr (y + 2 * parent.rows)[x] = voxel_id.z;
          }

          if (weight == 0)
            umap.ptr(y)[x] = -time_curr; // unknown
          else
            umap.ptr(y)[x] = time_curr; // occupied
        }

        PtrStep<float> umap; // intensity values
        PtrStep<int> voxel_map;
        RaycastFilter filter;
      };

      typedef SignedSensorDistanceStoreAction<STORE_POSE_NONE>
        SignedSensorDistanceNoPoseStoreAction;
      typedef SignedSensorDistanceStoreAction<STORE_POSE_WORLD>
        SignedSensorDistanceWithPoseStoreAction;
      typedef SignedSensorDistanceStoreAction<STORE_POSE_VOXEL>
        SignedSensorDistanceWithVoxelIndexStoreAction;

      template <class StoreCondition,class StoreAction,class SearchCondition>
      __global__ void
      rayCastKernel (const RayCaster<StoreCondition,StoreAction,SearchCondition> rc, pcl::gpu::kinfuLS::tsdf_buffer buffer) {
        rc (buffer);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      template <class StoreCondition,class StoreAction,class SearchCondition>
      void
      templatedRaycast (const Intr& intr, const Mat33& Rcurr, const float3& tcurr,
                            float tranc_dist, float min_range, const float3& volume_size,
                            const PtrStep<short2>& volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer,MapArr& vmap,
                            const StoreCondition & store_condition,const StoreAction & store_action,
                            const SearchCondition & search_condition)
      {
        RayCaster<StoreCondition,StoreAction,SearchCondition> rc;

        rc.Rcurr = Rcurr;
        rc.tcurr = tcurr;

        rc.time_step = tranc_dist * 0.8f;

        rc.volume_size = volume_size;
        rc.voxels_size = buffer->voxels_size;
        rc.voxels_volume_padding = buffer->voxels_volume_padding;

        rc.cell_size.x = volume_size.x / buffer->voxels_size.x;
        rc.cell_size.y = volume_size.y / buffer->voxels_size.y;
        rc.cell_size.z = volume_size.z / buffer->voxels_size.z;
        rc.cell_size_inv.x = 1.0 / rc.cell_size.x;
        rc.cell_size_inv.y = 1.0 / rc.cell_size.y;
        rc.cell_size_inv.z = 1.0 / rc.cell_size.z;

        rc.cols = vmap.cols ();
        rc.rows = vmap.rows () / 3;

        rc.intr = intr;

        rc.min_range = min_range;

        rc.volume = volume;
        rc.vmap = vmap;

        rc.search_condition = search_condition;
        rc.store_action = store_action;
        rc.store_condition = store_condition;

        dim3 block (RayCaster<StoreCondition,StoreAction,SearchCondition>::CTA_SIZE_X,
          RayCaster<StoreCondition,StoreAction,SearchCondition>::CTA_SIZE_Y);
        dim3 grid (divUp (rc.cols, block.x), divUp (rc.rows, block.y));

        rayCastKernel<StoreCondition,StoreAction,SearchCondition><<<grid, block>>>(rc, *buffer);
        cudaSafeCall (cudaGetLastError ());
        cudaSafeCall(cudaDeviceSynchronize());
      }

      void
      raycast (const Intr& intr, const Mat33& Rcurr, const float3& tcurr,
               float tranc_dist, float min_range, const float3& volume_size,
               const PtrStep<short2>& volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer, MapArr& vmap, MapArr& nmap)
      {
        InterpolatePointAndNormalStoreAction ipan;
        ipan.nmap = nmap;
        templatedRaycast<ZeroCrossingStoreCondition,InterpolatePointAndNormalStoreAction,TrueSearchCondition>
          (intr,Rcurr,tcurr,tranc_dist,min_range,volume_size,volume,buffer,vmap,
          ZeroCrossingStoreCondition(),ipan,TrueSearchCondition());
      }

      void
      unkRaycast (const Intr& intr, const Mat33& Rcurr, const float3& tcurr,
                  float tranc_dist, float min_range, const float3& volume_size,
                  const PtrStep<short2>& volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer,
                  const RaycastFilter & filter, const bool skip_unknown_outside_filter, MapArr& vmap, MapArr& umap)
      {
        NotEmptyStoreConditionUnkFiltered neuf;
        if (skip_unknown_outside_filter)
          neuf.filter = filter;
        SignedSensorDistanceWithPoseStoreAction nesc;
        nesc.umap = umap;
        nesc.filter = filter;
        templatedRaycast<NotEmptyStoreConditionUnkFiltered,SignedSensorDistanceWithPoseStoreAction,TrueSearchCondition>(
          intr,Rcurr,tcurr,tranc_dist,min_range,volume_size,volume,buffer,vmap,
          neuf,nesc,TrueSearchCondition());
      }

      void
      unkRaycastBBox (const Intr& intr, const Mat33& Rcurr, const float3& tcurr,
                      float tranc_dist, float min_range, const float3& volume_size,
                      const PtrStep<short2>& volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer,
                      const RaycastFilter & filter, const bool skip_unknown_outside_filter, MapArr& vmap, MapArr& umap,
                      const float3 & bbox_min,const float3 & bbox_max)
      {
        NotEmptyStoreConditionUnkFiltered neuf;
        if (skip_unknown_outside_filter)
          neuf.filter = filter;
        SignedSensorDistanceWithPoseStoreAction nesc;
        nesc.umap = umap;
        nesc.filter = filter;
        templatedRaycast<NotEmptyStoreConditionUnkFiltered,SignedSensorDistanceWithPoseStoreAction,BBoxSearchCondition>
          (intr,Rcurr,tcurr,tranc_dist,min_range,volume_size,volume,buffer,vmap,
          neuf,nesc,BBoxSearchCondition(bbox_min,bbox_max));
      }

      void
      unkRaycastInterp (const Intr& intr, const Mat33& Rcurr, const float3& tcurr,
                        float tranc_dist, float min_range, const float3& volume_size,
                        const PtrStep<short2>& volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer,
                        const RaycastFilter & filter, const bool skip_unknown_outside_filter,
                        MapArr& vmap, MapArr& umap, MapArr& nmap)
      {
        InterpolatePointAndNormalWithUnknownStoreAction ipanwu;
        ipanwu.umap = umap;
        ipanwu.nmap = nmap;
        templatedRaycast<ZeroCrossingOrUnknownStoreCondition,InterpolatePointAndNormalWithUnknownStoreAction,TrueSearchCondition>
          (intr,Rcurr,tcurr,tranc_dist,min_range,volume_size,volume,buffer,vmap,
          ZeroCrossingOrUnknownStoreCondition(),ipanwu,TrueSearchCondition());
      }

      void
      unkRaycastInterpBBox (const Intr& intr, const Mat33& Rcurr, const float3& tcurr,
                            float tranc_dist, float min_range, const float3& volume_size,
                            const PtrStep<short2>& volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer,
                            const RaycastFilter & filter, const bool skip_unknown_outside_filter,
                            MapArr& vmap, MapArr& umap, MapArr& nmap,
                            const float3 & bbox_min,const float3 & bbox_max)
      {
        InterpolatePointAndNormalWithUnknownStoreAction ipanwu;
        ipanwu.umap = umap;
        ipanwu.nmap = nmap;
        templatedRaycast<ZeroCrossingOrUnknownStoreCondition,InterpolatePointAndNormalWithUnknownStoreAction,BBoxSearchCondition>
          (intr,Rcurr,tcurr,tranc_dist,min_range,volume_size,volume,buffer,vmap,
          ZeroCrossingOrUnknownStoreCondition(),ipanwu,BBoxSearchCondition(bbox_min,bbox_max));
      }

      void
      unkRaycastVoxelIndex (const Intr& intr, const Mat33& Rcurr, const float3& tcurr,
                        float tranc_dist, float min_range, const float3& volume_size,
                        const PtrStep<short2>& volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer,
                        const RaycastFilter & filter, const bool skip_unknown_outside_filter,
                        MapArr& vmap, MapArr& umap, PtrStep<int> voxel_ids)
      {
        NotEmptyStoreConditionUnkFiltered neuf;
        if (skip_unknown_outside_filter)
          neuf.filter = filter;
        SignedSensorDistanceWithVoxelIndexStoreAction nesc;
        nesc.umap = umap;
        nesc.filter = filter;
        nesc.voxel_map = voxel_ids;
        templatedRaycast<NotEmptyStoreConditionUnkFiltered,SignedSensorDistanceWithVoxelIndexStoreAction,TrueSearchCondition>(
          intr,Rcurr,tcurr,tranc_dist,min_range,volume_size,volume,buffer,vmap,
          neuf,nesc,TrueSearchCondition());
      }

      void
      unkRaycastBBoxVoxelIndex (const Intr& intr, const Mat33& Rcurr, const float3& tcurr,
                            float tranc_dist, float min_range, const float3& volume_size,
                            const PtrStep<short2>& volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer,
                            const RaycastFilter & filter, const bool skip_unknown_outside_filter,
                            MapArr& vmap, MapArr& umap, PtrStep<int> voxel_ids,
                            const float3 & bbox_min, const float3 & bbox_max)
      {
        NotEmptyStoreConditionUnkFiltered neuf;
        if (skip_unknown_outside_filter)
          neuf.filter = filter;
        SignedSensorDistanceWithVoxelIndexStoreAction nesc;
        nesc.umap = umap;
        nesc.filter = filter;
        nesc.voxel_map = voxel_ids;
        templatedRaycast<NotEmptyStoreConditionUnkFiltered,SignedSensorDistanceWithVoxelIndexStoreAction,BBoxSearchCondition>(
          intr,Rcurr,tcurr,tranc_dist,min_range,volume_size,volume,buffer,vmap,
          neuf,nesc,BBoxSearchCondition(bbox_min,bbox_max));
      }

      void
      unkRaycastNoVertex (const Intr& intr, const Mat33& Rcurr, const float3& tcurr,
                        float tranc_dist, float min_range, const float3& volume_size,
                        const PtrStep<short2>& volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer,
                        const RaycastFilter & filter, const bool skip_unknown_outside_filter,
                        MapArr& vmap, MapArr& umap)
      {
        NotEmptyStoreConditionUnkFiltered neuf;
        if (skip_unknown_outside_filter)
          neuf.filter = filter;
        SignedSensorDistanceNoPoseStoreAction nesc;
        nesc.umap = umap;
        nesc.filter = filter;
        templatedRaycast<NotEmptyStoreConditionUnkFiltered,SignedSensorDistanceNoPoseStoreAction,TrueSearchCondition>(
          intr,Rcurr,tcurr,tranc_dist,min_range,volume_size,volume,buffer,vmap,
          neuf,nesc,TrueSearchCondition());
      }

      void
      unkRaycastBBoxNoVertex (const Intr& intr, const Mat33& Rcurr, const float3& tcurr,
                            float tranc_dist, float min_range, const float3& volume_size,
                            const PtrStep<short2>& volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer,
                            const RaycastFilter & filter, const bool skip_unknown_outside_filter,
                            MapArr& vmap, MapArr& umap,
                            const float3 & bbox_min, const float3 & bbox_max)
      {
        NotEmptyStoreConditionUnkFiltered neuf;
        if (skip_unknown_outside_filter)
          neuf.filter = filter;
        SignedSensorDistanceNoPoseStoreAction nesc;
        nesc.umap = umap;
        nesc.filter = filter;
        templatedRaycast<NotEmptyStoreConditionUnkFiltered,SignedSensorDistanceNoPoseStoreAction,BBoxSearchCondition>(
          intr,Rcurr,tcurr,tranc_dist,min_range,volume_size,volume,buffer,vmap,
          neuf,nesc,BBoxSearchCondition(bbox_min,bbox_max));
      }
    }
  }
}
