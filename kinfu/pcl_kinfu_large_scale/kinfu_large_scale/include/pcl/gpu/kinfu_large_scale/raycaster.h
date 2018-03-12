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


#ifndef PCL_KINFU_TSDF_RAYCASTERLS_H_
#define PCL_KINFU_TSDF_RAYCASTERLS_H_


#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/kinfu_large_scale/pixel_rgb.h>
#include <boost/shared_ptr.hpp>
//#include <boost/graph/buffer_concepts.hpp>
#include <Eigen/Geometry>

#include <pcl/gpu/kinfu_large_scale/tsdf_buffer.h>

namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {
      class TsdfVolume;

      /** \brief Class that performs raycasting for TSDF volume
        * \author Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
        */
      struct PCL_EXPORTS RayCaster
      {
      public:
        typedef boost::shared_ptr<RayCaster> Ptr;
        typedef pcl::gpu::DeviceArray2D<float> MapArr;
        typedef pcl::gpu::DeviceArray2D<PixelRGB> View;
        typedef pcl::gpu::DeviceArray2D<unsigned short> Depth;
        typedef pcl::gpu::DeviceArray2D<int> Indices;

        /** \brief Image with height */ 
        const int cols, rows;      
        
        /** \brief Constructor 
          * \param[in] rows image rows
          * \param[in] cols image cols
          * \param[in] fx focal x
          * \param[in] fy focal y
          * \param[in] cx principal point x
          * \param[in] cy principal point y
          */
        RayCaster(int rows = 480, int cols = 640, float fx = 525.f, float fy = 525.f, float cx = -1, float cy = -1);
        
        ~RayCaster();

        /** \brief Sets camera intrinsics */ 
        void
        setIntrinsics(float fx = 525.f, float fy = 525.f, float cx = -1, float cy = -1);

        /** \brief Sets bounding box */
        void
        setBoundingBox(const Eigen::Vector3f & bbox_min,const Eigen::Vector3f & bbox_max)
          {bbox_min_ = bbox_min; bbox_max_= bbox_max; has_bbox_ = true; }

        /** \brief clear bounding box flag */
        void
        clearBoundingBox() {has_bbox_ = false; }

        void
        setBoundingBoxFilter(const Eigen::Vector3f & bbox_min,const Eigen::Vector3f & bbox_max)
          {filter_data_.bbox_min = bbox_min; filter_data_.bbox_max = bbox_max; filter_data_.has_bbox = true; }

        void
        setSphereFilter(const Eigen::Vector3f & center,const float radius)
          {filter_data_.sphere_center = center; filter_data_.sphere_radius = radius; filter_data_.has_sphere = true; }

        void clearFilter()
          {filter_data_ = FilterData(); }

        /** \brief Extracts signed distance from the sensor (positive if occupied, negative if unknown). */
        void setWithKnown(const bool wk)
          {with_known_ = wk; }

        /** \brief If false, no vertex map will be extracted. */
        void setWithVertexMap(const bool wp)
          {with_vertex_ = wp; }

        void setWithVoxelIds(const bool wv)
          {with_voxel_ids_ = wv; }

        /** \brief set raycast step (-1 for default) */
        void
        setRaycastStep(float step) {raycast_step_ = step; }
        
        /** \brief Runs raycasting algorithm from given camera pose. It writes results to internal fiels.
          * \param[in] volume tsdf volume container
          * \param[in] camera_pose camera pose
          * \param buffer
          */
        void 
        run(const TsdfVolume& volume, const Eigen::Affine3f& camera_pose, tsdf_buffer* buffer);

        /** \brief Generates scene view using data raycasted by run method. So call it before.
          * \param[out] view output array for RGB image        
          */
        void
        generateSceneView(View& view) const;

        /** \brief Generates scene view using data raycasted by run method. So call it before.
          * \param[out] view output array for RGB image
          * \param[in] light_source_pose pose of light source
          */
        void
        generateSceneView(View& view, const Eigen::Vector3f& light_source_pose) const;

        /** \brief Generates depth image using data raycasted by run method. So call it before.
          * \param[out] depth output array for depth image        
          */
        void
        generateDepthImage(Depth& depth) const;
        
        /** \brief Returns raycasterd vertex map. */ 
        MapArr
        getVertexMap() const;

        /** \brief Returns raycasterd normal map. */ 
        MapArr
        getNormalMap() const;

        /** \brief Returns raycasterd known map. */
        MapArr
        getKnownMap() const {return known_map_; }

        Indices
        getVoxelIdsMap() const {return voxel_ids_map_; }

        void
        setMinRange(float range) {min_range_ = range; }

      private:
        /** \brief Camera intrinsics. */ 
        float fx_, fy_, cx_, cy_;
              
        /* Vertext/normal map internal representation example for rows=2 and cols=4
        *  X X X X
        *  X X X X
        *  Y Y Y Y
        *  Y Y Y Y
        *  Z Z Z Z
        *  Z Z Z Z     
        */

        /** \brief vertex map of 3D points*/
        MapArr vertex_map_;
        
        /** \brief normal map of 3D points*/
        MapArr normal_map_;

        /** \brief known status of 3D points*/
        MapArr known_map_;

        /** \brief voxel indices of 3D points*/
        Indices voxel_ids_map_;

        /** \brief camera pose from which raycasting was done */
        Eigen::Affine3f camera_pose_;

        /** \brief Last passed volume size */
        Eigen::Vector3f volume_size_;

        /** \brief true if voxels outside a bounding box must be ignored (i. e. set to empty) */
        bool has_bbox_;
        Eigen::Vector3f bbox_min_;
        Eigen::Vector3f bbox_max_;

        struct FilterData
        {
          bool has_bbox;
          Eigen::Vector3f bbox_min;
          Eigen::Vector3f bbox_max;

          bool has_sphere;
          Eigen::Vector3f sphere_center;
          float sphere_radius;

          FilterData(): has_bbox(false), has_sphere(false) {}
        };

        /** \brief voxels outside the filter are out of range */
        FilterData filter_data_;

        bool with_known_;
        bool with_vertex_;
        bool with_voxel_ids_;

        float raycast_step_;

        float min_range_;

public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      };
      
      /** \brief Converts from map representation to organized not-dence point cloud. */
      template<typename PointType>
      void convertMapToOranizedCloud(const RayCaster::MapArr& map, pcl::gpu::DeviceArray2D<PointType>& cloud);
    }
  }
}

#endif /* PCL_KINFU_TSDF_RAYCASTER_H_ */
