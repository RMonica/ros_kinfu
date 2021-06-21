/*
 * Copyright (c) 2015, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "kinfu_voxelgrid_conversions.h"

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// STL
#include <string>
#include <vector>
#include <sstream>
#include <stdint.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifdef WITH_ARM_NAVIGATION_MSGS
  #include <arm_navigation_msgs/CollisionMap.h>
#endif

#ifdef WITH_MOVEIT_MSGS
  #include <moveit_msgs/PlanningScene.h>
#endif

struct TCube
{
  typedef std::vector<TCube, Eigen::aligned_allocator<TCube> > Vector;

  TCube(const Eigen::Vector3f & c,float s): center(c), side(s), type(0.0) {}

  Eigen::Vector3f center;
  float side;
  float type;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class CollisionMapPublisher
{
  public:
  typedef boost::shared_ptr<CollisionMapPublisher> Ptr;
  typedef boost::shared_ptr<const CollisionMapPublisher> ConstPtr;

  virtual ~CollisionMapPublisher() {}

  virtual void Publish(const TCube::Vector & vec,const std::string & frame_id) = 0;

  protected:
  ros::Publisher m_pub;
};

#ifdef WITH_ARM_NAVIGATION_MSGS
class ARM_NAVIGATION_MSGS_CollisionMapPublisher: public CollisionMapPublisher
{
  public:
  ARM_NAVIGATION_MSGS_CollisionMapPublisher(ros::NodeHandle & nh,const std::string & topic_name)
  {
    m_pub = nh.advertise<arm_navigation_msgs::CollisionMap>(topic_name,1);
  }

  void Publish(const TCube::Vector & vec,const std::string & frame_id)
  {
    boost::shared_ptr<arm_navigation_msgs::CollisionMap> collision_map(new arm_navigation_msgs::CollisionMap);
    collision_map->header.stamp = ros::Time::now();
    collision_map->header.frame_id = frame_id;

    for (uint cube_i = 0; cube_i < vec.size(); cube_i++)
    {
      arm_navigation_msgs::OrientedBoundingBox new_box;
      const TCube & cube = vec[cube_i];
      // no rotation
      new_box.axis.x = 1.0;
      new_box.axis.y = 0.0;
      new_box.axis.z = 0.0;
      new_box.angle = 0.0;

      float side = cube.side / 2.0;
      new_box.extents.x = side;
      new_box.extents.y = side;
      new_box.extents.z = side;

      new_box.center.x = cube.center.x();
      new_box.center.y = cube.center.y();
      new_box.center.z = cube.center.z();

      collision_map->boxes.push_back(new_box);
    }

    m_pub.publish(collision_map);
  }
};
#endif

#ifdef WITH_MOVEIT_MSGS
class MOVEIT_MSGS_CollisionMapPublisher: public CollisionMapPublisher
{
  public:
  typedef uint64_t uint64;
  typedef unsigned int uint;

  MOVEIT_MSGS_CollisionMapPublisher(ros::NodeHandle & nh,const std::string & topic_name)
  {
    m_pub = nh.advertise<moveit_msgs::PlanningScene>(topic_name,1);
    nh.param<std::string>(PARAM_NAME_MOVEIT_NAMESPACE,m_moveit_namespace,PARAM_DEFAULT_MOVEIT_NAMESPACE);

    {
      std::string param_string;
      nh.param<std::string>(PARAM_NAME_COLOR,param_string,PARAM_DEFAULT_COLOR);
      std::istringstream ss(param_string);
      ss >> m_color.r >> m_color.g >> m_color.b >> m_color.a;
    }
  }

  void Publish(const TCube::Vector & vec,const std::string & frame_id)
  {
    moveit_msgs::PlanningScenePtr scene(new moveit_msgs::PlanningScene);
    moveit_msgs::PlanningSceneWorld & world = scene->world;
    std::vector<moveit_msgs::ObjectColor> & colors = scene->object_colors;

    scene->is_diff = true;

    const uint64 size = vec.size();

    world.collision_objects.resize(1);
    colors.resize(1);

    moveit_msgs::ObjectColor & color = colors[0];
    color.id = m_moveit_namespace;
    color.color = m_color;

    moveit_msgs::CollisionObject & coll_object = world.collision_objects[0];
    coll_object.header.frame_id = frame_id;
    coll_object.header.stamp = ros::Time::now();

    coll_object.id = m_moveit_namespace;
    coll_object.operation = coll_object.ADD;

    coll_object.primitives.resize(size);
    coll_object.primitive_poses.resize(size);

    for (uint i = 0; i < size; i++)
    {
      const TCube & cube = vec[i];

      shape_msgs::SolidPrimitive & primitive = coll_object.primitives[i];
      geometry_msgs::Pose & pose = coll_object.primitive_poses[i];

      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = cube.side;
      primitive.dimensions[primitive.BOX_Y] = cube.side;
      primitive.dimensions[primitive.BOX_Z] = cube.side;

      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;

      pose.position.x = cube.center.x();
      pose.position.y = cube.center.y();
      pose.position.z = cube.center.z();
    }

    m_pub.publish(scene);
  }

  private:
  std::string m_moveit_namespace;
  std_msgs::ColorRGBA m_color;
};
#endif

class PointCloud2Publisher: public CollisionMapPublisher
{
  public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;

  PointCloud2Publisher(ros::NodeHandle & nh,const std::string & topic_name,const bool with_intensity)
  {
    m_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name,1);
    m_with_intensity = with_intensity;
  }

  void Publish(const TCube::Vector & vec,const std::string & frame_id)
  {
    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);

    if (m_with_intensity)
    {
      PointCloudXYZI cloud;
      cloud.reserve(vec.size());

      for (uint cube_i = 0; cube_i < vec.size(); cube_i++)
      {
        const TCube & cube = vec[cube_i];
        pcl::PointXYZI pt;
        pt.x = cube.center.x();
        pt.y = cube.center.y();
        pt.z = cube.center.z();
        pt.intensity = cube.type;
        cloud.push_back(pt);
      }

      pcl::toROSMsg(cloud, *cloud_msg);
    }
    else
    {
      PointCloudXYZ cloud;
      cloud.reserve(vec.size());

      for (uint cube_i = 0; cube_i < vec.size(); cube_i++)
      {
        const TCube & cube = vec[cube_i];
        pcl::PointXYZ pt(cube.center.x(),cube.center.y(),cube.center.z());
        cloud.push_back(pt);
      }

      pcl::toROSMsg(cloud, *cloud_msg);
    }

    cloud_msg->header.frame_id = frame_id;
    cloud_msg->header.stamp = ros::Time::now();

    m_pub.publish(cloud_msg);
  }

  private:
  bool m_with_intensity;
};

class NullCollisionMapPublisher: public CollisionMapPublisher
{
  public:
  void Publish(const TCube::Vector &,const std::string &) {}
};

class VoxelgridToCollisionMap
{
  public:
  typedef unsigned int uint;
  typedef uint64_t uint64;
  typedef std::vector<bool> BoolVector;
  typedef std::vector<float> FloatVector;

  VoxelgridToCollisionMap(ros::NodeHandle & nh): m_nh(nh)
  {
    std::string param_string;
    double param_double;

    m_nh.param<std::string>(PARAM_NAME_INPUT_TOPIC,param_string,PARAM_DEFAULT_INPUT_TOPIC);
    m_sub = m_nh.subscribe(param_string,1,&VoxelgridToCollisionMap::onVoxelGrid,this);

    m_nh.param<std::string>(PARAM_NAME_OUTPUT_TOPIC,m_output_topic,PARAM_DEFAULT_OUTPUT_TOPIC);

    m_nh.param<std::string>(PARAM_NAME_FRAME_ID,m_frame_id,PARAM_DEFAULT_FRAME_ID);

    m_nh.param<double>(PARAM_NAME_INPUT_SCALE,param_double,PARAM_DEFAULT_INPUT_SCALE);
    m_input_scale = param_double;
    m_nh.param<double>(PARAM_NAME_OUTPUT_SCALE,param_double,PARAM_DEFAULT_OUTPUT_SCALE);
    m_output_scale = param_double;

    m_nh.param<std::string>(PARAM_NAME_CENTER,param_string,PARAM_DEFAULT_CENTER);
    {
      std::istringstream ss(param_string);
      m_center = Eigen::Vector3f::Zero();
      for (uint i = 0; i < 3 && ss; i++)
        ss >> (m_center[i]);
    }

    m_collision_unknown = false;
    m_collision_occupied = false;
    m_collision_empty = false;

    m_nh.param<std::string>(PARAM_NAME_COLLISION_VALUES,param_string,PARAM_DEFAULT_COLLISION_VALUES);
    for (uint i = 0; i < param_string.size(); i++)
      switch (param_string[i])
      {
        case PARAM_VALUE_COLLISION_EMPTY:
          m_collision_empty = true; break;
        case PARAM_VALUE_COLLISION_OCCUPIED:
          m_collision_occupied = true; break;
        case PARAM_VALUE_COLLISION_UNKNOWN:
          m_collision_unknown = true; break;
        default:
          ROS_ERROR("kinfu_voxelgrid_conversions: unknown collision value: %c",char(param_string[i]));
      }

    m_nh.param<bool>(PARAM_NAME_COMPRESS,m_compress,PARAM_DEFAULT_COMPRESS);

    m_nh.param<bool>(PARAM_NAME_CARVE,m_carve,PARAM_DEFAULT_CARVE);

    m_nh.param<bool>(PARAM_NAME_KEEP_VOXEL_TYPE,m_keep_voxel_type,PARAM_DEFAULT_KEEP_VOXEL_TYPE);

    m_nh.param<std::string>(PARAM_NAME_OUTPUT_TYPE,param_string,PARAM_DEFAULT_OUTPUT_TYPE);
    if (false) {}
#ifdef WITH_ARM_NAVIGATION_MSGS
    else if (param_string == PARAM_VALUE_OUTPUT_TYPE_ARM_NAVIGATION_MSGS)
    {
      ROS_INFO("kinfu_voxelgrid_to_collision_map: type is: %s",param_string.c_str());
      m_pub = CollisionMapPublisher::Ptr(new ARM_NAVIGATION_MSGS_CollisionMapPublisher(m_nh,m_output_topic));
    }
#endif
#ifdef WITH_MOVEIT_MSGS
    else if (param_string == PARAM_VALUE_OUTPUT_TYPE_MOVEIT_MSGS)
    {
      ROS_INFO("kinfu_voxelgrid_conversions: type is: %s",param_string.c_str());
      m_pub = CollisionMapPublisher::Ptr(new MOVEIT_MSGS_CollisionMapPublisher(m_nh,m_output_topic));
    }
#endif
    else if (param_string == PARAM_VALUE_OUTPUT_TYPE_POINTCLOUD)
    {
      ROS_INFO("kinfu_voxelgrid_conversions: type is: %s",param_string.c_str());
      m_pub = CollisionMapPublisher::Ptr(new PointCloud2Publisher(m_nh,m_output_topic,m_keep_voxel_type));
    }
    else
    {
      ROS_ERROR("kinfu_voxelgrid_conversions: unknown/not installed type: %s",param_string.c_str());
      m_pub = CollisionMapPublisher::Ptr(new NullCollisionMapPublisher);
    }
  }

  #define OVG_ADDR(xyz) (uint64(xyz.x()) * output_steps.x() + \
                         uint64(xyz.y()) * output_steps.y() + \
                         uint64(xyz.z()) * output_steps.z())

  #define IVG_ADDR(xyz) (uint64(xyz.x()) * input_steps.x() + \
                         uint64(xyz.y()) * input_steps.y() + \
                         uint64(xyz.z()) * input_steps.z())

  #define OVG_CHECK(xyz) ((xyz.array() >= 0).all() && (xyz.array() < output_sizes.array()).all())

  #define IVG_CHECK(xyz) ((xyz.array() >= 0).all() && (xyz.array() < input_sizes.array()).all())

  void onVoxelGrid(const std_msgs::Float32MultiArray & msg)
  {
    ROS_INFO("kinfu_voxelgrid_to_collision_map: message received.");

    Eigen::Vector3i input_sizes;
    uint xi = uint(-1);
    uint yi = uint(-1);
    uint zi = uint(-1);
    for (uint i = 0; i < msg.layout.dim.size(); i++)
    {
      if (msg.layout.dim[i].label == "x")
        xi = i;
      if (msg.layout.dim[i].label == "y")
        yi = i;
      if (msg.layout.dim[i].label == "z")
        zi = i;
    }
    if (xi == uint(-1) || yi == uint(-1) || zi == uint(-1))
    {
      ROS_ERROR("kinfu_voxelgrid_to_collision_map: could not find x, y, z in layout.");
      return;
    }

    input_sizes[0] = msg.layout.dim[xi].size;
    input_sizes[1] = msg.layout.dim[yi].size;
    input_sizes[2] = msg.layout.dim[zi].size;
    const uint64 input_size = msg.layout.dim[xi].size * msg.layout.dim[yi].size * msg.layout.dim[zi].size;
    const Eigen::Vector3i input_steps(1,input_sizes[0],input_sizes[1] * input_sizes[0]);

    const Eigen::Vector3i output_sizes =
      ((input_sizes - Eigen::Vector3i::Ones()).cast<float>() * m_input_scale / m_output_scale +
       Eigen::Vector3f::Ones() * 0.5).cast<int>() + Eigen::Vector3i::Ones();
    const Eigen::Vector3i output_steps(1,output_sizes[0],output_sizes[1] * output_sizes[0]);
    const uint64 output_size = uint64(output_sizes[2]) * output_sizes[1] * output_sizes[0];

    const FloatVector & data = msg.data;
    if (data.size() < input_size)
    {
      ROS_ERROR("kinfu_voxelgrid_to_collision_map: data size is %u, expected %u.", unsigned(data.size()), unsigned(input_size));
      return;
    }

    BoolVector dangerous(output_size,false);

    // scope only
    {
      ROS_INFO("kinfu_voxelgrid_to_collision_map: finding dangerous...");
      const uint64 input_steps_x = input_steps.x();
      const uint64 input_steps_y = input_steps.y();
      const uint64 input_steps_z = input_steps.z();
      const uint64 output_steps_x = output_steps.x();
      const uint64 output_steps_y = output_steps.y();
      const uint64 output_steps_z = output_steps.z();

      ROS_INFO("kinfu_voxelgrid_to_collision_map: resampling...");
      for (uint64 z = 0; z < input_sizes.z(); z++)
        for (uint64 y = 0; y < input_sizes.y(); y++)
          for (uint64 x = 0; x < input_sizes.x(); x++)
          {
            const uint64 addr = x * input_steps_x + y * input_steps_y + z * input_steps_z;
            const Eigen::Vector3f ifloat(x, y, z);
            bool input_dangerous = false;
            if ( (m_collision_empty && data[addr] < -0.5) ||
              (m_collision_unknown && data[addr] >= -0.5 && data[addr] < 0.5) ||
              (m_collision_occupied && data[addr] >= 0.5) )
              input_dangerous = true;

            if (input_dangerous)
            {
              Eigen::Vector3f out_ifloat = (ifloat * m_input_scale / m_output_scale + (Eigen::Vector3f::Ones() * 0.5));
              const uint64 ox = out_ifloat.x();
              const uint64 oy = out_ifloat.y();
              const uint64 oz = out_ifloat.z();
              const uint64 oaddr = ox * output_steps_x + oy * output_steps_y + oz * output_steps_z;
              dangerous[oaddr] = true;
            }
          }
      ROS_INFO("kinfu_voxelgrid_to_collision_map: resampled.");
    }

    if (m_carve)
      dangerous = Carve(dangerous,output_sizes,output_steps);

    TCube::Vector cubes;

    if (m_compress)
      dangerous = Compress8(dangerous,output_sizes,output_steps,cubes);

    // scope only
    {
      Eigen::Vector3i i;
      for (i.z() = 0; i.z() < output_sizes.z(); i.z()++)
        for (i.y() = 0; i.y() < output_sizes.y(); i.y()++)
          for (i.x() = 0; i.x() < output_sizes.x(); i.x()++)
          {
            const uint64 addr = OVG_ADDR(i);
            if (dangerous[addr])
            {
              Eigen::Vector3f c = (i.cast<float>() + Eigen::Vector3f::Ones() * 0.5) * m_output_scale + m_center;
              cubes.push_back(TCube(c,m_output_scale));
            }
          }
    }

    if (m_keep_voxel_type)
      cubes = AddVoxelTypeToCubes(data,m_center,m_input_scale,input_sizes,input_steps,cubes);

    ROS_INFO("kinfu_voxelgrid_to_collision_map: publishing %u cubes.",uint(cubes.size()));

    m_pub->Publish(cubes,m_frame_id);
  }

  TCube::Vector AddVoxelTypeToCubes(const FloatVector & data,const Eigen::Vector3f & center,
    const float input_scale,const Eigen::Vector3i input_sizes,const Eigen::Vector3i & input_steps,const TCube::Vector & cubes)
  {
    TCube::Vector result = cubes;
    const uint size = result.size();

    for (uint i = 0; i < size; i++)
    {
      TCube & cube = result[i];
      const Eigen::Vector3i input_center = ((cube.center - center) / input_scale).cast<int>();
      if (IVG_CHECK(input_center))
        cube.type = data[IVG_ADDR(input_center)];
    }

    return result;
  }

  BoolVector Compress8(const std::vector<bool> & odangerous,
    const Eigen::Vector3i & output_sizes,const Eigen::Vector3i & output_steps,
    TCube::Vector & cubes)
  {
    ROS_INFO("kinfu_voxelgrid_to_collision_map: compression...");
    BoolVector dangerous = odangerous;

    const uint MAX_MULTIPLIER = 5;
    for (int exponent = MAX_MULTIPLIER; exponent >= 0; exponent--)
    {
      const int multiplier = 2 << exponent;

      Eigen::Vector3i i;
      for (i.z() = 0; i.z() < output_sizes.z(); i.z() += multiplier)
        for (i.y() = 0; i.y() < output_sizes.y(); i.y() += multiplier)
          for (i.x() = 0; i.x() < output_sizes.x(); i.x() += multiplier)
          {
            bool any_nd = false;
            bool any_d = false;

            Eigen::Vector3i mi = i + ((multiplier - 1) * Eigen::Vector3i::Ones());
            if (!(mi.array() >= 0).all() || !(mi.array() < output_sizes.array()).all())
              continue;

            Eigen::Vector3i di;
            for (di.z() = 0; di.z() < multiplier; di.z()++)
              for (di.y() = 0; di.y() < multiplier; di.y()++)
                for (di.x() = 0; di.x() < multiplier; di.x()++)
                {
                  Eigen::Vector3i ci = di + i;
                  if (dangerous[OVG_ADDR(ci)])
                    any_d = true;
                  else
                    any_nd = true;
                }

            if (any_d && !any_nd)
            {
              for (di.z() = 0; di.z() < multiplier; di.z()++)
                for (di.y() = 0; di.y() < multiplier; di.y()++)
                  for (di.x() = 0; di.x() < multiplier; di.x()++)
                  {
                    Eigen::Vector3i ci = di + i;
                    if ((ci.array() >= 0).all() && (ci.array() < output_sizes.array()).all())
                      dangerous[OVG_ADDR(ci)] = false;
                  }
              Eigen::Vector3f c = (i.cast<float>() + Eigen::Vector3f::Ones() * 0.5 * float(multiplier)) * m_output_scale + m_center;
              cubes.push_back(TCube(c,m_output_scale * multiplier));
            }
          }
    }

    ROS_INFO("kinfu_voxelgrid_to_collision_map: compressed %u cubes.",uint(cubes.size()));
    return dangerous;
  }

  std::vector<bool> Carve(const std::vector<bool> & odangerous,const Eigen::Vector3i & output_sizes,const Eigen::Vector3i & output_steps)
  {
    ROS_INFO("kinfu_voxelgrid_to_collision_map: carving...");
    Eigen::Vector3i i;
    Eigen::Vector3i di;
    uint64 carved_count = 0;
    std::vector<bool> dangerous = odangerous;
    for (i.z() = 1; i.z() < output_sizes.z() - 1; i.z()++)
      for (i.y() = 1; i.y() < output_sizes.y() - 1; i.y()++)
        for (i.x() = 1; i.x() < output_sizes.x() - 1; i.x()++)
        {
          const uint64 addr = OVG_ADDR(i);
          const bool d = odangerous[addr];
          if (!d)
            continue;

          bool can_be_carved = true;
          for (di.z() = -1; di.z() <= 1 && can_be_carved; di.z()++)
            for (di.y() = -1; di.y() <= 1 && can_be_carved; di.y()++)
              for (di.x() = -1; di.x() <= 1 && can_be_carved; di.x()++)
              {
                if ((di.array() != 0).count() != 1)
                  continue; // skip 0,0,0 and all values with more than one != 0
                Eigen::Vector3i oi = i + di;
                const uint64 oaddr = OVG_ADDR(oi);
                if (!odangerous[oaddr])
                  can_be_carved = false;
              }
          if (can_be_carved)
          {
            dangerous[addr] = false;
            carved_count++;
          }
        }
    ROS_INFO("kinfu_voxelgrid_to_collision_map: carved %u cells.",uint(carved_count));
    return dangerous;
  }
  #undef OVG_ADDR
  #undef IVG_ADDR

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
  ros::NodeHandle & m_nh;

  CollisionMapPublisher::Ptr m_pub;
  ros::Subscriber m_sub;

  std::string m_output_topic;

  Eigen::Vector3f m_center;
  float m_input_scale;
  float m_output_scale;
  std::string m_frame_id;

  bool m_compress;
  bool m_carve;
  bool m_keep_voxel_type;

  bool m_collision_unknown;
  bool m_collision_occupied;
  bool m_collision_empty;
};

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"kinfu_voxelgrid_to_collision_map");

  ros::NodeHandle nh("~");

  VoxelgridToCollisionMap vtcm(nh);

  ros::spin();

  return 0;
}
