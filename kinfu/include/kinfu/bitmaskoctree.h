#ifndef BITMASKOCTREE_H
#define BITMASKOCTREE_H

// STL
#include <bitset>
#include <stdint.h>
#include <cstring>
#include <ostream>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_base.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_key.h>

namespace pcl
{
// the lower LOWER_BITS of the coordinates will be
// stored into a bitmask inside the leaf node
template <uint32_t LOWER_BITS>
class BitmaskOctree: private pcl::octree::OctreeBase<std::bitset<(0x1 << (LOWER_BITS * 3))> >
  {
  public:
  typedef uint32_t uint32;
  typedef int32_t int32;
  typedef uint8_t uint8;

  enum
    {
    BITMASK_SIZE = (0x1 << (LOWER_BITS * 3)),
    BITMASK_BYTE_SIZE = ((BITMASK_SIZE >> 3) + !!(BITMASK_SIZE % 3)), // bitmask size in bytes, rounded up
    LOWER_FULL_BITMASK = ((0x1 << LOWER_BITS) - 1)
    };

  typedef std::bitset<BITMASK_SIZE> OccupancyBitset;
  typedef pcl::octree::OctreeBase<OccupancyBitset> OccupancyOctree;
  typedef typename OccupancyOctree::BranchNode BranchNode;
  typedef typename OccupancyOctree::LeafNodeIterator LeafNodeIterator;

  typedef boost::shared_ptr<BitmaskOctree> Ptr;
  typedef boost::shared_ptr<const BitmaskOctree> ConstPtr;

  struct Key
    {
    uint32 x;
    uint32 y;
    uint32 z;

    Key(uint32 cx,uint32 cy,uint32 cz): x(cx),y(cy),z(cz) {}
    };

  BitmaskOctree()
    {
    m_cx = 0;
    m_cy = 0;
    m_cz = 0;

    m_point_count = 0;

    this->setTreeDepth(1);
    m_octree_side = (1 << this->getTreeDepth());
    }

  BitmaskOctree(const BitmaskOctree & other): OccupancyOctree(other)
    {
    m_cx = other.m_cx;
    m_cy = other.m_cy;
    m_cz = other.m_cz;
    m_octree_side = other.m_octree_side;
    m_point_count = other.m_point_count;
    }

  virtual ~BitmaskOctree()
    {
    }

  const BitmaskOctree & operator=(const BitmaskOctree & other)
    {
    this->OccupancyOctree::operator=(other);
    m_cx = other.m_cx;
    m_cy = other.m_cy;
    m_cz = other.m_cz;
    m_octree_side = other.m_octree_side;
    m_point_count = other.m_point_count;
    return *this;
    }

  static pcl::octree::OctreeKey BuildOctreeKey(const Key & k)
    {
    return pcl::octree::OctreeKey(k.x >> LOWER_BITS,k.y >> LOWER_BITS,k.z >> LOWER_BITS);
    }

  static uint32 BuildBitsetIndex(const Key & k)
    {
    return (k.x & LOWER_FULL_BITMASK) |
      ((k.y & LOWER_FULL_BITMASK) << LOWER_BITS) |
      ((k.z & LOWER_FULL_BITMASK) << (LOWER_BITS * 2));
    }

  void SetLeaf(const Key & k,bool v);

  void SetLeaf(uint32 x,uint32 y,uint32 z,bool v) {SetLeaf(Key(x,y,z),v); }

  bool GetLeaf(const Key & k) const;

  bool GetLeaf(uint32 x,uint32 y,uint32 z) const {return GetLeaf(Key(x,y,z)); }

  // after this, the tree will be empty
  void Clear()
    {
    this->deleteTree();
    m_cx = 0;
    m_cy = 0;
    m_cz = 0;
    m_octree_side = (1 << this->getTreeDepth());
    m_point_count = 0; // tree is now empty
    }

  // get value at x,y,z
  bool GetInt(int32 x,int32 y,int32 z) const
    {
    return GetLeaf(x + int32(m_cx << LOWER_BITS),y + int32(m_cy << LOWER_BITS),z + int32(m_cz << LOWER_BITS));
    }

  // adapts the octree to the new point
  void ExpandUntilPoint(int32 x,int32 y,int32 z);

  // set value at x,y,z
  void SetInt(int32 x,int32 y,int32 z,bool v)
    {
    ExpandUntilPoint(x,y,z);
    SetLeaf(x + int32(m_cx << LOWER_BITS),y + int32(m_cy << LOWER_BITS),z + int32(m_cz << LOWER_BITS),v);
    }

  // helper function for float values
  void SetFloat(float x,float y,float z,float scale,bool v)
    {
    int ax = x / scale;
    int ay = y / scale;
    int az = z / scale;
    SetInt(ax,ay,az,v);
    }

  // helper function for float values
  bool GetFloat(float x,float y,float z,float scale) const
    {
    int ax = x / scale;
    int ay = y / scale;
    int az = z / scale;
    return GetInt(ax,ay,az);
    }

  // this iterates over the TRUE voxels of the octree
  // the current center is accessed with GetFloatCoords(float &x,&y,&z,scale)
  class TrueIterator;

  uint32 GetPointCount() {return m_point_count; }
  uint32 GetLeafCount() {return this->getLeafCount(); }

  void Serialize(std::ostream & out);
  void Deserialize(std::istream & in);

  template <class PointT>
    void GetOccupiedVoxelsCenters(pcl::PointCloud<PointT> &cloud,float scale)
    {
    cloud.points.resize(m_point_count);
    cloud.height = 1;
    cloud.width = m_point_count;

    TrueIterator iter(*this);
    uint32 point_i = 0;
    for (/*iter*/; bool(iter); ++iter)
      {
      float x,y,z;
      if (!iter.GetFloatCoords(x,y,z,scale))
        continue;
      PointT point;
      point.x = x;
      point.y = y;
      point.z = z;
      cloud.points[point_i++] = point;
      }

    assert(point_i == m_point_count);
    }

  template <class PointT>
    void SetOccupiedVoxelsCenters(const pcl::PointCloud<PointT> &cloud,float scale)
    {
    this->Clear();

    for (uint32_t i = 0; i < cloud.size(); i++)
      {
      const PointT &pt = cloud[i];
      SetFloat(pt.x,pt.y,pt.z,scale,true);
      }
    }

  // returns the point cloud with octree voxel resolution,
  // but with the bitmask attached to every point
  template <class PointTWithBitmask>
    void GetOccupiedVoxelsCentersWithBitmask(pcl::PointCloud<PointTWithBitmask> & cloud,float scale)
    {
    cloud.points.resize(this->getLeafCount());
    cloud.height = 1;
    cloud.width = this->getLeafCount();

    LeafNodeIterator iter(this);
    uint32 point_i = 0;
    for (/*iter*/; *iter; ++iter)
      {
      pcl::octree::OctreeKey okey = iter.getCurrentOctreeKey();
      PointTWithBitmask &point = cloud[point_i++];
      point.x = (((int32(okey.x) - int32(m_cx)) << LOWER_BITS) + 0.5) * scale;
      point.y = (((int32(okey.y) - int32(m_cy)) << LOWER_BITS) + 0.5) * scale;
      point.z = (((int32(okey.z) - int32(m_cz)) << LOWER_BITS) + 0.5) * scale;
      _FromBitset(point.bitmask,iter.getLeafContainer());
      }

    assert(point_i == this->getLeafCount());
    }

  void SetIntWithBitmask(int32 x,int32 y,int32 z,uint8 bitmask[BITMASK_BYTE_SIZE]);

  void SetFloatWithBitmask(float x,float y,float z,float scale,uint8 bitmask[BITMASK_BYTE_SIZE])
    {
    int32 kx = x / scale;
    int32 ky = y / scale;
    int32 kz = z / scale;
    SetIntWithBitmask(kx,ky,kz,bitmask);
    }

  template <class PointTWithBitmask>
    void SetOccupiedVoxelsCentersWithBitmask(pcl::PointCloud<PointTWithBitmask> & cloud,float scale)
    {
    this->Clear();

    for (uint32_t i = 0; i < cloud.size(); i++)
      {
      PointTWithBitmask &pt = cloud[i];
      SetFloatWithBitmask(pt.x,pt.y,pt.z,scale,pt.bitmask);
      }
    }

  private:
  void _ToBitset(const uint8 arr[BITMASK_BYTE_SIZE],OccupancyBitset & bitset);
  void _FromBitset(uint8 arr[BITMASK_BYTE_SIZE],const OccupancyBitset & bitset);

  uint32 m_octree_side;

  uint32 m_cx;
  uint32 m_cy;
  uint32 m_cz;

  uint32 m_point_count;
  };

struct PointWithBitmask64
  {
  PCL_ADD_POINT4D;

  uint8_t bitmask[64];

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  } EIGEN_ALIGN16;

}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointWithBitmask64,
  (float,x,x)
  (float,y,y)
  (float,z,z)
  (uint8_t[64],bitmask,bitmask)
  );

#include "bitmaskoctree.hpp"

#endif // BITMASKOCTREE_H
