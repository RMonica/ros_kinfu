#include "bitmaskoctree.h"

#define pcl_BitmaskOctree_COUNT_FULL_LEAVES 0

template <uint32_t LOWER_BITS>
  void pcl::BitmaskOctree<LOWER_BITS>::SetLeaf(const Key & k,bool v)
    {
    pcl::octree::OctreeKey okey = BuildOctreeKey(k);
    OccupancyBitset * container = this->createLeaf(okey);
    uint32 bkey = BuildBitsetIndex(k);

    // if not already set, update it and update point counter
    if ((*container)[bkey] != v)
      {
#if pcl_BitmaskOctree_COUNT_FULL_LEAVES
      if (!v && (container->count() == container->size()))
        m_full_leaves--;
#endif

      (*container)[bkey] = v;
      v ? m_point_count++ : m_point_count--;

      if (!v && (container->count() == 0))
        this->removeLeaf(okey); // leaf is now empty -> remove it
#if pcl_BitmaskOctree_COUNT_FULL_LEAVES
      if (v && (container->count() == container->size()))
        m_full_leaves++;
#endif
      }
    }

template <uint32_t LOWER_BITS>
  void pcl::BitmaskOctree<LOWER_BITS>::SetLeafCached(const Key & k,bool v,Cache & cache)
    {
    pcl::octree::OctreeKey okey = BuildOctreeKey(k);

    if (cache.okey.x != okey.x || cache.okey.y != okey.y || cache.okey.z != okey.z)
    {
      cache.bitset = this->createLeaf(okey);
      cache.okey = okey;
    }
    OccupancyBitset * &container = cache.bitset;

    uint32 bkey = BuildBitsetIndex(k);

    // if not already set, update it and update point counter
    if ((*container)[bkey] != v)
      {
#if pcl_BitmaskOctree_COUNT_FULL_LEAVES
      if (!v && (container->count() == container->size()))
        m_full_leaves--;
#endif

      (*container)[bkey] = v;
      v ? m_point_count++ : m_point_count--;

      if (!v && (container->count() == 0))
        this->removeLeaf(okey); // leaf is now empty -> remove it
#if pcl_BitmaskOctree_COUNT_FULL_LEAVES
      if (v && (container->count() == container->size()))
        m_full_leaves++;
#endif
      }
    }

template <uint32_t LOWER_BITS>
  void pcl::BitmaskOctree<LOWER_BITS>::SetIntWithBitmask(int32 x,int32 y,int32 z,uint8 bitmask[BITMASK_BYTE_SIZE])
    {
    ExpandUntilPoint(x,y,z);

    uint32 kx = (x >> LOWER_BITS) + int32(m_cx);
    uint32 ky = (y >> LOWER_BITS) + int32(m_cy);
    uint32 kz = (z >> LOWER_BITS) + int32(m_cz);
    OccupancyBitset * container = this->createLeaf(pcl::octree::OctreeKey(kx,ky,kz));
    _ToBitset(bitmask,*container);
    m_point_count += container->count();
    }

template <uint32_t LOWER_BITS>
  bool pcl::BitmaskOctree<LOWER_BITS>::GetLeaf(const Key & k) const
    {
    pcl::octree::OctreeKey okey = BuildOctreeKey(k);
    if (okey.x >= m_octree_side || okey.y >= m_octree_side || okey.z >= m_octree_side)
      return false; // out of boundary -> all empty

    const OccupancyBitset * container = this->findLeaf(okey);
    if (!container)
      return false; // not existing -> all empty

    return (*container)[BuildBitsetIndex(k)];
    }

template <uint32_t LOWER_BITS>
  bool pcl::BitmaskOctree<LOWER_BITS>::GetLeafCached(const Key & k,Cache & cache) const
    {
    pcl::octree::OctreeKey okey = BuildOctreeKey(k);
    if (okey.x >= m_octree_side || okey.y >= m_octree_side || okey.z >= m_octree_side)
      return false; // out of boundary -> all empty

    if (cache.okey.x != okey.x || cache.okey.y != okey.y || cache.okey.z != okey.z)
    {
      cache.bitset = this->findLeaf(okey);
      cache.okey = okey;
    }

    if (!cache.bitset)
      return false; // not existing -> all empty

    return (*cache.bitset)[BuildBitsetIndex(k)];
    }

template <uint32_t LOWER_BITS>
  void pcl::BitmaskOctree<LOWER_BITS>::Serialize(std::ostream & out)
    {
    out.write(reinterpret_cast<const char*>(&m_octree_side),sizeof(m_octree_side));
    out.write(reinterpret_cast<const char*>(&m_cx),sizeof(m_cx));
    out.write(reinterpret_cast<const char*>(&m_cy),sizeof(m_cy));
    out.write(reinterpret_cast<const char*>(&m_cz),sizeof(m_cz));

    uint32 depth = this->getTreeDepth();
    out.write(reinterpret_cast<const char*>(&depth),sizeof(depth));

    uint32 leaf_count = this->getLeafCount();
    out.write(reinterpret_cast<const char*>(&leaf_count),sizeof(leaf_count));

    LeafNodeIterator iter(this);
    for (/*iter*/; *iter; ++iter)
      {
      // copy the key
      pcl::octree::OctreeKey key = iter.getCurrentOctreeKey();
      uint32 kx = key.x;
      uint32 ky = key.y;
      uint32 kz = key.z;
      out.write(reinterpret_cast<const char*>(&kx),sizeof(kx));
      out.write(reinterpret_cast<const char*>(&ky),sizeof(ky));
      out.write(reinterpret_cast<const char*>(&kz),sizeof(kz));

      // copy the leaf
      OccupancyBitset & container = iter.getLeafContainer();

      uint8 bits[BITMASK_SIZE];
      _FromBitset(bits,container);
      out.write(reinterpret_cast<const char*>(bits),sizeof(bits));
      }
    }

template <uint32_t LOWER_BITS>
  void pcl::BitmaskOctree<LOWER_BITS>::Deserialize(std::istream & in)
    {
    Clear();

    in.read(reinterpret_cast<char*>(&m_octree_side),sizeof(m_octree_side));
    in.read(reinterpret_cast<char*>(&m_cx),sizeof(m_cx));
    in.read(reinterpret_cast<char*>(&m_cy),sizeof(m_cy));
    in.read(reinterpret_cast<char*>(&m_cz),sizeof(m_cz));

    uint32 depth;
    in.read(reinterpret_cast<char*>(&depth),sizeof(depth));
    this->setTreeDepth(depth);

    uint32 leaf_count;
    in.read(reinterpret_cast<char*>(&leaf_count),sizeof(leaf_count));

    for (uint32 iter = 0; iter < leaf_count; iter++)
      {
      // read the key
      uint32 kx;
      uint32 ky;
      uint32 kz;
      in.read(reinterpret_cast<char*>(&kx),sizeof(kx));
      in.read(reinterpret_cast<char*>(&ky),sizeof(ky));
      in.read(reinterpret_cast<char*>(&kz),sizeof(kz));
      pcl::octree::OctreeKey key(kx,ky,kz);

      // read the leaf
      OccupancyBitset * container = this->createLeaf(key);

      uint8 bits[BITMASK_SIZE];
      in.read(reinterpret_cast<char*>(bits),sizeof(bits));
      _ToBitset(bits,*container);

      m_point_count += container->count();
      }
    }

template <uint32_t LOWER_BITS>
  bool pcl::BitmaskOctree<LOWER_BITS>::ExpandUntilPoint(int32 x,int32 y,int32 z)
    {
    bool changed = false;
    do
      {
      int32 kx = (x >> LOWER_BITS) + int32(m_cx);
      int32 ky = (y >> LOWER_BITS) + int32(m_cy);
      int32 kz = (z >> LOWER_BITS) + int32(m_cz);

      bool underflow_x = kx < 0;
      bool underflow_y = ky < 0;
      bool underflow_z = kz < 0;

      bool overflow_x = kx >= int32(m_octree_side);
      bool overflow_y = ky >= int32(m_octree_side);
      bool overflow_z = kz >= int32(m_octree_side);

      if (!overflow_x && !overflow_y && !overflow_z &&
        !underflow_x && !underflow_y && !underflow_z)
        break; // everything ok
      changed = true;

      // add another tree level and thus increase its size by a factor of 2*2*2
      uint8 child_idx = static_cast<unsigned char> (((!overflow_x) << 2) | ((!overflow_y) << 1)
        | ((!overflow_z)));

      BranchNode * newRootBranch = new BranchNode;

      this->branch_count_++;
      this->setBranchChildPtr (*newRootBranch, child_idx, this->getRootNode());
      this->root_node_ = newRootBranch;

      m_octree_side = 1 << this->getTreeDepth();

      if (!overflow_x)
        m_cx += m_octree_side;

      if (!overflow_y)
        m_cy += m_octree_side;

      if (!overflow_z)
        m_cz += m_octree_side;

      this->setTreeDepth(this->getTreeDepth() + 1);
      m_octree_side = 1 << (this->getTreeDepth());
      }
    while (true);

    return changed;
    }

template <uint32_t LOWER_BITS>
class pcl::BitmaskOctree<LOWER_BITS>::TrueIterator
  {
  public:
  TrueIterator(BitmaskOctree<LOWER_BITS> & octree): m_octree(octree),m_iter(&octree)
    {
    m_x = 0;
    m_y = 0;
    m_z = 0;

    _NextTrue();
    }

  TrueIterator & operator++()
    {
    _IncCoords(); // skip current one
    _NextTrue(); // find next true
    return *this;
    }

  const TrueIterator & operator++(int)
    {
    const TrueIterator copy = *this;
    ++(*this);
    return copy;
    }

  // returns false if ended
  bool GetLeafCoords(uint32 & x,uint32 & y,uint32 & z) const
    {
    if (!(*m_iter))
      return false;

    pcl::octree::OctreeKey okey = m_iter.getCurrentOctreeKey();
    x = (uint32(okey.x) << LOWER_BITS) + m_x;
    y = (uint32(okey.y) << LOWER_BITS) + m_y;
    z = (uint32(okey.z) << LOWER_BITS) + m_z;

    return true;
    }

  bool GetIntCoords(int32 & x,int32 & y,int32 & z) const
    {
    uint32 ux,uy,uz;
    if (!GetLeafCoords(ux,uy,uz))
      return false;

    x = int32(ux) - int32(m_octree.m_cx << LOWER_BITS);
    y = int32(uy) - int32(m_octree.m_cy << LOWER_BITS);
    z = int32(uz) - int32(m_octree.m_cz << LOWER_BITS);
    return true;
    }

  bool GetFloatCoords(float & x,float & y,float & z,float scale) const
    {
    int32 ix,iy,iz;
    if (!GetIntCoords(ix,iy,iz))
      return false;

    x = float(ix + 0.5) * scale;
    y = float(iy + 0.5) * scale;
    z = float(iz + 0.5) * scale;
    return true;
    }

  // this returns false when ended
  operator bool() const
    {
    return bool(*m_iter);
    }

  private:
  void _NextTrue()
    {
    while ((*m_iter) && !(m_iter.getLeafContainer()[BuildBitsetIndex(Key(m_x,m_y,m_z))]))
      _IncCoords();
    }

  void _IncCoords()
    {
    if (!(*m_iter))
      return;

    m_x++;
    if (m_x >= (0x1 << LOWER_BITS))
      {
      m_x = 0;
      m_y++;
      }
    if (m_y >= (0x1 << LOWER_BITS))
      {
      m_y = 0;
      m_z++;
      }

    if (m_z >= (0x1 << LOWER_BITS))
      {
      m_z = 0;
      ++m_iter;
      }
    }

  BitmaskOctree<LOWER_BITS> & m_octree;
  LeafNodeIterator m_iter;
  uint32 m_x;
  uint32 m_y;
  uint32 m_z;
  };

template <uint32_t LOWER_BITS>
  void pcl::BitmaskOctree<LOWER_BITS>::_ToBitset(const uint8 arr[BITMASK_BYTE_SIZE],OccupancyBitset & bitset)
  {
  for (uint32 biti = 0; biti < BITMASK_SIZE; biti++)
    {
    const uint8 current_byte = arr[biti / 8];
    bitset[biti] = bool((current_byte >> (biti % 8)) & 0x1);
    }
  }

template <uint32_t LOWER_BITS>
void pcl::BitmaskOctree<LOWER_BITS>::_FromBitset(uint8 arr[BITMASK_BYTE_SIZE],const OccupancyBitset & bitset)
  {
  for (uint32 biti = 0; biti < BITMASK_SIZE; biti++)
    {
    uint8 & current_byte = arr[biti / 8];
    if (bitset[biti])
      current_byte |= (0x1 << (biti % 8));
      else
        current_byte &= ~(0x1 << (biti % 8));
    }
  }

