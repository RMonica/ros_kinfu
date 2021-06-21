#include "world_upload.h"

#include "parameters.h"

WorldUploadManager::WorldUploadManager(ros::NodeHandle & nh,
                                       boost::mutex & mutex,
                                       boost::condition_variable & cond):
  m_nh(nh), m_shared_mutex(mutex), m_shared_cond(cond)
{
  std::string param_string;

  m_nh.param<std::string>(PARAM_NAME_WORLD_UPLOAD_ACTION_NAME, param_string, PARAM_DEFAULT_WORLD_UPLOAD_ACTION_NAME);
  m_action_server.reset(new ActionServer(m_nh, param_string, boost::bind(&WorldUploadManager::onAction, this, _1), false));

  m_initial_transformation = Eigen::Affine3f::Identity();

  m_is_action_done = false;
  m_is_action_waiting = false;
  m_is_action_succeeded = false;

  {
    boost::mutex::scoped_lock lock(m_shared_mutex);
    m_action_server->start();
  }
}

void WorldUploadManager::onAction(const kinfu_msgs::WorldUploadGoalConstPtr & goal)
{
  {
    boost::mutex::scoped_lock lock(m_shared_mutex);
    if (m_is_action_waiting)
      return; // already running

    m_is_action_waiting = true;
    m_is_action_done = false;

    m_current_goal = goal;

    m_shared_cond.notify_all();

    while (!m_is_action_done && !ros::isShuttingDown())
    {
      m_shared_cond.wait_for(lock, boost::chrono::milliseconds(100));
    }

    if (ros::isShuttingDown())
      return;
  }

  kinfu_msgs::WorldUploadResult result;
  result.ok = m_is_action_succeeded;


  if (m_is_action_succeeded)
    m_action_server->setSucceeded(result);
  else
    m_action_server->setAborted(result);
}

void WorldUploadManager::Execute(pcl::gpu::kinfuLS::KinfuTracker * kinfu)
{
  const bool reset = m_current_goal->reset;

  kinfu->waitForShiftEnd();

  if (reset)
  {
    ROS_INFO("kinfu: WorldUploadManager: reset.");
    kinfu->reset();
  }

  const float kinfu_voxel_size = kinfu->getVoxelSize();
  Eigen::Affine3f scaling;
  scaling = Eigen::Scaling(1.0f / kinfu_voxel_size);

  TSDFCloudPtr tsdf_cloud(new TSDFCloud);
  pcl::fromROSMsg(m_current_goal->tsdf_volume, *tsdf_cloud);
  pcl::transformPointCloud(*tsdf_cloud, *tsdf_cloud, scaling * m_initial_transformation);

  const Eigen::Vector3f min(m_current_goal->bbox_min.x, m_current_goal->bbox_min.y, m_current_goal->bbox_min.z);
  const Eigen::Vector3f max(m_current_goal->bbox_max.x, m_current_goal->bbox_max.y, m_current_goal->bbox_max.z);

  const Eigen::Vector3f kinfu_min = scaling * m_initial_transformation * min;
  const Eigen::Vector3f kinfu_max = scaling * m_initial_transformation * max;
  const Eigen::Vector3i kinfu_min_i = kinfu_min.cast<int>();
  const Eigen::Vector3i kinfu_max_i = kinfu_max.cast<int>();

  ROS_INFO_STREAM("kinfu: WorldUploadManager: bounding box " << min.transpose() << " - " << max.transpose());

  // update the TSDF
  ROS_INFO("kinfu: WorldUploadManager: adding %u new points into the TSDF volume.", unsigned(tsdf_cloud->size()));
  kinfu->pushTSDFCloudToTSDF(
        tsdf_cloud, kinfu_min_i, kinfu_max_i, !reset);

  const std_msgs::UInt8MultiArray & known_voxelgrid = m_current_goal->known_voxelgrid;
  if (known_voxelgrid.data.empty())
  {
    ROS_INFO("kinfu: WorldUploadManager: provided empty known voxelgrid, weights upload skipped.");
  }
  else
  {
    Eigen::Vector3i size;
    ShortVector weights;
    if (!ParseKnownVoxelgrid(known_voxelgrid, size, weights))
    {
      ROS_ERROR("kinfu: WorldUploadManager: could not upload weights, could not parse input voxelgrid.");
    }
    else
    {
      ROS_INFO_STREAM("kinfu: WorldUploadManager: uploading weights (grid size is " << size.transpose() << ").");
      kinfu->pushWeightsToTSDF(weights, kinfu_min_i, kinfu_min_i + size);
    }
  }

  ROS_INFO("kinfu: WorldUploadManager: done.");
  {
    boost::mutex::scoped_lock lock(m_shared_mutex);
    m_is_action_waiting = false;
    m_is_action_done = true;
    m_is_action_succeeded = true;
    m_shared_cond.notify_all();
  }
}

bool WorldUploadManager::ParseKnownVoxelgrid(const std_msgs::UInt8MultiArray & voxelgrid,
                                             Eigen::Vector3i & size,
                                             ShortVector & weights)
{
  const std_msgs::MultiArrayLayout & layout = voxelgrid.layout;

  size = -Eigen::Vector3i::Ones();
  size_t strides[3];
  size_t sizes[3];

  const std::string coords[3] = {"x", "y", "z"};
  for (size_t ci = 0; ci < 3; ci++)
  {
    for (std_msgs::MultiArrayDimension dim : layout.dim)
      if (dim.label == coords[ci])
      {
        strides[ci] = dim.stride;
        size[ci] = dim.size;
        sizes[ci] = dim.size;
      }
  }

  if ((size.array() < 0).any())
  {
    ROS_ERROR_STREAM("kinfu: WorldUploadManager: incomplete multiarray layout, expected positive (x, y, z), got " <<
                     size.transpose());
    return false;
  }

  const size_t max_expected_size = strides[2] * (sizes[2] - 1) + strides[1] * (sizes[1] - 1) +
      strides[0] * (sizes[0] - 1) + 1;
  if (voxelgrid.data.size() < max_expected_size)
    ROS_ERROR("kinfu: WorldUploadManager: incomplete multiarray, expected size at least %u, got %u",
              unsigned(max_expected_size), unsigned(voxelgrid.data.size()));

  const size_t total_size = size_t(size.x()) * size.y() * size.z();

  weights.assign(total_size, 0);

  for (size_t z = 0; z < sizes[2]; z++)
    for (size_t y = 0; y < sizes[1]; y++)
      for (size_t x = 0; x < sizes[0]; x++)
      {
        const size_t i = z * strides[2] + y * strides[1] + x * strides[0];
        if (voxelgrid.data[i])
          weights[x + y * sizes[0] + z * sizes[0] * sizes[1]] = 16;
      }

  return true;
}
