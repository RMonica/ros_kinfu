#ifndef KINFU_SHIFT_CHECKER_H
#define KINFU_SHIFT_CHECKER_H

#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include "parameters.h"

#include <string>

class KinfuShiftCheckerFactory
{
  public:
  typedef pcl::gpu::kinfuLS::KinfuTracker KinfuTracker;

  static KinfuTracker::IShiftChecker::Ptr BuildShiftChecker(const std::string description); // NULL if default
};

#endif // KINFU_SHIFT_CHECKER_H
