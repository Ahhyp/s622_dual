#pragma once

#include "fairino_planning_core/types.h"

namespace fairino_planning
{

class CollisionInterface
{
public:
  virtual ~CollisionInterface() = default;

  virtual bool isStateValid(const JointArray & q) const = 0;

  virtual bool isMotionValid(
    const JointArray & from,
    const JointArray & to,
    double resolution) const = 0;
};

}  // namespace fairino_planning