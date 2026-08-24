#pragma once

#include "fairino_planning_core/types.h"

namespace fairino_planning
{

  class PlanningAlgorithm
  {
  public:
    // 虚析构函数， 只要是 多态的基类， 就一定要 虚析构函数
    // 这样 比如在调用这个类的子类， delete 的时候才会调用 子类 和 基类 的析构函数
    virtual ~PlanningAlgorithm() = default;
    // = 0：纯虚函数——基类不提供实现，强制派生类必须实现。
    virtual PlanResult plan(const PlanRequest &request) = 0;
  };

} // namespace fairino_planning