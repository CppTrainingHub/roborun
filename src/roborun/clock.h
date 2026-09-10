#pragma once

#include <limits>
#include <stdexcept>

#include "roborun/types.h"

namespace roborun {

class Clock {
 public:
  virtual ~Clock() = default;

  virtual BusinessTime Now() const = 0;
  virtual void AdvanceTo(BusinessTime time) {
    if (time != Now()) {
      throw std::logic_error("clock cannot be advanced by the runtime");
    }
  }
};

class VirtualClock final : public Clock {
 public:
  BusinessTime Now() const override { return now_; }

  void AdvanceTo(BusinessTime time) override {
    if (time < now_) {
      throw std::invalid_argument("virtual clock cannot move backward");
    }
    now_ = time;
  }

  void AdvanceBy(BusinessTime duration) {
    if (duration < 0 || now_ > std::numeric_limits<BusinessTime>::max() - duration) {
      throw std::invalid_argument("virtual clock duration is outside the business time range");
    }
    AdvanceTo(now_ + duration);
  }

 private:
  BusinessTime now_ = 0;
};

}  // namespace roborun
