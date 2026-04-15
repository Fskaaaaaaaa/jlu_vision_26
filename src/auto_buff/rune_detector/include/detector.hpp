#pragma once
#include "single.hpp"


namespace auto_buff {
class RuneDetector : public Single<RuneDetector> {
  friend class Single<RuneDetector>;

protected:
  RuneDetector();
  ~RuneDetector();

public:
  
};
}