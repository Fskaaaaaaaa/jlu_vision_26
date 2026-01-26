#include "confs/IceoryxServiceDescription.hpp"
#include "iox/string.hpp"

iox::capro::ServiceDescription confs::IceoryxServiceDescription::get() {
  return {{iox::TruncateToCapacity, service.c_str()},
          {iox::TruncateToCapacity, instance.c_str()},
          {iox::TruncateToCapacity, event.c_str()}};
}
