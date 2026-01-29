#include "confs/IceoryxServiceDescription.hpp"
#include "types/IceoryxServiceDescription.hpp"

#include "iox/string.hpp"

types::IceoryxServiceDescription::IceoryxServiceDescription(
    const confs::IceoryxServiceDescription &description)
    : description({{iox::TruncateToCapacity, description.service.c_str()},
                   {iox::TruncateToCapacity, description.instance.c_str()},
                   {iox::TruncateToCapacity, description.event.c_str()}}) {}
