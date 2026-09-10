#pragma once

#include <istream>
#include <string>

#include "roborun/types.h"

namespace roborun {

ParseResult ParseProgram(std::istream& input, const std::string& source);

}  // namespace roborun
