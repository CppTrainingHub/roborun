#pragma once

#include "roborun/config.h"
#include "roborun/types.h"

namespace roborun {

ValidationResult ValidateProgram(const Program& program,
                                 const RobotConfiguration* robot_configuration = nullptr,
                                 const PointCatalog* point_catalog = nullptr,
                                 const IOConfiguration* io_configuration = nullptr,
                                 const ToolConfiguration* tool_configuration = nullptr);

}  // namespace roborun
