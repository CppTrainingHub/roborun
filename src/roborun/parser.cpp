#include "roborun/parser.h"

#include <cctype>
#include <cerrno>
#include <charconv>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "roborun/config.h"

namespace roborun {
namespace {

std::vector<std::string> SplitWhitespace(const std::string& line) {
  std::istringstream stream(line);
  std::vector<std::string> tokens;
  std::string token;
  while (stream >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

bool ParseDouble(const std::string& token, double* value) {
  errno = 0;
  char* end = nullptr;
  const double parsed = std::strtod(token.c_str(), &end);
  if (errno != 0 || end == token.c_str() || *end != '\0') {
    return false;
  }
  *value = parsed;
  return true;
}

void AddDiagnostic(ParseResult* result, const std::string& source, std::size_t line_number,
                   DiagnosticCode code, const std::string& message) {
  result->diagnostics.push_back(Diagnostic{code, source, line_number, 1, "", message});
}

bool ParseInt(const std::string& token, int* value) {
  const char* first = token.data();
  const char* last = first + token.size();
  const auto [end, error] = std::from_chars(first, last, *value);
  return error == std::errc() && end == last;
}

bool ParseMilliseconds(const std::string& token, BusinessTime* value) {
  if (token.empty() || token[0] == '-') {
    return false;
  }
  const char* first = token.data();
  const char* last = first + token.size();
  std::int64_t parsed = 0;
  const auto [end, error] = std::from_chars(first, last, parsed);
  if (error != std::errc() || end != last || parsed < 0) {
    return false;
  }
  *value = parsed;
  return true;
}

bool LooksLikePointReference(const std::vector<std::string>& tokens) {
  return tokens.size() >= 2 && !tokens[1].empty() &&
         std::isalpha(static_cast<unsigned char>(tokens[1][0])) != 0;
}

Command MakeCommand(CommandType type, std::size_t line_number) {
  Command command{};
  command.type = type;
  command.line = line_number;
  return command;
}

}  // namespace

ParseResult ParseProgram(std::istream& input, const std::string& source) {
  ParseResult result;
  result.program.source = source;

  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const std::vector<std::string> tokens = SplitWhitespace(line);
    if (tokens.empty()) {
      continue;
    }

    if (tokens[0] == "SERVO_ON") {
      if (tokens.size() != 1) {
        AddDiagnostic(&result, source, line_number, DiagnosticCode::kTaskSyntax,
                      "SERVO_ON does not accept parameters");
        continue;
      }
      result.program.commands.push_back(MakeCommand(CommandType::kServoOn, line_number));
      continue;
    }

    if (tokens[0] == "SERVO_OFF") {
      if (tokens.size() != 1) {
        AddDiagnostic(&result, source, line_number, DiagnosticCode::kTaskSyntax,
                      "SERVO_OFF does not accept parameters");
        continue;
      }
      result.program.commands.push_back(MakeCommand(CommandType::kServoOff, line_number));
      continue;
    }

    if (tokens[0] == "STOP") {
      if (tokens.size() != 1) {
        AddDiagnostic(&result, source, line_number, DiagnosticCode::kTaskSyntax,
                      "STOP does not accept parameters");
        continue;
      }
      result.program.commands.push_back(MakeCommand(CommandType::kStop, line_number));
      continue;
    }

    if (tokens[0] == "DELAY") {
      Command command = MakeCommand(CommandType::kDelay, line_number);
      if (tokens.size() != 2 || !ParseMilliseconds(tokens[1], &command.duration_ms)) {
        AddDiagnostic(&result, source, line_number, DiagnosticCode::kInvalidDuration,
                      "DELAY requires a non-negative integer number of milliseconds");
        continue;
      }
      result.program.commands.push_back(std::move(command));
      continue;
    }

    if (tokens[0] == "SET_DO") {
      Command command = MakeCommand(CommandType::kSetDo, line_number);
      if (tokens.size() != 3 || !IsValidPointName(tokens[1]) ||
          (tokens[2] != "ON" && tokens[2] != "OFF")) {
        AddDiagnostic(&result, source, line_number, DiagnosticCode::kTaskSyntax,
                      "SET_DO requires <NAME> <ON|OFF>");
        continue;
      }
      command.signal_name = tokens[1];
      command.signal_value = tokens[2] == "ON";
      result.program.commands.push_back(std::move(command));
      continue;
    }

    if (tokens[0] == "WAIT_DI") {
      Command command = MakeCommand(CommandType::kWaitDi, line_number);
      BusinessTime timeout = 0;
      if (tokens.size() != 5 || !IsValidPointName(tokens[1]) ||
          (tokens[2] != "ON" && tokens[2] != "OFF") || tokens[3] != "TIMEOUT" ||
          !ParseMilliseconds(tokens[4], &timeout)) {
        AddDiagnostic(&result, source, line_number, DiagnosticCode::kTaskSyntax,
                      "WAIT_DI requires <NAME> <ON|OFF> TIMEOUT <milliseconds>");
        continue;
      }
      command.signal_name = tokens[1];
      command.signal_value = tokens[2] == "ON";
      command.timeout_ms = timeout;
      result.program.commands.push_back(std::move(command));
      continue;
    }

    if (tokens[0] == "SET_TOOL") {
      Command command = MakeCommand(CommandType::kSetTool, line_number);
      BusinessTime timeout = 0;
      if (tokens.size() != 5 || !IsValidPointName(tokens[1]) || tokens[3] != "TIMEOUT" ||
          !ParseMilliseconds(tokens[4], &timeout) ||
          (tokens[2] != "OPEN" && tokens[2] != "CLOSED")) {
        AddDiagnostic(&result, source, line_number, DiagnosticCode::kTaskSyntax,
                      "SET_TOOL requires <NAME> <OPEN|CLOSED> TIMEOUT <milliseconds>");
        continue;
      }
      command.tool_name = tokens[1];
      command.tool_position = tokens[2] == "OPEN" ? ToolPosition::kOpen : ToolPosition::kClosed;
      command.timeout_ms = timeout;
      result.program.commands.push_back(std::move(command));
      continue;
    }

    if (tokens[0] != "MOVEJ") {
      AddDiagnostic(&result, source, line_number, DiagnosticCode::kTaskSyntax,
                    "unknown command: " + tokens[0]);
      continue;
    }

    if ((tokens.size() == 4 || tokens.size() == 6) && tokens[2] == "SPEED") {
      Command command = MakeCommand(CommandType::kMoveJ, line_number);
      command.point_reference = tokens[1];
      if (command.point_reference.size() > 64) {
        AddDiagnostic(&result, source, line_number, DiagnosticCode::kInvalidPointName,
                      "point reference exceeds 64 characters");
        continue;
      }
      if (!IsValidPointName(command.point_reference)) {
        AddDiagnostic(&result, source, line_number, DiagnosticCode::kInvalidPointName,
                      "point reference must be an uppercase identifier");
        continue;
      }
      if (!ParseInt(tokens[3], &command.movej.speed_percent)) {
        AddDiagnostic(&result, source, line_number, DiagnosticCode::kInvalidSpeed,
                      "SPEED value must be an integer from 1 to 100");
        continue;
      }
      if (tokens.size() == 6) {
        if (tokens[4] != "TIMEOUT" ||
            !ParseMilliseconds(tokens[5], &command.timeout_ms.emplace())) {
          AddDiagnostic(&result, source, line_number, DiagnosticCode::kInvalidDuration,
                        "MOVEJ TIMEOUT requires a non-negative integer number of milliseconds");
          continue;
        }
      }
      result.program.commands.push_back(std::move(command));
      continue;
    }

    if (LooksLikePointReference(tokens)) {
      AddDiagnostic(&result, source, line_number, DiagnosticCode::kInvalidSpeed,
                    "named MOVEJ requires SPEED <1-100>");
      continue;
    }
    if (tokens.size() < kJointCount + 1) {
      AddDiagnostic(&result, source, line_number, DiagnosticCode::kTaskSyntax,
                    "MOVEJ requires 6 joint targets or <POINT> SPEED <1-100>");
      continue;
    }
    if (tokens.size() != kJointCount + 1 && tokens.size() != kJointCount + 3) {
      AddDiagnostic(&result, source, line_number, DiagnosticCode::kTaskSyntax,
                    "MOVEJ requires exactly 6 joint targets in radians");
      continue;
    }

    Command command = MakeCommand(CommandType::kMoveJ, line_number);
    bool has_invalid_number = false;
    for (std::size_t index = 0; index < kJointCount; ++index) {
      if (!ParseDouble(tokens[index + 1], &command.movej.targets[index])) {
        AddDiagnostic(&result, source, line_number, DiagnosticCode::kTaskSyntax,
                      "MOVEJ target " + std::to_string(index + 1) + " is not a number");
        has_invalid_number = true;
        break;
      }
    }
    if (!has_invalid_number && tokens.size() == kJointCount + 3) {
      if (tokens[kJointCount + 1] != "TIMEOUT" ||
          !ParseMilliseconds(tokens[kJointCount + 2], &command.timeout_ms.emplace())) {
        AddDiagnostic(&result, source, line_number, DiagnosticCode::kInvalidDuration,
                      "MOVEJ TIMEOUT requires a non-negative integer number of milliseconds");
        has_invalid_number = true;
      }
    }
    if (!has_invalid_number) {
      result.program.commands.push_back(command);
    }
  }

  return result;
}

}  // namespace roborun
