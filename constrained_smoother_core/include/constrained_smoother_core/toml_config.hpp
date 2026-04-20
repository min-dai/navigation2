#pragma once

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "constrained_smoother_core/types.hpp"

namespace constrained_smoother_core
{

struct Config
{
  SmootherParams smoother;
  OptimizerParams optimizer;
};

inline std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t\n\r");
  if (b == std::string::npos) {
    return "";
  }
  const auto e = s.find_last_not_of(" \t\n\r");
  return s.substr(b, e - b + 1);
}

inline std::vector<double> parseArray(const std::string & raw)
{
  std::string s = trim(raw);
  if (s.size() < 2 || s.front() != '[' || s.back() != ']') {
    throw std::runtime_error("invalid array");
  }
  s = s.substr(1, s.size() - 2);
  std::vector<double> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    tok = trim(tok);
    if (!tok.empty()) {
      out.push_back(std::stod(tok));
    }
  }
  return out;
}

inline Config loadTomlConfig(const std::string & file_path)
{
  std::ifstream ifs(file_path);
  if (!ifs) {
    throw std::runtime_error("failed to open config: " + file_path);
  }

  Config config;
  std::string section;
  std::string line;
  while (std::getline(ifs, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    if (line.front() == '[' && line.back() == ']') {
      section = trim(line.substr(1, line.size() - 2));
      continue;
    }

    const auto eq = line.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    std::string key = trim(line.substr(0, eq));
    std::string value = trim(line.substr(eq + 1));

    auto as_bool = [](const std::string & v) {return v == "true" || v == "1";};

    if (section == "smoother") {
      if (key == "smooth_weight") {config.smoother.smooth_weight = std::stod(value);} else if (key == "cost_weight") {config.smoother.cost_weight = std::stod(value);} else if (key == "cusp_cost_multiplier") {config.smoother.cusp_cost_multiplier = std::stod(value);} else if (key == "cusp_zone_length") {config.smoother.cusp_zone_length = std::stod(value);} else if (key == "distance_weight") {config.smoother.distance_weight = std::stod(value);} else if (key == "curve_weight") {config.smoother.curve_weight = std::stod(value);} else if (key == "minimum_turning_radius") {config.smoother.minimum_turning_radius = std::stod(value);} else if (key == "path_downsampling_factor") {config.smoother.path_downsampling_factor = std::stoi(value);} else if (key == "keep_goal_orientation") {config.smoother.keep_goal_orientation = as_bool(value);} else if (key == "keep_start_orientation") {config.smoother.keep_start_orientation = as_bool(value);} else if (key == "reversing_enabled") {config.smoother.reversing_enabled = as_bool(value);} else if (key == "cost_check_points") {config.smoother.cost_check_points = parseArray(value);} 
    } else if (section == "optimizer") {
      if (key == "debug_optimizer") {config.optimizer.debug_optimizer = as_bool(value);} else if (key == "max_iterations") {config.optimizer.max_iterations = std::stoi(value);} else if (key == "gradient_tol") {config.optimizer.gradient_tol = std::stod(value);} else if (key == "fn_tol") {config.optimizer.fn_tol = std::stod(value);} else if (key == "param_tol") {config.optimizer.param_tol = std::stod(value);} 
    }
  }

  return config;
}

}  // namespace constrained_smoother_core
