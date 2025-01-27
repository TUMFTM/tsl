// Copyright 2024 Simon Sagmeister
#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <variant>
#include <vector>

#include "tsl_logger_cpp/types.hpp"
namespace tam::tsl
{
class LoggerAccessInterface
{
public:
  typedef std::shared_ptr<LoggerAccessInterface> SharedPtr;
  typedef std::unique_ptr<LoggerAccessInterface> UniquePtr;

public:
  virtual ~LoggerAccessInterface() = default;
  virtual data_map_t const & get_data() const = 0;
  virtual void get_data(data_map_t & map, std::string const & prefix = "") const = 0;
};
}  // namespace tam::tsl
