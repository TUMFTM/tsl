// Copyright 2024 Simon Sagmeister
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "tsl_logger_cpp/base.hpp"
#include "tsl_logger_cpp/type_support.hpp"
#include "tsl_logger_cpp/types.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
namespace tam::tsl
{
/// @brief A logger that logs by storing a raw pointer to the values beeing logged.
/// @warning You are responsible to keep the pointers stored in this logger valid!
/// @warning Otherwise your program will crash or result in undefined behavior!
class ReferenceLogger : public LoggerAccessInterface
{
public:
  typedef std::shared_ptr<ReferenceLogger> SharedPtr;
  typedef std::unique_ptr<ReferenceLogger> UniquePtr;

private:
  mutable ValueLogger value_logger_{};  // Use as cache for the values to be stored
  std::vector<std::function<void(ValueLogger &)>>
    update_callbacks_;  // Store the destructors of the stored pointers

private:
  void update_values() const
  {
    for (auto const & callback : update_callbacks_) {
      callback(value_logger_);
    }
  }

public:
  /// @brief Log any type by reference, as long as the type is supported by the type_support
  /// backend.
  /// @note You can easily add support for logging custom types.
  /// @note For this, look at the log_custom_types.cpp example.
  /// @warning You are responsible to keep the pointers stored in this logger valid!
  /// @warning Otherwise your program will crash or result in undefined behavior!
  /// @param name
  /// @param ptr | Memory address of the value beeing logged. You are responsible for keeping the
  /// ptr valid | T const *
  template <typename T>
  void log(std::string const & name, T const * ptr)
  {
    // Create an update callback
    update_callbacks_.emplace_back([name, ptr](ValueLogger & logger) { logger.log(name, *ptr); });
  }
  void get_data(data_map_t & map, std::string const & prefix = "") const override
  {
    // Copy over all elements
    update_values();
    value_logger_.get_data(map, prefix);
  }
  data_map_t const & get_data() const override
  {
    update_values();
    return value_logger_.get_data();
  }
};
};  // namespace tam::tsl
