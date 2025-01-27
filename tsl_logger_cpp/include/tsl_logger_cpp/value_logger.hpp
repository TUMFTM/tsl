// Copyright 2024 Simon Sagmeister
#pragma once

#include <memory>
#include <string>

#include "tsl_logger_cpp/base.hpp"
#include "tsl_logger_cpp/type_support.hpp"
#include "tsl_logger_cpp/types.hpp"
namespace tam::tsl
{
/// @brief A logger that logs signals by value
class ValueLogger : public LoggerAccessInterface, public type_support::TypeSupportInterface
{
public:
  typedef std::shared_ptr<ValueLogger> SharedPtr;
  typedef std::unique_ptr<ValueLogger> UniquePtr;

private:
  data_map_t data_{};

public:
  /// @brief Log a supported primitive numeric datatype type by value
  /// @param name
  /// @param value
  void log(std::string name, data_variant_t value) override { data_[name] = value; }
  // macro for disabling accidentally linking against the template for default types
#define TAM_TSL_DISABLE_FOR_SUPPORTED_TYPE(dtype, typename) std::is_same_v<dtype, T>
  /// @brief Log any type by value, as long as the type is supported by the type_support backend.
  /// @note You can easily add support for logging custom types.
  /// @note For this, look at the log_custom_types.cpp example.
  /// @param name
  /// @param value: T
  template <typename T>
  std::enable_if_t<!(TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE_WITH_SEPARATOR(
    TAM_TSL_DISABLE_FOR_SUPPORTED_TYPE, ||))>
  log(std::string const & name, T const & value)
  {
    type_support::log(this, name, value);
  }
  data_map_t const & get_data() const override { return data_; }
  void get_data(data_map_t & map, std::string const & prefix = "") const override
  {
    // Copy over all elements
    for (auto const & element : data_) {
      map[prefix + element.first] = element.second;
    }
  }
#undef TAM_TSL_DISABLE_FOR_SUPPORTED_TYPE
};
}  // namespace tam::tsl
