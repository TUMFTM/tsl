// Copyright 2024 Simon Sagmeister
#pragma once

#include <memory>
#include <string>
#include <tsl_logger_cpp/types.hpp>
#include <unordered_map>
#include <unordered_set>
#include <variant>
namespace tam::tsl
{
struct CompressionStrategy
{
  virtual ~CompressionStrategy() = default;
  using SharedPtr = std::shared_ptr<CompressionStrategy>;
  // clang-format off
  #define LOG_AS_DEFINITION(dtype, typename)  \
    virtual bool log_as_##typename(std::string const & signal_name, \
      data_variant_t const & value) const = 0;
  TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(LOG_AS_DEFINITION)
  #undef LOG_AS_DEFINITION
  // clang-format on
};
namespace builtin_compression_strategies
{
/// @brief Apply no compression. Publish signals as they were logged.
struct Uncompressed : public CompressionStrategy
{
  virtual ~Uncompressed() = default;
  // clang-format off
  #define LOG_AS_DEFINITION(dtype, typename)  \
    virtual bool log_as_##typename(std::string const & , /*NOLINT*/ \
      data_variant_t const & value) const override {\
    return std::holds_alternative<dtype>(value);}
  TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(LOG_AS_DEFINITION)
  #undef LOG_AS_DEFINITION
  // clang-format on
};
/// @brief  A compression strategy that publishes 64 bit floating point values as 32 bit floating
/// point
class DoubleAsFloat : public Uncompressed
{
protected:
  std::unordered_set<std::string> uncompressed_signals_;

public:
  virtual ~DoubleAsFloat() = default;
  /// @brief Disable the compression for a specific signal
  void disable_compression(std::string const & signal_name)
  {
    uncompressed_signals_.insert(signal_name);
  }
  bool log_as_float64(std::string const & signal_name, data_variant_t const &) const override
  {
    return uncompressed_signals_.find(signal_name) != uncompressed_signals_.end();
  }
  bool log_as_float32(std::string const & signal_name, data_variant_t const & value) const override
  {
    return (
      std::holds_alternative<float>(value) ||
      (std::holds_alternative<double>(value) &&
       (uncompressed_signals_.find(signal_name) == uncompressed_signals_.end())));
  }
};
/// @brief Compresses 64 bit numeric values to 32 bit numeric values before publishing
class Compress64to32Bit : public DoubleAsFloat
{
public:
  virtual ~Compress64to32Bit() = default;
  bool log_as_int64(std::string const & signal_name, data_variant_t const &) const override
  {
    return uncompressed_signals_.find(signal_name) != uncompressed_signals_.end();
  }
  bool log_as_uint64(std::string const & signal_name, data_variant_t const &) const override
  {
    return uncompressed_signals_.find(signal_name) != uncompressed_signals_.end();
  }
  bool log_as_int32(std::string const & signal_name, data_variant_t const & value) const override
  {
    return (
      std::holds_alternative<int32_t>(value) ||
      (std::holds_alternative<int64_t>(value) &&
       (uncompressed_signals_.find(signal_name) == uncompressed_signals_.end())));
  }
  bool log_as_uint32(std::string const & signal_name, data_variant_t const & value) const override
  {
    return (
      std::holds_alternative<uint32_t>(value) ||
      (std::holds_alternative<uint64_t>(value) &&
       (uncompressed_signals_.find(signal_name) == uncompressed_signals_.end())));
  }
};
}  // namespace builtin_compression_strategies
}  // namespace tam::tsl
