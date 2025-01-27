// Copyright 2024 Simon Sagmeister
#pragma once

#include "tsl_logger_cpp/types.hpp"
namespace tam::tsl
{
namespace type_support
{
class TypeSupportInterface
{
public:
  virtual ~TypeSupportInterface() = default;
  virtual void log(std::string name, data_variant_t value) = 0;
};
}  // namespace type_support
}  // namespace tam::tsl