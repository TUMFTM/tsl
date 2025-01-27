// Copyright 2024 Simon Sagmeister

#include <iostream>

#include "tsl_logger_cpp/base.hpp"
#include "tsl_logger_cpp/type_support.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
// As shown in the value_logger.cpp example, you can just log any numeric value as well as
// stl containers with numeric values.
// However you can also log custom types
// See our custom type here:
struct MyCustomType
{
  int a;
  double b;
};
struct MyBigCustomType
{
  MyCustomType my_other_custom_type;
  float c;
};
// The logger needs a type support definition so that it knows how to log your custom type
// This is done by specifying a template specialization of the following kind
// =================================================================================================
namespace tam::tsl::type_support
{
template <>
inline void log<MyCustomType>(
  TypeSupportInterface * logger, std::string const & name, MyCustomType const & value)
{
  // You can use the type support system to again log any type that is supported by the interface.
  type_support::log(logger, name + "/a", value.a);
  type_support::log(logger, name + "/b", value.b);
}
}  // namespace tam::tsl::type_support
// =================================================================================================

// Type support function for our big custom type
// ==================================================================================================
namespace tam::tsl::type_support
{
template <>
inline void log<MyBigCustomType>(
  TypeSupportInterface * logger, std::string const & name, MyBigCustomType const & value)
{
  type_support::log(
    logger, name + "/my_other_custom_type",
    value.my_other_custom_type);  // Here we utilize the type support definition for
                                  // MyCustomType that we already defined above
  type_support::log(logger, name + "/c", value.c);
}
}  // namespace tam::tsl::type_support
// ==================================================================================================
//
//
//
//
/*
 All loggers, no matter if its a reference or a value logger, can now log your custom types
*/
// After specifying this function once, the logger can now log your type
int main()
{
  tam::tsl::ValueLogger logger;

  MyCustomType custom_type{1, 2.0};

  logger.log("custom_type", custom_type);

  // Additionally by adding type support for your type, you can now log any stl container
  // (vector, array, map, ....) specialized for your type
  std::vector<MyBigCustomType> vec = {{{1, 2.0}, 3.0}, {{3, 4.0}, 5.0}};
  logger.log("custom_type_vector", vec);

  std::cout << "We logged the following signals:" << std::endl;
  for (auto const & signal : logger.get_data()) {
    std::cout << "- " << signal.first << std::endl;
  }

  return 0;
}
