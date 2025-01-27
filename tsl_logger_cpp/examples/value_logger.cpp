// Copyright 2024 Simon Sagmeister

#include "tsl_logger_cpp/value_logger.hpp"

#include <iostream>

#include "tsl_logger_cpp/base.hpp"
int main()
{
  tam::tsl::ValueLogger logger1;

  // You can just log any numeric value
  logger1.log("double_test", 1.0);
  logger1.log("int_test", 2);
  logger1.log("float_test", 3.0f);
  std::int64_t int64_val = 4;
  logger1.log("int64_test", int64_val);

  // You can even log a whole range of standard containers
  std::vector<float> vec = {5.0, 6, 7};
  std::array<double, 3> arr = {5.0, 6, 7};
  std::unordered_map<std::string, int> map = {{"sigal_a", 1}, {"signal_b", 2}};
  logger1.log("Testvector", vec);
  logger1.log("Testarray", arr);
  logger1.log("Testmap", map);

  // You can even log the values contained in another logger
  // Note: This will copy the data over. For a faster way of combining loggers, see the composition
  // example
  tam::tsl::ValueLogger logger2;
  logger2.log("logger1_data", logger1.get_data());

  std::cout << "We logged the following signals in Logger 1:" << std::endl;
  for (auto const & signal : logger1.get_data()) {
    std::cout << "- " << signal.first << std::endl;
  }
  std::cout << std::endl << std::endl << std::endl;

  std::cout << "We logged the following signals in Logger 2:" << std::endl;
  for (auto const & signal : logger2.get_data()) {
    std::cout << "- " << signal.first << std::endl;
  }

  return 0;
}
