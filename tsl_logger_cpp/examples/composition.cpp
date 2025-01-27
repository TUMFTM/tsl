// Copyright 2024 Simon Sagmeister

#include <iostream>

#include "tsl_logger_cpp/base.hpp"
#include "tsl_logger_cpp/composer.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
int main()
{
  tam::tsl::ValueLogger::SharedPtr logger1 = std::make_shared<tam::tsl::ValueLogger>();
  tam::tsl::ValueLogger::SharedPtr logger2 = std::make_shared<tam::tsl::ValueLogger>();
  tam::tsl::ValueLogger::SharedPtr logger3 = std::make_shared<tam::tsl::ValueLogger>();

  logger1->log("a", 1.0);
  logger2->log("b", 1.0);
  logger3->log("c", 1.0);

  // If you have multiple loggers, you can access them all as they would be
  // just a single instance. For this we use a LoggerComposer object.
  // Create the composer from a shared ptr to logger1 and logger2
  tam::tsl::LoggerComposer composer{
    std::vector<tam::tsl::LoggerAccessInterface::SharedPtr>({logger1, logger2})};

  // You can also add loggers later on
  // It is also possible to add a prefix to each added logger.
  // This prefix applies to all signal names within the logger
  composer.register_logger(logger3, "logger3/");

  std::cout << "The composer logged the following values:" << std::endl;
  for (auto const & signal : composer.get_data()) {
    std::cout << "- " << signal.first << std::endl;
  }

  return 0;
}
