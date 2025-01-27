// Copyright 2024 Simon Sagmeister

#include "tsl_logger_cpp/reference_logger.hpp"

#include <iostream>

#include "tsl_logger_cpp/base.hpp"
// DISCLAIMER:
/// ================================================================================================
// For certain applications (e.g. logging is triggered asynchronously) it
// may be beneficial use a reference logger instead of a value logger.
// The reference logger does not directly store the logged values but instead
// stores raw pointers to the values to be logged. If you want to get the logged values,
// the reference manager dereferences the pointers and returns the values.
// However: YOU ARE RESPONSIBLE FOR KEEPING THE PTR VALID
//
//
// =================================================================================================

class MySoftwareModule
{
private:
  int a = 0, b = 0, c = 0;
  tam::tsl::ReferenceLogger::SharedPtr logger_ptr_ = std::make_shared<tam::tsl::ReferenceLogger>();

public:
  MySoftwareModule()
  {
    setup_logger();  // You only need to call this once, since we store the ptr to the values
  }
  void setup_logger()
  {
    logger_ptr_->log("a", &a);
    logger_ptr_->log("b", &b);
    logger_ptr_->log("c", &c);
  }
  void step()
  {
    a++;
    b++;
    c++;
    // No need to explicitly loog the signals here, since the reference logger already stores the
    // memory location
  }
  tam::tsl::LoggerAccessInterface::SharedPtr get_logger() { return logger_ptr_; }
};
int main()
{
  MySoftwareModule module;
  for (int i = 0; i < 3; i++) {
    module.step();
    // Print the logged values
    std::cout << "For iteration " << i << " we logged the following signals:";
    for (auto const & signal : module.get_logger()->get_data()) {
      std::cout << " " << signal.first << ":" << std::get<int>(signal.second);
    }
    std::cout << std::endl;
  }

  return 0;
}
