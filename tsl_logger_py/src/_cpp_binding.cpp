// Copyright 2024 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <variant>
#include <vector>

#include "tsl_logger_cpp/base.hpp"
#include "tsl_logger_cpp/composer.hpp"
#include "tsl_logger_cpp/types.hpp"
#include "tsl_logger_cpp/value_logger.hpp"

// region macros
#define BIND_LOG_TYPE_SAFE(dtype, typename)                                             \
  .def(                                                                                 \
    "log_" #typename,                                                                   \
    [](tam::tsl::ValueLogger & logger, std::string const & name, dtype const & value) { \
      logger.log(name, value);                                                          \
    })
// endregion

namespace py = pybind11;
PYBIND11_MODULE(_cpp_binding, m)
{
  py::class_<tam::tsl::LoggerAccessInterface, tam::tsl::LoggerAccessInterface::SharedPtr>(
    m, "LoggerAccessInterface")
    .def("get_data", [](tam::tsl::LoggerAccessInterface const & itf) {
      return itf.get_data();
    });  // why does this not compile with the overload cast....
  // Composer
  py::class_<
    tam::tsl::LoggerComposer, tam::tsl::LoggerAccessInterface, tam::tsl::LoggerComposer::SharedPtr>(
    m, "LoggerComposer")
    .def(py::init())
    .def(py::init<std::vector<tam::tsl::LoggerAccessInterface::SharedPtr>>())
    .def("register_logger", &tam::tsl::LoggerComposer::register_logger);
  // Value Manager
  py::class_<
    tam::tsl::ValueLogger, tam::tsl::LoggerAccessInterface, tam::tsl::ValueLogger::SharedPtr>(
    m, "ValueLogger")
    .def(py::init())  // clang-format off
    TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(BIND_LOG_TYPE_SAFE);  // clang-format on
}
