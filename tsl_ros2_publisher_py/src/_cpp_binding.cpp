// Copyright 2024 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <tsl_logger_cpp/base.hpp>
#include <tsl_ros2_publisher_cpp/compression.hpp>
#include <tsl_ros2_publisher_cpp/helpers.hpp>
#include <tsl_ros2_publisher_cpp/msg_creator.hpp>

namespace py = pybind11;
PYBIND11_MODULE(_cpp_binding, m)
{
  m.def("clean_signal_name", &tam::tsl::helpers::clean_signal_name);
  py::class_<tam::tsl::CompressionStrategy, tam::tsl::CompressionStrategy::SharedPtr>(
    m, "CompressionStrategy");
  py::class_<
    tam::tsl::builtin_compression_strategies::Uncompressed,
    std::shared_ptr<tam::tsl::builtin_compression_strategies::Uncompressed>,
    tam::tsl::CompressionStrategy>(m, "Uncompressed")
    .def(py::init());
  py::class_<
    tam::tsl::builtin_compression_strategies::DoubleAsFloat,
    tam::tsl::builtin_compression_strategies::Uncompressed,
    std::shared_ptr<tam::tsl::builtin_compression_strategies::DoubleAsFloat>>(m, "DoubleAsFloat")
    .def(py::init())
    .def(
      "disable_compression",
      &tam::tsl::builtin_compression_strategies::DoubleAsFloat::disable_compression);
  py::class_<
    tam::tsl::builtin_compression_strategies::Compress64to32Bit,
    tam::tsl::builtin_compression_strategies::DoubleAsFloat,
    std::shared_ptr<tam::tsl::builtin_compression_strategies::Compress64to32Bit>>(
    m, "Compress64to32Bit")
    .def(py::init());
  // msg creator
  py::class_<tam::tsl::MsgCreator, std::shared_ptr<tam::tsl::MsgCreator>>(m, "MsgCreator")
    .def(py::init<tam::tsl::CompressionStrategy::SharedPtr>())
    .def(
      "get_definition",
      [](tam::tsl::MsgCreator const & msg_creator) {
        auto const & rcl_msg = msg_creator.get_definition().get_rcl_serialized_message();
        return py::bytes(std::string(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length));
      })
    .def(
      "get_values",
      [](tam::tsl::MsgCreator const & msg_creator) {
        auto const & rcl_msg = msg_creator.get_values().get_rcl_serialized_message();
        return py::bytes(std::string(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length));
      })
    .def(
      "process_data",
      [](tam::tsl::MsgCreator & msg_creator, tam::tsl::LoggerAccessInterface * logger) {
        msg_creator.process_data(logger->get_data());
      });
}
