// Copyright 2024 Simon Sagmeister

#include <chrono>
#include <iostream>

#include "tsl_logger_cpp/base.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tsl_ros2_publisher_cpp/compression.hpp"
#include "tsl_ros2_publisher_cpp/msg_creator.hpp"
int main()
{
  constexpr std::size_t NUM_LOG_SIGNALS = 1e3;
  constexpr std::size_t NUM_REPETITIONS = 1e4;

  tam::tsl::ValueLogger logger;  // Create the logger

  // Create a an array of random float
  std::array<double, NUM_LOG_SIGNALS> arr;

  std::chrono::nanoseconds duration_initial_cycle{0};
  std::chrono::nanoseconds duration_subsequent_cycles{0};
  std::chrono::nanoseconds duration_log_signals{0};

  std::size_t values_msg_size = 0;
  std::size_t def_msg_size = 0;
  for (std::size_t i = 0; i < NUM_REPETITIONS; ++i) {
    // Update the array with random values so that the compiler cannot optimize
    for (std::size_t i = 0; i < NUM_LOG_SIGNALS; ++i) {
      arr[i] = static_cast<double>(rand() / RAND_MAX);  // NOLINT
    }
    auto start = std::chrono::high_resolution_clock::now();
    logger.log("my_benchmark_data", arr);
    auto end = std::chrono::high_resolution_clock::now();
    duration_log_signals += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

    // Create a msg creator - This is the main workload of the TSLPublisher, everything else is just
    // publishing a serialized ros2 message
    auto compression_strategy =
      std::make_shared<tam::tsl::builtin_compression_strategies::Compress64to32Bit>();
    tam::tsl::MsgCreator msg_creator(compression_strategy);

    // Benchmark creating a message for the first time
    start = std::chrono::high_resolution_clock::now();
    msg_creator.process_data(logger.get_data());
    end = std::chrono::high_resolution_clock::now();
    duration_initial_cycle += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

    // Store the msg sizes
    values_msg_size = msg_creator.get_values().get_rcl_serialized_message().buffer_length;
    def_msg_size = msg_creator.get_definition().get_rcl_serialized_message().buffer_length;

    // Update the array with random values so that the compiler cannot optimize
    for (std::size_t i = 0; i < NUM_LOG_SIGNALS; ++i) {
      arr[i] = static_cast<double>(rand() / RAND_MAX);  // NOLINT
    }
    logger.log("my_benchmark_data", arr);

    // Benchmark creating a message for the subsequent times
    start = std::chrono::high_resolution_clock::now();
    msg_creator.process_data(logger.get_data());
    end = std::chrono::high_resolution_clock::now();
    duration_subsequent_cycles += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
  }
  std::cout << "Logging signals:\n=======================================" << std::endl;
  std::cout << "Logging " << NUM_LOG_SIGNALS << " signals " << NUM_REPETITIONS << " times took "
            << duration_log_signals.count() << " ns" << std::endl;
  // Print the average per logged signal
  std::cout << "Average time per signal: "
            << duration_log_signals.count() / (NUM_LOG_SIGNALS * NUM_REPETITIONS) << " ns"
            << std::endl;

  std::cout << "\nCreating messages:\n=======================================" << std::endl;
  std::cout << "Creating a message for the first time took on average: "
            << duration_initial_cycle.count() / NUM_REPETITIONS << " ns"
            << " | Average time per signal: "
            << duration_initial_cycle.count() / (NUM_LOG_SIGNALS * NUM_REPETITIONS) << " ns"
            << std::endl;
  std::cout << "Creating a message for the subsequent times took on average: "
            << duration_subsequent_cycles.count() / NUM_REPETITIONS << " ns"
            << " | Average time per signal: "
            << duration_subsequent_cycles.count() / (NUM_LOG_SIGNALS * NUM_REPETITIONS) << " ns"
            << std::endl;

  std::cout << "\nMessage sizes:\n=======================================" << std::endl;
  std::cout << "Definition message size: " << def_msg_size << " bytes" << std::endl;
  std::cout << "Values message size: " << values_msg_size << " bytes" << std::endl;
  return 0;
}