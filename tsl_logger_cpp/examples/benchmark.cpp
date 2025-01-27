// Copyright 2024 Simon Sagmeister

#include <chrono>
#include <iostream>

#include "tsl_logger_cpp/base.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
int main()
{
  constexpr std::size_t NUM_LOG_SIGNALS = 1000;
  constexpr std::size_t NUM_REPETITIONS = 10000;

  tam::tsl::ValueLogger logger;  // Create the logger

  // Create a an array of random float
  std::array<double, NUM_LOG_SIGNALS> arr;

  // Log the signals. This should be equivalent to logging NUM_SIGNALS individually signals
  std::chrono::nanoseconds duration{0};
  for (std::size_t i = 0; i < NUM_REPETITIONS; ++i) {
    // Update the array with random values so that the compiler cannot optimize
    for (std::size_t i = 0; i < NUM_LOG_SIGNALS; ++i) {
      arr[i] = static_cast<double>(rand() / RAND_MAX);  // NOLINT
    }
    auto start = std::chrono::high_resolution_clock::now();
    logger.log("my_benchmark_data", arr);
    auto end = std::chrono::high_resolution_clock::now();
    duration += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
  }

  std::cout << "Logging " << NUM_LOG_SIGNALS << " signals " << NUM_REPETITIONS << " times took "
            << duration.count() << " ns" << std::endl;
  // Print the average per logged signal
  std::cout << "Average time per signal: " << duration.count() / (NUM_LOG_SIGNALS * NUM_REPETITIONS)
            << " ns" << std::endl;

  return 0;
}