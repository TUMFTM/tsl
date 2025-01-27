// Copyright 2024 Simon Sagmeister
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "tsl_logger_cpp/base.hpp"
#include "tsl_logger_cpp/types.hpp"
namespace tam::tsl
{
/// @brief This composing logger allows to access multiple loggers at as they would be a single one
/// @brief This can be used to combine the loggers of multiple modules, that each have their own
/// logger.
/// @brief Additionally, this allows to combine a reference logger with a value logger to access the
/// both simulatneously.
class LoggerComposer : public LoggerAccessInterface
{
public:
  typedef std::shared_ptr<LoggerComposer> SharedPtr;
  typedef std::unique_ptr<LoggerComposer> UniquePtr;

private:
  std::vector<std::pair<std::string, std::shared_ptr<LoggerAccessInterface>>> managed_loggers_{};
  mutable data_map_t data_copy_{};

public:
  /// @brief Construct a composing logger from a vector of pairs. The first element of the pair is
  /// the
  /// @brief prefix that is added to all signal names of the corresponding logger. The second
  /// @brief element is a shared ptr to the logger.
  /// @param loggers: [(prefix, logger:SharedPtr)]
  explicit LoggerComposer(
    std::vector<std::pair<std::string, std::shared_ptr<LoggerAccessInterface>>> loggers)
  : managed_loggers_{loggers}
  {
  }
  /// @brief Construct a composing logger from a vector of loggers. No prefix will be added
  /// @brief to the signal names of the composed loggers.
  /// @param loggers
  explicit LoggerComposer(std::vector<std::shared_ptr<LoggerAccessInterface>> loggers)
  {
    for (auto const & logger : loggers) {
      managed_loggers_.push_back({"", logger});
    }
  }
  /// @brief Default-construct an empty composing logger
  LoggerComposer() {}
  /// @brief Register a logger to the composing logger after its construction.
  /// @param logger: Shared ptr to the logger that should be added to the composing logger.
  /// @param prefix: A prefix that will be applied to all signal names of the given logger.
  void register_logger(
    std::shared_ptr<LoggerAccessInterface> logger, std::string const & prefix = "")
  {
    managed_loggers_.push_back({prefix, logger});
  }
  void get_data(data_map_t & map, std::string const & prefix = "") const override
  {
    // Copy over all elements
    for (std::size_t i = 0; i < managed_loggers_.size(); i++) {
      managed_loggers_[i].second->get_data(map, prefix + managed_loggers_[i].first);
    }
  }
  data_map_t const & get_data() const override
  {
    get_data(data_copy_);
    return data_copy_;
  }
};
}  // namespace tam::tsl
