// Copyright 2024 Simon Sagmeister

#include "tsl_ros2_publisher_cpp/msg_creator.hpp"

#include <iostream>
namespace tam::tsl
{
rclcpp::SerializedMessage const & MsgCreator::get_definition() const { return def_msg_serialized_; }
rclcpp::SerializedMessage const & MsgCreator::get_values() const { return values_msg_serialized_; }
void MsgCreator::process_data(data_map_t const & data)
{
  if (init_size == 0) {
    setup_definition_msg(data);
    setup_values_msg();
    init_size = data.size();
  }
  update_values_msg(data);

  // True if values were added after the first publishment
  if (data.size() != init_size) {
    print_warning_added_signals(data);
  }
}
void MsgCreator::print_warning_added_signals(data_map_t const & data)
{
  // Accumulate all signals in the def_msg into a set
  std::unordered_set<std::string> def_set;
  std::unordered_set<std::string> data_set;
  // clang-format off
      #define ACCUMULATE_SIGNALS(dtype, typename)  \
        for (auto const & name : def_msg_.typename##_names) {  \
          def_set.insert(name);  \
        }
      TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(ACCUMULATE_SIGNALS)
      #undef ACCUMULATE_SIGNALS
  // clang-format on
  for (auto const & map_entry : data) {
    data_set.insert(map_entry.first);
  }

  // Do not print a warning if the number of non-logged signals did not change
  if (data_set.size() < def_set.size()) return;  // Ensure no underflow
  if (data_set.size() - def_set.size() == last_num_non_logged_signals_) return;
  last_num_non_logged_signals_ = data_set.size() - def_set.size();
  // Compare the 2 sets
  std::unordered_set<std::string> diff_set;
  for (auto const & name : data_set) {
    if (def_set.find(name) == def_set.end()) diff_set.insert(name);
  }

  // Return without updating the values
  std::cout << "TSLPublisher | Additional signals were logged after first publishing your "
               "time: ";
  for (auto const & new_signal : diff_set) {
    std::cout << new_signal << ", ";
  }
  std::cout << " | This is currently not supported and these signals will not be logged."
            << std::endl;
}
void MsgCreator::setup_definition_msg(data_map_t const & data)
{
  // Reset the old definition
  def_msg_ = tsl_msgs::msg::TSLDefinition();

  // clang-format off
    // Query the compression strategy to check as which type
    // each signal should be logged.
    #define CLASSIFY_SIGNAL_BY_TYPE(dtype, typename)                  \
      if (compression_strategy_->log_as_##typename(map_entry.first, map_entry.second)) { \
        def_msg_.typename##_names.push_back(map_entry.first);        \
        continue; \
      }
    // Iterate over all members of the data map
    for (auto const & map_entry : data) {
      // For every member, check which type it should be logged as
      // and append it to the corresponding names vector
      TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(CLASSIFY_SIGNAL_BY_TYPE)
    }
    #undef CLASSIFY_SIGNAL_BY_TYPE
  // clang-format on

  // Hash of the definition only depends on the signals names, not on the stamp
  // or if the has is already set within the msg.
  def_msg_.definition_hash = std::hash<tsl_msgs::msg::TSLDefinition>{}(def_msg_);

  // Serialize message
  rclcpp::Serialization<tsl_msgs::msg::TSLDefinition>().serialize_message(
    &def_msg_, &def_msg_serialized_);
}
void MsgCreator::setup_values_msg()
{
  // Clean the old data
  values_msg_ = tsl_msgs::msg::TSLValues();

  // clang-format off
      #define RESIZE_VALUES_VECTOR(dtype, typename)  \
        values_msg_.typename##_values.resize(def_msg_.typename##_names.size());
      TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(RESIZE_VALUES_VECTOR)
      #undef RESIZE_VALUES_VECTOR
  // clang-format on

  // Add the correct hash
  values_msg_.definition_hash = def_msg_.definition_hash;
}
void MsgCreator::update_values_msg(data_map_t const & data)
{
  // clang-format off
    #define UPDATE_VALUES_BY_TYPE(dtype, typename)  \
      for (std::size_t i = 0; i < def_msg_.typename##_names.size(); ++i) {  \
        values_msg_.typename##_values[i] = \
        helpers::numeric_variant_cast<dtype>(data.at(def_msg_.typename##_names[i]));  \
      }

    TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(UPDATE_VALUES_BY_TYPE)

    #undef UPDATE_VALUES_BY_TYPE
  // clang-format on

  // Serialize message
  values_serializer_.serialize_message(&values_msg_, &values_msg_serialized_);
}
}  // namespace tam::tsl
