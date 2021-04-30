#pragma once

#include <unordered_map>
#include <vector>

#include <dvs_msgs/EventArray.h>

namespace rosbag
{
class Bag;
class MessageInstance;
class View;
}

namespace dvs_filter_bursts
{
namespace utils
{

typedef std::unordered_map<std::string, dvs_msgs::EventArray> topic_eventArray;

template <typename T>
bool contains(T element, std::vector<T> my_vector)
{
  // returns true if my_vector contains element, otherwise false.
  for (auto topic_name : my_vector)
  {
    if (element == topic_name)
    {
      return true;
    }
  }
  return false;
}

bool parse_arguments(int argc, char* argv[],
                     std::string* path_to_input_rosbag);

bool load_events_from_bag(std::string path_to_input_rosbag,
                          topic_eventArray& events);

void load_events(rosbag::View& view, topic_eventArray& events_by_topic);

void sort_events(topic_eventArray& events_by_topic);

bool is_timestamp_in_order(dvs_msgs::Event first, dvs_msgs::Event second);

std::vector<bool> filter_bursts(topic_eventArray& events_by_topic, double n_std, double time_length_detection);

void write_events_to_bag(topic_eventArray& events_by_topic,
                         const int& max_num_events_per_packet,
                         const double& max_duration_event_packet,
                         std::string path_to_output_rosbag, std::vector<bool> use_events);

}  // namespace utils
}  // namespace dvs_filter_bursts
