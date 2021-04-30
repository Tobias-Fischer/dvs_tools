#include "../include/dvs_filter_bursts/utils.h"

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <gflags/gflags.h>
#include <iostream>
#include <numeric>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#define foreach BOOST_FOREACH

namespace dvs_filter_bursts {
namespace utils {
//const std::string OUTPUT_FOLDER = "./stats/";

bool parse_arguments(int argc, char* argv[],
                     std::string* path_to_input_rosbag
                     )
{
  constexpr int expected_num_arguments = 2;
  if(argc < expected_num_arguments)
  {
    std::cerr << "Not enough arguments, "<< argc << " given, " << expected_num_arguments << " expected." << std::endl;
    std::cerr << "Usage: rosrun dvs_filter_bursts dvs_filter_bursts path_to_bag.bag" << std::endl;
    return false;
  }

  *path_to_input_rosbag = std::string(argv[1]);

  return true;
}

bool load_events_from_bag(std::string path_to_input_rosbag,
                          topic_eventArray& events_by_topic)
{
  rosbag::Bag input_bag;
  try
  {
   input_bag.open(path_to_input_rosbag, rosbag::bagmode::Read);
  }
  catch(rosbag::BagIOException e)
  {
   std::cerr << "Error: could not open rosbag: " << path_to_input_rosbag << std::endl;
   return false;
  }

  rosbag::View view(input_bag);

  load_events(view, events_by_topic);

  input_bag.close();

  return true;
}

void load_events(rosbag::View& view, topic_eventArray& events_by_topic)
{
  std::cout << "loading events..." << std::endl;

  std::vector<std::string> seen_topics;
  for(const rosbag::MessageInstance& m : view)
  {
    if(m.getDataType() == "dvs_msgs/EventArray")
    {
      const std::string topic_name = m.getTopic();
      // pointer to the message
      dvs_msgs::EventArrayConstPtr s = m.instantiate<dvs_msgs::EventArray>();

      dvs_msgs::EventArray& event_array = events_by_topic[topic_name];

      // add to seen topic
      if ( !contains(topic_name, seen_topics) )
      {
        seen_topics.push_back(topic_name);
        event_array.width = s->width;
        event_array.height = s->height;
        std::cout << "added " << topic_name << " to seen_topics" << std::endl;
      }

      for(auto e : s->events)
      {
        // accumulate events without discrimination
        event_array.events.push_back(e);
      }
    }
  }

  std::cout << "...done!" << std::endl;
}

void sort_events(topic_eventArray& events_by_topic)
{
  for (auto& iter: events_by_topic)
  {
    auto& event_array = iter.second;
    std::sort(event_array.events.begin(), event_array.events.end(), is_timestamp_in_order);
  }
}

bool is_timestamp_in_order(dvs_msgs::Event first, dvs_msgs::Event second)
{
  return first.ts.toSec() < second.ts.toSec();
}

int closest(dvs_msgs::EventArray const& vec, double value) {
    dvs_msgs::Event ros_event;
    ros::Time ros_val(value);

    ros_event.ts = ros_val;
    auto const it = std::lower_bound(vec.events.begin(), vec.events.end(), ros_event, is_timestamp_in_order);
    return it-vec.events.begin();
}

std::vector<bool> filter_bursts(topic_eventArray& events_by_topic, double n_std, double time_length_detection)
{
  for (auto& iter: events_by_topic)
  {
    dvs_msgs::EventArray& event_array = iter.second;
    std::vector<bool> use_events(event_array.events.size(), true);

    unsigned int events_removed = 0;

    int first_idx = closest(event_array, 20);
    for(int idx=0; idx<first_idx; idx++) {
      use_events[idx] = false;
      events_removed++;
    }
    std::cout << "First idx: " << std::fixed << first_idx << std::endl;
    std::cout << "Removed " << std::fixed << events_removed << " events from beginning of sequence" << std::endl;

    double first_ts = event_array.events[first_idx].ts.toSec();
    double last_ts = event_array.events[event_array.events.size()-1].ts.toSec();
    std::cout << "First ts: " << std::fixed << first_ts << std::endl;;
    std::cout << "Last ts:  " << std::fixed << last_ts << std::endl;;

    std::vector<int> start_indices;
    std::vector<int> end_indices;
    std::vector<double> num_events_in_window;

    for(double ts=first_ts; ts<=last_ts; ts = ts + time_length_detection) {
        int start_idx = closest(event_array, ts);
        int end_idx = closest(event_array, ts + time_length_detection);
        num_events_in_window.push_back(end_idx-start_idx);
        start_indices.push_back(start_idx);
        end_indices.push_back(end_idx);
    }
    
    double sum = std::accumulate(num_events_in_window.begin(), num_events_in_window.end(), 0.0);
    double mean = sum / num_events_in_window.size();

    double sq_sum = std::inner_product(num_events_in_window.begin(), num_events_in_window.end(), num_events_in_window.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / num_events_in_window.size() - mean * mean);

    double max_num = mean + n_std * stdev;

    std::vector<bool> use_events_window(num_events_in_window.size(), true);
    for(size_t num_events_idx=0; num_events_idx<num_events_in_window.size(); num_events_idx++) {
      if(num_events_in_window[num_events_idx] > max_num) {
        use_events_window[num_events_idx] = false;
      }
    }

    for(size_t num_events_idx=0; num_events_idx<use_events_window.size(); num_events_idx++) {
      if(use_events_window[num_events_idx] == 0) {
        for(int idx=start_indices[num_events_idx]; idx<end_indices[num_events_idx]; idx++) {
          use_events[idx] = false;
          events_removed++;
        }
      }
    }
    std::cout << "Removed " << events_removed << " out of " << event_array.events.size() << " events" << std::fixed << std::setprecision(2) <<
                 " (" << double(events_removed)/double(event_array.events.size())*100.0 << "%)" << std::endl;

    return use_events;
  }
}

void write_events_to_bag(topic_eventArray& events_by_topic,
                         const int& max_num_events_per_packet,
                         const double& max_duration_event_packet,
                         std::string path_to_output_rosbag,
                         std::vector<bool> use_events)
{
  rosbag::Bag output_bag;
  output_bag.open(path_to_output_rosbag, rosbag::bagmode::Write);

  for (auto& iter: events_by_topic)
  {
    std::string topic = iter.first;
    dvs_msgs::EventArray& event_array_in = iter.second;
    std::vector<dvs_msgs::Event> events_out;
    unsigned int idx = 0;
    for(auto e : event_array_in.events)
    {
      if(use_events.at(idx)) {
        events_out.push_back(e);
        idx = idx + 1;
      }
      else {
        idx = idx + 1;
        continue;
      }
      if(events_out.size() == max_num_events_per_packet
         || (events_out.back().ts.toSec() - events_out.front().ts.toSec()) >= max_duration_event_packet)
      {
        // Write new event array message to output rosbag
        dvs_msgs::EventArray event_array_msg;
        event_array_msg.events = events_out;
        event_array_msg.width = event_array_in.width;
        event_array_msg.height = event_array_in.height;
        event_array_msg.header.stamp = events_out.back().ts;

        output_bag.write(topic, event_array_msg.header.stamp, event_array_msg);

        // Start new packet of events
        events_out.clear();
      }
    }
  }
  output_bag.close();
}

}  // namespace utils
}  // namespace dvs_filter_bursts
