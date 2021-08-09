# DVS filter bursts

Filter out bursts from a bag file

### Usage:
#### Single .bag file:

    rosrun dvs_filter_bursts filter_bursts path_to_input.bag --time_length_detection=<time_length_detection> --n_std=<n_std>
    
Saves output to ```path_to_input.bag.burstfiltered```.

