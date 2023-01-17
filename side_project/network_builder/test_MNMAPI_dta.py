# %%
import MNMAPI
from MNM_nb import MNM_network_builder

# %%
import os
import numpy as np
import pandas as pd

# %%
# specify the input folder
data_folder = '/home/qiling/Documents/MAC-POSTS/data/input_files_7link_fix'

# %%
# build the network from the input folder
# this will check if all the necessary files are included in data_folder
nb = MNM_network_builder()
nb.load_from_folder(data_folder)
print(nb)

# %%
a = MNMAPI.dta_api()

# %%
a.initialize(data_folder)

# %%
# store the links in order to output desired information in the following steps
link_array = np.array([e.ID for e in nb.link_list], dtype=int)
# link_array = np.array([4, 5])
a.register_links(link_array)

# %%
# store the paths in order to output desired information in the following steps
path_array = np.array(list(nb.path_table.ID2path.keys()), dtype=int)
a.register_paths(path_array)

# %%
a.install_cc()

# %%
a.install_cc_tree()

# %%
a.run_whole(False)


# %%
# assign_freq: the number of unit intervals between the two consecutive assignments
# assignment means assigning destinations and paths to vehicles 
# unit interval is usually set to 5s
# assign_freq is usually set to 180, leading to an assignment interval of 180*5/60 = 15 min 
assign_freq = nb.config.config_dict['DTA']['assign_frq']
# nb.config.config_dict['DTA']['max_interval']: maximum number of assignment intervals
# assign_intervals: [0, 180, 360, ...]
assign_intervals = np.arange(nb.config.config_dict['DTA']['max_interval']) * assign_freq
print("assign_intervals: ")
print(assign_intervals)
# after the simulation, a.get_cur_loading_interval() returns the last time interval of the simulation
end_interval = a.get_cur_loading_interval()
print("end interval: {}\n".format(end_interval))
# ensure the maximum assignment interval is less than the last simulation time interval
# ensure all the vehicles to be assigned are included in the simulation
assert(assign_intervals[-1] < end_interval)


# %%
# output: some statistics from the simulation
travel_stats = a.get_travel_stats()
print("\n************ travel stats ************")
print("vehicle count: {}".format(travel_stats[0]))
print("vehicle total travel time (hours): {}".format(travel_stats[1]))
print("************ travel stats ************\n")


# %%
# construct some time intervals for calculating link and path travel time
start_intervals = assign_intervals
assert(start_intervals[-1] < end_interval)
print(start_intervals)

# %%
# calculate time-dependent link travel time internally, it has no output but will be used in the following link and path travel time calculation
# so invoke this function first
a.build_link_cost_map(False)

# %%
# input: start_intervals is an arrays of time intervals
# output: time-dependent link travel time in second, dimension: number of links registered * number of time intervals in start_intervals
link_tt = a.get_link_tt(start_intervals, False)
assert(link_tt.shape[0] == len(link_array))
print(link_tt)  # seconds

# %%
# obtain a path from path table for calculating the path travel time and cost
# path_node_list: an array of node ID representing a path
# path_link_array: an array of link ID representing a path
path_node_list = nb.path_table.ID2path[path_array[0]].node_list
path_link_array = np.zeros(len(path_node_list)-1, dtype=int)
for i in range(len(path_node_list)-1):
    # get the corresponding link ID using two endpoint node IDs
    path_link_array[i] = nb.graph.G[path_node_list[i]][path_node_list[i+1]]['ID']


# %%
# input: path_link_array is an array of link IDs representing a path
# output: time-dependent path travel time and cost for car, dimension: 2 * number of time intervals
#         first row: travel time in seconds
#         second row: travel cost in monetary value = value of time * travel time + additional cost (e.g., link tolls)
path_tt_cost = a.get_path_tt(path_link_array, start_intervals)
print(path_tt_cost)  # seconds


# %%
# construct some time intervals for calculating vehicle inflows
start_intervals = assign_intervals
end_intervals = assign_intervals + assign_freq
assert(end_intervals[-1] <= end_interval)


# %%
# input: start_intervals and end_intervals are both arrays of time intervals
# output: link_inflow is time-dependent link inflows, dimension: number of links * number of time intervals
#         link_inflow[i, j] represents the flow entering link i within the time interval [start_intervals[j], end_intervals[j]]
link_inflow = a.get_link_inflow(start_intervals, end_intervals)
print(link_inflow)
assert(link_inflow.shape[0] == len(link_array))

# %%
