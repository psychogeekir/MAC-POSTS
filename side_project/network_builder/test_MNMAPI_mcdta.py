# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'


# %%
import os
import numpy as np
import pandas as pd

from MNM_mcnb import MNM_network_builder
import MNMAPI


# %%
# specify the input folder
data_folder = '/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multiclass_new/'


# %%
# build the network from the input folder
# this will check if all the necessary files are included in data_folder
nb = MNM_network_builder()
nb.load_from_folder(data_folder)
print(nb)

# %% 
# # if no path exists
# a = MNMAPI.mcdta_api()
# # manually set num_path = -1 under [FIXED] in config.conf
# a.initialize(nb.folder_path)  # generate at least one path for each mode for each OD pair, if connected
# max_iter = 2 # number of iterations to generate paths, if max_iter = 0, only one shortest path will be generate
# a.generate_shortest_pathsets(nb.folder_path, 0, nb.config.config_dict['ADAPTIVE']['vot'], 3, 6, 0)
# # after this step, check folder_path, path_table and path_table_buffer should be created
# # then manually set num_path = number of rows in path_table under [FIXED] in config.conf for the following simulation

# # re-read the folder to include the generated paths
# nb = MNM_network_builder()
# nb.load_from_folder(data_folder)
# print(nb)

# %%
# test destructor
# def test_dta(data_folder):
#     a = MNMAPI.mcdta_api()
#     a.initialize(data_folder)
#     print("test_dta")

# test_dta(data_folder)


# %%
# initialize the mcdta for the simulation
# all the data required is stored in data_folder
# simulation parameters are stored in data_folder/config.conf
a = MNMAPI.mcdta_api()
a.initialize(data_folder)
a


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
# install the cumulative curves for the links, important for calculating link travel time
a.install_cc()


# %%
# install the tree-based cumulative curves for the links, important for dynamic origin-destination estimation demand
# it can be omitted for simulation, which will save significant memory and time for large-scale networks
a.install_cc_tree()


# %%
# run the simulation, input: whether to show the detailed simulation steps
a.run_whole(False)

# %%
# output: average waiting time at the end of each link in the whole simulation horizon (in second), dimension: number of links
r = a.get_waiting_time_at_intersections()
print(r)
# output: average waiting time for car at the end of each link in the whole simulation horizon (in second), dimension: number of links
r = a.get_waiting_time_at_intersections_car()
print(r)
# output: average waiting time for truck at the end of each link in the whole simulation horizon (in second), dimension: number of links
r = a.get_waiting_time_at_intersections_truck()
print(r)


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
print("car count: {}".format(travel_stats[0]))
print("truck count: {}".format(travel_stats[1]))
print("car total travel time (hours): {}".format(travel_stats[2]))
print("truck total travel time (hours): {}".format(travel_stats[3]))
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
# output: time-dependent link travel time for car in second, dimension: number of links registered * number of time intervals in start_intervals
car_link_tt = a.get_car_link_tt(start_intervals, False)
assert(car_link_tt.shape[0] == len(link_array))
print(car_link_tt)  # seconds


# %%
# input: start_intervals is an arrays of time intervals
# output: time-dependent link travel time for truck in second, dimension: number of links registered * number of time intervals in start_intervals
truck_link_tt = a.get_truck_link_tt(start_intervals, False)
assert(truck_link_tt.shape[0] == len(link_array))
print(truck_link_tt)  # seconds


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
car_path_tt_cost = a.get_path_tt_car(path_link_array, start_intervals)
print(car_path_tt_cost)  # seconds


# %%
# input: path_link_array is an array of link IDs representing a path
# output: time-dependent path travel time and cost for car, dimension: 2 * number of time intervals
#         first row: travel time in seconds
#         second row: travel cost in monetary value = value of time * travel time + additional cost (e.g., link tolls)
truck_path_tt_cost = a.get_path_tt_truck(path_link_array, start_intervals)
print(truck_path_tt_cost)  # seconds


# %%
# construct some time intervals for calculating vehicle inflows
start_intervals = assign_intervals
end_intervals = assign_intervals + assign_freq
assert(end_intervals[-1] <= end_interval)


# %%
# input: start_intervals and end_intervals are both arrays of time intervals
# output: car_link_inflow is time-dependent link car inflows, dimension: number of links * number of time intervals
#         car_link_inflow[i, j] represents the car flow entering link i within the time interval [start_intervals[j], end_intervals[j]]
car_link_inflow = a.get_link_car_inflow(start_intervals, end_intervals)
print(car_link_inflow)
assert(car_link_inflow.shape[0] == len(link_array))


# %%
# input: start_intervals and end_intervals are both arrays of time intervals
# output: truck_link_inflow is time-dependent truck inflows, dimension: number of links * number of time intervals
#         truck_link_inflow[i, j] represents the truck flow entering link i within the time interval [start_intervals[j], end_intervals[j]]
truck_link_inflow = a.get_link_truck_inflow(start_intervals, end_intervals)
print(truck_link_inflow)
assert(truck_link_inflow.shape[0] == len(link_array))



# %%
