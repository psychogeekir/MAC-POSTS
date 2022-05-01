# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'


# %%
import os
import numpy as np
import pandas as pd

from MNM_mcnb import MNM_network_builder
import MNMAPI


# %%
data_folder = '/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multiclass_new/'


# %%
nb = MNM_network_builder()
nb.load_from_folder(data_folder)
print(nb)


# %%
# test destructor
# def test_dta(data_folder):
#     a = MNMAPI.mcdta_api()
#     a.initialize(data_folder)
#     print("test_dta")

# test_dta(data_folder)


# %%
a = MNMAPI.mcdta_api()
a.initialize(data_folder)
a


# %%
link_array = np.array([e.ID for e in nb.link_list], dtype=int)
# link_array = np.array([4, 5])
a.register_links(link_array)


# %%
path_array = np.array(list(nb.path_table.ID2path.keys()), dtype=int)
a.register_paths(path_array)


# %%
a.install_cc()


# %%
a.install_cc_tree()


# %%
a.run_whole(False)

r = a.get_waiting_time_at_intersections()
print(r)
r = a.get_waiting_time_at_intersections_car()
print(r)
r = a.get_waiting_time_at_intersections_truck()
print(r)


# %%
assign_freq = nb.config.config_dict['DTA']['assign_frq']
assign_intervals = np.arange(nb.config.config_dict['DTA']['max_interval']) * assign_freq
print("assign_intervals: ")
print(assign_intervals)
end_interval = a.get_cur_loading_interval()
print("end interval: {}\n".format(end_interval))
assert(assign_intervals[-1] < end_interval)


# %%
travel_stats = a.get_travel_stats()
print("\n************ travel stats ************")
print("car count: {}".format(travel_stats[0]))
print("truck count: {}".format(travel_stats[1]))
print("car total travel time (hours): {}".format(travel_stats[2]))
print("truck total travel time (hours): {}".format(travel_stats[3]))
print("************ travel stats ************\n")


# %%
start_intervals = assign_intervals
print(start_intervals)


# %%

car_link_tt = a.get_car_link_tt(start_intervals)
assert(car_link_tt.shape[0] == len(link_array))
print(car_link_tt)  # seconds


# %%
truck_link_tt = a.get_truck_link_tt(start_intervals)
assert(truck_link_tt.shape[0] == len(link_array))
print(truck_link_tt)  # seconds


# %%
path_node_list = nb.path_table.ID2path[path_array[0]].node_list
path_link_array = np.zeros(len(path_node_list)-1, dtype=int)
for i in range(len(path_node_list)-1):
    path_link_array[i] = nb.graph.G[path_node_list[i]][path_node_list[i+1]]['ID']

car_path_tt = a.get_path_tt_car(path_link_array, start_intervals)
print(car_path_tt)  # seconds


# %%
truck_path_tt = a.get_path_tt_truck(path_link_array, start_intervals)
print(truck_path_tt)  # seconds


# %%
start_intervals = assign_intervals
end_intervals = assign_intervals + assign_freq
assert(end_intervals[-1] <= end_interval)


# %%
car_link_inflow = a.get_link_car_inflow(start_intervals, end_intervals)
print(car_link_inflow)
assert(car_link_inflow.shape[0] == len(link_array))


# %%
truck_link_inflow = a.get_link_truck_inflow(start_intervals, end_intervals)
print(truck_link_inflow)
assert(truck_link_inflow.shape[0] == len(link_array))


