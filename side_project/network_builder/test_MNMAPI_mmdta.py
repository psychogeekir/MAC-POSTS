# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'


# %%
import os
import numpy as np
import pandas as pd

from MNM_mmnb import MNM_network_builder
import MNMAPI


# %%
# data_folder = os.path.join('.', 'MNM_cache', 'input_files_7link_multimodal_dode_generated')
# data_folder = '/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode_columngeneration/record/input_files_estimate_path_flow/'
data_folder = '/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode/record/input_files_estimate_path_flow/'
# data_folder = '/home/qiling/Documents/MAC-POSTS/c8b47612bcd2e63eff5ffe7433cc5c3bcc788dae/'


# %%
nb = MNM_network_builder()
nb.load_from_folder(data_folder)
nb.ID2path


# %%
# test destructor
def test_dta(data_folder):
    a = MNMAPI.mmdta_api()
    a.initialize(data_folder)
    print("test_dta")

test_dta(data_folder)


# %%
a = MNMAPI.mmdta_api()
a.initialize(data_folder)
a


# %%
link_driving_array = np.array([e.ID for e in nb.link_driving_list], dtype=int)
# link_driving_array = np.array([4, 5])
a.register_links_driving(link_driving_array)


# %%
link_bus_array = np.array([e.ID for e in nb.link_bus_list], dtype=int)
# link_bus_array = np.array([205, 206])
a.register_links_bus(link_bus_array)


# %%
link_walking_array = np.array([e.ID for e in nb.link_walking_list], dtype=int)
# link_walking_array = np.array([16, 17, 20])
a.register_links_walking(link_walking_array)


# %%
path_array = np.array(list(nb.ID2path.keys()), dtype=int)
path_array_driving = np.array([ID for ID in nb.path_table_driving.ID2path.keys()], dtype=int)
path_array_bustransit = np.array([ID for ID in nb.path_table_bustransit.ID2path.keys()], dtype=int)
path_array_pnr = np.array([ID for ID in nb.path_table_pnr.ID2path.keys()], dtype=int)
path_array_busroute = np.array([ID for ID in nb.path_table_bus.ID2path.keys()], dtype=int)
a.register_paths(path_array)
a.register_paths_driving(path_array_driving)
a.register_paths_bustransit(path_array_bustransit)
a.register_paths_pnr(path_array_pnr)
a.register_paths_bus(path_array_busroute)


# %%
a.install_cc()


# %%
a.install_cc_tree()


# %%
a.run_whole(False)


# %%
assign_freq = nb.config.config_dict['DTA']['assign_frq']
assign_intervals = np.arange(nb.config.config_dict['FIXED']['buffer_length']//2) * assign_freq
print("assign_intervals: ")
print(assign_intervals)
end_interval = a.get_cur_loading_interval()
print("end interval: {}\n".format(end_interval))
assert(assign_intervals[-1] < end_interval)


# %%
travel_stats = a.get_travel_stats()
print("\n************ travel stats ************\n")
print("car count: {}\n".format(travel_stats[0]))
print("PnR car count: {}\n".format(travel_stats[1]))
print("truck count: {}\n".format(travel_stats[2]))
print("bus count: {}\n".format(travel_stats[3]))
print("passenger count: {}\n".format(travel_stats[4]))
print("car total travel time (hours): {}\n".format(travel_stats[5]))
print("truck total travel time (hours): {}\n".format(travel_stats[6]))
print("bus total travel time (hours): {}\n".format(travel_stats[7]))
print("passenger total travel time (hours): {}\n".format(travel_stats[8]))
print("\n************ travel stats ************\n")

# %%
a.print_emission_stats()


# %%
start_intervals = assign_intervals
print(start_intervals)


# %%

car_link_tt = a.get_car_link_tt(start_intervals)
assert(car_link_tt.shape[0] == len(link_driving_array))
print(car_link_tt)  # seconds


# %%
truck_link_tt = a.get_truck_link_tt(start_intervals)
assert(truck_link_tt.shape[0] == len(link_driving_array))
print(truck_link_tt)  # seconds


# %%
bus_link_tt = a.get_bus_link_tt(start_intervals)
assert(bus_link_tt.shape[0] == len(link_bus_array))
print(bus_link_tt)  # seconds


# %%
walking_link_tt = a.get_passenger_walking_link_tt(start_intervals)
assert(walking_link_tt.shape[0] == len(link_walking_array))
print(walking_link_tt)  # seconds


# %%
a.build_link_cost_map(False)

# %%
truck_path_tt = a.get_registered_path_tt_truck(start_intervals)
print(truck_path_tt)  # seconds


# %%
driving_path_tt = a.get_registered_path_tt_driving(start_intervals)
print(driving_path_tt)  # seconds
driving_path_cost = a.get_registered_path_cost_driving(start_intervals)
print(driving_path_cost)


# %%
bustransit_path_tt = a.get_registered_path_tt_bustransit(start_intervals)
print(bustransit_path_tt)  # seconds
bustransit_path_cost = a.get_registered_path_cost_bustransit(start_intervals)
print(bustransit_path_cost)


# %%
pnr_path_tt = a.get_registered_path_tt_pnr(start_intervals)
print(pnr_path_tt)  # seconds
pnr_path_cost = a.get_registered_path_cost_pnr(start_intervals)
print(pnr_path_cost)


# %%
start_intervals = assign_intervals
end_intervals = assign_intervals + assign_freq
assert(end_intervals[-1] <= end_interval)


# %%
car_link_inflow = a.get_link_car_inflow(start_intervals, end_intervals)
print(car_link_inflow)
assert(car_link_inflow.shape[0] == len(link_driving_array))


# %%
truck_link_inflow = a.get_link_truck_inflow(start_intervals, end_intervals)
print(truck_link_inflow)
assert(truck_link_inflow.shape[0] == len(link_driving_array))


# %%
bus_link_inflow = a.get_link_bus_inflow(start_intervals, end_intervals)
print(bus_link_inflow)
assert(bus_link_inflow.shape[0] == len(link_bus_array))


# %%
bus_busstop_inflow = a.get_busstop_bus_inflow(start_intervals, end_intervals)
print(bus_busstop_inflow)
assert(bus_busstop_inflow.shape[0] == len(link_bus_array))


# %%
bus_passenger_link_inflow = a.get_link_bus_passenger_inflow(start_intervals, end_intervals)
print(bus_passenger_link_inflow)
assert(bus_passenger_link_inflow.shape[0] == len(link_bus_array))


# %%
walking_passenger_link_inflow = a.get_link_walking_passenger_inflow(start_intervals, end_intervals)
print(walking_passenger_link_inflow)
assert(walking_passenger_link_inflow.shape[0] == len(link_walking_array))


# %%
def get_dar_df(dar_matrix):
    assert(dar_matrix.shape[1] == 5)
    print("path ID, departure assign interval (1 min), link ID, time interval (5 s), flow")
    dar_df = pd.DataFrame(
                    data = dar_matrix, 
                    columns = ["path_ID", "depart_assign_interval", "link_ID", "time_interval", "flow"],
                    dtype = float
                )
    dar_df["path_ID"] = dar_df["path_ID"].astype(int)
    dar_df["depart_assign_interval"] = dar_df["depart_assign_interval"].astype(int)
    dar_df["link_ID"] = dar_df["link_ID"].astype(int)
    dar_df["time_interval"] = dar_df["time_interval"].astype(int)
    dar_df["flow"] = dar_df["flow"].astype(float)
    return dar_df


# %%
car_dar_driving_matrix = a.get_car_dar_matrix_driving(start_intervals, end_intervals)
car_dar_driving_df = get_dar_df(car_dar_driving_matrix)
print(car_dar_driving_df)
print(car_dar_driving_df.shape)
print([pathID for pathID in nb.path_table_driving.ID2path.keys()])
print(np.unique(car_dar_driving_df['path_ID']))


# %%
truck_dar_driving_matrix = a.get_truck_dar_matrix_driving(start_intervals, end_intervals)
truck_dar_driving_df = get_dar_df(truck_dar_driving_matrix)
print(truck_dar_driving_df)
print(truck_dar_driving_df.shape)
print(len(nb.path_table_driving.ID2path) * len(link_driving_array) * end_interval)
print([pathID for pathID in nb.path_table_driving.ID2path.keys()])
print(np.unique(truck_dar_driving_df['path_ID']))


# %%
car_dar_pnr_matrix = a.get_car_dar_matrix_pnr(start_intervals, end_intervals)
car_dar_pnr_df = get_dar_df(car_dar_pnr_matrix)
print(car_dar_pnr_df)
print(car_dar_pnr_df.shape)
print([pathID for pathID in nb.path_table_pnr.ID2path.keys()])
print(np.unique(car_dar_pnr_df['path_ID']))


# %%
bus_dar_matrix = a.get_bus_dar_matrix_bustransit_link(start_intervals, end_intervals)
bus_dar_df = get_dar_df(bus_dar_matrix)
print(bus_dar_df)
print(bus_dar_df.shape)
print([pathID for pathID in nb.path_table_bus.ID2path.keys()])
print(np.unique(bus_dar_df['path_ID']))


# %%
passenger_dar_bustransit_matrix = a.get_passenger_dar_matrix_bustransit(start_intervals, end_intervals)
passenger_dar_bustransit_df = get_dar_df(passenger_dar_bustransit_matrix)
print(passenger_dar_bustransit_df)
print(passenger_dar_bustransit_df.shape)
print([pathID for pathID in nb.path_table_bustransit.ID2path.keys()])
print(np.unique(passenger_dar_bustransit_df['path_ID']))


# %%
passenger_dar_pnr_matrix = a.get_passenger_dar_matrix_pnr(start_intervals, end_intervals)
passenger_dar_pnr_df = get_dar_df(passenger_dar_pnr_matrix)
print(passenger_dar_pnr_df)
print(passenger_dar_pnr_df.shape)
print([pathID for pathID in nb.path_table_pnr.ID2path.keys()])
print(np.unique(passenger_dar_pnr_df['path_ID']))

