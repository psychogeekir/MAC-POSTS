# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'

# %%
import numpy as np
import pandas as pd
import sys
import datetime
import os
import matplotlib.pyplot as plt
import matplotlib
import networkx as nx
import seaborn as sns


# %%
from matplotlib import colors
import six
import matplotlib.dates as mdates
import datetime
import pandas as pd
import seaborn as sns
sns.set()
plt.style.use('seaborn-poster')


# %%
# MNM_nb_folder = os.path.join('..', '..', '..', 'side_project', 'network_builder') # MNM_nb, MNM_mcnb
MNM_nb_folder = os.path.join('.', 'side_project', 'network_builder') # MNM_nb, MNM_mcnb
sys.path.append(MNM_nb_folder)
# python_lib_folder = os.path.join('..', '..', 'pylib') # covariance_tree and DODE, sDODE, mcDODE functions
python_lib_folder = os.path.join('.', 'src', 'pylib') # covariance_tree and DODE, sDODE, mcDODE functions
sys.path.append(python_lib_folder)


# %%
from MNM_mcnb import MNM_network_builder
from mcDODE import MCDODE  


# %%
# /home/qiling/Documents/MAC-POSTS
print(os.getcwd())
data_folder = os.path.join('/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multiclass_new')


# %% read initial input files
nb = MNM_network_builder()  # from MNM_mcnb, for python analysis
nb.load_from_folder(data_folder)
print(nb)


# %%
# prepare artifical observed data
observed_link_list = [3, 4, 5, 6]
num_interval = nb.config.config_dict['DTA']['max_interval']

# true time-dependen path flow
true_car_f = np.random.rand(num_interval * nb.config.config_dict['FIXED']['num_path']) * 300
true_truck_f = np.random.rand(num_interval * nb.config.config_dict['FIXED']['num_path']) * 30

# %%
config = dict()
config['use_car_link_flow'] = True
config['use_truck_link_flow'] = True
config['use_car_link_tt'] = True
config['use_truck_link_tt'] = True
config['car_count_agg'] = True
config['truck_count_agg'] = True
config['link_car_flow_weight'] = 1
config['link_truck_flow_weight'] = 1
config['link_car_tt_weight'] = 0.
config['link_truck_tt_weight'] = 0.
config['num_data'] = 1  
config['observed_links'] = observed_link_list
config['paths_list'] = range(nb.config.config_dict['FIXED']['num_path'])


config['compute_car_link_flow_loss'] = True
config['compute_truck_link_flow_loss'] = True
config['compute_car_link_tt_loss'] = True
config['compute_truck_link_tt_loss'] = True

dode = MCDODE(nb, config)

# %% modify link attributes, probably only for CTM links

def check_link_attributes(link):
    # link.ID
    # # length (unit: mile)
    # link.length  # mile
    # # link type: CTM, LQ, PQ
    # link.typ      # type
    # # free flow speed for car (unit: mile/hour)
    # link.ffs_car      # mile/h
    # # capacity for car (unit: vehicles/hour/lane)
    # link.cap_car       # v/hour/lane
    # # jam density for car (unit: vehicles/mile/lane)
    # link.rhoj_car      # v/mile/lane
    # # number of lanes
    # link.lanes   # num of lanes
    # # free flow speed for truck (unit: mile/hour)
    # link.ffs_truck       # mile/h
    # # capacity for truck (unit: vehicles/hour/lane)
    # link.cap_truck      # v/hour/lane
    # # jam density for truck (unit: vehicles/mile/lane)
    # link.rhoj_truck      # v/mile/lane
    # # factor for converting truck flow to equivalent car flow, only for calculating node demand for Inout, FWJ, and GRJ node types
    # link.convert_factor

    # mile -> meter, hour -> second
    _length = link.length * 1600  # m
    _ffs_car = link.ffs_car * 1600 / 3600  # m/s
    _lane_flow_cap_car = link.cap_car / 3600  # vehicles/s/lane
    _lane_hold_cap_car = link.rhoj_car / 1600  # vehicles/m/lane
    _ffs_truck = link.ffs_truck * 1600 / 3600  # m/s
    _lane_flow_cap_truck = link.cap_truck / 3600  # vehicles/s/lane
    _lane_hold_cap_truck = link.rhoj_truck / 1600  # vehicles/m/lane

    # Jam density for private cars and trucks cannot be negative
    if ((_lane_hold_cap_car < 0) or (_lane_hold_cap_truck < 0)):
        raise("lane_hold_cap can't be negative, current link ID is {:d}\n".format(link.ID))
	
	# Maximum flux for private cars and trucks cannot be negative
    if ((_lane_flow_cap_car < 0) or (_lane_flow_cap_truck < 0)):
        raise("lane_flow_cap can't be less than zero, current link ID is {:d}\n".format(link.ID))

    if ((_ffs_car < 0) or (_ffs_truck < 0)):
        raise("free-flow speed can't be less than zero, current link ID is {:d}\n".format(link.ID))
	
    if (link.convert_factor < 1):
        raise("veh_convert_factor can't be less than 1, current link ID is {:d}\n".format(link.ID))


    _lane_critical_density_car = _lane_flow_cap_car / _ffs_car
    _lane_critical_density_truck = _lane_flow_cap_truck / _ffs_truck

    if (_lane_hold_cap_car <= _lane_critical_density_car):
        raise("Wrong private car parameters, current link ID is {:d}\n".format(link.ID))

    if (_lane_hold_cap_truck <= _lane_critical_density_truck):
        raise("Wrong truck parameters, current link ID is {:d}\n".format(link.ID))
		
for link in dode.nb.link_list:
    if link.typ == 'CTM':
        link.ffs_car = link.ffs_car * 0.9
        link.ffs_truck = link.ffs_truck * 0.9
        check_link_attributes(link)

# %% run DNL
dta = dode._run_simulation(true_car_f, true_truck_f)
travel_stats = dta.get_travel_stats()

print("\n************ travel stats ************")
print("car count: {}".format(travel_stats[0]))
print("truck count: {}".format(travel_stats[1]))
print("car total travel time (hours): {}".format(travel_stats[2]))
print("truck total travel time (hours): {}".format(travel_stats[3]))
print("************ travel stats ************\n")



# %%
# nb.update_demand_path2(true_car_f, true_truck_f)
# nb.dump_to_folder("one")
# nb.update_demand_path2(car_flow, truck_flow)
# nb.dump_to_folder("two")


# %%
# pickle.dump((car_flow, truck_flow, None), open('test.pickle', 'w'))


# %%



