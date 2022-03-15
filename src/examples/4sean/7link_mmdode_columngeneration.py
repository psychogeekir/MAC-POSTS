# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'

# %%
import scipy
from scipy import stats
from scipy.sparse import csr_matrix
from scipy import io

from sklearn.metrics import r2_score
import numpy as np
import pandas as pd
import sys
import datetime
import os
sys.settrace

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import colors
import matplotlib.dates as mdates

import networkx as nx
import pickle
from collections import OrderedDict
import copy


import joblib
from joblib import Parallel, delayed
import random
import six

import seaborn as sns
sns.set()
plt.style.use('seaborn-poster')


# %%
# MNM_nb_folder = os.path.join('..', '..', '..', 'side_project', 'network_builder') # MNM_nb, MNM_mcnb, MNM_mmnb
MNM_nb_folder = os.path.join('.', 'side_project', 'network_builder')  # MNM_nb, MNM_mcnb, MNM_mmnb
sys.path.append(MNM_nb_folder)

# python_lib_folder = os.path.join('..', '..', 'pylib') # covariance_tree and DODE, sDODE, mcDODE, mmDODE functions
python_lib_folder = os.path.join('.', 'src', 'pylib') # covariance_tree and DODE, sDODE, mcDODE, mmDODE functions
sys.path.append(python_lib_folder)

import MNMAPI   # main DTA package
from MNM_mmnb import MNM_network_builder
from mmDODE import MMDODE


# %%
# /home/qiling/Documents/MAC-POSTS
data_folder_observed = os.path.join('/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode')
# data_folder_observed = os.path.join('/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode_columngeneration/record/input_files_estimate_demand')
# data_folder_observed = os.path.join('/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode_columngeneration/record/input_files_estimate_path_flow')
data_folder = data_folder_observed  # os.path.join('/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode_columngeneration')

# %%
nb = MNM_network_builder()  # from MNM_mcnb, for python analysis
nb.load_from_folder(data_folder_observed)
print(nb)

# %%

def r2(predictions, targets):
    y_bar = np.mean(targets)
    # diff = np.minimum(np.abs(predictions - targets), targets)
    diff = predictions - targets
    ss_e = np.sum(diff ** 2)
    ss_t = np.sum((targets) ** 2)
    return 1 - ss_e / ss_t


def rsquared(x, y):
    slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)
    return r_value**2


def rmse(predictions, targets):
    return np.sqrt(((predictions - targets) ** 2).mean())


def rmsn(predictions, targets):
    return np.sqrt(np.sum((predictions - targets) ** 2) * len(predictions)) / np.sum(targets)

# %%
# prepare artifical observed data
num_interval = nb.config.config_dict['DTA']['max_interval']

if os.path.isfile(os.path.join(data_folder_observed, 'true_7link_pathflow.pickle')):
    data_dict, config, true_f_car_driving, true_f_truck_driving, true_f_passenger_bustransit, true_f_car_pnr, true_f_bus, \
        m_car, m_truck, m_passenger, m_bus, L_car, L_truck, L_bus, L_passenger, \
        true_car_tt, true_truck_tt, true_passenger_tt, true_bus_tt \
         = pickle.load(open(os.path.join(data_folder_observed, 'true_7link_pathflow.pickle'), 'rb'))
else:

    # true time-dependen path flow
    true_f_car_driving = np.random.rand(
        num_interval * nb.config.config_dict['FIXED']['num_driving_path']) * 300
    true_f_truck_driving = np.random.rand(
        num_interval * nb.config.config_dict['FIXED']['num_driving_path']) * 30
    true_f_passenger_bustransit = np.random.rand(
        num_interval * nb.config.config_dict['FIXED']['num_bustransit_path']) * 100
    # true_f_passenger_bustransit = np.ones(
    #     num_interval * nb.config.config_dict['FIXED']['num_bustransit_path'])
    true_f_car_pnr = np.random.rand(
        num_interval * nb.config.config_dict['FIXED']['num_pnr_path']) * 50

    true_f_bus = nb.demand_bus.path_flow_matrix.flatten(order='F')


    # num_observed_link_driving = 4
    # assert(num_observed_link_driving <= len(nb.link_driving_list))
    # observed_link_driving_list = [3, 4, 5, 6]
    # assert(num_observed_link_driving <= len(observed_link_driving_list))

    # num_observed_link_bus = 5
    # assert(num_observed_link_bus <= len(nb.link_bus_list))
    # # observed_link_bus_list = list(map(lambda x: x.ID, np.random.choice(
    # #     nb.link_bus_list, num_observed_link_bus, replace=False)))
    # observed_link_bus_list = [203, 204, 205, 206, 207]
    # assert(num_observed_link_bus == len(observed_link_bus_list))

    # num_observed_link_walking = 5
    # assert(num_observed_link_walking <= len(nb.link_walking_list))
    # # observed_link_walking_list = list(map(lambda x: x.ID, np.random.choice(
    # #     nb.link_walking_list, num_observed_link_walking, replace=False)))
    # observed_link_walking_list = [12, 16, 17, 20, 24]
    # assert(num_observed_link_walking == len(observed_link_walking_list))

    num_observed_link_driving = 4
    assert(num_observed_link_driving <= len(nb.link_driving_list))
    observed_link_driving_list = [2, 3, 4, 5]
    assert(num_observed_link_driving == len(observed_link_driving_list))

    num_observed_link_bus = 3
    assert(num_observed_link_bus <= len(nb.link_bus_list))
    # observed_link_bus_list = list(map(lambda x: x.ID, np.random.choice(
    #     nb.link_bus_list, num_observed_link_bus, replace=False)))
    observed_link_bus_list = [201, 204, 208]
    assert(num_observed_link_bus == len(observed_link_bus_list))

    num_observed_link_walking = 0
    assert(num_observed_link_walking <= len(nb.link_walking_list))
    # observed_link_walking_list = list(map(lambda x: x.ID, np.random.choice(
    #     nb.link_walking_list, num_observed_link_walking, replace=False)))
    observed_link_walking_list = [] # [16, 17, 20]
    assert(num_observed_link_walking == len(observed_link_walking_list))


    ml_car = num_observed_link_driving
    ml_truck = num_observed_link_driving
    ml_bus = num_observed_link_bus
    ml_passenger = num_observed_link_bus + num_observed_link_walking

    L_car_one = np.eye(ml_car)
    L_truck_one = np.eye(ml_truck)
    L_bus_one = np.eye(ml_bus)
    L_passenger_one = np.eye(ml_passenger)

    L_car = csr_matrix(scipy.linalg.block_diag(
        *[L_car_one for i in range(num_interval)]))
    L_truck = csr_matrix(scipy.linalg.block_diag(
        *[L_truck_one for i in range(num_interval)]))
    L_bus = csr_matrix(scipy.linalg.block_diag(
        *[L_bus_one for i in range(num_interval)]))
    L_passenger = csr_matrix(scipy.linalg.block_diag(
        *[L_passenger_one for i in range(num_interval)]))

    # %%
    config = dict()

    config['use_car_link_flow'] = True
    config['use_truck_link_flow'] = True
    config['use_bus_link_flow'] = True
    config['use_passenger_link_flow'] = True

    config['use_car_link_tt'] = False
    config['use_truck_link_tt'] = False
    config['use_bus_link_tt'] = False
    config['use_passenger_link_tt'] = False

    config['car_count_agg'] = True
    config['truck_count_agg'] = True
    config['bus_count_agg'] = True
    config['passenger_count_agg'] = True

    config['link_car_flow_weight'] = 1
    config['link_truck_flow_weight'] = 1
    config['link_bus_flow_weight'] = 1
    config['link_passenger_flow_weight'] = 1

    config['link_car_tt_weight'] = 0
    config['link_truck_tt_weight'] = 0
    config['link_bus_tt_weight'] = 0
    config['link_passenger_tt_weight'] = 0

    config['num_data'] = 1

    config['observed_links_driving'] = np.array(observed_link_driving_list, dtype=int)
    config['observed_links_bus'] = np.array(observed_link_bus_list, dtype=int)
    config['observed_links_walking'] = np.array(observed_link_walking_list, dtype=int)

    config['paths_list_driving'] = np.array(
        [ID for ID in nb.path_table_driving.ID2path.keys()], dtype=int)
    config['paths_list_bustransit'] = np.array(
        [ID for ID in nb.path_table_bustransit.ID2path.keys()], dtype=int)
    config['paths_list_pnr'] = np.array(
        [ID for ID in nb.path_table_pnr.ID2path.keys()], dtype=int)
    config['paths_list_busroute'] = np.array(
        [ID for ID in nb.path_table_bus.ID2path.keys()], dtype=int)
    config['paths_list'] = np.concatenate((config['paths_list_driving'], config['paths_list_bustransit'],
                                           config['paths_list_pnr'], config['paths_list_busroute']))

    config['compute_car_link_flow_loss'] = True
    config['compute_truck_link_flow_loss'] = True
    config['compute_bus_link_flow_loss'] = True
    config['compute_passenger_link_flow_loss'] = True

    config['compute_car_link_tt_loss'] = True
    config['compute_truck_link_tt_loss'] = True
    config['compute_bus_link_tt_loss'] = True
    config['compute_passenger_link_tt_loss'] = True

    # %%
    dode = MMDODE(nb, config)
    dta = dode._run_simulation(true_f_car_driving, true_f_truck_driving,
                            true_f_passenger_bustransit, true_f_car_pnr)

    true_car_dar_matrix_driving, true_truck_dar_matrix_driving, true_car_dar_matrix_pnr, true_bus_dar_matrix_bustransit_link, true_bus_dar_matrix_driving_link, \
        true_passenger_dar_matrix_bustransit, true_passenger_dar_matrix_pnr = \
        dode.get_dar(dta, true_f_car_driving, true_f_truck_driving,
                    true_f_bus, true_f_passenger_bustransit, true_f_car_pnr)

    noise_level = 0.02

    data_dict = dict()

    data_dict['car_count_agg_L_list'] = list()
    data_dict['truck_count_agg_L_list'] = list()
    data_dict['bus_count_agg_L_list'] = list()
    data_dict['passenger_count_agg_L_list'] = list()

    data_dict['car_link_flow'] = []
    data_dict['truck_link_flow'] = []
    data_dict['bus_link_flow'] = []
    data_dict['passenger_link_flow'] = []

    data_dict['car_link_tt'] = []
    data_dict['truck_link_tt'] = []
    data_dict['bus_link_tt'] = []
    data_dict['passenger_link_tt'] = []

    start_intervals = np.arange(0, dode.num_loading_interval, dode.ass_freq)
    end_intervals = np.arange(0, dode.num_loading_interval, dode.ass_freq) + dode.ass_freq

    for i in range(config['num_data']):

        true_car_x = dta.get_link_car_inflow(start_intervals, end_intervals).flatten(order='F')
        true_truck_x = dta.get_link_truck_inflow(start_intervals, end_intervals).flatten(order='F')
        true_bus_x = dta.get_link_bus_inflow(start_intervals, end_intervals).flatten(order='F')
        true_passenger_x_bus = dta.get_link_bus_passenger_inflow(start_intervals, end_intervals)
        true_passenger_x_walking = dta.get_link_walking_passenger_inflow(start_intervals, end_intervals)
        true_passenger_x = np.concatenate((true_passenger_x_bus, true_passenger_x_walking), axis=0).flatten(order='F')

        # true_car_tt = dta.get_car_link_tt_robust(start_intervals, end_intervals).flatten(order = 'F')
        true_car_tt = dta.get_car_link_tt(start_intervals).flatten(order='F')
        true_truck_tt = dta.get_truck_link_tt(start_intervals).flatten(order='F')
        true_bus_tt = dta.get_bus_link_tt(start_intervals)
        true_walking_tt = dta.get_passenger_walking_link_tt(start_intervals)
        true_passenger_tt = np.concatenate((true_bus_tt, true_walking_tt), axis=0).flatten(order='F')
        true_bus_tt = true_bus_tt.flatten(order='F')

        m_car = L_car.dot(true_car_x)
        m_truck = L_truck.dot(true_truck_x)
        m_bus = L_bus.dot(true_bus_x)
        m_passenger = L_passenger.dot(true_passenger_x)

        data_dict['car_count_agg_L_list'].append(L_car)
        data_dict['truck_count_agg_L_list'].append(L_truck)
        data_dict['bus_count_agg_L_list'].append(L_bus)
        data_dict['passenger_count_agg_L_list'].append(L_passenger)

        data_dict['car_link_flow'].append(m_car + np.random.uniform(-1, 1, m_car.shape) * noise_level * m_car)
        data_dict['truck_link_flow'].append(m_truck + np.random.uniform(-1, 1, m_truck.shape) * noise_level * m_truck)
        data_dict['bus_link_flow'].append(m_bus + np.random.uniform(-1, 1, m_bus.shape) * noise_level * m_bus)
        data_dict['passenger_link_flow'].append(m_passenger + np.random.uniform(-1, 1, m_passenger.shape) * noise_level * m_passenger)

        data_dict['car_link_tt'].append(true_car_tt + np.random.uniform(-1, 1, true_car_tt.shape) * noise_level * true_car_tt)
        data_dict['truck_link_tt'].append(true_truck_tt + np.random.uniform(-1, 1, true_truck_tt.shape) * noise_level * true_truck_tt)
        data_dict['bus_link_tt'].append(true_bus_tt + np.random.uniform(-1, 1, true_bus_tt.shape) * noise_level * true_bus_tt)
        data_dict['passenger_link_tt'].append(true_passenger_tt + np.random.uniform(-1, 1, true_passenger_tt.shape) * noise_level * true_passenger_tt)

    pickle.dump(
        [data_dict, config, true_f_car_driving, true_f_truck_driving, true_f_passenger_bustransit, true_f_car_pnr, true_f_bus, 
         m_car, m_truck, m_passenger, m_bus, L_car, L_truck, L_bus, L_passenger,
         true_car_tt, true_truck_tt, true_passenger_tt, true_bus_tt],
        open(os.path.join(data_folder_observed, 'true_7link_pathflow.pickle'), 'wb')
    )

# %%
nb = MNM_network_builder()  # from MNM_mcnb, for python analysis
nb.load_from_folder(data_folder)
print(nb)

config['paths_list_driving'] = np.array(
    [ID for ID in nb.path_table_driving.ID2path.keys()], dtype=int)
config['paths_list_bustransit'] = np.array(
    [ID for ID in nb.path_table_bustransit.ID2path.keys()], dtype=int)
config['paths_list_pnr'] = np.array(
    [ID for ID in nb.path_table_pnr.ID2path.keys()], dtype=int)
config['paths_list_busroute'] = np.array(
    [ID for ID in nb.path_table_bus.ID2path.keys()], dtype=int)
config['paths_list'] = np.concatenate((config['paths_list_driving'], config['paths_list_bustransit'],
                                       config['paths_list_pnr'], config['paths_list_busroute']))


dode = MMDODE(nb, config)
dode.add_data(data_dict)

# %%
# max_epoch = 200
# column_generation = np.zeros(max_epoch, dtype=int)
# column_generation[:10] = 1
# passenger_step_size = np.ones(max_epoch)
# passenger_step_size[:100] = 0.01
# passenger_step_size[100:max_epoch] = 0.001
# truck_step_size = np.ones(max_epoch)
# truck_step_size[:100] = 0.1 
# truck_step_size[100:max_epoch] = 0.01
# f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, q_e_passenger, q_e_truck, loss_list = \
#     dode.estimate_demand(init_scale_passenger=900, init_scale_truck=10, 
#                          car_driving_scale=10, truck_driving_scale=1, passenger_bustransit_scale=1, car_pnr_scale=5,
#                          passenger_step_size=passenger_step_size, truck_step_size=truck_step_size,
#                          max_epoch=max_epoch, adagrad=True, column_generation=column_generation, use_tdsp=False,
#                          alpha_mode=(1., 1.5, 2.), beta_mode=1, alpha_path=1, beta_path=1, 
#                          use_file_as_init=None, save_folder=os.path.join(data_folder, 'record'), starting_epoch=0)

# max_epoch = 100
# column_generation = np.zeros(max_epoch, dtype=int)
# # column_generation[:10] = 1
# passenger_step_size = 0.1
# truck_step_size = 0.1
# f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, q_e_passenger, q_e_truck, loss_list = \
#     dode.estimate_demand_pytorch(init_scale_passenger=100, init_scale_truck=10, 
#                                 car_driving_scale=10, truck_driving_scale=1, passenger_bustransit_scale=5, car_pnr_scale=2,
#                                 passenger_step_size=passenger_step_size, truck_step_size=truck_step_size,
#                                 max_epoch=max_epoch, column_generation=column_generation, use_tdsp=False,
#                                 alpha_mode=(1., 1.5, 2.), beta_mode=1, alpha_path=1, beta_path=1, 
#                                 use_file_as_init=None, save_folder=os.path.join(data_folder, 'record'), starting_epoch=0)

max_epoch = 50
column_generation = np.zeros(max_epoch, dtype=bool)
# column_generation[:5] = 1

# f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, loss_list = \
#     dode.estimate_path_flow(car_driving_scale=100, truck_driving_scale=5, passenger_bustransit_scale=5, car_pnr_scale=5,
#                             car_driving_step_size=1.8, truck_driving_step_size=1, passenger_bustransit_step_size=1, car_pnr_step_size=1,
#                             max_epoch=max_epoch, adagrad=True, column_generation=column_generation, use_tdsp=False,
#                             use_file_as_init=None, save_folder=os.path.join(data_folder, 'record'), starting_epoch=0)

# f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, loss_list = \
#     dode.estimate_path_flow(car_driving_scale=10, truck_driving_scale=5, passenger_bustransit_scale=5, car_pnr_scale=10,
#                             car_driving_step_size=20, truck_driving_step_size=1, passenger_bustransit_step_size=5, car_pnr_step_size=2,
#                             max_epoch=max_epoch, adagrad=True, column_generation=column_generation, use_tdsp=False,
#                             use_file_as_init=None, save_folder=os.path.join(data_folder, 'record'), starting_epoch=0)

# f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, loss_list = \
#     dode.estimate_path_flow(car_driving_scale=10, truck_driving_scale=5, passenger_bustransit_scale=5, car_pnr_scale=5,
#                             car_driving_step_size=5, truck_driving_step_size=1, passenger_bustransit_step_size=2, car_pnr_step_size=5,
#                             max_epoch=max_epoch, adagrad=True, column_generation=column_generation, use_tdsp=False,
#                             use_file_as_init=None, 
#                             save_folder=os.path.join(data_folder, 'record'), starting_epoch=0)
# os.path.join(data_folder, 'record', str(24) + '_iteration_estimate_path_flow.pickle')

car_driving_step_size_modifier = np.ones(max_epoch, dtype=bool)
# car_driving_step_size_modifier[:10] = 0 
truck_driving_step_size_modifier = np.ones(max_epoch, dtype=bool)
# truck_driving_step_size_modifier[:10] = 0 
passenger_bustransit_step_size_modifier = np.ones(max_epoch, dtype=bool)

car_pnr_step_size_modifier = np.ones(max_epoch, dtype=bool)
# car_pnr_step_size_modifier[:10] = 0

f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, loss_list = \
    dode.estimate_path_flow_pytorch(car_driving_scale=10, truck_driving_scale=5, passenger_bustransit_scale=5, car_pnr_scale=5,
                                    car_driving_step_size=2, truck_driving_step_size=1, passenger_bustransit_step_size=1, car_pnr_step_size=1,
                                    car_driving_step_size_modifier=car_driving_step_size_modifier, 
                                    truck_driving_step_size_modifier=truck_driving_step_size_modifier, 
                                    passenger_bustransit_step_size_modifier=passenger_bustransit_step_size_modifier, 
                                    car_pnr_step_size_modifier=car_pnr_step_size_modifier,
                                    max_epoch=max_epoch, column_generation=column_generation, use_tdsp=False,
                                    use_file_as_init=None, 
                                    save_folder=os.path.join(data_folder, 'record'), starting_epoch=0)

# dode.config['link_car_flow_weight'] = 0
# dode.config['link_truck_flow_weight'] = 0
# dode.config['link_bus_flow_weight'] = 0
# dode.config['link_passenger_flow_weight'] = 0

# dode.config['link_car_tt_weight'] = 1
# dode.config['link_truck_tt_weight'] = 1
# dode.config['link_bus_tt_weight'] = 1
# dode.config['link_passenger_tt_weight'] = 1

# f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, loss_list = \
#     dode.estimate_path_flow_pytorch(car_driving_scale=50, truck_driving_scale=5, passenger_bustransit_scale=5, car_pnr_scale=5,
#                                     car_driving_step_size=1e-2, truck_driving_step_size=1e-2, passenger_bustransit_step_size=1e-2, car_pnr_step_size=1e-2,
#                                     car_driving_step_size_modifier=car_driving_step_size_modifier, 
#                                     truck_driving_step_size_modifier=truck_driving_step_size_modifier, 
#                                     passenger_bustransit_step_size_modifier=passenger_bustransit_step_size_modifier, 
#                                     car_pnr_step_size_modifier=car_pnr_step_size_modifier,
#                                     max_epoch=max_epoch, column_generation=column_generation, use_tdsp=False,
#                                     use_file_as_init=os.path.join(data_folder, 'record', str(49) + '_iteration_estimate_path_flow.pickle'), 
#                                     save_folder=os.path.join(data_folder, 'record'), starting_epoch=50)

# print("r2 --- f_car_driving: {}, f_truck_driving: {}, f_passenger_bustransit: {}, f_car_pnr: {}"
#       .format(r2_score(f_car_driving, true_f_car_driving), r2_score(f_truck_driving, true_f_truck_driving),
#               r2_score(f_passenger_bustransit, true_f_passenger_bustransit),
#               r2_score(f_car_pnr, true_f_car_pnr)))

# %% 
result_folder = os.path.join(data_folder, 'record')

# pickle.dump([true_f_car_driving, true_f_truck_driving, true_f_passenger_bustransit, true_f_car_pnr, 
#              f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, 
#              q_e_passenger, q_e_truck, 
#              loss_list, config, data_dict], open(os.path.join(result_folder, 'final_use_7link_demand.pickle'), 'wb'))

pickle.dump([true_f_car_driving, true_f_truck_driving, true_f_passenger_bustransit, true_f_car_pnr, 
             f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
             loss_list, config, data_dict], open(os.path.join(result_folder, 'final_use_7link_pathflow.pickle'), 'wb'))

# %%
color_list = ['teal', 'tomato', 'blue', 'sienna', 'plum', 'red', 'yellowgreen', 'khaki']
marker_list = ["o", "v", "^", "<", ">", "p", "D", "*", "s", "D", "p"]

plt.rc('font', size=20)          # controls default text sizes
plt.rc('axes', titlesize=20)     # fontsize of the axes title
plt.rc('axes', labelsize=20)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=20)    # fontsize of the tick labels
plt.rc('ytick', labelsize=20)    # fontsize of the tick labels
plt.rc('legend', fontsize=20)    # legend fontsize
plt.rc('figure', titlesize=20)  # fontsize of the figure title

# %%

# true_f_car_driving, true_f_truck_driving, true_f_passenger_bustransit, true_f_car_pnr, \
#     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, \
#     q_e_passenger, q_e_truck, \
#     loss_list, config, data_dict = pickle.load(open(os.path.join(result_folder, 'final_use_7link_demand.pickle'), 'rb'))

true_f_car_driving, true_f_truck_driving, true_f_passenger_bustransit, true_f_car_pnr, \
    f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, \
    loss_list, config, data_dict = pickle.load(open(os.path.join(result_folder, 'final_use_7link_pathflow.pickle'), 'rb'))

# %%
plt.figure(figsize = (16, 9), dpi=300)
plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[0], loss_list)), 
         color = color_list[0], marker = marker_list[0], linewidth = 3)
# plt.plot(range(len(l_list)), list(map(lambda x: x[0], l_list)),
#          color = color_list[4], linewidth = 3, label = "Total cost")

plt.ylabel('Loss', fontsize = 20)
plt.xlabel('Iteration', fontsize = 20)
# plt.legend()
# plt.ylim([0, 1])
plt.xlim([1, len(loss_list)])

# plt.savefig(os.path.join(result_folder, 'total_loss_demand.png'))
plt.savefig(os.path.join(result_folder, 'total_loss_pathflow.png'))

plt.show()

# %%
plt.figure(figsize = (16, 9), dpi=300)

plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[1]['car_count_loss']/loss_list[0][1]['car_count_loss'], loss_list)),
         color = color_list[0],  marker = marker_list[0], linewidth = 3, label = "Car observed flow")
plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[1]['truck_count_loss']/loss_list[0][1]['truck_count_loss'], loss_list)),
         color = color_list[1], marker = marker_list[1], linewidth = 3, label = "Truck observed flow")
plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[1]['passenger_count_loss']/loss_list[0][1]['passenger_count_loss'], loss_list)),
         color = color_list[2], marker = marker_list[2], linewidth = 3, label = "Passenger observed flow")

# plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[1]['car_tt_loss']/loss_list[0][1]['car_tt_loss'], loss_list)),
#          color = color_list[3],  marker = marker_list[3], linewidth = 3, label = "Car observed travel cost")
# plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[1]['truck_tt_loss']/loss_list[0][1]['truck_tt_loss'], loss_list)), 
#          color = color_list[4],  marker = marker_list[4], linewidth = 3, label = "Truck observed travel cost")
# plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[1]['passenger_tt_loss']/loss_list[0][1]['passenger_tt_loss'], loss_list)), 
#          color = color_list[5],  marker = marker_list[5], linewidth = 3, label = "Passenger observed travel cost")

# plt.plot(range(len(l_list)), list(map(lambda x: x[0]/8, l_list)),
#          color = color_list[6], linewidth = 3, label = "Total cost")

plt.ylabel('Loss')
plt.xlabel('Iteration')
plt.legend(loc='upper center', ncol=3)  # bbox_to_anchor=(1, 0.5)
plt.ylim([0, 1])
plt.xlim([1, len(loss_list)])

# plt.savefig(os.path.join(result_folder, 'breakdown_loss_demand.png'))
plt.savefig(os.path.join(result_folder, 'breakdown_loss_pathflow.png'))

plt.show()

# %%
dta = dode._run_simulation(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)
dta.print_simulation_results(os.path.join(data_folder, 'record'), 180)

start_intervals = np.arange(0, dode.num_loading_interval, dode.ass_freq)
end_intervals = np.arange(0, dode.num_loading_interval, dode.ass_freq) + dode.ass_freq

# tt_free = np.array(list(map(lambda x: dode.nb.get_link_driving(x).get_car_fft(), dode.observed_links_driving)))
# tt = np.zeros((len(dode.observed_links_driving), len(start_intervals)), dtype=np.float)
# car_link_in_cc = dict()
# car_link_out_cc = dict()
# for i, link_ID in enumerate(dode.observed_links_driving):
#     car_link_in_cc[link_ID] = dta.get_car_link_in_cc(link_ID)
#     car_link_out_cc[link_ID] = dta.get_car_link_out_cc(link_ID)
#     np.savetxt(os.path.join(data_folder, 'record', 'cc_in_py_link_{}.txt'.format(link_ID)), car_link_in_cc[link_ID])
#     np.savetxt(os.path.join(data_folder, 'record', 'cc_out_py_link_{}.txt'.format(link_ID)), car_link_out_cc[link_ID])
#     for j, assign_interval in enumerate(start_intervals):
#         tt[i, j] = dode._get_link_tt_from_cc(assign_interval, car_link_in_cc[link_ID], car_link_out_cc[link_ID], tt_free[i])

# tt_free = np.array(list(map(lambda x: dode.nb.get_link_driving(x).get_truck_fft(), dode.observed_links_driving)))
# tt = np.zeros((len(dode.observed_links_driving), len(start_intervals)), dtype=np.float)
# truck_link_in_cc = dict()
# truck_link_out_cc = dict()
# for i, link_ID in enumerate(dode.observed_links_driving):
#     truck_link_in_cc[link_ID] = dta.get_truck_link_in_cc(link_ID)
#     truck_link_out_cc[link_ID] = dta.get_truck_link_out_cc(link_ID)
#     np.savetxt(os.path.join(data_folder, 'record', 'cc_in_py_link_{}.txt'.format(link_ID)), truck_link_in_cc[link_ID])
#     np.savetxt(os.path.join(data_folder, 'record', 'cc_out_py_link_{}.txt'.format(link_ID)), truck_link_out_cc[link_ID])
#     for j, assign_interval in enumerate(start_intervals):
#         tt[i, j] = dode._get_link_tt_from_cc(assign_interval, truck_link_in_cc[link_ID], truck_link_out_cc[link_ID], tt_free[i])


estimated_car_x = dta.get_link_car_inflow(start_intervals, end_intervals).flatten(order='F')
estimated_truck_x = dta.get_link_truck_inflow(start_intervals, end_intervals).flatten(order='F')
estimated_bus_x = dta.get_link_bus_inflow(start_intervals, end_intervals).flatten(order='F')
estimated_passenger_x_bus = dta.get_link_bus_passenger_inflow(start_intervals, end_intervals)
estimated_passenger_x_walking = dta.get_link_walking_passenger_inflow(start_intervals, end_intervals)
estimated_passenger_x = np.concatenate((estimated_passenger_x_bus, estimated_passenger_x_walking), axis=0).flatten(order='F')

m_car_estimated = L_car.dot(estimated_car_x)
m_truck_estimated = L_truck.dot(estimated_truck_x)
m_bus_estimated = L_bus.dot(estimated_bus_x)
m_passenger_estimated = L_passenger.dot(estimated_passenger_x)

r2_car_count = r2_score(m_car_estimated, m_car)
r2_truck_count = r2_score(m_truck_estimated, m_truck)
r2_passenger_count = r2_score(m_passenger_estimated, m_passenger)
r2_bus_count = r2_score(m_bus_estimated, m_bus)
print("r2 count --- r2_car_count: {}, r2_truck_count: {}, r2_passenger_count: {}, r2_bus_count: {}"
      .format(r2_car_count, r2_truck_count, r2_passenger_count, r2_bus_count))

estimated_car_tt = dta.get_car_link_tt_robust(start_intervals, end_intervals).flatten(order = 'F')
# estimated_car_tt = dta.get_car_link_tt(start_intervals).flatten(order='F')
estimated_truck_tt = dta.get_truck_link_tt(start_intervals).flatten(order='F')
estimated_bus_tt = dta.get_bus_link_tt(start_intervals)
estimated_walking_tt = dta.get_passenger_walking_link_tt(start_intervals)
estimated_passenger_tt = np.concatenate((estimated_bus_tt, estimated_walking_tt), axis=0).flatten(order='F')
estimated_bus_tt = estimated_bus_tt.flatten(order='F')

estimated_car_tt /= dode.nb.config.config_dict['DTA']['unit_time']
true_car_tt /= dode.nb.config.config_dict['DTA']['unit_time']
estimated_truck_tt /= dode.nb.config.config_dict['DTA']['unit_time']
true_truck_tt /= dode.nb.config.config_dict['DTA']['unit_time']
estimated_passenger_tt /= dode.nb.config.config_dict['DTA']['unit_time']
true_passenger_tt /= dode.nb.config.config_dict['DTA']['unit_time']
estimated_bus_tt /= dode.nb.config.config_dict['DTA']['unit_time']
true_bus_tt /= dode.nb.config.config_dict['DTA']['unit_time']

r2_car_tt = r2_score(estimated_car_tt, true_car_tt)
r2_truck_tt = r2_score(estimated_truck_tt, true_truck_tt)
r2_passenger_tt = r2_score(estimated_passenger_tt, true_passenger_tt)
r2_bus_tt = r2_score(estimated_bus_tt, true_bus_tt)
print("r2 tt --- r2_car_tt: {}, r2_truck_tt: {}, r2_passenger_tt: {}, r2_bus_tt: {}"
      .format(r2_car_tt, r2_truck_tt, r2_passenger_tt, r2_bus_tt))

# pickle.dump([estimated_car_x, estimated_truck_x, estimated_passenger_x, estimated_bus_x,
#              m_car_estimated, m_truck_estimated, m_passenger_estimated, m_bus_estimated,
#              r2_car_count, r2_truck_count, r2_passenger_count, r2_bus_count,
#              start_intervals, end_intervals],
#              open(os.path.join(result_folder, 'final_use_7link_demand_r2_count.pickle'), 'wb'))

pickle.dump([estimated_car_x, estimated_truck_x, estimated_passenger_x, estimated_bus_x,
             m_car_estimated, m_truck_estimated, m_passenger_estimated, m_bus_estimated,
             r2_car_count, r2_truck_count, r2_passenger_count, r2_bus_count,
             estimated_car_tt, estimated_truck_tt, estimated_passenger_tt, estimated_bus_tt,
             r2_car_tt, r2_truck_tt, r2_passenger_tt, r2_bus_tt,
             start_intervals, end_intervals],
             open(os.path.join(result_folder, 'final_use_7link_pathflow_r2.pickle'), 'wb'))

# %%
m_car_max = int(np.max((np.max(m_car), np.max(m_car_estimated))) + 1)
m_truck_max = int(np.max((np.max(m_truck), np.max(m_truck_estimated))) + 1)
m_passenger_max = int(np.max((np.max(m_passenger), np.max(m_passenger_estimated))) + 1)

fig, axes = plt.subplots(1, 3, figsize=(24, 9), dpi=300)
# plt.figure(num=None, figsize=(16, 9), dpi=300, facecolor='w', edgecolor='k')
axes[0].scatter(m_car, m_car_estimated, color = 'teal', marker = '*', s = 100)
axes[0].plot(range(m_car_max), range(m_car_max), color = 'gray')
axes[1].scatter(m_truck, m_truck_estimated, color = 'tomato', marker = "^", s = 100)
axes[1].plot(range(m_truck_max), range(m_truck_max), color = 'gray')
axes[2].scatter(m_passenger, m_passenger_estimated, color = 'blue', marker = "o", s = 100)
axes[2].plot(range(m_passenger_max), range(m_passenger_max), color = 'gray')

axes[0].set_ylabel('Estimated observed flow for car')
axes[0].set_xlabel('True observed flow for car')
axes[1].set_ylabel('Estimated observed flow for truck')
axes[1].set_xlabel('True observed flow for truck')
axes[2].set_ylabel('Estimated observed flow for passenger')
axes[2].set_xlabel('True observed flow for passenger')

axes[0].set_xlim([0, m_car_max])
axes[0].set_ylim([0, m_car_max])
axes[1].set_xlim([0, m_truck_max])
axes[1].set_ylim([0, m_truck_max])
axes[2].set_xlim([0, m_passenger_max])
axes[2].set_ylim([0, m_passenger_max])

# plt.savefig(os.path.join(result_folder, 'link_flow_scatterplot_demand.png'))
plt.savefig(os.path.join(result_folder, 'link_flow_scatterplot_pathflow.png'))

plt.show()

# %%
# car_tt_min = np.min((np.min(true_car_tt), np.min(estimated_car_tt))) - 1
# truck_tt_min = np.min((np.min(true_truck_tt), np.min(estimated_truck_tt))) - 1
# passenger_tt_min = np.min((np.min(true_passenger_tt), np.min(estimated_passenger_tt))) - 1

# car_tt_max = np.max((np.max(true_car_tt), np.max(estimated_car_tt))) + 1
# truck_tt_max = np.max((np.max(true_truck_tt), np.max(estimated_truck_tt))) + 1
# passenger_tt_max = np.max((np.max(true_passenger_tt), np.max(estimated_passenger_tt))) + 1

# fig, axes = plt.subplots(1, 3, figsize=(24, 9), dpi=300)
# # plt.figure(num=None, figsize=(16, 9), dpi=300, facecolor='w', edgecolor='k')
# axes[0].scatter(true_car_tt, estimated_car_tt, color = 'teal', marker = '*', s = 100)
# axes[0].plot(np.linspace(car_tt_min, car_tt_max, 20), np.linspace(car_tt_min, car_tt_max, 20), color = 'gray')
# axes[1].scatter(true_truck_tt, estimated_truck_tt, color = 'tomato', marker = "^", s = 100)
# axes[1].plot(np.linspace(truck_tt_min, truck_tt_max, 20), np.linspace(truck_tt_min, truck_tt_max, 20), color = 'gray')
# axes[2].scatter(true_passenger_tt, estimated_passenger_tt, color = 'blue', marker = "o", s = 100)
# axes[2].plot(np.linspace(passenger_tt_min, passenger_tt_max, 20), np.linspace(passenger_tt_min, passenger_tt_max, 20), color = 'gray')

# axes[0].set_ylabel('Estimated observed travel cost for car')
# axes[0].set_xlabel('True observed travel cost for car')
# axes[1].set_ylabel('Estimated observed travel cost for truck')
# axes[1].set_xlabel('True observed travel cost for truck')
# axes[2].set_ylabel('Estimated observed travel cost for passenger')
# axes[2].set_xlabel('True observed travel cost for passenger')

# axes[0].set_xlim([car_tt_min, car_tt_max])
# axes[0].set_ylim([car_tt_min, car_tt_max])
# axes[1].set_xlim([truck_tt_min, truck_tt_max])
# axes[1].set_ylim([truck_tt_min, truck_tt_max])
# axes[2].set_xlim([passenger_tt_min, passenger_tt_max])
# axes[2].set_ylim([passenger_tt_min, passenger_tt_max])

# # plt.savefig(os.path.join(result_folder, 'link_tt_scatterplot_demand.png'))
# plt.savefig(os.path.join(result_folder, 'link_tt_scatterplot_pathflow.png'))

# plt.show()

# %%

# for i in range(max_epoch):
#     loss, loss_dict, f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
#         pickle.load(open(os.path.join(data_folder, 'record', str(i) + '_iteration_estimate_path_flow.pickle'), 'rb'))
#     print(loss)
#     print(loss_dict)
# %%
