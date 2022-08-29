# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'

# %%
import scipy
from scipy.sparse import csr_matrix
import numpy as np
import sys
import os
import pickle


# %%
# MNM_nb_folder = os.path.join('..', '..', '..', 'side_project', 'network_builder') # MNM_nb, MNM_mcnb, MNM_mmnb
MNM_nb_folder = os.path.join('.', 'side_project', 'network_builder')  # MNM_nb, MNM_mcnb, MNM_mmnb
sys.path.append(MNM_nb_folder)

# python_lib_folder = os.path.join('..', '..', 'pylib') # covariance_tree and DODE, sDODE, mcDODE, mmDODE functions
python_lib_folder = os.path.join('.', 'src', 'pylib') # covariance_tree and DODE, sDODE, mcDODE, mmDODE functions
sys.path.append(python_lib_folder)

import MNMAPI   # main DTA package
from MNM_mmnb import MNM_network_builder
from mmDODE import MMDODE, PostProcessing


# %% ****************************** network topology ******************************

# /home/qiling/Documents/MAC-POSTS
data_folder_observed = os.path.join('/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode')
# data_folder_observed = os.path.join('/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode_columngeneration/record/target_input_files_estimate_demand')
# data_folder = os.path.join('/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode_columngeneration/record/input_files_estimate_path_flow')
data_folder = os.path.join('/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode_columngeneration')


# %% ****************************** prepare observed data (data_dict) and dode configuration (config) ******************************
nb = MNM_network_builder()  # from MNM_mcnb, for python analysis
nb.load_from_folder(data_folder_observed)
print(nb)

num_interval = nb.config.config_dict['DTA']['max_interval']
override = False
if os.path.isfile(os.path.join(data_folder, 'true_7link_pathflow.pickle')) and not override:
    data_dict, config, true_f_car_driving, true_f_truck_driving, true_f_passenger_bustransit, true_f_car_pnr, true_f_bus, \
        m_car, m_truck, m_passenger, m_bus, L_car, L_truck, L_bus, L_passenger, \
            true_car_tt, true_truck_tt, true_passenger_tt, true_bus_tt \
                = pickle.load(open(os.path.join(data_folder, 'true_7link_pathflow.pickle'), 'rb'))
else:

    # true time-dependen path flow
    true_f_car_driving = np.random.rand(
        num_interval * nb.config.config_dict['FIXED']['num_driving_path']) * 800
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
    observed_link_driving_list = [2, 3, 5, 6]
    assert(num_observed_link_driving == len(observed_link_driving_list))

    num_observed_link_bus = 4  # len(nb.link_bus_list)
    assert(num_observed_link_bus <= len(nb.link_bus_list))
    # observed_link_bus_list = [x.ID for x in nb.link_bus_list]
    observed_link_bus_list = list(map(lambda x: x.ID, np.random.choice(
        nb.link_bus_list, num_observed_link_bus, replace=False)))
    # observed_link_bus_list = [201, 204, 207, 208, 211]
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

    config['use_car_link_tt'] = True
    config['use_truck_link_tt'] = True
    config['use_bus_link_tt'] = True
    config['use_passenger_link_tt'] = False

    config['car_count_agg'] = True
    config['truck_count_agg'] = True
    config['bus_count_agg'] = True
    config['passenger_count_agg'] = True

    config['link_car_flow_weight'] = 1
    config['link_truck_flow_weight'] = 1
    config['link_bus_flow_weight'] = 1
    config['link_passenger_flow_weight'] = 1

    config['link_car_tt_weight'] = 1
    config['link_truck_tt_weight'] = 1
    config['link_bus_tt_weight'] = 1
    config['link_passenger_tt_weight'] = 0.0

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
    config['compute_passenger_link_tt_loss'] = False

    # %%
    dode = MMDODE(nb, config)
    dta = dode._run_simulation(true_f_car_driving, true_f_truck_driving,
                               true_f_passenger_bustransit, true_f_car_pnr, true_f_bus, counter=0, run_mmdta_adaptive=False)
    
    # dta = dode._run_simulation(None, None, None, None, counter=0, run_mmdta_adaptive=False)

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

    true_car_x = dta.get_link_car_inflow(start_intervals, end_intervals).flatten(order='F')
    true_truck_x = dta.get_link_truck_inflow(start_intervals, end_intervals).flatten(order='F')
    true_bus_x = dta.get_link_bus_inflow(start_intervals, end_intervals).flatten(order='F')
    true_passenger_x_bus = dta.get_link_bus_passenger_inflow(start_intervals, end_intervals)
    true_passenger_x_walking = dta.get_link_walking_passenger_inflow(start_intervals, end_intervals)
    true_passenger_x = np.concatenate((true_passenger_x_bus, true_passenger_x_walking), axis=0).flatten(order='F')

    true_car_tt = dta.get_car_link_tt_robust(start_intervals, end_intervals, dode.ass_freq, True).flatten(order = 'F')
    true_truck_tt = dta.get_truck_link_tt_robust(start_intervals, end_intervals, dode.ass_freq, True).flatten(order='F')
    true_bus_tt = dta.get_bus_link_tt_robust(start_intervals, end_intervals, dode.ass_freq, True, True)
    true_walking_tt = dta.get_passenger_walking_link_tt_robust(start_intervals, end_intervals, dode.ass_freq)
    true_passenger_tt = np.concatenate((true_bus_tt, true_walking_tt), axis=0).flatten(order='F')
    true_bus_tt = true_bus_tt.flatten(order='F')

    # true_car_tt = dta.get_car_link_tt(start_intervals, True).flatten(order='F')
    # true_truck_tt = dta.get_truck_link_tt(start_intervals, True).flatten(order='F')
    # true_bus_tt = dta.get_bus_link_tt(start_intervals, True, True)
    # true_walking_tt = dta.get_passenger_walking_link_tt(start_intervals)
    # true_passenger_tt = np.concatenate((true_bus_tt, true_walking_tt), axis=0).flatten(order='F')
    # true_bus_tt = true_bus_tt.flatten(order='F')

    m_car = L_car.dot(true_car_x)
    m_truck = L_truck.dot(true_truck_x)
    m_bus = L_bus.dot(true_bus_x)
    m_passenger = L_passenger.dot(true_passenger_x)

    for i in range(config['num_data']):

        data_dict['car_count_agg_L_list'].append(L_car)
        data_dict['truck_count_agg_L_list'].append(L_truck)
        data_dict['bus_count_agg_L_list'].append(L_bus)
        data_dict['passenger_count_agg_L_list'].append(L_passenger)

        data_dict['car_link_flow'].append(m_car + np.random.uniform(-1, 1, m_car.shape) * noise_level * m_car)
        data_dict['truck_link_flow'].append(m_truck + np.random.uniform(-1, 1, m_truck.shape) * noise_level * m_truck)
        data_dict['bus_link_flow'].append(m_bus + np.random.uniform(-1, 1, m_bus.shape) * noise_level * m_bus)
        data_dict['passenger_link_flow'].append(m_passenger + np.random.uniform(-1, 1, m_passenger.shape) * noise_level * m_passenger)

        data_dict['car_link_tt'].append(true_car_tt + np.random.uniform(-1, 1, true_car_tt.shape) * noise_level * np.nan_to_num(true_car_tt, posinf=0))
        data_dict['truck_link_tt'].append(true_truck_tt + np.random.uniform(-1, 1, true_truck_tt.shape) * noise_level * np.nan_to_num(true_truck_tt, posinf=0))
        data_dict['bus_link_tt'].append(true_bus_tt + np.random.uniform(-1, 1, true_bus_tt.shape) * noise_level * np.nan_to_num(true_bus_tt, posinf=0))
        data_dict['passenger_link_tt'].append(true_passenger_tt + np.random.uniform(-1, 1, true_passenger_tt.shape) * noise_level * np.nan_to_num(true_passenger_tt, posinf=0))

    pickle.dump(
        [data_dict, config, true_f_car_driving, true_f_truck_driving, true_f_passenger_bustransit, true_f_car_pnr, true_f_bus, 
         m_car, m_truck, m_passenger, m_bus, L_car, L_truck, L_bus, L_passenger,
         true_car_tt, true_truck_tt, true_passenger_tt, true_bus_tt],
        open(os.path.join(data_folder, 'true_7link_pathflow.pickle'), 'wb')
    )

# %% ****************************** DODE ******************************
# don't regenerate path, modify data
nb = MNM_network_builder()  # from MNM_mmnb, for python analysis
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
is_updated, is_driving_link_covered, is_bus_walking_link_covered = dode.check_registered_links_covered_by_registered_paths(data_folder + '/corrected_input_files', add=False)
assert(is_updated == 0)
assert(len(is_driving_link_covered) == len(config['observed_links_driving']))
assert(len(is_bus_walking_link_covered) == len(config['observed_links_bus']) + len(config['observed_links_walking']))
data_dict['mask_driving_link'] = is_driving_link_covered
data_dict['mask_bus_link'] = is_bus_walking_link_covered[:len(config['observed_links_bus'])]
data_dict['mask_walking_link'] = is_bus_walking_link_covered[len(config['observed_links_bus']):]

print('coverage: driving link {}%, bus link {}%, walking link {}%'.format(
    data_dict['mask_driving_link'].sum() / len(data_dict['mask_driving_link']) * 100 if len(data_dict['mask_driving_link']) > 0 else 'NA',
    data_dict['mask_bus_link'].sum() / len(data_dict['mask_bus_link']) * 100 if len(data_dict['mask_bus_link']) > 0 else 'NA',
    data_dict['mask_walking_link'].sum() / len(data_dict['mask_walking_link']) * 100 if len(data_dict['mask_walking_link']) > 0 else 'NA'
))

# regenerate path to cover
# if os.path.exists(data_folder + '/corrected_input_files'):
#     nb = MNM_network_builder()  # from MNM_mmnb, for python analysis
#     nb.load_from_folder(data_folder + '/corrected_input_files')

#     config['paths_list_driving'] = np.array(
#         [ID for ID in nb.path_table_driving.ID2path.keys()], dtype=int)
#     config['paths_list_bustransit'] = np.array(
#         [ID for ID in nb.path_table_bustransit.ID2path.keys()], dtype=int)
#     config['paths_list_pnr'] = np.array(
#         [ID for ID in nb.path_table_pnr.ID2path.keys()], dtype=int)
#     config['paths_list_busroute'] = np.array(
#         [ID for ID in nb.path_table_bus.ID2path.keys()], dtype=int)
#     config['paths_list'] = np.concatenate((config['paths_list_driving'], config['paths_list_bustransit'],
#                                            config['paths_list_pnr'], config['paths_list_busroute']))

#     dode = MMDODE(nb, config)

# else:
#     nb = MNM_network_builder()  # from MNM_mmnb, for python analysis
#     nb.load_from_folder(data_folder)
#     print(nb)

#     config['paths_list_driving'] = np.array(
#         [ID for ID in nb.path_table_driving.ID2path.keys()], dtype=int)
#     config['paths_list_bustransit'] = np.array(
#         [ID for ID in nb.path_table_bustransit.ID2path.keys()], dtype=int)
#     config['paths_list_pnr'] = np.array(
#         [ID for ID in nb.path_table_pnr.ID2path.keys()], dtype=int)
#     config['paths_list_busroute'] = np.array(
#         [ID for ID in nb.path_table_bus.ID2path.keys()], dtype=int)
#     config['paths_list'] = np.concatenate((config['paths_list_driving'], config['paths_list_bustransit'],
#                                         config['paths_list_pnr'], config['paths_list_busroute']))

#     dode = MMDODE(nb, config)
#     is_updated, is_driving_link_covered, is_bus_walking_link_covered = dode.check_registered_links_covered_by_registered_paths(data_folder + '/corrected_input_files')
#     if is_updated > 0:
#         nb = MNM_network_builder()  # from MNM_mmnb, for python analysis
#         nb.load_from_folder(data_folder + '/corrected_input_files')

#         config['paths_list_driving'] = np.array(
#             [ID for ID in nb.path_table_driving.ID2path.keys()], dtype=int)
#         config['paths_list_bustransit'] = np.array(
#             [ID for ID in nb.path_table_bustransit.ID2path.keys()], dtype=int)
#         config['paths_list_pnr'] = np.array(
#             [ID for ID in nb.path_table_pnr.ID2path.keys()], dtype=int)
#         config['paths_list_busroute'] = np.array(
#             [ID for ID in nb.path_table_bus.ID2path.keys()], dtype=int)
#         config['paths_list'] = np.concatenate((config['paths_list_driving'], config['paths_list_bustransit'],
#                                             config['paths_list_pnr'], config['paths_list_busroute']))
#         dode.reinitialize(nb, config)

# add training data
dode.add_data(data_dict)

# %%
max_epoch = 100

column_generation = np.zeros(max_epoch, dtype=bool)
column_generation[:50] = 1

link_car_flow_weight = np.ones(max_epoch, dtype=float) * 1
# link_car_flow_weight[50:] = 0
link_truck_flow_weight = np.ones(max_epoch, dtype=float) * 1
# link_truck_flow_weight[50:] = 0
link_passenger_flow_weight = np.ones(max_epoch, dtype=float) * 1
# link_passenger_flow_weight[50:] = 0
link_bus_flow_weight = np.ones(max_epoch, dtype=float) * 1e-1 # * 1e-5
# link_bus_flow_weight[:50] = 0

link_car_tt_weight = np.ones(max_epoch, dtype=float) * 1e-1
# link_car_tt_weight[:50] = 0
link_truck_tt_weight = np.ones(max_epoch, dtype=float) * 1e-1
# link_truck_tt_weight[:50] = 0
link_passenger_tt_weight = np.zeros(max_epoch, dtype=float)
link_bus_tt_weight = np.ones(max_epoch, dtype=float) * 1e-1
# link_bus_tt_weight[:50] = 0

# %%
# passenger_step_size = np.ones(max_epoch)
# passenger_step_size[:100] = 0.01
# passenger_step_size[100:max_epoch] = 0.001
# truck_step_size = np.ones(max_epoch)
# truck_step_size[:100] = 0.1 
# truck_step_size[100:max_epoch] = 0.01
# f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, q_e_passenger, q_e_truck, x_e_car, x_e_truck, x_e_passenger, x_e_bus, tt_e_car, tt_e_truck, tt_e_passenger, tt_e_bus, loss_list = \
#     dode.estimate_demand(init_scale_passenger=900, init_scale_truck=10, 
#                          car_driving_scale=10, truck_driving_scale=1, passenger_bustransit_scale=1, car_pnr_scale=5,
#                          passenger_step_size=passenger_step_size, truck_step_size=truck_step_size,
#                          link_car_flow_weight=link_car_flow_weight, link_truck_flow_weight=link_truck_flow_weight, link_passenger_flow_weight=link_passenger_flow_weight, link_bus_flow_weight=link_bus_flow_weight,
#                          link_car_tt_weight=link_car_tt_weight, link_truck_tt_weight=link_truck_tt_weight, link_passenger_tt_weight=link_passenger_tt_weight, link_bus_tt_weight=link_bus_tt_weight,
#                          max_epoch=max_epoch, adagrad=True, column_generation=column_generation, use_tdsp=False,
#                          alpha_mode=(1., 1.5, 2.), beta_mode=1, alpha_path=1, beta_path=1, 
#                          use_file_as_init=None, save_folder=os.path.join(data_folder, 'record'), starting_epoch=0)

# passenger_step_size = 0.1
# truck_step_size = 0.03
# f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, q_e_passenger, q_e_truck, x_e_car, x_e_truck, x_e_passenger, x_e_bus, tt_e_car, tt_e_truck, tt_e_passenger, tt_e_bus, loss_list = \
#     dode.estimate_demand_pytorch(init_scale_passenger=100, init_scale_truck=20, 
#                                 car_driving_scale=10, truck_driving_scale=1, passenger_bustransit_scale=5, car_pnr_scale=2,
#                                 passenger_step_size=passenger_step_size, truck_step_size=truck_step_size,
#                                 link_car_flow_weight=link_car_flow_weight, link_truck_flow_weight=link_truck_flow_weight, link_passenger_flow_weight=link_passenger_flow_weight, link_bus_flow_weight=link_bus_flow_weight,
#                                 link_car_tt_weight=link_car_tt_weight, link_truck_tt_weight=link_truck_tt_weight, link_passenger_tt_weight=link_passenger_tt_weight, link_bus_tt_weight=link_bus_tt_weight,
#                                 max_epoch=max_epoch, column_generation=column_generation, use_tdsp=False,
#                                 alpha_mode=(1., 1.5, 2.), beta_mode=1, alpha_path=1, beta_path=1, 
#                                 use_file_as_init=None, save_folder=os.path.join(data_folder, 'record'), starting_epoch=0)

# %%
# for i in range(max_epoch):
#     if i % 5 == 0:
#         column_generation[i] = 1

# f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, x_e_car, x_e_truck, x_e_passenger, x_e_bus, tt_e_car, tt_e_truck, tt_e_passenger, tt_e_bus, loss_list = \
#     dode.estimate_path_flow(car_driving_scale=10, truck_driving_scale=5, passenger_bustransit_scale=5, car_pnr_scale=10,
#                             car_driving_step_size=10, truck_driving_step_size=1, passenger_bustransit_step_size=2, car_pnr_step_size=2,
#                             link_car_flow_weight=link_car_flow_weight, link_truck_flow_weight=link_truck_flow_weight, link_passenger_flow_weight=link_passenger_flow_weight, link_bus_flow_weight=link_bus_flow_weight,
#                             link_car_tt_weight=link_car_tt_weight, link_truck_tt_weight=link_truck_tt_weight, link_passenger_tt_weight=link_passenger_tt_weight, link_bus_tt_weight=link_bus_tt_weight,
#                             max_epoch=max_epoch, adagrad=True, column_generation=column_generation, use_tdsp=True,
#                             use_file_as_init=None, save_folder=os.path.join(data_folder, 'record'), starting_epoch=0)


# f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, x_e_car, x_e_truck, x_e_passenger, x_e_bus, tt_e_car, tt_e_truck, tt_e_passenger, tt_e_bus, loss_list = \
#     dode.estimate_path_flow_pytorch2(car_driving_scale=100, truck_driving_scale=5, passenger_bustransit_scale=5, car_pnr_scale=2, bus_scale=1,
#                                     car_driving_step_size=1, truck_driving_step_size=0.8, passenger_bustransit_step_size=1, car_pnr_step_size=1, bus_step_size=0.2,
#                                     link_car_flow_weight=link_car_flow_weight, link_truck_flow_weight=link_truck_flow_weight, link_passenger_flow_weight=link_passenger_flow_weight, link_bus_flow_weight=link_bus_flow_weight,
#                                     link_car_tt_weight=link_car_tt_weight, link_truck_tt_weight=link_truck_tt_weight, link_passenger_tt_weight=link_passenger_tt_weight, link_bus_tt_weight=link_bus_tt_weight,
#                                     max_epoch=max_epoch, algo="NAdam", fix_bus=True, column_generation=column_generation, use_tdsp=True,
#                                     use_file_as_init=None, 
#                                     save_folder=os.path.join(data_folder, 'record'), starting_epoch=0)

# f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, x_e_car, x_e_truck, x_e_passenger, x_e_bus, tt_e_car, tt_e_truck, tt_e_passenger, tt_e_bus, loss_list = \
#     dode.estimate_path_flow_pytorch2(car_driving_scale=10, truck_driving_scale=5, passenger_bustransit_scale=5, car_pnr_scale=1, bus_scale=1,
#                                     car_driving_step_size=0, truck_driving_step_size=0, passenger_bustransit_step_size=1, car_pnr_step_size=0.1, bus_step_size=0.2,
#                                     link_car_flow_weight=link_car_flow_weight, link_truck_flow_weight=link_truck_flow_weight, link_passenger_flow_weight=link_passenger_flow_weight, link_bus_flow_weight=link_bus_flow_weight,
#                                     link_car_tt_weight=link_car_tt_weight, link_truck_tt_weight=link_truck_tt_weight, link_passenger_tt_weight=link_passenger_tt_weight, link_bus_tt_weight=link_bus_tt_weight,
#                                     max_epoch=max_epoch, algo="NAdam", fix_bus=True, column_generation=column_generation, use_tdsp=True,
#                                     use_file_as_init=os.path.join(data_folder, 'record', str(99) + '_iteration_estimate_path_flow.pickle'), 
#                                     save_folder=os.path.join(data_folder, 'record'), starting_epoch=100)

log_f_car_driving, log_f_truck_driving, log_f_passenger_bustransit, log_f_car_pnr, log_f_bus, \
    f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, f_bus, \
    x_e_car, x_e_truck, x_e_passenger, x_e_bus, tt_e_car, tt_e_truck, tt_e_passenger, tt_e_bus, loss_list = \
    dode.estimate_path_flow_pytorch(car_driving_scale=10, truck_driving_scale=2, passenger_bustransit_scale=2, car_pnr_scale=1, bus_scale=1,
                                    car_driving_step_size=2, truck_driving_step_size=1, passenger_bustransit_step_size=1.8, car_pnr_step_size=1, bus_step_size=None,
                                    link_car_flow_weight=link_car_flow_weight, link_truck_flow_weight=link_truck_flow_weight, link_passenger_flow_weight=link_passenger_flow_weight, link_bus_flow_weight=link_bus_flow_weight,
                                    link_car_tt_weight=link_car_tt_weight, link_truck_tt_weight=link_truck_tt_weight, link_passenger_tt_weight=link_passenger_tt_weight, link_bus_tt_weight=link_bus_tt_weight,
                                    max_epoch=max_epoch, algo="NAdam", l2_coeff=0, fix_bus=True, column_generation=column_generation, use_tdsp=True,
                                    use_file_as_init=None, save_folder=os.path.join(data_folder, 'record'), starting_epoch=0)


# print("r2 --- f_car_driving: {}, f_truck_driving: {}, f_passenger_bustransit: {}, f_car_pnr: {}"
#       .format(r2_score(true_f_car_driving, f_car_driving), r2_score(true_f_truck_driving, f_truck_driving),
#               r2_score(true_f_passenger_bustransit, f_passenger_bustransit),
#               r2_score(true_f_car_pnr, f_car_pnr)))

# %% save best results
result_folder = os.path.join(data_folder, 'record')

# pickle.dump([true_f_car_driving, true_f_truck_driving, true_f_passenger_bustransit, true_f_car_pnr, true_f_bus,
#              f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, f_bus,
#              q_e_passenger, q_e_truck, 
#              x_e_car, x_e_truck, x_e_passenger, x_e_bus, tt_e_car, tt_e_truck, tt_e_passenger, tt_e_bus,
#              loss_list, config, data_dict], open(os.path.join(result_folder, 'final_use_7link_demand.pickle'), 'wb'))

pickle.dump([true_f_car_driving, true_f_truck_driving, true_f_passenger_bustransit, true_f_car_pnr, true_f_bus,
             f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, f_bus,
             x_e_car, x_e_truck, x_e_passenger, x_e_bus, tt_e_car, tt_e_truck, tt_e_passenger, tt_e_bus,
             loss_list, config, data_dict], open(os.path.join(result_folder, 'final_use_7link_pathflow.pickle'), 'wb'))

# %% ****************************** post processing ******************************

# true_f_car_driving, true_f_truck_driving, true_f_passenger_bustransit, true_f_car_pnr, true_f_bus, \
#     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, f_bus, \
#     q_e_passenger, q_e_truck, \
#     x_e_car, x_e_truck, x_e_passenger, x_e_bus, tt_e_car, tt_e_truck, tt_e_passenger, tt_e_bus, \
#     loss_list, config, data_dict = pickle.load(open(os.path.join(result_folder, 'final_use_7link_demand.pickle'), 'rb'))

true_f_car_driving, true_f_truck_driving, true_f_passenger_bustransit, true_f_car_pnr, true_f_bus, \
    f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, f_bus, \
    x_e_car, x_e_truck, x_e_passenger, x_e_bus, tt_e_car, tt_e_truck, tt_e_passenger, tt_e_bus, \
    loss_list, config, data_dict = pickle.load(open(os.path.join(result_folder, 'final_use_7link_pathflow.pickle'), 'rb'))


# %% rerun the dta with training result
nb = MNM_network_builder()  # from MNM_mmnb, for python analysis
# nb.load_from_folder(os.path.join(data_folder, 'record', 'input_files_estimate_demand'))
nb.load_from_folder(os.path.join(data_folder, 'record', 'input_files_estimate_path_flow'))

dode = MMDODE(nb, config)
dode.add_data(data_dict)
# dta = dode._run_simulation(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, f_bus, counter=0, run_mmdta_adaptive=False)
# dta.print_simulation_results(os.path.join(data_folder, 'record'), 180)

start_intervals = np.arange(0, dode.num_loading_interval, dode.ass_freq)
end_intervals = np.arange(0, dode.num_loading_interval, dode.ass_freq) + dode.ass_freq

# %%
# postproc = PostProcessing(dode, dta, result_folder=result_folder)
postproc = PostProcessing(dode, None, x_e_car, x_e_truck, x_e_passenger, x_e_bus, tt_e_car, tt_e_truck, tt_e_passenger, tt_e_bus, result_folder=result_folder)
postproc.get_one_data(start_intervals, end_intervals, j=0)

# %% total loss
postproc.plot_total_loss(loss_list, 'total_loss_pathflow.png')

# %% breakdown loss
postproc.plot_breakdown_loss(loss_list, 'breakdown_loss_pathflow.png')

# %% count
postproc.cal_r2_count()
postproc.scatter_plot_count('link_flow_scatterplot_pathflow.png')

# %% travel time
postproc.cal_r2_cost()
postproc.scatter_plot_cost('link_cost_scatterplot_pathflow.png')
