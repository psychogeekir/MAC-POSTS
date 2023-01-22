# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'

# %%
import scipy
from scipy.sparse import csr_matrix, eye, block_diag
import numpy as np
import sys
import os
import pickle

macposts_dir = '/home/qiling/Documents/MAC-POSTS'

# %%
MNM_nb_folder = os.path.join(macposts_dir, 'side_project', 'network_builder') # MNM_nb, MNM_mcnb
sys.path.append(MNM_nb_folder)
python_lib_folder = os.path.join(macposts_dir, 'src', 'pylib') # covariance_tree and DODE, sDODE, mcDODE functions
sys.path.append(python_lib_folder)


# %%
import MNMAPI   # main DTA package
from MNM_mcnb import MNM_network_builder
from mcDODE import MCDODE, mcSPSA, PostProcessing


# %%
# /home/qiling/Documents/MAC-POSTS
print(os.getcwd())
data_folder = os.path.join(macposts_dir, 'data/input_files_7link_multiclass_new')

# %% [markdown]
# ### Use of network builder

# %%
nb = MNM_network_builder()  # from MNM_mcnb, for python analysis
nb.load_from_folder(data_folder)
print(nb)

# %% [markdown]
# ### Use of DNL

# %%
# directly run dta from data_folder
# dta = MNMAPI.mcdta_api()
# # int Mcdta_Api::initialize(std::string folder) in MAC-POSTS/src/pybinder/src/dta_api.cpp
# # MNM_Dta_Multiclass::MNM_Dta_Multiclass(std::string file_folder) in MAC-POSTS/src/minami/multiclass.cpp
# dta.initialize(data_folder)  
# # int Mcdta_Api::run_whole() in MAC-POSTS/src/pybinder/src/dta_api.cpp
# # int MNM_Dta::loading(bool verbose) in in MAC-POSTS/src/minami/dta.cpp
# dta.run_whole(False)  


# %%


# %% [markdown]
# ### MCDODE

# %%
# nb.dump_to_folder('test')


# %%
# prepare artifical observed data

num_interval = nb.config.config_dict['DTA']['max_interval']
override = False
if os.path.isfile(os.path.join(data_folder, 'true_7link_pathflow.pickle')) and not override:
    data_dict, config, true_f_car, true_f_truck, \
        m_car, m_truck, L_car, L_truck, \
            true_car_tt, true_truck_tt \
                = pickle.load(open(os.path.join(data_folder, 'true_7link_pathflow.pickle'), 'rb'))
else:

    observed_link_list = [3, 4, 5, 6]
    ml_car = 6
    ml_truck = 5
    data_dict = dict()

    # true time-dependent path flow
    true_f_car = np.random.rand(num_interval * nb.config.config_dict['FIXED']['num_path']) * 300
    true_f_truck = np.random.rand(num_interval * nb.config.config_dict['FIXED']['num_path']) * 30

    # true_car_x = np.random.rand(num_interval * len(observed_link_list)) * 100
    # true_truck_x = np.random.rand(num_interval * len(observed_link_list)) * 10

    # L_car_one and L_truck are binary matrices representing different ways to aggregate link count data in the observed records
    # number of rows = number of different aggregations, number of columns = number of observed links
    # usually they can be set as eye(len(observed_link_list)), i.e., no aggregation
    # L_car_one = np.random.randint(2, size = (ml_car, len(observed_link_list)))
    # L_truck_one = np.random.randint(2, size = (ml_truck, len(observed_link_list)))
    L_car_one = eye(len(observed_link_list))
    L_truck_one = eye(len(observed_link_list))
    # TODO: not compatible with mask_driving_link
    # L_car_one = np.array([[1, 0, 0, 1],
    #                       [0, 0, 1, 1],
    #                       [1, 1, 0, 1],
    #                       [1, 0, 1, 1],
    #                       [1, 0, 0, 0],
    #                       [0, 1, 0, 1]])
    # L_truck_one = np.array([[1, 0, 0, 1],
    #                         [0, 0, 0, 1],
    #                         [1, 1, 0, 1],
    #                         [1, 0, 1, 0],
    #                         [0, 1, 0, 1]])
    # L_car and L_truck consider the time dimension
    # # size = (num_interval*ml_car, num_interval*len(observed_link_list))                        
    # L_car = csr_matrix(scipy.linalg.block_diag(*[L_car_one for i in range(num_interval)]))  
    # # size = (num_interval*ml_truck, num_interval*len(observed_link_list))
    # L_truck = csr_matrix(scipy.linalg.block_diag(*[L_truck_one for i in range(num_interval)]))  

    # size = (num_interval*ml_car, num_interval*len(observed_link_list))                        
    L_car = csr_matrix(block_diag([L_car_one for _ in range(num_interval)]))  
    # size = (num_interval*ml_truck, num_interval*len(observed_link_list))
    L_truck = csr_matrix(block_diag([L_truck_one for _ in range(num_interval)]))  

    config = dict()
    config['use_car_link_flow'] = True
    config['use_truck_link_flow'] = True
    config['use_car_link_tt'] = True
    config['use_truck_link_tt'] = True
    config['car_count_agg'] = True
    config['truck_count_agg'] = True
    config['link_car_flow_weight'] = 1
    config['link_truck_flow_weight'] = 1
    config['link_car_tt_weight'] = 0.1
    config['link_truck_tt_weight'] = 0.1
    config['num_data'] = 1  # number of data entries for training, e.e., number of days collected
    config['observed_links'] = observed_link_list
    config['paths_list'] = np.arange(nb.config.config_dict['FIXED']['num_path'])


    config['compute_car_link_flow_loss'] = True
    config['compute_truck_link_flow_loss'] = True
    config['compute_car_link_tt_loss'] = True
    config['compute_truck_link_tt_loss'] = True

    # origin registration data
    config['use_origin_vehicle_registration_data'] = True
    config['compute_origin_vehicle_registration_loss'] = True
    config['origin_vehicle_registration_weight'] = 1

    dode = MCDODE(nb, config)
    dta = dode._run_simulation(true_f_car, true_f_truck)
    dta.build_link_cost_map(False)
    # dta.print_simulation_results(os.path.join(data_folder, 'record'), 12)
    (true_dar_car, true_dar_truck) = dode.get_dar(dta, true_f_car, true_f_truck)

    noise_level = 0.01
    # true time-dependent link flow
    true_car_x = true_dar_car.dot(true_f_car)
    true_truck_x = true_dar_truck.dot(true_f_truck)

    data_dict['car_count_agg_L_list'] = list()
    data_dict['truck_count_agg_L_list'] = list()
    data_dict['car_link_flow'] = []
    data_dict['truck_link_flow'] = []
    data_dict['car_link_tt'] = []
    data_dict['truck_link_tt'] = []
    for i in range(config['num_data']):
        true_car_x = dta.get_link_car_inflow(np.arange(0, dode.num_loading_interval, dode.ass_freq), 
                    np.arange(0, dode.num_loading_interval, dode.ass_freq) + dode.ass_freq).flatten(order = 'F')
        true_truck_x = dta.get_link_truck_inflow(np.arange(0, dode.num_loading_interval, dode.ass_freq), 
                    np.arange(0, dode.num_loading_interval, dode.ass_freq) + dode.ass_freq).flatten(order = 'F')
        # true_car_tt = dta.get_car_link_tt_robust(np.arange(0, dode.num_loading_interval, dode.ass_freq),
        #                          np.arange(0, dode.num_loading_interval, dode.ass_freq) + dode.ass_freq, dode.ass_freq, False).flatten(order = 'F')
        # true_truck_tt = dta.get_truck_link_tt_robust(np.arange(0, dode.num_loading_interval, dode.ass_freq),
        #                          np.arange(0, dode.num_loading_interval, dode.ass_freq) + dode.ass_freq, dode.ass_freq, False).flatten(order = 'F')
        true_car_tt = dta.get_car_link_tt(np.arange(0, dode.num_loading_interval, dode.ass_freq), False).flatten(order = 'F')
        true_truck_tt = dta.get_truck_link_tt(np.arange(0, dode.num_loading_interval, dode.ass_freq), False).flatten(order = 'F')
        # observed count data
        m_car = L_car.dot(true_car_x)
        m_truck = L_truck.dot(true_truck_x)
        data_dict['car_count_agg_L_list'].append(L_car)
        data_dict['truck_count_agg_L_list'].append(L_truck)
        # add noise to true data
        data_dict['car_link_flow'].append(m_car + np.random.uniform(-1, 1, m_car.shape) * noise_level * m_car)
        data_dict['truck_link_flow'].append(m_truck + np.random.uniform(-1, 1, m_truck.shape) * noise_level * m_truck)
        data_dict['car_link_tt'].append(true_car_tt + np.random.uniform(-1, 1, true_car_tt.shape) * noise_level * true_car_tt)
        data_dict['truck_link_tt'].append(true_truck_tt + np.random.uniform(-1, 1, true_truck_tt.shape) * noise_level * true_truck_tt)

    # %% origin registration data
    true_O_demand = dode.aggregate_f(true_f_car, true_f_truck)
    import pandas as pd
    zipcode_origin_biclass_demand = pd.DataFrame([{"zipcode": 1000, "origin_ID": [1], "car": true_O_demand[1][0], "truck": true_O_demand[1][1]}])
    data_dict['origin_vehicle_registration_data'] = list() 
    data_dict['origin_vehicle_registration_data'].append(zipcode_origin_biclass_demand)

    # %%
    pickle.dump(
            [data_dict, config, true_f_car, true_f_truck,
            m_car, m_truck, L_car, L_truck,
            true_car_tt, true_truck_tt],
            open(os.path.join(data_folder, 'true_7link_pathflow.pickle'), 'wb')
        )

config['paths_list'] = np.arange(nb.config.config_dict['FIXED']['num_path'])

# %%
# SPSA
# dode = mcSPSA(nb, config)
# Computational graph
dode = MCDODE(nb, config, num_procs=1)

# %%
is_updated, is_driving_link_covered = dode.check_registered_links_covered_by_registered_paths(data_folder + '/corrected_input_files', add=False)
assert(is_updated == 0)
assert(len(is_driving_link_covered) == len(config['observed_links']))
data_dict['mask_driving_link'] = is_driving_link_covered

print('coverage: driving link {}%'.format(
    data_dict['mask_driving_link'].sum() / len(data_dict['mask_driving_link']) * 100 if len(data_dict['mask_driving_link']) > 0 else 'NA'
))

# %%
dode.add_data(data_dict)


# %%
# nb.update_demand_path2(true_car_f, true_truck_f)
# nb.dump_to_folder("one")
# nb.update_demand_path2(car_flow, truck_flow)
# nb.dump_to_folder("two")


# %%
# (car_flow, truck_flow) = dode.init_path_flow(car_scale = 10, truck_scale = 1)


# %%
# pickle.dump((car_flow, truck_flow, None), open('test.pickle', 'w'))


# %%
# SPSA
# (car_flow, truck_flow, l_list) = dode.estimate_path_flow(max_epoch = 100, car_step_size = 0.1, 
#                                                          truck_step_size = 0.01, car_init_scale = 100, 
#                                                          truck_init_scale = 10, adagrad = False,
#                                                          delta_car_scale = 0.1, 
#                                                          delta_truck_scale = 0.01)
# Computational graph
# (car_flow, truck_flow, l_list) = dode.estimate_path_flow(max_epoch = 100, car_step_size = 0.1, 
#                                                          truck_step_size = 0.01, car_init_scale = 100, 
#                                                          truck_init_scale = 10, adagrad = True)

max_epoch = 100
starting_epoch = 0
column_generation = np.zeros(max_epoch, dtype=bool)
# try to generate new path every 5 iterations
# column_generation = np.array([True if (i > 0) and (i // 5 == 0) else False for i in range(max_epoch)])


(f_car, f_truck, x_e_car, x_e_truck, tt_e_car, tt_e_truck, O_demand, loss_list) = \
    dode.estimate_path_flow_pytorch(max_epoch = max_epoch, algo='NAdam', normalized_by_scale = True,
                                    car_step_size = 2, truck_step_size = 1, 
                                    car_init_scale = 5, truck_init_scale = 2, 
                                    link_car_flow_weight=1, link_truck_flow_weight=1, 
                                    link_car_tt_weight=0.01, link_truck_tt_weight=0.01, 
                                    origin_vehicle_registration_weight=1e-4, 
                                    starting_epoch=starting_epoch, store_folder=os.path.join(data_folder, 'record'),
                                    use_file_as_init=None if starting_epoch == 0 else os.path.join(data_folder, 'record', '{}_iteration.pickle'.format(starting_epoch - 1)),
                                    column_generation=column_generation)                                 

result_folder = os.path.join(data_folder, 'record')
pickle.dump([true_f_car, true_f_truck, f_car, f_truck,
             x_e_car, x_e_truck, tt_e_car, tt_e_truck, O_demand,
             loss_list, config, data_dict], open(os.path.join(result_folder, 'final_use_7link_pathflow.pickle'), 'wb'))

# %%
true_f_car, true_f_truck, f_car, f_truck, \
    x_e_car, x_e_truck, tt_e_car, tt_e_truck, O_demand, \
    loss_list, config, data_dict = pickle.load(open(os.path.join(data_folder, 'record', 'final_use_7link_pathflow.pickle'), 'rb'))

# %%
# %% rerun the dta with training result
nb = MNM_network_builder()  # from MNM_mmnb, for python analysis
nb.load_from_folder(os.path.join(data_folder, 'record', 'input_files_estimate_path_flow'))

dode = MCDODE(nb, config)
dode.add_data(data_dict)
# dta = dode._run_simulation(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, f_bus, counter=0, run_mmdta_adaptive=False)
# dta.print_simulation_results(os.path.join(data_folder, 'record'), 180)

start_intervals = np.arange(0, dode.num_loading_interval, dode.ass_freq)
end_intervals = np.arange(0, dode.num_loading_interval, dode.ass_freq) + dode.ass_freq

# %%
# postproc = PostProcessing(dode, dta, f_car, f_truck, result_folder=result_folder)
link_length = np.array([nb.get_link(link_ID).length for link_ID in config['observed_links']])  # mile
postproc = PostProcessing(dode, None, f_car, f_truck, x_e_car, x_e_truck, tt_e_car, tt_e_truck, link_length, O_demand, result_folder=result_folder)
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

# # %% travel speed
# postproc.cal_r2_speed()
# postproc.scatter_plot_speed('link_speed_scatterplot_pathflow.png')
