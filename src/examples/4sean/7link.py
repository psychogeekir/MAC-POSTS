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
import pickle
from collections import OrderedDict
import copy
from scipy.sparse import csr_matrix
from scipy import io
import seaborn as sns
import joblib
# from base import *
from joblib import Parallel, delayed
import random
import scipy


# %%
from matplotlib import colors
import six
import matplotlib.dates as mdates
import datetime
import pandas as pd
import seaborn as sns
sns.set()
plt.style.use('seaborn-poster')
from sklearn.metrics import r2_score


# %%
# MNM_nb_folder = os.path.join('..', '..', '..', 'side_project', 'network_builder') # MNM_nb, MNM_mcnb
MNM_nb_folder = os.path.join('.', 'side_project', 'network_builder') # MNM_nb, MNM_mcnb
sys.path.append(MNM_nb_folder)
# python_lib_folder = os.path.join('..', '..', 'pylib') # covariance_tree and DODE, sDODE, mcDODE functions
python_lib_folder = os.path.join('.', 'src', 'pylib') # covariance_tree and DODE, sDODE, mcDODE functions
sys.path.append(python_lib_folder)


# %%
import MNMAPI   # main DTA package
from MNM_mcnb import MNM_network_builder
from mcDODE import MCDODE, mcSPSA  


# %%
# /home/qiling/Documents/MAC-POSTS
print(os.getcwd())
data_folder = os.path.join('/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multiclass_new')

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
dta = MNMAPI.mcdta_api()
# int Mcdta_Api::initialize(std::string folder) in MAC-POSTS/src/pybinder/src/dta_api.cpp
# MNM_Dta_Multiclass::MNM_Dta_Multiclass(std::string file_folder) in MAC-POSTS/src/minami/multiclass.cpp
dta.initialize(data_folder)  
# int Mcdta_Api::run_whole() in MAC-POSTS/src/pybinder/src/dta_api.cpp
# int MNM_Dta::loading(bool verbose) in in MAC-POSTS/src/minami/dta.cpp
dta.run_whole()  


# %%


# %% [markdown]
# ### MCDODE

# %%
# nb.dump_to_folder('test')


# %%
from sklearn.metrics import r2_score
from scipy import stats
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
observed_link_list = [3, 4, 5, 6]
ml_car = 6
ml_truck = 5
data_dict = dict()
num_interval = nb.config.config_dict['DTA']['max_interval']

# true time-dependen path flow
true_car_f = np.random.rand(num_interval * nb.config.config_dict['FIXED']['num_path']) * 300
true_truck_f = np.random.rand(num_interval * nb.config.config_dict['FIXED']['num_path']) * 30
# true_car_f, true_truck_f, _, _, _, _ = pickle.load(open('final_use.pickle', 'r'))

# true_car_x = np.random.rand(num_interval * len(observed_link_list)) * 100
# true_truck_x = np.random.rand(num_interval * len(observed_link_list)) * 10

# L_car_one and L_truck are binary matrices representing different ways to aggregate link count data in the observed records
# number of rows = number of different aggregations, number of columns = number of observed links
# usually they can be set as eye(len(observed_link_list)), i.e., no aggregation
# L_car_one = np.random.randint(2, size = (ml_car, len(observed_link_list)))
# L_truck_one = np.random.randint(2, size = (ml_truck, len(observed_link_list)))
L_car_one = np.array([[1, 0, 0, 1],
                      [0, 0, 1, 1],
                      [1, 1, 0, 1],
                      [1, 0, 1, 1],
                      [1, 0, 0, 0],
                      [0, 1, 0, 1]])
L_truck_one = np.array([[1, 0, 0, 1],
                        [0, 0, 0, 1],
                        [1, 1, 0, 1],
                        [1, 0, 1, 0],
                        [0, 1, 0, 1]])
# L_car and L_truck consider the time dimension
# size = (num_interval*ml_car, num_interval*len(observed_link_list))                        
L_car = csr_matrix(scipy.linalg.block_diag(*[L_car_one for i in range(num_interval)]))  
# size = (num_interval*ml_truck, num_interval*len(observed_link_list))
L_truck = csr_matrix(scipy.linalg.block_diag(*[L_truck_one for i in range(num_interval)]))  

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
config['num_data'] = 8  # number of data entries for training, e.e., number of days collected
config['observed_links'] = observed_link_list
config['paths_list'] = range(nb.config.config_dict['FIXED']['num_path'])


config['compute_car_link_flow_loss'] = True
config['compute_truck_link_flow_loss'] = True
config['compute_car_link_tt_loss'] = True
config['compute_truck_link_tt_loss'] = True

dode = MCDODE(nb, config)
dta = dode._run_simulation(true_car_f, true_truck_f)
(true_dar_car, true_dar_truck) = dode.get_dar(dta, true_car_f, true_truck_f)

noise_level = 0.1
# true time-dependent link flow
true_car_x = true_dar_car.dot(true_car_f)
true_truck_x = true_dar_truck.dot(true_truck_f)

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
    #                          np.arange(0, dode.num_loading_interval, dode.ass_freq) + dode.ass_freq).flatten(order = 'F')
    true_car_tt = dta.get_car_link_tt(np.arange(0, dode.num_loading_interval, dode.ass_freq)).flatten(order = 'F')
    true_truck_tt = dta.get_truck_link_tt(np.arange(0, dode.num_loading_interval, dode.ass_freq)).flatten(order = 'F')
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


# %%
# SPSA
# dode = mcSPSA(nb, config)
# Computational graph
dode = MCDODE(nb, config)


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
(car_flow, truck_flow, l_list) = dode.estimate_path_flow_pytorch(max_epoch = 100, car_step_size = 0.1, 
                                                                truck_step_size = 0.01, car_init_scale = 100, 
                                                                truck_init_scale = 10)
print(r2_score(car_flow, true_car_f), r2_score(truck_flow, true_truck_f))


# %%



