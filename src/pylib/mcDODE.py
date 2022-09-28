import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import hashlib
import time
import shutil
from scipy.sparse import coo_matrix, csr_matrix
import pickle
import multiprocessing as mp
from typing import Union
import torch
import torch.nn as nn
from sklearn.metrics import r2_score

import MNMAPI

def tensor_to_numpy(x: torch.Tensor):
    return x.data.cpu().numpy()

class torch_pathflow_solver(nn.Module):
    def __init__(self, num_assign_interval, num_path,
               car_scale=1, truck_scale=0.1, use_file_as_init=None):
        super(torch_pathflow_solver, self).__init__()

        self.num_assign_interval = num_assign_interval

        self.car_scale = car_scale
        self.truck_scale = truck_scale 

        self.num_path = num_path

        self.initialize(use_file_as_init)

        self.params = None

        self.algo_dict = {
            "SGD": torch.optim.SGD,
            "NAdam": torch.optim.NAdam,
            "Adam": torch.optim.Adam,
            "Adamax": torch.optim.Adamax,
            "AdamW": torch.optim.AdamW,
            "RAdam": torch.optim.RAdam,
            "Adagrad": torch.optim.Adagrad,
            "Adadelta": torch.optim.Adadelta
        }
        self.optimizer = None
        self.scheduler = None
    
    def init_tensor(self, x: torch.Tensor):
        # return torch.abs(nn.init.xavier_uniform_(x))
        return nn.init.xavier_uniform_(x)
        # return nn.init.uniform_(x, 0, 1)

    def initialize(self, use_file_as_init=None):
        log_f_car, log_f_truck = None, None

        if use_file_as_init is not None:
            # log f numpy
            _, _, _, _, log_f_car, log_f_truck, \
                _, _, _, _, _ = pickle.load(open(use_file_as_init, 'rb'))

        if log_f_car is None:
            self.log_f_car = nn.Parameter(self.init_tensor(torch.Tensor(self.num_assign_interval * self.num_path, 1)).squeeze(), requires_grad=True)
        else:
            assert(np.prod(log_f_car.shape) == self.num_assign_interval * self.num_path)
            self.log_f_car = nn.Parameter(torch.from_numpy(log_f_car).squeeze(), requires_grad=True)
        
        if log_f_truck is None:
            self.log_f_truck = nn.Parameter(self.init_tensor(torch.Tensor(self.num_assign_interval * self.num_path, 1)).squeeze(), requires_grad=True)
        else:
            assert(np.prod(log_f_truck.shape) == self.num_assign_interval * self.num_path)
            self.log_f_truck = nn.Parameter(torch.from_numpy(log_f_truck).squeeze(), requires_grad=True)

    def get_log_f_tensor(self):
        return self.log_f_car, self.log_f_truck

    def get_log_f_numpy(self):
        return tensor_to_numpy(self.log_f_car).flatten(), tensor_to_numpy(self.log_f_truck).flatten()

    def add_pathflow(self, num_path_add: int):
        self.num_path += num_path_add

        if num_path_add > 0:
            self.log_f_car = nn.Parameter(self.log_f_car.reshape(self.num_assign_interval, -1))
            log_f_car_add = nn.Parameter(self.init_tensor(torch.Tensor(self.num_assign_interval * num_path_add, 1)).squeeze().reshape(self.num_assign_interval, -1), requires_grad=True)
            self.log_f_car = nn.Parameter(torch.cat([self.log_f_car, log_f_car_add], dim=1))
            assert(self.log_f_car.shape[1] == self.num_path)
            self.log_f_car = nn.Parameter(self.log_f_car.reshape(-1))

            self.log_f_truck = nn.Parameter(self.log_f_truck_driving.reshape(self.num_assign_interval, -1))
            log_f_truck_add = nn.Parameter(self.init_tensor(torch.Tensor(self.num_assign_interval * num_path_add, 1)).squeeze().reshape(self.num_assign_interval, -1), requires_grad=True)
            self.log_f_truck = nn.Parameter(torch.cat([self.log_f_truck, log_f_truck_add], dim=1))
            assert(self.log_f_truck.shape[1] == self.num_path)
            self.log_f_truck = nn.Parameter(self.log_f_truck.reshape(-1))

    def generate_pathflow_tensor(self):
        # exp
        f_car = torch.exp(self.log_f_car * self.car_scale)
        f_truck = torch.exp(self.log_f_truck * self.truck_scale)

        f_car = torch.clamp(f_car, max=5e3)
        f_truck = torch.clamp(f_truck, max=5e3)

        # relu
        # f_car = torch.clamp(self.log_f_car * self.car_scale, min=1e-6)
        # f_truck = torch.clamp(self.log_f_truck * self.truck_scale, min=1e-6)

        return f_car, f_truck

    def generate_pathflow_numpy(self, f_car: Union[torch.Tensor, None] = None, f_truck: Union[torch.Tensor, None] = None):
        if (f_car is None) and (f_truck is None):
            f_car, f_truck = self.generate_pathflow_tensor()
       
        return tensor_to_numpy(f_car).flatten(), tensor_to_numpy(f_truck).flatten()

    def set_params_with_lr(self, car_step_size=1e-2, truck_step_size=1e-3):
        self.params = [
            {'params': self.log_f_car, 'lr': car_step_size},
            {'params': self.log_f_truck, 'lr': truck_step_size}
        ]
    
    def set_optimizer(self, algo='NAdam'):
        self.optimizer = self.algo_dict[algo](self.params)

    def compute_gradient(self, f_car: torch.Tensor, f_truck: torch.Tensor, 
                         f_car_grad: np.ndarray, f_truck_grad: np.ndarray, l2_coeff: float = 1e-5):

        f_car.backward(torch.from_numpy(f_car_grad) + l2_coeff * f_car.data)
        f_truck.backward(torch.from_numpy(f_truck_grad) + l2_coeff * f_truck.data)

        # torch.nn.utils.clip_grad_value_(self.parameters(), 0.4)
    
    def set_scheduler(self):
        self.scheduler = torch.optim.lr_scheduler.StepLR(self.optimizer, step_size=50, gamma=0.5)
        # self.scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        #     self.optimizer, mode='min', factor=0.75, patience=5, 
        #     threshold=0.15, threshold_mode='rel', cooldown=0, min_lr=0, eps=1e-08, verbose=False)


class MCDODE():
    def __init__(self, nb, config):
        self.config = config
        self.nb = nb
        self.num_assign_interval = nb.config.config_dict['DTA']['max_interval']
        self.ass_freq = nb.config.config_dict['DTA']['assign_frq']
        self.num_link = nb.config.config_dict['DTA']['num_of_link']
        self.num_path = nb.config.config_dict['FIXED']['num_path']
        # if nb.config.config_dict['DTA']['total_interval'] > 0 and nb.config.config_dict['DTA']['total_interval'] > self.num_assign_interval * self.ass_freq:
        #   self.num_loading_interval = nb.config.config_dict['DTA']['total_interval']
        # else:
        #   self.num_loading_interval = self.num_assign_interval * self.ass_freq  # not long enough
        self.num_loading_interval = self.num_assign_interval * self.ass_freq
        self.data_dict = dict()
        self.num_data = self.config['num_data']
        self.observed_links = self.config['observed_links']
        self.paths_list = self.config['paths_list']
        self.car_count_agg_L_list = None
        self.truck_count_agg_L_list = None
        assert (len(self.paths_list) == self.num_path)

    def check_registered_links_covered_by_registered_paths(self, folder, add=False):
        self.save_simulation_input_files(folder)

        a = MNMAPI.mcdta_api()
        a.initialize(folder)

        a.register_links(self.observed_links)

        a.register_paths(self.paths_list)

        is_driving_link_covered = a.are_registered_links_in_registered_paths()

        is_updated = 0
        if add:
            if np.any(is_driving_link_covered == False):
                is_driving_link_covered = a.generate_paths_to_cover_registered_links()
                is_updated = is_driving_link_covered[0]
                is_driving_link_covered = is_driving_link_covered[1:]
            else:
                is_updated = 0
        return is_updated, is_driving_link_covered

    def _add_car_link_flow_data(self, link_flow_df_list):
        # assert (self.config['use_car_link_flow'])
        assert (self.num_data == len(link_flow_df_list))
        self.data_dict['car_link_flow'] = link_flow_df_list

    def _add_truck_link_flow_data(self, link_flow_df_list):
        # assert (self.config['use_truck_link_flow'])
        assert (self.num_data == len(link_flow_df_list))
        self.data_dict['truck_link_flow'] = link_flow_df_list

    def _add_car_link_tt_data(self, link_spd_df_list):
        # assert (self.config['use_car_link_tt'])
        assert (self.num_data == len(link_spd_df_list))
        self.data_dict['car_link_tt'] = link_spd_df_list

    def _add_truck_link_tt_data(self, link_spd_df_list):
        # assert (self.config['use_truck_link_tt'])
        assert (self.num_data == len(link_spd_df_list))
        self.data_dict['truck_link_tt'] = link_spd_df_list

    def _add_origin_vehicle_registration_data(self, origin_vehicle_registration_data_list):
        # assert (self.config['use_truck_link_tt'])
        assert (self.num_data == len(origin_vehicle_registration_data_list))
        self.data_dict['origin_vehicle_registration_data'] = origin_vehicle_registration_data_list

    def add_data(self, data_dict):
        if self.config['car_count_agg']:
            self.car_count_agg_L_list = data_dict['car_count_agg_L_list']
        if self.config['truck_count_agg']:
            self.truck_count_agg_L_list = data_dict['truck_count_agg_L_list']
        if self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss']:
            self._add_car_link_flow_data(data_dict['car_link_flow'])
        if self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss'] :
            self._add_truck_link_flow_data(data_dict['truck_link_flow'])
        if self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss']:
            self._add_car_link_tt_data(data_dict['car_link_tt'])
        if self.config['use_truck_link_tt']or self.config['compute_car_link_tt_loss']:
            self._add_truck_link_tt_data(data_dict['truck_link_tt'])
        if self.config['use_origin_vehicle_registration_data'] or self.config['compute_origin_vehicle_registration_loss']:
            self._add_origin_vehicle_registration_data(data_dict['origin_vehicle_registration_data'])

        if 'mask_driving_link' in data_dict:
            self.data_dict['mask_driving_link'] = np.tile(data_dict['mask_driving_link'], self.num_assign_interval)
        else:
            self.data_dict['mask_driving_link'] = np.ones(len(self.observed_links) * self.num_assign_interval, dtype=bool)

    def save_simulation_input_files(self, folder_path, f_car=None, f_truck=None):

        if not os.path.exists(folder_path):
            os.mkdir(folder_path)

        # modify demand based on input path flows
        if (f_car is not None) and (f_truck is not None):
            self.nb.update_demand_path2(f_car, f_truck)
   
        # self.nb.config.config_dict['DTA']['flow_scalar'] = 3
        if self.config['use_car_link_tt'] or self.config['use_truck_link_tt']:
            self.nb.config.config_dict['DTA']['total_interval'] = self.num_loading_interval # * 2  # hopefully this is sufficient 
        else:
            self.nb.config.config_dict['DTA']['total_interval'] = self.num_loading_interval  # if only count data is used

        self.nb.config.config_dict['DTA']['routing_type'] = 'Biclass_Hybrid'

        # no output files saved from DNL
        self.nb.config.config_dict['STAT']['rec_volume'] = 1
        self.nb.config.config_dict['STAT']['volume_load_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['volume_record_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['rec_tt'] = 1
        self.nb.config.config_dict['STAT']['tt_load_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['tt_record_automatic_rec'] = 0
        # save modified files in new_folder
        self.nb.dump_to_folder(folder_path)

    def _run_simulation(self, f_car, f_truck, counter=0, show_loading=False):
        # create a new_folder with a unique name
        hash1 = hashlib.sha1()
        # python 2
        # hash1.update(str(time.time()) + str(counter))
        # python 3
        hash1.update((str(time.time()) + str(counter)).encode('utf-8'))

        new_folder = str(hash1.hexdigest())

        self.save_simulation_input_files(new_folder, f_car, f_truck)

        # invoke MNMAPI
        a = MNMAPI.mcdta_api()
        # read all files in new_folder
        a.initialize(new_folder)
        
        # register links and paths
        a.register_links(self.observed_links)
        a.register_paths(self.paths_list)
        # install cc and cc_tree on registered links
        a.install_cc()
        a.install_cc_tree()
        # run DNL
        a.run_whole(show_loading)
        # print("Finish simulation", time.time())

        travel_stats = a.get_travel_stats()
        assert(len(travel_stats) == 4)
        print("\n************ travel stats ************")
        print("car count: {}".format(travel_stats[0]))
        print("truck count: {}".format(travel_stats[1]))
        print("car total travel time (hours): {}".format(travel_stats[2]))
        print("truck total travel time (hours): {}".format(travel_stats[3]))
        print("************ travel stats ************\n")

        # print_emission_stats() only works if folder is not removed, cannot find reason
        a.print_emission_stats()

        # delete new_folder and all files and subdirectories below it.
        shutil.rmtree(new_folder)

        return a

    def get_dar(self, dta, f_car, f_truck):
        car_dar = csr_matrix((self.num_assign_interval * len(self.observed_links), self.num_assign_interval * len(self.paths_list)))
        truck_dar = csr_matrix((self.num_assign_interval * len(self.observed_links), self.num_assign_interval * len(self.paths_list)))
        if self.config['use_car_link_flow'] or self.config['use_car_link_tt']:
            # (num_assign_timesteps x num_links x num_path x num_assign_timesteps) x 5
            raw_car_dar = dta.get_car_dar_matrix(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            # print("raw car dar", raw_car_dar)
            # num_assign_interval * num_e_link, num_assign_interval * num_e_path
            car_dar = self._massage_raw_dar(raw_car_dar, self.ass_freq, f_car, self.num_assign_interval)
        if self.config['use_truck_link_flow'] or self.config['use_truck_link_tt']:
            # (num_assign_timesteps x num_links x num_path x num_assign_timesteps) x 5
            raw_truck_dar = dta.get_truck_dar_matrix(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            # num_assign_interval * num_e_link, num_assign_interval * num_e_path
            truck_dar = self._massage_raw_dar(raw_truck_dar, self.ass_freq, f_truck, self.num_assign_interval)
        # print("dar", car_dar, truck_dar)
        return (car_dar, truck_dar)

    def _massage_raw_dar(self, raw_dar, ass_freq, f, num_assign_interval):
        num_e_path = len(self.paths_list)
        num_e_link = len(self.observed_links)
        # 15 min
        small_assign_freq = ass_freq * self.nb.config.config_dict['DTA']['unit_time'] / 60

        raw_dar = raw_dar[(raw_dar[:, 1] < self.num_assign_interval * small_assign_freq) & (raw_dar[:, 3] < self.num_loading_interval), :]

        # raw_dar[:, 2]: link no.
        # raw_dar[:, 3]: the count of unit time interval (5s)
        # In Python 3, map() returns an iterable while, in Python 2, it returns a list.
        link_seq = (np.array(list(map(lambda x: self.observed_links.index(x), raw_dar[:, 2].astype(int))))
                    + (raw_dar[:, 3] / ass_freq).astype(int) * num_e_link).astype(int)
        # raw_dar[:, 0]: path no.
        # raw_dar[:, 1]: the count of 1 min interval
        path_seq = (raw_dar[:, 0].astype(int) + (raw_dar[:, 1] / small_assign_freq).astype(int) * num_e_path).astype(int)
        # print(path_seq)
        # raw_dar[:, 4]: flow
        p = raw_dar[:, 4] / f[path_seq]
        # print("Creating the coo matrix", time.time())
        mat = coo_matrix((p, (link_seq, path_seq)), 
                        shape=(num_assign_interval * num_e_link, num_assign_interval * num_e_path))
        # pickle.dump((p, link_seq, path_seq), open('test.pickle', 'wb'))
        # print('converting the csr', time.time())
        mat = mat.tocsr()
        # print('finish converting', time.time())
        return mat    

    def init_path_flow(self, car_scale=1, truck_scale=0.1):
        f_car = np.random.rand(self.num_assign_interval * self.num_path) * car_scale
        f_truck = np.random.rand(self.num_assign_interval * self.num_path) * truck_scale
        return f_car, f_truck

    def compute_path_flow_grad_and_loss(self, one_data_dict, f_car, f_truck, counter=0):
        # print("Running simulation", time.time())
        dta = self._run_simulation(f_car, f_truck, counter, show_loading=True)
        # print("Getting DAR", time.time())
        (car_dar, truck_dar) = self.get_dar(dta, f_car, f_truck)

        # print("Evaluating grad", time.time())
        car_grad = np.zeros(len(self.observed_links) * self.num_assign_interval)
        truck_grad = np.zeros(len(self.observed_links) * self.num_assign_interval)

        x_e_car, x_e_truck, tt_e_car, tt_e_truck, O_demand_est = None, None, None, None, None

        if self.config['use_car_link_flow']:
            # print("car link flow", time.time())
            grad, x_e_car = self._compute_grad_on_car_link_flow(dta, one_data_dict)
            car_grad += self.config['link_car_flow_weight'] * grad
        if self.config['use_truck_link_flow']:
            grad, x_e_truck = self._compute_grad_on_truck_link_flow(dta, one_data_dict)
            truck_grad += self.config['link_truck_flow_weight'] * grad
        if self.config['use_car_link_tt']:
            grad, tt_e_car = self._compute_grad_on_car_link_tt(dta, one_data_dict)
            car_grad += self.config['link_car_tt_weight'] * grad
        if self.config['use_truck_link_tt']:
            grad, tt_e_truck = self._compute_grad_on_truck_link_tt(dta, one_data_dict)
            truck_grad += self.config['link_truck_tt_weight'] * grad

        f_car_grad = car_dar.T.dot(car_grad)
        f_truck_grad = truck_dar.T.dot(truck_grad)

        if self.config['use_origin_vehicle_registration_data']:
            f_car_grad_add, f_truck_grad_add, O_demand_est = self._compute_grad_on_origin_vehicle_registration_data(one_data_dict, f_car, f_truck)
            f_car_grad += self.config['origin_vehicle_registration_weight'] * f_car_grad_add
            f_truck_grad += self.config['origin_vehicle_registration_weight'] * f_truck_grad_add

        # print("Getting Loss", time.time())
        total_loss, loss_dict = self._get_loss(one_data_dict, dta, f_car, f_truck)
        return f_car_grad, f_truck_grad, total_loss, loss_dict, x_e_car, x_e_truck, tt_e_car, tt_e_truck, O_demand_est

    def _compute_grad_on_car_link_flow(self, dta, one_data_dict):
        link_flow_array = one_data_dict['car_link_flow']
        x_e = dta.get_link_car_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                      np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        # print("x_e", x_e, link_flow_array)
        if self.config['car_count_agg']:
            x_e = one_data_dict['car_count_agg_L'].dot(x_e)
        discrepancy = np.nan_to_num(link_flow_array - x_e)
        grad = - discrepancy
        if self.config['car_count_agg']:
            grad = one_data_dict['car_count_agg_L'].T.dot(grad)
        # print("final link grad", grad)
        # assert(np.all(~np.isnan(grad)))
        return grad, x_e
  
    def _compute_grad_on_truck_link_flow(self, dta, one_data_dict):
        link_flow_array = one_data_dict['truck_link_flow']
        x_e = dta.get_link_truck_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                        np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        if self.config['truck_count_agg']:
            x_e = one_data_dict['truck_count_agg_L'].dot(x_e)
        discrepancy = np.nan_to_num(link_flow_array - x_e)
        grad = - discrepancy
        if self.config['truck_count_agg']:
            grad = one_data_dict['truck_count_agg_L'].T.dot(grad)
        # assert(np.all(~np.isnan(grad)))
        return grad, x_e

    def _compute_grad_on_car_link_tt(self, dta, one_data_dict):
        # tt_e = dta.get_car_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
        #                                   np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq, self.ass_freq, False).flatten(order='F')
        tt_e = dta.get_car_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq), False).flatten(order='F')
        # tt_free = np.tile(list(map(lambda x: self.nb.get_link(x).get_car_fft(), self.observed_links)), (self.num_assign_interval))
        tt_free = np.tile(dta.get_car_link_fftt(self.observed_links), (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['car_link_tt'], tt_free) 
        # tt_o = one_data_dict['car_link_tt']
        # print('o-----', tt_o)
        # print('e-----', tt_e)
        discrepancy = np.nan_to_num(tt_o - tt_e)
        grad = - discrepancy
        # print('g-----', grad)
        # if self.config['car_count_agg']:
        #   grad = one_data_dict['car_count_agg_L'].T.dot(grad)
        # print(tt_e, tt_o)
        # print("car_grad", grad)
        # assert(np.all(~np.isnan(grad)))
        return grad, tt_e

    def _compute_grad_on_truck_link_tt(self, dta, one_data_dict):
        # tt_e = dta.get_truck_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
        #                                     np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq, self.ass_freq, False).flatten(order='F')
        tt_e = dta.get_truck_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq), False).flatten(order='F')
        # tt_free = np.tile(list(map(lambda x: self.nb.get_link(x).get_truck_fft(), self.observed_links)), (self.num_assign_interval))
        tt_free = np.tile(dta.get_truck_link_fftt(self.observed_links), (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['truck_link_tt'], tt_free)
        discrepancy = np.nan_to_num(tt_o - tt_e)
        grad = - discrepancy
        # if self.config['truck_count_agg']:
        #   grad = one_data_dict['truck_count_agg_L'].T.dot(grad)
        # print("truck_grad", grad)
        # assert(np.all(~np.isnan(grad)))
        return grad, tt_e

    def _compute_grad_on_origin_vehicle_registration_data(self, one_data_dict, f_car, f_truck):
        # reshape car_flow and truck_flow into ndarrays with dimensions of intervals x number of total paths
        # loss in terms of total counts of cars and trucks for some origin nodes
        f_car = torch.from_numpy(f_car)
        f_truck = torch.from_numpy(f_truck)
        f_car.requires_grad = True
        f_truck.requires_grad = True

        f_car_reshaped = f_car.reshape(self.num_assign_interval, -1)
        f_truck_reshaped = f_truck.reshape(self.num_assign_interval, -1)

        O_demand_est = dict()
        for i, path_ID in enumerate(self.nb.path_table.ID2path.keys()):
            path = self.nb.path_table.ID2path[path_ID]
            O_node = path.origin_node
            O = self.nb.od.O_dict.inv[O_node]
            if O not in O_demand_est:
                O_demand_est[O] = [torch.Tensor([0.]), torch.Tensor([0.])]
            O_demand_est[O][0] = O_demand_est[O][0] + f_car_reshaped[:, i].sum()
            O_demand_est[O][1] = O_demand_est[O][1] + f_truck_reshaped[:, i].sum()

        # pandas DataFrame
        def process_one_row(row):
            loss = (sum(O_demand_est[Origin_ID][0] for Origin_ID in row['origin_ID']) - row['car'])**2 + \
                   (sum(O_demand_est[Origin_ID][1] for Origin_ID in row['origin_ID']) - row['truck'])**2
            loss.backward()
                
        one_data_dict['origin_vehicle_registration_data'].apply(lambda row: process_one_row(row), axis=1)

        O_demand_est = {O: (O_demand_est[O][0].item(), O_demand_est[O][1].item()) for O in O_demand_est}
        return f_car.grad.data.cpu().numpy(), f_truck.grad.data.cpu().numpy(), O_demand_est

    def _get_one_data(self, j):
        assert (self.num_data > j)
        one_data_dict = dict()
        if self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss']:
            one_data_dict['car_link_flow'] = self.data_dict['car_link_flow'][j]
        if self.config['use_truck_link_flow']or self.config['compute_truck_link_flow_loss']:
            one_data_dict['truck_link_flow'] = self.data_dict['truck_link_flow'][j]
        if self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss']:
            one_data_dict['car_link_tt'] = self.data_dict['car_link_tt'][j]
        if self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss']:
            one_data_dict['truck_link_tt'] = self.data_dict['truck_link_tt'][j]
        if self.config['car_count_agg']:
            one_data_dict['car_count_agg_L'] = self.car_count_agg_L_list[j]
        if self.config['truck_count_agg']:
            one_data_dict['truck_count_agg_L'] = self.truck_count_agg_L_list[j]
        if self.config['use_origin_vehicle_registration_data'] or self.config['compute_origin_vehicle_registration_loss']:
            # pandas DataFrame
            one_data_dict['origin_vehicle_registration_data'] = self.data_dict['origin_vehicle_registration_data'][j]

        if 'mask_driving_link' in self.data_dict:
            one_data_dict['mask_driving_link'] = self.data_dict['mask_driving_link']
        else:
            one_data_dict['mask_driving_link'] = np.ones(len(self.observed_links) * self.num_assign_interval, dtype=bool)
        return one_data_dict

    def aggregate_f(self, f_car, f_truck):
        # reshape car_flow and truck_flow into ndarrays with dimensions of intervals x number of total paths
        f_car = f_car.reshape(self.num_assign_interval, -1)
        f_truck = f_truck.reshape(self.num_assign_interval, -1)

        O_demand_est = dict()
        for i, path_ID in enumerate(self.nb.path_table.ID2path.keys()):
            path = self.nb.path_table.ID2path[path_ID]
            O_node = path.origin_node
            O = self.nb.od.O_dict.inv[O_node]
            if O not in O_demand_est:
                O_demand_est[O] = [0, 0]
            O_demand_est[O][0] = O_demand_est[O][0] + f_car[:, i].sum()
            O_demand_est[O][1] = O_demand_est[O][1] + f_truck[:, i].sum()
        f_car = f_car.flatten()
        f_truck = f_truck.flatten()
        return O_demand_est

    def _get_loss(self, one_data_dict, dta, f_car, f_truck):
        loss_dict = dict()
        assign_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)

        if self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss']:
            x_e = dta.get_link_car_inflow(assign_intervals, assign_intervals + self.ass_freq).flatten(order='F')
            if self.config['car_count_agg']:
                x_e = one_data_dict['car_count_agg_L'].dot(x_e)
            loss = self.config['link_car_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e[one_data_dict['mask_driving_link']] - one_data_dict['car_link_flow'][one_data_dict['mask_driving_link']]))
            # loss = np.linalg.norm(np.nan_to_num(x_e[one_data_dict['mask_driving_link']] - one_data_dict['car_link_flow'][one_data_dict['mask_driving_link']]))
            loss_dict['car_count_loss'] = loss

        if self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss']:
            x_e = dta.get_link_truck_inflow(assign_intervals, assign_intervals + self.ass_freq).flatten(order='F')
            if self.config['truck_count_agg']:
                x_e = one_data_dict['truck_count_agg_L'].dot(x_e)
            loss = self.config['link_truck_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e[one_data_dict['mask_driving_link']] - one_data_dict['truck_link_flow'][one_data_dict['mask_driving_link']]))
            # loss = np.linalg.norm(np.nan_to_num(x_e[one_data_dict['mask_driving_link']] - one_data_dict['truck_link_flow'][one_data_dict['mask_driving_link']]))
            loss_dict['truck_count_loss'] = loss

        if self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss']:
            x_tt_e = dta.get_car_link_tt(assign_intervals, False).flatten(order='F')
            # x_tt_e = dta.get_car_link_tt_robust(assign_intervals, assign_intervals + self.ass_freq, self.ass_freq, False).flatten(order='F')
            loss = self.config['link_car_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e[one_data_dict['mask_driving_link']] - one_data_dict['car_link_tt'][one_data_dict['mask_driving_link']]))
            # loss = np.linalg.norm(np.nan_to_num(x_tt_e[one_data_dict['mask_driving_link']] - one_data_dict['car_link_tt'][one_data_dict['mask_driving_link']]))
            loss_dict['car_tt_loss'] = loss

        if self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss']:
            x_tt_e = dta.get_truck_link_tt(assign_intervals, False).flatten(order='F')
            # x_tt_e = dta.get_truck_link_tt_robust(assign_intervals, assign_intervals + self.ass_freq, self.ass_freq, False).flatten(order='F')
            loss = self.config['link_truck_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e[one_data_dict['mask_driving_link']] - one_data_dict['truck_link_tt'][one_data_dict['mask_driving_link']]))
            # loss = np.linalg.norm(np.nan_to_num(x_tt_e[one_data_dict['mask_driving_link']] - one_data_dict['truck_link_tt'][one_data_dict['mask_driving_link']]))
            loss_dict['truck_tt_loss'] = loss

        if self.config['use_origin_vehicle_registration_data'] or self.config['compute_origin_vehicle_registration_loss']:
            O_demand_est = self.aggregate_f(f_car, f_truck)
            def process_one_row(row):
                return (sum(O_demand_est[Origin_ID][0] for Origin_ID in row['origin_ID']) - row['car'])**2 + \
                       (sum(O_demand_est[Origin_ID][1] for Origin_ID in row['origin_ID']) - row['truck'])**2
            loss = np.sqrt(np.nansum(one_data_dict['origin_vehicle_registration_data'].apply(lambda row: process_one_row(row), axis=1)))
            loss_dict['origin_vehicle_registration_loss'] = self.config['origin_vehicle_registration_weight'] * loss
            # loss_dict['origin_vehicle_registration_loss'] = loss

        total_loss = 0.0
        for loss_type, loss_value in loss_dict.items():
            total_loss += loss_value
        return total_loss, loss_dict

    def estimate_path_flow_pytorch(self, car_step_size=0.1, truck_step_size=0.1, 
                                    link_car_flow_weight=1, link_truck_flow_weight=1, 
                                    link_car_tt_weight=1, link_truck_tt_weight=1, origin_vehicle_registration_weight=1,
                                    max_epoch=10, algo='NAdam', normalized_by_scale = True,
                                    car_init_scale=10,
                                    truck_init_scale=1, 
                                    store_folder=None, 
                                    use_file_as_init=None,
                                    starting_epoch=0):
    
        if np.isscalar(link_car_flow_weight):
            link_car_flow_weight = np.ones(max_epoch, dtype=bool) * link_car_flow_weight
        assert(len(link_car_flow_weight) == max_epoch)

        if np.isscalar(link_truck_flow_weight):
            link_truck_flow_weight = np.ones(max_epoch, dtype=bool) * link_truck_flow_weight
        assert(len(link_truck_flow_weight) == max_epoch)

        if np.isscalar(link_car_tt_weight):
            link_car_tt_weight = np.ones(max_epoch, dtype=bool) * link_car_tt_weight
        assert(len(link_car_tt_weight) == max_epoch)

        if np.isscalar(link_truck_tt_weight):
            link_truck_tt_weight = np.ones(max_epoch, dtype=bool) * link_truck_tt_weight
        assert(len(link_truck_tt_weight) == max_epoch)

        if np.isscalar(origin_vehicle_registration_weight):
            origin_vehicle_registration_weight = np.ones(max_epoch, dtype=bool) * origin_vehicle_registration_weight
        assert(len(origin_vehicle_registration_weight) == max_epoch)

        loss_list = list()
        best_epoch = starting_epoch
        best_f_car_driving, best_f_truck_driving = 0, 0
        best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck, best_O_demand = 0, 0, 0, 0, 0
        # here the basic variables to be estimated are path flows, not OD demand, so no route choice model, unlike in sDODE.py
        if use_file_as_init is None:
            (f_car, f_truck) = self.init_path_flow(car_scale=car_init_scale, truck_scale=truck_init_scale)
        else:
            # most recent
            _, _, loss_list, best_epoch, _, _, \
                _, _, _, _, _ = pickle.load(open(use_file_as_init, 'rb'))
            # best 
            use_file_as_init = os.path.join(store_folder, '{}_iteration.pickle'.format(best_epoch))
            loss, loss_dict, loss_list, best_epoch, best_f_car, best_f_truck, \
                best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck, best_O_demand = pickle.load(open(use_file_as_init, 'rb'))
            
            f_car, f_truck = best_f_car, best_f_truck

        if normalized_by_scale:
            f_car_tensor = torch.from_numpy(f_car / np.maximum(car_init_scale, 1e-6))
            f_truck_tensor = torch.from_numpy(f_truck / np.maximum(truck_init_scale, 1e-6))
        else:
            f_car_tensor = torch.from_numpy(f_car)
            f_truck_tensor = torch.from_numpy(f_truck)

        f_car_tensor.requires_grad = True
        f_truck_tensor.requires_grad = True

        params = [
            {'params': f_car_tensor, 'lr': car_step_size},
            {'params': f_truck_tensor, 'lr': truck_step_size}
        ]

        algo_dict = {
            "SGD": torch.optim.SGD,
            "NAdam": torch.optim.NAdam,
            "Adam": torch.optim.Adam,
            "Adamax": torch.optim.Adamax,
            "AdamW": torch.optim.AdamW,
            "RAdam": torch.optim.RAdam,
            "Adagrad": torch.optim.Adagrad,
            "Adadelta": torch.optim.Adadelta
        }
        optimizer = algo_dict[algo](params)
        
        for i in range(max_epoch):
      
            seq = np.random.permutation(self.num_data)
            loss = np.float(0)
            # print("Start iteration", time.time())
            loss_dict = {'car_count_loss': 0.0, 'truck_count_loss': 0.0, 'car_tt_loss': 0.0, 'truck_tt_loss': 0.0, 'origin_vehicle_registration_loss': 0.0}

            self.config['link_car_flow_weight'] = link_car_flow_weight[i] * (self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss'])
            self.config['link_truck_flow_weight'] = link_truck_flow_weight[i] * (self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss'])
            
            self.config['link_car_tt_weight'] = link_car_tt_weight[i] * (self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss'])
            self.config['link_truck_tt_weight'] = link_truck_tt_weight[i] * (self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss'])

            self.config['origin_vehicle_registration_weight'] = origin_vehicle_registration_weight[i] * (self.config['use_origin_vehicle_registration_data'] or self.config['compute_origin_vehicle_registration_loss'])

            for j in seq:
                one_data_dict = self._get_one_data(j)
                car_grad, truck_grad, tmp_loss, tmp_loss_dict, x_e_car, x_e_truck, tt_e_car, tt_e_truck, O_demand = self.compute_path_flow_grad_and_loss(one_data_dict, f_car, f_truck)
                # print("gradient", car_grad, truck_grad)

                optimizer.zero_grad()

                if normalized_by_scale:
                    f_car_tensor.grad = torch.from_numpy(car_grad * car_init_scale)
                    f_truck_tensor.grad = torch.from_numpy(truck_grad * truck_init_scale)
                else:
                    f_car_tensor.grad = torch.from_numpy(car_grad)
                    f_truck_tensor.grad = torch.from_numpy(truck_grad)

                optimizer.step()

                car_grad, truck_grad = 0, 0
                optimizer.zero_grad()

                if normalized_by_scale:
                    f_car = f_car_tensor.data.cpu().numpy() * car_init_scale
                    f_truck = f_truck_tensor.data.cpu().numpy() * truck_init_scale
                else:
                    f_car = f_car_tensor.data.cpu().numpy()
                    f_truck = f_truck_tensor.data.cpu().numpy()
            
                f_car = np.maximum(f_car, 1e-6)
                f_truck = np.maximum(f_truck, 1e-6)

                loss += tmp_loss / np.float(self.num_data)
                for loss_type, loss_value in tmp_loss_dict.items():
                    loss_dict[loss_type] += loss_value / np.float(self.num_data)

            print("Epoch:", starting_epoch + i, "Loss:", loss, self.print_separate_accuracy(loss_dict))
            loss_list.append([loss, loss_dict])
                    
            if (best_epoch == 0) or (loss_list[best_epoch][0] > loss_list[-1][0]):
                best_epoch = starting_epoch + i
                best_f_car_driving, best_f_truck_driving = f_car, f_truck
                best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck, best_O_demand = x_e_car, x_e_truck, tt_e_car, tt_e_truck, O_demand

                if store_folder is not None:
                    self.save_simulation_input_files(os.path.join(store_folder, 'input_files_estimate_path_flow'), 
                                                     best_f_car_driving, best_f_truck_driving)
            
            # print(f_car, f_truck)
            # break
            if store_folder is not None:
                pickle.dump((loss, loss_dict, loss_list, best_epoch, f_car, f_truck,
                                x_e_car, x_e_truck, tt_e_car, tt_e_truck, O_demand), 
                                open(os.path.join(store_folder, str(starting_epoch + i) + '_iteration.pickle'), 'wb'))

        print("Best loss at Epoch:", best_epoch, "Loss:", loss_list[best_epoch][0], self.print_separate_accuracy(loss_list[best_epoch][1]))
        return best_f_car_driving, best_f_truck_driving, best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck, best_O_demand, loss_list

    def estimate_path_flow(self, car_step_size=0.1, truck_step_size=0.1, 
                            link_car_flow_weight=1, link_truck_flow_weight=1, 
                            link_car_tt_weight=1, link_truck_tt_weight=1, origin_vehicle_registration_weight=1e-6,
                            max_epoch=10, car_init_scale=10, truck_init_scale=1, store_folder=None, use_file_as_init=None, 
                            adagrad=False, starting_epoch=0):

        if np.isscalar(link_car_flow_weight):
            link_car_flow_weight = np.ones(max_epoch, dtype=bool) * link_car_flow_weight
        assert(len(link_car_flow_weight) == max_epoch)

        if np.isscalar(link_truck_flow_weight):
            link_truck_flow_weight = np.ones(max_epoch, dtype=bool) * link_truck_flow_weight
        assert(len(link_truck_flow_weight) == max_epoch)

        if np.isscalar(link_car_tt_weight):
            link_car_tt_weight = np.ones(max_epoch, dtype=bool) * link_car_tt_weight
        assert(len(link_car_tt_weight) == max_epoch)

        if np.isscalar(link_truck_tt_weight):
            link_truck_tt_weight = np.ones(max_epoch, dtype=bool) * link_truck_tt_weight
        assert(len(link_truck_tt_weight) == max_epoch)

        if np.isscalar(origin_vehicle_registration_weight):
            origin_vehicle_registration_weight = np.ones(max_epoch, dtype=bool) * origin_vehicle_registration_weight
        assert(len(origin_vehicle_registration_weight) == max_epoch)
    
        # here the basic variables to be estimated are path flows, not OD demand, so no route choice model, unlike in sDODE.py
        loss_list = list()
        best_epoch = starting_epoch
        best_f_car_driving, best_f_truck_driving = 0, 0
        best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck, best_O_demand = 0, 0, 0, 0, 0
        # here the basic variables to be estimated are path flows, not OD demand, so no route choice model, unlike in sDODE.py
        if use_file_as_init is None:
            (f_car, f_truck) = self.init_path_flow(car_scale=car_init_scale, truck_scale=truck_init_scale)
        else:
            # most recent
            _, _, loss_list, best_epoch, _, _, \
                _, _, _, _, _ = pickle.load(open(use_file_as_init, 'rb'))
            # best 
            use_file_as_init = os.path.join(store_folder, '{}_iteration.pickle'.format(best_epoch))
            loss, loss_dict, loss_list, best_epoch, best_f_car, best_f_truck, \
                best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck, best_O_demand = pickle.load(open(use_file_as_init, 'rb'))
            
            f_car, f_truck = best_f_car, best_f_truck

        for i in range(max_epoch):
            if adagrad:
                sum_g_square_car = 1e-6
                sum_g_square_truck = 1e-6
            seq = np.random.permutation(self.num_data)
            loss = np.float(0)
            # print("Start iteration", time.time())
            loss_dict = {'car_count_loss': 0.0, 'truck_count_loss': 0.0, 'car_tt_loss': 0.0, 'truck_tt_loss': 0.0, 'origin_vehicle_registration_loss': 0.0}

            self.config['link_car_flow_weight'] = link_car_flow_weight[i] * (self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss'])
            self.config['link_truck_flow_weight'] = link_truck_flow_weight[i] * (self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss'])
            
            self.config['link_car_tt_weight'] = link_car_tt_weight[i] * (self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss'])
            self.config['link_truck_tt_weight'] = link_truck_tt_weight[i] * (self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss'])

            self.config['origin_vehicle_registration_weight'] = origin_vehicle_registration_weight[i] * (self.config['use_origin_vehicle_registration_data'] or self.config['compute_origin_vehicle_registration_loss'])

            for j in seq:
                one_data_dict = self._get_one_data(j)
                car_grad, truck_grad, tmp_loss, tmp_loss_dict, x_e_car, x_e_truck, tt_e_car, tt_e_truck, O_demand = self.compute_path_flow_grad_and_loss(one_data_dict, f_car, f_truck)
                # print("gradient", car_grad, truck_grad)
                if adagrad:
                    sum_g_square_car = sum_g_square_car + np.power(car_grad, 2)
                    sum_g_square_truck = sum_g_square_truck + np.power(truck_grad, 2)
                    f_car = f_car - car_step_size * car_grad / np.sqrt(sum_g_square_car) 
                    f_truck = f_truck - truck_step_size * truck_grad / np.sqrt(sum_g_square_truck) 
                else:
                    f_car -= car_grad * car_step_size / np.sqrt(i+1)
                    f_truck -= truck_grad * truck_step_size / np.sqrt(i+1)
                f_car = np.maximum(f_car, 1e-3)
                f_truck = np.maximum(f_truck, 1e-3)
                # f_truck = np.minimum(f_truck, 30)
                loss += tmp_loss / np.float(self.num_data)
                for loss_type, loss_value in tmp_loss_dict.items():
                    loss_dict[loss_type] += loss_value / np.float(self.num_data)
            
            print("Epoch:", starting_epoch + i, "Loss:", loss, self.print_separate_accuracy(loss_dict))
            loss_list.append([loss, loss_dict])

            if (best_epoch == 0) or (loss_list[best_epoch][0] > loss_list[-1][0]):
                best_epoch = starting_epoch + i
                best_f_car_driving, best_f_truck_driving = f_car, f_truck
                best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck, best_O_demand = x_e_car, x_e_truck, tt_e_car, tt_e_truck, O_demand

                if store_folder is not None:
                    self.save_simulation_input_files(os.path.join(store_folder, 'input_files_estimate_path_flow'), 
                                                     best_f_car_driving, best_f_truck_driving)

            # print(f_car, f_truck)
            # break
            if store_folder is not None:
                pickle.dump((loss, loss_dict, loss_list, best_epoch, f_car, f_truck,
                             x_e_car, x_e_truck, tt_e_car, tt_e_truck, O_demand), open(os.path.join(store_folder, str(starting_epoch + i) + '_iteration.pickle'), 'wb'))

        print("Best loss at Epoch:", best_epoch, "Loss:", loss_list[best_epoch][0], self.print_separate_accuracy(loss_list[best_epoch][1]))
        return best_f_car_driving, best_f_truck_driving, best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck, best_O_demand, loss_list

    def estimate_path_flow_gd(self, car_step_size=0.1, truck_step_size=0.1, max_epoch=10, 
                                link_car_flow_weight=1, link_truck_flow_weight=1, 
                                link_car_tt_weight=1, link_truck_tt_weight=1,
                                car_init_scale=10, truck_init_scale=1, store_folder=None, use_file_as_init=None, adagrad=False, starting_epoch=0):
    
        if np.isscalar(link_car_flow_weight):
            link_car_flow_weight = np.ones(max_epoch, dtype=bool) * link_car_flow_weight
        assert(len(link_car_flow_weight) == max_epoch)

        if np.isscalar(link_truck_flow_weight):
            link_truck_flow_weight = np.ones(max_epoch, dtype=bool) * link_truck_flow_weight
        assert(len(link_truck_flow_weight) == max_epoch)

        if np.isscalar(link_car_tt_weight):
            link_car_tt_weight = np.ones(max_epoch, dtype=bool) * link_car_tt_weight
        assert(len(link_car_tt_weight) == max_epoch)

        if np.isscalar(link_truck_tt_weight):
            link_truck_tt_weight = np.ones(max_epoch, dtype=bool) * link_truck_tt_weight
        assert(len(link_truck_tt_weight) == max_epoch)
        
        # here the basic variables to be estimated are path flows, not OD demand, so no route choice model, unlike in sDODE.py
        loss_list = list()
        best_epoch = starting_epoch
        best_f_car_driving, best_f_truck_driving = 0, 0
        best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck = 0, 0, 0, 0
        # here the basic variables to be estimated are path flows, not OD demand, so no route choice model, unlike in sDODE.py
        if use_file_as_init is None:
            (f_car, f_truck) = self.init_path_flow(car_scale=car_init_scale, truck_scale=truck_init_scale)
        else:
            # most recent
            _, _, loss_list, best_epoch, _, _, \
                _, _, _, _ = pickle.load(open(use_file_as_init, 'rb'))
            # best 
            use_file_as_init = os.path.join(store_folder, '{}_iteration.pickle'.format(best_epoch))
            loss, loss_dict, loss_list, best_epoch, best_f_car, best_f_truck, \
                best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck = pickle.load(open(use_file_as_init, 'rb'))
            
            f_car, f_truck = best_f_car, best_f_truck

        start_time = time.time()
        for i in range(max_epoch):
            grad_car_sum = np.zeros(f_car.shape)
            grad_truck_sum = np.zeros(f_truck.shape)
            seq = np.random.permutation(self.num_data)
            loss = np.float(0)
            # print("Start iteration", time.time())
            loss_dict = {'car_count_loss': 0.0, 'truck_count_loss': 0.0, 'car_tt_loss': 0.0, 'truck_tt_loss': 0.0}

            self.config['link_car_flow_weight'] = link_car_flow_weight[i] * (self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss'])
            self.config['link_truck_flow_weight'] = link_truck_flow_weight[i] * (self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss'])
            
            self.config['link_car_tt_weight'] = link_car_tt_weight[i] * (self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss'])
            self.config['link_truck_tt_weight'] = link_truck_tt_weight[i] * (self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss'])

            for j in seq:
                one_data_dict = self._get_one_data(j)
                car_grad, truck_grad, tmp_loss, tmp_loss_dict, x_e_car, x_e_truck, tt_e_car, tt_e_truck = self.compute_path_flow_grad_and_loss(one_data_dict, f_car, f_truck)
                # print("gradient", car_grad, truck_grad)
            
                grad_car_sum += car_grad
                grad_truck_sum += truck_grad
                #f_truck = np.minimum(f_truck, 30)

                loss += tmp_loss / np.float(self.num_data)
                for loss_type, loss_value in tmp_loss_dict.items():
                    loss_dict[loss_type] += loss_value / np.float(self.num_data)
        
            print("Epoch:", starting_epoch + i, "Loss:", loss, self.print_separate_accuracy(loss_dict))
            loss_list.append([loss, loss_dict])

            if (best_epoch == 0) or (loss_list[best_epoch][0] > loss_list[-1][0]):
                best_epoch = starting_epoch + i
                best_f_car_driving, best_f_truck_driving = f_car, f_truck
                best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck = x_e_car, x_e_truck, tt_e_car, tt_e_truck

                if store_folder is not None:
                    self.save_simulation_input_files(os.path.join(store_folder, 'input_files_estimate_path_flow'), 
                                                     best_f_car_driving, best_f_truck_driving)

            f_car -= grad_car_sum * car_step_size / np.sqrt(i+1) / np.float(self.num_data)
            f_truck -= grad_truck_sum * truck_step_size / np.sqrt(i+1) / np.float(self.num_data)
            f_car = np.maximum(f_car, 1e-3)
            f_truck = np.maximum(f_truck, 1e-3)

            # print(f_car, f_truck)
            # break
            if store_folder is not None:
                pickle.dump((loss, loss_dict, loss_list, best_epoch, f_car, f_truck,
                             x_e_car, x_e_truck, tt_e_car, tt_e_truck, time.time() - start_time), open(os.path.join(store_folder, str(starting_epoch + i) + '_iteration.pickle'), 'wb'))

        print("Best loss at Epoch:", best_epoch, "Loss:", loss_list[best_epoch][0], self.print_separate_accuracy(loss_list[best_epoch][1]))
        return best_f_car_driving, best_f_truck_driving, best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck, loss_list

    def compute_path_flow_grad_and_loss_mpwrapper(self, one_data_dict, f_car, f_truck, j, output):
        car_grad, truck_grad, tmp_loss, tmp_loss_dict, x_e_car, x_e_truck, tt_e_car, tt_e_truck = self.compute_path_flow_grad_and_loss(one_data_dict, f_car, f_truck, counter=j)
        # print("finished original grad loss")
        output.put([car_grad, truck_grad, tmp_loss, tmp_loss_dict, x_e_car, x_e_truck, tt_e_car, tt_e_truck])
        # output.put(grad)
        # print("finished put")
        return

    def estimate_path_flow_mp(self, car_step_size=0.1, truck_step_size=0.1, 
                                link_car_flow_weight=1, link_truck_flow_weight=1, 
                                link_car_tt_weight=1, link_truck_tt_weight=1,
                                max_epoch=10, car_init_scale=10,
                                truck_init_scale=1, store_folder=None, use_file_as_init=None,
                                adagrad=False, starting_epoch=0, n_process=4, record_time=False):

        if np.isscalar(link_car_flow_weight):
            link_car_flow_weight = np.ones(max_epoch, dtype=bool) * link_car_flow_weight
        assert(len(link_car_flow_weight) == max_epoch)

        if np.isscalar(link_truck_flow_weight):
            link_truck_flow_weight = np.ones(max_epoch, dtype=bool) * link_truck_flow_weight
        assert(len(link_truck_flow_weight) == max_epoch)

        if np.isscalar(link_car_tt_weight):
            link_car_tt_weight = np.ones(max_epoch, dtype=bool) * link_car_tt_weight
        assert(len(link_car_tt_weight) == max_epoch)

        if np.isscalar(link_truck_tt_weight):
            link_truck_tt_weight = np.ones(max_epoch, dtype=bool) * link_truck_tt_weight
        assert(len(link_truck_tt_weight) == max_epoch)

        # here the basic variables to be estimated are path flows, not OD demand, so no route choice model, unlike in sDODE.py
        loss_list = list()
        best_epoch = starting_epoch
        best_f_car_driving, best_f_truck_driving = 0, 0
        best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck = 0, 0, 0, 0
        # here the basic variables to be estimated are path flows, not OD demand, so no route choice model, unlike in sDODE.py
        if use_file_as_init is None:
            (f_car, f_truck) = self.init_path_flow(car_scale=car_init_scale, truck_scale=truck_init_scale)
        else:
            # most recent
            _, _, loss_list, best_epoch, _, _, \
                _, _, _, _ = pickle.load(open(use_file_as_init, 'rb'))
            # best 
            use_file_as_init = os.path.join(store_folder, '{}_iteration.pickle'.format(best_epoch))
            loss, loss_dict, loss_list, best_epoch, best_f_car, best_f_truck, \
                best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck = pickle.load(open(use_file_as_init, 'rb'))
            
            f_car, f_truck = best_f_car, best_f_truck
 
        # print("Start iteration", time.time())
        start_time = time.time()
        for i in range(max_epoch):
            if adagrad:
                sum_g_square_car = 1e-6
                sum_g_square_truck = 1e-6
            seq = np.random.permutation(self.num_data)
            split_seq = np.array_split(seq, np.maximum(1, int(self.num_data/n_process)))

            loss = np.float(0)
            loss_dict = {'car_count_loss': 0.0, 'truck_count_loss': 0.0, 'car_tt_loss': 0.0, 'truck_tt_loss': 0.0}

            self.config['link_car_flow_weight'] = link_car_flow_weight[i] * (self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss'])
            self.config['link_truck_flow_weight'] = link_truck_flow_weight[i] * (self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss'])
            
            self.config['link_car_tt_weight'] = link_car_tt_weight[i] * (self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss'])
            self.config['link_truck_tt_weight'] = link_truck_tt_weight[i] * (self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss'])

            for part_seq in split_seq:
                output = mp.Queue()
                processes = [mp.Process(target=self.compute_path_flow_grad_and_loss_mpwrapper, args=(self._get_one_data(j), f_car, f_truck, j, output)) for j in part_seq]
                for p in processes:
                    p.start()
                results = list()
                while 1:
                    running = any(p.is_alive() for p in processes)
                    while not output.empty():
                        results.append(output.get())
                    if not running:
                        break
                for p in processes:
                    p.join()
                # results = [output.get() for p in processes]
                for res in results:
                    [car_grad, truck_grad, tmp_loss, tmp_loss_dict, x_e_car, x_e_truck, tt_e_car, tt_e_truck] = res
                    loss += tmp_loss / np.float(self.num_data)
                    for loss_type, loss_value in tmp_loss_dict.items():
                        loss_dict[loss_type] += loss_value / np.float(self.num_data)
                    if adagrad:
                        sum_g_square_car = sum_g_square_car + np.power(car_grad, 2)
                        sum_g_square_truck = sum_g_square_truck + np.power(truck_grad, 2)
                        f_car = f_car - car_step_size * car_grad / np.sqrt(sum_g_square_car)
                        f_truck = f_truck - truck_step_size * truck_grad / np.sqrt(sum_g_square_truck)
                    else:
                        f_car -= car_grad * car_step_size / np.sqrt(i+1)
                        f_truck -= truck_grad * truck_step_size / np.sqrt(i+1)       
                    f_car = np.maximum(f_car, 1e-3)
                    f_truck = np.maximum(f_truck, 1e-3)
                    f_truck = np.minimum(f_truck, 30)

        print("Epoch:", starting_epoch + i, "Loss:", loss, self.print_separate_accuracy(loss_dict))

        if (best_epoch == 0) or (loss_list[best_epoch][0] > loss_list[-1][0]):
            best_epoch = starting_epoch + i
            best_f_car_driving, best_f_truck_driving = f_car, f_truck
            best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck = x_e_car, x_e_truck, tt_e_car, tt_e_truck

            if store_folder is not None:
                    self.save_simulation_input_files(os.path.join(store_folder, 'input_files_estimate_path_flow'), 
                                                     best_f_car_driving, best_f_truck_driving)

        if store_folder is not None:
            pickle.dump((loss, loss_dict, loss_list, best_epoch, f_car, f_truck,
                         x_e_car, x_e_truck, tt_e_car, tt_e_truck, time.time() - start_time), open(os.path.join(store_folder, str(starting_epoch + i) + '_iteration.pickle'), 'wb'))
        if record_time:
            loss_list.append([loss, loss_dict, time.time() - start_time])
        else:
            loss_list.append([loss, loss_dict])

        print("Best loss at Epoch:", best_epoch, "Loss:", loss_list[best_epoch][0], self.print_separate_accuracy(loss_list[best_epoch][1]))
        return best_f_car_driving, best_f_truck_driving, best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck, loss_list

    def generate_route_choice(self):
        pass

    def print_separate_accuracy(self, loss_dict):
        tmp_str = ""
        for loss_type, loss_value in loss_dict.items():
            tmp_str += loss_type + ": " + str(np.round(loss_value, 2)) + "|"
        return tmp_str


# Simultaneous Perturbation Stochastic Approximation
class mcSPSA():
  def __init__(self, nb, config):
    self.config = config
    self.nb = nb
    self.num_assign_interval = nb.config.config_dict['DTA']['max_interval']
    self.ass_freq = nb.config.config_dict['DTA']['assign_frq']
    self.num_link = nb.config.config_dict['DTA']['num_of_link']
    self.num_path = nb.config.config_dict['FIXED']['num_path']
    self.num_loading_interval = self.num_assign_interval * self.ass_freq
    self.data_dict = dict()
    self.num_data = self.config['num_data']
    self.observed_links = self.config['observed_links']
    self.paths_list = self.config['paths_list']
    self.car_count_agg_L_list = None
    self.truck_count_agg_L_list = None
    assert (len(self.paths_list) == self.num_path)

  def _add_car_link_flow_data(self, link_flow_df_list):
    # assert (self.config['use_car_link_flow'])
    assert (self.num_data == len(link_flow_df_list))
    self.data_dict['car_link_flow'] = link_flow_df_list

  def _add_truck_link_flow_data(self, link_flow_df_list):
    # assert (self.config['use_truck_link_flow'])
    assert (self.num_data == len(link_flow_df_list))
    self.data_dict['truck_link_flow'] = link_flow_df_list

  def _add_car_link_tt_data(self, link_spd_df_list):
    # assert (self.config['use_car_link_tt'])
    assert (self.num_data == len(link_spd_df_list))
    self.data_dict['car_link_tt'] = link_spd_df_list

  def _add_truck_link_tt_data(self, link_spd_df_list):
    # assert (self.config['use_truck_link_tt'])
    assert (self.num_data == len(link_spd_df_list))
    self.data_dict['truck_link_tt'] = link_spd_df_list

  def add_data(self, data_dict):
    if self.config['car_count_agg']:
      self.car_count_agg_L_list = data_dict['car_count_agg_L_list']
    if self.config['truck_count_agg']:
      self.truck_count_agg_L_list = data_dict['truck_count_agg_L_list']
    if self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss']:
      self._add_car_link_flow_data(data_dict['car_link_flow'])
    if self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss'] :
      self._add_truck_link_flow_data(data_dict['truck_link_flow'])
    if self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss']:
      self._add_car_link_tt_data(data_dict['car_link_tt'])
    if self.config['use_truck_link_tt']or self.config['compute_car_link_tt_loss']:
      self._add_truck_link_tt_data(data_dict['truck_link_tt'])

  def save_simulation_input_files(self, folder_path, f_car=None, f_truck=None):

        if not os.path.exists(folder_path):
            os.mkdir(folder_path)

        # update demand for each mode
        if (f_car is not None) and (f_truck is not None):
            self.nb.update_demand_path2(f_car, f_truck)
   
        # self.nb.config.config_dict['DTA']['flow_scalar'] = 3
        if self.config['use_car_link_tt'] or self.config['use_truck_link_tt']:
            self.nb.config.config_dict['DTA']['total_interval'] = self.num_loading_interval # * 2  # hopefully this is sufficient 
        else:
            self.nb.config.config_dict['DTA']['total_interval'] = self.num_loading_interval  # if only count data is used

        self.nb.config.config_dict['DTA']['routing_type'] = 'Biclass_Hybrid'

        # no output files saved from DNL
        self.nb.config.config_dict['STAT']['rec_volume'] = 1
        self.nb.config.config_dict['STAT']['volume_load_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['volume_record_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['rec_tt'] = 1
        self.nb.config.config_dict['STAT']['tt_load_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['tt_record_automatic_rec'] = 0

        self.nb.dump_to_folder(folder_path)

  def _run_simulation(self, f_car, f_truck, counter=0, show_loading=False):
    hash1 = hashlib.sha1()
    hash1.update(str(time.time()) + str(counter))
    new_folder = str(hash1.hexdigest())

    self.save_simulation_input_files(new_folder, f_car, f_truck)

    a = MNMAPI.mcdta_api()
    a.initialize(new_folder)
    shutil.rmtree(new_folder)
    a.register_links(self.observed_links)
    a.register_paths(self.paths_list)
    a.install_cc()
    a.install_cc_tree()
    a.run_whole(show_loading)
    # print("Finish simulation", time.time())
    return a  

  def init_path_flow(self, car_scale=1, truck_scale=0.1):
    f_car = np.random.rand(self.num_assign_interval * self.num_path) * car_scale
    f_truck = np.random.rand(self.num_assign_interval * self.num_path) * truck_scale
    return f_car, f_truck

  def _get_one_data(self, j):
    assert (self.num_data > j)
    one_data_dict = dict()
    if self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss']:
      one_data_dict['car_link_flow'] = self.data_dict['car_link_flow'][j]
    if self.config['use_truck_link_flow']or self.config['compute_truck_link_flow_loss']:
      one_data_dict['truck_link_flow'] = self.data_dict['truck_link_flow'][j]
    if self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss']:
      one_data_dict['car_link_tt'] = self.data_dict['car_link_tt'][j]
    if self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss']:
      one_data_dict['truck_link_tt'] = self.data_dict['truck_link_tt'][j]
    if self.config['car_count_agg']:
      one_data_dict['car_count_agg_L'] = self.car_count_agg_L_list[j]
    if self.config['truck_count_agg']:
      one_data_dict['truck_count_agg_L'] = self.truck_count_agg_L_list[j]
    return one_data_dict

  def _get_loss(self, one_data_dict, dta):
    loss_dict = dict()
    if self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss']:
      x_e = dta.get_link_car_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
      if self.config['car_count_agg']:
        x_e = one_data_dict['car_count_agg_L'].dot(x_e)
      loss = self.config['link_car_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['car_link_flow']))
      loss_dict['car_count_loss'] = loss
    if self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss']:
      x_e = dta.get_link_truck_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                      np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
      if self.config['truck_count_agg']:
        x_e = one_data_dict['truck_count_agg_L'].dot(x_e)
      loss = self.config['link_truck_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['truck_link_flow']))
      loss_dict['truck_count_loss'] = loss
    if self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss']:
      x_tt_e = dta.get_car_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
      # x_tt_e = dta.get_car_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
      #                                     np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order = 'F')
      loss = self.config['link_car_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['car_link_tt']))
      loss_dict['car_tt_loss'] = loss
    if self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss']:
      x_tt_e = dta.get_truck_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
      loss = self.config['link_truck_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['truck_link_tt']))
      loss_dict['truck_tt_loss'] = loss

    total_loss = 0.0
    for loss_type, loss_value in loss_dict.items():
      total_loss += loss_value
    return total_loss, loss_dict

  def compute_path_flow_grad_and_loss(self, one_data_dict, f_car, f_truck, delta_car_scale=0.1, delta_truck_scale=0.01):
    delta_f_car = (np.random.rand(*f_car.shape) * 2 - 1) * delta_car_scale
    delta_f_truck = (np.random.rand(*f_truck.shape) * 2 - 1) * delta_truck_scale
    f_car_1 = np.maximum(f_car + delta_f_car, 1e-3)
    f_truck_1 = np.maximum(f_truck + delta_f_truck, 1e-3)
    f_car_2 = np.maximum(f_car - delta_f_car, 1e-3)
    f_truck_2 = np.maximum(f_truck - delta_f_truck, 1e-3)
    # print(f_car.sum(), f_truck.sum())
    # print(f_car_1.sum(), f_truck_1.sum())
    # print(f_car_2.sum(), f_truck_2.sum())
    dta_1 = self._run_simulation(f_car_1, f_truck_1, 11111)
    dta_2 = self._run_simulation(f_car_2, f_truck_2, 22222)
    loss_1 = self._get_loss(one_data_dict, dta_1)[0]
    loss_2 = self._get_loss(one_data_dict, dta_2)[0]
    grad_f_car = (loss_1 - loss_2) / (2 * delta_f_car)
    grad_f_truck = (loss_1 - loss_2) / (2 * delta_f_truck)
    dta = self._run_simulation(f_car, f_truck, 33333)
    total_loss, loss_dict = self._get_loss(one_data_dict, dta)
    return -grad_f_car, - grad_f_truck, total_loss, loss_dict

  def estimate_path_flow(self, car_step_size=0.1, truck_step_size=0.1, max_epoch=10, car_init_scale=10,
                         truck_init_scale=1, store_folder=None, use_file_as_init=None,
                         adagrad=False, delta_car_scale=0.1, delta_truck_scale=0.01):
    if use_file_as_init is None:
      (f_car, f_truck) = self.init_path_flow(car_scale=car_init_scale, truck_scale=truck_init_scale)
    else:
      (f_car, f_truck, _) = pickle.load(open(use_file_as_init, 'rb'))
    loss_list = list()
    for i in range(max_epoch):
      if adagrad:
        sum_g_square_car = 1e-6
        sum_g_square_truck = 1e-6
      seq = np.random.permutation(self.num_data)
      loss = np.float(0)
      # print("Start iteration", time.time())
      loss_dict = {'car_count_loss': 0.0, 'truck_count_loss': 0.0, 'car_tt_loss': 0.0, 'truck_tt_loss': 0.0}
      for j in seq:
        one_data_dict = self._get_one_data(j)
        car_grad, truck_grad, tmp_loss, tmp_loss_dict = self.compute_path_flow_grad_and_loss(one_data_dict, f_car, f_truck,
                                                                                             delta_car_scale=delta_car_scale,
                                                                                             delta_truck_scale=delta_truck_scale)
        # print("gradient", car_grad, truck_grad)
        if adagrad:
          sum_g_square_car = sum_g_square_car + np.power(car_grad, 2)
          sum_g_square_truck = sum_g_square_truck + np.power(truck_grad, 2)
          f_car = f_car - car_step_size * car_grad / np.sqrt(sum_g_square_car) 
          f_truck = f_truck - truck_step_size * truck_grad / np.sqrt(sum_g_square_truck) 
        else:
          f_car -= car_grad * car_step_size / np.sqrt(i+1)
          f_truck -= truck_grad * truck_step_size / np.sqrt(i+1)
        f_car = np.maximum(f_car, 1e-3)
        f_truck = np.maximum(f_truck, 1e-3)
        # f_truck = np.minimum(f_truck, 30)
        loss += tmp_loss
        for loss_type, loss_value in tmp_loss_dict.items():
          loss_dict[loss_type] += loss_value / np.float(self.num_data)
      print("Epoch:", i, "Loss:", np.round(loss / np.float(self.num_data),2), self.print_separate_accuracy(loss_dict))
      # print(f_car, f_truck)
      # break
      if store_folder is not None:
        pickle.dump((f_car, f_truck, loss), open(os.path.join(store_folder, str(i) + 'iteration.pickle'), 'wb'))
      loss_list.append([loss, loss_dict])
    return f_car, f_truck, loss_list

  def print_separate_accuracy(self, loss_dict):
    tmp_str = ""
    for loss_type, loss_value in loss_dict.items():
      tmp_str += loss_type + ": " + str(np.round(loss_value, 2)) + "|"
    return tmp_str


class PostProcessing:
    def __init__(self, dode, dta=None, f_car=None, f_truck=None,
                 estimated_car_count=None, estimated_truck_count=None,
                 estimated_car_cost=None, estimated_truck_cost=None,
                 estimated_origin_demand=None,
                 result_folder=None):
        self.dode = dode
        self.dta = dta
        self.result_folder = result_folder
        self.one_data_dict = None

        self.f_car = f_car
        self.f_truck = f_truck

        self.color_list = ['teal', 'tomato', 'blue', 'sienna', 'plum', 'red', 'yellowgreen', 'khaki']
        self.marker_list = ["o", "v", "^", "<", ">", "p", "D", "*", "s", "D", "p"]

        self.r2_car_count, self.r2_truck_count = "NA", "NA"
        self.true_car_count, self.estimated_car_count = None, estimated_car_count
        self.true_truck_count, self.estimated_truck_count = None, estimated_truck_count

        self.r2_car_cost, self.r2_truck_cost = "NA", "NA"
        self.true_car_cost, self.estimated_car_cost = None, estimated_car_cost
        self.true_truck_cost, self.estimated_truck_cost = None, estimated_truck_cost

        self.r2_origin_vehicle_registration = "NA"
        self.true_origin_vehicle_registration, self.estimated_origin_vehicle_registration = None, None
        self.estimated_origin_demand = estimated_origin_demand
        
        plt.rc('font', size=20)          # controls default text sizes
        plt.rc('axes', titlesize=20)     # fontsize of the axes title
        plt.rc('axes', labelsize=20)     # fontsize of the x and y labels
        plt.rc('xtick', labelsize=20)    # fontsize of the tick labels
        plt.rc('ytick', labelsize=20)    # fontsize of the tick labels
        plt.rc('legend', fontsize=20)    # legend fontsize
        plt.rc('figure', titlesize=20)   # fontsize of the figure title

        sns.set()
        plt.style.use('seaborn-poster')

    def plot_total_loss(self, loss_list, fig_name = 'total_loss_pathflow.png'):

        plt.figure(figsize = (16, 9), dpi=300)
        plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[0], loss_list)), color = self.color_list[0], marker = self.marker_list[0], linewidth = 3)
        # plt.plot(range(len(l_list)), list(map(lambda x: x[0], l_list)),
        #          color = color_list[4], linewidth = 3, label = "Total cost")

        plt.ylabel('Loss', fontsize = 20)
        plt.xlabel('Iteration', fontsize = 20)
        # plt.legend()
        # plt.ylim([0, 1])
        plt.xlim([1, len(loss_list)])

        plt.savefig(os.path.join(self.result_folder, fig_name))

        plt.show()

    def plot_breakdown_loss(self, loss_list, fig_name = 'breakdown_loss_pathflow.png'):

        if self.dode.config['use_car_link_flow'] + self.dode.config['use_truck_link_flow'] + \
            self.dode.config['use_car_link_tt'] + self.dode.config['use_truck_link_tt'] + self.dode.config['use_origin_vehicle_registration_data']:

            plt.figure(figsize = (16, 9), dpi=300)

            i = 0

            if self.dode.config['use_car_link_flow']:
                plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[1]['car_count_loss']/loss_list[0][1]['car_count_loss'], loss_list)),
                        color = self.color_list[i],  marker = self.marker_list[i], linewidth = 3, label = "Car observed flow")
            i += self.dode.config['use_car_link_flow']

            if self.dode.config['use_truck_link_flow']:
                plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[1]['truck_count_loss']/loss_list[0][1]['truck_count_loss'], loss_list)),
                        color = self.color_list[i],  marker = self.marker_list[i], linewidth = 3, label = "Truck observed flow")
            i += self.dode.config['use_truck_link_flow']

            if self.dode.config['use_car_link_tt']:
                plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[1]['car_tt_loss']/loss_list[0][1]['car_tt_loss'], loss_list)),
                    color = self.color_list[i],  marker = self.marker_list[i], linewidth = 3, label = "Car observed travel cost")
            i += self.dode.config['use_car_link_tt']

            if self.dode.config['use_truck_link_tt']:
                plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[1]['truck_tt_loss']/loss_list[0][1]['truck_tt_loss'], loss_list)), 
                        color = self.color_list[i],  marker = self.marker_list[i], linewidth = 3, label = "Truck observed travel cost")
            i += self.dode.config['use_truck_link_tt']

            if self.dode.config['use_origin_vehicle_registration_data']:
                plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[1]['origin_vehicle_registration_loss']/loss_list[0][1]['origin_vehicle_registration_loss'], loss_list)), 
                        color = self.color_list[i],  marker = self.marker_list[i], linewidth = 3, label = "Origin vehicle registration data")

                # plt.plot(np.arange(len(loss_list))+1, list(map(lambda x: x[1]['origin_vehicle_registration_loss'], loss_list)), 
                #         color = self.color_list[i],  marker = self.marker_list[i], linewidth = 3, label = "Origin vehicle registration data")

            plt.ylabel('Loss')
            plt.xlabel('Iteration')
            plt.legend(loc='lower center', ncol=3, bbox_to_anchor=(0.5, 1))
            plt.ylim([0, 1.1])
            plt.xlim([1, len(loss_list)])

            plt.savefig(os.path.join(self.result_folder, fig_name))

            plt.show()


    def get_one_data(self, start_intervals, end_intervals, j=0):
        assert(len(start_intervals) == len(end_intervals))

        # assume only one observation exists
        self.one_data_dict = self.dode._get_one_data(j)  

        if 'mask_driving_link' not in self.one_data_dict:
            self.one_data_dict['mask_driving_link'] = np.ones(len(self.dode.observed_links) * len(start_intervals), dtype=bool)
        
        # count
        if self.dode.config['use_car_link_flow']:
            self.true_car_count = self.one_data_dict['car_link_flow']
            if self.estimated_car_count is None:
                L_car = self.one_data_dict['car_count_agg_L']
                estimated_car_x = self.dta.get_link_car_inflow(start_intervals, end_intervals).flatten(order='F')
                self.estimated_car_count = L_car.dot(estimated_car_x)
            
            self.true_car_count, self.estimated_car_count = self.true_car_count[self.one_data_dict['mask_driving_link']], self.estimated_car_count[self.one_data_dict['mask_driving_link']]

        if self.dode.config['use_truck_link_flow']:
            self.true_truck_count = self.one_data_dict['truck_link_flow']
            if self.estimated_truck_count is None:
                L_truck = self.one_data_dict['truck_count_agg_L']
                estimated_truck_x = self.dta.get_link_truck_inflow(start_intervals, end_intervals).flatten(order='F')
                self.estimated_truck_count = L_truck.dot(estimated_truck_x)

            self.true_truck_count, self.estimated_truck_count = self.true_truck_count[self.one_data_dict['mask_driving_link']], self.estimated_truck_count[self.one_data_dict['mask_driving_link']]

        # travel cost
        if self.dode.config['use_car_link_tt']:
            self.true_car_cost = self.one_data_dict['car_link_tt']

            if self.estimated_car_cost is None:
                # self.estimated_car_cost = self.dta.get_car_link_tt_robust(start_intervals, end_intervals, self.dode.ass_freq, True).flatten(order = 'F')
                self.estimated_car_cost = self.dta.get_car_link_tt(start_intervals, False).flatten(order = 'F')

            self.true_car_cost, self.estimated_car_cost = self.true_car_cost[self.one_data_dict['mask_driving_link']], self.estimated_car_cost[self.one_data_dict['mask_driving_link']]

        if self.dode.config['use_truck_link_tt']:
            self.true_truck_cost = self.one_data_dict['truck_link_tt']

            if self.estimated_truck_cost is None:
                # self.estimated_truck_cost = self.dta.get_truck_link_tt_robust(start_intervals, end_intervals, self.dode.ass_freq, True).flatten(order = 'F')
                self.estimated_truck_cost = self.dta.get_truck_link_tt(start_intervals, False).flatten(order = 'F')
            
            self.true_truck_cost, self.estimated_truck_cost = self.true_truck_cost[self.one_data_dict['mask_driving_link']], self.estimated_truck_cost[self.one_data_dict['mask_driving_link']]

        # origin vehicle registration data
        if self.dode.config['use_origin_vehicle_registration_data']:
            assert(self.f_car is not None and self.f_truck is not None)
            # pandas DataFrame
            self.true_origin_vehicle_registration = self.one_data_dict['origin_vehicle_registration_data']

            if self.estimated_origin_demand is None:
                O_demand_est = self.dode.aggregate_f(self.f_car, self.f_truck)
            else:
                O_demand_est = self.estimated_origin_demand
            # pandas DataFrame
            self.estimated_origin_vehicle_registration = self.true_origin_vehicle_registration.copy()
            self.estimated_origin_vehicle_registration['car'] = 0.
            self.estimated_origin_vehicle_registration['truck'] = 0.
            def process_one_row_car(row):
                return sum(O_demand_est[Origin_ID][0] for Origin_ID in row['origin_ID'])
            def process_one_row_truck(row):
                return sum(O_demand_est[Origin_ID][1] for Origin_ID in row['origin_ID'])
            self.estimated_origin_vehicle_registration['car'] = self.estimated_origin_vehicle_registration.apply(lambda row: process_one_row_car(row), axis=1)
            self.estimated_origin_vehicle_registration['truck'] = self.estimated_origin_vehicle_registration.apply(lambda row: process_one_row_truck(row), axis=1)
            
            self.true_origin_vehicle_registration = self.true_origin_vehicle_registration.loc[:, ['car', 'truck']].sum(axis=1)
            self.estimated_origin_vehicle_registration = self.estimated_origin_vehicle_registration.loc[:, ['car', 'truck']].sum(axis=1)
            

    def cal_r2_count(self):

        if self.dode.config['use_car_link_flow']:
            print('----- car count -----')
            print(self.true_car_count)
            print(self.estimated_car_count)
            print('----- car count -----')
            ind = ~(np.isinf(self.true_car_count) + np.isinf(self.estimated_car_count) + np.isnan(self.true_car_count) + np.isnan(self.estimated_car_count))
            self.r2_car_count = r2_score(self.true_car_count[ind], self.estimated_car_count[ind])

        if self.dode.config['use_truck_link_flow']:
            print('----- truck count -----')
            print(self.true_truck_count)
            print(self.estimated_truck_count)
            print('----- truck count -----')
            ind = ~(np.isinf(self.true_truck_count) + np.isinf(self.estimated_truck_count) + np.isnan(self.true_truck_count) + np.isnan(self.estimated_truck_count))
            self.r2_truck_count = r2_score(self.true_truck_count[ind], self.estimated_truck_count[ind])

        if self.dode.config['use_origin_vehicle_registration_data']:
            print('----- origin vehicle registration data count -----')
            print(self.true_origin_vehicle_registration)
            print(self.estimated_origin_vehicle_registration)
            print('----- origin vehicle registration data count -----')
            self.r2_origin_vehicle_registration = r2_score(self.true_origin_vehicle_registration, self.estimated_origin_vehicle_registration)

        print("r2 count --- r2_car_count: {}, r2_truck_count: {}, r2_origin_vehicle_registration: {}"
            .format(
                self.r2_car_count,
                self.r2_truck_count,
                self.r2_origin_vehicle_registration
                ))
        
        return self.r2_car_count, self.r2_truck_count

    def scatter_plot_count(self, fig_name =  'link_flow_scatterplot_pathflow.png'):
        if self.dode.config['use_car_link_flow'] + self.dode.config['use_truck_link_flow'] + self.dode.config['use_origin_vehicle_registration_data']:

            fig, axes = plt.subplots(1,
                                    self.dode.config['use_car_link_flow'] + self.dode.config['use_truck_link_flow'] + self.dode.config['use_origin_vehicle_registration_data'], 
                                    figsize=(36, 9), dpi=300, squeeze=False)

            i = 0

            if self.dode.config['use_car_link_flow']:
                ind = ~(np.isinf(self.true_car_count) + np.isinf(self.estimated_car_count) + np.isnan(self.true_car_count) + np.isnan(self.estimated_car_count))
                m_car_max = int(np.max((np.max(self.true_car_count[ind]), np.max(self.estimated_car_count[ind]))) + 1)
                axes[0, i].scatter(self.true_car_count[ind], self.estimated_car_count[ind], color = self.color_list[i], marker = self.marker_list[i], s = 100)
                axes[0, i].plot(range(m_car_max + 1), range(m_car_max + 1), color = 'gray')
                axes[0, i].set_ylabel('Estimated observed flow for car')
                axes[0, i].set_xlabel('True observed flow for car')
                axes[0, i].set_xlim([0, m_car_max])
                axes[0, i].set_ylim([0, m_car_max])
                axes[0, i].text(0, 1, 'r2 = {}'.format(self.r2_car_count),
                            horizontalalignment='left',
                            verticalalignment='top',
                            transform=axes[0, i].transAxes)

            i += self.dode.config['use_car_link_flow']

            if self.dode.config['use_truck_link_flow']:
                ind = ~(np.isinf(self.true_truck_count) + np.isinf(self.estimated_truck_count) + np.isnan(self.true_truck_count) + np.isnan(self.estimated_truck_count))
                m_truck_max = int(np.max((np.max(self.true_truck_count[ind]), np.max(self.estimated_truck_count[ind]))) + 1)
                axes[0, i].scatter(self.true_truck_count[ind], self.estimated_truck_count[ind], color = self.color_list[i], marker = self.marker_list[i], s = 100)
                axes[0, i].plot(range(m_truck_max + 1), range(m_truck_max + 1), color = 'gray')
                axes[0, i].set_ylabel('Estimated observed flow for truck')
                axes[0, i].set_xlabel('True observed flow for truck')
                axes[0, i].set_xlim([0, m_truck_max])
                axes[0, i].set_ylim([0, m_truck_max])
                axes[0, i].text(0, 1, 'r2 = {}'.format(self.r2_truck_count),
                            horizontalalignment='left',
                            verticalalignment='top',
                            transform=axes[0, i].transAxes)

            i += self.dode.config['use_truck_link_flow']

            if self.dode.config['use_origin_vehicle_registration_data']:
                m_max = int(np.max((np.max(self.true_origin_vehicle_registration), np.max(self.estimated_origin_vehicle_registration))) + 1)
                axes[0, i].scatter(self.true_origin_vehicle_registration, self.estimated_origin_vehicle_registration, color = self.color_list[i], marker = self.marker_list[i], s = 100)
                axes[0, i].plot(range(m_max + 1), range(m_max + 1), color = 'gray')
                axes[0, i].set_ylabel('Estimated origin vehicle registration count')
                axes[0, i].set_xlabel('True origin vehicle registration count')
                axes[0, i].set_xlim([0, m_max])
                axes[0, i].set_ylim([0, m_max])
                axes[0, i].text(0, 1, 'r2 = {}'.format(self.r2_origin_vehicle_registration),
                            horizontalalignment='left',
                            verticalalignment='top',
                            transform=axes[0, i].transAxes)

            plt.savefig(os.path.join(self.result_folder, fig_name))

            plt.show()

    def cal_r2_cost(self):
        if self.dode.config['use_car_link_tt']:
            print('----- car cost -----')
            print(self.true_car_cost)
            print(self.estimated_car_cost)
            print('----- car cost -----')
            ind = ~(np.isinf(self.true_car_cost) + np.isinf(self.estimated_car_cost) + np.isnan(self.true_car_cost) + np.isnan(self.estimated_car_cost))
            self.r2_car_cost = r2_score(self.true_car_cost[ind], self.estimated_car_cost[ind])

        if self.dode.config['use_truck_link_tt']:
            print('----- truck cost -----')
            print(self.true_truck_cost)
            print(self.estimated_truck_cost)
            print('----- truck cost -----')
            ind = ~(np.isinf(self.true_truck_cost) + np.isinf(self.estimated_truck_cost) + np.isnan(self.true_truck_cost) + np.isnan(self.estimated_truck_cost))
            self.r2_truck_cost = r2_score(self.true_truck_cost[ind], self.estimated_truck_cost[ind])

        print("r2 cost --- r2_car_cost: {}, r2_truck_cost: {}"
            .format(
                self.r2_car_cost, 
                self.r2_truck_cost
                ))

        return self.r2_car_cost, self.r2_truck_cost

    def scatter_plot_cost(self, fig_name = 'link_cost_scatterplot_pathflow.png'):
        if self.dode.config['use_car_link_tt'] + self.dode.config['use_truck_link_tt']:

            fig, axes = plt.subplots(1, 
                                    self.dode.config['use_car_link_tt'] + self.dode.config['use_truck_link_tt'], 
                                    figsize=(36, 9), dpi=300, squeeze=False)
            
            i = 0

            if self.dode.config['use_car_link_tt']:
                ind = ~(np.isinf(self.true_car_cost) + np.isinf(self.estimated_car_cost) + np.isnan(self.true_car_cost) + np.isnan(self.estimated_car_cost))
                car_tt_min = np.min((np.min(self.true_car_cost[ind]), np.min(self.estimated_car_cost[ind]))) - 1
                car_tt_max = np.max((np.max(self.true_car_cost[ind]), np.max(self.estimated_car_cost[ind]))) + 1
                axes[0, i].scatter(self.true_car_cost[ind], self.estimated_car_cost[ind], color = self.color_list[i], marker = self.marker_list[i], s = 100)
                axes[0, i].plot(np.linspace(car_tt_min, car_tt_max, 20), np.linspace(car_tt_min, car_tt_max, 20), color = 'gray')
                axes[0, i].set_ylabel('Estimated observed travel cost for car')
                axes[0, i].set_xlabel('True observed travel cost for car')
                axes[0, i].set_xlim([car_tt_min, car_tt_max])
                axes[0, i].set_ylim([car_tt_min, car_tt_max])
                axes[0, i].text(0, 1, 'r2 = {}'.format(self.r2_car_cost),
                            horizontalalignment='left',
                            verticalalignment='top',
                            transform=axes[0, i].transAxes)

            i += self.dode.config['use_car_link_tt']

            if self.dode.config['use_truck_link_tt']:
                ind = ~(np.isinf(self.true_truck_cost) + np.isinf(self.estimated_truck_cost) + np.isnan(self.true_truck_cost) + np.isnan(self.estimated_truck_cost))
                truck_tt_min = np.min((np.min(self.true_truck_cost[ind]), np.min(self.estimated_truck_cost[ind]))) - 1
                truck_tt_max = np.max((np.max(self.true_truck_cost[ind]), np.max(self.estimated_truck_cost[ind]))) + 1
                axes[0, i].scatter(self.true_truck_cost[ind], self.estimated_truck_cost[ind], color = self.color_list[i], marker = self.marker_list[i], s = 100)
                axes[0, i].plot(np.linspace(truck_tt_min, truck_tt_max, 20), np.linspace(truck_tt_min, truck_tt_max, 20), color = 'gray')
                axes[0, i].set_ylabel('Estimated observed travel cost for truck')
                axes[0, i].set_xlabel('True observed travel cost for truck')
                axes[0, i].set_xlim([truck_tt_min, truck_tt_max])
                axes[0, i].set_ylim([truck_tt_min, truck_tt_max])
                axes[0, i].text(0, 1, 'r2 = {}'.format(self.r2_truck_cost),
                            horizontalalignment='left',
                            verticalalignment='top',
                            transform=axes[0, i].transAxes)

            plt.savefig(os.path.join(self.result_folder, fig_name))

            plt.show()