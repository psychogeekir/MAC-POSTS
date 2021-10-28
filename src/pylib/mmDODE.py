import os
import numpy as np
import pandas as pd
from collections import OrderedDict
import hashlib
import time
import shutil
from scipy.sparse import coo_matrix, csr_matrix
import pickle
import multiprocessing as mp
import torch

import MNMAPI


class MMDODE:
    def __init__(self, nb, config):
        self.config = config
        self.nb = nb

        self.num_assign_interval = nb.config.config_dict['DTA']['max_interval']
        self.ass_freq = nb.config.config_dict['DTA']['assign_frq']

        self.num_link_driving = nb.config.config_dict['DTA']['num_of_link']
        self.num_link_bus = nb.config.config_dict['DTA']['num_of_bus_link']
        self.num_link_walking = nb.config.config_dict['DTA']['num_of_walking_link']

        self.num_path_driving = nb.config.config_dict['FIXED']['num_driving_path']
        self.num_path_bustransit = nb.config.config_dict['FIXED']['num_bustransit_path']
        self.num_path_pnr = nb.config.config_dict['FIXED']['num_pnr_path']
        self.num_path_busroute = nb.config.config_dict['FIXED']['num_bus_routes']

        # if nb.config.config_dict['DTA']['total_interval'] > 0 and nb.config.config_dict['DTA']['total_interval'] > self.num_assign_interval * self.ass_freq:
        #     self.num_loading_interval = nb.config.config_dict['DTA']['total_interval']
        # else:
        #     self.num_loading_interval = self.num_assign_interval * self.ass_freq  # not long enough
        self.num_loading_interval = self.num_assign_interval * self.ass_freq

        self.data_dict = dict()

        # number of observed data
        self.num_data = self.config['num_data']

        # observed link IDs, np.array
        self.observed_links_driving = self.config['observed_links_driving']
        self.observed_links_bus = self.config['observed_links_bus']
        self.observed_links_walking = self.config['observed_links_walking']

        # observed path IDs, np.array
        self.paths_list = self.config['paths_list']
        self.paths_list_driving = self.config['paths_list_driving']
        self.paths_list_bustransit = self.config['paths_list_bustransit']
        self.paths_list_pnr = self.config['paths_list_pnr']
        self.paths_list_busroute = self.config['paths_list_busroute']
        assert (len(self.paths_list_driving) == self.num_path_driving)
        assert (len(self.paths_list_bustransit) == self.num_path_bustransit)
        assert (len(self.paths_list_pnr) == self.num_path_pnr)
        assert (len(self.paths_list_busroute) == self.num_path_busroute)
        assert (len(self.paths_list) == self.num_path_driving + self.num_path_bustransit + self.num_path_pnr + self.num_path_busroute)

        # observed aggregated link count data, length = self.num_data
        # every element of the list is a np.array
        # num_aggregation x (num_links_driving x num_assign_interval)
        self.car_count_agg_L_list = None
        # num_aggregation x (num_links_driving x num_assign_interval)
        self.truck_count_agg_L_list = None
        # num_aggregation x (num_links_bus x num_assign_interval)
        self.bus_count_agg_L_list = None
        # num_aggregation x ((num_links_bus + num_links_walking) x num_assign_interval)
        self.passenger_count_agg_L_list = None

        # demand
        self.demand_list_total_passenger = self.nb.demand_total_passenger.demand_list
        self.demand_list_truck_driving = self.nb.demand_driving.demand_list
        
    def _add_car_link_flow_data(self, link_flow_df_list):
        # assert(self.config['use_car_link_flow'])
        assert (self.num_data == len(link_flow_df_list))
        self.data_dict['car_link_flow'] = link_flow_df_list

    def _add_truck_link_flow_data(self, link_flow_df_list):
        # assert(self.config['use_truck_link_flow'])
        assert (self.num_data == len(link_flow_df_list))
        self.data_dict['truck_link_flow'] = link_flow_df_list

    def _add_bus_link_flow_data(self, link_flow_df_list):
        # assert(self.config['use_bus_link_flow'])
        assert (self.num_data == len(link_flow_df_list))
        self.data_dict['bus_link_flow'] = link_flow_df_list
    
    def _add_passenger_link_flow_data(self, link_flow_df_list):
        # assert(self.config['use_passenger_link_flow'])
        assert (self.num_data == len(link_flow_df_list))
        self.data_dict['passenger_link_flow'] = link_flow_df_list

    def _add_car_link_tt_data(self, link_spd_df_list):
        # assert(self.config['use_car_link_tt'])
        assert (self.num_data == len(link_spd_df_list))
        self.data_dict['car_link_tt'] = link_spd_df_list

    def _add_truck_link_tt_data(self, link_spd_df_list):
        # assert(self.config['use_truck_link_tt'])
        assert (self.num_data == len(link_spd_df_list))
        self.data_dict['truck_link_tt'] = link_spd_df_list

    def _add_bus_link_tt_data(self, link_spd_df_list):
        # assert(self.config['use_bus_link_tt'])
        assert (self.num_data == len(link_spd_df_list))
        self.data_dict['bus_link_tt'] = link_spd_df_list

    def _add_passenger_link_tt_data(self, link_spd_df_list):
        # assert(self.config['use_passenger_link_tt'])
        assert (self.num_data == len(link_spd_df_list))
        self.data_dict['passenger_link_tt'] = link_spd_df_list

    def add_data(self, data_dict):
        if self.config['car_count_agg']:
            self.car_count_agg_L_list = data_dict['car_count_agg_L_list']
        if self.config['truck_count_agg']:
            self.truck_count_agg_L_list = data_dict['truck_count_agg_L_list']
        if self.config['bus_count_agg']:
            self.bus_count_agg_L_list = data_dict['bus_count_agg_L_list']
        if self.config['passenger_count_agg']:
            self.passenger_count_agg_L_list = data_dict['passenger_count_agg_L_list']
        
        if self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss']:
            self._add_car_link_flow_data(data_dict['car_link_flow'])
        if self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss'] :
            self._add_truck_link_flow_data(data_dict['truck_link_flow'])
        if self.config['use_bus_link_flow'] or self.config['compute_bus_link_flow_loss']:
            self._add_bus_link_flow_data(data_dict['bus_link_flow'])
        if self.config['use_passenger_link_flow'] or self.config['compute_passenger_link_flow_loss']:
            self._add_passenger_link_flow_data(data_dict['passenger_link_flow'])

        if self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss']:
            self._add_car_link_tt_data(data_dict['car_link_tt'])
        if self.config['use_truck_link_tt']or self.config['compute_car_link_tt_loss']:
            self._add_truck_link_tt_data(data_dict['truck_link_tt'])
        if self.config['use_bus_link_tt'] or self.config['compute_bus_link_tt_loss']:
            self._add_bus_link_tt_data(data_dict['bus_link_tt'])
        if self.config['use_passenger_link_tt']or self.config['compute_passenger_link_tt_loss']:
            self._add_passenger_link_tt_data(data_dict['passenger_link_tt'])

    def save_simulation_input_files(self, folder_path, f_car_driving=None, f_truck_driving=None, 
                                    f_passenger_bustransit=None, f_car_pnr=None):

        if not os.path.exists(folder_path):
            os.mkdir(folder_path)

        if (f_car_driving is not None) and (f_truck_driving is not None):
            self.nb.update_demand_path_driving(f_car_driving, f_truck_driving)
        if f_passenger_bustransit is not None:
            self.nb.update_demand_path_bustransit(f_passenger_bustransit)
        if f_car_pnr is not None:
            self.nb.update_demand_path_pnr(f_car_pnr)

        # self.nb.config.config_dict['DTA']['flow_scalar'] = 3
        self.nb.config.config_dict['DTA']['total_interval'] = self.num_loading_interval
        self.nb.config.config_dict['DTA']['routing_type'] = 'Multimodal_Hybrid'

        # no output files saved from DNL
        self.nb.config.config_dict['STAT']['rec_volume'] = 1
        self.nb.config.config_dict['STAT']['volume_load_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['volume_record_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['rec_tt'] = 1
        self.nb.config.config_dict['STAT']['tt_load_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['tt_record_automatic_rec'] = 0

        self.nb.dump_to_folder(folder_path)
        

    def _run_simulation(self, f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, counter=0):
        # print("Start simulation", time.time())
        hash1 = hashlib.sha1()
        # python 2
        # hash1.update(str(time.time()) + str(counter))
        # python 3
        hash1.update((str(time.time()) + str(counter)).encode('utf-8'))
        new_folder = str(hash1.hexdigest())

        self.save_simulation_input_files(new_folder, f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)

        a = MNMAPI.mmdta_api()
        a.initialize(new_folder)
        shutil.rmtree(new_folder)

        a.register_links_driving(self.observed_links_driving)
        a.register_links_bus(self.observed_links_bus)
        a.register_links_walking(self.observed_links_walking)
        a.register_paths(self.paths_list)

        a.install_cc()
        a.install_cc_tree()

        a.run_whole()
        # print("Finish simulation", time.time())

        travel_stats = a.get_travel_stats()
        print("\n************ travel stats ************")
        print("car count: {}".format(travel_stats[0]))
        print("truck count: {}".format(travel_stats[1]))
        print("bus count: {}".format(travel_stats[2]))
        print("passenger count: {}".format(travel_stats[3]))
        print("car total travel time (hours): {}".format(travel_stats[4]))
        print("truck total travel time (hours): {}".format(travel_stats[5]))
        print("bus total travel time (hours): {}".format(travel_stats[6]))
        print("passenger total travel time (hours): {}".format(travel_stats[7]))
        print("************ travel stats ************\n")
        return a  

    def get_car_dar_matrix_driving(self, dta, f):
        start_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        end_intervals = start_intervals + self.ass_freq
        assert(end_intervals[-1] <= self.num_loading_interval)
        raw_dar = dta.get_car_dar_matrix_driving(start_intervals, end_intervals)
        dar = self._massage_raw_dar(raw_dar, self.ass_freq, f, self.num_assign_interval, self.paths_list_driving, self.observed_links_driving)
        return dar

    def get_truck_dar_matrix_driving(self, dta, f):
        start_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        end_intervals = start_intervals + self.ass_freq
        assert(end_intervals[-1] <= self.num_loading_interval)
        raw_dar = dta.get_truck_dar_matrix_driving(start_intervals, end_intervals)
        dar = self._massage_raw_dar(raw_dar, self.ass_freq, f, self.num_assign_interval, self.paths_list_driving, self.observed_links_driving)
        return dar

    def get_car_dar_matrix_pnr(self, dta, f):
        start_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        end_intervals = start_intervals + self.ass_freq
        assert(end_intervals[-1] <= self.num_loading_interval)
        raw_dar = dta.get_car_dar_matrix_pnr(start_intervals, end_intervals)
        dar = self._massage_raw_dar(raw_dar, self.ass_freq, f, self.num_assign_interval, self.paths_list_pnr, self.observed_links_driving)
        return dar

    def get_bus_dar_matrix(self, dta, f):
        start_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        end_intervals = start_intervals + self.ass_freq
        assert(end_intervals[-1] <= self.num_loading_interval)
        raw_dar = dta.get_bus_dar_matrix(start_intervals, end_intervals)
        dar = self._massage_raw_dar(raw_dar, self.ass_freq, f, self.num_assign_interval, self.paths_list_busroute, self.observed_links_bus)
        return dar

    def get_passenger_dar_matrix_bustransit(self, dta, f):
        start_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        end_intervals = start_intervals + self.ass_freq
        assert(end_intervals[-1] <= self.num_loading_interval)
        raw_dar = dta.get_passenger_dar_matrix_bustransit(start_intervals, end_intervals)
        dar = self._massage_raw_dar(raw_dar, self.ass_freq, f, self.num_assign_interval, self.paths_list_bustransit, 
                                    np.concatenate((self.observed_links_bus, self.observed_links_walking)))
        return dar

    def get_passenger_dar_matrix_pnr(self, dta, f):
        start_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        end_intervals = start_intervals + self.ass_freq
        assert(end_intervals[-1] <= self.num_loading_interval)
        raw_dar = dta.get_passenger_dar_matrix_pnr(start_intervals, end_intervals)
        dar = self._massage_raw_dar(raw_dar, self.ass_freq, f, self.num_assign_interval, self.paths_list_pnr, 
                                    np.concatenate((self.observed_links_bus, self.observed_links_walking)))
        return dar

    def get_dar(self, dta, f_car_driving, f_truck_driving, f_bus, f_passenger_bustransit, f_car_pnr):
        
        car_dar_matrix_driving = csr_matrix((self.num_assign_interval * len(self.observed_links_driving), 
                                             self.num_assign_interval * len(self.paths_list_driving)))
        car_dar_matrix_pnr = csr_matrix((self.num_assign_interval * len(self.observed_links_driving), 
                                         self.num_assign_interval * len(self.paths_list_pnr)))
        truck_dar_matrix_driving = csr_matrix((self.num_assign_interval * len(self.observed_links_driving), 
                                               self.num_assign_interval * len(self.paths_list_driving)))
        bus_dar_matrix = csr_matrix((self.num_assign_interval * len(self.observed_links_bus), 
                                     self.num_assign_interval * len(self.paths_list_busroute)))
        passenger_dar_matrix_bustransit = csr_matrix((self.num_assign_interval * (len(self.observed_links_bus) + len(self.observed_links_walking)), 
                                                      self.num_assign_interval * len(self.paths_list_bustransit)))
        passenger_dar_matrix_pnr = csr_matrix((self.num_assign_interval * (len(self.observed_links_bus) + len(self.observed_links_walking)), 
                                               self.num_assign_interval * len(self.paths_list_pnr)))
        
        if self.config['use_car_link_flow'] or self.config['use_car_link_tt']:
            car_dar_matrix_driving = self.get_car_dar_matrix_driving(dta, f_car_driving)   
            car_dar_matrix_pnr = self.get_car_dar_matrix_pnr(dta, f_car_pnr)
            
        if self.config['use_truck_link_flow'] or self.config['use_truck_link_tt']:
            truck_dar_matrix_driving = self.get_truck_dar_matrix_driving(dta, f_truck_driving)

        if self.config['use_bus_link_flow'] or self.config['use_bus_link_tt']:
            bus_dar_matrix = self.get_bus_dar_matrix(dta, f_bus)

        if self.config['use_passenger_link_flow'] or self.config['use_passenger_link_tt']:
            passenger_dar_matrix_bustransit = self.get_passenger_dar_matrix_bustransit(dta, f_passenger_bustransit)
            passenger_dar_matrix_pnr = self.get_passenger_dar_matrix_pnr(dta, f_car_pnr)
            
        return car_dar_matrix_driving, truck_dar_matrix_driving, car_dar_matrix_pnr, bus_dar_matrix, \
               passenger_dar_matrix_bustransit, passenger_dar_matrix_pnr

    def _massage_raw_dar(self, raw_dar, ass_freq, f, num_assign_interval, paths_list, observed_links):
        assert(raw_dar.shape[1] == 5)
        if raw_dar.shape[0] == 0:
            print("No dar. Consider increase the demand values")
            return csr_matrix((num_assign_interval * len(observed_links), 
                               num_assign_interval * len(paths_list)))

        num_e_path = len(paths_list)
        num_e_link = len(observed_links)
        # 15 min
        small_assign_freq = ass_freq * self.nb.config.config_dict['DTA']['unit_time'] / 60
        # raw_dar[:, 2]: link no.
        # raw_dar[:, 3]: the count of unit time interval (5s)
        if type(observed_links) == np.ndarray:
            # In Python 3, map() returns an iterable while, in Python 2, it returns a list.
            link_seq = (np.array(list(map(lambda x: np.where(observed_links == x)[0][0], raw_dar[:, 2].astype(int))))
                        + (raw_dar[:, 3] / ass_freq).astype(int) * num_e_link).astype(int)
        elif type(observed_links) == list:
            link_seq = (np.array(list(map(lambda x: observed_links.index(x), raw_dar[:, 2].astype(int))))
                        + (raw_dar[:, 3] / ass_freq).astype(int) * num_e_link).astype(int)
        # raw_dar[:, 0]: path no.
        # raw_dar[:, 1]: the count of 1 min interval
        if type(paths_list) == np.ndarray:
            path_seq = (np.array(list(map(lambda x: np.where(paths_list == x)[0][0], raw_dar[:, 0].astype(int))))
                        + (raw_dar[:, 1] / small_assign_freq).astype(int) * num_e_path).astype(int)
        elif type(paths_list) == list:
            path_seq = (np.array(list(map(lambda x: paths_list.index(x), raw_dar[:, 0].astype(int))))
                        + (raw_dar[:, 1] / small_assign_freq).astype(int) * num_e_path).astype(int)
        # print(path_seq)
        # raw_dar[:, 4]: flow
        p = raw_dar[:, 4] / f[path_seq]
        # print("Creating the coo matrix", time.time())
        mat = coo_matrix((p, (link_seq, path_seq)), shape=(num_assign_interval * num_e_link, num_assign_interval * num_e_path))
        # pickle.dump((p, link_seq, path_seq), open('test.pickle', 'wb'))
        # print('converting the csr', time.time())
        mat = mat.tocsr()
        # print('finish converting', time.time())
        return mat

    def init_demand_flow(self, num_OD, init_scale=0.1):
        # demand matrix (num_OD x num_assign_interval) flattened in F order
        return np.random.rand(self.num_assign_interval * num_OD) * init_scale

    def init_path_flow(self, car_driving_scale=1, truck_driving_scale=0.1, passenger_bustransit_scale=1, car_pnr_scale=0.5):
        f_car_driving = np.random.rand(self.num_assign_interval * self.num_path_driving) * car_driving_scale
        f_truck_driving = np.random.rand(self.num_assign_interval * self.num_path_driving) * truck_driving_scale
        f_passenger_bustransit = np.random.rand(self.num_assign_interval * self.num_path_bustransit) * passenger_bustransit_scale
        f_car_pnr = np.random.rand(self.num_assign_interval * self.num_path_pnr) * car_pnr_scale
        return f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr

    def compute_path_cost(self, dta):
        start_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)

        # for existing paths
        path_cost = dta.get_registered_path_cost_driving(start_intervals)
        assert(path_cost.shape[0] == self.num_path_driving)
        path_tt = dta.get_registered_path_tt_truck(start_intervals)
        assert(path_tt.shape[0] == self.num_path_driving)
        for i, path_ID in enumerate(self.nb.path_table_driving.ID2path.keys()):
            self.nb.path_table_driving.ID2path[path_ID].path_cost_car = path_cost[i, :]
            self.nb.path_table_driving.ID2path[path_ID].path_cost_truck = path_tt[i, :]
        
        path_cost = dta.get_registered_path_cost_bustransit(start_intervals)
        assert(path_cost.shape[0] == self.num_path_bustransit)
        for i, path_ID in enumerate(self.nb.path_table_bustransit.ID2path.keys()):
            self.nb.path_table_bustransit.ID2path[path_ID].path_cost = path_cost[i, :]
        
        path_cost = dta.get_registered_path_cost_pnr(start_intervals)
        assert(path_cost.shape[0] == self.num_path_pnr)
        for i, path_ID in enumerate(self.nb.path_table_pnr.ID2path.keys()):
            self.nb.path_table_pnr.ID2path[path_ID].path_cost = path_cost[i, :]

    def compute_path_flow_grad_and_loss(self, one_data_dict, f_car_driving, f_truck_driving, f_bus, 
                                        f_passenger_bustransit, f_car_pnr, counter=0):
        # print("Running simulation", time.time())
        dta = self._run_simulation(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, counter)

        # print("Getting DAR", time.time())
        car_dar_matrix_driving, truck_dar_matrix_driving, car_dar_matrix_pnr, bus_dar_matrix, \
               passenger_dar_matrix_bustransit, passenger_dar_matrix_pnr = \
                   self.get_dar(dta, f_car_driving, f_truck_driving, f_bus, f_passenger_bustransit, f_car_pnr)
        # print("Evaluating grad", time.time())

        # derivative of loss with respect to link flow
        car_grad = np.zeros(len(self.observed_links_driving) * self.num_assign_interval)
        truck_grad = np.zeros(len(self.observed_links_driving) * self.num_assign_interval)
        # bus_grad = np.zeros(len(self.observed_links_bus) * self.num_assign_interval)
        passenger_grad = np.zeros((len(self.observed_links_bus) + len(self.observed_links_walking)) * self.num_assign_interval)
        
        if self.config['use_car_link_flow']:
            car_grad += self.config['link_car_flow_weight'] * self._compute_grad_on_car_link_flow(dta, one_data_dict)
        if self.config['use_truck_link_flow']:
            truck_grad += self.config['link_truck_flow_weight'] * self._compute_grad_on_truck_link_flow(dta, one_data_dict)
        if self.config['use_passenger_link_flow']:
            passenger_grad += self.config['link_passenger_flow_weight'] * self._compute_grad_on_passenger_link_flow(dta, one_data_dict)

        if self.config['use_car_link_tt']:
            car_grad += self.config['link_car_tt_weight'] * self._compute_grad_on_car_link_tt(dta, one_data_dict)
        if self.config['use_truck_link_tt']:
            truck_grad += self.config['link_truck_tt_weight'] * self._compute_grad_on_truck_link_tt(dta, one_data_dict)
        if self.config['use_passenger_link_tt']:
            passenger_grad += self.config['link_passenger_tt_weight'] * self._compute_grad_on_passenger_link_tt(dta, one_data_dict)

        # derivative of loss with respect to path flow
        f_car_driving_grad = car_dar_matrix_driving.T.dot(car_grad)
        f_truck_driving_grad = truck_dar_matrix_driving.T.dot(truck_grad)
        f_passenger_bustransit_grad = passenger_dar_matrix_bustransit.T.dot(passenger_grad)
        f_car_pnr_grad = car_dar_matrix_pnr.T.dot(car_grad)
        f_passenger_pnr_grad = passenger_dar_matrix_pnr.T.dot(passenger_grad)

        # print("Getting Loss", time.time())
        total_loss, loss_dict = self._get_loss(one_data_dict, dta)

        return f_car_driving_grad, f_truck_driving_grad, f_passenger_bustransit_grad, f_car_pnr_grad, f_passenger_pnr_grad, \
               total_loss, loss_dict, dta

    def _compute_grad_on_car_link_flow(self, dta, one_data_dict):
        link_flow_array = one_data_dict['car_link_flow']
        # num_links_driving x num_assign_interval flatened in F order
        x_e = dta.get_link_car_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                      np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        assert(len(x_e) == len(self.observed_links_driving) * self.num_assign_interval)
        # print("x_e", x_e, link_flow_array)
        if self.config['car_count_agg']:
            x_e = one_data_dict['car_count_agg_L'].dot(x_e)
        grad = -np.nan_to_num(link_flow_array - x_e)
        if self.config['car_count_agg']:
            grad = one_data_dict['car_count_agg_L'].T.dot(grad)
        # print("final link grad", grad)
        return grad

    def _compute_grad_on_truck_link_flow(self, dta, one_data_dict):
        link_flow_array = one_data_dict['truck_link_flow']
        # num_links_driving x num_assign_interval flatened in F order
        x_e = dta.get_link_truck_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                        np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        assert(len(x_e) == len(self.observed_links_driving) * self.num_assign_interval)
        # print("x_e", x_e, link_flow_array)
        if self.config['truck_count_agg']:
            x_e = one_data_dict['truck_count_agg_L'].dot(x_e)
        grad = -np.nan_to_num(link_flow_array - x_e)
        if self.config['truck_count_agg']:
            grad = one_data_dict['truck_count_agg_L'].T.dot(grad)
        # print("final link grad", grad)
        return grad

    def _compute_grad_on_passenger_link_flow(self, dta, one_data_dict):
        link_flow_array = one_data_dict['passenger_link_flow']
        # (num_links_bus + num_links_walking)  x num_assign_interval flatened in F order
        x_e_bus_passenger = dta.get_link_bus_passenger_inflow(
            np.arange(0, self.num_loading_interval, self.ass_freq), 
            np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
        assert(x_e_bus_passenger.shape[0] == len(self.observed_links_bus))
        x_e_walking_passenger = dta.get_link_walking_passenger_inflow(
            np.arange(0, self.num_loading_interval, self.ass_freq), 
            np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
        assert(x_e_walking_passenger.shape[0] == len(self.observed_links_walking))
        x_e = np.concatenate((x_e_bus_passenger, x_e_walking_passenger), axis=0).flatten(order='F')
        # print("x_e", x_e, link_flow_array)
        if self.config['passenger_count_agg']:
            x_e = one_data_dict['passenger_count_agg_L'].dot(x_e)
        grad = -np.nan_to_num(link_flow_array - x_e)
        if self.config['passenger_count_agg']:
            grad = one_data_dict['passenger_count_agg_L'].T.dot(grad)
        # print("final link grad", grad)
        return grad

    def _compute_grad_on_car_link_tt(self, dta, one_data_dict):
        # tt_e = dta.get_car_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
        #                                   np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        tt_e = dta.get_car_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
        assert(len(tt_e) == len(self.observed_links_driving) * self.num_assign_interval)
        tt_free = np.tile(list(map(lambda x: self.nb.get_link_driving(x).get_car_fft(), self.observed_links_driving)), (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['car_link_tt'], tt_free) 
        # tt_o = one_data_dict['car_link_tt']
        # print('o-----', tt_o)
        # print('e-----', tt_e)
        grad = -np.nan_to_num(tt_o - tt_e)/tt_e
        # print('g-----', grad)
        # if self.config['car_count_agg']:
        #   grad = one_data_dict['car_count_agg_L'].T.dot(grad)
        # print(tt_e, tt_o)
        # print("car_grad", grad)
        return grad

    def _compute_grad_on_truck_link_tt(self, dta, one_data_dict):
        tt_e = dta.get_truck_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
        assert(len(tt_e) == len(self.observed_links_driving) * self.num_assign_interval)
        tt_free = np.tile(list(map(lambda x: self.nb.get_link_driving(x).get_truck_fft(), self.observed_links_driving)), (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['truck_link_tt'], tt_free)
        grad = -np.nan_to_num(tt_o - tt_e)/tt_e
        # if self.config['truck_count_agg']:
        #   grad = one_data_dict['truck_count_agg_L'].T.dot(grad)
        # print("truck_grad", grad)
        return grad

    def _compute_grad_on_passenger_link_tt(self, dta, one_data_dict):
        tt_e_bus = dta.get_bus_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq))
        assert(tt_e_bus.shape[0] == len(self.observed_links_bus))
        tt_e_walking = dta.get_passenger_walking_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq))
        assert(tt_e_walking.shape[0] == len(self.observed_links_walking))
        tt_e = np.concatenate((tt_e_bus, tt_e_walking), axis=0).flatten(order='F')
        tt_free = np.tile(list(map(lambda x: self.nb.get_link_bus(x).get_bus_fft(), self.observed_links_bus)) + 
                          list(map(lambda x: self.nb.get_link_walking(x).get_walking_fft(), self.observed_links_walking)), 
                          (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['passenger_link_tt'], tt_free)
        grad = -np.nan_to_num(tt_o - tt_e)/tt_e
        # if self.config['passenger_count_agg']:
        #   grad = one_data_dict['passenger_count_agg_L'].T.dot(grad)
        # print("passenger_grad", grad)
        return grad

    # def _compute_grad_on_bus_link_tt(self, dta, one_data_dict):
    #     tt_e = dta.get_bus_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
    #     tt_free = np.tile(list(map(lambda x: self.nb.get_link_bus(x).get_bus_fft(), self.observed_links_bus)), (self.num_assign_interval))
    #     tt_e = np.maximum(tt_e, tt_free)
    #     tt_o = np.maximum(one_data_dict['bus_link_tt'], tt_free)
    #     grad = -np.nan_to_num(tt_o - tt_e)/tt_e
    #     # if self.config['bus_count_agg']:
    #     #   grad = one_data_dict['bus_count_agg_L'].T.dot(grad)
    #     # print("bus_grad", grad)
    #     return grad

    # def _compute_grad_on_walking_link_tt(self, dta, one_data_dict):
    #     tt_e = dta.get_passenger_walking_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
    #     tt_free = np.tile(list(map(lambda x: self.nb.get_link_walking(x).get_walking_fft(), self.observed_links_walking)), (self.num_assign_interval))
    #     tt_e = np.maximum(tt_e, tt_free)
    #     tt_o = np.maximum(one_data_dict['walking_link_tt'], tt_free)
    #     grad = -np.nan_to_num(tt_o - tt_e)/tt_e
    #     # if self.config['bus_count_agg']:
    #     #   grad = one_data_dict['bus_count_agg_L'].T.dot(grad)
    #     # print("bus_grad", grad)
    #     return grad

    def _get_one_data(self, j):
        assert (self.num_data > j)
        one_data_dict = dict()
        if self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss']:
            one_data_dict['car_link_flow'] = self.data_dict['car_link_flow'][j]
        if self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss']:
            one_data_dict['truck_link_flow'] = self.data_dict['truck_link_flow'][j]
        if self.config['use_bus_link_flow']or self.config['compute_bus_link_flow_loss']:
            one_data_dict['bus_link_flow'] = self.data_dict['bus_link_flow'][j]
        if self.config['use_passenger_link_flow']or self.config['compute_passenger_link_flow_loss']:
            one_data_dict['passenger_link_flow'] = self.data_dict['passenger_link_flow'][j]

        if self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss']:
            one_data_dict['car_link_tt'] = self.data_dict['car_link_tt'][j]
        if self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss']:
            one_data_dict['truck_link_tt'] = self.data_dict['truck_link_tt'][j]
        if self.config['use_bus_link_tt'] or self.config['compute_bus_link_tt_loss']:
            one_data_dict['bus_link_tt'] = self.data_dict['bus_link_tt'][j]
        if self.config['use_passenger_link_tt'] or self.config['compute_passenger_link_tt_loss']:
            one_data_dict['passenger_link_tt'] = self.data_dict['passenger_link_tt'][j]

        if self.config['car_count_agg']:
            one_data_dict['car_count_agg_L'] = self.car_count_agg_L_list[j]
        if self.config['truck_count_agg']:
            one_data_dict['truck_count_agg_L'] = self.truck_count_agg_L_list[j]
        if self.config['bus_count_agg']:
            one_data_dict['bus_count_agg_L'] = self.bus_count_agg_L_list[j]
        if self.config['passenger_count_agg']:
            one_data_dict['passenger_count_agg_L'] = self.passenger_count_agg_L_list[j]
        return one_data_dict

    def _get_loss(self, one_data_dict, dta):
        start_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        end_intervals = start_intervals + self.ass_freq

        loss_dict = dict()

        # flow loss

        if self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss']:
            # num_links_driving x num_assign_intervals
            x_e = dta.get_link_car_inflow(start_intervals, end_intervals).flatten(order='F')
            assert(len(x_e) == len(self.observed_links_driving) * self.num_assign_interval)
            if self.config['car_count_agg']:
                x_e = one_data_dict['car_count_agg_L'].dot(x_e)
            loss = self.config['link_car_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['car_link_flow']))
            loss_dict['car_count_loss'] = loss

        if self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss']:
            # num_links_driving x num_assign_intervals
            x_e = dta.get_link_truck_inflow(start_intervals, end_intervals).flatten(order='F')
            assert(len(x_e) == len(self.observed_links_driving) * self.num_assign_interval)
            if self.config['truck_count_agg']:
                x_e = one_data_dict['truck_count_agg_L'].dot(x_e)
            loss = self.config['link_truck_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['truck_link_flow']))
            loss_dict['truck_count_loss'] = loss

        if self.config['use_bus_link_flow'] or self.config['compute_bus_link_flow_loss']:
            # num_links_bus x num_assign_intervals
            x_e = dta.get_link_bus_inflow(start_intervals, end_intervals).flatten(order='F')
            assert(len(x_e) == len(self.observed_links_bus) * self.num_assign_interval)
            if self.config['bus_count_agg']:
                x_e = one_data_dict['bus_count_agg_L'].dot(x_e)
            loss = self.config['link_bus_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['bus_link_flow']))
            loss_dict['bus_count_loss'] = loss

        if self.config['use_passenger_link_flow'] or self.config['compute_passenger_link_flow_loss']:
            # (num_links_bus + num_links_walking) x num_assign_intervals
            x_e_bus_passenger = dta.get_link_bus_passenger_inflow(start_intervals, end_intervals)
            assert(x_e_bus_passenger.shape[0] == len(self.observed_links_bus))
            x_e_walking_passenger = dta.get_link_walking_passenger_inflow(start_intervals, end_intervals)
            assert(x_e_walking_passenger.shape[0] == len(self.observed_links_walking))
            x_e = np.concatenate((x_e_bus_passenger, x_e_walking_passenger), axis=0).flatten(order='F')
            if self.config['passenger_count_agg']:
                x_e = one_data_dict['passenger_count_agg_L'].dot(x_e)
            loss = self.config['link_passenger_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['passenger_link_flow']))
            loss_dict['passenger_count_loss'] = loss

        # if self.config['use_bus_link_passenger_flow'] or self.config['compute_bus_link_passenger_flow_loss']:
        #     # num_links_bus x num_assign_intervals
        #     x_e = dta.get_link_bus_passenger_inflow(start_intervals, end_intervals).flatten(order='F')
        #     if self.config['bus_passenger_count_agg']:
        #         x_e = one_data_dict['bus_passenger_count_agg_L'].dot(x_e)
        #     loss = self.config['link_bus_passenger_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['bus_link_passenger_flow']))
        #     loss_dict['bus_passenger_count_loss'] = loss

        # if self.config['use_walking_link_passenger_flow'] or self.config['compute_walking_link_passenger_flow_loss']:
        #     # num_links_walking x num_assign_intervals
        #     x_e = dta.get_link_walking_passenger_inflow(start_intervals, end_intervals).flatten(order='F')
        #     if self.config['walking_passenger_count_agg']:
        #         x_e = one_data_dict['walking_passenger_count_agg_L'].dot(x_e)
        #     loss = self.config['link_walking_passenger_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['walking_link_passenger_flow']))
        #     loss_dict['walking_passenger_count_loss'] = loss

        # travel time loss

        if self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss']:
            # num_links_driving x num_assign_intervals
            x_tt_e = dta.get_car_link_tt(start_intervals).flatten(order='F')
            # x_tt_e = dta.get_car_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
            #                                     np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
            assert(len(x_tt_e) == len(self.observed_links_driving) * self.num_assign_interval)
            loss = self.config['link_car_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['car_link_tt']))
            loss_dict['car_tt_loss'] = loss
            
        if self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss']:
            # num_links_driving x num_assign_intervals
            x_tt_e = dta.get_truck_link_tt(start_intervals).flatten(order='F')
            assert(len(x_tt_e) == len(self.observed_links_driving) * self.num_assign_interval)
            loss = self.config['link_truck_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['truck_link_tt']))
            loss_dict['truck_tt_loss'] = loss

        if self.config['use_bus_link_tt'] or self.config['compute_bus_link_tt_loss']:
            # num_links_bus x num_assign_intervals
            x_tt_e = dta.get_bus_link_tt(start_intervals).flatten(order='F')
            assert(len(x_tt_e) == len(self.observed_links_bus) * self.num_assign_interval)
            loss = self.config['link_bus_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['bus_link_tt']))
            loss_dict['bus_tt_loss'] = loss
            
        if self.config['use_passenger_link_tt'] or self.config['compute_passenger_link_tt_loss']:
            # (num_links_bus + num_links_walking) x num_assign_intervals
            x_tt_e_bus = dta.get_bus_link_tt(start_intervals)
            assert(x_tt_e_bus.shape[0] == len(self.observed_links_bus))
            x_tt_e_walking = dta.get_passenger_walking_link_tt(start_intervals)
            assert(x_tt_e_walking.shape[0] == len(self.observed_links_walking))
            x_tt_e = np.concatenate((x_tt_e_bus, x_tt_e_walking), axis=0).flatten(order='F')
            loss = self.config['link_passenger_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['passenger_link_tt']))
            loss_dict['passenger_tt_loss'] = loss

        total_loss = 0.0
        for loss_type, loss_value in loss_dict.items():
            total_loss += loss_value
        return total_loss, loss_dict
        
    def estimate_path_flow_pytorch(self, car_driving_scale=10, truck_driving_scale=1, passenger_bustransit_scale=1, car_pnr_scale=5,
                                   car_driving_step_size=0.1, truck_driving_step_size=0.01, passenger_bustransit_step_size=0.01, car_pnr_step_size=0.05,
                                   max_epoch=100, column_generation=False, use_file_as_init=None, save_folder=None):
        
        if np.isscalar(column_generation):
            column_generation = np.ones(max_epoch, dtype=int) * column_generation
        assert(len(column_generation) == max_epoch)
        
        loss_list = list()

        # read from files as init values
        if use_file_as_init is not None:
            _, _, f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = pickle.load(use_file_as_init, 'rb')

            self.nb.update_demand_path_driving(f_car_driving, f_truck_driving)
            self.nb.update_demand_path_bustransit(f_passenger_bustransit)
            self.nb.update_demand_path_pnr(f_car_pnr)
        else:
            f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
                self.init_path_flow(car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)

        # fixed bus path flow
        f_bus = self.nb.demand_bus.path_flow_matrix.flatten(order='F')

        f_car_driving_tensor = torch.from_numpy(f_car_driving)
        f_truck_driving_tensor = torch.from_numpy(f_truck_driving)
        f_passenger_bustransit_tensor = torch.from_numpy(f_passenger_bustransit)
        f_car_pnr_tensor = torch.from_numpy(f_car_pnr)

        f_car_driving_tensor.requires_grad = True
        f_truck_driving_tensor.requires_grad = True
        f_passenger_bustransit_tensor.requires_grad = True
        f_car_pnr_tensor.requires_grad = True

        optimizer = torch.optim.Adam([
            {'params': f_car_driving_tensor, 'lr': car_driving_step_size},
            {'params': f_truck_driving_tensor, 'lr': truck_driving_step_size},
            {'params': f_passenger_bustransit_tensor, 'lr': passenger_bustransit_step_size},
            {'params': f_car_pnr_tensor, 'lr': car_pnr_step_size}
        ], betas=(0.99, 0.99))
        
        for i in range(max_epoch):
            seq = np.random.permutation(self.num_data)
            loss = np.float(0)
        
            for j in seq:
                # retrieve one record of observed data
                one_data_dict = self._get_one_data(j)

                # f_grad: num_path * num_assign_interval
                f_car_driving_grad, f_truck_driving_grad, f_passenger_bustransit_grad, f_car_pnr_grad, f_passenger_pnr_grad, \
                    tmp_loss, loss_dict, dta = \
                        self.compute_path_flow_grad_and_loss(one_data_dict, f_car_driving, f_truck_driving, f_bus, f_passenger_bustransit, f_car_pnr)
                
                optimizer.zero_grad()

                f_car_driving_tensor.grad = torch.from_numpy(f_car_driving_grad)
                f_truck_driving_tensor.grad = torch.from_numpy(f_truck_driving_grad)
                f_passenger_bustransit_tensor.grad = torch.from_numpy(f_passenger_bustransit_grad)
                f_car_pnr_tensor.grad = torch.from_numpy(f_car_pnr_grad + f_passenger_pnr_grad)

                optimizer.step()

                f_car_driving = f_car_driving_tensor.data.cpu().numpy()
                f_truck_driving = f_truck_driving_tensor.data.cpu().numpy()
                f_passenger_bustransit = f_passenger_bustransit_tensor.data.cpu().numpy()
                f_car_pnr = f_car_pnr_tensor.data.cpu().numpy()

                if column_generation[i]:
                    self.update_path_table(dta)
                    f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
                        self.update_path_flow(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
                                              car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)
                
                f_car_driving = np.maximum(f_car_driving, 1e-6)
                f_truck_driving = np.maximum(f_truck_driving, 1e-6)
                f_passenger_bustransit = np.maximum(f_passenger_bustransit, 1e-6)
                f_car_pnr = np.maximum(f_car_pnr, 1e-6)

                if column_generation[i]:
                    f_car_driving_tensor = torch.from_numpy(f_car_driving)
                    f_truck_driving_tensor = torch.from_numpy(f_truck_driving)
                    f_passenger_bustransit_tensor = torch.from_numpy(f_passenger_bustransit)
                    f_car_pnr_tensor = torch.from_numpy(f_car_pnr)

                    f_car_driving_tensor.requires_grad = True
                    f_truck_driving_tensor.requires_grad = True
                    f_passenger_bustransit_tensor.requires_grad = True
                    f_car_pnr_tensor.requires_grad = True

                    optimizer = torch.optim.Adam([
                        {'params': f_car_driving_tensor, 'lr': car_driving_step_size},
                        {'params': f_truck_driving_tensor, 'lr': truck_driving_step_size},
                        {'params': f_passenger_bustransit_tensor, 'lr': passenger_bustransit_step_size},
                        {'params': f_car_pnr_tensor, 'lr': car_pnr_step_size}
                    ], betas=(0.99, 0.99))

                loss += tmp_loss
            
            print("Epoch:", i, "Loss:", loss / np.float(self.num_data), self.print_separate_accuracy(loss_dict))
            loss_list.append([loss / np.float(self.num_data), loss_dict])
            if save_folder is not None:
                pickle.dump([loss / np.float(self.num_data), loss_dict, 
                             f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr], 
                            open(os.path.join(save_folder, str(i)+'_iteration_estimate_path_flow.pickle'), 'wb'))
                
                if column_generation[i]:
                    self.save_simulation_input_files(os.path.join(save_folder, 'input_files_estimate_path_flow'), 
                                                     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)

            # if column_generation[i]:
            #     self.update_path_table(dta)
            #     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
            #         self.update_path_flow(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
            #                               car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)

        return f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, loss_list
        

    def estimate_path_flow(self, car_driving_scale=10, truck_driving_scale=1, passenger_bustransit_scale=1, car_pnr_scale=5,
                           car_driving_step_size=0.1, truck_driving_step_size=0.01, passenger_bustransit_step_size=0.01, car_pnr_step_size=0.05,
                           max_epoch=100, adagrad=False, column_generation=False, use_file_as_init=None, save_folder=None):
        
        if np.isscalar(column_generation):
            column_generation = np.ones(max_epoch, dtype=int) * column_generation
        assert(len(column_generation) == max_epoch)
        
        loss_list = list()

        # read from files as init values
        if use_file_as_init is not None:
            _, _, f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = pickle.load(use_file_as_init, 'rb')

            self.nb.update_demand_path_driving(f_car_driving, f_truck_driving)
            self.nb.update_demand_path_bustransit(f_passenger_bustransit)
            self.nb.update_demand_path_pnr(f_car_pnr)
        else:
            f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
                self.init_path_flow(car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)

        # fixed bus path flow
        f_bus = self.nb.demand_bus.path_flow_matrix.flatten(order='F')

        for i in range(max_epoch):
            seq = np.random.permutation(self.num_data)
            loss = np.float(0)
            if adagrad:
                sum_g_square_car_driving = 1e-6
                sum_g_square_truck_driving = 1e-6
                sum_g_square_passenger_bustransit = 1e-6
                sum_g_square_car_pnr = 1e-6
            for j in seq:
                # retrieve one record of observed data
                one_data_dict = self._get_one_data(j)

                # f_grad: num_path * num_assign_interval
                f_car_driving_grad, f_truck_driving_grad, f_passenger_bustransit_grad, f_car_pnr_grad, f_passenger_pnr_grad, \
                    tmp_loss, loss_dict, dta = \
                        self.compute_path_flow_grad_and_loss(one_data_dict, f_car_driving, f_truck_driving, f_bus, f_passenger_bustransit, f_car_pnr)
                
                if adagrad:
                    sum_g_square_car_driving = sum_g_square_car_driving + np.power(f_car_driving_grad, 2)
                    f_car_driving -= f_car_driving_grad * car_driving_step_size / np.sqrt(sum_g_square_car_driving)

                    sum_g_square_truck_driving = sum_g_square_truck_driving + np.power(f_truck_driving_grad, 2)
                    f_truck_driving -= f_truck_driving_grad * truck_driving_step_size / np.sqrt(sum_g_square_truck_driving)

                    sum_g_square_passenger_bustransit = sum_g_square_passenger_bustransit + np.power(f_passenger_bustransit_grad, 2)
                    f_passenger_bustransit -= f_passenger_bustransit_grad * passenger_bustransit_step_size / np.sqrt(sum_g_square_passenger_bustransit)

                    sum_g_square_car_pnr = sum_g_square_car_pnr + np.power((f_car_pnr_grad + f_passenger_pnr_grad), 2)
                    f_car_pnr -= (f_car_pnr_grad + f_passenger_pnr_grad) * car_pnr_step_size / np.sqrt(sum_g_square_car_pnr)
                else:
                    f_car_driving -= f_car_driving_grad * car_driving_step_size / np.sqrt(i+1)
                    f_truck_driving -= f_truck_driving_grad * truck_driving_step_size / np.sqrt(i+1)
                    f_passenger_bustransit -= f_passenger_bustransit_grad * passenger_bustransit_step_size / np.sqrt(i+1)
                    f_car_pnr -= (f_car_pnr_grad + f_passenger_pnr_grad) * car_pnr_step_size / np.sqrt(i+1)

                if column_generation[i]:
                    self.update_path_table(dta)
                    f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
                        self.update_path_flow(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
                                              car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)
                
                f_car_driving = np.maximum(f_car_driving, 1e-6)
                f_truck_driving = np.maximum(f_truck_driving, 1e-6)
                f_passenger_bustransit = np.maximum(f_passenger_bustransit, 1e-6)
                f_car_pnr = np.maximum(f_car_pnr, 1e-6)

                loss += tmp_loss
            
            print("Epoch:", i, "Loss:", loss / np.float(self.num_data), self.print_separate_accuracy(loss_dict))
            loss_list.append([loss / np.float(self.num_data), loss_dict])
            if save_folder is not None:
                pickle.dump([loss / np.float(self.num_data), loss_dict, 
                             f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr], 
                            open(os.path.join(save_folder, str(i)+'_iteration_estimate_path_flow.pickle'), 'wb'))
                
                if column_generation[i]:
                    self.save_simulation_input_files(os.path.join(save_folder, 'input_files_estimate_path_flow'), 
                                                     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)

            # if column_generation[i]:
            #     self.update_path_table(dta)
            #     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
            #         self.update_path_flow(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
            #                               car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)

        return f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, loss_list

    def estimate_demand_pytorch(self, init_scale_passenger=10, init_scale_truck=10, 
                                car_driving_scale=10, truck_driving_scale=1, passenger_bustransit_scale=1, car_pnr_scale=5,
                                passenger_step_size=0.1, truck_step_size=0.01,
                                max_epoch=100, column_generation=False,
                                alpha_mode=(1., 1.5, 2.), beta_mode=1, alpha_path=1, beta_path=1, 
                                use_file_as_init=None, save_folder=None):

        if np.isscalar(column_generation):
            column_generation = np.ones(max_epoch, dtype=int) * column_generation
        assert(len(column_generation) == max_epoch)
        
        loss_list = list()

        # read from files as init values
        if use_file_as_init is not None:
            _, _, q_e_passenger, q_e_truck, q_e_mode_driving, q_e_mode_bustransit, q_e_mode_pnr, \
                f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = pickle.load(use_file_as_init, 'rb')
            
            self.nb.update_demand_path_driving(f_car_driving, f_truck_driving)
            self.nb.update_demand_path_bustransit(f_passenger_bustransit)
            self.nb.update_demand_path_pnr(f_car_pnr)
        else:
            # q_e: num_OD x num_assign_interval flattened in F order
            q_e_passenger = self.init_demand_flow(len(self.demand_list_total_passenger), init_scale=init_scale_passenger)
            q_e_truck = self.init_demand_flow(len(self.demand_list_truck_driving), init_scale=init_scale_truck)

            # uniform
            self.init_mode_route_portions()

        # fixed bus path flow
        f_bus = self.nb.demand_bus.path_flow_matrix.flatten(order='F')

        q_e_passenger_tensor = torch.from_numpy(q_e_passenger)
        q_e_truck_tensor = torch.from_numpy(q_e_truck)

        q_e_passenger_tensor.requires_grad = True
        q_e_truck_tensor.requires_grad = True

        optimizer = torch.optim.Adam([
            {'params': q_e_passenger_tensor, 'lr': passenger_step_size},
            {'params': q_e_truck_tensor, 'lr': truck_step_size}
        ])

        for i in range(max_epoch):
            seq = np.random.permutation(self.num_data)
            loss = np.float(0)
            
            for j in seq:
                # retrieve one record of observed data
                one_data_dict = self._get_one_data(j)

                # P_mode: (num_OD_one_mode * num_assign_interval, num_OD * num_assign_interval)
                P_mode_driving, P_mode_bustransit, P_mode_pnr = self.nb.get_mode_portion_matrix()

                # q_e_mode: num_OD_one_mode x num_assign_interval flattened in F order
                q_e_mode_driving = P_mode_driving.dot(q_e_passenger)
                q_e_mode_bustransit = P_mode_bustransit.dot(q_e_passenger)
                q_e_mode_pnr = P_mode_pnr.dot(q_e_passenger)

                q_e_mode_driving = np.maximum(q_e_mode_driving, 1e-6)
                q_e_mode_bustransit = np.maximum(q_e_mode_bustransit, 1e-6)
                q_e_mode_pnr = np.maximum(q_e_mode_pnr, 1e-6)

                # P_path: (num_path * num_assign_interval, num_OD_one_mode * num_assign_interval)
                P_path_car_driving, P_path_truck_driving = self.nb.get_route_portion_matrix_driving()
                P_path_passenger_bustransit = self.nb.get_route_portion_matrix_bustransit()
                P_path_car_pnr = self.nb.get_route_portion_matrix_pnr()

                # f_e: num_path x num_assign_interval flattened in F order
                f_car_driving = P_path_car_driving.dot(q_e_mode_driving)
                f_truck_driving = P_path_truck_driving.dot(q_e_truck)
                f_passenger_bustransit = P_path_passenger_bustransit.dot(q_e_mode_bustransit)
                f_car_pnr = P_path_car_pnr.dot(q_e_mode_pnr)

                f_car_driving = np.maximum(f_car_driving, 1e-6)
                f_truck_driving = np.maximum(f_truck_driving, 1e-6)
                f_passenger_bustransit = np.maximum(f_passenger_bustransit, 1e-6)
                f_car_pnr = np.maximum(f_car_pnr, 1e-6)
                
                # f_grad: num_path * num_assign_interval
                f_car_driving_grad, f_truck_driving_grad, f_passenger_bustransit_grad, f_car_pnr_grad, f_passenger_pnr_grad, \
                    tmp_loss, loss_dict, dta = \
                        self.compute_path_flow_grad_and_loss(one_data_dict, f_car_driving, f_truck_driving, f_bus, f_passenger_bustransit, f_car_pnr)
                
                # q_mode_grad: num_OD_one_mode * num_assign_interval
                q_grad_car_driving = P_path_car_driving.T.dot(f_car_driving_grad)
                q_grad_car_pnr = P_path_car_pnr.T.dot(f_car_pnr_grad)
                q_truck_grad = P_path_truck_driving.T.dot(f_truck_driving_grad)
                q_grad_passenger_bustransit = P_path_passenger_bustransit.T.dot(f_passenger_bustransit_grad)
                q_grad_passenger_pnr = P_path_car_pnr.T.dot(f_passenger_pnr_grad)

                # q_grad: num_OD * num_assign_interval
                q_passenger_grad = P_mode_driving.T.dot(q_grad_car_driving) \
                                   + P_mode_bustransit.T.dot(q_grad_passenger_bustransit) \
                                   + P_mode_pnr.T.dot(q_grad_passenger_pnr + q_grad_car_pnr)
                
                optimizer.zero_grad()

                q_e_passenger_tensor.grad = torch.from_numpy(q_passenger_grad)
                q_e_truck_tensor.grad = torch.from_numpy(q_truck_grad)

                optimizer.step()

                q_e_passenger_tensor = torch.maximum(q_e_passenger_tensor, torch.tensor(1e-6))
                q_e_truck_tensor = torch.maximum(q_e_truck_tensor, torch.tensor(1e-6))

                q_e_passenger = q_e_passenger_tensor.data.cpu().numpy()
                q_e_truck = q_e_truck_tensor.data.cpu().numpy()

                loss += tmp_loss

                self.compute_path_cost(dta)
                if column_generation[i]:
                    self.update_path_table(dta)
                    f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
                        self.update_path_flow(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
                                              car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)
                # adjust modal split and path flow portion based on path cost and logit choice model
                self.assign_mode_route_portions(alpha_mode, beta_mode, alpha_path, beta_path)

            print("Epoch:", i, "Loss:", loss / np.float(self.num_data), self.print_separate_accuracy(loss_dict))
            loss_list.append([loss / np.float(self.num_data), loss_dict])
            if save_folder is not None:
                pickle.dump([loss / np.float(self.num_data), loss_dict, 
                             q_e_passenger, q_e_truck, 
                             q_e_mode_driving, q_e_mode_bustransit, q_e_mode_pnr,
                             f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr], 
                            open(os.path.join(save_folder, str(i)+'_iteration_estimate_demand.pickle'), 'wb'))

                if column_generation[i]:
                    self.save_simulation_input_files(os.path.join(save_folder, 'input_files_estimate_demand'), 
                                                     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)
                
        return f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, q_e_passenger, q_e_truck, loss_list

    def estimate_demand(self, init_scale_passenger=10, init_scale_truck=10, 
                        car_driving_scale=10, truck_driving_scale=1, passenger_bustransit_scale=1, car_pnr_scale=5,
                        passenger_step_size=0.1, truck_step_size=0.01,
                        max_epoch=100, adagrad=False, column_generation=False,
                        alpha_mode=(1., 1.5, 2.), beta_mode=1, alpha_path=1, beta_path=1, 
                        use_file_as_init=None, save_folder=None):
        if np.isscalar(passenger_step_size):
            passenger_step_size = np.ones(max_epoch) * passenger_step_size
        if np.isscalar(truck_step_size):
            truck_step_size = np.ones(max_epoch) * truck_step_size
        assert(len(passenger_step_size) == max_epoch)
        assert(len(truck_step_size) == max_epoch)

        if np.isscalar(column_generation):
            column_generation = np.ones(max_epoch, dtype=int) * column_generation
        assert(len(column_generation) == max_epoch)
        
        loss_list = list()

        # read from files as init values
        if use_file_as_init is not None:
            _, _, q_e_passenger, q_e_truck, q_e_mode_driving, q_e_mode_bustransit, q_e_mode_pnr, \
                f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = pickle.load(use_file_as_init, 'rb')
            
            self.nb.update_demand_path_driving(f_car_driving, f_truck_driving)
            self.nb.update_demand_path_bustransit(f_passenger_bustransit)
            self.nb.update_demand_path_pnr(f_car_pnr)
        else:
            # q_e: num_OD x num_assign_interval flattened in F order
            q_e_passenger = self.init_demand_flow(len(self.demand_list_total_passenger), init_scale=init_scale_passenger)
            q_e_truck = self.init_demand_flow(len(self.demand_list_truck_driving), init_scale=init_scale_truck)

            # uniform
            self.init_mode_route_portions()

        # fixed bus path flow
        f_bus = self.nb.demand_bus.path_flow_matrix.flatten(order='F')

        for i in range(max_epoch):
            seq = np.random.permutation(self.num_data)
            loss = np.float(0)
            if adagrad:
                sum_g_square_passenger = 1e-6
                sum_g_square_truck = 1e-6
            for j in seq:
                # retrieve one record of observed data
                one_data_dict = self._get_one_data(j)

                # P_mode: (num_OD_one_mode * num_assign_interval, num_OD * num_assign_interval)
                P_mode_driving, P_mode_bustransit, P_mode_pnr = self.nb.get_mode_portion_matrix()

                # q_e_mode: num_OD_one_mode x num_assign_interval flattened in F order
                q_e_mode_driving = P_mode_driving.dot(q_e_passenger)
                q_e_mode_bustransit = P_mode_bustransit.dot(q_e_passenger)
                q_e_mode_pnr = P_mode_pnr.dot(q_e_passenger)

                q_e_mode_driving = np.maximum(q_e_mode_driving, 1e-6)
                q_e_mode_bustransit = np.maximum(q_e_mode_bustransit, 1e-6)
                q_e_mode_pnr = np.maximum(q_e_mode_pnr, 1e-6)

                # P_path: (num_path * num_assign_interval, num_OD_one_mode * num_assign_interval)
                P_path_car_driving, P_path_truck_driving = self.nb.get_route_portion_matrix_driving()
                P_path_passenger_bustransit = self.nb.get_route_portion_matrix_bustransit()
                P_path_car_pnr = self.nb.get_route_portion_matrix_pnr()

                # f_e: num_path x num_assign_interval flattened in F order
                f_car_driving = P_path_car_driving.dot(q_e_mode_driving)
                f_truck_driving = P_path_truck_driving.dot(q_e_truck)
                f_passenger_bustransit = P_path_passenger_bustransit.dot(q_e_mode_bustransit)
                f_car_pnr = P_path_car_pnr.dot(q_e_mode_pnr)

                f_car_driving = np.maximum(f_car_driving, 1e-6)
                f_truck_driving = np.maximum(f_truck_driving, 1e-6)
                f_passenger_bustransit = np.maximum(f_passenger_bustransit, 1e-6)
                f_car_pnr = np.maximum(f_car_pnr, 1e-6)
                
                # f_grad: num_path * num_assign_interval
                f_car_driving_grad, f_truck_driving_grad, f_passenger_bustransit_grad, f_car_pnr_grad, f_passenger_pnr_grad, \
                    tmp_loss, loss_dict, dta = \
                        self.compute_path_flow_grad_and_loss(one_data_dict, f_car_driving, f_truck_driving, f_bus, f_passenger_bustransit, f_car_pnr)
                
                # q_mode_grad: num_OD_one_mode * num_assign_interval
                q_grad_car_driving = P_path_car_driving.T.dot(f_car_driving_grad)
                q_grad_car_pnr = P_path_car_pnr.T.dot(f_car_pnr_grad)
                q_truck_grad = P_path_truck_driving.T.dot(f_truck_driving_grad)
                q_grad_passenger_bustransit = P_path_passenger_bustransit.T.dot(f_passenger_bustransit_grad)
                q_grad_passenger_pnr = P_path_car_pnr.T.dot(f_passenger_pnr_grad)

                # q_grad: num_OD * num_assign_interval
                q_passenger_grad = P_mode_driving.T.dot(q_grad_car_driving) \
                                   + P_mode_bustransit.T.dot(q_grad_passenger_bustransit) \
                                   + P_mode_pnr.T.dot(q_grad_passenger_pnr + q_grad_car_pnr)
                
                if adagrad:
                    sum_g_square_passenger = sum_g_square_passenger + np.power(q_passenger_grad, 2)
                    q_e_passenger -= q_passenger_grad * passenger_step_size[i] / np.sqrt(sum_g_square_passenger)
                    sum_g_square_truck = sum_g_square_truck + np.power(q_truck_grad, 2)
                    q_e_truck -= q_truck_grad * truck_step_size[i] / np.sqrt(sum_g_square_truck)
                else:
                    q_e_passenger -= q_passenger_grad * passenger_step_size[i] # / np.sqrt(i+1)
                    q_e_truck -= q_truck_grad * truck_step_size[i] # / np.sqrt(i+1)
                
                q_e_passenger = np.maximum(q_e_passenger, 1e-6)
                q_e_truck = np.maximum(q_e_truck, 1e-6)

                loss += tmp_loss

                self.compute_path_cost(dta)
                if column_generation[i]:
                    self.update_path_table(dta)
                    f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
                        self.update_path_flow(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
                                              car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)
                # adjust modal split and path flow portion based on path cost and logit choice model
                self.assign_mode_route_portions(alpha_mode, beta_mode, alpha_path, beta_path)

            print("Epoch:", i, "Loss:", loss / np.float(self.num_data), self.print_separate_accuracy(loss_dict))
            loss_list.append([loss / np.float(self.num_data), loss_dict])
            if save_folder is not None:
                pickle.dump([loss / np.float(self.num_data), loss_dict, 
                             q_e_passenger, q_e_truck, 
                             q_e_mode_driving, q_e_mode_bustransit, q_e_mode_pnr,
                             f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr], 
                            open(os.path.join(save_folder, str(i)+'_iteration_estimate_demand.pickle'), 'wb'))

                if column_generation[i]:
                    self.save_simulation_input_files(os.path.join(save_folder, 'input_files_estimate_demand'), 
                                                     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)
                
        return f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, q_e_passenger, q_e_truck, loss_list

    def update_path_table(self, dta):
        start_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        self.nb.update_path_table(dta, start_intervals)

        self.num_path_driving = self.nb.config.config_dict['FIXED']['num_driving_path']
        self.num_path_bustransit = self.nb.config.config_dict['FIXED']['num_bustransit_path']
        self.num_path_pnr = self.nb.config.config_dict['FIXED']['num_pnr_path']
        self.num_path_busroute = self.nb.config.config_dict['FIXED']['num_bus_routes']

        # observed path IDs, np.array
        self.config['paths_list_driving'] = np.array(list(self.nb.path_table_driving.ID2path.keys()), dtype=int)
        self.config['paths_list_bustransit'] = np.array(list(self.nb.path_table_bustransit.ID2path.keys()), dtype=int)
        self.config['paths_list_pnr'] = np.array(list(self.nb.path_table_pnr.ID2path.keys()), dtype=int)
        self.config['paths_list_busroute'] = np.array(list(self.nb.path_table_bus.ID2path.keys()), dtype=int)
        self.config['paths_list'] = np.concatenate((self.config['paths_list_driving'], self.config['paths_list_bustransit'],
                                                    self.config['paths_list_pnr'], self.config['paths_list_busroute']))
        assert(len(np.unique(self.config['paths_list'])) == len(self.config['paths_list']))
        self.paths_list_driving = self.config['paths_list_driving']
        self.paths_list_bustransit = self.config['paths_list_bustransit']
        self.paths_list_pnr = self.config['paths_list_pnr']
        self.paths_list_busroute = self.config['paths_list_busroute']
        self.paths_list = self.config['paths_list']
        assert (len(self.paths_list_driving) == self.num_path_driving)
        assert (len(self.paths_list_bustransit) == self.num_path_bustransit)
        assert (len(self.paths_list_pnr) == self.num_path_pnr)
        assert (len(self.paths_list_busroute) == self.num_path_busroute)
        assert (len(self.paths_list) == self.num_path_driving + self.num_path_bustransit + self.num_path_pnr + self.num_path_busroute)

    def update_path_flow(self, f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
                         car_driving_scale=1, truck_driving_scale=0.1, passenger_bustransit_scale=1, car_pnr_scale=0.5):
        max_interval = self.nb.config.config_dict['DTA']['max_interval']
        # reshape path flow into ndarrays with dimensions of intervals x number of total paths
        f_car_driving = f_car_driving.reshape(max_interval, -1)
        f_truck_driving = f_truck_driving.reshape(max_interval, -1)
        f_passenger_bustransit = f_passenger_bustransit.reshape(max_interval, -1)
        f_car_pnr = f_car_pnr.reshape(max_interval, -1)

        if len(self.nb.path_table_driving.ID2path) > f_car_driving.shape[1]:
            _add_f = np.random.rand(max_interval, len(self.nb.path_table_driving.ID2path) - f_car_driving.shape[1]) * car_driving_scale
            f_car_driving = np.concatenate((f_car_driving, _add_f), axis=1)
            assert(f_car_driving.shape[1] == len(self.nb.path_table_driving.ID2path))
            _add_f = np.random.rand(max_interval, len(self.nb.path_table_driving.ID2path) - f_truck_driving.shape[1]) * truck_driving_scale
            f_truck_driving = np.concatenate((f_truck_driving, _add_f), axis=1)
            assert(f_truck_driving.shape[1] == len(self.nb.path_table_driving.ID2path))

        if len(self.nb.path_table_bustransit.ID2path) > f_passenger_bustransit.shape[1]:
            _add_f = np.random.rand(max_interval, len(self.nb.path_table_bustransit.ID2path) - f_passenger_bustransit.shape[1]) * passenger_bustransit_scale
            f_passenger_bustransit = np.concatenate((f_passenger_bustransit, _add_f), axis=1)
            assert(f_passenger_bustransit.shape[1] == len(self.nb.path_table_bustransit.ID2path))

        if len(self.nb.path_table_pnr.ID2path) > f_car_pnr.shape[1]:
            _add_f = np.random.rand(max_interval, len(self.nb.path_table_pnr.ID2path) - f_car_pnr.shape[1]) * car_pnr_scale
            f_car_pnr = np.concatenate((f_car_pnr, _add_f), axis=1)
            assert(f_car_pnr.shape[1] == len(self.nb.path_table_pnr.ID2path))

        f_car_driving = f_car_driving.flatten(order='C')
        f_truck_driving = f_truck_driving.flatten(order='C')
        f_passenger_bustransit = f_passenger_bustransit.flatten(order='C')
        f_car_pnr = f_car_pnr.flatten(order='C')

        return f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr

    def print_separate_accuracy(self, loss_dict):
        tmp_str = ""
        for loss_type, loss_value in loss_dict.items():
            tmp_str += loss_type + ": " + str(np.round(loss_value, 2)) + "|"
        return tmp_str

    def assign_mode_route_portions(self, alpha_mode=(1., 1.5, 2.), beta_mode=1, alpha_path=1, beta_path=1):
        for OD_idx, (O, D) in enumerate(self.nb.demand_total_passenger.demand_list):
            O_node = self.nb.od.O_dict[O]
            D_node = self.nb.od.D_dict[D]
            
            min_mode_cost = OrderedDict()
            alpha_mode_existed = list()
            if self.nb.od_mode_connectivity.loc[OD_idx, 'driving'] == 1:
                tmp_path_set = self.nb.path_table_driving.path_dict[O_node][D_node]
                cost_array = np.zeros((len(tmp_path_set.path_list), self.num_assign_interval))
                truck_tt_array = np.zeros((len(tmp_path_set.path_list), self.num_assign_interval))
                for tmp_path_idx, tmp_path in enumerate(tmp_path_set.path_list):
                    cost_array[tmp_path_idx, :] = tmp_path.path_cost_car
                    truck_tt_array[tmp_path_idx, :] = tmp_path.path_cost_truck 
                p_array = generate_portion_array(cost_array, alpha_path, beta_path)
                p_array_truck = generate_portion_array(truck_tt_array, alpha_path, beta_path)
                for tmp_path_idx, tmp_path in enumerate(tmp_path_set.path_list):
                    tmp_path.attach_route_choice_portions(p_array[tmp_path_idx, :])
                    tmp_path.attach_route_choice_portions_truck(p_array_truck[tmp_path_idx, :])

                min_mode_cost["driving"] = np.min(cost_array, axis=0)
                alpha_mode_existed.append(alpha_mode[0])

            if self.nb.od_mode_connectivity.loc[OD_idx, 'bustransit'] == 1:
                tmp_path_set = self.nb.path_table_bustransit.path_dict[O_node][D_node]
                cost_array = np.zeros((len(tmp_path_set.path_list), self.num_assign_interval))
                for tmp_path_idx, tmp_path in enumerate(tmp_path_set.path_list):
                    cost_array[tmp_path_idx, :] = tmp_path.path_cost
                p_array = generate_portion_array(cost_array, alpha_path, beta_path)
                for tmp_path_idx, tmp_path in enumerate(tmp_path_set.path_list):
                    tmp_path.attach_route_choice_portions_bustransit(p_array[tmp_path_idx, :])

                min_mode_cost["bustransit"] = np.min(cost_array, axis=0)
                alpha_mode_existed.append(alpha_mode[1])

            if self.nb.od_mode_connectivity.loc[OD_idx, 'pnr'] == 1:
                tmp_path_set = self.nb.path_table_pnr.path_dict[O_node][D_node]
                cost_array = np.zeros((len(tmp_path_set.path_list), self.num_assign_interval))
                for tmp_path_idx, tmp_path in enumerate(tmp_path_set.path_list):
                    cost_array[tmp_path_idx, :] = tmp_path.path_cost
                p_array = generate_portion_array(cost_array, alpha_path, beta_path)
                for tmp_path_idx, tmp_path in enumerate(tmp_path_set.path_list):
                    tmp_path.attach_route_choice_portions_pnr(p_array[tmp_path_idx, :])
                
                min_mode_cost["pnr"] = np.min(cost_array, axis=0)
                alpha_mode_existed.append(alpha_mode[2])

            mode_p_dict = generate_mode_portion_array(min_mode_cost, np.array(alpha_mode_existed), beta_mode)

            if self.nb.od_mode_connectivity.loc[OD_idx, 'driving'] == 1:
                self.nb.demand_driving.demand_dict[O][D][0] = mode_p_dict["driving"]
            if self.nb.od_mode_connectivity.loc[OD_idx, 'bustransit'] == 1:
                self.nb.demand_bustransit.demand_dict[O][D] = mode_p_dict["bustransit"]
            if self.nb.od_mode_connectivity.loc[OD_idx, 'pnr'] == 1:
                self.nb.demand_pnr.demand_dict[O][D] = mode_p_dict["pnr"]

    def init_mode_route_portions(self):
        for O in self.nb.demand_driving.demand_dict.keys():
            for D in self.nb.demand_driving.demand_dict[O].keys():
                self.nb.demand_driving.demand_dict[O][D] = [np.ones(self.num_assign_interval), np.ones(self.num_assign_interval)] 
        for O in self.nb.demand_bustransit.demand_dict.keys():
            for D in self.nb.demand_bustransit.demand_dict[O].keys():
                self.nb.demand_bustransit.demand_dict[O][D] = np.ones(self.num_assign_interval)
        for O in self.nb.demand_pnr.demand_dict.keys():
            for D in self.nb.demand_pnr.demand_dict[O].keys():
                self.nb.demand_pnr.demand_dict[O][D] = np.ones(self.num_assign_interval) 

        for path in self.nb.path_table_driving.ID2path.values():
            path.attach_route_choice_portions(np.ones(self.num_assign_interval))
            path.attach_route_choice_portions_truck(np.ones(self.num_assign_interval))
        for path in self.nb.path_table_bustransit.ID2path.values():
            path.attach_route_choice_portions_bustransit(np.ones(self.num_assign_interval))
        for path in self.nb.path_table_pnr.ID2path.values():
            path.attach_route_choice_portions_pnr(np.ones(self.num_assign_interval))
        
                
###  Behavior related function
def generate_mode_portion_array(mode_cost_dict, alpha, beta=1.):
    assert(len(mode_cost_dict) == len(alpha))
    mode_cost_array = np.stack([v for _, v in mode_cost_dict.items()], axis=0)
    p_array = np.zeros(mode_cost_array.shape)
    for i in range(mode_cost_array.shape[1]):
        p_array[:, i] = logit_fn(mode_cost_array[:,i], alpha, beta)
    
    mode_p_dict = OrderedDict()
    for i, k in enumerate(mode_cost_dict.keys()):
        mode_p_dict[k] = p_array[i, :]
    return mode_p_dict

def generate_portion_array(cost_array, alpha=1.5, beta=1.):
    p_array = np.zeros(cost_array.shape)
    for i in range(cost_array.shape[1]):  # time
        p_array[:, i] = logit_fn(cost_array[:,i], alpha, beta)
    return p_array

def logit_fn(cost, alpha, beta, max_cut=False):
    scale_cost = - (alpha + beta * cost)
    if max_cut:
        e_x = np.exp(scale_cost - np.max(scale_cost))
    else:
        e_x = np.exp(scale_cost)
    p = np.maximum(e_x / e_x.sum(), 1e-6)
    return p