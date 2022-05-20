import os
import numpy as np
import pandas as pd
from collections import OrderedDict
import hashlib
import time
import shutil
from scipy.sparse import coo_matrix, csr_matrix, eye
import pickle
import multiprocessing as mp
import torch

import MNMAPI


class MMDODE:
    def __init__(self, nb, config):
        self.reinitialize(nb, config)

    def reinitialize(self, nb, config):
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

    def check_registered_links_covered_by_registered_paths(self, folder):
        self.save_simulation_input_files(folder)

        a = MNMAPI.mmdta_api()
        a.initialize(folder)

        a.register_links_driving(self.observed_links_driving)
        a.register_links_bus(self.observed_links_bus)
        a.register_links_walking(self.observed_links_walking)

        a.register_paths(self.paths_list)
        a.register_paths_driving(self.paths_list_driving)
        a.register_paths_bustransit(self.paths_list_bustransit)
        a.register_paths_pnr(self.paths_list_pnr)
        a.register_paths_bus(self.paths_list_busroute)

        is_updated = a.generate_paths_to_cover_registered_links_driving()
        is_updated += a.generate_paths_to_cover_registered_links_bus_walking()
        return is_updated
        
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

        _flg = False
        # update demand for each mode
        if (f_car_driving is not None) and (f_truck_driving is not None):
            self.nb.update_demand_path_driving(f_car_driving, f_truck_driving)
            _flg = True
        if f_passenger_bustransit is not None:
            self.nb.update_demand_path_bustransit(f_passenger_bustransit)
            _flg = True
        if f_car_pnr is not None:
            self.nb.update_demand_path_pnr(f_car_pnr)
            _flg = True
        # update nb.demand_total_passenger
        if _flg:
            self.nb.get_mode_portion_matrix()

        # self.nb.config.config_dict['DTA']['flow_scalar'] = 3
        if self.config['use_car_link_tt'] or self.config['use_truck_link_tt'] or self.config['use_passenger_link_tt']:
            self.nb.config.config_dict['DTA']['total_interval'] = 2 * self.num_loading_interval  # hopefully this is sufficient 
        else:
            self.nb.config.config_dict['DTA']['total_interval'] = self.num_loading_interval  # if only count data is used

        self.nb.config.config_dict['DTA']['routing_type'] = 'Multimodal_Hybrid'

        # no output files saved from DNL
        self.nb.config.config_dict['STAT']['rec_volume'] = 1
        self.nb.config.config_dict['STAT']['volume_load_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['volume_record_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['rec_tt'] = 1
        self.nb.config.config_dict['STAT']['tt_load_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['tt_record_automatic_rec'] = 0

        self.nb.dump_to_folder(folder_path)
        

    def _run_simulation(self, f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, counter=0, run_mmdta_adaptive=True):
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

        a.register_links_driving(self.observed_links_driving)
        a.register_links_bus(self.observed_links_bus)
        a.register_links_walking(self.observed_links_walking)

        a.register_paths(self.paths_list)
        a.register_paths_driving(self.paths_list_driving)
        a.register_paths_bustransit(self.paths_list_bustransit)
        a.register_paths_pnr(self.paths_list_pnr)
        a.register_paths_bus(self.paths_list_busroute)

        a.install_cc()
        a.install_cc_tree()

        if run_mmdta_adaptive:
            a.run_mmdta_adaptive(new_folder, -1, False)
        else:
            a.run_whole(False)
        # print("Finish simulation", time.time())

        travel_stats = a.get_travel_stats()
        assert(len(travel_stats) == 9)
        print("\n************ travel stats ************")
        print("car count: {}".format(travel_stats[0]))
        print("car pnr count: {}".format(travel_stats[1]))
        print("truck count: {}".format(travel_stats[2]))
        print("bus count: {}".format(travel_stats[3]))
        print("passenger count: {}".format(travel_stats[4]))
        print("car total travel time (hours): {}".format(travel_stats[5]))
        print("truck total travel time (hours): {}".format(travel_stats[6]))
        print("bus total travel time (hours): {}".format(travel_stats[7]))
        print("passenger total travel time (hours): {}".format(travel_stats[8]))
        print("************ travel stats ************\n")

        # print_emission_stats() only works if folder is not removed, cannot find reason
        a.print_emission_stats()

        shutil.rmtree(new_folder)
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

    def get_bus_dar_matrix_bustransit_link(self, dta, f):
        start_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        end_intervals = start_intervals + self.ass_freq
        assert(end_intervals[-1] <= self.num_loading_interval)
        raw_dar = dta.get_bus_dar_matrix_bustransit_link(start_intervals, end_intervals)
        dar = self._massage_raw_dar(raw_dar, self.ass_freq, f, self.num_assign_interval, self.paths_list_busroute, self.observed_links_bus)
        return dar

    def get_bus_dar_matrix_driving_link(self, dta, f):
        start_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        end_intervals = start_intervals + self.ass_freq
        assert(end_intervals[-1] <= self.num_loading_interval)
        raw_dar = dta.get_bus_dar_matrix_driving_link(start_intervals, end_intervals)
        dar = self._massage_raw_dar(raw_dar, self.ass_freq, f, self.num_assign_interval, self.paths_list_busroute, self.observed_links_driving)
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
        bus_dar_matrix_transit_link = csr_matrix((self.num_assign_interval * len(self.observed_links_bus), 
                                                  self.num_assign_interval * len(self.paths_list_busroute)))
        bus_dar_matrix_driving_link = csr_matrix((self.num_assign_interval * len(self.observed_links_driving), 
                                                  self.num_assign_interval * len(self.paths_list_busroute)))
        passenger_dar_matrix_bustransit = csr_matrix((self.num_assign_interval * (len(self.observed_links_bus) + len(self.observed_links_walking)), 
                                                      self.num_assign_interval * len(self.paths_list_bustransit)))
        passenger_dar_matrix_pnr = csr_matrix((self.num_assign_interval * (len(self.observed_links_bus) + len(self.observed_links_walking)), 
                                               self.num_assign_interval * len(self.paths_list_pnr)))
        
        if self.config['use_car_link_flow'] or self.config['use_car_link_tt']:
            car_dar_matrix_driving = self.get_car_dar_matrix_driving(dta, f_car_driving)
            if car_dar_matrix_driving.max() == 0.:
                print("car_dar_matrix_driving is empty!")
        
            car_dar_matrix_pnr = self.get_car_dar_matrix_pnr(dta, f_car_pnr)
            if car_dar_matrix_pnr.max() == 0.:
                print("car_dar_matrix_pnr is empty!")
            
        if self.config['use_truck_link_flow'] or self.config['use_truck_link_tt']:
            truck_dar_matrix_driving = self.get_truck_dar_matrix_driving(dta, f_truck_driving)
            if truck_dar_matrix_driving.max() == 0.:
                print("truck_dar_matrix_driving is empty!")

        if self.config['use_bus_link_flow'] or self.config['use_bus_link_tt']:
            bus_dar_matrix_transit_link = self.get_bus_dar_matrix_bustransit_link(dta, f_bus)
            if bus_dar_matrix_transit_link.max() == 0.:
                print("bus_dar_matrix_transit_link is empty!")

            bus_dar_matrix_driving_link = self.get_bus_dar_matrix_driving_link(dta, f_bus)
            if bus_dar_matrix_driving_link.max() == 0.:
                print("bus_dar_matrix_driving_link is empty!")

        if self.config['use_passenger_link_flow'] or self.config['use_passenger_link_tt']:
            passenger_dar_matrix_bustransit = self.get_passenger_dar_matrix_bustransit(dta, f_passenger_bustransit)
            if passenger_dar_matrix_bustransit.max() == 0.:
                print("passenger_dar_matrix_bustransit is empty!")

            passenger_dar_matrix_pnr = self.get_passenger_dar_matrix_pnr(dta, f_car_pnr)
            if passenger_dar_matrix_pnr.max() == 0.:
                print("passenger_dar_matrix_pnr is empty!")
            
        return car_dar_matrix_driving, truck_dar_matrix_driving, car_dar_matrix_pnr, bus_dar_matrix_transit_link, bus_dar_matrix_driving_link, \
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

        raw_dar = raw_dar[(raw_dar[:, 1] < self.num_assign_interval * small_assign_freq) & (raw_dar[:, 3] < self.num_loading_interval), :]
        
        # raw_dar[:, 0]: path no.
        # raw_dar[:, 1]: the count of 1 min interval

        # wrong raw_dar may contain paths from different modes (driving, pnr)
        # print(np.min(raw_dar[:, 0].astype(int)))
        # print(np.max(raw_dar[:, 0].astype(int)))
        # print(np.min(paths_list))
        # print(np.max(paths_list))
        # print('raw_dar path unique length: ', len(np.unique(raw_dar[:, 0].astype(int))))
        # print(paths_list)  
        # for x in raw_dar[:, 0].astype(int):
        #     if len(np.where(x == paths_list)[0]) == 0:
        #         print('*************************************')
        #         print(x)
        #         print('*************************************')
                
                
        if type(paths_list) == np.ndarray:
            ind = np.array(list(map(lambda x: True if len(np.where(paths_list == x)[0]) > 0 else False, raw_dar[:, 0].astype(int)))).astype(bool)
            assert(np.sum(ind) == len(ind))
            path_seq = (np.array(list(map(lambda x: np.where(paths_list == x)[0][0], raw_dar[ind, 0].astype(int))))
                        + (raw_dar[ind, 1] / small_assign_freq).astype(int) * num_e_path).astype(int)
        elif type(paths_list) == list:
            ind = np.array(list(map(lambda x: True if x in paths_list else False, raw_dar[:, 0].astype(int)))).astype(bool)
            assert(np.sum(ind) == len(ind))
            path_seq = (np.array(list(map(lambda x: paths_list.index(x), raw_dar[ind, 0].astype(int))))
                        + (raw_dar[ind, 1] / small_assign_freq).astype(int) * num_e_path).astype(int)

        # raw_dar[:, 2]: link no.
        # raw_dar[:, 3]: the count of unit time interval (5s)
        if type(observed_links) == np.ndarray:
            # In Python 3, map() returns an iterable while, in Python 2, it returns a list.
            link_seq = (np.array(list(map(lambda x: np.where(observed_links == x)[0][0], raw_dar[ind, 2].astype(int))))
                        + (raw_dar[ind, 3] / ass_freq).astype(int) * num_e_link).astype(int)
        elif type(observed_links) == list:
            link_seq = (np.array(list(map(lambda x: observed_links.index(x), raw_dar[ind, 2].astype(int))))
                        + (raw_dar[ind, 3] / ass_freq).astype(int) * num_e_link).astype(int)
                    
        # print(path_seq)
        # raw_dar[:, 4]: flow
        p = raw_dar[ind, 4] / f[path_seq]
        
        # print("Creating the coo matrix", time.time()), coo_matrix permits duplicate entries
        mat = coo_matrix((p, (link_seq, path_seq)), shape=(num_assign_interval * num_e_link, num_assign_interval * num_e_path))
        # pickle.dump((p, link_seq, path_seq), open('test.pickle', 'wb'))
        # print('converting the csr', time.time())
        
        # sum duplicate entries in coo_matrix
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
        # f_car_driving = np.random.normal(car_driving_scale, 0.1, self.num_assign_interval * self.num_path_driving)
        # f_truck_driving = np.random.normal(truck_driving_scale, 0.1, self.num_assign_interval * self.num_path_driving)
        # f_passenger_bustransit = np.random.normal(passenger_bustransit_scale, 0.1, self.num_assign_interval * self.num_path_bustransit)
        # f_car_pnr = np.random.normal(car_pnr_scale, 0.1, self.num_assign_interval * self.num_path_pnr)
        return f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr

    def compute_path_cost(self, dta):
        # dta.build_link_cost_map() should be called before this method
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
                                        f_passenger_bustransit, f_car_pnr, counter=0, run_mmdta_adaptive=True):
        # print("Running simulation", time.time())
        dta = self._run_simulation(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, counter, run_mmdta_adaptive)

        if self.config['use_car_link_tt'] or self.config['use_truck_link_tt'] or self.config['use_passenger_link_tt']:
            dta.build_link_cost_map(True)
            dta.get_link_queue_dissipated_time()
            # TODO: unfinished
            car_ltg_matrix_driving, truck_ltg_matrix_driving, car_ltg_matrix_pnr, bus_ltg_matrix_transit_link, bus_ltg_matrix_driving_link, \
               passenger_ltg_matrix_bustransit, passenger_ltg_matrix_pnr = \
                   self.get_ltg(dta)

        # print("Getting DAR", time.time())
        car_dar_matrix_driving, truck_dar_matrix_driving, car_dar_matrix_pnr, bus_dar_matrix_transit_link, bus_dar_matrix_driving_link, \
               passenger_dar_matrix_bustransit, passenger_dar_matrix_pnr = \
                   self.get_dar(dta, f_car_driving, f_truck_driving, f_bus, f_passenger_bustransit, f_car_pnr)
        # print("Evaluating grad", time.time())

        # test DAR
        # print('+++++++++++++++++++++++++++++++++++++ test DAR +++++++++++++++++++++++++++++++++++++')
        # x_e = dta.get_link_car_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
        #                               np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        # print(np.linalg.norm(x_e - car_dar_matrix_driving.dot(f_car_driving) - car_dar_matrix_pnr.dot(f_car_pnr)))

        # x_e = dta.get_link_truck_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
        #                                 np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        # print(np.linalg.norm(x_e - truck_dar_matrix_driving.dot(f_truck_driving) - bus_dar_matrix_driving_link.dot(f_bus)))

        # x_e_bus_passenger = dta.get_link_bus_passenger_inflow(
        #     np.arange(0, self.num_loading_interval, self.ass_freq), 
        #     np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
        # assert(x_e_bus_passenger.shape[0] == len(self.observed_links_bus))
        # x_e_walking_passenger = dta.get_link_walking_passenger_inflow(
        #     np.arange(0, self.num_loading_interval, self.ass_freq), 
        #     np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
        # assert(x_e_walking_passenger.shape[0] == len(self.observed_links_walking))
        # x_e = np.concatenate((x_e_bus_passenger, x_e_walking_passenger), axis=0).flatten(order='F')
        # print(np.linalg.norm(x_e - passenger_dar_matrix_bustransit.dot(f_passenger_bustransit) - passenger_dar_matrix_pnr.dot(f_car_pnr)))
        # print('+++++++++++++++++++++++++++++++++++++ test DAR +++++++++++++++++++++++++++++++++++++')

        # derivative of count loss with respect to link flow
        car_grad = np.zeros(len(self.observed_links_driving) * self.num_assign_interval)
        truck_grad = np.zeros(len(self.observed_links_driving) * self.num_assign_interval)
        # bus_grad = np.zeros(len(self.observed_links_bus) * self.num_assign_interval)
        passenger_grad = np.zeros((len(self.observed_links_bus) + len(self.observed_links_walking)) * self.num_assign_interval)
        
        if self.config['use_car_link_flow']:
            car_grad += self.config['link_car_flow_weight'] * self._compute_count_loss_grad_on_car_link_flow(dta, one_data_dict)
        if self.config['use_truck_link_flow']:
            truck_grad += self.config['link_truck_flow_weight'] * self._compute_count_loss_grad_on_truck_link_flow(dta, one_data_dict)
        if self.config['use_passenger_link_flow']:
            passenger_grad += self.config['link_passenger_flow_weight'] * self._compute_count_loss_grad_on_passenger_link_flow(dta, one_data_dict)

        # derivative of count loss with respect to path flow
        f_car_driving_grad = car_dar_matrix_driving.T.dot(car_grad)
        f_truck_driving_grad = truck_dar_matrix_driving.T.dot(truck_grad)
        f_passenger_bustransit_grad = passenger_dar_matrix_bustransit.T.dot(passenger_grad)
        f_car_pnr_grad = car_dar_matrix_pnr.T.dot(car_grad)
        f_passenger_pnr_grad = passenger_dar_matrix_pnr.T.dot(passenger_grad)

        # derivative of travel time loss with respect to link travel time
        car_grad = np.zeros(len(self.observed_links_driving) * self.num_assign_interval)
        truck_grad = np.zeros(len(self.observed_links_driving) * self.num_assign_interval)
        # bus_grad = np.zeros(len(self.observed_links_bus) * self.num_assign_interval)
        passenger_grad = np.zeros((len(self.observed_links_bus) + len(self.observed_links_walking)) * self.num_assign_interval)

        if self.config['use_car_link_tt']:
            car_grad += self.config['link_car_tt_weight'] * self._compute_tt_loss_grad_on_car_link_tt(dta, one_data_dict)
        if self.config['use_truck_link_tt']:
            truck_grad += self.config['link_truck_tt_weight'] * self._compute_tt_loss_grad_on_truck_link_tt(dta, one_data_dict)
        if self.config['use_passenger_link_tt']:
            passenger_grad += self.config['link_passenger_tt_weight'] * self._compute_tt_loss_grad_on_passenger_link_tt(dta, one_data_dict)

        # derivative of travel time loss with respect to path flow
        if self.config['use_car_link_tt']:
            # f_car_driving_grad += car_dar_matrix_driving.T.dot(self._compute_tt_loss_grad_on_car_link_flow(car_grad))
            # f_car_pnr_grad += car_dar_matrix_pnr.T.dot(self._compute_tt_loss_grad_on_car_link_flow(car_grad))

            f_car_driving_grad += car_ltg_matrix_driving.T.dot(car_grad)
            f_car_pnr_grad += car_ltg_matrix_pnr.T.dot(car_grad)
               
        if self.config['use_truck_link_tt']:
            # f_truck_driving_grad += truck_dar_matrix_driving.T.dot(self._compute_tt_loss_grad_on_truck_link_flow(truck_grad))

            f_truck_driving_grad += truck_ltg_matrix_driving.T.dot(truck_grad)

        if self.config['use_passenger_link_tt']:
            # f_passenger_bustransit_grad += passenger_dar_matrix_bustransit.T.dot(self._compute_tt_loss_grad_on_passenger_link_flow(passenger_grad))
            # f_passenger_pnr_grad += passenger_dar_matrix_pnr.T.dot(self._compute_tt_loss_grad_on_passenger_link_flow(passenger_grad))

            f_passenger_bustransit_grad += passenger_ltg_matrix_bustransit.T.dot(passenger_grad)
            f_passenger_pnr_grad += passenger_ltg_matrix_pnr.T.dot(passenger_grad)
        
        # print("Getting Loss", time.time())
        total_loss, loss_dict = self._get_loss(one_data_dict, dta)

        return f_car_driving_grad, f_truck_driving_grad, f_passenger_bustransit_grad, f_car_pnr_grad, f_passenger_pnr_grad, \
               total_loss, loss_dict, dta

    def _compute_count_loss_grad_on_car_link_flow(self, dta, one_data_dict):
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

    def _compute_count_loss_grad_on_truck_link_flow(self, dta, one_data_dict):
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

    def _compute_count_loss_grad_on_passenger_link_flow(self, dta, one_data_dict):
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

    def _compute_tt_loss_grad_on_car_link_tt(self, dta, one_data_dict):
        tt_e = dta.get_car_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
                                          np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq, self.ass_freq).flatten(order='F')
        # tt_e = dta.get_car_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
        assert(len(tt_e) == len(self.observed_links_driving) * self.num_assign_interval)
        # tt_free = np.tile(list(map(lambda x: self.nb.get_link_driving(x).get_car_fft(), self.observed_links_driving)), (self.num_assign_interval))
        tt_free = np.tile(dta.get_car_link_fftt(self.observed_links_driving), (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['car_link_tt'], tt_free) 
        # tt_o = one_data_dict['car_link_tt']
        # print('o-----', tt_o)
        # print('e-----', tt_e)
        grad = -np.nan_to_num(tt_o - tt_e)/tt_o
        # print('g-----', grad)
        # if self.config['car_count_agg']:
        #   grad = one_data_dict['car_count_agg_L'].T.dot(grad)
        # print(tt_e, tt_o)
        # print("car_grad", grad)
        return grad

    def _compute_tt_loss_grad_on_car_link_flow(self, dta, tt_loss_grad_on_car_link_tt):
        _link_tt_grad_link_flow_car = self._compute_link_tt_grad_link_flow_car(dta)
        grad = _link_tt_grad_link_flow_car.dot(tt_loss_grad_on_car_link_tt)
        return grad

    def _compute_tt_loss_grad_on_truck_link_tt(self, dta, one_data_dict):
        tt_e = dta.get_truck_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
                                            np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq, self.ass_freq).flatten(order='F')
        # tt_e = dta.get_truck_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
        assert(len(tt_e) == len(self.observed_links_driving) * self.num_assign_interval)
        # tt_free = np.tile(list(map(lambda x: self.nb.get_link_driving(x).get_truck_fft(), self.observed_links_driving)), (self.num_assign_interval))
        tt_free = np.tile(dta.get_truck_link_fftt(self.observed_links_driving), (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['truck_link_tt'], tt_free)
        grad = -np.nan_to_num(tt_o - tt_e)/tt_o
        # if self.config['truck_count_agg']:
        #   grad = one_data_dict['truck_count_agg_L'].T.dot(grad)
        # print("truck_grad", grad)
        return grad

    def _compute_tt_loss_grad_on_truck_link_flow(self, dta, tt_loss_grad_on_truck_link_tt):
        _link_tt_grad_link_flow_truck = self._compute_link_tt_grad_link_flow_truck(dta)
        grad = _link_tt_grad_link_flow_truck.dot(tt_loss_grad_on_truck_link_tt)
        return grad

    def _compute_tt_loss_grad_on_passenger_link_tt(self, dta, one_data_dict):
        tt_e_bus = dta.get_bus_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
                                              np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq, self.ass_freq)
        # tt_e_bus = dta.get_bus_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq))
        assert(tt_e_bus.shape[0] == len(self.observed_links_bus))
        tt_e_walking = dta.get_passenger_walking_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
                                                                np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq, self.ass_freq)
        # tt_e_walking = dta.get_passenger_walking_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq))
        assert(tt_e_walking.shape[0] == len(self.observed_links_walking))
        tt_e = np.concatenate((tt_e_bus, tt_e_walking), axis=0).flatten(order='F')
        # tt_free = np.tile(list(map(lambda x: self.nb.get_link_bus(x).get_bus_fft(), self.observed_links_bus)) + 
        #                   list(map(lambda x: self.nb.get_link_walking(x).get_walking_fft(), self.observed_links_walking)), 
        #                   (self.num_assign_interval))
        tt_free = np.tile(np.concatenate((dta.get_bus_link_fftt(self.observed_links_bus), dta.get_walking_link_fftt(self.observed_links_walking))), 
                          (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['passenger_link_tt'], tt_free)
        grad = -np.nan_to_num(tt_o - tt_e)/tt_o
        # if self.config['passenger_count_agg']:
        #   grad = one_data_dict['passenger_count_agg_L'].T.dot(grad)
        # print("passenger_grad", grad)
        return grad

    def _compute_tt_loss_grad_on_passenger_link_flow(self, dta, tt_loss_grad_on_passenger_link_tt):
        _link_tt_grad_link_flow_passenger = self._compute_link_tt_grad_link_flow_passenger(dta)
        grad = _link_tt_grad_link_flow_passenger.dot(tt_loss_grad_on_passenger_link_tt)
        return grad

    # def _compute_grad_on_bus_link_tt(self, dta, one_data_dict):
    #     tt_e = dta.get_bus_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
    #     tt_free = np.tile(list(map(lambda x: self.nb.get_link_bus(x).get_bus_fft(), self.observed_links_bus)), (self.num_assign_interval))
    #     tt_e = np.maximum(tt_e, tt_free)
    #     tt_o = np.maximum(one_data_dict['bus_link_tt'], tt_free)
    #     grad = -np.nan_to_num(tt_o - tt_e)/tt_o
    #     # if self.config['bus_count_agg']:
    #     #   grad = one_data_dict['bus_count_agg_L'].T.dot(grad)
    #     # print("bus_grad", grad)
    #     return grad

    # def _compute_grad_on_walking_link_tt(self, dta, one_data_dict):
    #     tt_e = dta.get_passenger_walking_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
    #     tt_free = np.tile(list(map(lambda x: self.nb.get_link_walking(x).get_walking_fft(), self.observed_links_walking)), (self.num_assign_interval))
    #     tt_e = np.maximum(tt_e, tt_free)
    #     tt_o = np.maximum(one_data_dict['walking_link_tt'], tt_free)
    #     grad = -np.nan_to_num(tt_o - tt_e)/tt_o
    #     # if self.config['bus_count_agg']:
    #     #   grad = one_data_dict['bus_count_agg_L'].T.dot(grad)
    #     # print("bus_grad", grad)
    #     return grad

    def _compute_link_tt_grad_on_link_flow_car(self, dta):
        assign_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        num_assign_intervals = len(assign_intervals)
        num_links = len(self.observed_links_driving)

        car_link_out_cc = dict()
        for link_ID in self.observed_links_driving:
            car_link_out_cc[link_ID] = dta.get_car_link_out_cc(link_ID)

        # tt_e = dta.get_car_link_tt(np.arange(assign_intervals[-1] + self.ass_freq))
        # assert(tt_e.shape[0] == num_links and tt_e.shape[1] == assign_intervals[-1] + self.ass_freq)
        # # average link travel time
        # tt_e = np.stack(list(map(lambda i : np.mean(tt_e[:, i : i+self.ass_freq], axis=1), assign_intervals)), axis=1)
        # assert(tt_e.shape[0] == num_links and tt_e.shape[1] == num_assign_intervals)

        # tt_e = dta.get_car_link_tt(np.arange(0, self.num_loading_interval))

        # tt_e = dta.get_car_link_tt(assign_intervals)
        tt_e = dta.get_car_link_tt_robust(assign_intervals, assign_intervals + self.ass_freq, self.ass_freq)

        # tt_free = np.array(list(map(lambda x: self.nb.get_link_driving(x).get_car_fft(), self.observed_links_driving)))
        tt_free = dta.get_car_link_fftt(self.observed_links_driving)
        mask = tt_e > tt_free[:, np.newaxis]

        # no congestions
        if np.sum(mask) == 0:
            return csr_matrix((num_assign_intervals * num_links, 
                               num_assign_intervals * num_links))
            # return eye(num_assign_intervals * num_links)

        # cc = np.zeros((num_links, num_assign_intervals + 1), dtype=np.float)
        # for j, link_ID in enumerate(self.observed_links_driving):
        #     cc[j, :] = np.array(list(map(lambda timestamp : self._get_flow_from_cc(timestamp, car_link_out_cc[link_ID]), 
        #                                  np.concatenate((assign_intervals, np.array([assign_intervals[-1] + self.ass_freq]))))))

        # outflow_rate = np.diff(cc, axis=1) / self.ass_freq / self.nb.config.config_dict['DTA']['unit_time']

        cc = np.zeros((num_links, assign_intervals[-1] + self.ass_freq + 1), dtype=np.float)
        for j, link_ID in enumerate(self.observed_links_driving):
            cc[j, :] = np.array(list(map(lambda timestamp : self._get_flow_from_cc(timestamp, car_link_out_cc[link_ID]), 
                                         np.arange(assign_intervals[-1] + self.ass_freq + 1))))

        outflow_rate = np.diff(cc, axis=1) / self.nb.config.config_dict['DTA']['unit_time']
        outflow_rate = outflow_rate[:, assign_intervals]

        if mask.shape[1] == outflow_rate.shape[1]:
            outflow_rate *= mask
        
        if outflow_rate.shape[1] == num_assign_intervals:
            outflow_avg_rate = outflow_rate
        else:
            outflow_avg_rate = np.stack(list(map(lambda i : np.mean(outflow_rate[:, i : i+self.ass_freq], axis=1), assign_intervals)), axis=1)
            if mask.shape[1] == outflow_avg_rate.shape[1]:
                outflow_avg_rate *= mask
        assert(outflow_avg_rate.shape[0] == num_links and outflow_avg_rate.shape[1] == num_assign_intervals)

        val = list()
        row = list()
        col = list()

        for i, assign_interval in enumerate(assign_intervals):
            for j, link_ID in enumerate(self.observed_links_driving):
                _tmp = outflow_avg_rate[j, i]
                if _tmp > 0:
                    _tmp = 1 / _tmp
                    val.append(_tmp)
                    row.append(j + num_links * i)
                    col.append(j + num_links * i)
                # else:
                #     _tmp = 1
                #     val.append(_tmp)
                #     row.append(j + num_links * i)
                #     col.append(j + num_links * i)

        grad = coo_matrix((val, (row, col)), 
                           shape=(num_links * num_assign_intervals, num_links * num_assign_intervals)).tocsr()
        
        return grad

    def _compute_link_tt_grad_on_link_flow_truck(self, dta):
        assign_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        num_assign_intervals = len(assign_intervals)
        num_links = len(self.observed_links_driving)

        truck_link_out_cc = dict()
        for link_ID in self.observed_links_driving:
            truck_link_out_cc[link_ID] = dta.get_truck_link_out_cc(link_ID)

        # tt_e = dta.get_truck_link_tt(np.arange(assign_intervals[-1] + self.ass_freq))
        # assert(tt_e.shape[0] == num_links and tt_e.shape[1] == assign_intervals[-1] + self.ass_freq)
        # # average link travel time
        # tt_e = np.stack(list(map(lambda i : np.mean(tt_e[:, i : i+self.ass_freq], axis=1), assign_intervals)), axis=1)
        # assert(tt_e.shape[0] == num_links and tt_e.shape[1] == num_assign_intervals)

        # tt_e = dta.get_truck_link_tt(assign_intervals)
        tt_e = dta.get_truck_link_tt_robust(assign_intervals, assign_intervals + self.ass_freq)

        # tt_free = np.array(list(map(lambda x: self.nb.get_link_driving(x).get_truck_fft(), self.observed_links_driving)))
        tt_free = dta.get_truck_link_fftt(self.observed_links_driving)
        mask = tt_e > tt_free[:, np.newaxis]

        # no congestions
        if np.sum(mask) == 0:
            return csr_matrix((num_assign_intervals * num_links, 
                               num_assign_intervals * num_links))
            # return eye(num_assign_intervals * num_links)

        # cc = np.zeros((num_links, num_assign_intervals + 1), dtype=np.float)
        # for j, link_ID in enumerate(self.observed_links_driving):
        #     cc[j, :] = np.array(list(map(lambda timestamp : self._get_flow_from_cc(timestamp, truck_link_out_cc[link_ID]), 
        #                                  np.concatenate((assign_intervals, np.array([assign_intervals[-1] + self.ass_freq]))))))

        # outflow_rate = np.diff(cc, axis=1) / self.ass_freq / self.nb.config.config_dict['DTA']['unit_time']

        cc = np.zeros((num_links, assign_intervals[-1] + self.ass_freq + 1), dtype=np.float)
        for j, link_ID in enumerate(self.observed_links_driving):
            cc[j, :] = np.array(list(map(lambda timestamp : self._get_flow_from_cc(timestamp, truck_link_out_cc[link_ID]), 
                                         np.arange(assign_intervals[-1] + self.ass_freq + 1))))

        outflow_rate = np.diff(cc, axis=1) / self.nb.config.config_dict['DTA']['unit_time']
        outflow_rate = outflow_rate[:, assign_intervals]
        
        if mask.shape[1] == outflow_rate.shape[1]:
            outflow_rate *= mask
        
        if outflow_rate.shape[1] == num_assign_intervals:
            outflow_avg_rate = outflow_rate
        else:
            outflow_avg_rate = np.stack(list(map(lambda i : np.mean(outflow_rate[:, i : i+self.ass_freq], axis=1), assign_intervals)), axis=1)
            if mask.shape[1] == outflow_avg_rate.shape[1]:
                outflow_avg_rate *= mask
        assert(outflow_avg_rate.shape[0] == num_links and outflow_avg_rate.shape[1] == num_assign_intervals)

        val = list()
        row = list()
        col = list()

        for i, assign_interval in enumerate(assign_intervals):
            for j, link_ID in enumerate(self.observed_links_driving):
                _tmp = outflow_avg_rate[j, i]
                if _tmp > 0:
                    _tmp = 1 / _tmp
                    val.append(_tmp)
                    row.append(j + num_links * i)
                    col.append(j + num_links * i)
                # else:
                #     _tmp = 1
                #     val.append(_tmp)
                #     row.append(j + num_links * i)
                #     col.append(j + num_links * i)

        grad = coo_matrix((val, (row, col)), 
                           shape=(num_links * num_assign_intervals, num_links * num_assign_intervals)).tocsr()
        
        return grad

    def _compute_link_tt_grad_on_link_flow_passenger(self, dta):
        assign_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        num_assign_intervals = len(assign_intervals)
        num_links = len(self.observed_links_bus) + len(self.observed_links_walking)

        passenger_link_out_cc = dict()
        for link_ID in self.observed_links_bus:
            passenger_link_out_cc[link_ID] = dta.get_bus_link_out_passenger_cc(link_ID)
        for link_ID in self.observed_links_walking:
            passenger_link_out_cc[link_ID] = dta.get_walking_link_out_cc(link_ID)

        # tt_e_bus = dta.get_bus_link_tt(np.arange(assign_intervals[-1] + self.ass_freq))
        # tt_e_walking = dta.get_passenger_walking_link_tt(np.arange(assign_intervals[-1] + self.ass_freq))
        # tt_e = np.concatenate((tt_e_bus, tt_e_walking), axis=0)
        # assert(tt_e.shape[0] == num_links and tt_e.shape[1] == assign_intervals[-1] + self.ass_freq)
        # # average link travel time
        # tt_e = np.stack(list(map(lambda i : np.mean(tt_e[:, i : i+self.ass_freq], axis=1), assign_intervals)), axis=1)
        # assert(tt_e.shape[0] == num_links and tt_e.shape[1] == num_assign_intervals)

        # tt_e_bus = dta.get_bus_link_tt(assign_intervals)
        tt_e_bus = dta.get_bus_link_tt_robust(assign_intervals, assign_intervals + self.ass_freq, self.ass_freq)
        # tt_e_walking = dta.get_passenger_walking_link_tt(assign_intervals)
        tt_e_walking = dta.get_passenger_walking_link_tt_robust(assign_intervals, assign_intervals + self.ass_freq, self.ass_freq)
        tt_e = np.concatenate((tt_e_bus, tt_e_walking), axis=0)
        assert(tt_e.shape[0] == num_links and tt_e.shape[1] == num_assign_intervals)

        # tt_free = np.array(list(map(lambda x: self.nb.get_link_bus(x).get_bus_fft(), self.observed_links_bus)) + 
        #                    list(map(lambda x: self.nb.get_link_walking(x).get_walking_fft(), self.observed_links_walking)))
        tt_free = np.concatenate((dta.get_bus_link_fftt(self.observed_links_bus), dta.get_walking_link_fftt(self.observed_links_walking)))
        mask = tt_e > tt_free[:, np.newaxis]

        # no congestions
        if np.sum(mask) == 0:
            return csr_matrix((num_assign_intervals * num_links, 
                               num_assign_intervals * num_links))
            # return eye(num_assign_intervals * num_links)

        # cc = np.zeros((num_links, num_assign_intervals + 1), dtype=np.float)
        # for j, link_ID in enumerate(np.concatenate((self.observed_links_bus, self.observed_links_walking))):
        #     cc[j, :] = np.array(list(map(lambda timestamp : self._get_flow_from_cc(timestamp, passenger_link_out_cc[link_ID]), 
        #                                  np.concatenate((assign_intervals, np.array([assign_intervals[-1] + self.ass_freq]))))))

        # outflow_rate = np.diff(cc, axis=1) / self.ass_freq / self.nb.config.config_dict['DTA']['unit_time']

        cc = np.zeros((num_links, assign_intervals[-1] + self.ass_freq + 1), dtype=np.float)
        for j, link_ID in enumerate(np.concatenate((self.observed_links_bus, self.observed_links_walking))):
            cc[j, :] = np.array(list(map(lambda timestamp : self._get_flow_from_cc(timestamp, passenger_link_out_cc[link_ID]), 
                                         np.arange(assign_intervals[-1] + self.ass_freq + 1))))

        outflow_rate = np.diff(cc, axis=1) / self.nb.config.config_dict['DTA']['unit_time']
        outflow_rate = outflow_rate[:, assign_intervals]

        if mask.shape[1] == outflow_rate.shape[1]:
            outflow_rate *= mask
        
        if outflow_rate.shape[1] == num_assign_intervals:
            outflow_avg_rate = outflow_rate
        else:
            outflow_avg_rate = np.stack(list(map(lambda i : np.mean(outflow_rate[:, i : i+self.ass_freq], axis=1), assign_intervals)), axis=1)
            if mask.shape[1] == outflow_avg_rate.shape[1]:
                outflow_avg_rate *= mask
        assert(outflow_avg_rate.shape[0] == num_links and outflow_avg_rate.shape[1] == num_assign_intervals)
        

        val = list()
        row = list()
        col = list()

        for i, assign_interval in enumerate(assign_intervals):
            for j, link_ID in enumerate(np.concatenate((self.observed_links_bus, self.observed_links_walking))):
                _tmp = outflow_avg_rate[j, i]
                if _tmp > 0:
                    _tmp = 1 / _tmp
                    val.append(_tmp)
                    row.append(j + num_links * i)
                    col.append(j + num_links * i)
                # else:
                #     _tmp = 1
                #     val.append(_tmp)
                #     row.append(j + num_links * i)
                #     col.append(j + num_links * i)

        grad = coo_matrix((val, (row, col)), 
                           shape=(num_links * num_assign_intervals, num_links * num_assign_intervals)).tocsr()
        
        return grad

    def _compute_link_tt_grad_on_path_flow_car_driving(self, dta):
        # dta.build_link_cost_map() is invoked already
        assign_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        num_assign_intervals = len(assign_intervals)

        release_freq = 60
        # this is in terms of 5-s intervals
        release_intervals = np.arange(0, self.num_loading_interval, release_freq // self.nb.config.config_dict['DTA']['unit_time'])

        raw_ltg = dta.get_car_ltg_matrix_driving(release_intervals, self.num_loading_interval)

        ltg = self._massage_raw_ltg(raw_ltg, self.ass_freq, num_assign_intervals, self.paths_list_driving, self.observed_links_driving)
        return ltg

    def _compute_link_tt_grad_on_path_flow_car_pnr(self, dta):
        # dta.build_link_cost_map() is invoked already
        assign_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        num_assign_intervals = len(assign_intervals)

        release_freq = 60
        release_intervals = np.arange(0, self.num_loading_interval, release_freq // self.nb.config.config_dict['DTA']['unit_time'])

        raw_ltg = dta.get_car_ltg_matrix_pnr(release_intervals, self.num_loading_interval)

        ltg = self._massage_raw_ltg(raw_ltg, self.ass_freq, num_assign_intervals, self.paths_list_pnr, self.observed_links_driving)
        return ltg

    def _compute_link_tt_grad_on_path_flow_truck_driving(self, dta):
        # dta.build_link_cost_map() is invoked already
        assign_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        num_assign_intervals = len(assign_intervals)

        release_freq = 60
        release_intervals = np.arange(0, self.num_loading_interval, release_freq // self.nb.config.config_dict['DTA']['unit_time'])

        raw_ltg = dta.get_truck_ltg_matrix_driving(release_intervals, self.num_loading_interval)

        ltg = self._massage_raw_ltg(raw_ltg, self.ass_freq, num_assign_intervals, self.paths_list_driving, self.observed_links_driving)
        return ltg

    def _compute_link_tt_grad_on_path_flow_passenger_bustransit(self, dta):
        # dta.build_link_cost_map() is invoked already
        assign_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        num_assign_intervals = len(assign_intervals)

        release_freq = 60
        release_intervals = np.arange(0, self.num_loading_interval, release_freq // self.nb.config.config_dict['DTA']['unit_time'])

        raw_ltg = dta.get_passenger_ltg_matrix_bustransit(release_intervals, self.num_loading_interval)

        ltg = self._massage_raw_ltg(raw_ltg, self.ass_freq, num_assign_intervals, self.paths_list_bustransit, 
                                    np.concatenate((self.observed_links_bus, self.observed_links_walking)))
        return ltg
    
    def _compute_link_tt_grad_on_path_flow_passenger_pnr(self, dta):
        # dta.build_link_cost_map() is invoked already
        assign_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        num_assign_intervals = len(assign_intervals)

        release_freq = 60
        release_intervals = np.arange(0, self.num_loading_interval, release_freq // self.nb.config.config_dict['DTA']['unit_time'])

        raw_ltg = dta.get_passenger_ltg_matrix_pnr(release_intervals, self.num_loading_interval)

        ltg = self._massage_raw_ltg(raw_ltg, self.ass_freq, num_assign_intervals, self.paths_list_pnr,
                                    np.concatenate((self.observed_links_bus, self.observed_links_walking)))
        return ltg

    def _massage_raw_ltg(self, raw_ltg, ass_freq, num_assign_interval, paths_list, observed_links):
        assert(raw_ltg.shape[1] == 5)
        if raw_ltg.shape[0] == 0:
            print("No ltg. No congestion.")
            return csr_matrix((num_assign_interval * len(observed_links), 
                               num_assign_interval * len(paths_list)))

        num_e_path = len(paths_list)
        num_e_link = len(observed_links)
        # 15 min
        small_assign_freq = ass_freq * self.nb.config.config_dict['DTA']['unit_time'] / 60

        raw_ltg = raw_ltg[(raw_ltg[:, 1] < self.num_loading_interval) & (raw_ltg[:, 3] < self.num_loading_interval), :]

        # raw_ltg[:, 0]: path no.
        # raw_ltg[:, 1]: the count of 1 min interval in terms of 5s intervals
                
        if type(paths_list) == np.ndarray:
            ind = np.array(list(map(lambda x: True if len(np.where(paths_list == x)[0]) > 0 else False, raw_ltg[:, 0].astype(int)))).astype(bool)
            assert(np.sum(ind) == len(ind))
            path_seq = (np.array(list(map(lambda x: np.where(paths_list == x)[0][0], raw_ltg[ind, 0].astype(int))))
                        + (raw_ltg[ind, 1] / ass_freq).astype(int) * num_e_path).astype(int)
        elif type(paths_list) == list:
            ind = np.array(list(map(lambda x: True if x in paths_list else False, raw_ltg[:, 0].astype(int)))).astype(bool)
            assert(np.sum(ind) == len(ind))
            path_seq = (np.array(list(map(lambda x: paths_list.index(x), raw_ltg[ind, 0].astype(int))))
                        + (raw_ltg[ind, 1] / ass_freq).astype(int) * num_e_path).astype(int)

        # raw_ltg[:, 2]: link no.
        # raw_ltg[:, 3]: the count of unit time interval (5s)
        if type(observed_links) == np.ndarray:
            # In Python 3, map() returns an iterable while, in Python 2, it returns a list.
            link_seq = (np.array(list(map(lambda x: np.where(observed_links == x)[0][0], raw_ltg[ind, 2].astype(int))))
                        + (raw_ltg[ind, 3] / ass_freq).astype(int) * num_e_link).astype(int)
        elif type(observed_links) == list:
            link_seq = (np.array(list(map(lambda x: observed_links.index(x), raw_ltg[ind, 2].astype(int))))
                        + (raw_ltg[ind, 3] / ass_freq).astype(int) * num_e_link).astype(int)
                    
        # print(path_seq)
        # raw_ltg[:, 4]: gradient, to be averaged for each large assign interval 
        p = raw_ltg[ind, 4] / (ass_freq * small_assign_freq)
        
        # print("Creating the coo matrix", time.time()), coo_matrix permits duplicate entries
        mat = coo_matrix((p, (link_seq, path_seq)), shape=(num_assign_interval * num_e_link, num_assign_interval * num_e_path))
        # pickle.dump((p, link_seq, path_seq), open('test.pickle', 'wb'))
        # print('converting the csr', time.time())
        
        # sum duplicate entries in coo_matrix
        mat = mat.tocsr()
        # print('finish converting', time.time())
        return mat

    def get_ltg(self, dta):
        
        car_ltg_matrix_driving = csr_matrix((self.num_assign_interval * len(self.observed_links_driving), 
                                             self.num_assign_interval * len(self.paths_list_driving)))
        car_ltg_matrix_pnr = csr_matrix((self.num_assign_interval * len(self.observed_links_driving), 
                                         self.num_assign_interval * len(self.paths_list_pnr)))
        truck_ltg_matrix_driving = csr_matrix((self.num_assign_interval * len(self.observed_links_driving), 
                                               self.num_assign_interval * len(self.paths_list_driving)))
        bus_ltg_matrix_transit_link = csr_matrix((self.num_assign_interval * len(self.observed_links_bus), 
                                                  self.num_assign_interval * len(self.paths_list_busroute)))
        bus_ltg_matrix_driving_link = csr_matrix((self.num_assign_interval * len(self.observed_links_driving), 
                                                  self.num_assign_interval * len(self.paths_list_busroute)))
        passenger_ltg_matrix_bustransit = csr_matrix((self.num_assign_interval * (len(self.observed_links_bus) + len(self.observed_links_walking)), 
                                                      self.num_assign_interval * len(self.paths_list_bustransit)))
        passenger_ltg_matrix_pnr = csr_matrix((self.num_assign_interval * (len(self.observed_links_bus) + len(self.observed_links_walking)), 
                                               self.num_assign_interval * len(self.paths_list_pnr)))
        
        if self.config['use_car_link_tt']:
            car_ltg_matrix_driving = self._compute_link_tt_grad_on_path_flow_car_driving(dta)
            if car_ltg_matrix_driving.max() == 0.:
                print("car_ltg_matrix_driving is empty!")

            # TODO:
            # car_ltg_matrix_pnr = self._compute_link_tt_grad_on_path_flow_car_pnr(dta)
            if car_ltg_matrix_pnr.max() == 0.:
                print("car_ltg_matrix_pnr is empty!")
            
        if self.config['use_truck_link_tt']:
            truck_ltg_matrix_driving = self._compute_link_tt_grad_on_path_flow_truck_driving(dta)
            if truck_ltg_matrix_driving.max() == 0.:
                print("truck_ltg_matrix_driving is empty!")

        if self.config['use_passenger_link_tt']:
            passenger_ltg_matrix_bustransit = self._compute_link_tt_grad_on_path_flow_passenger_bustransit(dta)
            if passenger_ltg_matrix_bustransit.max() == 0.:
                print("passenger_ltg_matrix_bustransit is empty!")

            passenger_ltg_matrix_pnr = self._compute_link_tt_grad_on_path_flow_passenger_pnr(dta)
            if passenger_ltg_matrix_pnr.max() == 0.:
                print("passenger_ltg_matrix_pnr is empty!")
            
        return car_ltg_matrix_driving, truck_ltg_matrix_driving, car_ltg_matrix_pnr, bus_ltg_matrix_transit_link, bus_ltg_matrix_driving_link, \
               passenger_ltg_matrix_bustransit, passenger_ltg_matrix_pnr

    def _get_flow_from_cc(self, timestamp, cc):
        # precision issue, consistent with C++
        cc = np.around(cc, decimals=4)
        if any(timestamp >= cc[:, 0]):
            ind = np.nonzero(timestamp >= cc[:, 0])[0][-1]
        else:
            ind = 0
        return cc[ind, 1]

    def _get_timestamp_from_cc(self, flow, cc):
        # precision issue, consistent with C++
        cc = np.around(cc, decimals=4)
        if any(flow == cc[:, 1]):
            ind = np.nonzero(flow == cc[:, 1])[0][0]
        elif any(flow > cc[:, 1]):
            ind = np.nonzero(flow > cc[:, 1])[0][-1]
        else:
            ind = 0
        return cc[ind, 0]

    def _get_link_tt_from_cc(self, timestamp, cc_in, cc_out, fft):
        _flow = self._get_flow_from_cc(timestamp, cc_in)
        _timestamp_1 = self._get_timestamp_from_cc(_flow, cc_in)
        _timestamp_2 = self._get_timestamp_from_cc(_flow, cc_out)
        tt = (_timestamp_2 - _timestamp_1) * self.nb.config.config_dict['DTA']['unit_time']
        assert(tt >= 0)
        if tt < fft:
            tt = fft
        return tt

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
            # loss = self.config['link_car_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['car_link_flow']))
            loss = np.linalg.norm(np.nan_to_num(x_e - one_data_dict['car_link_flow']))
            loss_dict['car_count_loss'] = loss

        if self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss']:
            # num_links_driving x num_assign_intervals
            x_e = dta.get_link_truck_inflow(start_intervals, end_intervals).flatten(order='F')
            assert(len(x_e) == len(self.observed_links_driving) * self.num_assign_interval)
            if self.config['truck_count_agg']:
                x_e = one_data_dict['truck_count_agg_L'].dot(x_e)
            # loss = self.config['link_truck_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['truck_link_flow']))
            loss = np.linalg.norm(np.nan_to_num(x_e - one_data_dict['truck_link_flow']))
            loss_dict['truck_count_loss'] = loss

        if self.config['use_bus_link_flow'] or self.config['compute_bus_link_flow_loss']:
            # num_links_bus x num_assign_intervals
            x_e = dta.get_link_bus_inflow(start_intervals, end_intervals).flatten(order='F')
            assert(len(x_e) == len(self.observed_links_bus) * self.num_assign_interval)
            if self.config['bus_count_agg']:
                x_e = one_data_dict['bus_count_agg_L'].dot(x_e)
            # loss = self.config['link_bus_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['bus_link_flow']))
            loss = np.linalg.norm(np.nan_to_num(x_e - one_data_dict['bus_link_flow']))
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
            # loss = self.config['link_passenger_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['passenger_link_flow']))
            loss = np.linalg.norm(np.nan_to_num(x_e - one_data_dict['passenger_link_flow']))
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
            x_tt_e = dta.get_car_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
                                                np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq,
                                                self.ass_freq).flatten(order='F')
            # x_tt_e = dta.get_car_link_tt(start_intervals).flatten(order='F')
            assert(len(x_tt_e) == len(self.observed_links_driving) * self.num_assign_interval)
            # loss = self.config['link_car_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['car_link_tt']))
            loss = np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['car_link_tt']))
            loss_dict['car_tt_loss'] = loss
            
        if self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss']:
            # num_links_driving x num_assign_intervals
            x_tt_e = dta.get_truck_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
                                                  np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq,
                                                  self.ass_freq).flatten(order='F')
            # x_tt_e = dta.get_truck_link_tt(start_intervals).flatten(order='F')
            assert(len(x_tt_e) == len(self.observed_links_driving) * self.num_assign_interval)
            # loss = self.config['link_truck_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['truck_link_tt']))
            loss = np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['truck_link_tt']))
            loss_dict['truck_tt_loss'] = loss

        if self.config['use_bus_link_tt'] or self.config['compute_bus_link_tt_loss']:
            # num_links_bus x num_assign_intervals
            x_tt_e = dta.get_bus_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
                                                np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq,
                                                self.ass_freq).flatten(order='F')
            # x_tt_e = dta.get_bus_link_tt(start_intervals).flatten(order='F')
            assert(len(x_tt_e) == len(self.observed_links_bus) * self.num_assign_interval)
            # loss = self.config['link_bus_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['bus_link_tt']))
            loss = np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['bus_link_tt']))
            loss_dict['bus_tt_loss'] = loss
            
        if self.config['use_passenger_link_tt'] or self.config['compute_passenger_link_tt_loss']:
            # (num_links_bus + num_links_walking) x num_assign_intervals
            x_tt_e_bus = dta.get_bus_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq,
                                                    self.ass_freq)
            # x_tt_e_bus = dta.get_bus_link_tt(start_intervals)
            assert(x_tt_e_bus.shape[0] == len(self.observed_links_bus))
            x_tt_e_walking = dta.get_passenger_walking_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
                                                                      np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq,
                                                                      self.ass_freq)
            # x_tt_e_walking = dta.get_passenger_walking_link_tt(start_intervals)
            assert(x_tt_e_walking.shape[0] == len(self.observed_links_walking))
            x_tt_e = np.concatenate((x_tt_e_bus, x_tt_e_walking), axis=0).flatten(order='F')
            # loss = self.config['link_passenger_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['passenger_link_tt']))
            loss = np.linalg.norm(np.nan_to_num(x_tt_e - one_data_dict['passenger_link_tt']))
            loss_dict['passenger_tt_loss'] = loss

        total_loss = 0.0
        for loss_type, loss_value in loss_dict.items():
            total_loss += loss_value
        return total_loss, loss_dict
        
    def estimate_path_flow_pytorch(self, car_driving_scale=10, truck_driving_scale=1, passenger_bustransit_scale=1, car_pnr_scale=5,
                                   car_driving_step_size=0.1, truck_driving_step_size=0.01, passenger_bustransit_step_size=0.01, car_pnr_step_size=0.05,
                                   link_car_flow_weight=1, link_truck_flow_weight=1, link_passenger_flow_weight=1, link_bus_flow_weight=1,
                                   link_car_tt_weight=1, link_truck_tt_weight=1, link_passenger_tt_weight=1, link_bus_tt_weight=1,
                                   max_epoch=100, column_generation=False, use_tdsp=False, use_file_as_init=None, save_folder=None, starting_epoch=0):
        
        if np.isscalar(column_generation):
            column_generation = np.ones(max_epoch, dtype=int) * column_generation
        assert(len(column_generation) == max_epoch)

        if np.isscalar(link_car_flow_weight):
            link_car_flow_weight = np.ones(max_epoch, dtype=int) * link_car_flow_weight
        assert(len(link_car_flow_weight) == max_epoch)

        if np.isscalar(link_truck_flow_weight):
            link_truck_flow_weight = np.ones(max_epoch, dtype=int) * link_truck_flow_weight
        assert(len(link_truck_flow_weight) == max_epoch)

        if np.isscalar(link_passenger_flow_weight):
            link_passenger_flow_weight = np.ones(max_epoch, dtype=int) * link_passenger_flow_weight
        assert(len(link_passenger_flow_weight) == max_epoch)

        if np.isscalar(link_bus_flow_weight):
            link_bus_flow_weight = np.ones(max_epoch, dtype=int) * link_bus_flow_weight
        assert(len(link_bus_flow_weight) == max_epoch)

        if np.isscalar(link_car_tt_weight):
            link_car_tt_weight = np.ones(max_epoch, dtype=int) * link_car_tt_weight
        assert(len(link_car_tt_weight) == max_epoch)

        if np.isscalar(link_truck_tt_weight):
            link_truck_tt_weight = np.ones(max_epoch, dtype=int) * link_truck_tt_weight
        assert(len(link_truck_tt_weight) == max_epoch)

        if np.isscalar(link_passenger_tt_weight):
            link_passenger_tt_weight = np.ones(max_epoch, dtype=int) * link_passenger_tt_weight
        assert(len(link_passenger_tt_weight) == max_epoch)

        if np.isscalar(link_bus_tt_weight):
            link_bus_tt_weight = np.ones(max_epoch, dtype=int) * link_bus_tt_weight
        assert(len(link_bus_tt_weight) == max_epoch)
        
        loss_list = list()

        # read from files as init values
        if use_file_as_init is not None:
            _, _, f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = pickle.load(open(use_file_as_init, 'rb'))
            
            self.nb.update_demand_path_driving(f_car_driving, f_truck_driving)
            self.nb.update_demand_path_bustransit(f_passenger_bustransit)
            self.nb.update_demand_path_pnr(f_car_pnr)
            self.nb.get_mode_portion_matrix()
        else:
            f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
                self.init_path_flow(car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)

        # fixed bus path flow
        f_bus = self.nb.demand_bus.path_flow_matrix.flatten(order='F')

        f_car_driving = np.maximum(f_car_driving, 1e-3 / self.nb.config.config_dict['DTA']['flow_scalar'])
        f_truck_driving = np.maximum(f_truck_driving, 1e-3 / self.nb.config.config_dict['DTA']['flow_scalar'])
        # this alleviate trapping in local minima, when f = 0 -> grad = 0 is not helping
        f_passenger_bustransit = np.maximum(f_passenger_bustransit, 1)
        f_car_pnr = np.maximum(f_car_pnr, 1e-3 / self.nb.config.config_dict['DTA']['flow_scalar'])
        f_bus = np.maximum(f_bus, 1e-6)

        f_car_driving_tensor = torch.from_numpy(f_car_driving / np.maximum(car_driving_scale, 1e-6))
        f_truck_driving_tensor = torch.from_numpy(f_truck_driving / np.maximum(truck_driving_scale, 1e-6))
        f_passenger_bustransit_tensor = torch.from_numpy(f_passenger_bustransit / np.maximum(passenger_bustransit_scale, 1e-6))
        f_car_pnr_tensor = torch.from_numpy(f_car_pnr / np.maximum(car_pnr_scale, 1e-6))

        # f_car_driving_tensor = torch.from_numpy(f_car_driving)
        # f_truck_driving_tensor = torch.from_numpy(f_truck_driving)
        # f_passenger_bustransit_tensor = torch.from_numpy(f_passenger_bustransit)
        # f_car_pnr_tensor = torch.from_numpy(f_car_pnr)

        f_car_driving_tensor.requires_grad = True
        f_truck_driving_tensor.requires_grad = True
        f_passenger_bustransit_tensor.requires_grad = True
        f_car_pnr_tensor.requires_grad = True

        optimizer = torch.optim.NAdam([
            {'params': f_car_driving_tensor, 'lr': car_driving_step_size},
            {'params': f_truck_driving_tensor, 'lr': truck_driving_step_size},
            {'params': f_passenger_bustransit_tensor, 'lr': passenger_bustransit_step_size},
            {'params': f_car_pnr_tensor, 'lr': car_pnr_step_size}
        ])

        # optimizer = torch.optim.Adam([
        #     {'params': f_car_driving_tensor, 'lr': car_driving_step_size},
        #     {'params': f_truck_driving_tensor, 'lr': truck_driving_step_size},
        #     {'params': f_passenger_bustransit_tensor, 'lr': passenger_bustransit_step_size},
        #     {'params': f_car_pnr_tensor, 'lr': car_pnr_step_size}
        # ], betas=(0.9, 0.99))

        # optimizer = torch.optim.Adagrad([
        #     {'params': f_car_driving_tensor, 'lr': car_driving_step_size},
        #     {'params': f_truck_driving_tensor, 'lr': truck_driving_step_size},
        #     {'params': f_passenger_bustransit_tensor, 'lr': passenger_bustransit_step_size},
        #     {'params': f_car_pnr_tensor, 'lr': car_pnr_step_size}
        # ])

        # scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        #     optimizer, mode='min', factor=0.95, patience=5, 
        #     threshold=0.15, threshold_mode='rel', cooldown=0, min_lr=0, eps=1e-08, verbose=False)

        best_epoch = starting_epoch
        best_f_car_driving, best_f_truck_driving, best_f_passenger_bustransit, best_f_car_pnr = 0, 0, 0, 0
        
        for i in range(max_epoch):
            seq = np.random.permutation(self.num_data)
            loss = np.float(0)
            loss_dict = {
                "car_count_loss": 0.,
                "truck_count_loss": 0.,
                "bus_count_loss": 0.,
                "passenger_count_loss": 0.,
                "car_tt_loss": 0.,
                "truck_tt_loss": 0.,
                "bus_tt_loss": 0.,
                "passenger_tt_loss": 0.
            }

            self.config['link_car_flow_weight'] = link_car_flow_weight[i] * (self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss'])
            self.config['link_truck_flow_weight'] = link_truck_flow_weight[i] * (self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss'])
            self.config['link_passenger_flow_weight'] = link_passenger_flow_weight[i] * (self.config['use_passenger_link_flow'] or self.config['compute_passenger_link_flow_loss'])
            self.config['link_bus_flow_weight'] = link_bus_flow_weight[i] * (self.config['use_bus_link_flow'] or self.config['compute_bus_link_flow_loss'])

            self.config['link_car_tt_weight'] = link_car_tt_weight[i] * (self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss'])
            self.config['link_truck_tt_weight'] = link_truck_tt_weight[i] * (self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss'])
            self.config['link_passenger_tt_weight'] = link_passenger_tt_weight[i] * (self.config['use_passenger_link_tt'] or self.config['compute_passenger_link_tt_loss'])
            self.config['link_bus_tt_weight'] = link_bus_tt_weight[i] * (self.config['use_bus_link_tt'] or self.config['compute_bus_link_tt_loss'])

            for j in seq:
                # retrieve one record of observed data
                one_data_dict = self._get_one_data(j)

                # f_grad: num_path * num_assign_interval
                f_car_driving_grad, f_truck_driving_grad, f_passenger_bustransit_grad, f_car_pnr_grad, f_passenger_pnr_grad, \
                    tmp_loss, tmp_loss_dict, dta = \
                        self.compute_path_flow_grad_and_loss(one_data_dict, f_car_driving, f_truck_driving, f_bus, f_passenger_bustransit, f_car_pnr, counter=0, run_mmdta_adaptive=False)

                if ~column_generation[i]:
                    dta = 0
                
                optimizer.zero_grad()

                f_car_driving_tensor.grad = torch.from_numpy(f_car_driving_grad * car_driving_scale)
                f_truck_driving_tensor.grad = torch.from_numpy(f_truck_driving_grad * truck_driving_scale)
                f_passenger_bustransit_tensor.grad = torch.from_numpy(f_passenger_bustransit_grad * passenger_bustransit_scale)
                f_car_pnr_tensor.grad = torch.from_numpy((f_car_pnr_grad + f_passenger_pnr_grad) * car_pnr_scale)

                # f_car_driving_tensor.grad = torch.from_numpy(f_car_driving_grad)
                # f_truck_driving_tensor.grad = torch.from_numpy(f_truck_driving_grad)
                # f_passenger_bustransit_tensor.grad = torch.from_numpy(f_passenger_bustransit_grad)
                # f_car_pnr_tensor.grad = torch.from_numpy((f_car_pnr_grad + f_passenger_pnr_grad) )

                optimizer.step()

                # release memory
                f_car_driving_grad, f_truck_driving_grad, f_passenger_bustransit_grad, f_car_pnr_grad, f_passenger_pnr_grad = 0, 0, 0, 0, 0
                optimizer.zero_grad()

                f_car_driving = f_car_driving_tensor.data.cpu().numpy() * car_driving_scale
                f_truck_driving = f_truck_driving_tensor.data.cpu().numpy() * truck_driving_scale
                f_passenger_bustransit = f_passenger_bustransit_tensor.data.cpu().numpy() * passenger_bustransit_scale
                f_car_pnr = f_car_pnr_tensor.data.cpu().numpy() * car_pnr_scale

                # f_car_driving = f_car_driving_tensor.data.cpu().numpy()
                # f_truck_driving = f_truck_driving_tensor.data.cpu().numpy()
                # f_passenger_bustransit = f_passenger_bustransit_tensor.data.cpu().numpy()
                # f_car_pnr = f_car_pnr_tensor.data.cpu().numpy()

                if column_generation[i]:
                    print("***************** generate new paths *****************")
                    if not self.config['use_car_link_tt'] and not self.config['use_truck_link_tt'] and not self.config['use_passenger_link_tt']:
                        # if any of these is true, dta.build_link_cost_map(True) is already invoked in compute_path_flow_grad_and_loss()
                        dta.build_link_cost_map(False)
                    self.update_path_table(dta, use_tdsp)
                    f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
                        self.update_path_flow(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
                                              car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)
                    dta = 0

                f_car_driving = np.maximum(f_car_driving, 1e-3 / self.nb.config.config_dict['DTA']['flow_scalar'])
                f_truck_driving = np.maximum(f_truck_driving, 1e-3 / self.nb.config.config_dict['DTA']['flow_scalar'])
                # this helps jump out of local minima, when f = 0 -> grad = 0 is not helping
                # f_passenger_bustransit = np.maximum(f_passenger_bustransit, np.random.choice([1e-3, 1]))
                f_passenger_bustransit = np.maximum(f_passenger_bustransit, 1)
                f_car_pnr = np.maximum(f_car_pnr, 1e-3 / self.nb.config.config_dict['DTA']['flow_scalar'])

                if column_generation[i]:
                    f_car_driving_tensor = torch.from_numpy(f_car_driving / np.maximum(car_driving_scale, 1e-6))
                    f_truck_driving_tensor = torch.from_numpy(f_truck_driving / np.maximum(truck_driving_scale, 1e-6))
                    f_passenger_bustransit_tensor = torch.from_numpy(f_passenger_bustransit / np.maximum(passenger_bustransit_scale, 1e-6))
                    f_car_pnr_tensor = torch.from_numpy(f_car_pnr / np.maximum(car_pnr_scale, 1e-6))

                    # f_car_driving_tensor = torch.from_numpy(f_car_driving)
                    # f_truck_driving_tensor = torch.from_numpy(f_truck_driving)
                    # f_passenger_bustransit_tensor = torch.from_numpy(f_passenger_bustransit)
                    # f_car_pnr_tensor = torch.from_numpy(f_car_pnr)

                    f_car_driving_tensor.requires_grad = True
                    f_truck_driving_tensor.requires_grad = True
                    f_passenger_bustransit_tensor.requires_grad = True
                    f_car_pnr_tensor.requires_grad = True

                    # optimizer = torch.optim.Adam([
                    #     {'params': f_car_driving_tensor, 'lr': car_driving_step_size},
                    #     {'params': f_truck_driving_tensor, 'lr': truck_driving_step_size},
                    #     {'params': f_passenger_bustransit_tensor, 'lr': passenger_bustransit_step_size},
                    #     {'params': f_car_pnr_tensor, 'lr': car_pnr_step_size}
                    # ], betas=(0.9, 0.99))

                    optimizer = torch.optim.NAdam([
                        {'params': f_car_driving_tensor, 'lr': car_driving_step_size},
                        {'params': f_truck_driving_tensor, 'lr': truck_driving_step_size},
                        {'params': f_passenger_bustransit_tensor, 'lr': passenger_bustransit_step_size},
                        {'params': f_car_pnr_tensor, 'lr': car_pnr_step_size}
                    ])

                    # optimizer = torch.optim.Adagrad([
                    #     {'params': f_car_driving_tensor, 'lr': car_driving_step_size},
                    #     {'params': f_truck_driving_tensor, 'lr': truck_driving_step_size},
                    #     {'params': f_passenger_bustransit_tensor, 'lr': passenger_bustransit_step_size},
                    #     {'params': f_car_pnr_tensor, 'lr': car_pnr_step_size}
                    # ])

                    # scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
                    #     optimizer, mode='min', factor=0.95, patience=5, 
                    #     threshold=0.15, threshold_mode='rel', cooldown=0, min_lr=0, eps=1e-08, verbose=False)

                loss += tmp_loss / np.float(self.num_data)
                for loss_type, loss_value in tmp_loss_dict.items():
                    loss_dict[loss_type] += loss_value / np.float(self.num_data)
            
            print("Epoch:", starting_epoch + i, "Loss:", loss, self.print_separate_accuracy(loss_dict))
            loss_list.append([loss, loss_dict])
            
            if (i == 0) or (loss_list[-1][0] < loss_list[-2][0]):
                best_epoch = starting_epoch + i
                best_f_car_driving, best_f_truck_driving, best_f_passenger_bustransit, best_f_car_pnr = f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr

                if save_folder is not None:
                    self.save_simulation_input_files(os.path.join(save_folder, 'input_files_estimate_path_flow'), 
                                                     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)
            
            # if 'passenger_count_loss' in loss_list[-1][-1]:
            #     scheduler.step(loss_list[-1][-1]['passenger_count_loss'])
            # else:
            #     scheduler.step(loss_list[-1][0])
            
            # scheduler.step(loss_list[-1][0])

            if save_folder is not None:
                pickle.dump([loss, loss_dict, 
                             f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr], 
                            open(os.path.join(save_folder, str(starting_epoch + i) + '_iteration_estimate_path_flow.pickle'), 'wb'))

                # if column_generation[i]:
                #     self.save_simulation_input_files(os.path.join(save_folder, 'input_files_estimate_path_flow'), 
                #                                      f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)

            # if column_generation[i]:
            #     dta.build_link_cost_map()
            #     self.update_path_table(dta, use_tdsp)
            #     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
            #         self.update_path_flow(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
            #                               car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)

        print("Best loss at Epoch:", best_epoch, "Loss:", loss_list[best_epoch - starting_epoch][0], self.print_separate_accuracy(loss_list[best_epoch - starting_epoch][1]))
        return best_f_car_driving, best_f_truck_driving, best_f_passenger_bustransit, best_f_car_pnr, loss_list
        

    def estimate_path_flow(self, car_driving_scale=10, truck_driving_scale=1, passenger_bustransit_scale=1, car_pnr_scale=5,
                           car_driving_step_size=0.1, truck_driving_step_size=0.01, passenger_bustransit_step_size=0.01, car_pnr_step_size=0.05,
                           link_car_flow_weight=1, link_truck_flow_weight=1, link_passenger_flow_weight=1, link_bus_flow_weight=1,
                           link_car_tt_weight=1, link_truck_tt_weight=1, link_passenger_tt_weight=1, link_bus_tt_weight=1,
                           max_epoch=100, adagrad=False, column_generation=False, use_tdsp=False, use_file_as_init=None, save_folder=None, starting_epoch=0):
        
        if np.isscalar(column_generation):
            column_generation = np.ones(max_epoch, dtype=int) * column_generation
        assert(len(column_generation) == max_epoch)

        if np.isscalar(car_driving_step_size):
            car_driving_step_size = np.ones(max_epoch, dtype=float) * car_driving_step_size
        assert(len(car_driving_step_size) == max_epoch)

        if np.isscalar(truck_driving_step_size):
            truck_driving_step_size = np.ones(max_epoch, dtype=float) * truck_driving_step_size
        assert(len(truck_driving_step_size) == max_epoch)

        if np.isscalar(passenger_bustransit_step_size):
            passenger_bustransit_step_size = np.ones(max_epoch, dtype=float) * passenger_bustransit_step_size
        assert(len(passenger_bustransit_step_size) == max_epoch)

        if np.isscalar(car_pnr_step_size):
            car_pnr_step_size = np.ones(max_epoch, dtype=float) * car_pnr_step_size
        assert(len(car_pnr_step_size) == max_epoch)

        if np.isscalar(link_car_flow_weight):
            link_car_flow_weight = np.ones(max_epoch, dtype=bool) * link_car_flow_weight
        assert(len(link_car_flow_weight) == max_epoch)

        if np.isscalar(link_truck_flow_weight):
            link_truck_flow_weight = np.ones(max_epoch, dtype=bool) * link_truck_flow_weight
        assert(len(link_truck_flow_weight) == max_epoch)

        if np.isscalar(link_passenger_flow_weight):
            link_passenger_flow_weight = np.ones(max_epoch, dtype=bool) * link_passenger_flow_weight
        assert(len(link_passenger_flow_weight) == max_epoch)

        if np.isscalar(link_bus_flow_weight):
            link_bus_flow_weight = np.ones(max_epoch, dtype=bool) * link_bus_flow_weight
        assert(len(link_bus_flow_weight) == max_epoch)

        if np.isscalar(link_car_tt_weight):
            link_car_tt_weight = np.ones(max_epoch, dtype=bool) * link_car_tt_weight
        assert(len(link_car_tt_weight) == max_epoch)

        if np.isscalar(link_truck_tt_weight):
            link_truck_tt_weight = np.ones(max_epoch, dtype=bool) * link_truck_tt_weight
        assert(len(link_truck_tt_weight) == max_epoch)

        if np.isscalar(link_passenger_tt_weight):
            link_passenger_tt_weight = np.ones(max_epoch, dtype=bool) * link_passenger_tt_weight
        assert(len(link_passenger_tt_weight) == max_epoch)

        if np.isscalar(link_bus_tt_weight):
            link_bus_tt_weight = np.ones(max_epoch, dtype=bool) * link_bus_tt_weight
        assert(len(link_bus_tt_weight) == max_epoch)
        
        loss_list = list()

        # read from files as init values
        if use_file_as_init is not None:
            _, _, f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = pickle.load(open(use_file_as_init, 'rb'))

            self.nb.update_demand_path_driving(f_car_driving, f_truck_driving)
            self.nb.update_demand_path_bustransit(f_passenger_bustransit)
            self.nb.update_demand_path_pnr(f_car_pnr)
        else:
            f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
                self.init_path_flow(car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)

        # fixed bus path flow
        f_bus = self.nb.demand_bus.path_flow_matrix.flatten(order='F')

        f_car_driving = np.maximum(f_car_driving, 1e-3 / self.nb.config.config_dict['DTA']['flow_scalar'])
        f_truck_driving = np.maximum(f_truck_driving, 1e-3 / self.nb.config.config_dict['DTA']['flow_scalar'])
        # this alleviate trapping in local minima, when f = 0 -> grad = 0 is not helping
        f_passenger_bustransit = np.maximum(f_passenger_bustransit, 1)
        f_car_pnr = np.maximum(f_car_pnr, 1e-3 / self.nb.config.config_dict['DTA']['flow_scalar'])
        f_bus = np.maximum(f_bus, 1e-6)

        best_epoch = starting_epoch
        best_f_car_driving, best_f_truck_driving, best_f_passenger_bustransit, best_f_car_pnr = 0, 0, 0, 0

        for i in range(max_epoch):
            seq = np.random.permutation(self.num_data)
            loss = np.float(0)
            loss_dict = {
                "car_count_loss": 0.,
                "truck_count_loss": 0.,
                "bus_count_loss": 0.,
                "passenger_count_loss": 0.,
                "car_tt_loss": 0.,
                "truck_tt_loss": 0.,
                "bus_tt_loss": 0.,
                "passenger_tt_loss": 0.
            }

            self.config['link_car_flow_weight'] = link_car_flow_weight[i] * (self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss'])
            self.config['link_truck_flow_weight'] = link_truck_flow_weight[i] * (self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss'])
            self.config['link_passenger_flow_weight'] = link_passenger_flow_weight[i] * (self.config['use_passenger_link_flow'] or self.config['compute_passenger_link_flow_loss'])
            self.config['link_bus_flow_weight'] = link_bus_flow_weight[i] * (self.config['use_bus_link_flow'] or self.config['compute_bus_link_flow_loss'])

            self.config['link_car_tt_weight'] = link_car_tt_weight[i] * (self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss'])
            self.config['link_truck_tt_weight'] = link_truck_tt_weight[i] * (self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss'])
            self.config['link_passenger_tt_weight'] = link_passenger_tt_weight[i] * (self.config['use_passenger_link_tt'] or self.config['compute_passenger_link_tt_loss'])
            self.config['link_bus_tt_weight'] = link_bus_tt_weight[i] * (self.config['use_bus_link_tt'] or self.config['compute_bus_link_tt_loss'])

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
                    tmp_loss, tmp_loss_dict, dta = \
                        self.compute_path_flow_grad_and_loss(one_data_dict, f_car_driving, f_truck_driving, f_bus, f_passenger_bustransit, f_car_pnr, counter=0, run_mmdta_adaptive=False)

                if ~column_generation[i]:
                    dta = 0
                
                if adagrad:
                    sum_g_square_car_driving = sum_g_square_car_driving + np.power(f_car_driving_grad, 2)
                    f_car_driving -= f_car_driving_grad * car_driving_step_size[i] / np.sqrt(sum_g_square_car_driving)

                    sum_g_square_truck_driving = sum_g_square_truck_driving + np.power(f_truck_driving_grad, 2)
                    f_truck_driving -= f_truck_driving_grad * truck_driving_step_size[i] / np.sqrt(sum_g_square_truck_driving)

                    sum_g_square_passenger_bustransit = sum_g_square_passenger_bustransit + np.power(f_passenger_bustransit_grad, 2)
                    f_passenger_bustransit -= f_passenger_bustransit_grad * passenger_bustransit_step_size[i] / np.sqrt(sum_g_square_passenger_bustransit)

                    sum_g_square_car_pnr = sum_g_square_car_pnr + np.power((f_car_pnr_grad + f_passenger_pnr_grad), 2)
                    f_car_pnr -= (f_car_pnr_grad + f_passenger_pnr_grad) * car_pnr_step_size[i] / np.sqrt(sum_g_square_car_pnr)
                else:
                    f_car_driving -= f_car_driving_grad * car_driving_step_size[i] / np.sqrt(i+1)
                    f_truck_driving -= f_truck_driving_grad * truck_driving_step_size[i] / np.sqrt(i+1)
                    f_passenger_bustransit -= f_passenger_bustransit_grad * passenger_bustransit_step_size[i] / np.sqrt(i+1)
                    f_car_pnr -= (f_car_pnr_grad + f_passenger_pnr_grad) * car_pnr_step_size[i] / np.sqrt(i+1)

                # release memory
                f_car_driving_grad, f_truck_driving_grad, f_passenger_bustransit_grad, f_car_pnr_grad, f_passenger_pnr_grad = 0, 0, 0, 0, 0

                if column_generation[i]:
                    print("***************** generate new paths *****************")
                    if not self.config['use_car_link_tt'] and not self.config['use_truck_link_tt'] and not self.config['use_passenger_link_tt']:
                        # if any of these is true, dta.build_link_cost_map(True) is already invoked in compute_path_flow_grad_and_loss()
                        dta.build_link_cost_map(False)
                    self.update_path_table(dta, use_tdsp)
                    f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
                        self.update_path_flow(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
                                              car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)
                    dta = 0
                
                f_car_driving = np.maximum(f_car_driving, 1e-3 / self.nb.config.config_dict['DTA']['flow_scalar'])
                f_truck_driving = np.maximum(f_truck_driving, 1e-3 / self.nb.config.config_dict['DTA']['flow_scalar'])
                # this alleviate trapping in local minima, when f = 0 -> grad = 0 is not helping
                f_passenger_bustransit = np.maximum(f_passenger_bustransit, 1)
                f_car_pnr = np.maximum(f_car_pnr, 1e-3 / self.nb.config.config_dict['DTA']['flow_scalar'])

                loss += tmp_loss / np.float(self.num_data)
                for loss_type, loss_value in tmp_loss_dict.items():
                    loss_dict[loss_type] += loss_value / np.float(self.num_data)
            
            print("Epoch:", starting_epoch + i, "Loss:", loss, self.print_separate_accuracy(loss_dict))
            loss_list.append([loss, loss_dict])

            if (i == 0) or (loss_list[-1][0] < loss_list[-2][0]):
                best_epoch = starting_epoch + i
                best_f_car_driving, best_f_truck_driving, best_f_passenger_bustransit, best_f_car_pnr = f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr

                if save_folder is not None:
                    self.save_simulation_input_files(os.path.join(save_folder, 'input_files_estimate_path_flow'), 
                                                     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)

            if save_folder is not None:
                pickle.dump([loss, loss_dict, 
                             f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr], 
                            open(os.path.join(save_folder, str(starting_epoch + i) + '_iteration_estimate_path_flow.pickle'), 'wb'))

                # if column_generation[i]:
                #     self.save_simulation_input_files(os.path.join(save_folder, 'input_files_estimate_path_flow'), 
                #                                      f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)

            # if column_generation[i]:
            #     dta.build_link_cost_map()
            #     self.update_path_table(dta, use_tdsp)
            #     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
            #         self.update_path_flow(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
            #                               car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)

        print("Best loss at Epoch:", best_epoch, "Loss:", loss_list[best_epoch - starting_epoch][0], self.print_separate_accuracy(loss_list[best_epoch - starting_epoch][1]))
        return best_f_car_driving, best_f_truck_driving, best_f_passenger_bustransit, best_f_car_pnr, loss_list

    def estimate_demand_pytorch(self, init_scale_passenger=10, init_scale_truck=10, 
                                car_driving_scale=10, truck_driving_scale=1, passenger_bustransit_scale=1, car_pnr_scale=5,
                                passenger_step_size=0.1, truck_step_size=0.01,
                                link_car_flow_weight=1, link_truck_flow_weight=1, link_passenger_flow_weight=1, link_bus_flow_weight=1,
                                link_car_tt_weight=1, link_truck_tt_weight=1, link_passenger_tt_weight=1, link_bus_tt_weight=1,
                                max_epoch=100, column_generation=False, use_tdsp=False,
                                alpha_mode=(1., 1.5, 2.), beta_mode=1, alpha_path=1, beta_path=1, 
                                use_file_as_init=None, save_folder=None, starting_epoch=0):

        if np.isscalar(column_generation):
            column_generation = np.ones(max_epoch, dtype=int) * column_generation
        assert(len(column_generation) == max_epoch)

        if np.isscalar(link_car_flow_weight):
            link_car_flow_weight = np.ones(max_epoch, dtype=bool) * link_car_flow_weight
        assert(len(link_car_flow_weight) == max_epoch)

        if np.isscalar(link_truck_flow_weight):
            link_truck_flow_weight = np.ones(max_epoch, dtype=bool) * link_truck_flow_weight
        assert(len(link_truck_flow_weight) == max_epoch)

        if np.isscalar(link_passenger_flow_weight):
            link_passenger_flow_weight = np.ones(max_epoch, dtype=bool) * link_passenger_flow_weight
        assert(len(link_passenger_flow_weight) == max_epoch)

        if np.isscalar(link_bus_flow_weight):
            link_bus_flow_weight = np.ones(max_epoch, dtype=bool) * link_bus_flow_weight
        assert(len(link_bus_flow_weight) == max_epoch)

        if np.isscalar(link_car_tt_weight):
            link_car_tt_weight = np.ones(max_epoch, dtype=bool) * link_car_tt_weight
        assert(len(link_car_tt_weight) == max_epoch)

        if np.isscalar(link_truck_tt_weight):
            link_truck_tt_weight = np.ones(max_epoch, dtype=bool) * link_truck_tt_weight
        assert(len(link_truck_tt_weight) == max_epoch)

        if np.isscalar(link_passenger_tt_weight):
            link_passenger_tt_weight = np.ones(max_epoch, dtype=bool) * link_passenger_tt_weight
        assert(len(link_passenger_tt_weight) == max_epoch)

        if np.isscalar(link_bus_tt_weight):
            link_bus_tt_weight = np.ones(max_epoch, dtype=bool) * link_bus_tt_weight
        assert(len(link_bus_tt_weight) == max_epoch)
        
        loss_list = list()

        # read from files as init values
        if use_file_as_init is not None:
            _, _, q_e_passenger, q_e_truck, q_e_mode_driving, q_e_mode_bustransit, q_e_mode_pnr, \
                f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = pickle.load(open(use_file_as_init, 'rb'))
            
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
        f_bus = np.maximum(f_bus, 1e-6)

        q_e_passenger_tensor = torch.from_numpy(q_e_passenger / np.maximum(init_scale_passenger, 1e-6))
        q_e_truck_tensor = torch.from_numpy(q_e_truck / np.maximum(init_scale_truck, 1e-6))

        q_e_passenger_tensor.requires_grad = True
        q_e_truck_tensor.requires_grad = True

        optimizer = torch.optim.NAdam([
            {'params': q_e_passenger_tensor, 'lr': passenger_step_size},
            {'params': q_e_truck_tensor, 'lr': truck_step_size}
        ])

        best_epoch = starting_epoch
        best_f_car_driving, best_f_truck_driving, best_f_passenger_bustransit, best_f_car_pnr, best_q_e_passenger, best_q_e_truck = 0, 0, 0, 0, 0, 0

        for i in range(max_epoch):
            seq = np.random.permutation(self.num_data)
            loss = np.float(0)
            loss_dict = {
                "car_count_loss": 0.,
                "truck_count_loss": 0.,
                "bus_count_loss": 0.,
                "passenger_count_loss": 0.,
                "car_tt_loss": 0.,
                "truck_tt_loss": 0.,
                "bus_tt_loss": 0.,
                "passenger_tt_loss": 0.
            }

            self.config['link_car_flow_weight'] = link_car_flow_weight[i] * (self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss'])
            self.config['link_truck_flow_weight'] = link_truck_flow_weight[i] * (self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss'])
            self.config['link_passenger_flow_weight'] = link_passenger_flow_weight[i] * (self.config['use_passenger_link_flow'] or self.config['compute_passenger_link_flow_loss'])
            self.config['link_bus_flow_weight'] = link_bus_flow_weight[i] * (self.config['use_bus_link_flow'] or self.config['compute_bus_link_flow_loss'])

            self.config['link_car_tt_weight'] = link_car_tt_weight[i] * (self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss'])
            self.config['link_truck_tt_weight'] = link_truck_tt_weight[i] * (self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss'])
            self.config['link_passenger_tt_weight'] = link_passenger_tt_weight[i] * (self.config['use_passenger_link_tt'] or self.config['compute_passenger_link_tt_loss'])
            self.config['link_bus_tt_weight'] = link_bus_tt_weight[i] * (self.config['use_bus_link_tt'] or self.config['compute_bus_link_tt_loss'])
            

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

                f_car_driving = np.maximum(f_car_driving, 1 / self.nb.config.config_dict['DTA']['flow_scalar'])
                f_truck_driving = np.maximum(f_truck_driving, 1 / self.nb.config.config_dict['DTA']['flow_scalar'])
                # this alleviate trapping in local minima, when f = 0 -> grad = 0 is not helping
                f_passenger_bustransit = np.maximum(f_passenger_bustransit, 1)
                f_car_pnr = np.maximum(f_car_pnr, 1 / self.nb.config.config_dict['DTA']['flow_scalar'])
                
                # f_grad: num_path * num_assign_interval
                f_car_driving_grad, f_truck_driving_grad, f_passenger_bustransit_grad, f_car_pnr_grad, f_passenger_pnr_grad, \
                    tmp_loss, tmp_loss_dict, dta = \
                        self.compute_path_flow_grad_and_loss(one_data_dict, f_car_driving, f_truck_driving, f_bus, f_passenger_bustransit, f_car_pnr, counter=0, run_mmdta_adaptive=False)
                
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

                q_e_passenger_tensor.grad = torch.from_numpy(q_passenger_grad * init_scale_passenger)
                q_e_truck_tensor.grad = torch.from_numpy(q_truck_grad * init_scale_truck)

                optimizer.step()

                # release memory
                f_car_driving_grad, f_truck_driving_grad, f_passenger_bustransit_grad, f_car_pnr_grad, f_passenger_pnr_grad = 0, 0, 0, 0, 0
                q_grad_car_driving, q_grad_car_pnr, q_truck_grad, q_grad_passenger_bustransit, q_grad_passenger_pnr, q_passenger_grad = 0, 0, 0, 0, 0, 0
                optimizer.zero_grad()

                q_e_passenger = q_e_passenger_tensor.data.cpu().numpy() * init_scale_passenger
                q_e_truck = q_e_truck_tensor.data.cpu().numpy() * init_scale_truck

                loss += tmp_loss / np.float(self.num_data)
                for loss_type, loss_value in tmp_loss_dict.items():
                    loss_dict[loss_type] += loss_value / np.float(self.num_data)

                if not self.config['use_car_link_tt'] and not self.config['use_truck_link_tt'] and not self.config['use_passenger_link_tt']:
                    # if any of these is true, dta.build_link_cost_map(True) is already invoked in compute_path_flow_grad_and_loss()
                    dta.build_link_cost_map(False)
                self.compute_path_cost(dta)
                if column_generation[i]:
                    print("***************** generate new paths *****************")
                    self.update_path_table(dta, use_tdsp)
                    f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
                        self.update_path_flow(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
                                              car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)
                    dta = 0
                # adjust modal split and path flow portion based on path cost and logit choice model
                self.assign_mode_route_portions(alpha_mode, beta_mode, alpha_path, beta_path)

            print("Epoch:", starting_epoch + i, "Loss:", loss, self.print_separate_accuracy(loss_dict))
            loss_list.append([loss, loss_dict])

            if (i == 0) or (loss_list[-1][0] < loss_list[-2][0]):
                best_epoch = starting_epoch + i
                best_f_car_driving, best_f_truck_driving, best_f_passenger_bustransit, best_f_car_pnr, best_q_e_passenger, best_q_e_truck = f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, q_e_passenger, q_e_truck

                if save_folder is not None:
                    self.save_simulation_input_files(os.path.join(save_folder, 'input_files_estimate_demand'), 
                                                     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)

            if save_folder is not None:
                pickle.dump([loss, loss_dict, 
                             q_e_passenger, q_e_truck, 
                             q_e_mode_driving, q_e_mode_bustransit, q_e_mode_pnr,
                             f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr], 
                            open(os.path.join(save_folder, str(starting_epoch + i) + '_iteration_estimate_demand.pickle'), 'wb'))

                # if column_generation[i]:
                #     self.save_simulation_input_files(os.path.join(save_folder, 'input_files_estimate_demand'), 
                #                                      f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)
        
        print("Best loss at Epoch:", best_epoch, "Loss:", loss_list[best_epoch - starting_epoch][0], self.print_separate_accuracy(loss_list[best_epoch - starting_epoch][1]))
        return best_f_car_driving, best_f_truck_driving, best_f_passenger_bustransit, best_f_car_pnr, best_q_e_passenger, best_q_e_truck, loss_list

    def estimate_demand(self, init_scale_passenger=10, init_scale_truck=10, 
                        car_driving_scale=10, truck_driving_scale=1, passenger_bustransit_scale=1, car_pnr_scale=5,
                        passenger_step_size=0.1, truck_step_size=0.01,
                        link_car_flow_weight=1, link_truck_flow_weight=1, link_passenger_flow_weight=1, link_bus_flow_weight=1,
                        link_car_tt_weight=1, link_truck_tt_weight=1, link_passenger_tt_weight=1, link_bus_tt_weight=1,
                        max_epoch=100, adagrad=False, column_generation=False, use_tdsp=False,
                        alpha_mode=(1., 1.5, 2.), beta_mode=1, alpha_path=1, beta_path=1, 
                        use_file_as_init=None, save_folder=None, starting_epoch=0):
        if np.isscalar(passenger_step_size):
            passenger_step_size = np.ones(max_epoch) * passenger_step_size
        if np.isscalar(truck_step_size):
            truck_step_size = np.ones(max_epoch) * truck_step_size
        assert(len(passenger_step_size) == max_epoch)
        assert(len(truck_step_size) == max_epoch)

        if np.isscalar(column_generation):
            column_generation = np.ones(max_epoch, dtype=int) * column_generation
        assert(len(column_generation) == max_epoch)

        if np.isscalar(link_car_flow_weight):
            link_car_flow_weight = np.ones(max_epoch, dtype=bool) * link_car_flow_weight
        assert(len(link_car_flow_weight) == max_epoch)

        if np.isscalar(link_truck_flow_weight):
            link_truck_flow_weight = np.ones(max_epoch, dtype=bool) * link_truck_flow_weight
        assert(len(link_truck_flow_weight) == max_epoch)

        if np.isscalar(link_passenger_flow_weight):
            link_passenger_flow_weight = np.ones(max_epoch, dtype=bool) * link_passenger_flow_weight
        assert(len(link_passenger_flow_weight) == max_epoch)

        if np.isscalar(link_bus_flow_weight):
            link_bus_flow_weight = np.ones(max_epoch, dtype=bool) * link_bus_flow_weight
        assert(len(link_bus_flow_weight) == max_epoch)

        if np.isscalar(link_car_tt_weight):
            link_car_tt_weight = np.ones(max_epoch, dtype=bool) * link_car_tt_weight
        assert(len(link_car_tt_weight) == max_epoch)

        if np.isscalar(link_truck_tt_weight):
            link_truck_tt_weight = np.ones(max_epoch, dtype=bool) * link_truck_tt_weight
        assert(len(link_truck_tt_weight) == max_epoch)

        if np.isscalar(link_passenger_tt_weight):
            link_passenger_tt_weight = np.ones(max_epoch, dtype=bool) * link_passenger_tt_weight
        assert(len(link_passenger_tt_weight) == max_epoch)

        if np.isscalar(link_bus_tt_weight):
            link_bus_tt_weight = np.ones(max_epoch, dtype=bool) * link_bus_tt_weight
        assert(len(link_bus_tt_weight) == max_epoch)
        
        loss_list = list()

        # read from files as init values
        if use_file_as_init is not None:
            _, _, q_e_passenger, q_e_truck, q_e_mode_driving, q_e_mode_bustransit, q_e_mode_pnr, \
                f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = pickle.load(open(use_file_as_init, 'rb'))
            
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
        f_bus = np.maximum(f_bus, 1e-6)

        best_epoch = starting_epoch
        best_f_car_driving, best_f_truck_driving, best_f_passenger_bustransit, best_f_car_pnr, best_q_e_passenger, best_q_e_truck = 0, 0, 0, 0, 0, 0

        for i in range(max_epoch):
            seq = np.random.permutation(self.num_data)
            loss = np.float(0)
            loss_dict = {
                "car_count_loss": 0.,
                "truck_count_loss": 0.,
                "bus_count_loss": 0.,
                "passenger_count_loss": 0.,
                "car_tt_loss": 0.,
                "truck_tt_loss": 0.,
                "bus_tt_loss": 0.,
                "passenger_tt_loss": 0.
            }

            self.config['link_car_flow_weight'] = link_car_flow_weight[i] * (self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss'])
            self.config['link_truck_flow_weight'] = link_truck_flow_weight[i] * (self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss'])
            self.config['link_passenger_flow_weight'] = link_passenger_flow_weight[i] * (self.config['use_passenger_link_flow'] or self.config['compute_passenger_link_flow_loss'])
            self.config['link_bus_flow_weight'] = link_bus_flow_weight[i] * (self.config['use_bus_link_flow'] or self.config['compute_bus_link_flow_loss'])

            self.config['link_car_tt_weight'] = link_car_tt_weight[i] * (self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss'])
            self.config['link_truck_tt_weight'] = link_truck_tt_weight[i] * (self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss'])
            self.config['link_passenger_tt_weight'] = link_passenger_tt_weight[i] * (self.config['use_passenger_link_tt'] or self.config['compute_passenger_link_tt_loss'])
            self.config['link_bus_tt_weight'] = link_bus_tt_weight[i] * (self.config['use_bus_link_tt'] or self.config['compute_bus_link_tt_loss'])

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

                f_car_driving = np.maximum(f_car_driving, 1 / self.nb.config.config_dict['DTA']['flow_scalar'])
                f_truck_driving = np.maximum(f_truck_driving, 1 / self.nb.config.config_dict['DTA']['flow_scalar'])
                # this alleviate trapping in local minima, when f = 0 -> grad = 0 is not helping
                f_passenger_bustransit = np.maximum(f_passenger_bustransit, 1)
                f_car_pnr = np.maximum(f_car_pnr, 1 / self.nb.config.config_dict['DTA']['flow_scalar'])
                
                # f_grad: num_path * num_assign_interval
                f_car_driving_grad, f_truck_driving_grad, f_passenger_bustransit_grad, f_car_pnr_grad, f_passenger_pnr_grad, \
                    tmp_loss, tmp_loss_dict, dta = \
                        self.compute_path_flow_grad_and_loss(one_data_dict, f_car_driving, f_truck_driving, f_bus, f_passenger_bustransit, f_car_pnr, counter=0, run_mmdta_adaptive=False)
                
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

                # release memory
                f_car_driving_grad, f_truck_driving_grad, f_passenger_bustransit_grad, f_car_pnr_grad, f_passenger_pnr_grad = 0, 0, 0, 0, 0
                q_grad_car_driving, q_grad_car_pnr, q_truck_grad, q_grad_passenger_bustransit, q_grad_passenger_pnr, q_passenger_grad = 0, 0, 0, 0, 0, 0
                
                # q_e_passenger = np.maximum(q_e_passenger, 1e-6)
                # q_e_truck = np.maximum(q_e_truck, 1e-6)

                loss += tmp_loss / np.float(self.num_data)
                for loss_type, loss_value in tmp_loss_dict.items():
                    loss_dict[loss_type] += loss_value / np.float(self.num_data)

                if not self.config['use_car_link_tt'] and not self.config['use_truck_link_tt'] and not self.config['use_passenger_link_tt']:
                    # if any of these is true, dta.build_link_cost_map(True) is already invoked in compute_path_flow_grad_and_loss()
                    dta.build_link_cost_map(False)
                self.compute_path_cost(dta)
                if column_generation[i]:
                    print("***************** generate new paths *****************")
                    self.update_path_table(dta, use_tdsp)
                    f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr = \
                        self.update_path_flow(f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr,
                                              car_driving_scale, truck_driving_scale, passenger_bustransit_scale, car_pnr_scale)
                    dta = 0
                # adjust modal split and path flow portion based on path cost and logit choice model
                self.assign_mode_route_portions(alpha_mode, beta_mode, alpha_path, beta_path)

            print("Epoch:", starting_epoch + i, "Loss:", loss, self.print_separate_accuracy(loss_dict))
            loss_list.append([loss, loss_dict])

            if (i == 0) or (loss_list[-1][0] < loss_list[-2][0]):
                best_epoch = starting_epoch + i
                best_f_car_driving, best_f_truck_driving, best_f_passenger_bustransit, best_f_car_pnr, best_q_e_passenger, best_q_e_truck = f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr, q_e_passenger, q_e_truck

                if save_folder is not None:
                    self.save_simulation_input_files(os.path.join(save_folder, 'input_files_estimate_demand'), 
                                                     f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)

            if save_folder is not None:
                pickle.dump([loss, loss_dict, 
                             q_e_passenger, q_e_truck, 
                             q_e_mode_driving, q_e_mode_bustransit, q_e_mode_pnr,
                             f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr], 
                            open(os.path.join(save_folder, str(starting_epoch + i) + '_iteration_estimate_demand.pickle'), 'wb'))

                # if column_generation[i]:
                #     self.save_simulation_input_files(os.path.join(save_folder, 'input_files_estimate_demand'), 
                #                                      f_car_driving, f_truck_driving, f_passenger_bustransit, f_car_pnr)
        
        print("Best loss at Epoch:", best_epoch, "Loss:", loss_list[best_epoch - starting_epoch][0], self.print_separate_accuracy(loss_list[best_epoch - starting_epoch][1]))
        return best_f_car_driving, best_f_truck_driving, best_f_passenger_bustransit, best_f_car_pnr, best_q_e_passenger, best_q_e_truck, loss_list

    def update_path_table(self, dta, use_tdsp=True):
        # dta.build_link_cost_map() should be called before this method

        start_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        self.nb.update_path_table(dta, start_intervals, use_tdsp)

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
    # given alpha >=0, beta > 0, decreasing alpha and beta will leading to a more evenly distribution; otherwise more concentrated distribution
    scale_cost = - (alpha + beta * cost)
    if max_cut:
        e_x = np.exp(scale_cost - np.max(scale_cost))
    else:
        e_x = np.exp(scale_cost)
    p = np.maximum(e_x / e_x.sum(), 1e-6)
    return p