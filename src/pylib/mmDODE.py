import os
import numpy as np
import pandas as pd
import hashlib
import time
import shutil
from scipy.sparse import coo_matrix, csr_matrix
import pickle
import multiprocessing as mp

import MNMAPI

class MMDODE:
    def __init__(self, nb, config):
        self.config = config
        self.nb = nb
        self.num_assign_interval = nb.config.config_dict['DTA']['max_interval']
        self.ass_freq = nb.config.config_dict['DTA']['assign_frq']
        self.num_link = nb.config.config_dict['DTA']['num_of_link']
        self.num_path = nb.config.config_dict['FIXED']['num_path']
        if nb.config.config_dict['DTA']['total_interval'] > 0 and nb.config.config_dict['DTA']['total_interval'] > self.num_assign_interval * self.ass_freq:
        self.num_loading_interval = nb.config.config_dict['DTA']['total_interval']
        else:
        self.num_loading_interval = self.num_assign_interval * self.ass_freq  # not long enough
        self.data_dict = dict()
        self.num_data = self.config['num_data']
        self.observed_links = self.config['observed_links']
        self.paths_list = self.config['paths_list']
        self.car_count_agg_L_list = None
        self.truck_count_agg_L_list = None
        assert (len(self.paths_list) == self.num_path)

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

    def _add_walking_link_flow_data(self, link_flow_df_list):
        # assert(self.config['use_walking_link_flow'])
        assert (self.num_data == len(link_flow_df_list))
        self.data_dict['walking_link_flow'] = link_flow_df_list

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

    def _add_walking_link_tt_data(self, link_spd_df_list):
        # assert(self.config['use_bus_link_tt'])
        assert (self.num_data == len(link_spd_df_list))
        self.data_dict['walking_link_tt'] = link_spd_df_list