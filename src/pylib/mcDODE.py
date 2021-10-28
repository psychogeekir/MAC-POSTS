import os
import numpy as np
import pandas as pd
import hashlib
import time
import shutil
from scipy.sparse import coo_matrix, csr_matrix
import pickle
import multiprocessing as mp
import torch

import MNMAPI


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

  def _run_simulation(self, f_car, f_truck, counter=0):
    # create a new_folder with a unique name
    hash1 = hashlib.sha1()
    # python 2
    # hash1.update(str(time.time()) + str(counter))
    # python 3
    hash1.update((str(time.time()) + str(counter)).encode('utf-8'))
    new_folder = str(hash1.hexdigest())
    # modify demand based on input path flows
    self.nb.update_demand_path2(f_car, f_truck)
    self.nb.config.config_dict['DTA']['total_interval'] = self.num_loading_interval
    # save modified files in new_folder
    self.nb.dump_to_folder(new_folder)
    # invoke MNMAPI
    a = MNMAPI.mcdta_api()
    # read all files in new_folder
    a.initialize(new_folder)
    # delete new_folder and all files and subdirectories below it.
    shutil.rmtree(new_folder)
    # register links and paths
    a.register_links(self.observed_links)
    a.register_paths(self.paths_list)
    # install cc and cc_tree on registered links
    a.install_cc()
    a.install_cc_tree()
    # run DNL
    a.run_whole()
    # print("Finish simulation", time.time())
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
    dta = self._run_simulation(f_car, f_truck, counter)
    # print("Getting DAR", time.time())
    (car_dar, truck_dar) = self.get_dar(dta, f_car, f_truck)
    # print("Evaluating grad", time.time())
    car_grad = np.zeros(len(self.observed_links) * self.num_assign_interval)
    truck_grad = np.zeros(len(self.observed_links) * self.num_assign_interval)
    if self.config['use_car_link_flow']:
      # print("car link flow", time.time())
      car_grad += self.config['link_car_flow_weight'] * self._compute_grad_on_car_link_flow(dta, one_data_dict)
    if self.config['use_truck_link_flow']:
      truck_grad += self.config['link_truck_flow_weight'] * self._compute_grad_on_truck_link_flow(dta, one_data_dict)
    if self.config['use_car_link_tt']:
      car_grad += self.config['link_car_tt_weight'] * self._compute_grad_on_car_link_tt(dta, one_data_dict)
    if self.config['use_truck_link_tt']:
      truck_grad += self.config['link_truck_tt_weight'] * self._compute_grad_on_truck_link_tt(dta, one_data_dict)
    # print("Getting Loss", time.time())
    total_loss, loss_dict = self._get_loss(one_data_dict, dta)
    return car_dar.T.dot(car_grad), truck_dar.T.dot(truck_grad), total_loss, loss_dict

  def _compute_grad_on_car_link_flow(self, dta, one_data_dict):
    link_flow_array = one_data_dict['car_link_flow']
    x_e = dta.get_link_car_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                  np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
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
    x_e = dta.get_link_truck_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
    if self.config['truck_count_agg']:
        x_e = one_data_dict['truck_count_agg_L'].dot(x_e)
    grad = -np.nan_to_num(link_flow_array - x_e)
    if self.config['truck_count_agg']:
      grad = one_data_dict['truck_count_agg_L'].T.dot(grad)
    return grad

  def _compute_grad_on_car_link_tt(self, dta, one_data_dict):
    # tt_e = dta.get_car_link_tt_robust(np.arange(0, self.num_loading_interval, self.ass_freq),
    #                                   np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
    tt_e = dta.get_car_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
    tt_free = np.tile(list(map(lambda x: self.nb.get_link(x).get_car_fft(), self.observed_links)), (self.num_assign_interval))
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
    tt_free = np.tile(list(map(lambda x: self.nb.get_link(x).get_truck_fft(), self.observed_links)), (self.num_assign_interval))
    tt_e = np.maximum(tt_e, tt_free)
    tt_o = np.maximum(one_data_dict['truck_link_tt'], tt_free)
    grad = -np.nan_to_num(tt_o - tt_e)/tt_e
    # if self.config['truck_count_agg']:
    #   grad = one_data_dict['truck_count_agg_L'].T.dot(grad)
    # print("truck_grad", grad)
    return grad

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
      #                                     np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
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

  def estimate_path_flow_pytorch(self, car_step_size=0.1, truck_step_size=0.1, max_epoch=10, car_init_scale=10,
                                 truck_init_scale=1, store_folder=None, use_file_as_init=None):
    # here the basic variables to be estimated are path flows, not OD demand, so no route choice model, unlike in sDODE.py
    if use_file_as_init is None:
      (f_car, f_truck) = self.init_path_flow(car_scale=car_init_scale, truck_scale=truck_init_scale)
    else:
      (f_car, f_truck, _) = pickle.load(open(use_file_as_init, 'rb'))
    loss_list = list()

    f_car_tensor = torch.from_numpy(f_car)
    f_truck_tensor = torch.from_numpy(f_truck)

    f_car_tensor.requires_grad = True
    f_truck_tensor.requires_grad = True

    optimizer = torch.optim.Adam([
        {'params': f_car_tensor, 'lr': car_step_size},
        {'params': f_truck_tensor, 'lr': truck_step_size}
    ])

    for i in range(max_epoch):
      
      seq = np.random.permutation(self.num_data)
      loss = np.float(0)
      # print("Start iteration", time.time())
      loss_dict = {'car_count_loss': 0.0, 'truck_count_loss': 0.0, 'car_tt_loss': 0.0, 'truck_tt_loss': 0.0}
      for j in seq:
        one_data_dict = self._get_one_data(j)
        car_grad, truck_grad, tmp_loss, tmp_loss_dict = self.compute_path_flow_grad_and_loss(one_data_dict, f_car, f_truck)
        # print("gradient", car_grad, truck_grad)

        optimizer.zero_grad()

        f_car_tensor.grad = torch.from_numpy(car_grad)
        f_truck_tensor.grad = torch.from_numpy(truck_grad)

        optimizer.step()
        
        f_car_tensor = torch.maximum(f_car_tensor, torch.tensor(1e-3))
        f_truck_tensor = torch.maximum(f_truck_tensor, torch.tensor(1e-3))
        # f_truck_tensor = torch.minimum(f_truck_tensor, torch.tensor(30))

        f_car = f_car_tensor.data.cpu().numpy()
        f_truck = f_truck_tensor.data.cpu().numpy()

        loss += tmp_loss
        for loss_type, loss_value in tmp_loss_dict.items():
          loss_dict[loss_type] += loss_value / np.float(self.num_data)
      print("Epoch:", i, "Loss:", np.round(loss / np.float(self.num_data), 2), self.print_separate_accuracy(loss_dict))
      # print(f_car, f_truck)
      # break
      if store_folder is not None:
        pickle.dump((f_car, f_truck, loss), open(os.path.join(store_folder, str(i) + 'iteration.pickle'), 'wb'))
      loss_list.append([loss, loss_dict])
    return f_car, f_truck, loss_list

  def estimate_path_flow(self, car_step_size=0.1, truck_step_size=0.1, max_epoch=10, car_init_scale=10,
                         truck_init_scale=1, store_folder=None, use_file_as_init=None, adagrad=False):
    # here the basic variables to be estimated are path flows, not OD demand, so no route choice model, unlike in sDODE.py
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
        car_grad, truck_grad, tmp_loss, tmp_loss_dict = self.compute_path_flow_grad_and_loss(one_data_dict, f_car, f_truck)
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
      print("Epoch:", i, "Loss:", np.round(loss / np.float(self.num_data), 2), self.print_separate_accuracy(loss_dict))
      # print(f_car, f_truck)
      # break
      if store_folder is not None:
        pickle.dump((f_car, f_truck, loss), open(os.path.join(store_folder, str(i) + 'iteration.pickle'), 'wb'))
      loss_list.append([loss, loss_dict])
    return f_car, f_truck, loss_list

  def estimate_path_flow_gd(self, car_step_size=0.1, truck_step_size=0.1, max_epoch=10, car_init_scale=10,
                            truck_init_scale=1, store_folder=None, use_file_as_init=None, adagrad=False):
    if use_file_as_init is None:
      (f_car, f_truck) = self.init_path_flow(car_scale=car_init_scale, truck_scale=truck_init_scale)
    else:
      (f_car, f_truck, _) = pickle.load(open(use_file_as_init, 'rb'))
    loss_list = list()
    start_time = time.time()
    for i in range(max_epoch):
      grad_car_sum = np.zeros(f_car.shape)
      grad_truck_sum = np.zeros(f_truck.shape)
      seq = np.random.permutation(self.num_data)
      loss = np.float(0)
      # print("Start iteration", time.time())
      loss_dict = {'car_count_loss': 0.0, 'truck_count_loss': 0.0, 'car_tt_loss': 0.0, 'truck_tt_loss': 0.0}
      for j in seq:
        one_data_dict = self._get_one_data(j)
        car_grad, truck_grad, tmp_loss, tmp_loss_dict = self.compute_path_flow_grad_and_loss(one_data_dict, f_car, f_truck)
        # print("gradient", car_grad, truck_grad)
       
        grad_car_sum += car_grad
        grad_truck_sum += truck_grad
        #f_truck = np.minimum(f_truck, 30)
        loss += tmp_loss
        for loss_type, loss_value in tmp_loss_dict.items():
          loss_dict[loss_type] += loss_value / np.float(self.num_data)
      f_car -= grad_car_sum * car_step_size / np.sqrt(i+1) / np.float(self.num_data)
      f_truck -= grad_truck_sum * truck_step_size / np.sqrt(i+1) / np.float(self.num_data)
      f_car = np.maximum(f_car, 1e-3)
      f_truck = np.maximum(f_truck, 1e-3)
      print("Epoch:", i, "Loss:", np.round(loss / np.float(self.num_data), 2), self.print_separate_accuracy(loss_dict))
      # print(f_car, f_truck)
      # break
      if store_folder is not None:
        pickle.dump((f_car, f_truck, loss, loss_dict, time.time() - start_time), open(os.path.join(store_folder, str(i) + 'iteration.pickle'), 'wb'))
      loss_list.append([loss, loss_dict])
    return f_car, f_truck, loss_list

  def compute_path_flow_grad_and_loss_mpwrapper(self, one_data_dict, f_car, f_truck, j, output):
    car_grad, truck_grad, tmp_loss, tmp_loss_dict = self.compute_path_flow_grad_and_loss(one_data_dict, f_car, f_truck, counter=j)
    # print("finished original grad loss")
    output.put([car_grad, truck_grad, tmp_loss, tmp_loss_dict])
    # output.put(grad)
    # print("finished put")
    return

  def estimate_path_flow_mp(self, car_step_size=0.1, truck_step_size=0.1, max_epoch=10, car_init_scale=10,
                            truck_init_scale=1, store_folder=None, use_file_as_init=None,
                            adagrad=False, n_process=4, record_time=False):
    if use_file_as_init is None:
      (f_car, f_truck) = self.init_path_flow(car_scale=car_init_scale, truck_scale=truck_init_scale)
    else:
      (f_car, f_truck, _) = pickle.load(open(use_file_as_init, 'rb'))
    loss_list = list()
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
          [car_grad, truck_grad, tmp_loss, tmp_loss_dict] = res
          loss += tmp_loss
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
      print("Epoch:", i, "Loss:", loss / np.float(self.num_data), self.print_separate_accuracy(loss_dict))
      if store_folder is not None:
        pickle.dump((f_car, f_truck, loss, loss_dict, time.time() - start_time), open(os.path.join(store_folder, str(i) + 'iteration.pickle'), 'wb'))
      if record_time:
        loss_list.append([loss, loss_dict, time.time() - start_time])
      else:
        loss_list.append([loss, loss_dict])
    return f_car, f_truck, loss_list

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

  def _run_simulation(self, f_car, f_truck, counter=0):
    hash1 = hashlib.sha1()
    hash1.update(str(time.time()) + str(counter))
    new_folder = str(hash1.hexdigest())
    self.nb.update_demand_path2(f_car, f_truck)
    self.nb.config.config_dict['DTA']['total_interval'] = self.num_loading_interval
    self.nb.dump_to_folder(new_folder)
    a = MNMAPI.mcdta_api()
    a.initialize(new_folder)
    shutil.rmtree(new_folder)
    a.register_links(self.observed_links)
    a.register_paths(self.paths_list)
    a.install_cc()
    a.install_cc_tree()
    a.run_whole()
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
