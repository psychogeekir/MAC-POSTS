import os
import numpy as np
import sys

sys.path.append("/home/lemma/Documents/MAC-POSTS/side_project/network_builder")
from MNM_mcnb import *
import MNMAPI


class Scenario():
  def __init__(self, name, folder):
    self.name = name
    self.folder = folder
    self.nb = MNM_network_builder()
    self.nb.load_from_folder(self.folder)

  def run_simulation(self):
    self.simulation = Simulator(self.nb)
    self.simulation.run_simulation()

class ScenarioManager():
  def __init__(self):
    pass

  def add_scenarios(self, s):
    pass


class LinkAttention():
  def __init__(self, link_ID_list, name):
    self.name = name
    self.link_ID_list = link_ID_list


class ODAttention():
  def __init__(self, name):
    self.name = name
  
  def build_from_links(self, O_links, D_links, nb):
    self.D_nodes = list(set(list(map(lambda x: nb.graph.edgeID_dict[x][1], D_links))))
    self.Ds = list(map(lambda x: nb.od.D_dict.inv[x], self.D_nodes))
    self.O_nodes = list(set(list(map(lambda x: nb.graph.edgeID_dict[x][0], O_links))))
    self.Os = list(map(lambda x: nb.od.O_dict.inv[x], self.O_nodes))
    need_path_list = list()
    for path_ID, path in nb.path_table.ID2path.items():
        if nb.od.O_dict.inv[path.origin_node] in self.Os and nb.od.D_dict.inv[path.destination_node] in self.Ds:
            need_path_list.append(self._nodelist2linklist(path.node_list, nb))
    if len(need_path_list) > 0:
      self.path_list = need_path_list
      return
    else:
      for O_node in self.O_nodes:
        for D_node in self.D_nodes:
          need_path_list.append(self._get_shortest_node_path(O_node, D_node, nb))
      self.path_list = need_path_list


  def _nodelist2linklist(self, node_list, nb):
      link_list = list()
      for i in range(len(node_list) - 1):
          st_node = node_list[i]
          ed_node = node_list[i+1]
          link_list.append(nb.graph.G[st_node][ed_node]['ID'])
      return link_list

  def _get_shortest_node_path(self, O_node, D_node, nb):
    G = nb.graph.G.copy()
    ID2link = dict()
    for link in nb.link_list:
        ID2link[link.ID] = link
    for edge in G.edges():
        link_ID = G[edge[0]][edge[1]]['ID']
        link = ID2link[link_ID]
        G[edge[0]][edge[1]]['w'] = link.get_car_fft()
    path_node_list = nx.shortest_path(G, O_node, D_node, weight = "w")
    path_link_list = self._nodelist2linklist(path_node_list, nb)
    return path_link_list



class AttentionSet():
  def __init__(self):
    self.la_list = list()
    self.oda_list = list()

  def add_link_attention(self, la):
    self.la_list.append(la)

  def add_OD_attention(self, oda):
    self.oda_list.append(oda)



class Center():
  def __init__(self):
    self.attention = None
    self.scenarios = None

  def init_attentionset(self, link_ID_dict, OD_dict):
    self.attentionset = AttentionSet()
    for name, ID in link_ID_dict.items():
      tmp_la = LinkAttention(link_ID, name)
      self.attentionset.add_link_attention(tmp_la)
    for name, (O,D) in OD_dict.items():
      tmp_oda = ODAttention(O, D, name)
      self.attentionset.add_OD_attention(tmp_oda)

  def init_scenarios(self, scenario_dict):
    self.scenarios = ScenarioManager()
    for name, folder in scenario_dict.items():
      s = Scenario(name, folder)
      self.scenarios.add_scenarios(s)

  def prepare(self):
    pass

  def link_delays(self):
    pass

  def link_volumes(self):
    pass

  def agg_stats(self):
    pass

  def path_tt(self):
    pass

  def emission(self):
    pass


class Simulator():
  def __init__(self, nb):
    self.num_assign_interval = nb.config.config_dict['DTA']['max_interval']
    self.nb = nb
    self.link_ID_list = list(map(lambda x: x.ID, filter(lambda x: x.typ in ['CTM', 'LQ'], self.nb.link_list)))
    self.dta = MNMAPI.mcdta_api()

  def run_simulation(self):
    self.dta.initialize(self.nb.path)
    self.dta.register_links(self.link_ID_list)
    self.dta.install_cc()
    self.dta.run_whole()

  def print_out_overall_stats(self):
    self.dta.print_emission_stats()

  def get_enroute_and_queue_veh_stats_agg(self):
    return self.dta.get_enroute_and_queue_veh_stats_agg()

  def get_queue_veh_each_link(self, useful_links, intervals):
    return self.dta.get_queue_veh_each_link(useful_links, intervals)

  def get_link_car_count(self, start_intervals, end_intervals):
    return self.dta.get_link_car_inflow(start_intervals, end_intervals)

  def get_link_truck_count(self, start_intervals, end_intervals):
    return self.dta.get_link_truck_inflow(start_intervals, end_intervals)

  def get_link_car_speed(self, start_intervals):
    return self.dta.get_car_link_speed(start_intervals)

  def get_link_truck_speed(self, start_intervals):
    return self.dta.get_truck_link_speed(start_intervals)

  def get_link_car_tt(self, start_intervals):
    return self.dta.get_car_link_tt(start_intervals) * 5  # seconds

  def get_link_truck_tt(self, start_intervals):
    return self.dta.get_truck_link_tt(start_intervals) * 5 # seconds

  def get_waiting_time_at_intersections(self):
    return self.dta.get_waiting_time_at_intersections()

  def get_link_spillback(self):
    return self.dta.get_link_spillback()

  def get_path_tt_car(self, path_links, start_intervals):
    return self.dta.get_path_tt_car(path_links, start_intervals)
  
  def get_path_tt_truck(self, path_links, start_intervals):
    return self.dta.get_path_tt_truck(path_links, start_intervals)
