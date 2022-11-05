import os
import numpy as np
import networkx as nx
from bidict import bidict
from collections import OrderedDict

# link type in MAC-POSTS/src/minami/dlink.cpp: cell transmission model (CTM), link queue (LQ), point queue (PQ)
DLINK_ENUM = ['CTM', 'LQ', 'PQ']
# node type in MAC-POSTS/src/minami/dnode.cpp: junction (FWJ), origin (DMOND), destination (DMDND)
DNODE_ENUM = ['FWJ', 'DMOND', 'DMDND']


class MNM_dlink():
    # Python Does Not Support Multiple Constructors, the last one overwrites the previous ones
    # def __init__(self):
    #   # link ID
    #   self.ID = None
    #   # length (unit: mile)
    #   self.length = None
    #   # link type: CTM, LQ, PQ
    #   self.typ = None
    #   # free flow speed for car (unit: mile/hour)
    #   self.ffs_car = None
    #   # capacity for car (unit: vehicles/hour/lane)
    #   self.cap_car = None
    #   # jam density for car (unit: vehicles/mile/lane)
    #   self.rhoj_car = None
    #   # number of lanes
    #   self.lanes = None
    #   # free flow speed for truck (unit: mile/hour)
    #   self.ffs_truck = None
    #   # capacity for truck (unit: vehicles/hour/lane)
    #   self.cap_truck = None
    #   # jam density for truck (unit: vehicles/mile/lane)
    #   self.rhoj_truck = None
    #   # factor for converting truck flow to equivalent car flow, only for calculating node demand for Inout, FWJ, and GRJ node types
    #   self.convert_factor = None

    def __init__(self, ID, length, typ, ffs_car, cap_car, rhoj_car, lanes,
                 ffs_truck, cap_truck, rhoj_truck, convert_factor):
        # link ID
        self.ID = ID
        # length (unit: mile)
        self.length = length  # mile
        # link type: CTM, LQ, PQ
        self.typ = typ  # type
        # free flow speed for car (unit: mile/hour)
        self.ffs_car = ffs_car  # mile/h
        # capacity for car (unit: vehicles/hour/lane)
        self.cap_car = cap_car  # v/hour/lane
        # jam density for car (unit: vehicles/mile/lane)
        self.rhoj_car = rhoj_car  # v/mile/lane
        # number of lanes
        self.lanes = lanes  # num of lanes
        # free flow speed for truck (unit: mile/hour)
        self.ffs_truck = ffs_truck  # mile/h
        # capacity for truck (unit: vehicles/hour/lane)
        self.cap_truck = cap_truck  # v/hour/lane
        # jam density for truck (unit: vehicles/mile/lane)
        self.rhoj_truck = rhoj_truck  # v/mile/lane
        # factor for converting truck flow to equivalent car flow, only for calculating node demand for Inout, FWJ, and GRJ node types
        self.convert_factor = convert_factor

        # increase length of links which are too short for CTM
        # if self.typ == 'CTM' and np.floor(self.length / self.ffs_car * 3600 / 5) <= 1:
        #     self.length = 2 * 5 / 3600 * self.ffs_car

    def get_car_fft(self):
        # get free flow time for car (unit: second)
        return self.length / self.ffs_car * 3600

    def get_truck_fft(self):
        # get free flow time for truck (unit: second)
        return self.length / self.ffs_truck * 3600

    def is_ok(self, unit_time=5):
        # unit_time: loading interval (unit: second)
        # print(self.ID, self.cap_truck, self.ffs_truck, self.rhoj_truck)
        assert(self.length > 0.0)
        assert(self.ffs_car > 0.0)
        assert(self.cap_car > 0.0)
        assert(self.rhoj_car > 0.0)
        assert(self.ffs_truck > 0.0)
        assert(self.cap_truck > 0.0)
        assert(self.rhoj_truck > 0.0)
        assert(self.lanes > 0)
        assert(self.convert_factor >= 1)
        assert(self.typ in DLINK_ENUM)
        assert(self.cap_car / self.ffs_car < self.rhoj_car)
        assert(self.cap_truck / self.ffs_truck < self.rhoj_truck)
        # if self.ffs < 9999:
        #   assert(unit_time * self.ffs / 3600 <= self.length)

    def __str__(self):
        return ("MNM_dlink, ID: {}, type: {}, length: {} miles, ffs_car: {} mph, ffs_truck: {} mph".format(
                self.ID, self.typ, self.length, self.ffs_car, self.ffs_truck))

    def __repr__(self):
        return self.__str__()

    def generate_text(self):
        return ' '.join([str(e) for e in [self.ID, self.typ,
                                          self.length, self.ffs_car, self.cap_car, self.rhoj_car, self.lanes,
                                          self.ffs_truck, self.cap_truck, self.rhoj_truck, self.convert_factor]])


class MNM_dnode():
    # Python Does Not Support Multiple Constructors, the last one overwrites the previous ones
    # def __init__(self):
    #   # node ID
    #   self.ID = None
    #   # node type: FWJ, DMOND, DMDND
    #   self.typ = None
    #   # factor for converting truck flow to equivalent car flow, only for calculating node demand for Inout, FWJ, and GRJ node types
    #   self.convert_factor = None

    def __init__(self, ID, typ, convert_factor):
        # node ID
        self.ID = ID
        # node type: FWJ, DMOND, DMDND
        self.typ = typ
        # factor for converting truck flow to equivalent car flow, only for calculating node demand for Inout, FWJ, and GRJ node types
        self.convert_factor = convert_factor

    def is_ok(self):
        assert(self.typ in DNODE_ENUM)

    def __str__(self):
        return "MNM_dnode, ID: {}, type: {}, convert_factor: {}".format(self.ID, self.typ, self.convert_factor)

    def __repr__(self):
        return self.__str__()

    def generate_text(self):
        return ' '.join([str(e) for e in [self.ID, self.typ, self.convert_factor]])


class MNM_demand():
    def __init__(self):
        self.demand_dict = dict()
        self.demand_list = None

    def add_demand(self, O, D, car_demand, truck_demand, overwriting=False):
        # check if car_demand is a 1D np.ndarray
        assert(type(car_demand) is np.ndarray)
        assert(len(car_demand.shape) == 1)
        # check if truck_demand is a 1D np.ndarray
        assert(type(truck_demand) is np.ndarray)
        assert(len(truck_demand.shape) == 1)

        if O not in self.demand_dict.keys():
            self.demand_dict[O] = dict()
        if (not overwriting) and (D in self.demand_dict[O].keys()):
            # check if repeated demand data for OD pair is added
            raise("Error, exists OD demand already")
        else:
            # demand of each OD pair is a list
            self.demand_dict[O][D] = [car_demand, truck_demand]

    def build_from_file(self, file_name):
        self.demand_list = list()
        # f = file(file_name)
        f = open(file_name, 'r')
        # MNM_input_demand: the first line is the title line, keep lines from second to end
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            # OD IDs are integers
            O_ID = np.int(words[0])
            D_ID = np.int(words[1])
            demand = np.array(words[2:]).astype(np.float)
            total_l = len(demand)
            # check the total demand array consists of two demand arrays (car and truck) with equal length
            assert (total_l % 2 == 0)
            # the first half is car_demand, the last half is truck demand
            self.add_demand(
                O_ID, D_ID, demand[0: total_l // 2], demand[- total_l // 2:])
            self.demand_list.append((O_ID, D_ID))

    def __str__(self):
        return "MNM_demand, number of O: {}, number of OD: {}".format(len(self.demand_dict), len(self.demand_list))

    def __repr__(self):
        return self.__str__()

    def generate_text(self):
        # generate OD demand in text
        tmp_str = '#Origin_ID Destination_ID <car demand by interval> <truck demand by interval>\n'
        for O in self.demand_dict.keys():
            for D in self.demand_dict[O].keys():
                tmp_str += ' '.join([str(e) for e in [O, D] +
                                     self.demand_dict[O][D][0].tolist() + self.demand_dict[O][D][1].tolist()]) + '\n'
        return tmp_str


class MNM_od():
    # the mapping between OD ID and the corresponding node ID
    def __init__(self):
        self.O_dict = bidict()
        self.D_dict = bidict()

    def add_origin(self, O, Onode_ID, overwriting=False):
        if (not overwriting) and (O in self.O_dict.keys()):
            raise("Error, exists origin node already")
        else:
            self.O_dict[O] = Onode_ID

    def add_destination(self, D, Dnode_ID, overwriting=False):
        if (not overwriting) and (D in self.D_dict.keys()):
            raise("Error, exists destination node already")
        else:
            self.D_dict[D] = Dnode_ID

    def build_from_file(self, file_name):
        self.O_dict = bidict()
        self.D_dict = bidict()
        flip = False
        # f = file(file_name)
        f = open(file_name, 'r')
        # MNM_input_od: the first line is the title line, keep lines from second to end
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            if tmp_str.startswith('#'):
                # the file includes two parts, the first half about origins, the last half about destinations
                # the title lines of both parts start with '#', when it is encountered, origin -> destination, destination -> origin
                flip = True
                continue
            words = tmp_str.split()
            # OD IDs are integers
            od = np.int(words[0])
            # node IDs are integers
            node = np.int(words[1])
            if not flip:
                # if line starting with '#' is not encountered
                self.add_origin(od, node)
            else:
                # if line starting with '#' is encountered
                self.add_destination(od, node)

    def generate_text(self):
        # generate OD node mapping in text
        tmp_str = '#Origin_ID <-> node_ID\n'
        # python 2
        # for O_ID, node_ID in self.O_dict.iteritems():
        # python 3
        for O_ID, node_ID in self.O_dict.items():
            tmp_str += ' '.join([str(e) for e in [O_ID, node_ID]]) + '\n'
        tmp_str += '#Dest_ID <-> node_ID\n'
        # python 2
        # for D_ID, node_ID in self.D_dict.iteritems():
        # python 3
        for D_ID, node_ID in self.D_dict.items():
            tmp_str += ' '.join([str(e) for e in [D_ID, node_ID]]) + '\n'
        return tmp_str

    def __str__(self):
        return "MNM_od, number of O:" + str(len(self.O_dict)) + ", number of D:" + str(len(self.D_dict))

    def __repr__(self):
        return self.__str__()


class MNM_graph():
    # directed graph
    def __init__(self):
        self.G = nx.DiGraph()
        self.edgeID_dict = OrderedDict()

    def add_edge(self, s, e, ID, create_node=False, overwriting=False):
        # s: starting node ID
        # e: ending node ID
        # ID: link ID
        if (not overwriting) and ((s, e) in self.G.edges()):
            raise("Error, exists edge in graph")
        elif (not create_node) and s in self.G.nodes():
            raise("Error, exists start node of edge in graph")
        elif (not create_node) and e in self.G.nodes():
            raise("Error, exists end node of edge in graph")
        else:
            self.G.add_edge(s, e, ID=ID)
            self.edgeID_dict[ID] = (s, e)

    def add_node(self, node, overwriting=True):
        if (not overwriting) and node in self.G.nodes():
            raise("Error, exists node in graph")
        else:
            self.G.add_node(node)

    def build_from_file(self, file_name):
        self.G = nx.DiGraph()
        self.edgeID_dict = OrderedDict()
        # f = file(file_name)
        f = open(file_name, 'r')
        # Snap_graph: the first line is the title line, keep lines from second to end
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            assert(len(words) == 3)
            # link IDs are integers
            edge_id = np.int(words[0])
            # starting node IDs are integers
            from_id = np.int(words[1])
            # ending node IDs are integers
            to_id = np.int(words[2])
            self.add_edge(from_id, to_id, edge_id,
                          create_node=True, overwriting=False)

    def __str__(self):
        return "MNM_graph, number of node:" + str(self.G.number_of_nodes()) + ", number of edges:" + str(self.G.number_of_edges())

    def __repr__(self):
        return self.__str__()

    def generate_text(self):
        # generate graph topology info in text
        tmp_str = '# EdgeId FromNodeId  ToNodeId\n'
        # python 2
        # for edge_id, (from_id, to_id) in self.edgeID_dict.iteritems():
        # python 3
        for edge_id, (from_id, to_id) in self.edgeID_dict.items():
            tmp_str += ' '.join([str(e)
                                for e in [edge_id, from_id, to_id]]) + '\n'
        return tmp_str


class MNM_path():
    # Python Does Not Support Multiple Constructors, the last one overwrites the previous ones
    # class of one path: a sequence of node IDs
    # def __init__(self):
    #   # print("MNM_path")
    #   self.path_ID = None
    #   self.origin_node = None
    #   self.destination_node = None
    #   self.node_list = list()
    #   # car
    #   self.route_portions = None
    #   # truck
    #   self.truck_route_portions = None

    def __init__(self, node_list, path_ID):
        # print("MNM_path")
        self.path_ID = path_ID
        self.origin_node = node_list[0]
        self.destination_node = node_list[-1]
        self.node_list = node_list
        # car
        self.route_portions = None
        # truck
        self.truck_route_portions = None

        # car
        self.route_portions_tensor = None
        # truck
        self.truck_route_portions_tensor = None

        self.path_cost_car = None
        self.path_cost_truck = None

        self.length = None

    def __eq__(self, other):
        if not isinstance(other, MNM_path):
            return False
        # determine if two paths are equal
        if ((self.origin_node is None) or (self.destination_node is None) or
                (other.origin_node is None) or (other.destination_node is None)):
            return False
        if (other.origin_node != self.origin_node):
            return False
        if (other.destination_node != self.destination_node):
            return False
        if (len(self.node_list) != len(other.node_list)):
            return False
        for i in range(len(self.node_list)):
            if self.node_list[i] != other.node_list[i]:
                return False
        return True

    def create_route_choice_portions(self, num_intervals):
        # for car: portion of cars using this path to the total car demand for this OD pair
        self.route_portions = np.zeros(num_intervals)
        # for truck: portion of trucks using this path to the total truck demand for this OD pair
        self.truck_route_portions = np.zeros(num_intervals)

    def attach_route_choice_portions(self, portions):
        # for car: portion of cars using this path to the total car demand for this OD pair
        self.route_portions = portions

    def attach_route_choice_portions_truck(self, portions):
        # for truck: portion of trucks using this path to the total truck demand for this OD pair
        self.truck_route_portions = portions

    def attach_route_choice_portions_tensor(self, portions):
        # for car: portion of cars using this path to the total car demand for this OD pair
        self.route_portions_tensor = portions

    def attach_route_choice_portions_truck_tensor(self, portions):
        # for truck: portion of trucks using this path to the total truck demand for this OD pair
        self.truck_route_portions_tensor = portions

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return "MNM_path, path ID {}, O node: {}, D node {}".format(self.path_ID, self.origin_node, self.destination_node)

    def __repr__(self):
        return self.__str__()

    def generate_node_list_text(self):
        return ' '.join([str(e) for e in self.node_list])

    def generate_portion_text(self):
        assert(self.route_portions is not None)
        assert(self.truck_route_portions is not None)
        return ' '.join([str(e) for e in np.append(self.route_portions, self.truck_route_portions)])


class MNM_pathset():
    # path set for one OD pair: a list of paths
    def __init__(self):
        self.origin_node = None
        self.destination_node = None
        self.path_list = list()

    def add_path(self, path, overwriting=False):
        assert(path.origin_node == self.origin_node)
        assert(path.destination_node == self.destination_node)
        if (not overwriting) and (path in self.path_list):
            raise ("Error, path in path set")
        else:
            self.path_list.append(path)

    def normalize_route_portions(self, sum_to_OD=False):
        # for car: normalizing the time-varing portion of cars using each path within [0, 1]
        for i in range(len(self.path_list) - 1):
            assert(self.path_list[i].route_portions.shape ==
                   self.path_list[i + 1].route_portions.shape)
        tmp_sum = np.zeros(self.path_list[0].route_portions.shape)
        for i in range(len(self.path_list)):
            tmp_sum += self.path_list[i].route_portions
        for i in range(len(self.path_list)):
            self.path_list[i].route_portions = self.path_list[i].route_portions / \
                np.maximum(tmp_sum, 1e-6)
        if sum_to_OD:
            return tmp_sum

    def normalize_truck_route_portions(self, sum_to_OD=False):
        # for truck: normalizing the time-varying portion of trucks using each path within [0, 1]
        for i in range(len(self.path_list) - 1):
            assert(self.path_list[i].truck_route_portions.shape ==
                   self.path_list[i + 1].truck_route_portions.shape)
        tmp_sum = np.zeros(self.path_list[0].truck_route_portions.shape)
        for i in range(len(self.path_list)):
            tmp_sum += self.path_list[i].truck_route_portions
        for i in range(len(self.path_list)):
            self.path_list[i].truck_route_portions = self.path_list[i].truck_route_portions / \
                np.maximum(tmp_sum, 1e-6)
        if sum_to_OD:
            return tmp_sum

    def __str__(self):
        return "MNM_pathset, O node: {}, D node: {}, number_of_paths: {}".format(self.origin_node, self.destination_node, len(self.path_list))

    def __repr__(self):
        return self.__str__()


class MNM_pathtable():
    # paths for all OD pairs: dictionary with value being class MNM_pathset()
    def __init__(self):
        # print("MNM_pathtable")
        self.path_dict = dict()
        self.ID2path = OrderedDict()

    def add_pathset(self, pathset, overwriting=False):
        if pathset.origin_node not in self.path_dict.keys():
            self.path_dict[pathset.origin_node] = dict()
        if (not overwriting) and (pathset.destination_node in self.path_dict[pathset.origin_node]):
            raise ("Error, pathset exists in the pathtable")
        else:
            self.path_dict[pathset.origin_node][pathset.destination_node] = pathset

    def build_from_file(self, file_name, w_ID=False, starting_ID=0):
        if w_ID:
            raise ("Error, path table build_from_file with ID not implemented")
        self.path_dict = dict()
        self.ID2path = OrderedDict()
        # path_table: no title line
        # f = file(file_name)
        f = open(file_name, 'r')
        log = f.readlines()
        f.close()
        for i in range(len(log)):
            tmp_str = log[i]
            if tmp_str == '':
                continue
            words = tmp_str.split()
            # OD node IDs are integers
            origin_node = np.int(words[0])
            destination_node = np.int(words[-1])
            if origin_node not in self.path_dict.keys():
                self.path_dict[origin_node] = dict()
            if destination_node not in self.path_dict[origin_node].keys():
                tmp_path_set = MNM_pathset()
                tmp_path_set.origin_node = origin_node
                tmp_path_set.destination_node = destination_node
                self.add_pathset(tmp_path_set)
            # node IDs are integers
            tmp_node_list = list(map(lambda x: np.int(x), words))
            tmp_path = MNM_path(tmp_node_list, starting_ID + i)
            self.path_dict[origin_node][destination_node].add_path(tmp_path)
            self.ID2path[starting_ID + i] = tmp_path

    def load_route_choice_from_file(self, file_name, w_ID=False, buffer_length=None, starting_ID=0):
        if w_ID:
            raise ("Error, pathtable load_route_choice_from_file not implemented")
        # path_table_buffer: each entry is a sequence of time-dependent portions using each corresponding path, the first half for car, the last half for truck
        # buffer_length = number of total intervals
        # f = file(file_name)
        f = open(file_name, 'r')
        log = list(filter(lambda x: not x.strip() == '', f.readlines()))
        f.close()
        assert(len(log) == len(self.ID2path))
        for i in range(len(log)):
            tmp_portions = np.array(log[i].strip().split()).astype(np.float)
            if buffer_length is not None:
                # if 2*len(tmp_portions) == buffer_length:
                #   tmp_portions = np.concatenate((tmp_portions, tmp_portions))
                if len(tmp_portions) == buffer_length:
                    buffer_length = buffer_length // 2
                assert(len(tmp_portions) == buffer_length * 2)
                self.ID2path[starting_ID +
                             i].attach_route_choice_portions(tmp_portions[:buffer_length])
                self.ID2path[starting_ID +
                             i].attach_route_choice_portions_truck(tmp_portions[buffer_length:])
            else:
                raise("deprecated")

    def __str__(self):
        return "MNM_pathtable, number of paths: " + str(len(self.ID2path))

    def __repr__(self):
        return self.__str__()

    def generate_table_text(self):
        tmp_str = ""
        # python 2
        # for path_ID, path in self.ID2path.iteritems():
        # python 3
        for path_ID, path in self.ID2path.items():
            tmp_str += path.generate_node_list_text() + '\n'
        return tmp_str

    def generate_portion_text(self):
        tmp_str = ""
        # python 2
        # for path_ID, path in self.ID2path.iteritems():
        # python 3
        for path_ID, path in self.ID2path.items():
            tmp_str += path.generate_portion_text() + '\n'
        return tmp_str

# class MNM_routing():
#   def __init__(self):
#     print "MNM_routing"

# class MNM_routing_fixed(MNM_routing):
#   def __init__(self):
#     super(MNM_routing_fixed, self).__init__()
#     self.required_items = ['num_path', 'choice_portion', 'route_frq', 'path_table']

# class MNM_routing_adaptive(MNM_routing):
#   """docstring for MNM_routing_adaptive"""
#   def __init__(self):
#     super(MNM_routing_adaptive, self).__init__()
#     self.required_items = ['route_frq']

# class MNM_routing_hybrid(MNM_routing):
#   """docstring for MNM_routing_hybrid"""
#   def __init__(self):
#     super(MNM_routing_hybrid, self).__init__()
#     self.required_items = []


class MNM_config():
    def __init__(self):
        # print("MNM_config")
        self.config_dict = OrderedDict()
        self.type_dict = {
            # DTA
            'network_name': str, 
            'unit_time': np.int, 
            'total_interval': np.int,
            'assign_frq': np.int, 
            'start_assign_interval': np.int, 
            'max_interval': np.int,
            'flow_scalar': np.int, 

            'num_of_link': np.int, 
            'num_of_node': np.int,
            'num_of_O': np.int, 
            'num_of_D': np.int, 
            'OD_pair': np.int,

            'adaptive_ratio': np.float,
            'adaptive_ratio_car': np.float,
            'adaptive_ratio_truck': np.float,
            'routing_type': str, 
            'init_demand_split': np.int, 
            'num_of_car_labels': np.int,
            'num_of_truck_labels': np.int,
            'num_of_tolled_link': np.int,

            # STAT
            'rec_mode': str, 
            'rec_mode_para': str, 
            'rec_folder': str,
            'rec_volume': np.int, 
            'volume_load_automatic_rec': np.int, 
            'volume_record_automatic_rec': np.int,
            'rec_tt': np.int, 
            'tt_load_automatic_rec': np.int, 
            'tt_record_automatic_rec': np.int,
            'rec_gridlock': np.int,

            # FIXED, ADAPTIVE
            'route_frq': np.int, 
            'vot': np.float,
            'path_file_name': str, 
            'num_path': np.int,
            'choice_portion': str, 
            'buffer_length': np.int
        
        }

    def build_from_file(self, file_name):
        self.config_dict = OrderedDict()
        # config.conf
        # f = file(file_name)
        f = open(file_name, 'r')
        log = f.readlines()
        f.close()
        for i in range(len(log)):
            tmp_str = log[i]
            # print tmp_str
            if tmp_str == '' or tmp_str.startswith('#') or tmp_str.strip() == '':
                continue
            if tmp_str.startswith('['):
                # different parts in config.conf are separated using [text]
                new_item = tmp_str.strip().strip('[]')
                if new_item in self.config_dict.keys():
                    raise ("Error, MNM_config, multiple items", new_item)
                self.config_dict[new_item] = OrderedDict()
                continue
            words = tmp_str.split('=')
            name = words[0].strip()
            if name not in self.type_dict:
                raise ("Error, MNM_config, setting not in type_dict", name)
            self.config_dict[new_item][name] = self.type_dict[name](words[1].strip())

    def __str__(self):
        tmp_str = ''
        tmp_str += '[DTA]\n'
        # python 2
        # for name, value in self.config_dict['DTA'].iteritems():
        # python 3
        for name, value in self.config_dict['DTA'].items():
            tmp_str += "{} = {}\n".format(str(name), str(value))
        tmp_str += '\n[STAT]\n'
        # python 2
        # for name, value in self.config_dict['STAT'].iteritems():
        # python 3
        for name, value in self.config_dict['STAT'].items():
            tmp_str += "{} = {}\n".format(str(name), str(value))
        if (self.config_dict['DTA']['routing_type'] == 'Fixed'
            or self.config_dict['DTA']['routing_type'] == 'Hybrid'
                or self.config_dict['DTA']['routing_type'] == 'Biclass_Hybrid'):
            tmp_str += '\n[FIXED]\n'
            # python 2
            # for name, value in self.config_dict['FIXED'].iteritems():
            # python 3
            for name, value in self.config_dict['FIXED'].items():
                tmp_str += "{} = {}\n".format(str(name), str(value))
        if (self.config_dict['DTA']['routing_type'] == 'Adaptive'
            or self.config_dict['DTA']['routing_type'] == 'Hybrid'
                or self.config_dict['DTA']['routing_type'] == 'Biclass_Hybrid'):
            tmp_str += '\n[ADAPTIVE]\n'
            # python 2
            # for name, value in self.config_dict['ADAPTIVE'].iteritems():
            # python 3
            for name, value in self.config_dict['ADAPTIVE'].items():
                tmp_str += "{} = {}\n".format(str(name), str(value))
        return tmp_str

    def __repr__(self):
        return self.__str__()


class MNM_network_builder():
    # build network for DTA considering bi-class flows: car and truck
    def __init__(self):
        self.folder_path = None
        self.config = MNM_config()
        self.network_name = None
        self.link_list = list()
        self.node_list = list()
        self.graph = MNM_graph()
        self.od = MNM_od()
        self.demand = MNM_demand()
        self.path_table = MNM_pathtable()
        self.route_choice_flag = False
        self.emission_link_array = None

    def get_link(self, ID):
        for link in self.link_list:
            if link.ID == ID:
                return link
        return None

    def load_from_folder(self, path, config_file_name='config.conf',
                         link_file_name='MNM_input_link', node_file_name='MNM_input_node',
                         graph_file_name='Snap_graph', od_file_name='MNM_input_od',
                         pathtable_file_name='path_table', path_p_file_name='path_table_buffer',
                         demand_file_name='MNM_input_demand', emission_link_file_name='MNM_input_emission_linkID'):

        self.folder_path = path

        if os.path.isfile(os.path.join(path, config_file_name)):
            self.config.build_from_file(os.path.join(path, config_file_name))
            self.config.config_dict['FIXED']['buffer_length'] = 2 * \
                self.config.config_dict['DTA']['max_interval']
        else:
            print("No config file")

        if os.path.isfile(os.path.join(path, link_file_name)):
            self.link_list = self.read_link_input(
                os.path.join(path, link_file_name))
        else:
            print("No link input")

        if os.path.isfile(os.path.join(path, node_file_name)):
            self.node_list = self.read_node_input(
                os.path.join(path, node_file_name))
        else:
            print("No node input")

        if os.path.isfile(os.path.join(path, graph_file_name)):
            self.graph.build_from_file(os.path.join(path, graph_file_name))
        else:
            print("No graph input")

        if os.path.isfile(os.path.join(path, od_file_name)):
            self.od.build_from_file(os.path.join(path, od_file_name))
        else:
            print("No OD input")

        if os.path.isfile(os.path.join(path, demand_file_name)):
            self.demand.build_from_file(os.path.join(path, demand_file_name))
        else:
            print("No demand input")
            # pass

        if os.path.isfile(os.path.join(path, pathtable_file_name)):
            self.path_table.build_from_file(
                os.path.join(path, pathtable_file_name))
            if os.path.isfile(os.path.join(path, path_p_file_name)):
                self.path_table.load_route_choice_from_file(os.path.join(path, path_p_file_name),
                                                            buffer_length=self.config.config_dict['FIXED']['buffer_length'])
                self.route_choice_flag = True
            else:
                self.route_choice_flag = False
                # print "No route choice portition for path table"

        else:
            print("No path table input")

        if os.path.isfile(os.path.join(path, emission_link_file_name)):
            self.emission_link_array = np.loadtxt(
                os.path.join(path, emission_link_file_name), dtype=int)
            if(len(self.emission_link_array.shape) == 1):
                self.emission_link_array = self.emission_link_array[:, np.newaxis]
        else:
            print("No emission link file")

    def dump_to_folder(self, path, config_file_name='config.conf',
                       link_file_name='MNM_input_link', node_file_name='MNM_input_node',
                       graph_file_name='Snap_graph', od_file_name='MNM_input_od',
                       pathtable_file_name='path_table', path_p_file_name='path_table_buffer',
                       demand_file_name='MNM_input_demand', emission_link_file_name='MNM_input_emission_linkID'):
        if not os.path.isdir(path):
            os.makedirs(path)

        if not os.path.exists(os.path.join(path, self.config.config_dict['STAT']['rec_folder'])):
            os.mkdir(os.path.join(
                path, self.config.config_dict['STAT']['rec_folder']))

        # python 2
        # f = open(os.path.join(path, link_file_name), 'wb')
        # python 3
        f = open(os.path.join(path, link_file_name), 'w')
        f.write(self.generate_link_text())
        f.close()

        # python 2
        # f = open(os.path.join(path, node_file_name), 'wb')
        # python 3
        f = open(os.path.join(path, node_file_name), 'w')
        f.write(self.generate_node_text())
        f.close()

        # python 2
        # f = open(os.path.join(path, config_file_name), 'wb')
        # python 3
        f = open(os.path.join(path, config_file_name), 'w')
        f.write(str(self.config))
        f.close()

        # python 2
        # f = open(os.path.join(path, od_file_name), 'wb')
        # python 3
        f = open(os.path.join(path, od_file_name), 'w')
        f.write(self.od.generate_text())
        f.close()

        # python 2
        # f = open(os.path.join(path, demand_file_name), 'wb')
        # python 3
        f = open(os.path.join(path, demand_file_name), 'w')
        f.write(self.demand.generate_text())
        f.close()

        # python 2
        # f = open(os.path.join(path, graph_file_name), 'wb')
        # python 3
        f = open(os.path.join(path, graph_file_name), 'w')
        f.write(self.graph.generate_text())
        f.close()

        # python 2
        # f = open(os.path.join(path, pathtable_file_name), 'wb')
        # python 3
        f = open(os.path.join(path, pathtable_file_name), 'w')
        f.write(self.path_table.generate_table_text())
        f.close()

        if self.route_choice_flag:
            # python 2
            # f = open(os.path.join(path, path_p_file_name), 'wb')
            # python 3
            f = open(os.path.join(path, path_p_file_name), 'w')
            f.write(self.path_table.generate_portion_text())
            f.close()

        if self.emission_link_array is not None:
            np.savetxt(os.path.join(path, emission_link_file_name),
                       self.emission_link_array, fmt='%d')
        else:
            print("No emission link file")

    def read_link_input(self, file_name):
        link_list = list()
        # f = file(file_name)
        f = open(file_name, 'r')
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            # column order in MNM_input_link
            ID = np.int(words[0])
            typ = words[1]
            length = np.float(words[2])
            ffs_car = np.float(words[3])
            cap_car = np.float(words[4])
            rhoj_car = np.float(words[5])
            lanes = np.int(words[6])
            ffs_truck = np.float(words[7])
            cap_truck = np.float(words[8])
            rhoj_truck = np.float(words[9])
            convert_factor = np.float(words[10])
            l = MNM_dlink(ID, length, typ, ffs_car, cap_car, rhoj_car,
                          lanes, ffs_truck, cap_truck, rhoj_truck, convert_factor)
            l.is_ok()
            link_list.append(l)
        return link_list

    def read_node_input(self, file_name):
        node_list = list()
        # f = file(file_name)
        f = open(file_name, 'r')
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            # column order in MNM_input_node
            ID = np.int(words[0])
            typ = words[1]
            convert_factor = np.float(words[2])
            n = MNM_dnode(ID, typ, convert_factor)
            n.is_ok()
            node_list.append(n)
        return node_list

    def generate_link_text(self):
        tmp_str = '#ID Type LEN(mile) FFS_car(mile/h) Cap_car(v/hour) RHOJ_car(v/miles) Lane FFS_truck(mile/h) Cap_truck(v/hour) RHOJ_truck(v/miles) Convert_factor(1)\n'
        for link in self.link_list:
            tmp_str += link.generate_text() + '\n'
        return tmp_str

    def generate_node_text(self):
        tmp_str = '#ID Type Convert_factor(only for Inout node)\n'
        for node in self.node_list:
            tmp_str += node.generate_text() + '\n'
        return tmp_str

    def set_network_name(self, name):
        self.network_name = name

    def update_demand_path(self, car_flow, truck_flow, choice_dict):
        # for MMDUE, car and bus
        # an OD pair is either for car or for truck, but cannot be both. So add OD connectors separately for car or truck
        assert (len(car_flow) == len(self.path_table.ID2path) *
                self.config.config_dict['DTA']['max_interval'])
        assert (len(truck_flow) == len(self.path_table.ID2path)
                * self.config.config_dict['DTA']['max_interval'])
        max_interval = self.config.config_dict['DTA']['max_interval']
        car_flow = car_flow.reshape(
            self.config.config_dict['DTA']['max_interval'], len(self.path_table.ID2path))
        truck_flow = truck_flow.reshape(
            self.config.config_dict['DTA']['max_interval'], len(self.path_table.ID2path))
        for i, path_ID in enumerate(self.path_table.ID2path.keys()):
            path = self.path_table.ID2path[path_ID]
            path.attach_route_choice_portions(
                car_flow[:, i] + truck_flow[:, i])
        self.demand.demand_dict = dict()
        for O_node in self.path_table.path_dict.keys():
            O = self.od.O_dict.inv[O_node]
            for D_node in self.path_table.path_dict[O_node].keys():
                D = self.od.D_dict.inv[D_node]
                demand = self.path_table.path_dict[O_node][D_node].normalize_route_portions(
                    sum_to_OD=True)
                if choice_dict[O][D] == 0:  # for car only
                    self.demand.add_demand(O, D, demand, np.zeros(
                        max_interval), overwriting=True)
                elif choice_dict[O][D] == 1:  # for truck only
                    self.demand.add_demand(O, D, np.zeros(
                        max_interval), demand, overwriting=True)
                else:
                    raise("Error, wrong truck flow or car flow in update_demand_path")
        self.route_choice_flag = True

    def update_demand_path2(self, car_flow, truck_flow):
        # car_flow and truck_flow are 1D ndarrays recording the time-dependent flows with length of number of total paths x intervals
        assert (len(car_flow) == len(self.path_table.ID2path) *
                self.config.config_dict['DTA']['max_interval'])
        assert (len(truck_flow) == len(self.path_table.ID2path)
                * self.config.config_dict['DTA']['max_interval'])
        max_interval = self.config.config_dict['DTA']['max_interval']
        # reshape car_flow and truck_flow into ndarrays with dimensions of intervals x number of total paths
        car_flow = car_flow.reshape(
            self.config.config_dict['DTA']['max_interval'], len(self.path_table.ID2path))
        truck_flow = truck_flow.reshape(
            self.config.config_dict['DTA']['max_interval'], len(self.path_table.ID2path))
        # update time-varying portions
        for i, path_ID in enumerate(self.path_table.ID2path.keys()):
            path = self.path_table.ID2path[path_ID]
            path.attach_route_choice_portions(car_flow[:, i])
            path.attach_route_choice_portions_truck(truck_flow[:, i])
        # update time-varying OD demands
        self.demand.demand_dict = dict()
        for O_node in self.path_table.path_dict.keys():
            O = self.od.O_dict.inv[O_node]
            for D_node in self.path_table.path_dict[O_node].keys():
                D = self.od.D_dict.inv[D_node]
                demand = self.path_table.path_dict[O_node][D_node].normalize_route_portions(
                    sum_to_OD=True)
                truck_demand = self.path_table.path_dict[O_node][D_node].normalize_truck_route_portions(
                    sum_to_OD=True)
                self.demand.add_demand(
                    O, D, demand, truck_demand, overwriting=True)
        self.route_choice_flag = True

    # def get_path_flow(self):
    #   f = np.zeros((self.config.config_dict['DTA']['max_interval'], len(self.path_table.ID2path)))
    #   f_truck = np.zeros((self.config.config_dict['DTA']['max_interval'], len(self.path_table.ID2path)))
    #   for i, ID in enumerate(self.path_table.ID2path.keys()):
    #     path = self.path_table.ID2path[ID]
    #     # print path.route_portions
    #     f[:, i] = path.route_portions * self.demand.demand_dict[self.od.O_dict.inv[path.origin_node]][self.od.D_dict.inv[path.destination_node]][0]
    #     f_truck[:, i] = path.truck_route_portions  * self.demand.demand_dict[self.od.O_dict.inv[path.origin_node]][self.od.D_dict.inv[path.destination_node]][1]
    #   return f.flatten(), f_truck.flatten()

    def update_path_table(self, dta, start_intervals):
        # start_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)

        dta.update_tdsp_tree()  # dta.build_link_cost_map() should be called before this method

        _reorder_flg = False
        _path_result_rec_dict = dict()
        for interval in start_intervals:

            for (O_ID, D_ID) in self.demand.demand_list:
                O_node_ID = self.od.O_dict[O_ID]
                D_node_ID = self.od.D_dict[D_ID]
                if (O_ID, D_ID) not in _path_result_rec_dict:
                    _path_result_rec_dict[(O_ID, D_ID)] = list()
                _path_result_rec = _path_result_rec_dict[(O_ID, D_ID)]

                _path_result = dta.get_lowest_cost_path(interval, O_node_ID, D_node_ID)
                
                assert(_path_result.shape[0] == 4)
                _exist = _path_result[0, 0]
                if not _exist:
                    _reorder_flg = True
                    _add_flag = True
                    if len(_path_result_rec) > 0:
                        for _path_result_old in _path_result_rec:
                            if len(_path_result_old.flatten()) == len(_path_result.flatten()):
                                if np.sum(np.abs(_path_result_old - _path_result)) == 0:
                                    _add_flag = False
                                    break
                    if _add_flag:
                        _path_result_rec.append(_path_result)
                assert(len(_path_result_rec_dict[(O_ID, D_ID)]) == len(_path_result_rec))
        
        if _reorder_flg:
            for (O_ID, D_ID) in self.demand.demand_list:
                O_node_ID = self.od.O_dict[O_ID]
                D_node_ID = self.od.D_dict[D_ID]
                _path_result_rec = _path_result_rec_dict[(O_ID, D_ID)]

                if len(_path_result_rec) > 0:
                    for _path_result in _path_result_rec:
                        _node_vec_driving = _path_result[2, :]
                        assert(np.min(_node_vec_driving) >= 0)
                        _link_vec_driving = _path_result[3, :-1]
                        path_tt_cost_car = dta.get_path_tt_car(_link_vec_driving, start_intervals)
                        path_tt_cost_truck = dta.get_path_tt_truck(_link_vec_driving, start_intervals)
                        assert(path_tt_cost_car.shape[0] == 2 and path_tt_cost_car.shape[1] == len(start_intervals) and path_tt_cost_truck.shape[0] == 2 and path_tt_cost_truck.shape[1] == len(start_intervals))

                        id_max = np.max(np.array([i for i in self.path_table.ID2path.keys()], dtype=int))
                        tmp_path = MNM_path(_node_vec_driving, id_max+1)
                        tmp_path.create_route_choice_portions(len(start_intervals))
                        tmp_path.path_cost_car = path_tt_cost_car[1, :]
                        tmp_path.path_cost_truck = path_tt_cost_truck[1, :]
                        self.path_table.path_dict[O_node_ID][D_node_ID].add_path(tmp_path)
                        self.path_table.ID2path[id_max+1] = tmp_path
                        

            # reorder
            self.config.config_dict['FIXED']['num_path'] = len(self.path_table.ID2path)

            # driving
            path_count = 0
            new_ID2path = OrderedDict()
            for k, v in enumerate(self.path_table.ID2path.values()):
                new_ID2path[k + path_count] = v
                v.path_ID = k + path_count
            self.path_table.ID2path = new_ID2path

            print("update_path_table")