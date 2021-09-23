import os
import numpy as np
import networkx as nx
from bidict import bidict
from collections import OrderedDict

from MNM_mcnb import MNM_dlink, MNM_dnode, MNM_graph, MNM_path, MNM_pathset, MNM_pathtable, MNM_demand

WALKING_NODE_ENUM = ['bus_stop_physical', 'bus_stop_virtual', 'origin', 'destination', 'parking_lot']
WALKING_LINK_ENUM = ['normal', 'boarding', 'alighting']


class MNM_demand_driving():
    def __init__(self):
        self.demand_dict = dict()

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
            raise("Error, OD driving demand exists already")
        else:
            # demand of each OD pair is a list
            self.demand_dict[O][D] = [car_demand, truck_demand]

    def build_from_file(self, file_name):
        f = file(file_name)
        # driving_demand: the first line is the title line, keep lines from second to end
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
            self.add_demand(O_ID, D_ID, demand[0: total_l//2], demand[- total_l//2:])

    def __str__(self):
        return "MNM_demand_driving, number of O: {}".format(len(self.demand_dict))

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


class MNM_demand_bus():
    def __init__(self):
        self.demand_dict = dict()

    def add_demand(self, O, D, route_ID, bus_demand, overwriting=False):
        # check if bus_demand is a 1D np.ndarray
        assert(type(bus_demand) is np.ndarray)
        assert(len(bus_demand.shape) == 1)

        if O not in self.demand_dict.keys():
            self.demand_dict[O] = dict()
        if (not overwriting) and (D in self.demand_dict[O].keys()) and (route_ID in self.demand_dict[O][D].keys()):
            # check if repeated demand data for OD pair is added
            raise("Error, OD driving demand exists already")
        else:
            if D not in self.demand_dict[O].keys():
                self.demand_dict[O][D] = dict()
            # demand of each OD pair is a np.array
            self.demand_dict[O][D][route_ID] = bus_demand

    def build_from_file(self, file_name):
        f = file(file_name)
        # bus_demand: the first line is the title line, keep lines from second to end
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            # OD IDs and route ID are integers
            O_ID = np.int(words[0])
            D_ID = np.int(words[1])
            route_ID = np.int(words[2])
            demand = np.array(words[3:]).astype(np.float)
            total_l = len(demand)
            self.add_demand(O_ID, D_ID, route_ID, demand)

    def __str__(self):
        return "MNM_demand_bus, number of O: {}".format(len(self.demand_dict))

    def __repr__(self):
        return self.__str__()

    def generate_text(self):
        # generate OD demand in text
        tmp_str = '#OriginID DestID RouteID <bus demand by interval>\n'
        for O in self.demand_dict.keys():
            for D in self.demand_dict[O].keys():
                for route_ID in self.demand_dict[O][D].keys():
                    tmp_str += ' '.join([str(e) for e in [O, D, route_ID] +
                                        self.demand_dict[O][D][route_ID].tolist()]) + '\n'
        return tmp_str


class MNM_demand_bustransit():
    def __init__(self):
        self.demand_dict = dict()

    def add_demand(self, O, D, bustransit_demand, overwriting=False):
        # check if bustransit_demand is a 1D np.ndarray
        assert(type(bustransit_demand) is np.ndarray)
        assert(len(bustransit_demand.shape) == 1)

        if O not in self.demand_dict.keys():
            self.demand_dict[O] = dict()
        if (not overwriting) and (D in self.demand_dict[O].keys()):
            # check if repeated demand data for OD pair is added
            raise("Error, OD bustransit demand exists already")
        else:
            # demand of each OD pair is a np.array
            self.demand_dict[O][D] = bustransit_demand

    def build_from_file(self, file_name):
        f = file(file_name)
        # bustransit_demand: the first line is the title line, keep lines from second to end
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
            self.add_demand(O_ID, D_ID, demand)

    def __str__(self):
        return "MNM_demand_bustransit, number of O: {}".format(len(self.demand_dict))

    def __repr__(self):
        return self.__str__()

    def generate_text(self):
        # generate OD demand in text
        tmp_str = '#Origin_ID Destination_ID <passenger demand by interval>\n'
        for O in self.demand_dict.keys():
            for D in self.demand_dict[O].keys():
                tmp_str += ' '.join([str(e) for e in [O, D] +
                                    self.demand_dict[O][D].tolist()]) + '\n'
        return tmp_str


class MNM_demand_pnr():
    def __init__(self):
        self.demand_dict = dict()

    def add_demand(self, O, D, pnr_demand, overwriting=False):
        # check if pnr_demand is a 1D np.ndarray
        assert(type(pnr_demand) is np.ndarray)
        assert(len(pnr_demand.shape) == 1)

        if O not in self.demand_dict.keys():
            self.demand_dict[O] = dict()
        if (not overwriting) and (D in self.demand_dict[O].keys()):
            # check if repeated demand data for OD pair is added
            raise("Error, OD pnr demand exists already")
        else:
            # demand of each OD pair is a np.array
            self.demand_dict[O][D] = pnr_demand

    def build_from_file(self, file_name):
        f = file(file_name)
        # pnr_demand: the first line is the title line, keep lines from second to end
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
            self.add_demand(O_ID, D_ID, demand)

    def __str__(self):
        return "MNM_demand_pnr, number of O: {}".format(len(self.demand_dict))

    def __repr__(self):
        return self.__str__()

    def generate_text(self):
        # generate OD demand in text
        tmp_str = '#OriginID DestID <passenger demand by interval>\n'
        for O in self.demand_dict.keys():
            for D in self.demand_dict[O].keys():
                tmp_str += ' '.join([str(e) for e in [O, D] +
                                    self.demand_dict[O][D].tolist()]) + '\n'
        return tmp_str


class MNM_demand_total_passenger():
    def __init__(self):
        self.demand_dict = dict()

    def add_demand(self, O, D, total_passenger_demand, overwriting=False):
        # check if total_passenger_demand is a 1D np.ndarray
        assert(type(total_passenger_demand) is np.ndarray)
        assert(len(total_passenger_demand.shape) == 1)

        if O not in self.demand_dict.keys():
            self.demand_dict[O] = dict()
        if (not overwriting) and (D in self.demand_dict[O].keys()):
            # check if repeated demand data for OD pair is added
            raise("Error, OD total passenger demand exists already")
        else:
            # demand of each OD pair is a np.array
            self.demand_dict[O][D] = total_passenger_demand

    def build_from_file(self, file_name):
        f = file(file_name)
        # passenger_demand: the first line is the title line, keep lines from second to end
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
            self.add_demand(O_ID, D_ID, demand)

    def __str__(self):
        return "MNM_demand_total_passenger, number of O: {}".format(len(self.demand_dict))

    def __repr__(self):
        return self.__str__()

    def generate_text(self):
        # generate OD demand in text
        tmp_str = '#OriginID DestID <passenger demand by interval>\n'
        for O in self.demand_dict.keys():
            for D in self.demand_dict[O].keys():
                tmp_str += ' '.join([str(e) for e in [O, D] +
                                    self.demand_dict[O][D].tolist()]) + '\n'
        return tmp_str


class MNM_od():
    # the mapping between OD ID and the corresponding node ID
    def __init__(self):
        self.O_dict = bidict()
        self.O_data_dict = dict()
        self.D_dict = bidict()

    def add_origin(self, O, Onode_ID, pickup_waiting_time, overwriting=False):
        if (not overwriting) and ((O in self.O_dict.keys()) or (O in self.O_data_dict.keys())):
            raise("Error, origin node exists already")
        else:
            self.O_dict[O] = Onode_ID
            self.O_data_dict[O] = pickup_waiting_time

    def add_destination(self, D, Dnode_ID, overwriting=False):
        if (not overwriting) and (D in self.D_dict.keys()):
            raise("Error, destination node exists already")
        else:
            self.D_dict[D] = Dnode_ID

    def build_from_file(self, file_name):
        self.O_dict = bidict()
        self.O_data_dict = dict()
        self.D_dict = bidict()
        flip = False
        f = file(file_name)
        # od: the first line is the origin node line, keep lines from second to end
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
                pickup_waiting_time = np.float(words[2])
                self.add_origin(od, node, pickup_waiting_time)
            else:
                # if line starting with '#' is encountered
                self.add_destination(od, node)

    def generate_text(self):
        # generate OD node mapping in text
        tmp_str = '#OriginID <-> NodeID pickup_waiting_time(s)\n'
        for O_ID, node_ID in self.O_dict.iteritems():
            tmp_str += ' '.join([str(e) for e in [O_ID,
                                node_ID, self.O_data_dict[O_ID]]]) + '\n'
        tmp_str += '#Dest_ID <-> node_ID\n'
        for D_ID, node_ID in self.D_dict.iteritems():
            tmp_str += ' '.join([str(e) for e in [D_ID, node_ID]]) + '\n'
        return tmp_str

    def __str__(self):
        return "MNM_od, number of O:" + str(len(self.O_dict)) + ", number of D:" + str(len(self.D_dict))

    def __repr__(self):
        return self.__str__()


class MNM_link_bus():
    def __init__(self, ID, from_busstop_ID, to_busstop_ID, length, fftt, route_ID, overlapped_dlink_ID_vec):
        # link ID
        self.ID = ID
        self.from_busstop_ID = from_busstop_ID
        self.to_busstop_ID = to_busstop_ID
        # length (unit: mile)
        self.length = length
        # free flow travel time (unit: hour)
        self.fftt = fftt
        self.route_ID = route_ID
        # np.array
        self.overlapped_dlink_ID_vec = overlapped_dlink_ID_vec

    def __str__(self):
        return ("MNM_link_bus, ID: {}, from_busstop_ID: {}, to_busstop_ID: {}, length: {} miles, fftt: {} hours, route_ID: {}, ".format(
                self.ID, self.from_busstop_ID, self.to_busstop_ID, self.length, self.fftt, self.route_ID)
                + 'overlapped_dlink_ID_vec: ' + ' '.join([str(e) for e in self.overlapped_dlink_ID_vec.tolist()]))

    def __repr__(self):
        return self.__str__()

    def generate_text(self):
        return ' '.join([str(e) for e in [self.ID, self.from_busstop_ID, self.to_busstop_ID, self.length, self.fftt, self.route_ID]
                                         + self.overlapped_dlink_ID_vec.tolist()])


class MNM_link_walking():
    def __init__(self, ID, from_node_ID, to_node_ID, from_node_type, to_node_type, walking_type, walking_time):
        assert((from_node_type in WALKING_NODE_ENUM) and (to_node_type in WALKING_NODE_ENUM)
               and (from_node_type != 'destination') and (to_node_type != 'origin'))
        assert(walking_type in WALKING_LINK_ENUM)
        # link ID
        self.ID = ID
        self.from_node_ID = from_node_ID
        self.to_node_ID = to_node_ID
        self.from_node_type = from_node_type
        self.to_node_type = to_node_type
        self.walking_type = walking_type
        # free flow travel time (unit: second)
        self.walking_time = walking_time

    def __str__(self):
        return ("MNM_link_walking, ID: {}, from_node_ID: {}, to_node_ID: {}, from_node_type: {}, to_node_type: {}, walking_type: {}, walking_time: {} seconds".format(
                self.ID, self.from_node_ID, self.to_node_ID, self.from_node_type, self.to_node_type, self.walking_type, self.walking_time))

    def __repr__(self):
        return self.__str__()

    def generate_text(self):
        return ' '.join([str(e) for e in [self.ID, self.from_node_ID, self.to_node_ID, self.from_node_type, self.to_node_type, self.walking_type, self.walking_time]])


class MNM_graph_bustransit():
    # directed graph
    def __init__(self):
        self.G = nx.MultiDiGraph()
        self.edgeID_dict = OrderedDict()

    def add_edge(self, s, e, ID, overwriting=False):
        # s: starting node ID
        # e: ending node ID
        # ID: link ID
        if (not overwriting) and self.G.has_edge(s, e, key=ID):
            raise("Error, edge exists in bustransit graph")
        else:
            self.G.add_edge(s, e, key=ID, ID=ID)
            self.edgeID_dict[ID] = (s, e)

    def add_node(self, node, overwriting=True):
        if (not overwriting) and node in self.G.nodes():
            raise("Error, node exists in bustransit graph")
        else:
            self.G.add_node(node)

    def build_from_file(self, bus_link_file_name, walking_link_file_name):
        self.G = nx.MultiDiGraph()
        self.edgeID_dict = OrderedDict()

        f = file(walking_link_file_name)
        # walking_link: the first line is the title line, keep lines from second to end
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            assert(len(words) >= 3)
            # link IDs are integers
            edge_id = np.int(words[0])
            # starting node IDs are integers
            from_id = np.int(words[1])
            # ending node IDs are integers
            to_id = np.int(words[2])
            self.add_edge(from_id, to_id, edge_id, overwriting=False)

        f = file(bus_link_file_name)
        # bus_link: the first line is the title line, keep lines from second to end
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            assert(len(words) >= 3)
            # link IDs are integers
            edge_id = np.int(words[0])
            # starting node IDs are integers
            from_id = np.int(words[1])
            # ending node IDs are integers
            to_id = np.int(words[2])
            self.add_edge(from_id, to_id, edge_id, overwriting=False)

    def __str__(self):
        return "MNM_graph_bustransit, number of node:" + str(self.G.number_of_nodes()) + ", number of edges:" + str(self.G.number_of_edges())

    def __repr__(self):
        return self.__str__()


class MNM_parkinglot():
    def __init__(self, ID, dest_node_ID, price, price_surge_coeff, avg_parking_time, capacity):
        self.ID = ID
        self.dest_node_ID = dest_node_ID
        self.price = price
        self.price_surge_coeff = price_surge_coeff
        self.avg_parking_time = avg_parking_time
        self.capacity = capacity

    def __str__(self):
        return ("MNM_parkinglot, ID: {}, dest_node_ID: {}, price: {}, price_surge_coeff: {}, avg_parking_time: {}, capacity: {}".format(
                self.ID, self.dest_node_ID, self.price, self.price_surge_coeff, self.avg_parking_time, self.capacity))

    def __repr__(self):
        return self.__str__()

    def generate_text(self):
        return ' '.join([str(e) for e in [self.ID, self.dest_node_ID, self.price, self.price_surge_coeff, self.avg_parking_time, self.capacity]])


class MNM_busstop_physical():
    def __init__(self, physical_busstop_ID, link_ID, location, route_ID_vec):
        self.physical_busstop_ID = physical_busstop_ID
        self.link_ID = link_ID
        self.location = location
        # np.array
        self.route_ID_vec = route_ID_vec

    def __str__(self):
        return ("MNM_busstop_physical, physical_busstop_ID: {}, link_ID: {}, location: {}, ".format(self.physical_busstop_ID, self.link_ID, self.location)
                + 'route_ID_vec: ' + ' '.join([str(e) for e in self.route_ID_vec.tolist()]))

    def __repr__(self):
        return self.__str__()

    def generate_text(self):
        return ' '.join([str(e) for e in [self.physical_busstop_ID, self.link_ID, self.location]
                                         + self.route_ID_vec.tolist()])


class MNM_busstop_virtual():
    def __init__(self, virtual_busstop_ID, physical_busstop_ID, route_ID):
        self.virtual_busstop_ID = virtual_busstop_ID
        self.physical_busstop_ID = physical_busstop_ID
        self.route_ID = route_ID

    def __str__(self):
        return ("MNM_busstop_virtual, virtual_busstop_ID: {}, physical_busstop_ID: {}, route_ID: {} ".format(self.virtual_busstop_ID, self.physical_busstop_ID, self.route_ID))

    def __repr__(self):
        return self.__str__()

    def generate_text(self):
        return ' '.join([str(e) for e in [self.virtual_busstop_ID, self.physical_busstop_ID, self.route_ID]])


class MNM_bus_route():
    # class of one bus route: a sequence of node IDs
    def __init__(self, path_ID, route_ID, node_list, virtual_busstop_list):
        # print("MNM_bus_route")
        self.path_ID = path_ID
        self.route_ID = route_ID
        self.virtual_busstop_list = virtual_busstop_list
        self.origin_node = node_list[0]
        self.destination_node = node_list[-1]
        self.node_list = node_list
        self.bus_route_portions = None

    def __eq__(self, other):
        # determine if two paths are equal
        if ((self.origin_node is None) or (self.destination_node is None) or
            (other.origin_node is None) or (other.destination_node is None)):
            return False
        if (other.origin_node != self.origin_node):
            return False
        if (other.destination_node != self.destination_node):
            return False
        if (other.route_ID != self.route_ID):
            return False
        if (len(self.node_list) != len(other.node_list)):
            return False
        for i in range(len(self.node_list)):
            if self.node_list[i] != other.node_list[i]:
                return False
        if (len(self.virtual_busstop_list) != len(other.virtual_busstop_list)):
            return False
        for i in range(len(self.virtual_busstop_list)):
            if self.virtual_busstop_list[i] != other.virtual_busstop_list[i]:
                return False

        return True

    def create_route_choice_portions_bus(self, num_intervals):
        # for bus: portion of buses using this path to the total bus demand for this OD pair
        self.bus_route_portions = np.zeros(num_intervals)

    def attach_route_choice_portions_bus(self, portions):
        # for bus: portion of buses using this path to the total bus demand for this OD pair
        self.bus_route_portions = portions

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return "MNM_bus_route, path ID {}, route ID {}, O node: {}, D node {}".format(self.path_ID, self.route_ID, self.origin_node, self.destination_node)

    def __repr__(self):
        return self.__str__()

    def generate_node_list_text(self):
        return ' '.join([str(e) for e in self.node_list])

    def generate_virtual_busstop_list_text(self):
        return ' '.join([str(e) for e in self.virtual_busstop_list])

    def generate_portion_text(self):
        assert(self.bus_route_portions is not None)
        return ' '.join([str(e) for e in self.bus_route_portions.tolist()])


class MNM_path_bustransit():
    # class of one bustransit path: a sequence of node IDs
    def __init__(self, path_ID, link_list, bustransit_graph):
        # bustransit_graph: MNM_graph_bustransit
        # print("MNM_path_bustransit")
        assert(isinstance(bustransit_graph, MNM_graph_bustransit))
        self.path_ID = path_ID
        self.link_list = link_list
        self.node_list = list()
        for l in link_list:
            self.node_list.append(bustransit_graph.edgeID_dict[l][0])
        self.node_list.append(bustransit_graph.edgeID_dict[link_list[-1]][1])
        self.origin_node = self.node_list[0]
        self.destination_node = self.node_list[-1]
        self.passenger_route_portions = None

    def __eq__(self, other):
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
        if (len(self.link_list) != len(other.link_list)):
            return False
        for i in range(len(self.link_list)):
            if self.link_list[i] != other.link_list[i]:
                return False

        return True

    def create_route_choice_portions_bustransit(self, num_intervals):
        # for passengers taking bus: portion of passengers using this path to the total bustransit demand for this OD pair
        self.passenger_route_portions = np.zeros(num_intervals)

    def attach_route_choice_portions_bustransit(self, portions):
        # for passengers taking bus: portion of passengers using this path to the total bustransit demand for this OD pair
        self.passenger_route_portions = portions

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return "MNM_path_bustransit, path ID {}, O node: {}, D node {}".format(self.path_ID, self.origin_node, self.destination_node)

    def __repr__(self):
        return self.__str__()

    def generate_node_list_text(self):
        return ' '.join([str(e) for e in self.node_list])

    def generate_link_list_text(self):
        return ' '.join([str(e) for e in self.link_list])

    def generate_portion_text(self):
        assert(self.passenger_route_portions is not None)
        return ' '.join([str(e) for e in self.passenger_route_portions.tolist()])


class MNM_path_pnr():
    # class of one pnr path: a sequence of node IDs
    def __init__(self, path_ID, mid_parkinglot_ID, driving_node_list, bustransit_link_list, bustransit_graph):
        # bustransit_graph: MNM_graph_bustransit
        # print("MNM_path_pnr")
        assert(isinstance(bustransit_graph, MNM_graph_bustransit))
        self.path_ID = path_ID
        self.mid_parkinglot_ID = mid_parkinglot_ID
        self.driving_path = MNM_path(driving_node_list, path_ID)
        self.bustransit_path = MNM_path_bustransit(path_ID, bustransit_link_list, bustransit_graph)
        self.origin_node = self.driving_path.node_list[0]
        self.mid_destination_node = self.driving_path.node_list[-1]
        self.destination_node = self.bustransit_path.node_list[-1]
        self.passenger_route_portions = None

    def __eq__(self, other):
        # determine if two paths are equal
        if ((self.origin_node is None) or (self.destination_node is None) or
            (other.origin_node is None) or (other.destination_node is None)):
            return False
        if (other.origin_node != self.origin_node):
            return False
        if (other.destination_node != self.destination_node):
            return False
        if (other.driving_path != self.driving_path) or (other.bustransit_path != self.bustransit_path):
            return False

        return True

    def create_route_choice_portions_pnr(self, num_intervals):
        # for passengers taking pnr: portion of passengers using this path to the total pnr demand for this OD pair
        self.passenger_route_portions = np.zeros(num_intervals)

    def attach_route_choice_portions_pnr(self, portions):
        # for passengers taking pnr: portion of passengers using this path to the total pnr demand for this OD pair
        self.passenger_route_portions = portions

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return "MNM_path_pnr, path ID {}, O node: {}, D node {}".format(self.path_ID, self.origin_node, self.destination_node)

    def __repr__(self):
        return self.__str__()

    def generate_driving_node_list_text(self):
        return ' '.join([str(e) for e in self.driving_path.node_list])

    def generate_bustransit_link_list_text(self):
        return ' '.join([str(e) for e in self.bustransit_path.link_list])

    def generate_portion_text(self):
        assert(self.passenger_route_portions is not None)
        return ' '.join([str(e) for e in self.passenger_route_portions.tolist()])


class MNM_pathset_bus_route():
    # path set for one OD pair: a list of paths
    def __init__(self):
        self.origin_node = None
        self.destination_node = None
        self.path_list = list()

    def add_path(self, path, overwriting=False):
        assert(path.origin_node == self.origin_node)
        assert(path.destination_node == self.destination_node)
        if (not overwriting) and (path in self.path_list):
            raise ("Error, path exists in path set")
        else:
            self.path_list.append(path)

    def normalize_route_portions(self, sum_to_OD=False):
        # for bus: normalizing the time-varing portion of buses using each path within [0, 1]
        for i in range(len(self.path_list) - 1):
            assert(self.path_list[i].bus_route_portions.shape ==
                   self.path_list[i + 1].bus_route_portions.shape)
        tmp_sum = np.zeros(self.path_list[0].bus_route_portions.shape)
        for i in range(len(self.path_list)):
            tmp_sum += self.path_list[i].bus_route_portions
        for i in range(len(self.path_list)):
            self.path_list[i].bus_route_portions = self.path_list[i].bus_route_portions / \
                np.maximum(tmp_sum, 1e-6)
        if sum_to_OD:
            return tmp_sum

    def __str__(self):
        return "MNM_pathset_bus_route, O node: {}, D node: {}, number_of_paths: {}".format(self.origin_node, self.destination_node, len(self.path_list))

    def __repr__(self):
        return self.__str__()
    

class MNM_pathset_bustransit():
    # path set for one OD pair: a list of paths
    def __init__(self):
        self.origin_node = None
        self.destination_node = None
        self.path_list = list()

    def add_path(self, path, overwriting=False):
        assert(path.origin_node == self.origin_node)
        assert(path.destination_node == self.destination_node)
        if (not overwriting) and (path in self.path_list):
            raise ("Error, path exists in path set")
        else:
            self.path_list.append(path)

    def normalize_route_portions(self, sum_to_OD=False):
        # for car: normalizing the time-varing portion of cars using each path within [0, 1]
        for i in range(len(self.path_list) - 1):
            assert(self.path_list[i].passenger_route_portions.shape ==
                   self.path_list[i + 1].passenger_route_portions.shape)
        tmp_sum = np.zeros(self.path_list[0].passenger_route_portions.shape)
        for i in range(len(self.path_list)):
            tmp_sum += self.path_list[i].passenger_route_portions
        for i in range(len(self.path_list)):
            self.path_list[i].passenger_route_portions = self.path_list[i].passenger_route_portions / \
                np.maximum(tmp_sum, 1e-6)
        if sum_to_OD:
            return tmp_sum

    def __str__(self):
        return "MNM_pathset_bustransit, O node: {}, D node: {}, number_of_paths: {}".format(self.origin_node, self.destination_node, len(self.path_list))

    def __repr__(self):
        return self.__str__()


class MNM_pathset_pnr():
    # path set for one OD pair: a list of paths
    def __init__(self):
        self.origin_node = None
        self.destination_node = None
        self.path_list = list()

    def add_path(self, path, overwriting=False):
        assert(path.origin_node == self.origin_node)
        assert(path.destination_node == self.destination_node)
        if (not overwriting) and (path in self.path_list):
            raise ("Error, path exists in path set")
        else:
            self.path_list.append(path)

    def normalize_route_portions(self, sum_to_OD=False):
        # for car: normalizing the time-varing portion of cars using each path within [0, 1]
        for i in range(len(self.path_list) - 1):
            assert(self.path_list[i].passenger_route_portions.shape ==
                   self.path_list[i + 1].passenger_route_portions.shape)
        tmp_sum = np.zeros(self.path_list[0].passenger_route_portions.shape)
        for i in range(len(self.path_list)):
            tmp_sum += self.path_list[i].passenger_route_portions
        for i in range(len(self.path_list)):
            self.path_list[i].passenger_route_portions = self.path_list[i].passenger_route_portions / \
                np.maximum(tmp_sum, 1e-6)
        if sum_to_OD:
            return tmp_sum

    def __str__(self):
        return "MNM_pathset_pnr, O node: {}, D node: {}, number_of_paths: {}".format(self.origin_node, self.destination_node, len(self.path_list))

    def __repr__(self):
        return self.__str__()


class MNM_pathtable_bus_route():
    # paths for all OD pairs: dictionary with value being class MNM_pathset_bus_route()
    def __init__(self):
        # print("MNM_pathtable_bus_route")
        self.path_dict = dict()
        self.ID2path = OrderedDict()

    def add_pathset(self, pathset, overwriting=False):
        if pathset.origin_node not in self.path_dict.keys():
            self.path_dict[pathset.origin_node] = dict()
        if (not overwriting) and (pathset.destination_node in self.path_dict[pathset.origin_node]):
            raise ("Error, pathset exists in the pathtable")
        else:
            self.path_dict[pathset.origin_node][pathset.destination_node] = pathset

    def build_from_file(self, path_table_file_name, route_file_time, w_ID=False, starting_ID=0):
        if w_ID:
            raise ("Error, path table build_from_file with ID not implemented")
        self.path_dict = dict()
        self.ID2path = OrderedDict()

        f = file(path_table_file_name)
        # bus_path_table: the first line is the title line, keep lines from second to end
        log_path = f.readlines()[1:]
        f.close()

        f = file(route_file_time)
        # bus_route: the first line is the title line, keep lines from second to end
        log_route = f.readlines()[1:]
        f.close()

        assert(len(log_path) == len(log_route))
        for i in range(len(log_path)):
            tmp_str = log_path[i]
            if tmp_str == '':
                continue
            words_path = tmp_str.split()
            assert(len(words_path) >= 4)

            tmp_str = log_route[i]
            if tmp_str == '':
                continue
            words_route = tmp_str.split()
            assert(len(words_route) >= 4)

            # OD node IDs and route ID are integers
            assert(np.int(words_path[0]) == np.int(words_route[0]))
            assert(np.int(words_path[1]) == np.int(words_route[1]))
            assert(np.int(words_path[2]) == np.int(words_route[2]))
            origin_node = np.int(words_path[0])
            destination_node = np.int(words_path[1])
            route_ID = np.int(words_path[2])

            if origin_node not in self.path_dict.keys():
                self.path_dict[origin_node] = dict()
            if destination_node not in self.path_dict[origin_node].keys():
                tmp_path_set = MNM_pathset_bus_route()
                tmp_path_set.origin_node = origin_node
                tmp_path_set.destination_node = destination_node
                self.add_pathset(tmp_path_set)
            # node IDs and busstop IDs are integers
            tmp_node_list = list(map(lambda x : np.int(x), words_path[3:]))
            tmp_busstop_list = list(map(lambda x : np.int(x), words_route[3:]))
            tmp_path = MNM_bus_route(i, route_ID, tmp_node_list, tmp_busstop_list)
            self.path_dict[origin_node][destination_node].add_path(tmp_path)
            self.ID2path[starting_ID + i] = tmp_path

    def __str__(self):
        return "MNM_pathtable_bus_route, number of paths: " + str(len(self.ID2path)) 

    def __repr__(self):
        return self.__str__()

    def generate_table_text(self):
        tmp_str = "#OriginNodeID DestNodeID RouteID <driving node IDs>\n"
        for path_ID, path in self.ID2path.iteritems():
            tmp_str += str(path.origin_node) + ' ' + str(path.destination_node) + ' ' \
                       + str(path.route_ID) + ' ' \
                       + path.generate_node_list_text() + '\n'
        return tmp_str

    def generate_route_text(self):
        tmp_str = "#OriginNodeID DestNodeID RouteID <virtual busstop IDs>\n"
        for path_ID, path in self.ID2path.iteritems():
            tmp_str += str(path.origin_node) + ' ' + str(path.destination_node) + ' ' \
                       + str(path.route_ID) + ' ' \
                       + path.generate_virtual_busstop_list_text() + '\n'
        return tmp_str 


class MNM_pathtable_bustransit():
    # paths for all OD pairs: dictionary with value being class MNM_pathset_bustransit()
    def __init__(self, bustransit_graph=None):
        # print("MNM_pathtable_bustransit")
        self.path_dict = dict()
        self.ID2path = OrderedDict()
        self.bustransit_graph = bustransit_graph

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
        assert(isinstance(self.bustransit_graph, MNM_graph_bustransit))
        self.path_dict = dict()
        self.ID2path = OrderedDict()
        f = file(file_name)
        # bustransit_path_table: the first line is the title line, keep lines from second to end
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i]
            if tmp_str == '':
                continue
            words = tmp_str.split()
            assert(len(words) >= 3)
            # OD node IDs are integers
            origin_node = np.int(words[0])
            destination_node = np.int(words[1])
            if origin_node not in self.path_dict.keys():
                self.path_dict[origin_node] = dict()
            if destination_node not in self.path_dict[origin_node].keys():
                tmp_path_set = MNM_pathset_bustransit()
                tmp_path_set.origin_node = origin_node
                tmp_path_set.destination_node = destination_node
                self.add_pathset(tmp_path_set)
            # link IDs are integers
            tmp_link_list = list(map(lambda x : np.int(x), words[2:]))
            tmp_path = MNM_path_bustransit(i, tmp_link_list, self.bustransit_graph)
            assert((tmp_path.origin_node == origin_node) and (tmp_path.destination_node == destination_node))
            self.path_dict[origin_node][destination_node].add_path(tmp_path)
            self.ID2path[starting_ID + i] = tmp_path

    def load_route_choice_from_file(self, file_name, w_ID=False, buffer_length=None, starting_ID=0):
        if w_ID:
            raise ("Error, pathtable load_route_choice_from_file not implemented")
        # bustransit_path_table_buffer: each entry is a sequence of time-dependent portions using each corresponding path
        # buffer_length = number of total intervals
        f = file(file_name)
        log = list(filter(lambda x: not x.strip() == '', f.readlines()))
        f.close()
        assert(len(log) == len(self.ID2path))
        for i in range(len(log)):
            tmp_portions = np.array(log[i].strip().split()).astype(np.float)
            if buffer_length is not None:
                assert(len(tmp_portions) == buffer_length)
                self.ID2path[starting_ID + i].attach_route_choice_portions_bustransit(tmp_portions[:buffer_length])
            else:
                raise("deprecated")

    def __str__(self):
        return "MNM_pathtable_bustransit, number of paths: " + str(len(self.ID2path)) 

    def __repr__(self):
        return self.__str__()

    def generate_table_text(self):
        tmp_str = "#OriginNodeID DestNodeID <transit link IDs in MultiGraph>\n"
        for path_ID, path in self.ID2path.iteritems():
            tmp_str += str(path.origin_node) + ' ' + str(path.destination_node) + ' ' + path.generate_link_list_text() + '\n'
        return tmp_str

    def generate_portion_text(self):
        tmp_str = ""
        for path_ID, path in self.ID2path.iteritems():
            tmp_str += path.generate_portion_text() + '\n'
        return tmp_str  


class MNM_pathtable_pnr():
    # paths for all OD pairs: dictionary with value being class MNM_pathset_pnr()
    def __init__(self, bustransit_graph=None):
        # print("MNM_pathtable_pnr")
        self.path_dict = dict()
        self.ID2path = OrderedDict()
        self.bustransit_graph = bustransit_graph

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
        assert(isinstance(self.bustransit_graph, MNM_graph_bustransit))
        self.path_dict = dict()
        self.ID2path = OrderedDict()
        f = file(file_name)
        # pnr_path_table: the first line is the title line, keep lines from second to end
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i]
            if tmp_str == '':
                continue
            words = tmp_str.split()
            assert(len(words) >= 6)
            # OD node IDs and parking lot ID are integers
            origin_node = np.int(words[0])
            destination_node = np.int(words[1])
            mid_parkinglot_ID = np.int(words[2])
            mid_destination_node = np.int(words[3])
            if origin_node not in self.path_dict.keys():
                self.path_dict[origin_node] = dict()
            if destination_node not in self.path_dict[origin_node].keys():
                tmp_path_set = MNM_pathset_bustransit()
                tmp_path_set.origin_node = origin_node
                tmp_path_set.destination_node = destination_node
                self.add_pathset(tmp_path_set)
            # link IDs are integers
            assert(str(mid_destination_node) in words[4:])
            tmp_ind = words[4:].index(str(mid_destination_node)) + 1
            assert(tmp_ind < len(words[4:]))
            tmp_node_list = list(map(lambda x : np.int(x), words[4:4+tmp_ind]))
            tmp_link_list = list(map(lambda x : np.int(x), words[4+tmp_ind:]))
            tmp_path = MNM_path_pnr(i, mid_parkinglot_ID, tmp_node_list, tmp_link_list, self.bustransit_graph)
            assert((tmp_path.origin_node == origin_node) and (tmp_path.destination_node == destination_node)
                   and (tmp_path.mid_destination_node == mid_destination_node))
            self.path_dict[origin_node][destination_node].add_path(tmp_path)
            self.ID2path[starting_ID + i] = tmp_path

    def load_route_choice_from_file(self, file_name, w_ID=False, buffer_length=None, starting_ID=0):
        if w_ID:
            raise ("Error, pathtable load_route_choice_from_file not implemented")
        # pnr_path_table_buffer: each entry is a sequence of time-dependent portions using each corresponding path
        # buffer_length = number of total intervals
        f = file(file_name)
        log = list(filter(lambda x: not x.strip() == '', f.readlines()))
        f.close()
        assert(len(log) == len(self.ID2path))
        for i in range(len(log)):
            tmp_portions = np.array(log[i].strip().split()).astype(np.float)
            if buffer_length is not None:
                assert(len(tmp_portions) == buffer_length)
                self.ID2path[starting_ID + i].attach_route_choice_portions_pnr(tmp_portions[:buffer_length])
            else:
                raise("deprecated")

    def __str__(self):
        return "MNM_pathtable_pnr, number of paths: " + str(len(self.ID2path)) 

    def __repr__(self):
        return self.__str__()

    def generate_table_text(self):
        tmp_str = "#OriginNodeID DestNodeID MidParkinglotID MidDestNodeID <driving node IDs> <transit link IDs in MultiGraph>\n"
        for path_ID, path in self.ID2path.iteritems():
            tmp_str += str(path.origin_node) + ' ' \
                       + str(path.destination_node) + ' ' \
                       + str(path.mid_parkinglot_ID) + ' ' \
                       + str(path.mid_destination_node) + ' ' \
                       + path.generate_driving_node_list_text() + ' ' \
                       + path.generate_bustransit_link_list_text() + '\n'
        return tmp_str

    def generate_portion_text(self):
        tmp_str = ""
        for path_ID, path in self.ID2path.iteritems():
            tmp_str += path.generate_portion_text() + '\n'
        return tmp_str


class MNM_config():
    def __init__(self):
        # print("MNM_config")
        self.config_dict = OrderedDict()
        self.type_dict = {
            # DTA
            'unit_time': np.int, 
            'total_interval': np.int,
            'assign_frq': np.int, 
            'start_assign_interval': np.int, 
            'max_interval': np.int,
            'flow_scalar': np.int, 

            'network_name': str, 
            'num_of_link': np.int, 
            'num_of_node': np.int,
            'num_of_O': np.int, 
            'num_of_D': np.int, 
            'OD_pair_passenger': np.int,
            'OD_pair_driving': np.int, 
            'num_bus_routes': np.int,
            'num_of_bus_stop_physical': np.int, 
            'num_of_bus_stop_virtual': np.int,
            'num_of_bus_link': np.int,
            'num_of_walking_link': np.int,
            'num_of_parking_lot': np.int,
            'OD_pair_pnr': np.int,
            'OD_pair_bustransit': np.int,

            'adaptive_ratio_passenger': np.float,
            'adaptive_ratio_car': np.float,
            'adaptive_ratio_truck': np.float,
            'routing_type': str, 

            'bus_capacity': np.int,
            'boarding_lost_time': np.float,
            'fixed_dwell_time': np.float,
            'boarding_time_per_passenger': np.float,
            'alighting_time_per_passenger': np.float,

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
            
            # FIXED, ADAPTIVE
            'route_frq': np.int,
            # self.config.config_dict['FIXED']['buffer_length'] is the number of columns in driving_path_table_buffer
            # when this is a multiclass problem, this is equal to total number of car and truck columns
            # DIFFERENT FROM Wei's MULTICLASS
            'buffer_length': np.int,
            'choice_portion': str,

            'driving_path_file_name': str, 
            'num_driving_path': np.int,
            'bus_path_file_name': str,
            'bus_route_file_name': str, 
            'num_bus_routes': np.int,
            'pnr_path_file_name': str, 
            'num_pnr_path': np.int,
            'bustransit_path_file_name': str, 
            'num_bustransit_path': np.int,

            # MMDUE
            'driving': np.int,
            'transit': np.int,
            'pnr': np.int,
            'alpha1_driving': np.float,
            'alpha1_transit': np.float,
            'alpha1_pnr': np.float,
            'beta1': np.float,

            'max_iter': np.int,
            'step_size': np.float,

            'vot': np.float,
            'early_penalty': np.float,
            'late_penalty': np.float,
            'target_time': np.int,

            'carpool_cost_multiplier': np.float,
            'pnr_inconvenience': np.float,
            'bus_inconvenience': np.float,
            'parking_lot_to_destination_walking_time': np.float,
            'bus_fare': np.float,
            'metro_fare': np.float,

            'init_demand_split': np.int
        }

    def build_from_file(self, file_name):
        self.config_dict = OrderedDict()
        # config.conf
        f = file(file_name)
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
            self.config_dict[new_item][name] = self.type_dict[name](words[1].strip())

    def __str__(self):
        tmp_str = ''

        tmp_str += '[DTA]\n'
        for name, value in self.config_dict['DTA'].iteritems():
            tmp_str += "{} = {}\n".format(str(name), str(value))

        tmp_str += '\n[STAT]\n'
        for name, value in self.config_dict['STAT'].iteritems():
            tmp_str += "{} = {}\n".format(str(name), str(value))

        assert(self.config_dict['DTA']['routing_type'] in 
               ['Multimodal_DUE_FixedPath', 'Multimodal_DUE_ColumnGeneration', 'Multimodal_Hybrid'])
            
        tmp_str += '\n[ADAPTIVE]\n'
        for name, value in self.config_dict['ADAPTIVE'].iteritems():
            tmp_str += "{} = {}\n".format(str(name), str(value))
            
        tmp_str += '\n[FIXED]\n'
        for name, value in self.config_dict['FIXED'].iteritems():
            tmp_str += "{} = {}\n".format(str(name), str(value))

        tmp_str += '\n[MMDUE]\n'
        for name, value in self.config_dict['MMDUE'].iteritems():
            tmp_str += "{} = {}\n".format(str(name), str(value))
            
        return tmp_str

    def __repr__(self):
        return self.__str__()


class MNM_network_builder():
    # build network for MMDTA considering three modes: driving (car and truck), bus transit, and PnR
    def __init__(self):
        self.config = MNM_config()

        self.network_driving_name = None
        
        self.link_driving_list = list()
        self.link_walking_list = list()
        self.link_bus_list = list()

        self.node_driving_list = list()
        self.busstop_virtual_list = list()
        self.busstop_physical_list = list()
        self.parkinglot_list = list()

        self.graph_driving = MNM_graph()
        self.graph_bustransit = MNM_graph_bustransit()

        self.od = MNM_od()

        self.demand_driving = MNM_demand()
        self.demand_bus = MNM_demand_bus()
        self.demand_bustransit = MNM_demand_bustransit()
        self.demand_pnr = MNM_demand_pnr()
        self.demand_total_passenger = MNM_demand_total_passenger()

        # dictionary recording all paths in all modes, order must match Mmdta_Api::m_ID_path_mapping in dta_api.cpp
        # <path_ID, (mode, path)>
        self.ID2path = OrderedDict()

        self.path_table_driving = MNM_pathtable()
        self.path_table_bus = MNM_pathtable_bus_route()
        self.path_table_bustransit = MNM_pathtable_bustransit(self.graph_bustransit)
        self.path_table_pnr = MNM_pathtable_pnr(self.graph_bustransit)

        self.route_choice_driving_flag = False
        self.route_choice_bustransit_flag = False
        self.route_choice_pnr_flag = False

    def set_network_driving_name(self, name):
        self.network_driving_name = name

    def get_link_driving(self, ID):
        for link in self.link_driving_list:
            if link.ID == ID:
                return link
        return None

    def get_link_walking(self, ID):
        for link in self.link_walking_list:
            if link.ID == ID:
                return link
        return None

    def get_link_bus(self, ID):
        for link in self.link_bus_list:
            if link.ID == ID:
                return link
        return None

    def read_link_driving_input(self, file_name):
        link_list = list()
        f = file(file_name)
        # skip the first title line
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            # column order in driving_link
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
            l = MNM_dlink(ID, length, typ, ffs_car, cap_car, rhoj_car, lanes, ffs_truck, cap_truck, rhoj_truck, convert_factor)
            l.is_ok()
            link_list.append(l)
        return link_list

    def read_link_bus_input(self, file_name):
        link_list = list()
        f = file(file_name)
        # skip the first title line
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            # column order in bus_link
            ID = np.int(words[0])
            from_busstop_ID = np.int(words[1])
            to_busstop_ID = np.int(words[2])
            length = np.float(words[3])
            fftt = np.float(words[4])
            route_ID = np.int(words[5])
            overlapped_dlink_ID_vec = list()
            for e in words[6:]:
                overlapped_dlink_ID_vec.append(np.int(e))
            overlapped_dlink_ID_vec = np.array(overlapped_dlink_ID_vec, dtype=np.int)
            l = MNM_link_bus(ID, from_busstop_ID, to_busstop_ID, length, fftt, route_ID, overlapped_dlink_ID_vec)
            link_list.append(l)
        return link_list

    def read_link_walking_input(self, file_name):
        link_list = list()
        f = file(file_name)
        # skip the first title line
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            # column order in walking_link
            ID = np.int(words[0])
            from_node_ID = np.int(words[1])
            to_node_ID = np.int(words[2])
            from_node_type = words[3]
            to_node_type = words[4]
            walking_type = words[5]
            walking_time = np.float(words[6])
            l = MNM_link_walking(ID, from_node_ID, to_node_ID, from_node_type, to_node_type, walking_type, walking_time)
            link_list.append(l)
        return link_list

    def read_node_driving_input(self, file_name):
        node_list = list()
        f = file(file_name)
        # skip the first title line
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            # column order in driving_node
            ID = np.int(words[0])
            typ = words[1]
            convert_factor = np.float(words[2])
            n = MNM_dnode(ID, typ, convert_factor)
            n.is_ok()
            node_list.append(n)
        return node_list

    def read_busstop_virtual_input(self, file_name):
        busstop_list = list()
        f = file(file_name)
        # skip the first title line
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            # column order in bus_stop_virtual
            virtual_busstop_ID = np.int(words[0])
            physical_busstop_ID = np.int(words[1])
            route_ID = np.int(words[2])
            n = MNM_busstop_virtual(virtual_busstop_ID, physical_busstop_ID, route_ID)
            busstop_list.append(n)
        return busstop_list

    def read_busstop_physical_input(self, file_name):
        busstop_list = list()
        f = file(file_name)
        # skip the first title line
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            # column order in bus_stop_physical
            physical_busstop_ID = np.int(words[0])
            link_ID = np.int(words[1])
            location = np.float(words[2])
            route_ID_vec = list()
            for e in words[3:]:
                route_ID_vec.append(np.int(e))
            route_ID_vec = np.array(route_ID_vec, dtype=np.int)
            n = MNM_busstop_physical(physical_busstop_ID, link_ID, location, route_ID_vec)
            busstop_list.append(n)
        return busstop_list

    def read_parkinglot_input(self, file_name):
        parkinglot_list = list()
        f = file(file_name)
        # skip the first title line
        log = f.readlines()[1:]
        f.close()
        for i in range(len(log)):
            tmp_str = log[i].strip()
            if tmp_str == '':
                continue
            words = tmp_str.split()
            # column order in parking_lot
            ID = np.int(words[0])
            dest_node_ID = np.int(words[1])
            price = np.float(words[2])
            price_surge_coeff = np.float(words[3])
            avg_parking_time = np.float(words[4])
            capacity = np.int(words[5])
            n = MNM_parkinglot(ID, dest_node_ID, price, price_surge_coeff, avg_parking_time, capacity)
            parkinglot_list.append(n)
        return parkinglot_list

    def get_ID_path_mapping_all_mode(self):
        # reading path table files must follow the order in MNM::get_ID_path_mapping_all_mode() in multimodal.cpp
        # driving -> bustransit -> pnr -> bus route
        assert(len(self.ID2path) == 0)
        for mode, path_table in zip(['driving', 'bustransit', 'pnr', 'bus'], 
                                    [self.path_table_driving, self.path_table_bustransit, self.path_table_pnr, self.path_table_bus]):
            for path_ID, path in path_table.ID2path.iteritems():
                assert(path_ID not in self.ID2path)
                self.ID2path[path_ID] = (mode, path)

    def load_from_folder(self, folder_path, config_file_name = 'config.conf', 
                        bus_link_file_name = 'bus_link', walking_link_file_name = 'walking_link',
                        bus_stop_virtual_file_name = 'bus_stop_virtual', bus_stop_physical_file_name = 'bus_stop_physical',
                        parking_lot_file_name = 'parking_lot', od_file_name = 'od',
                        driving_link_file_name = 'driving_link', driving_node_file_name = 'driving_node',
                        driving_graph_file_name = 'driving_graph', 
                        driving_pathtable_file_name = 'driving_path_table', driving_path_p_file_name = 'driving_path_table_buffer',
                        driving_demand_file_name = 'driving_demand',
                        bus_pathtable_file_name = 'bus_path_table', bus_route_file_name = 'bus_route',
                        bus_demand_file_name = 'bus_demand',
                        bustransit_pathtable_file_name = 'bustransit_path_table', bustransit_path_p_file_name = 'bustransit_path_table_buffer',
                        bustransit_demand_file_name = 'bustransit_demand',
                        pnr_pathtable_file_name = 'pnr_path_table', pnr_path_p_file_name = 'pnr_path_table_buffer',
                        pnr_demand_file_name = 'pnr_demand',
                        total_passenger_demand_file_name = 'passenger_demand'):
            
        if os.path.isfile(os.path.join(folder_path, config_file_name)):
            self.config.build_from_file(os.path.join(folder_path, config_file_name))
        else:
            raise("No config file")

        if not os.path.exists(os.path.join(folder_path, self.config.config_dict['STAT']['rec_folder'])):
            os.mkdir(os.path.join(folder_path, self.config.config_dict['STAT']['rec_folder']))

        if os.path.isfile(os.path.join(folder_path, bus_link_file_name)):
            self.link_bus_list = self.read_link_bus_input(os.path.join(folder_path, bus_link_file_name))
        else:
            raise("No bus link input")

        if os.path.isfile(os.path.join(folder_path, walking_link_file_name)):
            self.link_walking_list = self.read_link_walking_input(os.path.join(folder_path, walking_link_file_name))
        else:
            raise("No walking link input")

        self.graph_bustransit.build_from_file(os.path.join(folder_path, bus_link_file_name), 
                                              os.path.join(folder_path, walking_link_file_name))

        if os.path.isfile(os.path.join(folder_path, bus_stop_virtual_file_name)):
            self.busstop_virtual_list = self.read_busstop_virtual_input(os.path.join(folder_path, bus_stop_virtual_file_name))
        else:
            raise("No virtual bus stop input")

        if os.path.isfile(os.path.join(folder_path, bus_stop_physical_file_name)):
            self.busstop_physical_list = self.read_busstop_physical_input(os.path.join(folder_path, bus_stop_physical_file_name))
        else:
            raise("No physical bus stop input")

        if os.path.isfile(os.path.join(folder_path, parking_lot_file_name)):
            self.parkinglot_list = self.read_parkinglot_input(os.path.join(folder_path, parking_lot_file_name))
        else:
            raise("No parking lot input")

        if os.path.isfile(os.path.join(folder_path, od_file_name)):
            self.od.build_from_file(os.path.join(folder_path, od_file_name))
        else:
            raise("No OD input")

        if os.path.isfile(os.path.join(folder_path, driving_link_file_name)):
            self.link_driving_list = self.read_link_driving_input(os.path.join(folder_path, driving_link_file_name))
        else:
            raise("No driving link input")

        if os.path.isfile(os.path.join(folder_path, driving_node_file_name)):
            self.node_driving_list = self.read_node_driving_input(os.path.join(folder_path, driving_node_file_name))
        else:
            raise("No node input")

        if os.path.isfile(os.path.join(folder_path, driving_graph_file_name)):
            self.graph_driving.build_from_file(os.path.join(folder_path, driving_graph_file_name))
        else:
            raise("No driving graph input")

        # reading path table files must follow the order in MNM::get_ID_path_mapping_all_mode() in multimodal.cpp
        # driving -> bustransit -> pnr -> bus route
        path_count = 0

        # self.config.config_dict['FIXED']['buffer_length'] is the number of columns in driving_path_table_buffer
        # when this is a multiclass problem, this is equal to total number of car and truck columns
        # DIFFERENT FROM Wei's MULTICLASS
        if os.path.isfile(os.path.join(folder_path, driving_pathtable_file_name)):
            self.path_table_driving.build_from_file(os.path.join(folder_path, driving_pathtable_file_name), starting_ID=path_count)
            if os.path.isfile(os.path.join(folder_path, driving_path_p_file_name)):
                self.path_table_driving.load_route_choice_from_file(os.path.join(folder_path, driving_path_p_file_name), 
                                                                    buffer_length = self.config.config_dict['FIXED']['buffer_length']//2,
                                                                    starting_ID=path_count)
                self.route_choice_driving_flag = True
            else:
                self.route_choice_driving_flag = False
                print("No route choice portition for driving path table")

        else:
            raise("No driving path table input")

        if os.path.isfile(os.path.join(folder_path, driving_demand_file_name)):
            self.demand_driving.build_from_file(os.path.join(folder_path, driving_demand_file_name))
        else:
            raise("No driving demand input")

        path_count += len(self.path_table_driving.ID2path)

        if os.path.isfile(os.path.join(folder_path, bustransit_pathtable_file_name)):
            self.path_table_bustransit.build_from_file(os.path.join(folder_path, bustransit_pathtable_file_name), starting_ID=path_count)
            if os.path.isfile(os.path.join(folder_path, bustransit_path_p_file_name)):
                self.path_table_bustransit.load_route_choice_from_file(os.path.join(folder_path, bustransit_path_p_file_name), 
                                                                    buffer_length = self.config.config_dict['FIXED']['buffer_length']//2,
                                                                    starting_ID=path_count)
                self.route_choice_bustransit_flag = True
            else:
                self.route_choice_bustransit_flag = False
                print("No route choice portition for bus-transit path table")

        else:
            raise("No bus-transit path table input")

        if os.path.isfile(os.path.join(folder_path, bustransit_demand_file_name)):
            self.demand_bustransit.build_from_file(os.path.join(folder_path, bustransit_demand_file_name))
        else:
            raise("No bus-transit demand input")

        path_count += len(self.path_table_bustransit.ID2path)

        if os.path.isfile(os.path.join(folder_path, pnr_pathtable_file_name)):
            self.path_table_pnr.build_from_file(os.path.join(folder_path, pnr_pathtable_file_name), starting_ID=path_count)
            if os.path.isfile(os.path.join(folder_path, pnr_pathtable_file_name)):
                self.path_table_pnr.load_route_choice_from_file(os.path.join(folder_path, pnr_path_p_file_name), 
                                                                buffer_length = self.config.config_dict['FIXED']['buffer_length']//2,
                                                                starting_ID=path_count)
                self.route_choice_pnr_flag = True
            else:
                self.route_choice_pnr_flag = False
                print("No route choice portition for PnR path table")

        else:
            raise("No PnR path table input")

        if os.path.isfile(os.path.join(folder_path, pnr_demand_file_name)):
            self.demand_pnr.build_from_file(os.path.join(folder_path, pnr_demand_file_name))
        else:
            raise("No PnR demand input")

        path_count += len(self.path_table_pnr.ID2path)

        if os.path.isfile(os.path.join(folder_path, bus_pathtable_file_name)) and os.path.isfile(os.path.join(folder_path, bus_route_file_name)):
            self.path_table_bus.build_from_file(os.path.join(folder_path, bus_pathtable_file_name),
                                                os.path.join(folder_path, bus_route_file_name),
                                                starting_ID=path_count)
        else:
            raise("No bus path table or bus route input")

        if os.path.isfile(os.path.join(folder_path, bus_demand_file_name)):
            self.demand_bus.build_from_file(os.path.join(folder_path, bus_demand_file_name))
        else:
            raise("No bus demand input")

        path_count += len(self.path_table_bus.ID2path)

        self.get_ID_path_mapping_all_mode()
        assert(len(self.ID2path) == path_count)

        if os.path.isfile(os.path.join(folder_path, total_passenger_demand_file_name)):
            self.demand_total_passenger.build_from_file(os.path.join(folder_path, total_passenger_demand_file_name))
        else:
            print("No totoal passenger demand input")

    def generate_link_driving_text(self):
        tmp_str = '#ID Type LEN(mile) FFS_car(mile/h) Cap_car(v/hour) RHOJ_car(v/miles) Lane FFS_truck(mile/h) Cap_truck(v/hour) RHOJ_truck(v/miles) Convert_factor(1)\n'
        for link in self.link_driving_list:
            tmp_str += link.generate_text() + '\n'
        return tmp_str

    def generate_link_bus_text(self):
        tmp_str = '#EdgeID FromVirtualBusstopID ToVirtualBusstopID LEN(mile) FFTT(h) RouteID(each link belongs only one route ID, MultiGraph) <overlapped driving link IDs>\n'
        for link in self.link_bus_list:
            tmp_str += link.generate_text() + '\n'
        return tmp_str

    def generate_link_walking_text(self):
        tmp_str = '#EdgeId	FromNodeId ToNodeId FromNodeType ToNodeType WalkingType WalkingTime(s)(for boarding and alighting links, means s/person)\n'
        for link in self.link_walking_list:
            tmp_str += link.generate_text() + '\n'
        return tmp_str

    def generate_node_driving_text(self):
        tmp_str = '#ID Type Convert_factor(only for Inout node)\n'
        for node in self.node_driving_list:
            tmp_str += node.generate_text() + '\n'
        return tmp_str

    def generate_busstop_virtual_text(self):
        tmp_str = '#VirtualBusstopID PhysicalBusstopID RouteID\n'
        for node in self.busstop_virtual_list:
            tmp_str += node.generate_text() + '\n'
        return tmp_str

    def generate_busstop_physical_text(self):
        tmp_str = '#PhysicalBusstopID LinkID Loc(mile) <Route IDs>\n'
        for node in self.busstop_physical_list:
            tmp_str += node.generate_text() + '\n'
        return tmp_str

    def generate_parkinglot_text(self):
        tmp_str = '#ID DestNodeID Price PriceSurgeCoeff AvgParkingTime(s) Capacity \n'
        for node in self.parkinglot_list:
            tmp_str += node.generate_text() + '\n'
        return tmp_str

    def dump_to_folder(self, folder_path, config_file_name = 'config.conf', 
                        bus_link_file_name = 'bus_link', walking_link_file_name = 'walking_link',
                        bus_stop_virtual_file_name = 'bus_stop_virtual', bus_stop_physical_file_name = 'bus_stop_physical',
                        parking_lot_file_name = 'parking_lot', od_file_name = 'od',
                        driving_link_file_name = 'driving_link', driving_node_file_name = 'driving_node',
                        driving_graph_file_name = 'driving_graph', 
                        driving_pathtable_file_name = 'driving_path_table', driving_path_p_file_name = 'driving_path_table_buffer',
                        driving_demand_file_name = 'driving_demand',
                        bus_pathtable_file_name = 'bus_path_table', bus_route_file_name = 'bus_route',
                        bus_demand_file_name = 'bus_demand',
                        bustransit_pathtable_file_name = 'bustransit_path_table', bustransit_path_p_file_name = 'bustransit_path_table_buffer',
                        bustransit_demand_file_name = 'bustransit_demand',
                        pnr_pathtable_file_name = 'pnr_path_table', pnr_path_p_file_name = 'pnr_path_table_buffer',
                        pnr_demand_file_name = 'pnr_demand',
                        total_passenger_demand_file_name = 'passenger_demand'):
        if not os.path.isdir(folder_path):
            os.makedirs(folder_path)

        if not os.path.exists(os.path.join(folder_path, self.config.config_dict['STAT']['rec_folder'])):
            os.mkdir(os.path.join(folder_path, self.config.config_dict['STAT']['rec_folder']))

        f = open(os.path.join(folder_path, config_file_name), 'wb')
        f.write(str(self.config))
        f.close()

        f = open(os.path.join(folder_path, bus_link_file_name), 'wb')
        f.write(self.generate_link_bus_text())
        f.close()

        f = open(os.path.join(folder_path, walking_link_file_name), 'wb')
        f.write(self.generate_link_walking_text())
        f.close()

        f = open(os.path.join(folder_path, bus_stop_virtual_file_name), 'wb')
        f.write(self.generate_busstop_virtual_text())
        f.close()

        f = open(os.path.join(folder_path, bus_stop_physical_file_name), 'wb')
        f.write(self.generate_busstop_physical_text())
        f.close()

        f = open(os.path.join(folder_path, parking_lot_file_name), 'wb')
        f.write(self.generate_parkinglot_text())
        f.close()

        f = open(os.path.join(folder_path, od_file_name), 'wb')
        f.write(self.od.generate_text())
        f.close()

        f = open(os.path.join(folder_path, driving_link_file_name), 'wb')
        f.write(self.generate_link_driving_text())
        f.close()

        f = open(os.path.join(folder_path, driving_node_file_name), 'wb')
        f.write(self.generate_node_driving_text())
        f.close()

        f = open(os.path.join(folder_path, driving_graph_file_name), 'wb')
        f.write(self.graph_driving.generate_text())
        f.close()

        f = open(os.path.join(folder_path, driving_pathtable_file_name), 'wb')
        f.write(self.path_table_driving.generate_table_text())
        f.close()

        if self.route_choice_driving_flag:
            f = open(os.path.join(folder_path, driving_path_p_file_name), 'wb')
            f.write(self.path_table_driving.generate_portion_text())
            f.close()

        f = open(os.path.join(folder_path, driving_demand_file_name), 'wb')
        f.write(self.demand_driving.generate_text())
        f.close()

        f = open(os.path.join(folder_path, bus_pathtable_file_name), 'wb')
        f.write(self.path_table_bus.generate_table_text())
        f.close()

        f = open(os.path.join(folder_path, bus_route_file_name), 'wb')
        f.write(self.path_table_bus.generate_route_text())
        f.close()

        f = open(os.path.join(folder_path, bus_demand_file_name), 'wb')
        f.write(self.demand_bus.generate_text())
        f.close()

        f = open(os.path.join(folder_path, bustransit_pathtable_file_name), 'wb')
        f.write(self.path_table_bustransit.generate_table_text())
        f.close()

        if self.route_choice_bustransit_flag:
            f = open(os.path.join(folder_path, bustransit_path_p_file_name), 'wb')
            f.write(self.path_table_bustransit.generate_portion_text())
            f.close()

        f = open(os.path.join(folder_path, bustransit_demand_file_name), 'wb')
        f.write(self.demand_bustransit.generate_text())
        f.close()

        f = open(os.path.join(folder_path, pnr_pathtable_file_name), 'wb')
        f.write(self.path_table_pnr.generate_table_text())
        f.close()

        if self.route_choice_pnr_flag:
            f = open(os.path.join(folder_path, pnr_path_p_file_name), 'wb')
            f.write(self.path_table_pnr.generate_portion_text())
            f.close()

        f = open(os.path.join(folder_path, pnr_demand_file_name), 'wb')
        f.write(self.demand_pnr.generate_text())
        f.close()

        f = open(os.path.join(folder_path, total_passenger_demand_file_name), 'wb')
        f.write(self.demand_total_passenger.generate_text())
        f.close()

    def update_demand_path_driving(self, car_flow, truck_flow):
        # car_flow and truck_flow are 1D ndarrays recording the time-dependent flows with length of number of total paths x intervals
        assert (len(car_flow) == len(self.path_table_driving.ID2path) * self.config.config_dict['DTA']['max_interval'])
        assert (len(truck_flow) == len(self.path_table_driving.ID2path) * self.config.config_dict['DTA']['max_interval'])
        max_interval = self.config.config_dict['DTA']['max_interval']
        # reshape car_flow and truck_flow into ndarrays with dimensions of intervals x number of total paths
        car_flow = car_flow.reshape(self.config.config_dict['DTA']['max_interval'], len(self.path_table_driving.ID2path))
        truck_flow = truck_flow.reshape(self.config.config_dict['DTA']['max_interval'], len(self.path_table_driving.ID2path))
        # update time-varying portions
        for i, path_ID in enumerate(self.path_table_driving.ID2path.keys()):
            path = self.path_table_driving.ID2path[path_ID]
            path.attach_route_choice_portions(car_flow[:, i])
            path.attach_route_choice_portions_truck(truck_flow[:, i])
        self.demand_driving.demand_dict = dict()
        for O_node in self.path_table_driving.path_dict.keys():
            O = self.od.O_dict.inv[O_node]
            for D_node in self.path_table_driving.path_dict[O_node].keys():
                D = self.od.D_dict.inv[D_node]
                car_demand = self.path_table_driving.path_dict[O_node][D_node].normalize_route_portions(sum_to_OD = True)
                truck_demand = self.path_table_driving.path_dict[O_node][D_node].normalize_truck_route_portions(sum_to_OD = True)
                self.demand_driving.add_demand(O, D, car_demand, truck_demand, overwriting = True)
        self.route_choice_driving_flag = True

    def update_demand_path_bustransit(self, bustransit_flow):
        # bustransit_flow is 1D ndarray recording the time-dependent flows with length of number of total paths x intervals
        assert (len(bustransit_flow) == len(self.path_table_bustransit.ID2path) * self.config.config_dict['DTA']['max_interval'])
        max_interval = self.config.config_dict['DTA']['max_interval']
        # reshape bustransit_flow into ndarray with dimensions of intervals x number of total paths
        bustransit_flow = bustransit_flow.reshape(self.config.config_dict['DTA']['max_interval'], len(self.path_table_bustransit.ID2path))
        # update time-varying portions
        for i, path_ID in enumerate(self.path_table_bustransit.ID2path.keys()):
            path = self.path_table_bustransit.ID2path[path_ID]
            path.attach_route_choice_portions_bustransit(bustransit_flow[:, i])
        self.demand_bustransit.demand_dict = dict()
        for O_node in self.path_table_bustransit.path_dict.keys():
            O = self.od.O_dict.inv[O_node]
            for D_node in self.path_table_bustransit.path_dict[O_node].keys():
                D = self.od.D_dict.inv[D_node]
                bustransit_demand = self.path_table_bustransit.path_dict[O_node][D_node].normalize_route_portions(sum_to_OD = True)
                self.demand_bustransit.add_demand(O, D, bustransit_demand, overwriting = True)

    def update_demand_path_pnr(self, pnr_flow):
        # pnr_flow is 1D ndarray recording the time-dependent flows with length of number of total paths x intervals
        assert (len(pnr_flow) == len(self.path_table_pnr.ID2path) * self.config.config_dict['DTA']['max_interval'])
        max_interval = self.config.config_dict['DTA']['max_interval']
        # reshape pnr_flow into ndarray with dimensions of intervals x number of total paths
        pnr_flow = pnr_flow.reshape(self.config.config_dict['DTA']['max_interval'], len(self.path_table_pnr.ID2path))
        # update time-varying portions
        for i, path_ID in enumerate(self.path_table_pnr.ID2path.keys()):
            path = self.path_table_pnr.ID2path[path_ID]
            path.attach_route_choice_portions_pnr(pnr_flow[:, i])
        self.demand_pnr.demand_dict = dict()
        for O_node in self.path_table_pnr.path_dict.keys():
            O = self.od.O_dict.inv[O_node]
            for D_node in self.path_table_pnr.path_dict[O_node].keys():
                D = self.od.D_dict.inv[D_node]
                pnr_demand = self.path_table_pnr.path_dict[O_node][D_node].normalize_route_portions(sum_to_OD = True)
                self.demand_pnr.add_demand(O, D, pnr_demand, overwriting = True)