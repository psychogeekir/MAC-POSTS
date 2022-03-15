#ifndef DTA_API_H
#define DTA_API_H

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include "Snap.h"
#include "dta.h"
#include "multiclass.h"
#include "multimodal.h"

#include <set>

namespace py = pybind11;

using SparseMatrixR = Eigen::SparseMatrix<double, Eigen::RowMajor>;

int run_dta(std::string folder);

class Tdsp_Api
{
public:
  Tdsp_Api();
  ~Tdsp_Api();
  int initialize(std::string folder, int max_interval, int num_rows_link_file, int num_rows_node_file,
                 std::string link_cost_file_name, std::string node_cost_file_name);
  int build_tdsp_tree(int dest_node_ID);
  py::array_t<double> extract_tdsp(int origin_node_ID, int timestamp);

  TInt m_num_rows_link_file, m_num_rows_node_file, m_dest_node_ID, m_max_interval;
  PNEGraph m_graph;
  MNM_TDSP_Tree *m_tdsp_tree;
  std::unordered_map<TInt, TFlt*> m_td_link_cost;
  std::unordered_map<TInt, std::unordered_map<TInt, TFlt*>> m_td_node_cost;
};


class Dta_Api
{
public:
  Dta_Api();
  ~Dta_Api();
  int initialize(std::string folder);
  int install_cc();
  int install_cc_tree();
  int run_once();
  int run_whole();
  int register_links(py::array_t<int> links);
  int register_paths(py::array_t<int> paths);
  int get_cur_loading_interval();
  py::array_t<double> get_link_inflow(py::array_t<int>start_intervals, 
                                        py::array_t<int>end_intervals);
  py::array_t<double> get_link_tt(py::array_t<int>start_intervals);
  py::array_t<double> get_path_tt(py::array_t<int>start_intervals);
  py::array_t<double> get_link_in_cc(int link_ID);
  py::array_t<double> get_link_out_cc(int link_ID);
  py::array_t<double> get_dar_matrix(py::array_t<int>link_start_intervals, py::array_t<int>link_end_intervals);
  SparseMatrixR get_complete_dar_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals,
                                                int num_intervals, py::array_t<double> f);
  MNM_Dta *m_dta;
  std::vector<MNM_Dlink*> m_link_vec;
  std::vector<MNM_Path*> m_path_vec;
  std::unordered_map<MNM_Path*, int> m_path_map; 
  // std::unordered_map<MNM_Dlink*, int> m_link_map; 
  std::unordered_map<TInt, MNM_Path*> m_ID_path_mapping;
};


class Mcdta_Api
{
public:
  Mcdta_Api();
  ~Mcdta_Api();
  int initialize(std::string folder);
  int install_cc();
  int install_cc_tree();
  int run_whole();
  int register_links(py::array_t<int> links);
  int get_cur_loading_interval();
  py::array_t<double> get_travel_stats();
  int print_emission_stats();
  int print_simulation_results(std::string folder, int cong_frequency = 180);
  
  py::array_t<double> get_car_link_tt(py::array_t<double>start_intervals);
  py::array_t<double> get_car_link_tt_robust(py::array_t<double>start_intervals, py::array_t<double>end_intervals);
  py::array_t<double> get_truck_link_tt(py::array_t<double>start_intervals);
  
  py::array_t<double> get_car_link_speed(py::array_t<double>start_intervals);
  py::array_t<double> get_truck_link_speed(py::array_t<double>start_intervals);
  
  py::array_t<double> get_link_car_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_link_truck_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  
  int register_paths(py::array_t<int> paths);
  py::array_t<double> get_car_link_out_cc(int link_ID); 
  py::array_t<double> get_car_link_in_cc(int link_ID); 
  py::array_t<double> get_truck_link_out_cc(int link_ID); 
  py::array_t<double> get_truck_link_in_cc(int link_ID); 

  py::array_t<double> get_enroute_and_queue_veh_stats_agg();
  py::array_t<double> get_queue_veh_each_link(py::array_t<int>useful_links, py::array_t<int>intervals);
  
  double get_car_link_out_num(int link_ID, double time);
  double get_truck_link_out_num(int link_ID, double time);

  py::array_t<double> get_car_dar_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_truck_dar_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  
  py::array_t<double> get_waiting_time_at_intersections();
  py::array_t<int> get_link_spillback();
  py::array_t<double> get_path_tt_car(py::array_t<int>link_IDs, py::array_t<double>start_intervals);
  py::array_t<double> get_path_tt_truck(py::array_t<int>link_IDs, py::array_t<double>start_intervals);

  MNM_Dta_Multiclass *m_mcdta;
  std::vector<MNM_Dlink_Multiclass*> m_link_vec;
  std::vector<MNM_Path*> m_path_vec;
  std::set<MNM_Path*> m_path_set; 
  std::unordered_map<TInt, MNM_Path*> m_ID_path_mapping;
};


class Mmdta_Api
{
public:
    Mmdta_Api();
    ~Mmdta_Api();
    int initialize(std::string folder);
    int initialize_mmdue(std::string folder);
    int install_cc();
    int install_cc_tree();
    int run_whole();
    int run_mmdue(std::string &folder);
    int run_mmdue_adaptive(std::string &folder);
    int register_links_driving(py::array_t<int> links_driving);
    int register_links_walking(py::array_t<int> links_walking);
    int register_links_bus(py::array_t<int> links_bus);
    int get_cur_loading_interval();
    py::array_t<double> get_travel_stats();
    int print_emission_stats();
    int print_simulation_results(std::string folder, int cong_frequency = 180);

    py::array_t<int> get_od_mode_connectivity();
    int generate_init_mode_demand_file(const std::string &file_folder);

    py::array_t<double> get_car_link_tt(py::array_t<double>start_intervals);
    py::array_t<double> get_car_link_tt_robust(py::array_t<double>start_intervals, py::array_t<double>end_intervals);
    py::array_t<double> get_truck_link_tt(py::array_t<double>start_intervals);
    py::array_t<double> get_bus_link_tt(py::array_t<double>start_intervals);
    // on boarding links, this is bus waiting time
    py::array_t<double> get_passenger_walking_link_tt(py::array_t<double>start_intervals);

    py::array_t<double> get_car_link_speed(py::array_t<double>start_intervals);
    py::array_t<double> get_truck_link_speed(py::array_t<double>start_intervals);
    py::array_t<double> get_bus_link_speed(py::array_t<double>start_intervals);

    py::array_t<double> get_link_car_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
    py::array_t<double> get_link_truck_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
    py::array_t<double> get_link_bus_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
    py::array_t<double> get_busstop_bus_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
    py::array_t<double> get_link_bus_passenger_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
    py::array_t<double> get_link_walking_passenger_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);

    py::array_t<double> get_car_link_out_cc(int link_ID);
    py::array_t<double> get_car_link_in_cc(int link_ID);
    py::array_t<double> get_truck_link_out_cc(int link_ID);
    py::array_t<double> get_truck_link_in_cc(int link_ID);
    py::array_t<double> get_bus_link_out_passenger_cc(int link_ID);
    py::array_t<double> get_bus_link_in_passenger_cc(int link_ID);
    py::array_t<double> get_bus_link_to_busstop_in_cc(int link_ID);
    py::array_t<double> get_bus_link_to_busstop_out_cc(int link_ID);
    py::array_t<double> get_bus_link_from_busstop_in_cc(int link_ID);
    py::array_t<double> get_bus_link_from_busstop_out_cc(int link_ID);
    py::array_t<double> get_walking_link_out_cc(int link_ID);
    py::array_t<double> get_walking_link_in_cc(int link_ID);

    py::array_t<double> get_enroute_and_queue_veh_stats_agg();
    py::array_t<double> get_enroute_and_queue_passenger_stats_agg();
    py::array_t<double> get_queue_veh_each_link(py::array_t<int>useful_links, py::array_t<int>intervals);
    py::array_t<double> get_queue_passenger_each_link(py::array_t<int>useful_links, py::array_t<int>intervals);

    double get_car_link_out_num(int link_ID, double time);
    double get_truck_link_out_num(int link_ID, double time);
    double get_passenger_link_out_num(int link_ID, double time);
    double get_bus_stop_arrival_num(int busstop_ID, double time);
    double get_bus_stop_departure_num(int busstop_ID, double time);

    int register_paths(py::array_t<int> paths);
    int register_paths_driving(py::array_t<int> paths);
    int register_paths_bustransit(py::array_t<int> paths);
    int register_paths_pnr(py::array_t<int> paths);
    int register_paths_bus(py::array_t<int> paths);

    int save_passenger_path_table(const std::string &file_folder);
    int save_mode_path_table(const std::string &file_folder);

    py::array_t<int> link_seq_to_node_seq_driving(py::array_t<int>link_IDs);
    py::array_t<int> link_seq_to_node_seq_bustransit(py::array_t<int>link_IDs);
    py::array_t<int> node_seq_to_link_seq_driving(py::array_t<int>node_IDs);
    py::array_t<int> node_seq_to_link_seq_bustransit(py::array_t<int>node_IDs);

    py::array_t<double> get_passenger_path_cost_driving(py::array_t<int>link_IDs, py::array_t<double>start_intervals);
    py::array_t<double> get_passenger_path_cost_bus(py::array_t<int>link_IDs, py::array_t<double>start_intervals);
    py::array_t<double> get_passenger_path_cost_pnr(py::array_t<int>link_IDs_driving, py::array_t<int>link_IDs_bustransit,
                                                    py::array_t<double>start_intervals);

    py::array_t<double> get_path_tt_car(py::array_t<int>link_IDs, py::array_t<double>start_intervals);
    py::array_t<double> get_path_tt_truck(py::array_t<int>link_IDs, py::array_t<double>start_intervals);
    py::array_t<double> get_registered_path_tt_truck(py::array_t<double>start_intervals);
    py::array_t<double> get_registered_path_tt_driving(py::array_t<double>start_intervals);
    py::array_t<double> get_registered_path_tt_bustransit(py::array_t<double>start_intervals);
    py::array_t<double> get_registered_path_tt_pnr(py::array_t<double>start_intervals);

    py::array_t<double> get_registered_path_cost_driving(py::array_t<double>start_intervals);
    py::array_t<double> get_registered_path_cost_bustransit(py::array_t<double>start_intervals);
    py::array_t<double> get_registered_path_cost_pnr(py::array_t<double>start_intervals);

    int update_tdsp_tree();
    py::array_t<int> get_lowest_cost_path(int start_interval, int o_node_ID, int d_node_ID);

    int build_link_cost_map();
    int build_link_cost_map_snapshot(int start_interval);
    int update_snapshot_route_table(int start_interval);
    py::array_t<int> get_lowest_cost_path_snapshot(int start_interval, int o_node_ID, int d_node_ID);

    py::array_t<double> get_waiting_time_at_intersections();
    py::array_t<int> get_link_spillback();

    py::array_t<double> get_car_dar_matrix_driving(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
    py::array_t<double> get_truck_dar_matrix_driving(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
    py::array_t<double> get_car_dar_matrix_pnr(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
    py::array_t<double> get_bus_dar_matrix_bustransit_link(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
    py::array_t<double> get_bus_dar_matrix_driving_link(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
    py::array_t<double> get_passenger_dar_matrix_bustransit(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
    py::array_t<double> get_passenger_dar_matrix_pnr(py::array_t<int>start_intervals, py::array_t<int>end_intervals);

    MNM_MM_Due *m_mmdue;
    MNM_Dta_Multimodal *m_mmdta;

    std::vector<MNM_Dlink_Multiclass*> m_link_vec_driving;
    std::vector<MNM_Walking_Link*> m_link_vec_walking;
    std::vector<MNM_Bus_Link*> m_link_vec_bus;

    std::vector<MNM_Path*> m_path_vec_driving;
    std::vector<MNM_Path*> m_path_vec_bus;
    std::vector<MNM_Path*> m_path_vec_bustransit;
    std::vector<MNM_Path*> m_path_vec_pnr;

    std::set<MNM_Path*> m_path_set_driving;
    std::set<MNM_Path*> m_path_set_bustransit;
    std::set<MNM_Path*> m_path_set_pnr;
    std::set<MNM_Path*> m_path_set_bus;

    std::set<TInt> m_pathID_set_driving;
    std::set<TInt> m_pathID_set_bustransit;
    std::set<TInt> m_pathID_set_pnr;
    std::set<TInt> m_pathID_set_bus;

    TInt m_num_path_driving;
    TInt m_num_path_bustransit;
    TInt m_num_path_pnr;
    TInt m_num_path_bus;

    // all paths from all modes
    std::vector<MNM_Path*> m_path_vec;
    std::set<MNM_Path*> m_path_set;
    std::unordered_map<TInt, std::pair<MNM_Path*, MNM_Passenger_Path_Base*>> m_ID_path_mapping;

    std::unordered_map<TInt, MNM_TDSP_Tree*> m_tdsp_tree_map_driving;
    std::unordered_map<TInt, MNM_TDSP_Tree*> m_tdsp_tree_map_bus;
};

#endif