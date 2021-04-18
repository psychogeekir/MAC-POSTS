#ifndef DTA_API_H
#define DTA_API_H

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include "Snap.h"
#include "dta.h"
#include "multiclass.h"

#include <set>

namespace py = pybind11;

using SparseMatrixR = Eigen::SparseMatrix<double, Eigen::RowMajor>;

int run_dta(std::string folder);


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

#endif