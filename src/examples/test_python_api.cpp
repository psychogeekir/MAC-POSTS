//
// Created by qiling on 9/20/21.
//

#include "io.h"
#include "Snap.h"
#include "multimodal.h"

#include <vector>

//#include "../pybinder/pybind11/include/pybind11/pybind11.h"
//#include "../pybinder/pybind11/include/pybind11/numpy.h"
//namespace py = pybind11;

int main() {
    // print cwd
    char buffer[256];
    char *val = getcwd(buffer, sizeof(buffer));
    if (val) {
        std::cout << buffer << std::endl;
    }

    printf("BEGIN MNMAPI test!\n");

    // std::string folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode";
    // std::string folder = "/home/qiling/Documents/MAC-POSTS/side_project/network_builder/MNM_cache/input_files_7link_multimodal_dode_generated";
    // std::string folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode_columngeneration";
    // std::string folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_16link_multimodal";
    std::string folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode/record/input_files_estimate_path_flow";
    // std::string folder = "/srv/data/qiling/Projects/CentralOhio_Honda_Project/Multimodal/f04af81f922bd2ceba668c959b08f6d083e110d7";

    MNM_ConfReader *config = new MNM_ConfReader(folder + "/config.conf", "STAT");
    std::string rec_folder = config -> get_string("rec_folder");

    TInt m_num_path_driving = TInt(0);
    TInt m_num_path_bustransit = TInt(0);
    TInt m_num_path_pnr = TInt(0);
    TInt m_num_path_bus = TInt(0);

    // Mmdta_Api in dta_api.h
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

    // all paths from all modes
    std::vector<MNM_Path*> m_path_vec;
    std::set<MNM_Path*> m_path_set;
    std::unordered_map<TInt, std::pair<MNM_Path*, MNM_Passenger_Path_Base*>> m_ID_path_mapping;

    std::unordered_map<TInt, MNM_TDSP_Tree*> m_tdsp_tree_map_driving;
    std::unordered_map<TInt, MNM_TDSP_Tree*> m_tdsp_tree_map_bus;

    std::unordered_map<TInt, std::unordered_map<TInt, TInt>> m_driving_table_snapshot;
    std::unordered_map<TInt, std::unordered_map<TInt, TInt>> m_bustransit_table_snapshot;
    std::unordered_map<TInt, TFlt> m_driving_link_cost_map_snapshot;
    std::unordered_map<TInt, TFlt> m_bustransit_link_cost_map_snapshot;

    // ******************************************************
    // Mmdta_Api::initialize()
    // ******************************************************
    auto *m_mmdue = new MNM_MM_Due(folder);
    m_mmdue -> initialize();
    IAssert(m_mmdue -> m_mmdta_config -> get_string("routing_type") == "Multimodal_Hybrid" ||
            m_mmdue -> m_mmdta_config -> get_string("routing_type") == "Multimodal_Hybrid_ColumnGeneration");
    IAssert(m_mmdue -> m_passenger_path_table != nullptr && !m_mmdue -> m_passenger_path_table -> empty());

    MNM_Dta_Multimodal *m_mmdta = m_mmdue -> m_mmdta;
    // m_mmdta = new MNM_Dta_Multimodal(folder);
    // m_mmdta -> build_from_files();
    // m_mmdta -> hook_up_node_and_link();
    // m_mmdta -> find_connected_pnr_parkinglot_for_destination();
    // m_mmdta -> is_ok();

    auto* _tmp_conf = new MNM_ConfReader(m_mmdue -> m_file_folder + "/config.conf", "FIXED");
    if (_tmp_conf -> get_int("num_driving_path") > 0) {
        m_num_path_driving = _tmp_conf -> get_int("num_driving_path");
    }
    if (_tmp_conf -> get_int("num_bustransit_path") > 0) {
        m_num_path_bustransit = _tmp_conf -> get_int("num_bustransit_path");
    }
    if (_tmp_conf -> get_int("num_pnr_path") > 0) {
        m_num_path_pnr = _tmp_conf -> get_int("num_pnr_path");
    }
    if (_tmp_conf -> get_int("num_bus_routes") > 0) {
        m_num_path_bus = _tmp_conf -> get_int("num_bus_routes");
    }
    delete _tmp_conf;

    if (MNM_Routing_Multimodal_Hybrid *_routing = dynamic_cast<MNM_Routing_Multimodal_Hybrid*>(m_mmdta -> m_routing)) {
        // !!!!!! make sure path_IDs across all modes are unique
        printf("MNM_Routing_Multimodal_Hybrid start load ID path mapping\n");
        // car and truck share the same path_table
        // m_mmdue -> m_passenger_path_table is also affected
        MNM::get_ID_path_mapping_all_mode(m_ID_path_mapping,
                                          _routing->m_routing_fixed_car->m_path_table,
                                          _routing->m_routing_bus_fixed->m_bus_path_table,
                                          _routing->m_routing_car_pnr_fixed->m_pnr_path_table,
                                          _routing->m_routing_passenger_fixed->m_bustransit_path_table,
                                          m_mmdue->m_passenger_path_table,
                                          m_num_path_driving, m_num_path_bustransit, m_num_path_pnr, m_num_path_bus);
        printf("MNM_Routing_Multimodal_Hybrid mapping size %d\n", (int) m_ID_path_mapping.size());
    }

    // ******************************************************
    // Mmdta_Api::save_mode_path_table
    // ******************************************************
    if (m_mmdue -> m_mmdta_config -> get_string("routing_type") == "Multimodal_Hybrid_ColumnGeneration") {
        m_mmdue -> passenger_path_table_to_multimodal_path_table(m_mmdta);
        if (m_mmdue -> m_driving_path_table == nullptr) {
            throw std::runtime_error("Error, null driving path table");
        }
        if (m_mmdue -> m_bustransit_path_table == nullptr) {
            throw std::runtime_error("Error, null bustransit path table");
        }
        if (m_mmdue -> m_pnr_path_table == nullptr) {
            throw std::runtime_error("Error, null pnr path table");
        }

        MNM::save_driving_path_table(folder, m_mmdue -> m_driving_path_table,
                                     "driving_path_table", "driving_path_table_buffer", true);
        MNM::save_bustransit_path_table(folder, m_mmdue -> m_bustransit_path_table,
                                        "bustransit_path_table", "bustransit_path_table_buffer", true);
        MNM::save_pnr_path_table(folder, m_mmdue -> m_pnr_path_table,
                                 "pnr_path_table", "pnr_path_table_buffer", true);
    }


    // ******************************************************
    // Mmdta_Api::get_od_mode_connectivity
    // ******************************************************
    // {
    //     int _num_col = 5;
    //     int _num_OD = m_mmdue -> m_mmdta_config -> get_int("OD_pair_passenger");
    //     // O_node, D_node, driving, bustransit, pnr

    //     int result_prt[_num_OD * _num_col];
    //     int i = 0;
    //     for (const auto& _o_it : m_mmdue -> m_od_mode_connectivity) {
    //         result_prt[i*_num_col] = _o_it.first;
    //         for (const auto& _d_it : _o_it.second) {
    //             result_prt[i*_num_col + 1] = _d_it.first;
    //             for (auto _mode_it : _d_it.second) {
    //                 if (_mode_it.first == driving) {
    //                     result_prt[i*_num_col + 2] = (int) _mode_it.second;
    //                 }
    //                 else if (_mode_it.first == transit) {
    //                     result_prt[i*_num_col + 3] = (int) _mode_it.second;
    //                 }
    //                 else if (_mode_it.first == pnr) {
    //                     result_prt[i*_num_col + 4] = (int) _mode_it.second;
    //                 } else {
    //                     throw std::runtime_error("Error, Mmdta_Api::get_od_mode_connectivity, mode not implemented");
    //                 }
    //             }
    //             printf("O node: %d, D node: %d, driving: %d, bustransit: %d, pnr: %d\n",
    //                    result_prt[i*_num_col], result_prt[i*_num_col+1], result_prt[i*_num_col+2], result_prt[i*_num_col+3], result_prt[i*_num_col+4]);
    //             i++;
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::generate_init_mode_demand_file
    // ******************************************************
    {
        if (m_mmdue->m_mmdta_config->get_string("routing_type") == "Multimodal_Hybrid_ColumnGeneration") {
            MNM::generate_init_mode_demand_file(m_mmdue, folder, "driving_demand", "bustransit_demand", "pnr_demand");
        }
    }

    // ******************************************************
    // Mmdta_Api::register_links_driving
    // ******************************************************
    if (m_link_vec_driving.size() > 0){
        printf("Warning, Mmdta_Api::register_links_driving, link exists\n");
        m_link_vec_driving.clear();
    }
    std::vector<int> links_ptr = std::vector<int>();
    int _count = 0;
    //    for (auto it: m_mmdta -> m_link_factory -> m_link_map) {
    //        links_ptr.push_back((int)it.first);
    //        _count++;
    //    }
    std::vector<int> links_ID = {2, 3, 4, 5};
    for (auto it: links_ID) {
        links_ptr.push_back(it);
        _count++;
    }
    MNM_Dlink *_link;
    for (int i = 0; i < _count; i++){
        _link = m_mmdta -> m_link_factory -> get_link(TInt(links_ptr[i]));
        // printf("%d\n", links_ptr[i]);
        if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
            if(std::find(m_link_vec_driving.begin(), m_link_vec_driving.end(), _link) != m_link_vec_driving.end()) {
                throw std::runtime_error("Error, Mmdta_Api::register_links_driving, link does not exist");
            }
            else {
                m_link_vec_driving.push_back(_mclink);
            }
        }
        else{
            throw std::runtime_error("Mmdta_Api::register_links_driving: link type is not multiclass");
        }
    }

    // ******************************************************
    // Mmdta_Api::register_links_walking
    // ******************************************************
    // if (m_link_vec_walking.size() > 0){
    //     printf("Warning, Mmdta_Api::register_links_walking, link exists\n");
    //     m_link_vec_walking.clear();
    // }
    // links_ptr.clear();
    // _count = 0;
    // //    for (auto it: m_mmdta -> m_transitlink_factory -> m_transit_link_map) {
    // //        if (MNM_Walking_Link * _wlink = dynamic_cast<MNM_Walking_Link *>(it.second)) {
    // //            links_ptr.push_back((int)it.first);
    // //            _count++;
    // //        }
    // //    }
    // links_ID.clear();
    // links_ID = {16, 17, 20};
    // for (auto it: links_ID) {
    //     links_ptr.push_back(it);
    //     _count++;
    // }
    // MNM_Transit_Link *_twlink;
    // for (int i = 0; i < _count; i++){
    //     _twlink = m_mmdta -> m_transitlink_factory ->get_transit_link(TInt(links_ptr[i]));
    //     // printf("%d\n", links_ptr[i]);
    //     if (MNM_Walking_Link * _wlink = dynamic_cast<MNM_Walking_Link *>(_twlink)){
    //         if(std::find(m_link_vec_walking.begin(), m_link_vec_walking.end(), _wlink) != m_link_vec_walking.end()) {
    //             throw std::runtime_error("Error, Mmdta_Api::register_links_walking, link does not exist");
    //         }
    //         else {
    //             m_link_vec_walking.push_back(_wlink);
    //         }
    //     }
    //     else{
    //         throw std::runtime_error("Mmdta_Api::register_links_walking: link type is not walking");
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::register_links_bus
    // ******************************************************
    // if (m_link_vec_bus.size() > 0){
    //     printf("Warning, Mmdta_Api::register_links_bus, link exists\n");
    //     m_link_vec_bus.clear();
    // }
    // links_ptr.clear();
    // _count = 0;
    // //    for (auto it: m_mmdta -> m_transitlink_factory -> m_transit_link_map) {
    // //        if (MNM_Bus_Link * _blink = dynamic_cast<MNM_Bus_Link *>(it.second)) {
    // //            links_ptr.push_back((int)it.first);
    // //            _count++;
    // //        }
    // //    }
    // links_ID.clear();
    // links_ID = {205, 206};
    // for (auto it: links_ID) {
    //     links_ptr.push_back(it);
    //     _count++;
    // }
    // MNM_Transit_Link *_tblink;
    // for (int i = 0; i < _count; i++){
    //     _tblink = m_mmdta -> m_transitlink_factory ->get_transit_link(TInt(links_ptr[i]));
    //     // printf("%d\n", links_ptr[i]);
    //     if (MNM_Bus_Link * _blink = dynamic_cast<MNM_Bus_Link *>(_tblink)){
    //         if(std::find(m_link_vec_bus.begin(), m_link_vec_bus.end(), _blink) != m_link_vec_bus.end()) {
    //             throw std::runtime_error("Error, Mmdta_Api::register_links_bus, link does not exist");
    //         }
    //         else {
    //             m_link_vec_bus.push_back(_blink);
    //         }
    //     }
    //     else{
    //         throw std::runtime_error("Mmdta_Api::register_links_bus: link type is not walking");
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::register_paths
    // ******************************************************
    if (m_path_vec.size() > 0){
        printf("Warning, Mmdta_Api::register_paths, path exists\n");
        m_path_vec.clear();
        m_path_set.clear();
    }

    std::vector<int> paths_ptr = std::vector<int>(); // = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    for (int i = 0; i < (int)m_ID_path_mapping.size(); i++){
        paths_ptr.push_back(i);
    }
    TInt _path_ID;
    for (int i = 0; i < (int)paths_ptr.size(); i++){
        _path_ID = TInt(paths_ptr[i]);
        // printf("registering path %d, %d\n", _path_ID(), (int)m_ID_path_mapping.size());
        if (m_ID_path_mapping.find(_path_ID) == m_ID_path_mapping.end()){
            throw std::runtime_error("Mmdta_Api::register_paths: No such path");
        }
        else {
            m_path_vec.push_back(m_ID_path_mapping[_path_ID].first);
            if (m_ID_path_mapping[_path_ID].first -> m_path_type == driving) {
                m_path_vec_driving.push_back(m_ID_path_mapping[_path_ID].first);
            }
            else if (m_ID_path_mapping[_path_ID].first -> m_path_type == transit) {
                m_path_vec_bustransit.push_back(m_ID_path_mapping[_path_ID].first);
            }
            else if (m_ID_path_mapping[_path_ID].first -> m_path_type == pnr) {
                m_path_vec_pnr.push_back(m_ID_path_mapping[_path_ID].first);
            }
            else if (m_ID_path_mapping[_path_ID].first -> m_path_type == bus_route) {
                m_path_vec_bus.push_back(m_ID_path_mapping[_path_ID].first);
            }
        }
    }
    m_path_set_driving = std::set<MNM_Path*>(m_path_vec_driving.begin(), m_path_vec_driving.end());
    m_path_set_bustransit = std::set<MNM_Path*>(m_path_vec_bustransit.begin(), m_path_vec_bustransit.end());
    m_path_set_pnr = std::set<MNM_Path*>(m_path_vec_pnr.begin(), m_path_vec_pnr.end());
    m_path_set_bus = std::set<MNM_Path*>(m_path_vec_bus.begin(), m_path_vec_bus.end());
    m_path_set = std::set<MNM_Path*>(m_path_vec.begin(), m_path_vec.end());

    if (m_path_set_driving.size() != m_path_vec_driving.size() || int(m_path_set_driving.size()) != m_num_path_driving) {
        printf("repeated driving paths\n");
        exit(-1);
    }
    if (m_path_set_bustransit.size() != m_path_vec_bustransit.size() || int(m_path_set_bustransit.size()) != m_num_path_bustransit) {
        printf("repeated bustransit paths\n");
        exit(-1);
    }
    if (m_path_set_pnr.size() != m_path_vec_pnr.size() || int(m_path_set_pnr.size()) != m_num_path_pnr) {
        printf("repeated pnr paths\n");
        exit(-1);
    }
    if (m_path_set_bus.size() != m_path_vec_bus.size() || int(m_path_set_bus.size()) != m_num_path_bus) {
        printf("repeated bus routes\n");
        exit(-1);
    }
    if (m_path_set.size() != m_path_vec.size() || int(m_path_set.size()) != m_num_path_driving + m_num_path_bustransit + m_num_path_pnr + m_num_path_bus) {
        printf("repeated paths\n");
        exit(-1);
    }

    // ******************************************************
    // Mmdta_Api::install_cc()
    // ******************************************************
    // // car and truck
    // for (size_t i = 0; i < m_link_vec_driving.size(); ++i){
    //     m_link_vec_driving[i] -> install_cumulative_curve_multiclass();
    // }
    // // passenger
    // for (size_t i = 0; i < m_link_vec_walking.size(); ++i){
    //     m_link_vec_walking[i] -> install_cumulative_curve();
    // }
    // // bus and passenger
    // for (size_t i = 0; i < m_link_vec_bus.size(); ++i){
    //     // passenger
    //     m_link_vec_bus[i] -> install_cumulative_curve();
    //     // bus
    //     m_link_vec_bus[i] -> m_from_busstop -> install_cumulative_curve_multiclass();
    //     m_link_vec_bus[i] -> m_to_busstop -> install_cumulative_curve_multiclass();
    // }

    // ******************************************************
    // Mmdta_Api::install_cc_tree()
    // ******************************************************
    // // car and truck
    // for (size_t i = 0; i < m_link_vec_driving.size(); ++i){
    //     m_link_vec_driving[i] -> install_cumulative_curve_tree_multiclass();
    // }
    // // passenger
    // for (size_t i = 0; i < m_link_vec_walking.size(); ++i){
    //     m_link_vec_walking[i] -> install_cumulative_curve_tree();
    // }
    // // bus and passenger
    // for (size_t i = 0; i < m_link_vec_bus.size(); ++i){
    //     m_link_vec_bus[i] -> install_cumulative_curve_tree();
    // }

    // ******************************************************
    // Mmdta_Api::run_whole()
    // ******************************************************
    m_mmdta -> pre_loading();
    m_mmdta -> loading(true);


    // ******************************************************
    // Mmdta_Api::get_cur_loading_interval()
    // ******************************************************
    printf("loading interval is %d\n", m_mmdta -> m_current_loading_interval());

    // ******************************************************
    // Mmdta_Api::get_travel_stats()
    // ******************************************************
    {
        TInt _count_car = 0, _count_truck = 0, _count_bus = 0, _count_passenger = 0;
        TFlt _tot_tt_car = 0.0, _tot_tt_truck = 0.0, _tot_tt_bus = 0.0, _tot_tt_passenger = 0.0;
        MNM_Veh_Multimodal *_veh;
        MNM_Passenger *_passenger;
        int _end_time = m_mmdta -> m_current_loading_interval();

        for (auto _map_it : m_mmdta -> m_veh_factory -> m_veh_map){
            _veh = dynamic_cast<MNM_Veh_Multimodal *>(_map_it.second);
            if (_veh -> m_class == 0){
                _count_car += 1;
                if (_veh -> m_finish_time > 0) {
                    _tot_tt_car += (_veh -> m_finish_time - _veh -> m_start_time) * m_mmdta -> m_unit_time / 3600.0;
                }
                else {
                    _tot_tt_car += (_end_time - _veh -> m_start_time) * m_mmdta -> m_unit_time / 3600.0;
                }
            }
            else {
                if (_veh -> m_bus_route_ID == TInt(-1)) {
                    _count_truck += 1;
                    if (_veh -> m_finish_time > 0) {
                        _tot_tt_truck += (_veh -> m_finish_time - _veh -> m_start_time) * m_mmdta -> m_unit_time / 3600.0;
                    }
                    else {
                        _tot_tt_truck += (_end_time - _veh -> m_start_time) * m_mmdta -> m_unit_time / 3600.0;
                    }
                }
                else {
                    _count_bus += 1;
                    if (_veh -> m_finish_time > 0) {
                        _tot_tt_bus += (_veh -> m_finish_time - _veh -> m_start_time) * m_mmdta -> m_unit_time / 3600.0;
                    }
                    else {
                        _tot_tt_bus += (_end_time - _veh -> m_start_time) * m_mmdta -> m_unit_time / 3600.0;
                    }
                }

            }
        }

        for (auto _map_it : m_mmdta -> m_passenger_factory -> m_passenger_map){
            if (_map_it.second -> m_finish_time > 0) {
                _passenger = _map_it.second;
                _count_passenger += 1;
                if (_passenger -> m_finish_time > 0) {
                    _tot_tt_passenger += (_passenger -> m_finish_time - _passenger -> m_start_time) * m_mmdta -> m_unit_time / 3600.0;
                }
                else {
                    _tot_tt_passenger += (_end_time - _passenger -> m_start_time) * m_mmdta -> m_unit_time / 3600.0;
                }
            }
        }

//    printf("\n\nTotal car: %d, Total truck: %d, Total bus : %d, Total passenger: %d, Total car tt: %.2f hours, Total truck tt: %.2f hours, Total bus tt: %.2f hours, Total passenger tt: %.2f hours\n\n",
//           int(_count_car/m_mmdta -> m_flow_scalar), int(_count_truck/m_mmdta -> m_flow_scalar), int(_count_bus/m_mmdta -> m_flow_scalar), int(_count_passenger),
//           float(_tot_tt_car/m_mmdta -> m_flow_scalar), float(_tot_tt_truck/m_mmdta -> m_flow_scalar), float(_tot_tt_bus/m_mmdta -> m_flow_scalar), float(_tot_tt_passenger));
//    m_mmdta -> m_emission -> output();

        printf("\n************ travel stats ************\n");
        printf("car count: %f\n", _count_car/m_mmdta -> m_flow_scalar);
        printf("truck count: %f\n", _count_truck/m_mmdta -> m_flow_scalar);
        printf("bus count: %f\n", _count_bus/m_mmdta -> m_flow_scalar);
        printf("passenger count: %f\n", (float)_count_passenger);
        printf("car total travel time: %f\n", _tot_tt_car/m_mmdta -> m_flow_scalar);
        printf("truck total travel time: %f\n", _tot_tt_truck/m_mmdta -> m_flow_scalar);
        printf("bus total travel time: %f\n", _tot_tt_bus/m_mmdta -> m_flow_scalar);
        printf("passenger total travel time: %f\n", (float)_tot_tt_passenger);
        printf("************ travel stats ************\n");
    }

    // ******************************************************
    // Mmdta_Api::update_tdsp_tree()
    // ******************************************************
    // {
    //     if (!m_tdsp_tree_map_driving.empty()) {
    //         for (auto _it : m_tdsp_tree_map_driving) {
    //             delete _it.second;
    //         }
    //         m_tdsp_tree_map_driving.clear();
    //     }
    //     if (!m_tdsp_tree_map_bus.empty()) {
    //         for (auto _it : m_tdsp_tree_map_bus) {
    //             delete _it.second;
    //         }
    //         m_tdsp_tree_map_bus.clear();
    //     }

    //     MNM_Destination *_dest;
    //     TInt _dest_node_ID;
    //     MNM_TDSP_Tree *_tdsp_tree;

    //     m_mmdue -> build_link_cost_map(m_mmdta);
    //     for (auto _d_it : m_mmdta->m_od_factory->m_destination_map) {
    //         _dest = _d_it.second;
    //         _dest_node_ID = _dest->m_dest_node->m_node_ID;

    //         // for driving
    //         _tdsp_tree = new MNM_TDSP_Tree(_dest_node_ID, m_mmdta->m_graph, m_mmdue -> m_total_loading_inter);
    //         _tdsp_tree->initialize();
    //         _tdsp_tree->update_tree(m_mmdue -> m_link_cost_map);
    //         m_tdsp_tree_map_driving.insert(std::pair<TInt, MNM_TDSP_Tree*>(_dest_node_ID, _tdsp_tree));
    //         _tdsp_tree = nullptr;
    //         IAssert(m_tdsp_tree_map_driving.find(_dest_node_ID) -> second != nullptr);

    //         // for bus transit
    //         if (m_mmdta -> m_bus_transit_graph -> IsNode(_dest_node_ID)) {
    //             _tdsp_tree = new MNM_TDSP_Tree(_dest_node_ID, m_mmdta->m_bus_transit_graph, m_mmdue -> m_total_loading_inter);
    //             _tdsp_tree->initialize();
    //             _tdsp_tree->update_tree(m_mmdue -> m_transitlink_cost_map);
    //             m_tdsp_tree_map_bus.insert(std::pair<TInt, MNM_TDSP_Tree*>(_dest_node_ID, _tdsp_tree));
    //             _tdsp_tree = nullptr;
    //             IAssert(m_tdsp_tree_map_bus.find(_dest_node_ID) -> second != nullptr);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_lowest_cost_path()
    // ******************************************************
    // {
    //     // get lowest cost path departing at start_interval
    //     int start_interval, o_node_ID, d_node_ID;

    //     for (int assign_interval = 0; assign_interval < m_mmdue -> m_total_assign_inter; ++assign_interval) {
    //         start_interval = assign_interval * m_mmdue -> m_mmdta_config->get_int("assign_frq");
    //         for (auto _o_it : m_mmdue -> m_passenger_demand) {
    //             o_node_ID = _o_it.first;
    //             for (auto _d_it : _o_it.second) {
    //                 d_node_ID = _d_it.first;

    //                 IAssert(start_interval < m_mmdue -> m_total_assign_inter * m_mmdue -> m_mmdta_config->get_int("assign_frq"));
    //                 IAssert(m_mmdue -> m_passenger_demand.find(o_node_ID) != m_mmdue -> m_passenger_demand.end() &&
    //                         m_mmdue -> m_passenger_demand.find(o_node_ID) -> second.find(d_node_ID) != m_mmdue -> m_passenger_demand.find(o_node_ID) -> second.end());

    //                 MNM_Passenger_Path_Base *_p_path;
    //                 MNM_Path *_path;
    //                 TInt _mode;
    //                 TFlt _cost;
    //                 int _best_time_col, _best_assign_col, _num_col;
    //                 bool _exist;
    //                 MNM_Passenger_Pathset *_path_set_driving;
    //                 MNM_Passenger_Pathset *_path_set_bus;
    //                 MNM_Passenger_Pathset *_path_set_pnr;
    //                 std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int> _path_result;

    //                 _path_set_driving = nullptr;
    //                 _path_set_bus = nullptr;
    //                 _path_set_pnr = nullptr;
    //                 if (std::find(m_mmdue -> m_mode_vec.begin(), m_mmdue -> m_mode_vec.end(), driving) != m_mmdue -> m_mode_vec.end() &&
    //                     m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.find(driving) -> second) {
    //                     _path_set_driving = m_mmdue -> m_passenger_path_table -> find(o_node_ID) -> second -> find(d_node_ID) -> second -> find(driving) -> second;
    //                 }
    //                 if (std::find(m_mmdue -> m_mode_vec.begin(), m_mmdue -> m_mode_vec.end(), transit) != m_mmdue -> m_mode_vec.end() &&
    //                     m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.find(transit) -> second) {
    //                     _path_set_bus = m_mmdue -> m_passenger_path_table -> find(o_node_ID) -> second -> find(d_node_ID) -> second -> find(transit) -> second;
    //                 }
    //                 if (std::find(m_mmdue -> m_mode_vec.begin(), m_mmdue -> m_mode_vec.end(), pnr) != m_mmdue -> m_mode_vec.end() &&
    //                     m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.find(pnr) -> second) {
    //                     _path_set_pnr = m_mmdue -> m_passenger_path_table -> find(o_node_ID) -> second -> find(d_node_ID) -> second -> find(pnr) -> second;
    //                 }

    //                 _path_result = m_mmdue -> get_best_path_for_single_interval(start_interval, o_node_ID, d_node_ID,
    //                                                                             m_tdsp_tree_map_driving,
    //                                                                             m_tdsp_tree_map_bus,
    //                                                                             m_mmdta);

    //                 _p_path = std::get<0>(_path_result.first);
    //                 _cost = std::get<2>(_path_result.first);
    //                 _mode = _path_result.second;
    //                 _best_time_col = std::get<1>(_path_result.first);
    //                 _best_assign_col = (int)_best_time_col / m_mmdue -> m_mmdta_config->get_int("assign_frq");
    //                 if (_best_assign_col >= m_mmdue -> m_total_assign_inter) _best_assign_col = m_mmdue -> m_total_assign_inter - 1;

    //                 _exist = false;
    //                 _path = nullptr;
    //                 if (_mode == driving && _path_set_driving != nullptr) {
    //                     _exist = _path_set_driving -> is_in(_p_path);
    //                     _path = dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) -> m_path;
    //                     _num_col = (int) _path -> m_node_vec.size();
    //                 }
    //                 else if (_mode == transit && _path_set_bus != nullptr) {
    //                     _exist = _path_set_bus -> is_in(_p_path);
    //                     _path = dynamic_cast<MNM_Passenger_Path_Bus*>(_p_path) -> m_path;
    //                     _num_col = (int) _path -> m_link_vec.size();
    //                 }
    //                 else if (_mode == pnr && _path_set_pnr != nullptr) {
    //                     _exist = _path_set_pnr -> is_in(_p_path);
    //                     _path = dynamic_cast<MNM_Passenger_Path_PnR*>(_p_path) -> m_path;
    //                     _num_col = std::max(int(dynamic_cast<MNM_PnR_Path*>(_path) -> m_driving_path -> m_node_vec.size()),
    //                                         int(dynamic_cast<MNM_PnR_Path*>(_path) -> m_transit_path -> m_link_vec.size()));
    //                 }
    //                 else {
    //                     printf("Mode not implemented!\n");
    //                     exit(-1);
    //                 }
    //                 IAssert(_path != nullptr);

    //                 // row: _exist, _mode, driving path node vec, transit path link vec
    //                 int result_prt[4*_num_col];

    //                 for (int i = 0; i < _num_col; ++i) {
    //                     if (i == 0) {
    //                         result_prt[i + _num_col * 0] = (int) _exist;
    //                         result_prt[i + _num_col * 1] = (int) _mode;
    //                     }
    //                     else {
    //                         result_prt[i + _num_col * 0] = -1;
    //                         result_prt[i + _num_col * 1] = -1;
    //                     }


    //                     if (_mode == driving) {
    //                         result_prt[i + _num_col * 2] = _path -> m_node_vec[i];
    //                         result_prt[i + _num_col * 3] = -1;
    //                     }
    //                     else if (_mode == transit) {
    //                         result_prt[i + _num_col * 2] = -1;
    //                         result_prt[i + _num_col * 3] = _path -> m_link_vec[i];
    //                     }
    //                     else if (_mode == pnr) {
    //                         if (i < int(dynamic_cast<MNM_PnR_Path*>(_path) -> m_driving_path -> m_node_vec.size())) {
    //                             result_prt[i + _num_col * 2] = dynamic_cast<MNM_PnR_Path*>(_path) -> m_driving_path -> m_node_vec[i];
    //                         }
    //                         else {
    //                             result_prt[i + _num_col * 2] = -1;
    //                         }

    //                         if (i < int(dynamic_cast<MNM_PnR_Path*>(_path) -> m_transit_path -> m_link_vec.size())) {
    //                             result_prt[i + _num_col * 3] = dynamic_cast<MNM_PnR_Path*>(_path) -> m_transit_path -> m_link_vec[i];
    //                         }
    //                         else {
    //                             result_prt[i + _num_col * 3] = -1;
    //                         }
    //                     }
    //                 }
    //                 printf("tdsp for OD pair: %d -- %d, at interval %d\n", o_node_ID, d_node_ID, start_interval);
    //                 printf("existing: %d\n", result_prt[0]);
    //                 printf("mode: %d\n", result_prt[_num_col]);

    //                 std::string _str = "driving path node vec: ";
    //                 for (int i = 0; i < _num_col; ++i) {
    //                     _str += std::to_string(result_prt[i+_num_col*2]) + " ";
    //                 }
    //                 _str += "\n";
    //                 std::cout << _str << std::endl;

    //                 _str = "bustransit path link vec: ";
    //                 for (int i = 0; i < _num_col; ++i) {
    //                     _str += std::to_string(result_prt[i+_num_col*3]) + " ";
    //                 }
    //                 _str += "\n";
    //                 std::cout << _str << std::endl;

    //             }
    //         }
    //     }
    // }
    
    // ******************************************************
    // Mmdta_Api::build_link_cost_map()
    // ******************************************************
    // {
    //     m_mmdue -> build_link_cost_map(m_mmdta);
    // }

    // ******************************************************
    // Mmdta_Api::build_link_cost_map_snapshot()
    // Mmdta_Api::update_snapshot_route_table()
    // Mmdta_Api::get_lowest_cost_path_snapshot()
    // ******************************************************
    // {   
    //     TInt _dest_node_ID;
    //     int start_interval;

    //     MNM_Dlink_Multiclass *_link;
    //     MNM_Transit_Link *_transitlink;

    //     // <d_node_ID, <node_ID, out_link_ID>>
    //     std::unordered_map<TInt, TInt> _shortest_path_tree_driving;
    //     std::unordered_map<TInt, TInt> _shortest_path_tree_bustransit;
    //     std::unordered_map<TInt, TInt> _shortest_path_tree_pnr;

    //     MNM_Parking_Lot* _best_mid_parkinglot;
    //     MNM_Destination_Multimodal *_dest;
    //     TInt _mid_dest_node_ID;
    //     TFlt _cur_best_path_tt, _path_tt, _tot_dmd_one_mode, _tmp_cost;

    //     // std::unordered_map<TInt, TInt> _shortest_path_tree_driving;
    //     // std::unordered_map<TInt, TInt> _shortest_path_tree_bustransit;
    //     // std::unordered_map<TInt, TInt> _shortest_path_tree_pnr;

    //     MNM_Path *_path;
    //     MNM_Path *_driving_path;
    //     MNM_Path *_transit_path;
    //     MNM_PnR_Path *_pnr_path;

    //     // driving
    //     MNM_Passenger_Path_Driving *_p_path_driving;
    //     // bus
    //     MNM_Passenger_Path_Bus *_p_path_bus;
    //     // pnr
    //     MNM_Passenger_Path_PnR *_p_path_pnr;

    //     MNM_Passenger_Path_Base *_p_path;

    //     TInt _mode;
    //     TFlt _cost;
    //     int _best_time_col, _best_assign_col, _num_col;
    //     bool _exist;
    //     MNM_Passenger_Pathset *_path_set_driving;
    //     MNM_Passenger_Pathset *_path_set_bus;
    //     MNM_Passenger_Pathset *_path_set_pnr;

        
    //     for (int _assign_interval = 0; _assign_interval < m_mmdue -> m_total_assign_inter; ++_assign_interval) {
    //         start_interval = _assign_interval * m_mmdue -> m_mmdta_config->get_int("assign_frq");
            
    //         // ******************************************************
    //         // Mmdta_Api::build_link_cost_map_snapshot()
    //         // ******************************************************
    //         m_driving_link_cost_map_snapshot.clear();
    //         m_bustransit_link_cost_map_snapshot.clear();
            
    //         std::cout << "********************** build_link_cost_map_snapshot interval " << start_interval << " **********************\n";
    //         for (auto _link_it : m_mmdta->m_link_factory->m_link_map) {
    //             _link = dynamic_cast<MNM_Dlink_Multiclass*>(_link_it.second);
    //             m_driving_link_cost_map_snapshot.insert(std::pair<TInt, TFlt>(_link_it.first, MNM_DTA_GRADIENT::get_travel_time_car(_link, TFlt(start_interval), m_mmdue -> m_unit_time)));
    //             // std::cout << "interval: " << i << ", link: " << _link_it.first << ", tt: " << m_link_cost_map[_link_it.first] << "\n";
    //         }
    //         for (auto _link_it : m_mmdta->m_transitlink_factory->m_transit_link_map) {
    //             _transitlink = _link_it.second;
    //             if (_transitlink -> m_link_type == MNM_TYPE_WALKING_MULTIMODAL) {
    //                 m_bustransit_link_cost_map_snapshot.insert(std::pair<TInt, TFlt>(_link_it.first, MNM_DTA_GRADIENT::get_travel_time_walking(dynamic_cast<MNM_Walking_Link*>(_transitlink), TFlt(start_interval), m_mmdue -> m_unit_time)));
    //             }
    //             else if (_transitlink -> m_link_type == MNM_TYPE_BUS_MULTIMODAL) {
    //                 m_bustransit_link_cost_map_snapshot.insert(std::pair<TInt, TFlt>(_link_it.first, MNM_DTA_GRADIENT::get_travel_time_bus(dynamic_cast<MNM_Bus_Link*>(_transitlink), TFlt(start_interval), m_mmdue -> m_unit_time)));
    //             }
    //             else {
    //                 printf("Wrong transit link type!\n");
    //                 exit(-1);
    //             }
    //             // std::cout << "interval: " << i << ", link: " << _link_it.first << ", tt: " << m_transitlink_cost_map[_link_it.first] << "\n";
    //         }
            
    //         // ******************************************************
    //         // Mmdta_Api::update_snapshot_route_table()
    //         // ******************************************************
    //         for (auto _d_it : m_driving_table_snapshot) {
    //             _d_it.second.clear();
    //         }
    //         m_driving_table_snapshot.clear();
    //         for (auto _d_it : m_bustransit_table_snapshot) {
    //             _d_it.second.clear();
    //         }
    //         m_bustransit_table_snapshot.clear();

    //         _shortest_path_tree_driving.clear();
    //         _shortest_path_tree_bustransit.clear();
    //         _shortest_path_tree_pnr.clear();

    //         for (auto _d_it : m_mmdta -> m_od_factory -> m_destination_map) {
    //             _dest_node_ID = _d_it.second->m_dest_node->m_node_ID;

    //             _shortest_path_tree_driving = std::unordered_map<TInt, TInt>();
    //             MNM_Shortest_Path::all_to_one_FIFO(_dest_node_ID, m_mmdta -> m_graph, m_driving_link_cost_map_snapshot,
    //                                             _shortest_path_tree_driving);
    //             m_driving_table_snapshot.insert(std::pair<TInt, std::unordered_map<TInt, TInt>>(_dest_node_ID, _shortest_path_tree_driving));

    //             if (m_mmdta -> m_bus_transit_graph -> IsNode(_dest_node_ID)) {
    //                 _shortest_path_tree_bustransit = std::unordered_map<TInt, TInt>();
    //                 MNM_Shortest_Path::all_to_one_FIFO(_dest_node_ID, m_mmdta -> m_bus_transit_graph, m_bustransit_link_cost_map_snapshot,
    //                                                 _shortest_path_tree_bustransit);
    //                 m_bustransit_table_snapshot.insert(std::pair<TInt, std::unordered_map<TInt, TInt>>(_dest_node_ID, _shortest_path_tree_bustransit));
    //             }
    //         }

    //         // ******************************************************
    //         // Mmdta_Api::get_lowest_cost_path_snapshot()
    //         // ******************************************************
    //         // get lowest cost path departing at start_interval snapshot
    //         // int _assign_inter = (int)start_interval / m_mmdue -> m_mmdta_config->get_int("assign_frq");
    //         // if (_assign_inter >= m_mmdue -> m_total_assign_inter) _assign_inter = m_mmdue -> m_total_assign_inter - 1;

    //         IAssert(start_interval < m_mmdue -> m_total_assign_inter * m_mmdue -> m_mmdta_config->get_int("assign_frq"));
    //         for (auto _o_it : m_mmdue -> m_passenger_demand) {
    //             int o_node_ID = _o_it.first;
    //             for (auto _d_it : _o_it.second) {
    //                 int d_node_ID = _d_it.first;

    //                 _p_path = nullptr;
    //                 _cost = TFlt(std::numeric_limits<double>::max());

    //                 _path_set_driving = nullptr;
    //                 _path_set_bus = nullptr;
    //                 _path_set_pnr = nullptr;
    //                 if (std::find(m_mmdue -> m_mode_vec.begin(), m_mmdue -> m_mode_vec.end(), driving) != m_mmdue -> m_mode_vec.end() &&
    //                     m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.find(driving) -> second) {
    //                     _path_set_driving = m_mmdue -> m_passenger_path_table -> find(o_node_ID) -> second -> find(d_node_ID) -> second -> find(driving) -> second;
    //                 }
    //                 if (std::find(m_mmdue -> m_mode_vec.begin(), m_mmdue -> m_mode_vec.end(), transit) != m_mmdue -> m_mode_vec.end() &&
    //                     m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.find(transit) -> second) {
    //                     _path_set_bus = m_mmdue -> m_passenger_path_table -> find(o_node_ID) -> second -> find(d_node_ID) -> second -> find(transit) -> second;
    //                 }
    //                 if (std::find(m_mmdue -> m_mode_vec.begin(), m_mmdue -> m_mode_vec.end(), pnr) != m_mmdue -> m_mode_vec.end() &&
    //                     m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.find(pnr) -> second) {
    //                     _path_set_pnr = m_mmdue -> m_passenger_path_table -> find(o_node_ID) -> second -> find(d_node_ID) -> second -> find(pnr) -> second;
    //                 }
                    
    //                 _dest = dynamic_cast<MNM_Destination_Multimodal*>(((MNM_DMDND*)m_mmdta -> m_node_factory -> get_node(d_node_ID)) -> m_dest);

    //                 _shortest_path_tree_driving = m_driving_table_snapshot.find(d_node_ID) -> second;
    //                 if (m_mmdta -> m_bus_transit_graph -> IsNode(d_node_ID)) {
    //                     _shortest_path_tree_bustransit = m_bustransit_table_snapshot.find(d_node_ID) -> second;
    //                 }

    //                 // driving
    //                 if (m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.find(driving) != m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.end() &&
    //                     m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.find(driving) -> second) {
    //                     _path = MNM::extract_path(o_node_ID, d_node_ID, _shortest_path_tree_driving, m_mmdta -> m_graph);
    //                     _path -> eliminate_cycles();
    //                     IAssert(_path != nullptr);
    //                     _p_path_driving = new MNM_Passenger_Path_Driving(driving, _path, m_mmdue -> m_vot, m_mmdue -> m_early_penalty,
    //                                                                     m_mmdue -> m_late_penalty, m_mmdue -> m_target_time,
    //                                                                     1, m_mmdue -> m_carpool_cost_multiplier, 0.0,
    //                                                                     _dest -> m_parking_lot, m_mmdue -> m_parking_lot_to_destination_walking_time);
    //                     _tmp_cost = _p_path_driving -> get_travel_cost(TFlt(start_interval), m_mmdta);
    //                     _tot_dmd_one_mode = m_mmdue -> compute_total_passenger_demand_for_one_mode(driving, o_node_ID, d_node_ID, _assign_interval);
    //                     _tmp_cost = m_mmdue -> get_disutility(driving, _tmp_cost, _tot_dmd_one_mode);

    //                     if (_cost > _tmp_cost) {
    //                         _mode = driving;
    //                         _cost = _tmp_cost;
    //                         delete _p_path;
    //                         _p_path = _p_path_driving;
    //                     }
    //                     _path = nullptr;
    //                     IAssert(_p_path_driving -> m_path != nullptr);
    //                 }

    //                 // bus transit
    //                 if (m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.find(transit) != m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.end() &&
    //                     m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.find(transit) -> second) {
    //                     _path = MNM::extract_path(o_node_ID, d_node_ID, _shortest_path_tree_bustransit, m_mmdta -> m_bus_transit_graph);
    //                     _path -> eliminate_cycles();
    //                     IAssert(_path != nullptr);
    //                     _p_path_bus = new MNM_Passenger_Path_Bus(transit, _path, m_mmdue -> m_vot, m_mmdue -> m_early_penalty,
    //                                                             m_mmdue -> m_late_penalty, m_mmdue -> m_target_time, m_mmdue -> m_bus_fare, m_mmdue -> m_bus_inconvenience);
    //                     _tmp_cost = _p_path_bus -> get_travel_cost(TFlt(start_interval), m_mmdta);
    //                     _tot_dmd_one_mode = m_mmdue -> compute_total_passenger_demand_for_one_mode(transit, o_node_ID, d_node_ID, _assign_interval);
    //                     _tmp_cost = m_mmdue -> get_disutility(transit, _tmp_cost, _tot_dmd_one_mode);

    //                     if (_cost > _tmp_cost) {
    //                         _mode = transit;
    //                         _cost = _tmp_cost;
    //                         delete _p_path;
    //                         _p_path = _p_path_bus;
    //                     }

    //                     _path = nullptr;
    //                     IAssert(_p_path_bus -> m_path != nullptr);
    //                 }

    //                 // pnr
    //                 if (m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.find(pnr) != m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.end() &&
    //                     m_mmdue -> m_od_mode_connectivity.find(o_node_ID) -> second.find(d_node_ID) -> second.find(pnr) -> second) {
    //                     _cur_best_path_tt = DBL_MAX;
    //                     _best_mid_parkinglot = nullptr;
    //                     _pnr_path = nullptr;
    //                     for (auto _parkinglot : _dest -> m_connected_pnr_parkinglot_vec) {
    //                         _mid_dest_node_ID = _parkinglot -> m_dest_node -> m_node_ID;
    //                         _path_tt = 0.;
    //                         _shortest_path_tree_pnr = m_driving_table_snapshot.find(_mid_dest_node_ID) -> second;
    //                         _driving_path = MNM::extract_path(o_node_ID, _mid_dest_node_ID, _shortest_path_tree_pnr, m_mmdta -> m_graph);
    //                         _driving_path -> eliminate_cycles();
    //                         IAssert(_driving_path != nullptr);
    //                         _path_tt += MNM::get_path_tt_snapshot(_driving_path, m_driving_link_cost_map_snapshot);

    //                         _transit_path = MNM::extract_path(_mid_dest_node_ID, d_node_ID, _shortest_path_tree_bustransit, m_mmdta -> m_bus_transit_graph);
    //                         _transit_path -> eliminate_cycles();
    //                         IAssert(_transit_path != nullptr);
    //                         _path_tt += MNM::get_path_tt_snapshot(_transit_path, m_bustransit_link_cost_map_snapshot);

    //                         if (_cur_best_path_tt > _path_tt) {
    //                             _cur_best_path_tt = _path_tt;
    //                             _best_mid_parkinglot = _parkinglot;
    //                             delete _pnr_path;
    //                             _pnr_path = new MNM_PnR_Path(0, _best_mid_parkinglot -> m_ID, _mid_dest_node_ID, _driving_path, _transit_path);
    //                             _driving_path = nullptr;
    //                             _transit_path = nullptr;
    //                         }
    //                         else {
    //                             delete _driving_path;
    //                             delete _transit_path;
    //                         }
    //                     }
    //                     IAssert(_pnr_path != nullptr && _best_mid_parkinglot != nullptr);
    //                     _p_path_pnr = new MNM_Passenger_Path_PnR(pnr, _pnr_path, m_mmdue -> m_vot, m_mmdue -> m_early_penalty,
    //                                                             m_mmdue -> m_late_penalty, m_mmdue -> m_target_time,
    //                                                             0.0, _best_mid_parkinglot, m_mmdue -> m_bus_fare,
    //                                                             m_mmdue -> m_pnr_inconvenience);
    //                     _tmp_cost = _p_path_pnr -> get_travel_cost(TFlt(start_interval), m_mmdta);
    //                     _tot_dmd_one_mode = m_mmdue -> compute_total_passenger_demand_for_one_mode(pnr, o_node_ID, d_node_ID, _assign_interval);
    //                     _tmp_cost = m_mmdue -> get_disutility(pnr, _tmp_cost, _tot_dmd_one_mode);

    //                     if (_cost > _tmp_cost) {
    //                         _mode = pnr;
    //                         _cost = _tmp_cost;
    //                         delete _p_path;
    //                         _p_path = _p_path_pnr;
    //                     }

    //                     _pnr_path = nullptr;
    //                     IAssert(_p_path_pnr -> m_path != nullptr);
    //                 }

    //                 IAssert(_p_path != nullptr);

    //                 _best_time_col = start_interval;
    //                 _best_assign_col = (int)_best_time_col / m_mmdue -> m_mmdta_config->get_int("assign_frq");
    //                 if (_best_assign_col >= m_mmdue -> m_total_assign_inter) _best_assign_col = m_mmdue -> m_total_assign_inter - 1;

    //                 _exist = false;
    //                 _path = nullptr;
    //                 if (_mode == driving && _path_set_driving != nullptr) {
    //                     _exist = _path_set_driving -> is_in(_p_path);
    //                     _path = dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) -> m_path;
    //                     _num_col = (int) _path -> m_node_vec.size();
    //                 }
    //                 else if (_mode == transit && _path_set_bus != nullptr) {
    //                     _exist = _path_set_bus -> is_in(_p_path);
    //                     _path = dynamic_cast<MNM_Passenger_Path_Bus*>(_p_path) -> m_path;
    //                     _num_col = (int) _path -> m_link_vec.size();
    //                 }
    //                 else if (_mode == pnr && _path_set_pnr != nullptr) {
    //                     _exist = _path_set_pnr -> is_in(_p_path);
    //                     _path = dynamic_cast<MNM_Passenger_Path_PnR*>(_p_path) -> m_path;
    //                     _num_col = std::max(int(dynamic_cast<MNM_PnR_Path*>(_path) -> m_driving_path -> m_node_vec.size()),
    //                                         int(dynamic_cast<MNM_PnR_Path*>(_path) -> m_transit_path -> m_link_vec.size()));
    //                 }
    //                 else {
    //                     printf("Mode not implemented!\n");
    //                     exit(-1);
    //                 }
    //                 IAssert(_path != nullptr);

    //                 // int new_shape [2] = {4,  _num_col}; // row: _exist, _mode, driving path node vec, transit path link vec
    //                 int result_prt[4*_num_col];

    //                 for (int i = 0; i < _num_col; ++i) {
    //                     if (i == 0) {
    //                         result_prt[i + _num_col * 0] = (int) _exist;
    //                         result_prt[i + _num_col * 1] = (int) _mode;
    //                     }
    //                     else {
    //                         result_prt[i + _num_col * 0] = -1;
    //                         result_prt[i + _num_col * 1] = -1;
    //                     }


    //                     if (_mode == driving) {
    //                         result_prt[i + _num_col * 2] = _path -> m_node_vec[i];
    //                         result_prt[i + _num_col * 3] = -1;
    //                     }
    //                     else if (_mode == transit) {
    //                         result_prt[i + _num_col * 2] = -1;
    //                         result_prt[i + _num_col * 3] = _path -> m_link_vec[i];
    //                     }
    //                     else if (_mode == pnr) {
    //                         if (i < int(dynamic_cast<MNM_PnR_Path*>(_path) -> m_driving_path -> m_node_vec.size())) {
    //                             result_prt[i + _num_col * 2] = dynamic_cast<MNM_PnR_Path*>(_path) -> m_driving_path -> m_node_vec[i];
    //                         }
    //                         else {
    //                             result_prt[i + _num_col * 2] = -1;
    //                         }

    //                         if (i < int(dynamic_cast<MNM_PnR_Path*>(_path) -> m_transit_path -> m_link_vec.size())) {
    //                             result_prt[i + _num_col * 3] = dynamic_cast<MNM_PnR_Path*>(_path) -> m_transit_path -> m_link_vec[i];
    //                         }
    //                         else {
    //                             result_prt[i + _num_col * 3] = -1;
    //                         }
    //                     }
    //                 }

    //                 printf("tdsp for OD pair: %d -- %d, at interval %d\n", o_node_ID, d_node_ID, start_interval);
    //                 printf("existing: %d\n", result_prt[0]);
    //                 printf("mode: %d\n", result_prt[_num_col]);

    //                 std::string _str = "driving path node vec: ";
    //                 for (int i = 0; i < _num_col; ++i) {
    //                     _str += std::to_string(result_prt[i+_num_col*2]) + " ";
    //                 }
    //                 _str += "\n";
    //                 std::cout << _str << std::endl;

    //                 _str = "bustransit path link vec: ";
    //                 for (int i = 0; i < _num_col; ++i) {
    //                     _str += std::to_string(result_prt[i+_num_col*3]) + " ";
    //                 }
    //                 _str += "\n";
    //                 std::cout << _str << std::endl;

    //             }
    //         }
            

    //     }

        
    // }

    // ******************************************************
    // Mmdta_Api::get_car_link_tt()
    // ******************************************************
    {
        // int l = m_mmdta -> m_current_loading_interval();
        int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
        int new_shape [2] = { (int) m_link_vec_driving.size(), l};
        
        double result_prt[l*m_link_vec_driving.size()];
        std::vector<double> start_prt = std::vector<double>();
        for (int t = 0; t < l; ++t){
            start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
            if (start_prt[t] > m_mmdta -> m_current_loading_interval()){
                throw std::runtime_error("Error, Mmdta_Api::get_car_link_tt, loaded data not enough");
            }
            for (size_t i = 0; i < m_link_vec_driving.size(); ++i){
                double _tmp = MNM_DTA_GRADIENT::get_travel_time_car(m_link_vec_driving[i], TFlt(start_prt[t]), m_mmdta->m_unit_time)();
                // if (_tmp * m_mmdta -> m_unit_time > 20 * (m_link_vec_driving[i] -> m_length / m_link_vec[i] -> m_ffs_car)){
                //     _tmp = 20 * m_link_vec_driving[i] -> m_length / m_link_vec_driving[i] -> m_ffs_car / m_mmdta -> m_unit_time;
                // }
                result_prt[i * l + t] = _tmp * m_mmdta -> m_unit_time;  // seconds
                printf("link: i %d, time: t %d, tt: %f\n", (int)i, t, result_prt[i * l + t]);
            }
        }
    }

    // ******************************************************
    // Mmdta_Api::get_truck_link_tt()
    // ******************************************************
    //    {
    //        int l = m_mmdta -> m_current_loading_interval();
    //        int new_shape [2] = { (int) m_link_vec_driving.size(), l};
    //
    //        double result_prt[l*m_link_vec_driving.size()];
    //        std::vector<double> start_prt = std::vector<double>();
    //        for (int t = 0; t < l; ++t){
    //            start_prt.push_back((double)t);
    //            if (start_prt[t] > m_mmdta -> m_current_loading_interval()){
    //                throw std::runtime_error("Error, Mmdta_Api::get_truck_link_tt, loaded data not enough");
    //            }
    //            for (size_t i = 0; i < m_link_vec_driving.size(); ++i){
    //                double _tmp = MNM_DTA_GRADIENT::get_travel_time_truck(m_link_vec_driving[i], TFlt(start_prt[t]), m_mmdta -> m_unit_time)();
    //                // if (_tmp * 5 > 20 * (m_link_vec_driving[i] -> m_length / m_link_vec_driving[i] -> m_ffs_truck)){
    //                //     _tmp = 20 * m_link_vec_driving[i] -> m_length / m_link_vec_driving[i] -> m_ffs_truck / m_mmdta -> m_unit_time;
    //                // }
    //                result_prt[i * l + t] = _tmp * m_mmdta -> m_unit_time;  // seconds
    //                printf("link: i %d, time: t %d, tt: %f\n", (int)i, t, result_prt[i * l + t]);
    //            }
    //        }
    //    } 

    // ******************************************************
    // Mmdta_Api::get_bus_link_tt()
    // ******************************************************
    //    {
    //        int l = m_mmdta -> m_current_loading_interval();
    //        int new_shape [2] = { (int) m_link_vec_bus.size(), l};
    //
    //        double result_prt[l*m_link_vec_bus.size()];
    //        std::vector<double> start_prt = std::vector<double>();
    //        for (int t = 0; t < l; ++t){
    //            start_prt.push_back((double)t);
    //            if (start_prt[t] > m_mmdta -> m_current_loading_interval()){
    //                throw std::runtime_error("Error, Mmdta_Api::get_bus_link_tt, loaded data not enough");
    //            }
    //            for (size_t i = 0; i < m_link_vec_bus.size(); ++i){
    //                double _tmp = MNM_DTA_GRADIENT::get_travel_time_bus(m_link_vec_bus[i], TFlt(start_prt[t]), m_mmdta -> m_unit_time)();
    //                result_prt[i * l + t] = _tmp * m_mmdta -> m_unit_time;  // seconds
    //                printf("link: i %d, time: t %d, tt: %f\n", (int)i, t, result_prt[i * l + t]);
    //            }
    //        }
    //    }

    // ******************************************************
    // Mmdta_Api::get_passenger_walking_link_tt()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     int new_shape [2] = {(int) m_link_vec_walking.size(), l};

    //     double result_prt[l*m_link_vec_walking.size()];
    //     std::vector<double> start_prt = std::vector<double>();
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back((double)t);
    //         if (start_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_passenger_walking_link_tt, loaded data not enough");
    //         }
    //         for (size_t i = 0; i < m_link_vec_walking.size(); ++i){
    //             double _tmp = MNM_DTA_GRADIENT::get_travel_time_walking(m_link_vec_walking[i], TFlt(start_prt[t]), m_mmdta -> m_unit_time)();
    //             result_prt[i * l + t] = _tmp * m_mmdta -> m_unit_time;  // seconds
    //             printf("link: i %d, time: t %d, tt: %f\n", (int)i, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_link_car_inflow()
    // ******************************************************
    //    {
    //        int l = m_mmdta -> m_current_loading_interval()-1;
    //        double result_prt[l*m_link_vec_driving.size()];
    //        std::vector<int> start_prt = std::vector<int>();
    //        std::vector<int> end_prt = std::vector<int>();
    //        for (int t = 0; t < l; ++t){
    //            start_prt.push_back(t);
    //            end_prt.push_back(t+1);
    //            if (end_prt[t] < start_prt[t]){
    //                throw std::runtime_error("Error, Mmdta_Api::get_link_car_inflow, end time smaller than start time");
    //            }
    //            if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //                throw std::runtime_error("Error, Mmdta_Api::get_link_car_inflow, loaded data not enough");
    //            }
    //            for (size_t i = 0; i < m_link_vec_driving.size(); ++i){
    //                result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_inflow_car(m_link_vec_driving[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
    //                printf("link: i %d, time: t %d, flow: %f\n", (int)i, t, result_prt[i * l + t]);
    //            }
    //        }
    //    }

    // ******************************************************
    // Mmdta_Api::get_link_truck_inflow()()
    // ******************************************************
    //    {
    //        int l = m_mmdta -> m_current_loading_interval()-1;
    //        double result_prt[l*m_link_vec_driving.size()];
    //        std::vector<int> start_prt = std::vector<int>();
    //        std::vector<int> end_prt = std::vector<int>();
    //        for (int t = 0; t < l; ++t){
    //            start_prt.push_back(t);
    //            end_prt.push_back(t+1);
    //            if (end_prt[t] < start_prt[t]){
    //                throw std::runtime_error("Error, Mmdta_Api::get_link_truck_inflow, end time smaller than start time");
    //            }
    //            if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //                throw std::runtime_error("Error, Mmdta_Api::get_link_truck_inflow, loaded data not enough");
    //            }
    //            for (size_t i = 0; i < m_link_vec_driving.size(); ++i){
    //                result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_inflow_truck(m_link_vec_driving[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
    //                printf("link: i %d, time: t %d, flow: %f\n", (int)i, t, result_prt[i * l + t]);
    //            }
    //        }
    //    } 

    // ******************************************************
    // Mmdta_Api::get_link_bus_inflow()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval()-1;
    //     double result_prt[l*m_link_vec_bus.size()];
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         if (end_prt[t] < start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_bus_inflow, end time smaller than start time");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_bus_inflow, loaded data not enough");
    //         }
    //         for (size_t i = 0; i < m_link_vec_bus.size(); ++i){
    //             result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_inflow_bus(m_link_vec_bus[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
    //             printf("link: i %d, time: t %d, flow: %f\n", (int)i, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_busstop_bus_inflow()
    // ******************************************************
    //    {
    //        int l = m_mmdta -> m_current_loading_interval()-1;
    //        double result_prt[l*m_link_vec_bus.size()];
    //        std::vector<int> start_prt = std::vector<int>();
    //        std::vector<int> end_prt = std::vector<int>();
    //        for (int t = 0; t < l; ++t){
    //            start_prt.push_back(t);
    //            end_prt.push_back(t+1);
    //            if (end_prt[t] < start_prt[t]){
    //                throw std::runtime_error("Error, Mmdta_Api::get_busstop_bus_inflow, end time smaller than start time");
    //            }
    //            if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //                throw std::runtime_error("Error, Mmdta_Api::get_busstop_bus_inflow, loaded data not enough");
    //            }
    //            for (size_t i = 0; i < m_link_vec_bus.size(); ++i){
    //                result_prt[i * l + t] = MNM_DTA_GRADIENT::get_busstop_inflow_bus(m_link_vec_bus[i] -> m_to_busstop, TFlt(start_prt[t]), TFlt(end_prt[t]))();
    //                printf("link: i %d, time: t %d, flow: %f\n", (int)i, t, result_prt[i * l + t]);
    //            }
    //        }
    //    }

    // ******************************************************
    // Mmdta_Api::get_link_bus_passenger_inflow()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval()-1;
    //     double result_prt[l*m_link_vec_bus.size()];
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         if (end_prt[t] < start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_bus_passenger_inflow, end time smaller than start time");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_bus_passenger_inflow, loaded data not enough");
    //         }
    //         for (size_t i = 0; i < m_link_vec_bus.size(); ++i){
    //             result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_inflow_passenger(m_link_vec_bus[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
    //             printf("link: i %d, time: t %d, flow: %f\n", (int)i, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_link_walking_passenger_inflow()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval()-1;
    //     double result_prt[l*m_link_vec_walking.size()];
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         if (end_prt[t] < start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_walking_passenger_inflow, end time smaller than start time");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_walking_passenger_inflow, loaded data not enough");
    //         }
    //         for (size_t i = 0; i < m_link_vec_walking.size(); ++i){
    //             result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_inflow_passenger(m_link_vec_walking[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
    //             printf("link: i %d, time: t %d, flow: %f\n", (int)i, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_car_dar_matrix_driving()
    // ******************************************************
    {
        int l = m_mmdta -> m_current_loading_interval()-1;
        std::vector<int> start_prt = std::vector<int>();
        std::vector<int> end_prt = std::vector<int>();
        std::vector<dar_record*> _record = std::vector<dar_record*>();
        // for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
        //   m_link_vec_driving[i] -> m_N_in_tree -> print_out();
        // }
        for (int t = 0; t < l; ++t){
            start_prt.push_back(t);
            end_prt.push_back(t+1);
            // printf("Current processing time: %d\n", t);
            if (end_prt[t] < start_prt[t]){
                throw std::runtime_error("Error, Mmdta_Api::get_car_dar_matrix_driving, end time smaller than start time");
            }
            if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
                throw std::runtime_error("Error, Mmdta_Api::get_car_dar_matrix_driving, loaded data not enough");
            }
            for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
                MNM_DTA_GRADIENT::add_dar_records_car(_record, m_link_vec_driving[i], m_path_set_driving, TFlt(start_prt[t]), TFlt(end_prt[t]));
            }
        }
        // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
        // path_ID, assign_time, link_ID, start_int, flow
        double result_prt[int(_record.size())*5];
        // int new_shape [2] = { (int) _record.size(), 5};
        // auto result = py::array_t<double>(new_shape);
        // auto result_buf = result.request();
        // double *result_prt = (double *) result_buf.ptr;
        dar_record* tmp_record;
        for (size_t i = 0; i < _record.size(); ++i){
            tmp_record = _record[i];
            result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
            // the count of 1 min interval
            result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
            result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
            // the count of unit time interval (5s)
            result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
            result_prt[i * 5 + 4] = tmp_record -> flow();
            printf("path ID: %f, departure assign interval (1 min): %f, link ID: %f, time interval (5 s): %f, flow: %f\n",
                   result_prt[i * 5 + 0], result_prt[i * 5 + 1], result_prt[i * 5 + 2], result_prt[i * 5 + 3], result_prt[i * 5 + 4]);
        }
        for (size_t i = 0; i < _record.size(); ++i){
            delete _record[i];
        }
        _record.clear();
    }

    // ******************************************************
    // Mmdta_Api::get_truck_dar_matrix_driving()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval()-1;
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     std::vector<dar_record*> _record = std::vector<dar_record*>();
    //     // for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
    //     //   m_link_vec_driving[i] -> m_N_in_tree -> print_out();
    //     // }
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         if (end_prt[t] < start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_truck_dar_matrix_driving, end time smaller than start time");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_truck_dar_matrix_driving, loaded data not enough");
    //         }
    //         for (size_t i = 0; i < m_link_vec_driving.size(); ++i){
    //             MNM_DTA_GRADIENT::add_dar_records_truck(_record, m_link_vec_driving[i], m_path_set_driving, TFlt(start_prt[t]), TFlt(end_prt[t]));
    //         }
    //     }
    //     // path_ID, assign_time, link_ID, start_int, flow
    //     double result_prt[int(_record.size())*5];
    //     // int new_shape [2] = { (int) _record.size(), 5};
    //     // auto result = py::array_t<double>(new_shape);
    //     // auto result_buf = result.request();
    //     // double *result_prt = (double *) result_buf.ptr;
    //     dar_record* tmp_record;
    //     for (size_t i = 0; i < _record.size(); ++i){
    //         tmp_record = _record[i];
    //         result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    //         // the count of 1 min interval
    //         result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    //         result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    //         // the count of unit time interval (5s)
    //         result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    //         result_prt[i * 5 + 4] = tmp_record -> flow();
    //         printf("path ID: %f, departure assign interval (1 min): %f, link ID: %f, time interval (5 s): %f, flow: %f\n",
    //                result_prt[i * 5 + 0], result_prt[i * 5 + 1], result_prt[i * 5 + 2], result_prt[i * 5 + 3], result_prt[i * 5 + 4]);
    //     }
    //     for (size_t i = 0; i < _record.size(); ++i){
    //         delete _record[i];
    //     }
    //     _record.clear();
    // }

    // ******************************************************
    // Mmdta_Api::get_car_dar_matrix_pnr()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval()-1;
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     std::vector<dar_record*> _record = std::vector<dar_record*>();
    //     // for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
    //     //   m_link_vec_driving[i] -> m_N_in_tree -> print_out();
    //     // }
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         // printf("Current processing time: %d\n", t);
    //         if (end_prt[t] < start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_car_dar_matrix_pnr, end time smaller than start time");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_car_dar_matrix_pnr, loaded data not enough");
    //         }
    //         for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
    //             MNM_DTA_GRADIENT::add_dar_records_car(_record, m_link_vec_driving[i], m_path_set_pnr, TFlt(start_prt[t]), TFlt(end_prt[t]));
    //         }
    //     }
    //     // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
    //     // path_ID, assign_time, link_ID, start_int, flow
    //     double result_prt[int(_record.size())*5];
    //     // int new_shape [2] = { (int) _record.size(), 5};
    //     // auto result = py::array_t<double>(new_shape);
    //     // auto result_buf = result.request();
    //     // double *result_prt = (double *) result_buf.ptr;
    //     dar_record* tmp_record;
    //     for (size_t i = 0; i < _record.size(); ++i){
    //         tmp_record = _record[i];
    //         result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    //         // the count of 1 min interval
    //         result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    //         result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    //         // the count of unit time interval (5s)
    //         result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    //         result_prt[i * 5 + 4] = tmp_record -> flow();
    //         printf("path ID: %f, departure assign interval (1 min): %f, link ID: %f, time interval (5 s): %f, flow: %f\n",
    //                result_prt[i * 5 + 0], result_prt[i * 5 + 1], result_prt[i * 5 + 2], result_prt[i * 5 + 3], result_prt[i * 5 + 4]);
    //     }
    //     for (size_t i = 0; i < _record.size(); ++i){
    //         delete _record[i];
    //     }
    //     _record.clear();
    // }

    // ******************************************************
    // Mmdta_Api::get_bus_dar_matrix()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval()-1;
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     std::vector<dar_record*> _record = std::vector<dar_record*>();
    //     // for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
    //     //   m_link_vec_bus[i] -> m_N_in_tree -> print_out();
    //     // }
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
    //             if (end_prt[t] < start_prt[t]){
    //                 throw std::runtime_error("Error, Mmdta_Api::get_bus_dar_matrix, end time smaller than start time");
    //             }
    //             if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //                 throw std::runtime_error("Error, Mmdta_Api::get_bus_dar_matrix, loaded data not enough");
    //             }
    //             MNM_DTA_GRADIENT::add_dar_records_bus(_record, m_link_vec_bus[i], m_path_set_bus, TFlt(start_prt[t]), TFlt(end_prt[t]));
    //         }
    //     }
    //     // path_ID, assign_time, link_ID, start_int, flow
    //     double result_prt[int(_record.size())*5];
    //     // int new_shape [2] = { (int) _record.size(), 5};
    //     // auto result = py::array_t<double>(new_shape);
    //     // auto result_buf = result.request();
    //     // double *result_prt = (double *) result_buf.ptr;
    //     dar_record* tmp_record;
    //     for (size_t i = 0; i < _record.size(); ++i){
    //         tmp_record = _record[i];
    //         result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    //         // the count of 1 min interval
    //         result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    //         result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    //         // the count of unit time interval (5s)
    //         result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    //         result_prt[i * 5 + 4] = tmp_record -> flow();
    //         printf("path ID: %f, departure assign interval (1 min): %f, link ID: %f, time interval (5 s): %f, flow: %f\n",
    //                result_prt[i * 5 + 0], result_prt[i * 5 + 1], result_prt[i * 5 + 2], result_prt[i * 5 + 3], result_prt[i * 5 + 4]);
    //     }
    //     for (size_t i = 0; i < _record.size(); ++i){
    //         delete _record[i];
    //     }
    //     _record.clear();
    // }

    // ******************************************************
    // Mmdta_Api::get_passenger_dar_matrix_bustransit()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval()-1;
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     std::vector<dar_record*> _record = std::vector<dar_record*>();
    //     // for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
    //     //   m_link_vec_bus[i] -> m_N_in_tree -> print_out();
    //     // }
    //     // for (size_t i = 0; i<m_link_vec_walking.size(); ++i){
    //     //   m_link_vec_walking[i] -> m_N_in_tree -> print_out();
    //     // }
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         if (end_prt[t] < start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_bustransit, end time smaller than start time");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_bustransit, loaded data not enough");
    //         }
    //         for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
    //             MNM_DTA_GRADIENT::add_dar_records_passenger(_record, m_link_vec_bus[i], m_path_set_bustransit, TFlt(start_prt[t]), TFlt(end_prt[t]));
    //         }
    //         for (size_t i = 0; i<m_link_vec_walking.size(); ++i){
    //             MNM_DTA_GRADIENT::add_dar_records_passenger(_record, m_link_vec_walking[i], m_path_set_bustransit, TFlt(start_prt[t]), TFlt(end_prt[t]));
    //         }
    //     }
    //     // path_ID, assign_time, link_ID, start_int, flow
    //     double result_prt[int(_record.size())*5];
    //     // int new_shape [2] = { (int) _record.size(), 5};
    //     // auto result = py::array_t<double>(new_shape);
    //     // auto result_buf = result.request();
    //     // double *result_prt = (double *) result_buf.ptr;
    //     dar_record* tmp_record;
    //     for (size_t i = 0; i < _record.size(); ++i){
    //         tmp_record = _record[i];
    //         result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    //         // the count of 1 min interval
    //         result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    //         result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    //         // the count of unit time interval (5s)
    //         result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    //         result_prt[i * 5 + 4] = tmp_record -> flow();
    //         printf("path ID: %f, departure assign interval (1 min): %f, link ID: %f, time interval (5 s): %f, flow: %f\n",
    //                result_prt[i * 5 + 0], result_prt[i * 5 + 1], result_prt[i * 5 + 2], result_prt[i * 5 + 3], result_prt[i * 5 + 4]);
    //     }
    //     for (size_t i = 0; i < _record.size(); ++i){
    //         delete _record[i];
    //     }
    //     _record.clear();
    // }

    // ******************************************************
    // Mmdta_Api::get_passenger_dar_matrix_pnr()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval()-1;
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     std::vector<dar_record*> _record = std::vector<dar_record*>();
    //     // for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
    //     //   m_link_vec_bus[i] -> m_N_in_tree -> print_out();
    //     // }
    //     // for (size_t i = 0; i<m_link_vec_walking.size(); ++i){
    //     //   m_link_vec_walking[i] -> m_N_in_tree -> print_out();
    //     // }
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         if (end_prt[t] < start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_pnr, end time smaller than start time");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_pnr, loaded data not enough");
    //         }
    //         for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
    //             MNM_DTA_GRADIENT::add_dar_records_passenger(_record, m_link_vec_bus[i], m_path_set_pnr, TFlt(start_prt[t]), TFlt(end_prt[t]));
    //         }
    //         for (size_t i = 0; i<m_link_vec_walking.size(); ++i){
    //             MNM_DTA_GRADIENT::add_dar_records_passenger(_record, m_link_vec_walking[i], m_path_set_pnr, TFlt(start_prt[t]), TFlt(end_prt[t]));
    //         }
    //     }
    //     // path_ID, assign_time, link_ID, start_int, flow
    //     double result_prt[int(_record.size())*5];
    //     // int new_shape [2] = { (int) _record.size(), 5};
    //     // auto result = py::array_t<double>(new_shape);
    //     // auto result_buf = result.request();
    //     // double *result_prt = (double *) result_buf.ptr;
    //     dar_record* tmp_record;
    //     for (size_t i = 0; i < _record.size(); ++i){
    //         tmp_record = _record[i];
    //         result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    //         // the count of 1 min interval
    //         result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    //         result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    //         // the count of unit time interval (5s)
    //         result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    //         result_prt[i * 5 + 4] = tmp_record -> flow();
    //         printf("path ID: %f, departure assign interval (1 min): %f, link ID: %f, time interval (5 s): %f, flow: %f\n",
    //                result_prt[i * 5 + 0], result_prt[i * 5 + 1], result_prt[i * 5 + 2], result_prt[i * 5 + 3], result_prt[i * 5 + 4]);
    //     }
    //     for (size_t i = 0; i < _record.size(); ++i){
    //         delete _record[i];
    //     }
    //     _record.clear();
    // }

    // ******************************************************
    // Mmdta_Api::get_registered_path_tt_truck()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     std::vector<double> start_prt = std::vector<double>();

    //     double result_prt[int(m_path_vec_driving.size()) * l];
    //     // int new_shape [2] = { (int) m_path_vec_driving.size(), l};
    //     // auto result = py::array_t<double>(new_shape);
    //     // auto result_buf = result.request();
    //     // double *result_prt = (double *) result_buf.ptr;

    //     MNM_Passenger_Path_Base *_p_path;
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back((double)t);
    //         if (start_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_truck, loaded data not enough");
    //         }
    //         for (size_t i = 0; i < m_path_vec_driving.size(); ++i){
    //             if (m_ID_path_mapping.find(m_path_vec_driving[i] -> m_path_ID) == m_ID_path_mapping.end()) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_truck, invalid path");
    //             }
    //             _p_path = m_ID_path_mapping.find(m_path_vec_driving[i] -> m_path_ID) -> second.second;
    //             if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) == nullptr) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_truck, invalid passenger path");
    //             }
    //             double _tmp = dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) ->get_travel_time_truck(TFlt(start_prt[t]), m_mmdta)() * m_mmdta -> m_unit_time;
    //             result_prt[i * l + t] = _tmp;
    //             printf("path ID: %d, time interval (5 s): %d, path travel time: %f\n",
    //                    (int)m_path_vec_driving[i] -> m_path_ID, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_registered_path_tt_driving()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     std::vector<double> start_prt = std::vector<double>();

    //     double result_prt[int(m_path_vec_driving.size()) * l];
    //     // int new_shape [2] = { (int) m_path_vec_driving.size(), l};
    //     // auto result = py::array_t<double>(new_shape);
    //     // auto result_buf = result.request();
    //     // double *result_prt = (double *) result_buf.ptr;

    //     MNM_Passenger_Path_Base *_p_path;
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back((double)t);
    //         if (start_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_driving, loaded data not enough");
    //         }
    //         for (size_t i = 0; i < m_path_vec_driving.size(); ++i){
    //             if (m_ID_path_mapping.find(m_path_vec_driving[i] -> m_path_ID) == m_ID_path_mapping.end()) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_driving, invalid path");
    //             }
    //             _p_path = m_ID_path_mapping.find(m_path_vec_driving[i] -> m_path_ID) -> second.second;
    //             if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) == nullptr) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_driving, invalid passenger path");
    //             }
    //             double _tmp = _p_path ->get_travel_time(TFlt(start_prt[t]), m_mmdta)() * m_mmdta -> m_unit_time;
    //             result_prt[i * l + t] = _tmp;
    //             printf("path ID: %d, time interval (5 s): %d, path travel time: %f\n",
    //                    (int)m_path_vec_driving[i] -> m_path_ID, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_registered_path_tt_bustransit()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     std::vector<double> start_prt = std::vector<double>();

    //     double result_prt[int(m_path_vec_bustransit.size()) * l];
    //     // int new_shape [2] = { (int) m_path_vec_bustransit.size(), l};
    //     // auto result = py::array_t<double>(new_shape);
    //     // auto result_buf = result.request();
    //     // double *result_prt = (double *) result_buf.ptr;

    //     MNM_Passenger_Path_Base *_p_path;
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back((double)t);
    //         if (start_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_bustransit, loaded data not enough");
    //         }
    //         for (size_t i = 0; i < m_path_vec_bustransit.size(); ++i){
    //             if (m_ID_path_mapping.find(m_path_vec_bustransit[i] -> m_path_ID) == m_ID_path_mapping.end()) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_bustransit, invalid path");
    //             }
    //             _p_path = m_ID_path_mapping.find(m_path_vec_bustransit[i] -> m_path_ID) -> second.second;
    //             if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_Bus*>(_p_path) == nullptr) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_bustransit, invalid passenger path");
    //             }
    //             double _tmp = _p_path ->get_travel_time(TFlt(start_prt[t]), m_mmdta)() * m_mmdta -> m_unit_time;
    //             result_prt[i * l + t] = _tmp;
    //             printf("path ID: %d, time interval (5 s): %d, path travel time: %f\n",
    //                    (int)m_path_vec_bustransit[i] -> m_path_ID, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_registered_path_tt_pnr()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     std::vector<double> start_prt = std::vector<double>();

    //     double result_prt[int(m_path_vec_pnr.size()) * l];
    //     // int new_shape [2] = { (int) m_path_vec_pnr.size(), l};
    //     // auto result = py::array_t<double>(new_shape);
    //     // auto result_buf = result.request();
    //     // double *result_prt = (double *) result_buf.ptr;

    //     MNM_Passenger_Path_Base *_p_path;
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back((double)t);
    //         if (start_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_pnr, loaded data not enough");
    //         }
    //         for (size_t i = 0; i < m_path_vec_pnr.size(); ++i){
    //             if (m_ID_path_mapping.find(m_path_vec_pnr[i] -> m_path_ID) == m_ID_path_mapping.end()) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_pnr, invalid path");
    //             }
    //             _p_path = m_ID_path_mapping.find(m_path_vec_pnr[i] -> m_path_ID) -> second.second;
    //             if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_PnR*>(_p_path) == nullptr) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_pnr, invalid passenger path");
    //             }
    //             double _tmp = _p_path ->get_travel_time(TFlt(start_prt[t]), m_mmdta)() * m_mmdta -> m_unit_time;
    //             result_prt[i * l + t] = _tmp;
    //             printf("path ID: %d, time interval (5 s): %d, path travel time: %f\n",
    //                    (int)m_path_vec_pnr[i] -> m_path_ID, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_registered_path_cost_driving()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     std::vector<double> start_prt = std::vector<double>();

    //     double result_prt[int(m_path_vec_driving.size()) * l];
    //     // int new_shape [2] = { (int) m_path_vec_driving.size(), l};
    //     // auto result = py::array_t<double>(new_shape);
    //     // auto result_buf = result.request();
    //     // double *result_prt = (double *) result_buf.ptr;

    //     MNM_Passenger_Path_Base *_p_path;
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back((double)t);
    //         if (start_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_driving, loaded data not enough");
    //         }
    //         for (size_t i = 0; i < m_path_vec_driving.size(); ++i){
    //             if (m_ID_path_mapping.find(m_path_vec_driving[i] -> m_path_ID) == m_ID_path_mapping.end()) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_driving, invalid path");
    //             }
    //             _p_path = m_ID_path_mapping.find(m_path_vec_driving[i] -> m_path_ID) -> second.second;
    //             if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) == nullptr) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_driving, invalid passenger path");
    //             }
    //             double _tmp = _p_path ->get_travel_cost(TFlt(start_prt[t]), m_mmdta)();
    //             result_prt[i * l + t] = _tmp;
    //             printf("path ID: %d, time interval (5 s): %d, path travel cost: %f\n",
    //                    (int)m_path_vec_driving[i] -> m_path_ID, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_registered_path_cost_bustransit()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     std::vector<double> start_prt = std::vector<double>();

    //     double result_prt[int(m_path_vec_bustransit.size()) * l];
    //     // int new_shape [2] = { (int) m_path_vec_bustransit.size(), l};
    //     // auto result = py::array_t<double>(new_shape);
    //     // auto result_buf = result.request();
    //     // double *result_prt = (double *) result_buf.ptr;

    //     MNM_Passenger_Path_Base *_p_path;
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back((double)t);
    //         if (start_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_bustransit, loaded data not enough");
    //         }
    //         for (size_t i = 0; i < m_path_vec_bustransit.size(); ++i){
    //             if (m_ID_path_mapping.find(m_path_vec_bustransit[i] -> m_path_ID) == m_ID_path_mapping.end()) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_bustransit, invalid path");
    //             }
    //             _p_path = m_ID_path_mapping.find(m_path_vec_bustransit[i] -> m_path_ID) -> second.second;
    //             if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_Bus*>(_p_path) == nullptr) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_bustransit, invalid passenger path");
    //             }
    //             double _tmp = _p_path ->get_travel_cost(TFlt(start_prt[t]), m_mmdta)();
    //             result_prt[i * l + t] = _tmp;
    //             printf("path ID: %d, time interval (5 s): %d, path travel cost: %f\n",
    //                    (int)m_path_vec_bustransit[i] -> m_path_ID, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_registered_path_cost_pnr()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     std::vector<double> start_prt = std::vector<double>();

    //     double result_prt[int(m_path_vec_pnr.size()) * l];
    //     // int new_shape [2] = { (int) m_path_vec_pnr.size(), l};
    //     // auto result = py::array_t<double>(new_shape);
    //     // auto result_buf = result.request();
    //     // double *result_prt = (double *) result_buf.ptr;

    //     MNM_Passenger_Path_Base *_p_path;
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back((double)t);
    //         if (start_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_pnr, loaded data not enough");
    //         }
    //         for (size_t i = 0; i < m_path_vec_pnr.size(); ++i){
    //             if (m_ID_path_mapping.find(m_path_vec_pnr[i] -> m_path_ID) == m_ID_path_mapping.end()) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_pnr, invalid path");
    //             }
    //             _p_path = m_ID_path_mapping.find(m_path_vec_pnr[i] -> m_path_ID) -> second.second;
    //             if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_PnR*>(_p_path) == nullptr) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_pnr, invalid passenger path");
    //             }
    //             double _tmp = _p_path ->get_travel_cost(TFlt(start_prt[t]), m_mmdta)();
    //             result_prt[i * l + t] = _tmp;
    //             printf("path ID: %d, time interval (5 s): %d, path travel cost: %f\n",
    //                    (int)m_path_vec_pnr[i] -> m_path_ID, t, result_prt[i * l + t]);
    //         }
    //     }
    // }
};