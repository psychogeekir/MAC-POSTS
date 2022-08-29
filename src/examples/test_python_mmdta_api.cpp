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
    // std::string folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode_insufficient_paths";
    // std::string folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_16link_multimodal";
    // std::string folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_dode/record/input_files_estimate_path_flow";
    // std::string folder = "/srv/data/qiling/Projects/CentralOhio_Honda_Project/Multimodal/input_files_CentralOhio_multimodal_AM_updated";
    std::string folder = "/srv/data/qiling/Projects/Philly/input_files_philly";

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
    std::vector<MNM_Dlink_Multiclass*> m_link_vec_bus_driving;

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
    m_mmdue -> init_passenger_path_table();
    IAssert(m_mmdue -> m_mmdta_config -> get_string("routing_type") == "Multimodal_Hybrid" ||
            m_mmdue -> m_mmdta_config -> get_string("routing_type") == "Multimodal_Hybrid_ColumnGeneration" ||
            m_mmdue -> m_mmdta_config -> get_string("routing_type") == "Multimodal_DUE_FixedPath" ||
            m_mmdue -> m_mmdta_config -> get_string("routing_type") == "Multimodal_DUE_ColumnGeneration");
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
    // for (auto it: m_mmdta -> m_link_factory -> m_link_map) {
    //     links_ptr.push_back((int)it.first);
    //     _count++;
    // }
    std::vector<int> links_ID = {2, 3, 4, 5};
    // std::vector<int> links_ID = {21695};
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
    if (m_link_vec_walking.size() > 0){
        printf("Warning, Mmdta_Api::register_links_walking, link exists\n");
        m_link_vec_walking.clear();
    }
    links_ptr.clear();
    _count = 0;
    // for (auto it: m_mmdta -> m_transitlink_factory -> m_transit_link_map) {
    //     if (MNM_Walking_Link * _wlink = dynamic_cast<MNM_Walking_Link *>(it.second)) {
    //         links_ptr.push_back((int)it.first);
    //         _count++;
    //     }
    // }
    links_ID.clear();
    links_ID = {};  // {2, 33};
    for (auto it: links_ID) {
        links_ptr.push_back(it);
        _count++;
    }
    MNM_Transit_Link *_twlink;
    for (int i = 0; i < _count; i++){
        _twlink = m_mmdta -> m_transitlink_factory ->get_transit_link(TInt(links_ptr[i]));
        // printf("%d\n", links_ptr[i]);
        if (MNM_Walking_Link * _wlink = dynamic_cast<MNM_Walking_Link *>(_twlink)){
            if(std::find(m_link_vec_walking.begin(), m_link_vec_walking.end(), _wlink) != m_link_vec_walking.end()) {
                throw std::runtime_error("Error, Mmdta_Api::register_links_walking, link does not exist");
            }
            else {
                m_link_vec_walking.push_back(_wlink);
            }
        }
        else{
            throw std::runtime_error("Mmdta_Api::register_links_walking: link type is not walking");
        }
    }

    // ******************************************************
    // Mmdta_Api::register_links_bus
    // ******************************************************
    if (m_link_vec_bus.size() > 0){
        printf("Warning, Mmdta_Api::register_links_bus, link exists\n");
        m_link_vec_bus.clear();
    }
    links_ptr.clear();
    _count = 0;
    for (auto it: m_mmdta -> m_transitlink_factory -> m_transit_link_map) {
        if (MNM_Bus_Link * _blink = dynamic_cast<MNM_Bus_Link *>(it.second)) {
            links_ptr.push_back((int)it.first);
            _count++;
        }
    }
    // links_ID.clear();
    // links_ID = {};  // {207, 211};
    for (auto it: links_ID) {
        links_ptr.push_back(it);
        _count++;
    }
    MNM_Transit_Link *_tblink;
    for (int i = 0; i < _count; i++){
        _tblink = m_mmdta -> m_transitlink_factory ->get_transit_link(TInt(links_ptr[i]));
        // printf("%d\n", links_ptr[i]);
        if (MNM_Bus_Link * _blink = dynamic_cast<MNM_Bus_Link *>(_tblink)){
            if(std::find(m_link_vec_bus.begin(), m_link_vec_bus.end(), _blink) != m_link_vec_bus.end()) {
                throw std::runtime_error("Error, Mmdta_Api::register_links_bus, link does not exist");
            }
            else {
                m_link_vec_bus.push_back(_blink);
            }
        }
        else{
            throw std::runtime_error("Mmdta_Api::register_links_bus: link type is not walking");
        }
    }

    // ******************************************************
    // Mmdta_Api::get_links_overlapped_bus_driving
    // ******************************************************
    {
        if (m_link_vec_bus.empty()){
            printf("Warning, Mmdta_Api::get_links_overlapped_bus_driving, bus links do not exist\n");
            exit(-1);
        }
        struct _record { 
            TInt bus_link_ID; 
            TInt driving_link_ID; 
            TFlt length_proportion;
        };
        std::vector<_record*> _overlapped_driving_link_records = std::vector<_record*> ();
        for (auto _bus_link : m_link_vec_bus) {
            size_t i = 0;
            for (auto _driving_link : _bus_link -> m_overlapped_driving_link_vec) {
                auto new_record = new _record();
                new_record->bus_link_ID = _bus_link -> m_link_ID;
                new_record->driving_link_ID = _driving_link -> m_link_ID;
                new_record->length_proportion = _bus_link -> m_overlapped_driving_link_length_portion_vec[i];
                _overlapped_driving_link_records.push_back(new_record);
                i++;
            }
        }

        double result_prt[_overlapped_driving_link_records.size()*3];
        _record* tmp_record;
        for (size_t i = 0; i < _overlapped_driving_link_records.size(); ++i){
            tmp_record = _overlapped_driving_link_records[i];
            result_prt[i * 3 + 0] = (double) tmp_record -> bus_link_ID();
            result_prt[i * 3 + 1] = (double) tmp_record -> driving_link_ID();
            result_prt[i * 3 + 2] = (double) tmp_record -> length_proportion();
            printf("bus_link_ID: %f, driving_link_ID: %f, length_proportion: %f\n",
                   result_prt[i * 3 + 0], result_prt[i * 3 + 1], result_prt[i * 3 + 2]);
        }

        for (size_t i = 0; i < _overlapped_driving_link_records.size(); ++i){
            delete _overlapped_driving_link_records[i];
        }
        _overlapped_driving_link_records.clear();
    }

    // ******************************************************
    // Mmdta_Api::register_links_bus_driving
    // ******************************************************
    {
        if (m_link_vec_bus.empty()){
            printf("Warning, Mmdta_Api::get_links_overlapped_bus_driving, bus links do not exist\n");
            exit(-1);
        }
        std::vector<TInt> _overlapped_driving_link_records = std::vector<TInt>();
        for (auto _bus_link : m_link_vec_bus) {
            for (auto _driving_link : _bus_link -> m_overlapped_driving_link_vec) {
                if (std::find(_overlapped_driving_link_records.begin(), _overlapped_driving_link_records.end(),  _driving_link -> m_link_ID) == _overlapped_driving_link_records.end()) {
                    _overlapped_driving_link_records.push_back(_driving_link -> m_link_ID);
                }
            }
        }

        MNM_Dlink *_link;
        for (size_t i = 0; i < _overlapped_driving_link_records.size(); i++){
            _link = m_mmdta -> m_link_factory -> get_link(TInt(_overlapped_driving_link_records[i]));
            // printf("%d\n", links_ptr[i]);
            if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
                if(std::find(m_link_vec_bus_driving.begin(), m_link_vec_bus_driving.end(), _link) != m_link_vec_bus_driving.end()) {
                    throw std::runtime_error("Error, Mmdta_Api::register_links_bus_driving, link does not exist");
                }
                else {
                    m_link_vec_bus_driving.push_back(_mclink);
                }
            }
            else{
                throw std::runtime_error("Mmdta_Api::register_links_bus_driving: link type is not multiclass");
            }
        }
    }


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
    // Mmdta_Api::check_registered_links_in_registered_paths_driving
    // Mmdta_Api::generate_paths_to_cover_registered_links_driving
    // ******************************************************
    // {
    //     // Mmdta_Api::check_registered_links_in_registered_paths_driving
    //     std::vector<bool> _link_existing_driving = std::vector<bool> ();
    //     if (m_link_vec_driving.empty()){
    //         printf("Warning, Mmdta_Api::check_registered_links_in_registered_paths_driving, no link registered\n");
    //         // return _link_existing;
    //         exit(-1);
    //     }
    //     for (size_t k = 0; k < m_link_vec_driving.size(); ++k) {
    //         _link_existing_driving.push_back(false);
    //     }
    //     if (m_path_vec_driving.empty() && m_path_vec_pnr.empty()){
    //         printf("Warning, Mmdta_Api::check_registered_links_in_registered_paths_driving, no path registered\n");
    //         // return _link_existing;
    //         exit(-1);
    //     }
    //     for (auto* _path : m_path_vec_driving) {
    //         for (size_t i = 0; i < m_link_vec_driving.size(); ++i) {
    //             if (!_link_existing_driving[i]) {
    //                 _link_existing_driving[i] = _path -> is_link_in(m_link_vec_driving[i] -> m_link_ID);
    //             }
    //         }
    //         if (std::all_of(_link_existing_driving.cbegin(), _link_existing_driving.cend(), [](bool v){return v;})) {
    //             break;
    //         }
    //     }
    //     if (std::any_of(_link_existing_driving.cbegin(), _link_existing_driving.cend(), [](bool v){return !v;})) {
    //         for (auto* _path : m_path_vec_pnr) {
    //             for (size_t i = 0; i < m_link_vec_driving.size(); ++i) {
    //                 if (!_link_existing_driving[i]) {
    //                     _link_existing_driving[i] = _path -> is_link_in(m_link_vec_driving[i] -> m_link_ID);
    //                 }
    //             }
    //             if (std::all_of(_link_existing_driving.cbegin(), _link_existing_driving.cend(), [](bool v){return v;})) {
    //                 break;
    //             }
    //         }
    //     }
    //     if (std::any_of(_link_existing_driving.cbegin(), _link_existing_driving.cend(), [](bool v){return !v;})) {
    //         printf("Warning: some observed driving links in m_link_vec_driving are not covered by generated paths in m_path_vec_driving and m_path_vec_pnr!\n");
    //     }

    //     // Mmdta_Api::generate_paths_to_cover_registered_links_driving
    //     PNEGraph reversed_graph = MNM_Ults::reverse_graph(m_mmdta -> m_graph);
    //     std::unordered_map<TInt, TFlt> _cost_map = std::unordered_map<TInt, TFlt> ();
    //     for (auto _link_it : m_mmdta -> m_link_factory -> m_link_map) {
    //         _cost_map.insert(std::pair<TInt, TFlt>(_link_it.first, dynamic_cast<MNM_Dlink_Multiclass*>(_link_it.second) -> get_link_freeflow_tt_car()));
    //     }
    //     std::unordered_map<TInt, TInt> _shortest_path_tree = std::unordered_map<TInt, TInt>();
    //     std::unordered_map<TInt, TInt> _shortest_path_tree_reversed = std::unordered_map<TInt, TInt>();
    //     TInt _from_node_ID, _to_node_ID;
    //     MNM_Origin_Multimodal *_origin;
    //     MNM_Destination_Multimodal *_dest;
    //     MNM_Path *_path_1, *_path_2, *_path;
    //     std::random_device rng; // random sequence
    //     std::vector<std::pair<TInt, MNM_Origin*>> pair_ptrs_1 = std::vector<std::pair<TInt, MNM_Origin*>> ();
    //     std::vector<std::pair<MNM_Destination*, TFlt*>> pair_ptrs_2 = std::vector<std::pair<MNM_Destination*, TFlt*>> ();

    //     for (size_t i = 0; i < m_link_vec_driving.size(); ++i) {
    //         if (!_link_existing_driving[i]) {
    //             // generate new path including this link
    //             _from_node_ID = m_mmdta -> m_graph -> GetEI(m_link_vec_driving[i] -> m_link_ID).GetSrcNId(); 
    //             _to_node_ID = m_mmdta -> m_graph -> GetEI(m_link_vec_driving[i] -> m_link_ID).GetDstNId();

    //             // path from origin to from_node_ID
    //             if (!_shortest_path_tree.empty()) {
    //                 _shortest_path_tree.clear();
    //             }
    //             if (dynamic_cast<MNM_DMOND_Multiclass*>(m_mmdta -> m_node_factory -> get_node(_from_node_ID)) != nullptr) {
    //                 _path_1 = new MNM_Path();
    //                 _path_1->m_node_vec.push_back(_from_node_ID);
    //             }
    //             else {
    //                 MNM_Shortest_Path::all_to_one_FIFO(_from_node_ID, m_mmdta -> m_graph, _cost_map, _shortest_path_tree);
    //             }
                
    //             // path from to_node_ID to destination
    //             if (!_shortest_path_tree_reversed.empty()) {
    //                 _shortest_path_tree_reversed.clear();
    //             }
    //             if (dynamic_cast<MNM_DMDND_Multiclass*>(m_mmdta -> m_node_factory -> get_node(_to_node_ID)) != nullptr) {
    //                 _path_2 = new MNM_Path();
    //                 _path_2 -> m_node_vec.push_back(_to_node_ID);
    //             }
    //             else {
    //                 MNM_Shortest_Path::all_to_one_FIFO(_to_node_ID, reversed_graph, _cost_map, _shortest_path_tree_reversed);
    //             }

    //             _origin = nullptr;
    //             _dest = nullptr;
    //             bool _flg = false;

    //             if (!pair_ptrs_1.empty()) {
    //                 pair_ptrs_1.clear();
    //             }
    //             for(const auto& p : m_mmdta -> m_od_factory -> m_origin_map) 
    //             {
    //                 pair_ptrs_1.emplace_back(p);
    //             }
    //             std::shuffle(std::begin(pair_ptrs_1), std::end(pair_ptrs_1), rng);
    //             for (auto _it : pair_ptrs_1) {
    //                 _origin = dynamic_cast<MNM_Origin_Multimodal*>(_it.second);
    //                 if (_origin -> m_demand_car.empty()) {
    //                     continue;
    //                 }

    //                 if (!pair_ptrs_2.empty()) {
    //                     pair_ptrs_2.clear();
    //                 }
    //                 for(const auto& p : _origin -> m_demand_car) 
    //                 {
    //                     pair_ptrs_2.emplace_back(p);
    //                 }
    //                 std::shuffle(std::begin(pair_ptrs_2), std::end(pair_ptrs_2), rng);
    //                 for (auto _it_it : pair_ptrs_2) {
    //                     _dest = dynamic_cast<MNM_Destination_Multimodal*>(_it_it.first);
    //                     if (_shortest_path_tree.find(_origin -> m_origin_node -> m_node_ID) -> second != -1 &&
    //                         _shortest_path_tree_reversed.find(_dest -> m_dest_node -> m_node_ID) -> second != -1) {
    //                         _flg = true;
    //                         break;
    //                     }
    //                 }
    //                 if (_flg) {
    //                     break;
    //                 }
    //             }

    //             if (!_flg) {
    //                 printf("Cannot generate driving path covering this link\n");
    //                 // exit(-1);
    //                 continue;
    //             }
    //             IAssert(_origin != nullptr && _dest != nullptr);

    //             if (!_shortest_path_tree.empty()) {
    //                 _path_1 = MNM::extract_path(_origin -> m_origin_node -> m_node_ID, _from_node_ID, 
    //                                             _shortest_path_tree, m_mmdta -> m_graph); 
    //             }
    //             if (!_shortest_path_tree_reversed.empty()) {
    //                 _path_2 = MNM::extract_path(_dest -> m_dest_node -> m_node_ID, _to_node_ID,
    //                                             _shortest_path_tree_reversed, reversed_graph); 
    //             }
                
    //             // merge the paths to a complete path
    //             _path = new MNM_Path();
    //             _path -> m_link_vec = _path_1 -> m_link_vec;
    //             _path -> m_link_vec.push_back(m_link_vec_driving[i] -> m_link_ID);
    //             _path -> m_link_vec.insert(_path -> m_link_vec.end(), _path_2 -> m_link_vec.rbegin(), _path_2 -> m_link_vec.rend());
    //             _path -> m_node_vec = _path_1 -> m_node_vec;
    //             _path -> m_node_vec.insert(_path -> m_node_vec.end(), _path_2 -> m_node_vec.rbegin(), _path_2 -> m_node_vec.rend());
    //             _path -> allocate_buffer(2 * m_mmdta -> m_config -> get_int("max_interval"));
    //             delete _path_1;
    //             delete _path_2;
    //             // add this new path to path table, not the passenger_path_table
    //             dynamic_cast<MNM_Routing_Multimodal_Hybrid*>(m_mmdta -> m_routing) -> m_routing_fixed_car -> m_path_table->find(_origin -> m_origin_node -> m_node_ID) -> second->find(_dest -> m_dest_node -> m_node_ID) -> second -> m_path_vec.push_back(_path);
    //             m_path_vec_driving.push_back(_path);
    //             _link_existing_driving[i] = true;

    //             // check if this new path cover other links
    //             for (size_t j = 0; j < m_link_vec_driving.size(); ++j) {
    //                 if (!_link_existing_driving[j]) {
    //                     _link_existing_driving[j] = _path -> is_link_in(m_link_vec_driving[j] -> m_link_ID);
    //                 }
    //             }
    //             if (std::all_of(_link_existing_driving.cbegin(), _link_existing_driving.cend(), [](bool v){return v;})) {
    //                 printf("All links in m_link_vec_driving are covered by paths in m_path_vec_driving!\n");
    //                 break;
    //             }
    //         }
    //     }

    //     if (std::all_of(_link_existing_driving.cbegin(), _link_existing_driving.cend(), [](bool v){return v;})) {
    //         printf("driving_path_table updated! All links in m_link_vec_driving are covered by paths in m_path_vec_driving!\n");
    //     }
    //     else {
    //         printf("Mmdta_Api::generate_paths_to_cover_registered_links_driving, NOT all links in m_link_vec_driving are covered by paths in m_path_vec_driving!\n");
    //         // exit(-1);
    //     }

    //     // MNM::save_driving_path_table(m_mmdta -> m_file_folder, dynamic_cast<MNM_Routing_Multimodal_Hybrid*>(m_mmdta -> m_routing) -> m_routing_fixed_car -> m_path_table,
    //     //                             "driving_path_table", "driving_path_table_buffer", true);
        
    //     _link_existing_driving.clear();
    //     _cost_map.clear();
    //     _shortest_path_tree.clear();
    //     _shortest_path_tree_reversed.clear();
    //     reversed_graph.Clr();
    //     pair_ptrs_1.clear();
    //     pair_ptrs_2.clear();
        
    // }

    // ******************************************************
    // Mmdta_Api::check_registered_links_in_registered_paths_bus
    // Mmdta_Api::check_registered_links_in_registered_paths_walking
    // Mmdta_Api::generate_paths_to_cover_registered_links_bus_walking
    // ******************************************************
    // {
    //     // Mmdta_Api::check_registered_links_in_registered_paths_bus
    //     std::vector<bool> _link_existing_bus = std::vector<bool> ();
    //     if (m_link_vec_bus.empty()){
    //         printf("Warning, Mmdta_Api::check_registered_links_in_registered_paths_bus, no link registered\n");
    //         // return _link_existing;
    //         exit(-1);
    //     }
    //     for (size_t k = 0; k < m_link_vec_bus.size(); ++k) {
    //         _link_existing_bus.push_back(false);
    //     }
    //     if (m_path_vec_bustransit.empty() && m_path_vec_pnr.empty()){
    //         printf("Warning, Mmdta_Api::check_registered_links_in_registered_paths_bus, no path registered\n");
    //         // return _link_existing;
    //         exit(-1);
    //     }
    //     for (auto* _path : m_path_vec_bustransit) {
    //         for (size_t i = 0; i < m_link_vec_bus.size(); ++i) {
    //             if (!_link_existing_bus[i]) {
    //                 _link_existing_bus[i] = _path -> is_link_in(m_link_vec_bus[i] -> m_link_ID);
    //             }
    //         }
    //         if (std::all_of(_link_existing_bus.cbegin(), _link_existing_bus.cend(), [](bool v){return v;})) {
    //             break;
    //         }
    //     }
    //     if (std::any_of(_link_existing_bus.cbegin(), _link_existing_bus.cend(), [](bool v){return !v;})) {
    //         for (auto* _path : m_path_vec_pnr) {
    //             for (size_t i = 0; i < m_link_vec_bus.size(); ++i) {
    //                 if (!_link_existing_bus[i]) {
    //                     _link_existing_bus[i] = _path -> is_link_in(m_link_vec_bus[i] -> m_link_ID);
    //                 }
    //             }
    //             if (std::all_of(_link_existing_bus.cbegin(), _link_existing_bus.cend(), [](bool v){return v;})) {
    //                 break;
    //             }
    //         }
    //     }
    //     if (std::any_of(_link_existing_bus.cbegin(), _link_existing_bus.cend(), [](bool v){return !v;})) {
    //         printf("Warning: some observed bus links in m_link_vec_bus are not covered by generated paths in m_path_vec_bustransit and m_path_vec_pnr!\n");
    //     }

    //     // Mmdta_Api::check_registered_links_in_registered_paths_walking
    //     std::vector<bool> _link_existing_walking = std::vector<bool> ();
    //     if (m_link_vec_walking.empty()){
    //         printf("Warning, Mmdta_Api::check_registered_links_in_registered_paths_walking, no link registered\n");
    //         // return _link_existing;
    //         exit(-1);
    //     }
    //     for (size_t k = 0; k < m_link_vec_walking.size(); ++k) {
    //         _link_existing_walking.push_back(false);
    //     }
    //     if (m_path_vec_bustransit.empty() && m_path_vec_pnr.empty()){
    //         printf("Warning, Mmdta_Api::check_registered_links_in_registered_paths_walking, no path registered\n");
    //         // return _link_existing;
    //         exit(-1);
    //     }
    //     for (auto* _path : m_path_vec_bustransit) {
    //         for (size_t i = 0; i < m_link_vec_walking.size(); ++i) {
    //             if (!_link_existing_walking[i]) {
    //                 _link_existing_walking[i] = _path -> is_link_in(m_link_vec_walking[i] -> m_link_ID);
    //             }
    //         }
    //         if (std::all_of(_link_existing_walking.cbegin(), _link_existing_walking.cend(), [](bool v){return v;})) {
    //             break;
    //         }
    //     }
    //     if (std::any_of(_link_existing_walking.cbegin(), _link_existing_walking.cend(), [](bool v){return !v;})) {
    //         for (auto* _path : m_path_vec_pnr) {
    //             for (size_t i = 0; i < m_link_vec_walking.size(); ++i) {
    //                 if (!_link_existing_walking[i]) {
    //                     _link_existing_walking[i] = _path -> is_link_in(m_link_vec_walking[i] -> m_link_ID);
    //                 }
    //             }
    //             if (std::all_of(_link_existing_walking.cbegin(), _link_existing_walking.cend(), [](bool v){return v;})) {
    //                 break;
    //             }
    //         }
    //     }
    //     if (std::any_of(_link_existing_walking.cbegin(), _link_existing_walking.cend(), [](bool v){return !v;})) {
    //         printf("Warning: some observed walking links in m_link_vec_walking are not covered by generated paths in m_path_vec_bustransit and m_path_vec_pnr!\n");
    //     }

    //     // Mmdta_Api::generate_paths_to_cover_registered_links_bus_walking
    //     PNEGraph reversed_graph = MNM_Ults::reverse_graph(m_mmdta -> m_bus_transit_graph);
    //     std::unordered_map<TInt, TFlt> _cost_map_driving = std::unordered_map<TInt, TFlt> ();
    //     for (auto _link_it : m_mmdta -> m_link_factory -> m_link_map) {
    //         _cost_map_driving.insert(std::pair<TInt, TFlt>(_link_it.first, dynamic_cast<MNM_Dlink_Multiclass*>(_link_it.second) -> get_link_freeflow_tt_car()));
    //     }
    //     std::unordered_map<TInt, TFlt> _cost_map = std::unordered_map<TInt, TFlt> ();
    //     for (auto _link_it : m_mmdta -> m_transitlink_factory -> m_transit_link_map) {
    //         _cost_map.insert(std::pair<TInt, TFlt>(_link_it.first, _link_it.second -> m_fftt));
    //     }
    //     std::unordered_map<TInt, TInt> _shortest_path_tree = std::unordered_map<TInt, TInt>();
    //     std::unordered_map<TInt, TInt> _shortest_path_tree_reversed = std::unordered_map<TInt, TInt>();
    //     TInt _from_node_ID, _to_node_ID;
    //     MNM_Origin_Multimodal *_origin;
    //     MNM_Destination_Multimodal *_dest;
    //     MNM_Path *_path_1, *_path_2, *_path;
    //     MNM_PnR_Path *_pnr_path;
    //     TInt _mid_parking_lot_ID = -1;
    //     TInt _mid_dest_node_ID = -1;
    //     bool _is_bustransit, _is_pnr;
    //     std::random_device rng; // random sequence
    //     std::vector<std::pair<TInt, MNM_Origin*>> pair_ptrs_1 = std::vector<std::pair<TInt, MNM_Origin*>> ();
    //     std::vector<std::pair<MNM_Destination*, TFlt*>> pair_ptrs_2 = std::vector<std::pair<MNM_Destination*, TFlt*>> ();

    //     for (size_t i = 0; i < m_link_vec_bus.size(); ++i) {
    //         if (!_link_existing_bus[i]) {
    //             _is_bustransit = false;
    //             _is_pnr = false;

    //             // generate new path including this link
    //             _from_node_ID = m_mmdta -> m_bus_transit_graph -> GetEI(m_link_vec_bus[i] -> m_link_ID).GetSrcNId(); 
    //             _to_node_ID = m_mmdta -> m_bus_transit_graph -> GetEI(m_link_vec_bus[i] -> m_link_ID).GetDstNId();

    //             // path from origin to from_node_ID
    //             if (!_shortest_path_tree.empty()) {
    //                 _shortest_path_tree.clear();
    //             } 
    //             if (m_mmdta -> m_bus_transit_graph -> GetNI(_from_node_ID).GetInDeg() == 0) {
    //                 _path_1 = new MNM_Path();
    //                 _path_1->m_node_vec.push_back(_from_node_ID);
    //             }
    //             else {
    //                 MNM_Shortest_Path::all_to_one_FIFO(_from_node_ID, m_mmdta -> m_bus_transit_graph, _cost_map, _shortest_path_tree);
    //             }
                
    //             // path from to_node_ID to destination
    //             if (!_shortest_path_tree_reversed.empty()) {
    //                 _shortest_path_tree_reversed.clear();
    //             }
    //             if (m_mmdta -> m_bus_transit_graph -> GetNI(_from_node_ID).GetOutDeg() == 0) {
    //                 _path_2 = new MNM_Path();
    //                 _path_2 -> m_node_vec.push_back(_to_node_ID);
    //             }
    //             else {
    //                 MNM_Shortest_Path::all_to_one_FIFO(_to_node_ID, reversed_graph, _cost_map, _shortest_path_tree_reversed);
    //             }

    //             _origin = nullptr;
    //             _dest = nullptr;
    //             bool _flg = false;

    //             if (!pair_ptrs_1.empty()) {
    //                 pair_ptrs_1.clear();
    //             }
    //             for(const auto& p : m_mmdta -> m_od_factory -> m_origin_map) 
    //             {
    //                 pair_ptrs_1.emplace_back(p);
    //             }
    //             std::shuffle(std::begin(pair_ptrs_1), std::end(pair_ptrs_1), rng);
    //             for (auto _it : pair_ptrs_1) {
    //                 _origin = dynamic_cast<MNM_Origin_Multimodal*>(_it.second);
    //                 if (_origin -> m_demand_passenger_bus.empty()) {
    //                     continue;
    //                 }
                    
    //                 if (!pair_ptrs_2.empty()) {
    //                     pair_ptrs_2.clear();
    //                 }
    //                 for(const auto& p : _origin -> m_demand_passenger_bus) 
    //                 {
    //                     pair_ptrs_2.emplace_back(p);
    //                 }
    //                 std::shuffle(std::begin(pair_ptrs_2), std::end(pair_ptrs_2), rng);
    //                 for (auto _it_it : pair_ptrs_2) {
    //                     _dest = dynamic_cast<MNM_Destination_Multimodal*>(_it_it.first);
    //                     if (_shortest_path_tree.find(_origin -> m_origin_node -> m_node_ID) -> second != -1 &&
    //                         _shortest_path_tree_reversed.find(_dest -> m_dest_node -> m_node_ID) -> second != -1) {
    //                         _flg = true;
    //                         _is_bustransit = true;
    //                         break;
    //                     }
    //                 }
    //                 if (_flg) {
    //                     break;
    //                 }
    //             }

    //             if (!_flg) {
    //                 // PnR path
    //                 IAssert(!pair_ptrs_1.empty());
    //                 for (auto _it : pair_ptrs_1) {
    //                     _origin = dynamic_cast<MNM_Origin_Multimodal*>(_it.second);
    //                     if (_origin -> m_demand_pnr_car.empty()) {
    //                         continue;
    //                     }
                        
    //                     if (!pair_ptrs_2.empty()) {
    //                         pair_ptrs_2.clear();
    //                     }
    //                     for(const auto& p : _origin -> m_demand_pnr_car) 
    //                     {
    //                         pair_ptrs_2.emplace_back(p);
    //                     }
    //                     std::shuffle(std::begin(pair_ptrs_2), std::end(pair_ptrs_2), rng);
    //                     for (auto _it_it : pair_ptrs_2) {
    //                         _dest = dynamic_cast<MNM_Destination_Multimodal*>(_it_it.first);
    //                         if (!_dest -> m_connected_pnr_parkinglot_vec.empty()) {
    //                             for (auto *_parking_lot : _dest -> m_connected_pnr_parkinglot_vec) {
    //                                 if (_shortest_path_tree.find(_parking_lot -> m_node_ID) -> second != -1 &&
    //                                     _shortest_path_tree_reversed.find(_dest -> m_dest_node -> m_node_ID) -> second != -1) {
    //                                     _mid_dest_node_ID = _parking_lot -> m_node_ID;
    //                                     _mid_parking_lot_ID = _parking_lot -> m_ID;
    //                                     _flg = true;
    //                                     _is_pnr = true;
    //                                     break;
    //                                 }
    //                             }
    //                         }
    //                         if (_flg) {
    //                             break;
    //                         }
    //                     }
    //                     if (_flg) {
    //                         break;
    //                     }
    //                 }
    //             }

    //             if (!_flg) {
    //                 printf("Cannot generate bustransit or PnR path covering this link\n");
    //                 // exit(-1);
    //                 continue;
    //             }
    //             IAssert(_origin != nullptr && _dest != nullptr);
    //             if (_is_pnr) {
    //                 IAssert(_mid_dest_node_ID > -1);
    //             }

    //             if (!_shortest_path_tree.empty()) {
    //                 if (_is_bustransit) {
    //                     _path_1 = MNM::extract_path(_origin -> m_origin_node -> m_node_ID, _from_node_ID, 
    //                                                 _shortest_path_tree, m_mmdta -> m_bus_transit_graph); 
    //                 }
    //                 else if (_is_pnr) {
    //                     _path_1 = MNM::extract_path(_mid_dest_node_ID, _from_node_ID, 
    //                                                 _shortest_path_tree, m_mmdta -> m_bus_transit_graph); 
    //                 }
    //                 else {
    //                     printf("Cannot generate bustransit or PnR path covering this link\n");
    //                     exit(-1);
    //                 }
    //             }
    //             if (!_shortest_path_tree_reversed.empty()) {
    //                 _path_2 = MNM::extract_path(_dest -> m_dest_node -> m_node_ID, _to_node_ID,
    //                                             _shortest_path_tree_reversed, reversed_graph); 
    //             }
                
    //             // merge the paths to a complete path
    //             _path = new MNM_Path();
    //             _path -> m_link_vec = _path_1 -> m_link_vec;
    //             _path -> m_link_vec.push_back(m_link_vec_bus[i] -> m_link_ID);
    //             _path -> m_link_vec.insert(_path -> m_link_vec.end(), _path_2 -> m_link_vec.rbegin(), _path_2 -> m_link_vec.rend());
    //             _path -> m_node_vec = _path_1 -> m_node_vec;
    //             _path -> m_node_vec.insert(_path -> m_node_vec.end(), _path_2 -> m_node_vec.rbegin(), _path_2 -> m_node_vec.rend());
    //             delete _path_1;
    //             delete _path_2;

    //             // add this new path to path table, not the passenger_path_table
    //             if (_is_bustransit) {
    //                 _path -> allocate_buffer(m_mmdta -> m_config -> get_int("max_interval"));
    //                 dynamic_cast<MNM_Routing_Multimodal_Hybrid*>(m_mmdta -> m_routing) -> m_routing_passenger_fixed -> m_bustransit_path_table->find(_origin -> m_origin_node -> m_node_ID) -> second->find(_dest -> m_dest_node -> m_node_ID) -> second -> m_path_vec.push_back(_path);
    //                 m_path_vec_bustransit.push_back(_path);
    //             }
    //             else if (_is_pnr) {
    //                 _shortest_path_tree.clear();
    //                 MNM_Shortest_Path::all_to_one_FIFO(_mid_dest_node_ID, m_mmdta -> m_graph, _cost_map_driving, _shortest_path_tree);
    //                 IAssert(_shortest_path_tree.find(_origin -> m_origin_node -> m_node_ID) -> second != -1);
    //                 _path_1 = MNM::extract_path(_origin -> m_origin_node -> m_node_ID, _mid_dest_node_ID, 
    //                                             _shortest_path_tree, m_mmdta -> m_graph); 
    //                 _pnr_path = new MNM_PnR_Path((int)m_path_vec_pnr.size(), _mid_parking_lot_ID, _mid_dest_node_ID, _path_1, _path);
    //                 _pnr_path -> allocate_buffer(m_mmdta -> m_config -> get_int("max_interval"));
    //                 delete _path_1;
    //                 delete _path;
    //                 dynamic_cast<MNM_Routing_Multimodal_Hybrid*>(m_mmdta -> m_routing) -> m_routing_car_pnr_fixed -> m_pnr_path_table ->find(_origin -> m_origin_node -> m_node_ID) -> second->find(_dest -> m_dest_node -> m_node_ID) -> second -> m_path_vec.push_back(_pnr_path);
    //                 m_path_vec_pnr.push_back(_pnr_path);
    //             }
    //             else {
    //                 printf("Cannot generate bustransit or PnR path covering this link\n");
    //                 // exit(-1);
    //                 continue;
    //             }
    //             _link_existing_bus[i] = true;
                
    //             // check if this new path cover other links
    //             for (size_t j = 0; j < m_link_vec_bus.size(); ++j) {
    //                 if (!_link_existing_bus[j]) {
    //                     if (_is_bustransit) {
    //                         _link_existing_bus[j] = _path -> is_link_in(m_link_vec_bus[j] -> m_link_ID);
    //                     }
    //                     else if (_is_pnr) {
    //                         _link_existing_bus[j] = _pnr_path -> is_link_in(m_link_vec_bus[j] -> m_link_ID);
    //                     }
    //                 }
    //             }
    //             for (size_t j = 0; j < m_link_vec_walking.size(); ++j) {
    //                 if (!_link_existing_walking[j]) {
    //                     if (_is_bustransit) {
    //                         _link_existing_walking[j] = _path -> is_link_in(m_link_vec_walking[j] -> m_link_ID);
    //                     }
    //                     else if (_is_pnr) {
    //                         _link_existing_walking[j] = _pnr_path -> is_link_in(m_link_vec_walking[j] -> m_link_ID);
    //                     }
    //                 }
    //             }
    //             if (std::all_of(_link_existing_bus.cbegin(), _link_existing_bus.cend(), [](bool v){return v;})) {
    //                 printf("All links in m_link_vec_bus are covered by paths in m_path_vec_bustransit and m_path_vec_pnr!\n");
    //                 break;
    //             }
    //         }
    //     }

    //     if (std::all_of(_link_existing_bus.cbegin(), _link_existing_bus.cend(), [](bool v){return v;})) {
    //         printf("bustransit_path_table updated! All links in m_link_vec_bus are covered by paths in m_path_vec_bustransit and m_path_vec_pnr!\n");
    //     }
    //     else {
    //         printf("Mmdta_Api::generate_paths_to_cover_registered_links_bus_walking, NOT all links in m_link_vec_bus are covered by paths in m_path_vec_bustransit and m_path_vec_pnr!\n");
    //         // exit(-1);
    //     }

    //     for (size_t i = 0; i < m_link_vec_walking.size(); ++i) {
    //         if (!_link_existing_walking[i]) {
    //             _is_bustransit = false;
    //             _is_pnr = false;

    //             // generate new path including this link
    //             _from_node_ID = m_mmdta -> m_bus_transit_graph -> GetEI(m_link_vec_walking[i] -> m_link_ID).GetSrcNId(); 
    //             _to_node_ID = m_mmdta -> m_bus_transit_graph -> GetEI(m_link_vec_walking[i] -> m_link_ID).GetDstNId();

    //             // path from origin to from_node_ID
    //             if (!_shortest_path_tree.empty()) {
    //                 _shortest_path_tree.clear();
    //             }
    //             if (m_mmdta -> m_bus_transit_graph -> GetNI(_from_node_ID).GetInDeg() == 0) {
    //                 _path_1 = new MNM_Path();
    //                 _path_1->m_node_vec.push_back(_from_node_ID);
    //             }
    //             else {
    //                 MNM_Shortest_Path::all_to_one_FIFO(_from_node_ID, m_mmdta -> m_bus_transit_graph, _cost_map, _shortest_path_tree);
    //             }
                
    //             // path from to_node_ID to destination
    //             if (!_shortest_path_tree_reversed.empty()) {
    //                 _shortest_path_tree_reversed.clear();
    //             }
    //             if (m_mmdta -> m_bus_transit_graph -> GetNI(_from_node_ID).GetOutDeg() == 0) {
    //                 _path_2 = new MNM_Path();
    //                 _path_2 -> m_node_vec.push_back(_to_node_ID);
    //             }
    //             else {
    //                 MNM_Shortest_Path::all_to_one_FIFO(_to_node_ID, reversed_graph, _cost_map, _shortest_path_tree_reversed);
    //             }

    //             _origin = nullptr;
    //             _dest = nullptr;
    //             bool _flg = false;

    //             if (!pair_ptrs_1.empty()) {
    //                 pair_ptrs_1.clear();
    //             }
    //             for(const auto& p : m_mmdta -> m_od_factory -> m_origin_map) 
    //             {
    //                 pair_ptrs_1.emplace_back(p);
    //             }
    //             std::shuffle(std::begin(pair_ptrs_1), std::end(pair_ptrs_1), rng);
    //             for (auto _it : pair_ptrs_1) {
    //                 _origin = dynamic_cast<MNM_Origin_Multimodal*>(_it.second);
    //                 if (_origin -> m_demand_passenger_bus.empty()) {
    //                     continue;
    //                 }
                    
    //                 if (!pair_ptrs_2.empty()) {
    //                     pair_ptrs_2.clear();
    //                 }
    //                 for(const auto& p : _origin -> m_demand_passenger_bus) 
    //                 {
    //                     pair_ptrs_2.emplace_back(p);
    //                 }
    //                 std::shuffle(std::begin(pair_ptrs_2), std::end(pair_ptrs_2), rng);
    //                 for (auto _it_it : pair_ptrs_2) {
    //                     _dest = dynamic_cast<MNM_Destination_Multimodal*>(_it_it.first);
    //                     if (_shortest_path_tree.find(_origin -> m_origin_node -> m_node_ID) -> second != -1 &&
    //                         _shortest_path_tree_reversed.find(_dest -> m_dest_node -> m_node_ID) -> second != -1) {
    //                         _flg = true;
    //                         _is_bustransit = true;
    //                         break;
    //                     }
    //                 }
    //                 if (_flg) {
    //                     break;
    //                 }
    //             }

    //             if (!_flg) {
    //                 // PnR path
    //                 IAssert(!pair_ptrs_1.empty());
    //                 for (auto _it : pair_ptrs_1) {
    //                     _origin = dynamic_cast<MNM_Origin_Multimodal*>(_it.second);
    //                     if (_origin -> m_demand_pnr_car.empty()) {
    //                         continue;
    //                     }
                        
    //                     if (!pair_ptrs_2.empty()) {
    //                         pair_ptrs_2.clear();
    //                     }
    //                     for(const auto& p : _origin -> m_demand_pnr_car) 
    //                     {
    //                         pair_ptrs_2.emplace_back(p);
    //                     }
    //                     std::shuffle(std::begin(pair_ptrs_2), std::end(pair_ptrs_2), rng);
    //                     for (auto _it_it : pair_ptrs_2) {
    //                         _dest = dynamic_cast<MNM_Destination_Multimodal*>(_it_it.first);
    //                         if (!_dest -> m_connected_pnr_parkinglot_vec.empty()) {
    //                             for (auto *_parking_lot : _dest -> m_connected_pnr_parkinglot_vec) {
    //                                 if (_shortest_path_tree.find(_parking_lot -> m_node_ID) -> second != -1 &&
    //                                     _shortest_path_tree_reversed.find(_dest -> m_dest_node -> m_node_ID) -> second != -1) {
    //                                     _mid_dest_node_ID = _parking_lot -> m_node_ID;
    //                                     _mid_parking_lot_ID = _parking_lot -> m_ID;
    //                                     _flg = true;
    //                                     _is_pnr = true;
    //                                     break;
    //                                 }
    //                             }
    //                         }
    //                         if (_flg) {
    //                             break;
    //                         }
    //                     }
    //                     if (_flg) {
    //                         break;
    //                     }
    //                 }
    //             }

    //             if (!_flg) {
    //                 printf("Cannot generate bustransit or PnR path covering this link\n");
    //                 // exit(-1);
    //                 continue;
    //             }
    //             IAssert(_origin != nullptr && _dest != nullptr);
    //             if (_is_pnr) {
    //                 IAssert(_mid_dest_node_ID > -1);
    //             }

    //             if (!_shortest_path_tree.empty()) {
    //                 if (_is_bustransit) {
    //                     _path_1 = MNM::extract_path(_origin -> m_origin_node -> m_node_ID, _from_node_ID, 
    //                                                 _shortest_path_tree, m_mmdta -> m_bus_transit_graph); 
    //                 }
    //                 else if (_is_pnr) {
    //                     _path_1 = MNM::extract_path(_mid_dest_node_ID, _from_node_ID, 
    //                                                 _shortest_path_tree, m_mmdta -> m_bus_transit_graph); 
    //                 }
    //                 else {
    //                     printf("Cannot generate bustransit or PnR path covering this link\n");
    //                     // exit(-1);
    //                     continue;
    //                 }
    //             }
    //             if (!_shortest_path_tree_reversed.empty()) {
    //                 _path_2 = MNM::extract_path(_dest -> m_dest_node -> m_node_ID, _to_node_ID,
    //                                             _shortest_path_tree_reversed, reversed_graph); 
    //             }
                
    //             // merge the paths to a complete path
    //             _path = new MNM_Path();
    //             _path -> m_link_vec = _path_1 -> m_link_vec;
    //             _path -> m_link_vec.push_back(m_link_vec_walking[i] -> m_link_ID);
    //             _path -> m_link_vec.insert(_path -> m_link_vec.end(), _path_2 -> m_link_vec.rbegin(), _path_2 -> m_link_vec.rend());
    //             _path -> m_node_vec = _path_1 -> m_node_vec;
    //             _path -> m_node_vec.insert(_path -> m_node_vec.end(), _path_2 -> m_node_vec.rbegin(), _path_2 -> m_node_vec.rend());
    //             delete _path_1;
    //             delete _path_2;

    //             // add this new path to path table, not the passenger_path_table
    //             if (_is_bustransit) {
    //                 _path -> allocate_buffer(m_mmdta -> m_config -> get_int("max_interval"));
    //                 dynamic_cast<MNM_Routing_Multimodal_Hybrid*>(m_mmdta -> m_routing) -> m_routing_passenger_fixed -> m_bustransit_path_table->find(_origin -> m_origin_node -> m_node_ID) -> second->find(_dest -> m_dest_node -> m_node_ID) -> second -> m_path_vec.push_back(_path);
    //                 m_path_vec_bustransit.push_back(_path);
    //             }
    //             else if (_is_pnr) {
    //                 _shortest_path_tree.clear();
    //                 MNM_Shortest_Path::all_to_one_FIFO(_mid_dest_node_ID, m_mmdta -> m_graph, _cost_map_driving, _shortest_path_tree);
    //                 IAssert(_shortest_path_tree.find(_origin -> m_origin_node -> m_node_ID) -> second != -1);
    //                 _path_1 = MNM::extract_path(_origin -> m_origin_node -> m_node_ID, _mid_dest_node_ID, 
    //                                             _shortest_path_tree, m_mmdta -> m_graph); 
    //                 _pnr_path = new MNM_PnR_Path((int)m_path_vec_pnr.size(), _mid_parking_lot_ID, _mid_dest_node_ID, _path_1, _path);
    //                 _pnr_path -> allocate_buffer(m_mmdta -> m_config -> get_int("max_interval"));
    //                 delete _path_1;
    //                 delete _path;
    //                 dynamic_cast<MNM_Routing_Multimodal_Hybrid*>(m_mmdta -> m_routing) -> m_routing_car_pnr_fixed -> m_pnr_path_table ->find(_origin -> m_origin_node -> m_node_ID) -> second->find(_dest -> m_dest_node -> m_node_ID) -> second -> m_path_vec.push_back(_pnr_path);
    //                 m_path_vec_pnr.push_back(_pnr_path);
    //             }
    //             else {
    //                 printf("Cannot generate bustransit or PnR path covering this link\n");
    //                 // exit(-1);
    //                 continue;
    //             }
    //             _link_existing_walking[i] = true;
                
    //             // check if this new path cover other links
    //             for (size_t j = 0; j < m_link_vec_walking.size(); ++j) {
    //                 if (!_link_existing_walking[j]) {
    //                     if (_is_bustransit) {
    //                         _link_existing_walking[j] = _path -> is_link_in(m_link_vec_walking[j] -> m_link_ID);
    //                     }
    //                     else if (_is_pnr) {
    //                         _link_existing_walking[j] = _pnr_path -> is_link_in(m_link_vec_walking[j] -> m_link_ID);
    //                     }
    //                 }
    //             }
    //             if (std::all_of(_link_existing_walking.cbegin(), _link_existing_walking.cend(), [](bool v){return v;})) {
    //                 printf("All links in m_link_vec_walking are covered by paths in m_path_vec_bustransit and m_path_vec_pnr!\n");
    //                 break;
    //             }
    //         }
    //     }

    //     if (std::all_of(_link_existing_bus.cbegin(), _link_existing_bus.cend(), [](bool v){return v;}) &&
    //         std::all_of(_link_existing_walking.cbegin(), _link_existing_walking.cend(), [](bool v){return v;})) {
    //         printf("bustransit_path_table and/or pnr_path_table updated! All links in m_link_vec_bus and m_link_vec_walking are covered by paths in m_path_vec_bustransit and m_path_vec_pnr!\n");
    //     }
    //     else {
    //         printf("Mmdta_Api::generate_paths_to_cover_registered_links_bus_walking, NOT all links in m_link_vec_bus and m_link_vec_walking are covered by paths in m_path_vec_bustransit and m_path_vec_pnr!\n");
    //         // exit(-1);
    //     }


    //     // MNM::save_bustransit_path_table(m_mmdta -> m_file_folder, dynamic_cast<MNM_Routing_Multimodal_Hybrid*>(m_mmdta -> m_routing) -> m_routing_passenger_fixed -> m_bustransit_path_table,
    //     //                                 "bustransit_path_table", "bustransit_path_table_buffer", true);
    //     // MNM::save_pnr_path_table(m_mmdta -> m_file_folder, dynamic_cast<MNM_Routing_Multimodal_Hybrid*>(m_mmdta -> m_routing) -> m_routing_car_pnr_fixed -> m_pnr_path_table,
    //     //                         "pnr_path_table", "pnr_path_table_buffer", true);
        
    //     _link_existing_bus.clear();
    //     _cost_map_driving.clear();
    //     _cost_map.clear();
    //     _shortest_path_tree.clear();
    //     _shortest_path_tree_reversed.clear();
    //     reversed_graph.Clr();
    //     pair_ptrs_1.clear();
    //     pair_ptrs_2.clear();
    // }

    // ******************************************************
    // Mmdta_Api::install_cc()
    // ******************************************************
    // car and truck
    for (size_t i = 0; i < m_link_vec_driving.size(); ++i){
        m_link_vec_driving[i] -> install_cumulative_curve_multiclass();
    }
    // passenger
    for (size_t i = 0; i < m_link_vec_walking.size(); ++i){
        m_link_vec_walking[i] -> install_cumulative_curve();
    }
    // bus and passenger
    for (size_t i = 0; i < m_link_vec_bus.size(); ++i){
        // passenger
        m_link_vec_bus[i] -> install_cumulative_curve();
        // bus
        m_link_vec_bus[i] -> m_from_busstop -> install_cumulative_curve_multiclass();
        m_link_vec_bus[i] -> m_to_busstop -> install_cumulative_curve_multiclass();
    }
    // truck traversing bus links
    for (size_t i = 0; i < m_link_vec_bus_driving.size(); ++i){
        m_link_vec_bus_driving[i] -> install_cumulative_curve_multiclass();
    }

    // ******************************************************
    // Mmdta_Api::install_cc_tree()
    // ******************************************************
    // car and truck
    for (size_t i = 0; i < m_link_vec_driving.size(); ++i){
        m_link_vec_driving[i] -> install_cumulative_curve_tree_multiclass();
    }
    // passenger
    for (size_t i = 0; i < m_link_vec_walking.size(); ++i){
        m_link_vec_walking[i] -> install_cumulative_curve_tree();
    }
    // bus and passenger
    for (size_t i = 0; i < m_link_vec_bus.size(); ++i){
        m_link_vec_bus[i] -> install_cumulative_curve_tree();
    }
    // truck traversing bus links
    for (size_t i = 0; i < m_link_vec_bus_driving.size(); ++i){
        m_link_vec_bus_driving[i] -> install_cumulative_curve_tree_multiclass();
    }

    // ******************************************************
    // Mmdta_Api::run_whole()
    // ******************************************************
    m_mmdta -> pre_loading();
    m_mmdta -> loading(true);

    // ******************************************************
    // Mmdta_Api::run_mmdta_adaptive()
    // ******************************************************
    // m_mmdue -> run_mmdta_adaptive(true);

    // ******************************************************
    // Mmdta_Api::get_cur_loading_interval()
    // ******************************************************
    printf("loading interval is %d\n", m_mmdta -> m_current_loading_interval());

    // ******************************************************
    // Mmdta_Api::get_travel_stats()
    // ******************************************************
    {
        // finished
        TInt _count_car = 0, _count_pnr_car = 0, _count_truck = 0, _count_bus = 0, _count_passenger = 0;
        TFlt _tot_tt_car = 0.0, _tot_tt_truck = 0.0, _tot_tt_bus = 0.0, _tot_tt_passenger = 0.0;

        auto *_veh_factory = dynamic_cast<MNM_Veh_Factory_Multimodal*>(m_mmdta -> m_veh_factory);
        _count_car = _veh_factory -> m_finished_car;
        _count_pnr_car = _veh_factory -> m_finished_car_pnr;
        _count_truck = _veh_factory -> m_finished_truck;
        _count_bus = _veh_factory -> m_finished_bus;
        _count_passenger = m_mmdta -> m_passenger_factory -> m_finished_passenger;

        _tot_tt_car = _veh_factory -> m_total_time_car * m_mmdta -> m_unit_time / 3600.0;
        _tot_tt_truck = _veh_factory -> m_total_time_truck * m_mmdta -> m_unit_time / 3600.0;
        _tot_tt_bus = _veh_factory -> m_total_time_bus * m_mmdta -> m_unit_time / 3600.0;
        _tot_tt_passenger = m_mmdta -> m_passenger_factory -> m_total_time_passenger * m_mmdta -> m_unit_time / 3600.0;

        // unfinished 
        MNM_Veh_Multimodal *_veh;
        MNM_Passenger *_passenger;
        int _end_time = m_mmdta -> m_current_loading_interval();

        for (auto _map_it : m_mmdta -> m_veh_factory -> m_veh_map){
        _veh = dynamic_cast<MNM_Veh_Multimodal *>(_map_it.second);
        IAssert(_veh -> m_finish_time < 0);
        if (_veh -> m_class == 0){
            if (_veh -> get_ispnr()) {
                _count_pnr_car += 1;
            }
            else {
                _count_car += 1;
            }
            _tot_tt_car += (_end_time - _veh -> m_start_time) * m_mmdta -> m_unit_time / 3600.0;
        }
        else {
            if (_veh -> m_bus_route_ID == TInt(-1)) {
                _count_truck += 1;
            }
            else {
                _count_bus += 1;
            }
            _tot_tt_bus += (_end_time - _veh -> m_start_time) * m_mmdta -> m_unit_time / 3600.0;
        }
    }

        for (auto _map_it : m_mmdta -> m_passenger_factory -> m_passenger_map){
            _passenger = _map_it.second;
            IAssert(_passenger -> m_finish_time < 0);
            _count_passenger += 1;
            _tot_tt_passenger += (_end_time - _passenger -> m_start_time) * m_mmdta -> m_unit_time / 3600.0;
        }

        // printf("\n\nTotal car: %d, Total truck: %d, Total bus : %d, Total passenger: %d, Total car tt: %.2f hours, Total truck tt: %.2f hours, Total bus tt: %.2f hours, Total passenger tt: %.2f hours\n\n",
        //         int(_count_car/m_mmdta -> m_flow_scalar), int(_count_truck/m_mmdta -> m_flow_scalar), int(_count_bus/m_mmdta -> m_flow_scalar), int(_count_passenger),
        //         float(_tot_tt_car/m_mmdta -> m_flow_scalar), float(_tot_tt_truck/m_mmdta -> m_flow_scalar), float(_tot_tt_bus/m_mmdta -> m_flow_scalar), float(_tot_tt_passenger));
        // m_mmdta -> m_emission -> output();

        // add flow_scalar to passenger
        // printf("\n\nTotal car: %d, Total truck: %d, Total bus : %d, Total passenger: %d, Total car tt: %.2f hours, Total truck tt: %.2f hours, Total bus tt: %.2f hours, Total passenger tt: %.2f hours\n\n",
        //         int(_count_car/m_mmdta -> m_flow_scalar), int(_count_truck/m_mmdta -> m_flow_scalar), int(_count_bus/m_mmdta -> m_flow_scalar), int(_count_passenger / m_mmdta -> m_flow_scalar),
        //         float(_tot_tt_car/m_mmdta -> m_flow_scalar), float(_tot_tt_truck/m_mmdta -> m_flow_scalar), float(_tot_tt_bus/m_mmdta -> m_flow_scalar), float(_tot_tt_passenger / m_mmdta -> m_flow_scalar));
        // m_mmdta -> m_emission -> output();

        printf("\n************ travel stats ************\n");
        printf("car count: %f\n", _count_car/m_mmdta -> m_flow_scalar);
        printf("car pnr count: %f\n", _count_pnr_car/m_mmdta -> m_flow_scalar);
        printf("truck count: %f\n", _count_truck/m_mmdta -> m_flow_scalar);
        printf("bus count: %f\n", _count_bus/m_mmdta -> m_flow_scalar);
        // printf("passenger count: %f\n", (float)_count_passenger);
        // add flow_scalar to passenger
        printf("passenger count: %f\n", (float)_count_passenger / m_mmdta -> m_flow_scalar);
        printf("car total travel time (hours): %f\n", _tot_tt_car/m_mmdta -> m_flow_scalar);
        printf("truck total travel time (hours): %f\n", _tot_tt_truck/m_mmdta -> m_flow_scalar);
        printf("bus total travel time (hours): %f\n", _tot_tt_bus/m_mmdta -> m_flow_scalar);
        // printf("passenger total travel time (hours): %f\n", (float)_tot_tt_passenger);
        // add flow_scalar to passenger
        printf("passenger total travel time (hours): %f\n", (float)_tot_tt_passenger / m_mmdta -> m_flow_scalar);
        printf("************ travel stats ************\n");
    }

    // ******************************************************
    // Mmdta_Api::build_link_cost_map()
    // ******************************************************
    {
        bool _flg = false;
        m_mmdue -> build_link_cost_map(m_mmdta, _flg);
        if (_flg) {
            m_mmdue -> get_link_queue_dissipated_time(m_mmdta);
        }
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
    // Mmdta_Api::build_link_cost_map_snapshot()
    // Mmdta_Api::update_snapshot_route_table()
    // Mmdta_Api::get_lowest_cost_path_snapshot()
    // ******************************************************
    // {   
    //     int start_interval;

    //     MNM_Path *_path;

    //     MNM_Passenger_Path_Base *_p_path;

    //     TInt _mode;
    //     TFlt _cost;
    //     int _best_time_col, _best_assign_col, _num_col;
    //     bool _exist;
        
    //     for (int _assign_interval = 0; _assign_interval < m_mmdue -> m_total_assign_inter; ++_assign_interval) {
    //         start_interval = _assign_interval * m_mmdue -> m_mmdta_config->get_int("assign_frq");
            
    //         // ******************************************************
    //         // Mmdta_Api::build_link_cost_map_snapshot()
    //         // ******************************************************
    //         m_mmdue -> build_link_cost_map_snapshot(m_mmdta, start_interval, false);
            
    //         // ******************************************************
    //         // Mmdta_Api::update_snapshot_route_table()
    //         // ******************************************************
    //         m_mmdue -> update_snapshot_route_table(m_mmdta, start_interval);

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

    //                 std::pair<std::tuple<MNM_Passenger_Path_Base*, TInt, TFlt, bool>, int> _best = m_mmdue -> get_lowest_cost_path_snapshot(start_interval, o_node_ID, d_node_ID, m_mmdta);

    //                 _mode = _best.second;
    //                 _p_path = std::get<0>(_best.first);
    //                 _exist = std::get<3>(_best.first);

    //                 _path = nullptr;

    //                 if (_mode == driving) {
    //                     _path = dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) -> m_path;
    //                     _num_col = (int) _path -> m_node_vec.size();
    //                 }
    //                 else if (_mode == transit) {
    //                     _path = dynamic_cast<MNM_Passenger_Path_Bus*>(_p_path) -> m_path;
    //                     _num_col = (int) _path -> m_link_vec.size();
    //                 }
    //                 else if (_mode == pnr) {
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

    //                 printf("sp for OD pair: %d -- %d, at interval %d\n", o_node_ID, d_node_ID, start_interval);
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
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     // int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
    //     int new_shape [2] = { (int) m_link_vec_driving.size(), l};
        
    //     double result_prt[l*m_link_vec_driving.size()];
    //     std::vector<double> start_prt = std::vector<double>();
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back((double)t);
    //         // start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_car_link_tt, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         for (size_t i = 0; i < m_link_vec_driving.size(); ++i){
    //             double _tmp = MNM_DTA_GRADIENT::get_travel_time_car(m_link_vec_driving[i], TFlt(start_prt[t] + 1), m_mmdta->m_unit_time, m_mmdta -> m_current_loading_interval)();
    //             // double _tmp = MNM_DTA_GRADIENT::get_travel_time_car_robust(m_link_vec_driving[i], TFlt(start_prt[t] + 1), TFlt(start_prt[t] + m_mmdue -> m_mmdta_config -> get_int("assign_frq") + 1), m_mmdta->m_unit_time, m_mmdta -> m_current_loading_interval)();
    //             // if (_tmp * m_mmdta -> m_unit_time > 20 * (m_link_vec_driving[i] -> m_length / m_link_vec[i] -> m_ffs_car)){
    //             //     _tmp = 20 * m_link_vec_driving[i] -> m_length / m_link_vec_driving[i] -> m_ffs_car / m_mmdta -> m_unit_time;
    //             // }
    //             result_prt[i * l + t] = _tmp * m_mmdta -> m_unit_time;  // seconds
    //             printf("link: i %d, time: t %d, tt: %f\n", (int)i, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_truck_link_tt()
    // ******************************************************
    // {
    //     // int l = m_mmdta -> m_current_loading_interval();
    //     int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
    //     int new_shape [2] = { (int) m_link_vec_driving.size(), l};

    //     double result_prt[l*m_link_vec_driving.size()];
    //     std::vector<double> start_prt = std::vector<double>();
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_truck_link_tt, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         for (size_t i = 0; i < m_link_vec_driving.size(); ++i){
    //             // double _tmp = MNM_DTA_GRADIENT::get_travel_time_truck(m_link_vec_driving[i], TFlt(start_prt[t] + 1), m_mmdta -> m_unit_time, m_mmdta -> m_current_loading_interval)();
    //             double _tmp = MNM_DTA_GRADIENT::get_travel_time_truck_robust(m_link_vec_driving[i], TFlt(start_prt[t] + 1), TFlt(start_prt[t] + m_mmdue -> m_mmdta_config -> get_int("assign_frq") + 1), m_mmdta -> m_unit_time, m_mmdta -> m_current_loading_interval)();
    //             // if (_tmp * 5 > 20 * (m_link_vec_driving[i] -> m_length / m_link_vec_driving[i] -> m_ffs_truck)){
    //             //     _tmp = 20 * m_link_vec_driving[i] -> m_length / m_link_vec_driving[i] -> m_ffs_truck / m_mmdta -> m_unit_time;
    //             // }
    //             result_prt[i * l + t] = _tmp * m_mmdta -> m_unit_time;  // seconds
    //             printf("link: i %d, time: t %d, tt: %f\n", (int)i, t, result_prt[i * l + t]);
    //         }
    //     }
    // } 

    // ******************************************************
    // Mmdta_Api::get_bus_link_tt()
    // ******************************************************
    {
        // int l = m_mmdta -> m_current_loading_interval();
        int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
        int new_shape [2] = { (int) m_link_vec_bus.size(), l};

        double result_prt[l*m_link_vec_bus.size()];
        std::vector<double> start_prt = std::vector<double>();
        for (int t = 0; t < l; ++t){
            start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
            if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
                throw std::runtime_error("Error, Mmdta_Api::get_bus_link_tt, input start intervals exceeds the total loading intervals - 1");
            }
            for (size_t i = 0; i < m_link_vec_bus.size(); ++i){
                // double _tmp = MNM_DTA_GRADIENT::get_travel_time_bus(m_link_vec_bus[i], TFlt(start_prt[t] + 1), m_mmdta -> m_unit_time, m_mmdta -> m_current_loading_interval)();
                double _tmp = MNM_DTA_GRADIENT::get_travel_time_bus_robust(m_link_vec_bus[i], TFlt(start_prt[t] + 1), TFlt(start_prt[t] + m_mmdue -> m_mmdta_config -> get_int("assign_frq") + 1), m_mmdta -> m_unit_time, m_mmdta -> m_current_loading_interval)();
                // if (std::isinf(_tmp)) {
                //     result_prt[i * l + t] = std::numeric_limits<double>::quiet_NaN();
                // }
                // else {
                //     result_prt[i * l + t] = _tmp * m_mmdta -> m_unit_time;  // seconds
                // }
                result_prt[i * l + t] = _tmp * m_mmdta -> m_unit_time;  // seconds
                printf("link: i %d, time: t %d, tt: %f\n", (int)i, t, result_prt[i * l + t]);
            }
        }
    }

    // ******************************************************
    // Mmdta_Api::get_passenger_walking_link_tt()
    // ******************************************************
    // {
    //     // int l = m_mmdta -> m_current_loading_interval();
    //     int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
    //     int new_shape [2] = {(int) m_link_vec_walking.size(), l};

    //     double result_prt[l*m_link_vec_walking.size()];
    //     std::vector<double> start_prt = std::vector<double>();
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_passenger_walking_link_tt, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         for (size_t i = 0; i < m_link_vec_walking.size(); ++i){
    //             // double _tmp = MNM_DTA_GRADIENT::get_travel_time_walking(m_link_vec_walking[i], TFlt(start_prt[t] + 1), m_mmdta -> m_unit_time, m_mmdta -> m_current_loading_interval)();
    //             double _tmp = MNM_DTA_GRADIENT::get_travel_time_walking_robust(m_link_vec_walking[i], TFlt(start_prt[t] + 1), TFlt(start_prt[t] + m_mmdue -> m_mmdta_config -> get_int("assign_frq") + 1), m_mmdta -> m_unit_time, m_mmdta -> m_current_loading_interval)();
    //             result_prt[i * l + t] = _tmp * m_mmdta -> m_unit_time;  // seconds
    //             printf("link: i %d, time: t %d, tt: %f\n", (int)i, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_link_car_inflow()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     double result_prt[l*m_link_vec_driving.size()];
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         if (end_prt[t] <= start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_car_inflow, end time is smaller than or equal to start time");
    //         }
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_car_inflow, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_car_inflow, input end intervals exceeds the total loading intervals");
    //         }
    //         for (size_t i = 0; i < m_link_vec_driving.size(); ++i){
    //             result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_inflow_car(m_link_vec_driving[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
    //             printf("link: i %d, time: t %d, flow: %f\n", (int)i, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_link_truck_inflow()()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     double result_prt[l*m_link_vec_driving.size()];
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         if (end_prt[t] <= start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_truck_inflow, end time is smaller than or equal to start time");
    //         }
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_truck_inflow, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_truck_inflow, input end intervals exceeds the total loading intervals");
    //         }
    //         for (size_t i = 0; i < m_link_vec_driving.size(); ++i){
    //             result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_inflow_truck(m_link_vec_driving[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
    //             printf("link: i %d, time: t %d, flow: %f\n", (int)i, t, result_prt[i * l + t]);
    //         }
    //     }
    // } 

    // ******************************************************
    // Mmdta_Api::get_link_bus_inflow()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     double result_prt[l*m_link_vec_bus.size()];
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         if (end_prt[t] <= start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_bus_inflow, end time is smaller than or equal to start time");
    //         }
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_bus_inflow, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_bus_inflow, input end intervals exceeds the total loading intervals");
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
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     double result_prt[l*m_link_vec_bus.size()];
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         if (end_prt[t] <= start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_busstop_bus_inflow, end time is smaller than or equal to start time");
    //         }
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_busstop_bus_inflow, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_busstop_bus_inflow, input end intervals exceeds the total loading intervals");
    //         }
    //         for (size_t i = 0; i < m_link_vec_bus.size(); ++i){
    //             result_prt[i * l + t] = MNM_DTA_GRADIENT::get_busstop_inflow_bus(m_link_vec_bus[i] -> m_to_busstop, TFlt(start_prt[t]), TFlt(end_prt[t]))();
    //             printf("link: i %d, time: t %d, flow: %f\n", (int)i, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_link_bus_passenger_inflow()
    // ******************************************************
    // {
    //     int l = m_mmdta -> m_current_loading_interval();
    //     double result_prt[l*m_link_vec_bus.size()];
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         if (end_prt[t] <= start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_bus_passenger_inflow, end time is smaller than or equal to start time");
    //         }
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_bus_passenger_inflow, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_bus_passenger_inflow, input end intervals exceeds the total loading intervals");
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
    //     int l = m_mmdta -> m_current_loading_interval();
    //     double result_prt[l*m_link_vec_walking.size()];
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     for (int t = 0; t < l; ++t){
    //         start_prt.push_back(t);
    //         end_prt.push_back(t+1);
    //         if (end_prt[t] <= start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_walking_passenger_inflow, end time is smaller than or equal to start time");
    //         }
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_walking_passenger_inflow, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_link_walking_passenger_inflow, input end intervals exceeds the total loading intervals");
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
    // {
    //     // int l = m_mmdta -> m_current_loading_interval();
    //     int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     std::vector<dar_record*> _record = std::vector<dar_record*>();
    //     // for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
    //     //   m_link_vec_driving[i] -> m_N_in_tree -> print_out();
    //     // }
    //     for (int t = 0; t < l; ++t){
    //         // start_prt.push_back(t);
    //         // end_prt.push_back(t+1);
    //         start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         end_prt.push_back((double)(t+1)*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         // printf("Current processing time: %d\n", t);
    //         if (end_prt[t] <= start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_car_dar_matrix_driving, end time is smaller than or equal to start time");
    //         }
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_car_dar_matrix_driving, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_car_dar_matrix_driving, input end intervals exceeds the total loading intervals");
    //         }
    //         for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
    //             MNM_DTA_GRADIENT::add_dar_records_car(_record, m_link_vec_driving[i], m_path_set_driving, TFlt(start_prt[t]), TFlt(end_prt[t]));
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
    // Mmdta_Api::get_truck_dar_matrix_driving()
    // ******************************************************
    // {
    //     // int l = m_mmdta -> m_current_loading_interval();
    //     int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     std::vector<dar_record*> _record = std::vector<dar_record*>();
    //     // for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
    //     //   m_link_vec_driving[i] -> m_N_in_tree -> print_out();
    //     // }
    //     for (int t = 0; t < l; ++t){
    //         // start_prt.push_back(t);
    //         // end_prt.push_back(t+1);
    //         start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         end_prt.push_back((double)(t+1)*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         if (end_prt[t] <= start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_truck_dar_matrix_driving, end time is smaller than or equal to start time");
    //         }
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_truck_dar_matrix_driving, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_truck_dar_matrix_driving, input end intervals exceeds the total loading intervals");
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
    //     // int l = m_mmdta -> m_current_loading_interval();
    //     int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     std::vector<dar_record*> _record = std::vector<dar_record*>();
    //     // for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
    //     //   m_link_vec_driving[i] -> m_N_in_tree -> print_out();
    //     // }
    //     for (int t = 0; t < l; ++t){
    //         // start_prt.push_back(t);
    //         // end_prt.push_back(t+1);
    //         start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         end_prt.push_back((double)(t+1)*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         // printf("Current processing time: %d\n", t);
    //         if (end_prt[t] <= start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_car_dar_matrix_pnr, end time is smaller than or equal to start time");
    //         }
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_car_dar_matrix_pnr, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_car_dar_matrix_pnr, input end intervals exceeds the total loading intervals");
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
    // Mmdta_Api::get_bus_dar_matrix_bustransit_link()
    // ******************************************************
    // {
    //     // int l = m_mmdta -> m_current_loading_interval();
    //     int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
    //     std::vector<int> start_prt = std::vector<int>();
    //     std::vector<int> end_prt = std::vector<int>();
    //     std::vector<dar_record*> _record = std::vector<dar_record*>();
    //     // for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
    //     //   m_link_vec_bus[i] -> m_N_in_tree -> print_out();
    //     // }
    //     for (int t = 0; t < l; ++t){
    //         // start_prt.push_back(t);
    //         // end_prt.push_back(t+1);
    //         start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         end_prt.push_back((double)(t+1)*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
    //             if (end_prt[t] <= start_prt[t]){
    //                 throw std::runtime_error("Error, Mmdta_Api::get_bus_dar_matrix_bustransit_link, end time is smaller than or equal to start time");
    //             }
    //             if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //                 throw std::runtime_error("Error, Mmdta_Api::get_bus_dar_matrix_bustransit_link, input start intervals exceeds the total loading intervals - 1");
    //             }
    //             if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //                 throw std::runtime_error("Error, Mmdta_Api::get_bus_dar_matrix_bustransit_link, input end intervals exceeds the total loading intervals");
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
    //     // int l = m_mmdta -> m_current_loading_interval();
    //     int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
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
    //         // start_prt.push_back(t);
    //         // end_prt.push_back(t+1);
    //         start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         end_prt.push_back((double)(t+1)*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         if (end_prt[t] <= start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_bustransit, end time is smaller than or equal to start time");
    //         }
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_bustransit, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_bustransit, input end intervals exceeds the total loading intervals");
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
    //     // int l = m_mmdta -> m_current_loading_interval();
    //     int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
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
    //         // start_prt.push_back(t);
    //         // end_prt.push_back(t+1);
    //         start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         end_prt.push_back((double)(t+1)*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
    //         if (end_prt[t] <= start_prt[t]){
    //             throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_pnr, end time is smaller than or equal to start time");
    //         }
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_pnr, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_pnr, input end intervals exceeds the total loading intervals");
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
    // Mmdta_Api::get_truck_dar_matrix_bus_driving_link()
    // ******************************************************
    {
        // int l = m_mmdta -> m_current_loading_interval();
        int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
        std::vector<int> start_prt = std::vector<int>();
        std::vector<int> end_prt = std::vector<int>();
        std::vector<dar_record*> _record = std::vector<dar_record*>();
        // for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
        //   m_link_vec_driving[i] -> m_N_in_tree -> print_out();
        // }
        for (int t = 0; t < l; ++t){
            // start_prt.push_back(t);
            // end_prt.push_back(t+1);
            start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
            end_prt.push_back((double)(t+1)*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
            if (end_prt[t] <= start_prt[t]){
                throw std::runtime_error("Error, Mmdta_Api::get_truck_dar_matrix_bus_driving_link, end time is smaller than or equal to start time");
            }
            if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
                throw std::runtime_error("Error, Mmdta_Api::get_truck_dar_matrix_bus_driving_link, input start intervals exceeds the total loading intervals - 1");
            }
            if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
                throw std::runtime_error("Error, Mmdta_Api::get_truck_dar_matrix_bus_driving_link, input end intervals exceeds the total loading intervals");
            }
            for (size_t i = 0; i<m_link_vec_bus_driving.size(); ++i){
                MNM_DTA_GRADIENT::add_dar_records_truck(_record, m_link_vec_bus_driving[i], m_path_set_driving, TFlt(start_prt[t]), TFlt(end_prt[t]));
            }
        }
        // path_ID, assign_time, link_ID, start_int, flow
        double result_prt[int(_record.size())*5];
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
    // Mmdta_Api::get_passenger_dar_matrix_bustransit_bus_link()
    // ******************************************************
    {
        // int l = m_mmdta -> m_current_loading_interval();
        int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
        std::vector<int> start_prt = std::vector<int>();
        std::vector<int> end_prt = std::vector<int>();
        std::vector<dar_record*> _record = std::vector<dar_record*>();
        // for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
        //   m_link_vec_bus[i] -> m_N_in_tree -> print_out();
        // }
        for (int t = 0; t < l; ++t){
            // start_prt.push_back(t);
            // end_prt.push_back(t+1);
            start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
            end_prt.push_back((double)(t+1)*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
            if (end_prt[t] <= start_prt[t]){
                throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_bustransit_bus_link, end time is smaller than or equal to start time");
            }
            if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
                throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_bustransit_bus_link, input start intervals exceeds the total loading intervals - 1");
            }
            if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
                throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_bustransit_bus_link, input end intervals exceeds the total loading intervals");
            }
            for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
                MNM_DTA_GRADIENT::add_dar_records_passenger(_record, m_link_vec_bus[i], m_path_set_bustransit, TFlt(start_prt[t]), TFlt(end_prt[t]));
            }
        }
        // path_ID, assign_time, link_ID, start_int, flow
        double result_prt[int(_record.size())*5];
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
    // Mmdta_Api::get_passenger_dar_matrix_pnr_bus_link()
    // ******************************************************
    {
        // int l = m_mmdta -> m_current_loading_interval();
        int l = m_mmdue -> m_mmdta_config -> get_int("max_interval");
        std::vector<int> start_prt = std::vector<int>();
        std::vector<int> end_prt = std::vector<int>();
        std::vector<dar_record*> _record = std::vector<dar_record*>();
        // for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
        //   m_link_vec_bus[i] -> m_N_in_tree -> print_out();
        // }
        for (int t = 0; t < l; ++t){
            // start_prt.push_back(t);
            // end_prt.push_back(t+1);
            start_prt.push_back((double)t*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
            end_prt.push_back((double)(t+1)*m_mmdue -> m_mmdta_config -> get_int("assign_frq"));
            if (end_prt[t] <= start_prt[t]){
                throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_pnr_bus_link, end time is smaller than or equal to start time");
            }
            if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
                throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_pnr_bus_link, input start intervals exceeds the total loading intervals - 1");
            }
            if (end_prt[t] > m_mmdta -> m_current_loading_interval()){
                throw std::runtime_error("Error, Mmdta_Api::get_passenger_dar_matrix_pnr_bus_link, input end intervals exceeds the total loading intervals");
            }
            for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
                MNM_DTA_GRADIENT::add_dar_records_passenger(_record, m_link_vec_bus[i], m_path_set_pnr, TFlt(start_prt[t]), TFlt(end_prt[t]));
            }
        }
        // path_ID, assign_time, link_ID, start_int, flow
        double result_prt[int(_record.size())*5];
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
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_truck, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         for (size_t i = 0; i < m_path_vec_driving.size(); ++i){
    //             if (m_ID_path_mapping.find(m_path_vec_driving[i] -> m_path_ID) == m_ID_path_mapping.end()) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_truck, invalid path");
    //             }
    //             _p_path = m_ID_path_mapping.find(m_path_vec_driving[i] -> m_path_ID) -> second.second;
    //             if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) == nullptr) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_truck, invalid passenger path");
    //             }
    //             // double _tmp = dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) ->get_travel_time_truck(TFlt(start_prt[t]), m_mmdta)() * m_mmdta -> m_unit_time;
    //             double _tmp = dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) ->get_travel_time_truck(TFlt(start_prt[t]), m_mmdta, m_mmdue -> m_link_cost_map_truck)() * m_mmdta -> m_unit_time;
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
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_driving, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         for (size_t i = 0; i < m_path_vec_driving.size(); ++i){
    //             if (m_ID_path_mapping.find(m_path_vec_driving[i] -> m_path_ID) == m_ID_path_mapping.end()) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_driving, invalid path");
    //             }
    //             _p_path = m_ID_path_mapping.find(m_path_vec_driving[i] -> m_path_ID) -> second.second;
    //             if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) == nullptr) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_driving, invalid passenger path");
    //             }
    //             double _tmp = _p_path ->get_travel_time(TFlt(start_prt[t]), m_mmdta, m_mmdue -> m_link_cost_map, m_mmdue -> m_transitlink_cost_map)() * m_mmdta -> m_unit_time;
    //             result_prt[i * l + t] = _tmp;
    //             printf("path ID: %d, time interval (5 s): %d, path travel time: %f\n",
    //                    (int)m_path_vec_driving[i] -> m_path_ID, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_registered_path_tt_bustransit()
    // ******************************************************
    {
        int l = m_mmdta -> m_current_loading_interval();
        std::vector<double> start_prt = std::vector<double>();

        double result_prt[int(m_path_vec_bustransit.size()) * l];
        // int new_shape [2] = { (int) m_path_vec_bustransit.size(), l};
        // auto result = py::array_t<double>(new_shape);
        // auto result_buf = result.request();
        // double *result_prt = (double *) result_buf.ptr;

        MNM_Passenger_Path_Base *_p_path;
        for (int t = 0; t < l; ++t){
            start_prt.push_back((double)t);
            if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
                throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_bustransit, input start intervals exceeds the total loading intervals - 1");
            }
            for (size_t i = 0; i < m_path_vec_bustransit.size(); ++i){
                if (m_ID_path_mapping.find(m_path_vec_bustransit[i] -> m_path_ID) == m_ID_path_mapping.end()) {
                    throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_bustransit, invalid path");
                }
                _p_path = m_ID_path_mapping.find(m_path_vec_bustransit[i] -> m_path_ID) -> second.second;
                if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_Bus*>(_p_path) == nullptr) {
                    throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_bustransit, invalid passenger path");
                }
                double _tmp = _p_path ->get_travel_time(TFlt(start_prt[t]), m_mmdta, m_mmdue -> m_link_cost_map, m_mmdue -> m_transitlink_cost_map)() * m_mmdta -> m_unit_time;
                result_prt[i * l + t] = _tmp;
                printf("path ID: %d, time interval (5 s): %d, path travel time: %f\n",
                       (int)m_path_vec_bustransit[i] -> m_path_ID, t, result_prt[i * l + t]);
            }
        }
    }

    // ******************************************************
    // Mmdta_Api::get_registered_path_tt_pnr()
    // ******************************************************
    {
        int l = m_mmdta -> m_current_loading_interval();
        std::vector<double> start_prt = std::vector<double>();

        double result_prt[int(m_path_vec_pnr.size()) * l];
        // int new_shape [2] = { (int) m_path_vec_pnr.size(), l};
        // auto result = py::array_t<double>(new_shape);
        // auto result_buf = result.request();
        // double *result_prt = (double *) result_buf.ptr;

        MNM_Passenger_Path_Base *_p_path;
        for (int t = 0; t < l; ++t){
            start_prt.push_back((double)t);
            if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
                throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_pnr, input start intervals exceeds the total loading intervals - 1");
            }
            for (size_t i = 0; i < m_path_vec_pnr.size(); ++i){
                if (m_ID_path_mapping.find(m_path_vec_pnr[i] -> m_path_ID) == m_ID_path_mapping.end()) {
                    throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_pnr, invalid path");
                }
                _p_path = m_ID_path_mapping.find(m_path_vec_pnr[i] -> m_path_ID) -> second.second;
                if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_PnR*>(_p_path) == nullptr) {
                    throw std::runtime_error("Error, Mmdta_Api::get_registered_path_tt_pnr, invalid passenger path");
                }
                double _tmp = _p_path ->get_travel_time(TFlt(start_prt[t]), m_mmdta, m_mmdue -> m_link_cost_map, m_mmdue -> m_transitlink_cost_map)() * m_mmdta -> m_unit_time;
                result_prt[i * l + t] = _tmp;
                printf("path ID: %d, time interval (5 s): %d, path travel time: %f\n",
                       (int)m_path_vec_pnr[i] -> m_path_ID, t, result_prt[i * l + t]);
            }
        }
    }

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
    //         if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //             throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_driving, input start intervals exceeds the total loading intervals - 1");
    //         }
    //         for (size_t i = 0; i < m_path_vec_driving.size(); ++i){
    //             if (m_ID_path_mapping.find(m_path_vec_driving[i] -> m_path_ID) == m_ID_path_mapping.end()) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_driving, invalid path");
    //             }
    //             _p_path = m_ID_path_mapping.find(m_path_vec_driving[i] -> m_path_ID) -> second.second;
    //             if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) == nullptr) {
    //                 throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_driving, invalid passenger path");
    //             }
    //             double _tmp = _p_path ->get_travel_cost(TFlt(start_prt[t]), m_mmdta, m_mmdue -> m_link_cost_map, m_mmdue -> m_transitlink_cost_map)();
    //             result_prt[i * l + t] = _tmp;
    //             printf("path ID: %d, time interval (5 s): %d, path travel cost: %f\n",
    //                    (int)m_path_vec_driving[i] -> m_path_ID, t, result_prt[i * l + t]);
    //         }
    //     }
    // }

    // ******************************************************
    // Mmdta_Api::get_registered_path_cost_bustransit()
    // ******************************************************
    {
        int l = m_mmdta -> m_current_loading_interval();
        std::vector<double> start_prt = std::vector<double>();

        double result_prt[int(m_path_vec_bustransit.size()) * l];
        // int new_shape [2] = { (int) m_path_vec_bustransit.size(), l};
        // auto result = py::array_t<double>(new_shape);
        // auto result_buf = result.request();
        // double *result_prt = (double *) result_buf.ptr;

        MNM_Passenger_Path_Base *_p_path;
        for (int t = 0; t < l; ++t){
            start_prt.push_back((double)t);
            if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
                throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_bustransit, input start intervals exceeds the total loading intervals - 1");
            }
            for (size_t i = 0; i < m_path_vec_bustransit.size(); ++i){
                if (m_ID_path_mapping.find(m_path_vec_bustransit[i] -> m_path_ID) == m_ID_path_mapping.end()) {
                    throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_bustransit, invalid path");
                }
                _p_path = m_ID_path_mapping.find(m_path_vec_bustransit[i] -> m_path_ID) -> second.second;
                if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_Bus*>(_p_path) == nullptr) {
                    throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_bustransit, invalid passenger path");
                }
                double _tmp = _p_path ->get_travel_cost(TFlt(start_prt[t]), m_mmdta, m_mmdue -> m_link_cost_map, m_mmdue -> m_transitlink_cost_map)();
                result_prt[i * l + t] = _tmp;
                printf("path ID: %d, time interval (5 s): %d, path travel cost: %f\n",
                       (int)m_path_vec_bustransit[i] -> m_path_ID, t, result_prt[i * l + t]);
            }
        }
    }

    // ******************************************************
    // Mmdta_Api::get_registered_path_cost_pnr()
    // ******************************************************
    {
        int l = m_mmdta -> m_current_loading_interval();
        std::vector<double> start_prt = std::vector<double>();

        double result_prt[int(m_path_vec_pnr.size()) * l];
        // int new_shape [2] = { (int) m_path_vec_pnr.size(), l};
        // auto result = py::array_t<double>(new_shape);
        // auto result_buf = result.request();
        // double *result_prt = (double *) result_buf.ptr;

        MNM_Passenger_Path_Base *_p_path;
        for (int t = 0; t < l; ++t){
            start_prt.push_back((double)t);
            if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
                throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_pnr, input start intervals exceeds the total loading intervals - 1");
            }
            for (size_t i = 0; i < m_path_vec_pnr.size(); ++i){
                if (m_ID_path_mapping.find(m_path_vec_pnr[i] -> m_path_ID) == m_ID_path_mapping.end()) {
                    throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_pnr, invalid path");
                }
                _p_path = m_ID_path_mapping.find(m_path_vec_pnr[i] -> m_path_ID) -> second.second;
                if (_p_path == nullptr || dynamic_cast<MNM_Passenger_Path_PnR*>(_p_path) == nullptr) {
                    throw std::runtime_error("Error, Mmdta_Api::get_registered_path_cost_pnr, invalid passenger path");
                }
                double _tmp = _p_path ->get_travel_cost(TFlt(start_prt[t]), m_mmdta, m_mmdue -> m_link_cost_map, m_mmdue -> m_transitlink_cost_map)();
                result_prt[i * l + t] = _tmp;
                printf("path ID: %d, time interval (5 s): %d, path travel cost: %f\n",
                       (int)m_path_vec_pnr[i] -> m_path_ID, t, result_prt[i * l + t]);
            }
        }
    }

    // ******************************************************
    // Mmdta_Api::get_car_ltg_matrix_driving()
    // ******************************************************
    // {
    //     // input: intervals in which the agents are released for each path, 1 min interval
    //     int l = m_mmdta -> m_current_loading_interval();
    //     std::vector<int> start_prt = std::vector<int>();
    //     for (int k = 0; k < l; ++k) {
    //         if (k % 12 == 0) {  // 1 min interval
    //             start_prt.push_back(k);
    //         }
    //     }

    //     std::vector<ltg_record*> _record = std::vector<ltg_record*>();
    //     bool _flg; 
    //     TFlt _fftt, _gradient;
    //     int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up, _t_depart_prime,  _t_queue_dissipated_valid, _t_depart_lift_up_valid;
    //     for (auto *_path : m_path_vec_driving) {
    //         // check if the path does not include any link in m_link_vec_driving
    //         _flg = false;
    //         for (auto *_link : m_link_vec_driving) {
    //             if (_path -> is_link_in(_link -> m_link_ID)) {
    //                 _flg = true;
    //                 break;
    //             }
    //         }
    //         if (!_flg) {
    //             continue;
    //         }

    //         for (int t = 0; t < (int)start_prt.size(); ++t){
    //             // printf("Current processing time: %d\n", t);
    //             if (start_prt[t] >= m_mmdta -> m_current_loading_interval()){
    //                 throw std::runtime_error("Error, Mmdta_Api::get_car_ltg_matrix_driving, input start intervals exceeds the total loading intervals - 1");
    //             }

    //             _t_arrival = -1, _t_depart = -1, _t_arrival_lift_up = -1, _t_depart_lift_up = -1, _t_depart_prime = -1;
    //             // trace one additional veh departing from origin of path at start_prt[t]
    //             _t_depart = start_prt[t];
    //             for (TInt _link_ID : _path -> m_link_vec) {
    //                 // arrival and departure time of original perturbation vehicle
    //                 _t_arrival = _t_depart;
    //                 _t_depart = _t_arrival + MNM_Ults::round_up_time(m_mmdue -> m_link_cost_map[_link_ID][_t_arrival < m_mmdta -> m_current_loading_interval() ? _t_arrival : m_mmdta -> m_current_loading_interval() - 1]);
                    
    //                 // arrival time of the new perturbation vehicle
    //                 auto *_link = dynamic_cast<MNM_Dlink_Multiclass*>(m_mmdta -> m_link_factory -> get_link(_link_ID));
    //                 if (dynamic_cast<MNM_Dlink_Pq_Multimodal*>(_link) != nullptr) {
    //                     _t_arrival_lift_up = _t_arrival;  // for last pq, _t_arrival_lift_up >= get_cur_loading_interval()
    //                 }
    //                 else {
    //                     IAssert(dynamic_cast<MNM_Dlink_Ctm_Multimodal*>(_link) != nullptr);
    //                     IAssert(_t_depart_lift_up >= 0);  // from its upstream link
    //                     _t_arrival_lift_up = _t_depart_lift_up;
    //                 }
    //                 IAssert(_t_arrival_lift_up >= _t_arrival);

    //                 IAssert(_link -> m_last_valid_time > 0);
    //                 if (_t_arrival_lift_up > int(round(_link -> m_last_valid_time - 1))) {
    //                     break;
    //                 }

    //                 // departure time of new perturbation vehicle
    //                 _t_depart_prime = _t_arrival_lift_up + MNM_Ults::round_up_time(m_mmdue -> m_link_cost_map[_link_ID][_t_arrival_lift_up < m_mmdta -> m_current_loading_interval() ? _t_arrival_lift_up : m_mmdta -> m_current_loading_interval() - 1]);

    //                 // arrival time of the NEXT new perturbation for the NEXT link
    //                 _fftt = dynamic_cast<MNM_Dlink_Multiclass*>(m_mmdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_loading_car();
    //                 // _fftt = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(m_mmdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_car() / m_mmdue -> m_unit_time;
    //                 _t_depart_lift_up = m_mmdue -> m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] + MNM_Ults::round_up_time(_fftt);
                        
    //                 if (!m_mmdue -> m_link_congested_car[_link_ID][_t_arrival_lift_up]) {
    //                     if (m_mmdue -> m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] == _t_arrival_lift_up) {
    //                         IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) == _t_depart_lift_up);
    //                     }
    //                     else {
    //                         // critical state where subgradient applies
    //                         IAssert(m_mmdue -> m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
    //                         IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
    //                     }
    //                 }
    //                 else {
    //                     IAssert(m_mmdue -> m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
    //                     IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
    //                 }

    //                 if (_t_depart_lift_up < _t_depart_prime) {
    //                     printf("Error, Mmdta_Api::get_car_ltg_matrix_driving, something is wrong");
    //                     exit(-1);
    //                 }

    //                 if (_t_depart_prime < m_mmdta -> m_current_loading_interval() - 1 &&
    //                     std::find_if(m_link_vec_driving.begin(), m_link_vec_driving.end(), 
    //                                 [&_link_ID](const MNM_Dlink_Multiclass *_l){return _l -> m_link_ID == _link_ID;}) != m_link_vec_driving.end()) {
    //                     if (m_mmdue -> m_link_congested_car[_link_ID][_t_arrival_lift_up] && _t_depart_lift_up > _t_depart_prime) {

    //                         // std::cout << "\ninterval: " << start_prt[t] << ", link: " << _link_ID << "\n";
    //                         // std::cout << "car in" << "\n";
    //                         // std::cout << _link -> m_N_in_car -> to_string() << "\n";
    //                         // std::cout << "car out" << "\n";
    //                         // std::cout << _link -> m_N_out_car -> to_string() << "\n";

    //                         IAssert(_link -> m_last_valid_time > 0);
    //                         if (m_mmdue -> m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] <= int(round(_link -> m_last_valid_time - 1))) {
    //                             _t_queue_dissipated_valid = m_mmdue -> m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up];
    //                             _t_depart_lift_up_valid = _t_depart_lift_up;
    //                         }
    //                         else {
    //                             _t_queue_dissipated_valid = int(round(_link -> m_last_valid_time));
    //                             _t_depart_lift_up_valid = _t_queue_dissipated_valid - 1 + MNM_Ults::round_up_time(m_mmdue -> m_link_cost_map[_link_ID][_t_queue_dissipated_valid - 1 < m_mmdta -> m_current_loading_interval() ? _t_queue_dissipated_valid - 1 : m_mmdta -> m_current_loading_interval() - 1]);
    //                         }
    //                         IAssert(_t_depart_lift_up_valid <= m_mmdta -> m_current_loading_interval() - 1);
    //                         IAssert(_t_arrival_lift_up < _t_queue_dissipated_valid);
    //                         IAssert(_t_depart_prime <= _t_depart_lift_up_valid);
    //                         if (_t_depart_prime < _t_depart_lift_up_valid) {

    //                             // _gradient = MNM_DTA_GRADIENT::get_departure_cc_slope_car(_link, TFlt(_t_depart_prime), TFlt(_t_depart_lift_up_valid + 1));
    //                             // if (_gradient > DBL_EPSILON) {
    //                             //     _gradient = m_mmdue -> m_unit_time / _gradient;  // seconds
    //                             //     for (int t_prime = _t_arrival_lift_up; t_prime < _t_queue_dissipated_valid; ++t_prime) {
    //                             //         MNM_DTA_GRADIENT::add_ltg_records_veh(_record, _link, _path, start_prt[t], t_prime, _gradient);
    //                             //     }
    //                             // }

    //                             for (int t_prime = _t_arrival_lift_up; t_prime < _t_queue_dissipated_valid; ++t_prime) {
    //                                 _gradient = MNM_DTA_GRADIENT::get_departure_cc_slope_car(_link, 
    //                                                                                          TFlt(t_prime + MNM_Ults::round_up_time(m_mmdue -> m_link_cost_map[_link_ID][t_prime < m_mmdta -> m_current_loading_interval() ? t_prime : m_mmdta -> m_current_loading_interval() - 1])), 
    //                                                                                          TFlt(t_prime + MNM_Ults::round_up_time(m_mmdue -> m_link_cost_map[_link_ID][t_prime < m_mmdta -> m_current_loading_interval() ? t_prime : m_mmdta -> m_current_loading_interval() - 1]) + 1) 
    //                                                                                          );
    //                                 if (_gradient > DBL_EPSILON) {
    //                                     _gradient = m_mmdue -> m_unit_time / _gradient;  // seconds
    //                                     MNM_DTA_GRADIENT::add_ltg_records_veh(_record, _link, _path, start_prt[t], t_prime, _gradient);
    //                                 }
    //                             }
                                
    //                         }
                            
    //                     }
    //                 }
    //             }
                
    //         }
    //     }

    //     // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
    //     // path_ID, assign_time, link_ID, start_int, gradient
    //     double result_prt[(int) _record.size() * 5];
    //     ltg_record* tmp_record;
    //     for (size_t i = 0; i < _record.size(); ++i){
    //         tmp_record = _record[i];
    //         result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    //         // the count of 1 min interval
    //         result_prt[i * 5 + 1] = (double) tmp_record -> assign_int;
    //         result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    //         // the count of unit time interval (5s)
    //         result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int;
    //         result_prt[i * 5 + 4] = tmp_record -> gradient();
    //         printf("path ID: %f, departure assign interval (5 s): %f, link ID: %f, time interval (5 s): %f, gradient: %f\n",
    //                 result_prt[i * 5 + 0], result_prt[i * 5 + 1], result_prt[i * 5 + 2], result_prt[i * 5 + 3], result_prt[i * 5 + 4]);
    //     }
    //     for (size_t i = 0; i < _record.size(); ++i){
    //         delete _record[i];
    //     }
    //     _record.clear();
    //     // return result;
    // }

};