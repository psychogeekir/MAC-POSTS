#include "due.h"
#include "dta_gradient_utls.h"
#include "ults.h"

// #include <assert.h>
#include "limits.h"

MNM_Due::MNM_Due(std::string file_folder) {
    m_file_folder = file_folder;
    m_dta_config = new MNM_ConfReader(m_file_folder + "/config.conf", "DTA");
    IAssert(m_dta_config->get_string("routing_type") == "Due");
    IAssert(m_dta_config->get_int("total_interval") > 0);
    IAssert(m_dta_config->get_int("total_interval") >=
            m_dta_config->get_int("assign_frq") * m_dta_config->get_int("max_interval"));
    m_unit_time = m_dta_config->get_float("unit_time");
    m_total_loading_inter = m_dta_config->get_int("total_interval");
    m_due_config = new MNM_ConfReader(m_file_folder + "/config.conf", "DUE");
    m_total_assign_inter = m_dta_config->get_int("max_interval");
    m_path_table = nullptr;
    // m_od_factory = nullptr;
    m_early_rate = m_due_config->get_float("early_rate");
    m_late_rate = m_due_config->get_float("late_rate");
    m_target_time = m_due_config->get_float("target_time");
    m_step_size = m_due_config->get_float("lambda");  //0.01;  // MSA
    m_cost_map = std::unordered_map<TInt, TFlt *>();  // time-varying link cost
}

// int MNM_Due::init_path_table()
// {
//   std::string _file_name = m_due_conf -> get_string("path_file_name");
//   std::string _whole_path = m_file_folder + "/" + _file_name;
//   TInt _num_path = m_due_conf -> get_int("num_path");
//   m_path_table = MNM_IO::load_path_table(_whole_path, m_graph, _num_path);
//   return 0;
// }




MNM_Due::~MNM_Due() {
    if (m_dta_config != nullptr) delete m_dta_config;
    if (m_due_config != nullptr) delete m_due_config;
    // if (m_od_factory != nullptr) delete m_od_factory;
}

// int MNM_Due::initialize()
// {
//   // m_graph = MNM_IO::build_graph(m_file_folder, m_dta_config);
//   // MNM_IO::build_od_factory(m_file_folder, m_dta_config, m_od_factory);
//   return 0;
// }

MNM_Dta *MNM_Due::run_dta(bool verbose) {
    MNM_Dta *_dta = new MNM_Dta(m_file_folder);
    // printf("dd\n");
    _dta->build_from_files();
    // _dta -> m_od_factory = m_od_factory;
    // printf("ddd\n");
    update_demand_from_path_table(_dta);

    _dta->m_routing->init_routing(m_path_table);
    // printf("dddd\n");
    _dta->hook_up_node_and_link();  // in and out links for node, beginning and ending nodes for link
    // printf("Checking......\n");
    // _dta -> is_ok();

    for (auto _link_it = _dta->m_link_factory->m_link_map.begin();
         _link_it != _dta->m_link_factory->m_link_map.end(); _link_it++) {
        _link_it->second->install_cumulative_curve();
    }

    _dta->pre_loading();  // initiate record file, junction model for node, and vehicle queue for link
    _dta->loading(verbose);
    return _dta;
}


int MNM_Due::update_demand_from_path_table(MNM_Dta *dta) {
    TFlt _tot_dmd;
    MNM_Origin *_org;
    MNM_Destination *_dest;
    MNM_Path *_one_path;
    for (auto _it : *m_path_table) {  // origin node
        for (auto _it_it : *(_it.second)) {  // destination node
            for (int _col = 0; _col < m_total_assign_inter; _col++) {
                _tot_dmd = 0.0;
                for (MNM_Path *_path : _it_it.second->m_path_vec) {  // path
                    _tot_dmd += _path->m_buffer[_col];
                }
                if (_it_it.second->m_path_vec.size() > 0) {
                    _one_path = _it_it.second->m_path_vec[0];
                    _org = ((MNM_DMOND *) dta->m_node_factory->get_node(_one_path->m_node_vec.front()))->m_origin;
                    _dest = ((MNM_DMDND *) dta->m_node_factory->get_node(_one_path->m_node_vec.back()))->m_dest;
                    _org->m_demand[_dest][_col] = _tot_dmd;
                }
            }
        }
    }
    return 0;
}

TFlt MNM_Due::compute_total_demand(MNM_Origin *orig, MNM_Destination *dest, TInt total_assign_inter) {
    TFlt _tot_dmd = 0.0;
    for (int i = 0; i < total_assign_inter(); ++i) {
        _tot_dmd += orig->m_demand[dest][i];
    }
    return _tot_dmd;
}

TFlt MNM_Due::compute_merit_function() {
    TFlt _tt, _depart_time, _dis_utl, _lowest_dis_utl;
    TFlt _total_gap = 0.0;
    for (auto _it : *m_path_table) {
        for (auto _it_it : *(_it.second)) {
            _lowest_dis_utl = DBL_MAX;
            for (int _col = 0; _col < m_total_assign_inter; _col++) {
                _depart_time = TFlt(_col * m_dta_config->get_int("assign_frq"));
                for (MNM_Path *_path : _it_it.second->m_path_vec) {  // get lowest disutility route at time interval _col and current OD pair
                    _tt = get_tt(_depart_time, _path);
                    _dis_utl = get_disutility(_depart_time, _tt);
                    if (_dis_utl < _lowest_dis_utl) _lowest_dis_utl = _dis_utl;
                }
                for (MNM_Path *_path : _it_it.second->m_path_vec) {
                    _tt = get_tt(_depart_time, _path);
                    _dis_utl = get_disutility(_depart_time, _tt);
                    _total_gap += (_dis_utl - _lowest_dis_utl) * _path->m_buffer[_col];
                }
            }
        }
    }
    return _total_gap;
}

TFlt MNM_Due::compute_merit_function_fixed_departure_time_choice() {
    TFlt _tt, _depart_time, _dis_utl, _lowest_dis_utl;
    TFlt _total_gap = 0.0;
    TFlt _min_flow_cost = 0.0;
    for (auto _it : *m_path_table) {
        for (auto _it_it : *(_it.second)) {
            for (int _col = 0; _col < m_total_assign_inter; _col++) {
                _lowest_dis_utl = DBL_MAX;
                _depart_time = TFlt(_col * m_dta_config->get_int("assign_frq"));
                for (MNM_Path *_path : _it_it.second->m_path_vec) {  // get lowest disutility route at time interval _col and current OD pair
                    if (_path -> m_buffer[_col] > 0) {
//                        for (int i=0; i<m_dta_config->get_int("assign_frq"); i++) {
//                            _tt = get_tt(_depart_time + TFlt(i), _path);
//                            _dis_utl = get_disutility(_depart_time + TFlt(i), _tt);
//                            if (_dis_utl < _lowest_dis_utl) _lowest_dis_utl = _dis_utl;
//                        }
                        _tt = get_tt(_depart_time, _path);
                        _dis_utl = get_disutility(_depart_time, _tt);
                        if (_dis_utl < _lowest_dis_utl) _lowest_dis_utl = _dis_utl;
                    }
                }
                for (MNM_Path *_path : _it_it.second->m_path_vec) {
                    if (_path -> m_buffer[_col] > 0) {
                        _tt = get_tt(_depart_time, _path);
                        _dis_utl = get_disutility(_depart_time, _tt);
                        _total_gap += (_dis_utl - _lowest_dis_utl) * _path->m_buffer[_col];
                        _min_flow_cost += _lowest_dis_utl * _path->m_buffer[_col];
                    }
                }
            }
        }
    }
    return _total_gap / _min_flow_cost;
}

TFlt MNM_Due::get_disutility(TFlt depart_time, TFlt tt) {
    TFlt _arrival_time = depart_time + tt;
    if (_arrival_time >= m_target_time) {
        return tt + m_late_rate * (_arrival_time - m_target_time);
    } else {
        return tt + m_early_rate * (m_target_time - _arrival_time);
    }
    throw std::runtime_error("Error in MNM_Due::get_disutility");
}


TFlt MNM_Due::get_tt(TFlt depart_time, MNM_Path *path) {
    // true path tt
    TFlt _cur_time = depart_time;
    TInt _query_time;
    for (TInt _link_ID : path->m_link_vec) {
        // rounding down, consistent with MNM_TDSP_Tree::update_tree() in shortest_path.cpp
        _query_time = MNM_TDSP_Tree::round_time(_cur_time, m_total_loading_inter);
        // original rounding down
        // _query_time = MNM_Ults::min(TInt(_cur_time), TInt(m_total_loading_inter - 1));
        _cur_time += m_cost_map[_link_ID][_query_time];
    }
    return _cur_time - depart_time;
}


int MNM_Due::build_cost_map(MNM_Dta *dta) {
    MNM_Dlink *_link;
    for (int i = 0; i < m_total_loading_inter; i++) {
        // std::cout << "********************** interval " << i << " **********************\n";
        for (auto _link_it : dta->m_link_factory->m_link_map) {
            _link = _link_it.second;
            m_cost_map[_link_it.first][i] = MNM_DTA_GRADIENT::get_travel_time(_link, TFlt(i), m_unit_time) ;
            // std::cout << "interval: " << i << ", link: " << _link_it.first << ", tt: " << m_cost_map[_link_it.first][i] << "\n";
        }
    }
    return 0;
}
//*************************************************************************
//                                MNM_Due_Msa
//*************************************************************************

MNM_Due_Msa::MNM_Due_Msa(std::string file_folder)
        : MNM_Due::MNM_Due(file_folder) {
    m_base_dta = NULL;
}

MNM_Due_Msa::~MNM_Due_Msa() {
    for (auto _cost_it: m_cost_map) {
        delete _cost_it.second;
    }
    m_cost_map.clear();
    delete m_base_dta;
}


int MNM_Due_Msa::initialize() {
    m_base_dta = new MNM_Dta(m_file_folder);
    m_base_dta->build_from_files();
    m_path_table = MNM::build_shortest_pathset(m_base_dta->m_graph,
                                               m_base_dta->m_od_factory, m_base_dta->m_link_factory);
    MNM::allocate_path_table_buffer(m_path_table, m_total_assign_inter);  // create zero buffer
    // MNM::save_path_table(m_path_table, m_base_dta -> m_od_factory, true);

    for (auto _link_it : m_base_dta->m_link_factory->m_link_map) {
        m_cost_map[_link_it.first] = new TFlt[m_total_loading_inter];
    }
    // switch(m_due_config -> get_int("init_demand_split")){
    //   case 0:
    //     m_od_factory = m_base_dta -> m_od_factory;
    //     m_base_dta -> m_od_factory = NULL; // avoiding destructor
    //     break;
    //   case 1:
    //     throw std::runtime_error("Not implemented yet");
    //     break;
    //   default:
    //     throw std::runtime_error("Unknown init method");
    // }

    // printf("%d, %d\n", m_od_factory -> m_origin_map.size(), m_od_factory -> m_destination_map.size());
    printf("finish initialize\n");
    return 0;
}

// spread the OD demand evenly over the initial paths
int MNM_Due_Msa::init_path_flow() {
    TFlt _len, _dmd;
    MNM_Origin *_org;
    MNM_Destination *_dest;
    for (auto _it : *m_path_table) {
        // printf("22m\n");
        for (auto _it_it : *(_it.second)) {
            // printf("23m\n");
            _len = TFlt(_it_it.second->m_path_vec.size());
            for (MNM_Path *_path : _it_it.second->m_path_vec) {
                // printf("24m\n");
                _org = ((MNM_DMOND *) m_base_dta->m_node_factory->get_node(_path->m_node_vec.front()))->m_origin;
                _dest = ((MNM_DMDND *) m_base_dta->m_node_factory->get_node(_path->m_node_vec.back()))->m_dest;
                for (int _col = 0; _col < m_total_assign_inter; _col++) {
                    // printf("25m\n");
                    if (_org->m_demand.find(_dest) ==
                        _org->m_demand.end()) {  // if this OD pair not in demand file, demand = 0
                        _dmd = TFlt(0.0);
                    } else {
                        _dmd = _org->m_demand[_dest][_col];
                    }
                    // printf("%lf\n", _dmd());
                    _path->m_buffer[_col] = TFlt(1.0) / _len * _dmd;
                }
            }
        }
    }
    // MNM::save_path_table(m_path_table, m_od_factory, true);
    printf("Finish init route choice\n");
    return 0;
}


// best route for all assignment intervals
std::pair<MNM_Path *, TInt> MNM_Due_Msa::get_best_route(TInt o_node_ID,
                                                        MNM_TDSP_Tree *tdsp_tree) {
    TFlt _cur_best_cost = TFlt(std::numeric_limits<double>::max());
    TInt _cur_best_time = -1;
    TFlt _tmp_tt, _tmp_cost;
    for (int i = 0; i < tdsp_tree->m_max_interval; ++i) {  //tdsp_tree -> m_max_interval = total_loading_interval
        _tmp_tt = tdsp_tree->get_distance_to_destination(o_node_ID, TFlt(i));
        std::cout << "interval: " << i <<", tdsp_tt: "<< _tmp_tt <<"\n";
        _tmp_cost = get_disutility(TFlt(i), _tmp_tt);
        if (_tmp_cost < _cur_best_cost) {
            _cur_best_cost = _tmp_cost;
            _cur_best_time = i;
        }
    }
    IAssert(_cur_best_time >= 0);
    MNM_Path *_path = new MNM_Path();
    tdsp_tree->get_tdsp(o_node_ID, _cur_best_time, m_cost_map, _path);
    std::pair<MNM_Path *, TInt> _best = std::make_pair(_path, _cur_best_time);
    return _best;
}

// best route for a single interval
std::pair<MNM_Path *, TInt> MNM_Due_Msa::get_best_route_for_single_interval(
        TInt interval, TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree) {

    IAssert(interval + m_base_dta -> m_assign_freq < tdsp_tree->m_max_interval);  // tdsp_tree -> m_max_interval = total_loading_interval
    TFlt _cur_best_cost = TFlt(std::numeric_limits<double>::max());
    TInt _cur_best_time = -1;
    TFlt _tmp_tt, _tmp_cost;
    for (int i = interval; i < interval + 1; ++i) {
    // for (int i = interval; i < interval + m_base_dta -> m_assign_freq; ++i) {  // tdsp_tree -> m_max_interval = total_loading_interval
        _tmp_tt = tdsp_tree->get_distance_to_destination(o_node_ID, TFlt(i));
        std::cout << "interval: " << i <<", tdsp_tt: "<< _tmp_tt <<"\n";
        _tmp_cost = get_disutility(TFlt(i), _tmp_tt);
        if (_tmp_cost < _cur_best_cost) {
            _cur_best_cost = _tmp_cost;
            _cur_best_time = i;
        }
    }
    IAssert(_cur_best_time >= 0);
    MNM_Path *_path = new MNM_Path();
    tdsp_tree->get_tdsp(o_node_ID, _cur_best_time, m_cost_map, _path);
    std::pair<MNM_Path *, TInt> _best = std::make_pair(_path, _cur_best_time);
    return _best;
}


int MNM_Due_Msa::update_path_table(MNM_Dta *dta, int iter) {
    MNM_Origin *_orig;
    MNM_Destination *_dest;
    TInt _orig_node_ID, _dest_node_ID;
    std::pair<MNM_Path *, TInt> _path_result;
    MNM_Path *_path;
    MNM_Pathset *_path_set;
    TFlt _tot_change, _tmp_change, _tot_oneOD_demand, _len;
    MNM_Path *_best_path;
    int _best_time_col;
    int _best_assign_col;
    for (auto _it : dta->m_od_factory->m_destination_map) {
        _dest = _it.second;
        _dest_node_ID = _dest->m_dest_node->m_node_ID;
        MNM_TDSP_Tree *_tdsp_tree = new MNM_TDSP_Tree(_dest_node_ID, dta->m_graph, m_total_loading_inter);
        _tdsp_tree->initialize();
        build_cost_map(dta);
        // printf("111\n");
        _tdsp_tree->update_tree(m_cost_map);
        for (auto _map_it : dta->m_od_factory->m_origin_map) {
            _orig = _map_it.second;
            _orig_node_ID = _orig->m_origin_node->m_node_ID;

            // if no demand for this OD pair
            if (_orig->m_demand.find(_dest) == _orig->m_demand.end()) {
                continue;
            }

            _path_result = get_best_route(_orig_node_ID, _tdsp_tree);
            _path = _path_result.first;
            _best_time_col = int(_path_result.second);
            _best_assign_col = floor(_best_time_col / m_base_dta->m_assign_freq);
            // printf("Best time col %d\n", _best_time_col);

            _path_set = MNM::get_pathset(m_path_table, _orig_node_ID, _dest_node_ID);

            _tot_oneOD_demand = compute_total_demand(_orig, _dest, m_total_assign_inter);
            _tot_change = 0.0;
            _best_path = NULL;

            if (_path_set->is_in(_path)) {
                printf("Update current pathset\n");
                for (auto _tmp_path : _path_set->m_path_vec) {
                    for (int _col = 0; _col < m_total_assign_inter; _col++) {
                        if ((!(*_tmp_path == *_path)) || (_col != _best_assign_col)) {
                            // printf("dd %lf\n", _tmp_path -> m_buffer[_col]());
                            // printf("iter %d\n", iter);
                            // Original
                            _tmp_change = MNM_Ults::min(_tmp_path -> m_buffer[_col], _tot_oneOD_demand * m_step_size / TFlt(iter + 1));
                            // Modified by Zou
                            // _tmp_change = _tmp_path->m_buffer[_col] * m_step_size / TFlt(iter + 1);
                            // printf("tmp change %lf\n", _tmp_change());
                            _tmp_path->m_buffer[_col] -= _tmp_change;
                            _tot_change += _tmp_change;
                        }
                    }
                    if (*_tmp_path == *_path) {
                        _best_path = _tmp_path;
                    }
                }
                // printf("dddd, %lf\n", _tot_change());
                _best_path->m_buffer[_best_assign_col] += _tot_change;
                // printf("sssss\n");
                delete _path;
            } else {
                printf("Adding new path\n");

                // Original
                _path -> allocate_buffer(m_total_assign_inter);
                _path -> m_buffer[_best_assign_col] += m_step_size / TFlt(iter + 1);
                _len = TFlt(_path_set -> m_path_vec.size());
                for (auto tmp_path : _path_set -> m_path_vec){
                  tmp_path -> m_buffer[_best_assign_col] -= m_step_size / TFlt(iter + 1) / _len;
                }
                _path_set->m_path_vec.push_back(_path);

                // Modified by Zou
//                _path->allocate_buffer(m_total_assign_inter);
//                _path_set->m_path_vec.push_back(_path);
//                for (auto _tmp_path : _path_set->m_path_vec) {
//                    for (int _col = 0; _col < m_total_assign_inter; _col++) {
//                        if ((!(*_tmp_path == *_path)) || (_col != _best_assign_col)) {
//                            _tmp_change = _tmp_path->m_buffer[_col] * m_step_size / TFlt(iter + 1);
//                            // printf("tmp change %lf\n", _tmp_change());
//                            _tmp_path->m_buffer[_col] -= _tmp_change;
//                            _tot_change += _tmp_change;
//                        }
//                    }
//                    if (*_tmp_path == *_path) {
//                        _best_path = _tmp_path;
//                    }
//                }
//                _best_path->m_buffer[_best_assign_col] += _tot_change;
            }
        }
    }
    MNM::print_path_table(m_path_table, dta->m_od_factory, true);
    MNM::save_path_table(dta -> m_file_folder + "/" + dta -> m_statistics -> m_self_config -> get_string("rec_folder"), m_path_table, dta->m_od_factory, true);
    return 0;
}


int MNM_Due_Msa::update_path_table_fixed_departure_time_choice(MNM_Dta *dta, int iter) {
    MNM_Origin *_orig;
    MNM_Destination *_dest;
    TInt _orig_node_ID, _dest_node_ID;
    std::pair<MNM_Path *, TInt> _path_result;
    MNM_Path *_path;
    MNM_Pathset *_path_set;
    TFlt _tot_change, _tmp_change;
    MNM_Path *_best_path;

    for (auto _it : dta->m_od_factory->m_destination_map) {
        _dest = _it.second;
        _dest_node_ID = _dest->m_dest_node->m_node_ID;

        MNM_TDSP_Tree *_tdsp_tree = new MNM_TDSP_Tree(_dest_node_ID, dta->m_graph, m_total_loading_inter);
        _tdsp_tree->initialize();
        build_cost_map(dta);
        _tdsp_tree->update_tree(m_cost_map);

        for (auto _map_it : dta->m_od_factory->m_origin_map) {
            _orig = _map_it.second;
            _orig_node_ID = _orig->m_origin_node->m_node_ID;

            // if no demand for this OD pair
            if (_orig->m_demand.find(_dest) == _orig->m_demand.end()) {
                continue;
            }

            _path_set = MNM::get_pathset(m_path_table, _orig_node_ID, _dest_node_ID);

            for (int _col = 0; _col < m_total_assign_inter; _col++) {

                _tot_change = 0.0;
                _best_path = NULL;

                _path_result = get_best_route_for_single_interval(_col * m_base_dta -> m_assign_freq, _orig_node_ID, _tdsp_tree);
                _path = _path_result.first;

                if (_path_set->is_in(_path)) {
                    printf("Update current pathset\n");
                } else {
                    printf("Adding new path\n");
                    _path->allocate_buffer(m_total_assign_inter);
                    _path_set->m_path_vec.push_back(_path);
                }

                for (auto _tmp_path : _path_set->m_path_vec) {
                    if (!(*_tmp_path == *_path)) {
                        // printf("dd %lf\n", _tmp_path -> m_buffer[_col]());
                        // printf("iter %d\n", iter);
                        _tmp_change = _tmp_path->m_buffer[_col] * m_step_size / sqrt(TFlt(iter + 1));
                        // printf("tmp change %lf\n", _tmp_change());
                        _tmp_path->m_buffer[_col] -= _tmp_change;
                        _tot_change += _tmp_change;
                    }else {
                        _best_path = _tmp_path;
                    }
                }
                _best_path->m_buffer[_col] += _tot_change;
            }
        }
    }
    MNM::print_path_table(m_path_table, dta->m_od_factory, true);
    MNM::save_path_table(dta -> m_file_folder + "/" + dta -> m_statistics -> m_self_config -> get_string("rec_folder"), m_path_table, dta->m_od_factory, true);
    return 0;
}



int MNM_Due_Msa::update_path_table_gp_fixed_departure_time_choice(MNM_Dta *dta, int iter) {
    MNM_Origin *_orig;
    MNM_Destination *_dest;
    TInt _orig_node_ID, _dest_node_ID, _tot_nonzero_path;
    std::pair<MNM_Path *, TInt> _path_result;
    MNM_Path *_path;
    MNM_Pathset *_path_set;
    TFlt _tot_path_cost, _tmp_change, _tau, _tmp_tt, _tmp_cost, _min_flow;

    for (auto _it : dta->m_od_factory->m_destination_map) {
        _dest = _it.second;
        _dest_node_ID = _dest->m_dest_node->m_node_ID;

        MNM_TDSP_Tree *_tdsp_tree = new MNM_TDSP_Tree(_dest_node_ID, dta->m_graph, m_total_loading_inter);
        _tdsp_tree->initialize();
        build_cost_map(dta);
        _tdsp_tree->update_tree(m_cost_map);

        for (auto _map_it : dta->m_od_factory->m_origin_map) {
            _orig = _map_it.second;
            _orig_node_ID = _orig->m_origin_node->m_node_ID;

            // if no demand for this OD pair
            if (_orig->m_demand.find(_dest) == _orig->m_demand.end()) {
                continue;
            }

            _path_set = MNM::get_pathset(m_path_table, _orig_node_ID, _dest_node_ID);

            for (int _col = 0; _col < m_total_assign_inter; _col++) {

                _path_result = get_best_route_for_single_interval(_col * m_base_dta -> m_assign_freq, _orig_node_ID, _tdsp_tree);
                _path = _path_result.first;

                _tot_path_cost = 0.0;
                _tot_nonzero_path = 0;
                _tau = TFlt(std::numeric_limits<double>::max());
                _min_flow = TFlt(std::numeric_limits<double>::max());
                _tmp_change = 0.0;
                if (_path_set->is_in(_path)) {
                    printf("Update current pathset\n");

                } else {
                    printf("Adding new path\n");
                    _path->allocate_buffer(m_total_assign_inter);
                    _path_set->m_path_vec.push_back(_path);
                }

                // average path cost
                for (auto _tmp_path : _path_set->m_path_vec) {
                    if ((*_tmp_path == *_path) || (_tmp_path->m_buffer[_col] > 0)) {
                        _tmp_tt = get_tt(_col * m_base_dta -> m_assign_freq, _tmp_path);
                        printf("path in pathset, tt %lf\n", (float)_tmp_tt);
                        _tmp_cost = get_disutility(TFlt(_col * m_base_dta -> m_assign_freq), _tmp_tt);
                        _tot_path_cost += _tmp_cost;
                        _tot_nonzero_path += 1;
                        if ((_tmp_path->m_buffer[_col] > 0) && (_min_flow > _tmp_path->m_buffer[_col])) {
                            _min_flow = _tmp_path->m_buffer[_col];
                        }
                    }
                }
                // minimum tau
                for (auto _tmp_path : _path_set->m_path_vec) {
                    if ((*_tmp_path == *_path) || (_tmp_path->m_buffer[_col] > 0)) {
                        _tmp_tt = get_tt(_col * m_base_dta -> m_assign_freq, _tmp_path);
                        _tmp_cost = get_disutility(TFlt(_col * m_base_dta -> m_assign_freq), _tmp_tt);
                        _tmp_change = _tmp_cost - _tot_path_cost / _tot_nonzero_path;
                        if ((_tmp_change > 0) && (_tau > m_step_size * _min_flow / _tmp_change)) {
                            _tau = m_step_size * _min_flow / _tmp_change;
                        }
//                        if ((_tmp_change > 0) && (_tau > 1.0 / _tmp_change)) {
//                            _tau = 1.0 / _tmp_change;
//                        }
                    }
                }
                // flow adjustment
                for (auto _tmp_path : _path_set->m_path_vec) {
                    if ((*_tmp_path == *_path) || (_tmp_path->m_buffer[_col] > 0)) {
                        _tmp_tt = get_tt(_col * m_base_dta -> m_assign_freq, _tmp_path);
                        _tmp_cost = get_disutility(TFlt(_col * m_base_dta -> m_assign_freq), _tmp_tt);
                        _tmp_change = _tmp_cost - _tot_path_cost / _tot_nonzero_path;
                        _tmp_path -> m_buffer[_col] -= MNM_Ults::min(_tmp_path -> m_buffer[_col], _tau * _tmp_change);
                    }
                }

            }
        }
    }
    MNM::print_path_table(m_path_table, dta->m_od_factory, true);
    MNM::save_path_table(dta -> m_file_folder + "/" + dta -> m_statistics -> m_self_config -> get_string("rec_folder"), m_path_table, dta->m_od_factory, true);
    return 0;
}