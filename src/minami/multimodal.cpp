//
// Created by qiling on 2/18/21.
//

#include "multimodal.h"
#include <iostream>

/******************************************************************************************************************
*******************************************************************************************************************
												Bus Stop Models
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Busstop::MNM_Busstop(TInt ID, TInt linkID, TFlt linkloc, std::vector<TInt>* routID_vec) {
    m_busstop_ID = ID;
    m_link_ID = linkID;
    m_link_loc = linkloc; // unit: m
    m_cell_ID = TInt(-1);  // for CTM link

    m_routeID_vec = std::vector<TInt>();
    std::copy(routID_vec -> begin(),
              routID_vec -> end(),
              std::back_inserter(m_routeID_vec));
    routID_vec -> clear();

    m_passed_bus_counter = std::unordered_map<TInt, TInt>();
    for (auto _routeID: m_routeID_vec) {
        m_passed_bus_counter.insert(std::pair<TInt, TInt>(_routeID, TInt(0)));
    }

    m_N_in_bus = std::unordered_map<TInt, MNM_Cumulative_Curve*> ();
    m_N_out_bus = std::unordered_map<TInt, MNM_Cumulative_Curve*> ();

//    m_N_in_car = NULL;
//    m_N_out_car = NULL;
//    m_N_in_truck = NULL;
//    m_N_out_truck = NULL;

    install_cumulative_curve_multiclass();
}

MNM_Busstop::~MNM_Busstop()
{
    m_routeID_vec.clear();

    m_passed_bus_counter.clear();

    for (auto it: m_N_in_bus) {
        if (it.second != nullptr) delete it.second;
    }
    m_N_in_bus.clear();

    for (auto it: m_N_out_bus) {
        if (it.second != nullptr) delete it.second;
    }
    m_N_out_bus.clear();

//    if (m_N_out_car != NULL) delete m_N_out_car;
//    if (m_N_in_car != NULL) delete m_N_in_car;
//    if (m_N_out_truck != NULL) delete m_N_out_truck;
//    if (m_N_in_truck != NULL) delete m_N_in_truck;
}

int MNM_Busstop::install_cumulative_curve_multiclass() {
    if (m_routeID_vec.empty()) {
        printf("busstop has no bus route\n");
        exit(-1);
    }

    for (auto it: m_N_in_bus) {
        if (it.second != nullptr) delete it.second;
    }

    for (auto it: m_N_out_bus) {
        if (it.second != nullptr) delete it.second;
    }

    MNM_Cumulative_Curve *N_in;
    MNM_Cumulative_Curve *N_out;
    for (TInt _route_ID: m_routeID_vec) {
        N_in = new MNM_Cumulative_Curve();
        N_out = new MNM_Cumulative_Curve();
        N_in -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
        N_out -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
        m_N_in_bus.insert(std::pair<TInt, MNM_Cumulative_Curve*>(_route_ID, N_in));
        m_N_out_bus.insert(std::pair<TInt, MNM_Cumulative_Curve*>(_route_ID, N_out));
    }

//    if (m_N_out_car != NULL) delete m_N_out_car;
//    if (m_N_in_car != NULL) delete m_N_in_car;
//    if (m_N_out_truck != NULL) delete m_N_out_truck;
//    if (m_N_in_truck != NULL) delete m_N_in_truck;
//    m_N_in_car = new MNM_Cumulative_Curve();
//    m_N_out_car = new MNM_Cumulative_Curve();
//    m_N_in_truck = new MNM_Cumulative_Curve();
//    m_N_out_truck = new MNM_Cumulative_Curve();
//    m_N_in_car -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
//    m_N_out_car -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
//    m_N_in_truck -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
//    m_N_out_truck -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
    return 0;
}

TFlt MNM_Busstop::get_bus_waiting_time(TFlt time, TInt routeID) {
    auto N_in_it = m_N_in_bus.find(routeID);
    if (N_in_it == m_N_in_bus.end()){
        throw std::runtime_error("Error, MNM_Busstop::get_bus_waiting_time, route does not stop at this stop");
        return 0.;
    } else {
        MNM_Cumulative_Curve* N_in = N_in_it -> second;
        TFlt _cc = N_in -> get_result(time);
        TFlt _target_cc = ceil(_cc); // next bus : next integer in cc
        TFlt _end_time = N_in -> get_time(_target_cc);
        if (_end_time <= time) {
            return TFlt(0);
        } else {
            return _end_time - time;  // interval
        }
    }
}

/******************************************************************************************************************
*******************************************************************************************************************
												Bus Stop Factory
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Busstop_Factory::MNM_Busstop_Factory() {
    m_busstop_map = std::unordered_map<TInt, MNM_Busstop*>();
}

MNM_Busstop_Factory::~MNM_Busstop_Factory() {
    for (auto _map_it = m_busstop_map.begin(); _map_it!= m_busstop_map.end(); _map_it++){
        delete _map_it -> second;
    }
    m_busstop_map.clear();
}

MNM_Busstop * MNM_Busstop_Factory::make_busstop(TInt ID, TInt linkID, TFlt linkloc, std::vector<TInt>* routeID_vec) {
    MNM_Busstop *_busstop = new MNM_Busstop(ID, linkID, linkloc, routeID_vec);
    m_busstop_map.insert(std::pair<TInt, MNM_Busstop*>(ID, _busstop));
    return _busstop;
}

MNM_Busstop * MNM_Busstop_Factory::get_busstop(TInt ID) {
    auto _busstop_it = m_busstop_map.find(ID);
    if (_busstop_it == m_busstop_map.end()){
        throw std::runtime_error("Error, MNM_Busstop_Factory::get_busstop, busstop does not exist");
    }
    return _busstop_it -> second;
}

/******************************************************************************************************************
*******************************************************************************************************************
											  Multimodal Vehicle
*******************************************************************************************************************
******************************************************************************************************************/

MNM_Veh_Multimodal::MNM_Veh_Multimodal(TInt ID, TInt vehicle_class, TInt start_time, TInt bus_route_ID)
    : MNM_Veh_Multiclass::MNM_Veh_Multiclass(ID, vehicle_class, start_time)
{
    m_bus_route_ID = bus_route_ID;
    m_stopped_intervals = TInt(0);
}

MNM_Veh_Multimodal::~MNM_Veh_Multimodal()
{
    ;
}

/******************************************************************************************************************
*******************************************************************************************************************
												Vehicle Factory
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Veh_Factory_Multimodal::MNM_Veh_Factory_Multimodal()
    : MNM_Veh_Factory_Multiclass::MNM_Veh_Factory_Multiclass(){
    ;
}

MNM_Veh_Factory_Multimodal::~MNM_Veh_Factory_Multimodal(){
    ;
}

MNM_Veh_Multimodal* MNM_Veh_Factory_Multimodal::make_veh_multimodal(TInt timestamp, Vehicle_type veh_type,
                                                                    TInt vehicle_cls, TInt bus_route_ID) {
    // printf("A vehicle is produce at time %d, ID is %d\n", (int)timestamp, (int)m_num_veh + 1);
    MNM_Veh_Multimodal *_veh = new MNM_Veh_Multimodal(m_num_veh + 1, vehicle_cls, timestamp, bus_route_ID);
    _veh -> m_type = veh_type;
    m_veh_map.insert({m_num_veh + 1, _veh});
    m_num_veh += 1;
    return _veh;
}

/******************************************************************************************************************
*******************************************************************************************************************
												Multimodal OD
*******************************************************************************************************************
******************************************************************************************************************/

MNM_Destination_Multimodal::MNM_Destination_Multimodal(TInt ID)
        : MNM_Destination_Multiclass::MNM_Destination_Multiclass(ID)
{
    ;
}


MNM_Destination_Multimodal::~MNM_Destination_Multimodal()
{
    ;
}

MNM_Origin_Multimodal::MNM_Origin_Multimodal(TInt ID, TInt max_interval, TFlt flow_scalar, TInt frequency)
    : MNM_Origin_Multiclass::MNM_Origin_Multiclass(ID, max_interval, flow_scalar, frequency){
    m_demand_bus = std::unordered_map<MNM_Destination_Multimodal*, std::unordered_map<TInt, TFlt*>>();
}

MNM_Origin_Multimodal::~MNM_Origin_Multimodal()
{
    for (auto _demand_it : m_demand_car) {
        free(_demand_it.second);
    }
    m_demand_car.clear();

    for (auto _demand_it : m_demand_truck) {
        free(_demand_it.second);
    }
    m_demand_truck.clear();

    for (auto _demand_it : m_demand_bus) {
        for (auto _demand_it_it : _demand_it.second) {
            free(_demand_it_it.second);
        }
        _demand_it.second.clear();
    }
    m_demand_bus.clear();
}

int MNM_Origin_Multimodal::add_dest_demand_bus(MNM_Destination_Multimodal *dest, TInt routeID, TFlt *demand_bus) {
    // split (15-mins demand) to (15 * 1-minute demand)
    // bus demand
    TFlt* _demand_bus = (TFlt*) malloc(sizeof(TFlt) * m_max_assign_interval * 15);
    for (int i = 0; i < m_max_assign_interval * 15; ++i) {
        _demand_bus[i] =  TFlt(demand_bus[i]);
    }
    if (m_demand_bus.find(dest) == m_demand_bus.end()) {
        std::unordered_map<TInt, TFlt*> _demand_route = std::unordered_map<TInt, TFlt*>();
        _demand_route.insert({routeID, _demand_bus});
        m_demand_bus.insert({dest, _demand_route});
    } else {
        m_demand_bus.find(dest)->second.insert({routeID, _demand_bus});
    }
    return 0;
}

int MNM_Origin_Multimodal::release_one_interval(TInt current_interval,
                                                MNM_Veh_Factory* veh_factory,
                                                TInt assign_interval,
                                                TFlt adaptive_ratio)
{
    if (assign_interval < 0) return 0;
    TInt _veh_to_release;
    MNM_Veh_Multimodal *_veh;
    MNM_Veh_Factory_Multimodal *_vfactory = dynamic_cast<MNM_Veh_Factory_Multimodal *>(veh_factory);
    // release all car
    for (auto _demand_it = m_demand_car.begin(); _demand_it != m_demand_car.end(); _demand_it++) {
        _veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
        if (_veh_to_release == 0) {
            throw std::runtime_error("Error, m_flow_scalar is too small for car demand, increase it so that m_flow_scalar * original_15min_demand / 15 is at least 1");
        }
        for (int i = 0; i < _veh_to_release; ++i) {
            if (adaptive_ratio == TFlt(0)){
                _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_STATIC, TInt(0));
            }
            else if (adaptive_ratio == TFlt(1)){
                _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
            }
            else{
                TFlt _r = MNM_Ults::rand_flt();
                if (_r <= adaptive_ratio){
                    _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
                }
                else{
                    _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_STATIC, TInt(0));
                }
            }
            _veh -> set_destination(_demand_it -> first);
            _veh -> set_origin(this);
            _veh -> m_assign_interval = assign_interval;
            m_origin_node -> m_in_veh_queue.push_back(_veh);
        }
    }
    // release all truck
    for (auto _demand_it = m_demand_truck.begin(); _demand_it != m_demand_truck.end(); _demand_it++) {
        _veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
        if (_veh_to_release == 0) {
            throw std::runtime_error("Error, m_flow_scalar is too small for truck demand, increase it so that m_flow_scalar * original_15min_demand / 15 is at least 1");
        }
        for (int i = 0; i < _veh_to_release; ++i) {
            if (adaptive_ratio == TFlt(0)){
                _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_STATIC, TInt(1));
            }
            else if (adaptive_ratio == TFlt(1)){
                _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
            }
            else{
                TFlt _r = MNM_Ults::rand_flt();
                if (_r <= adaptive_ratio){
                    _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
                }
                else{
                    _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_STATIC, TInt(1));
                }
            }
            _veh -> set_destination(_demand_it -> first);
            _veh -> set_origin(this);
            _veh -> m_assign_interval = assign_interval;
            m_origin_node -> m_in_veh_queue.push_back(_veh);
        }
    }
    // release all bus, which have all veh_type = MNM_TYPE_STATIC, vehicle_cls = 1, and different bus_route
    for (auto _demand_it = m_demand_bus.begin(); _demand_it != m_demand_bus.end(); _demand_it++) { // destination
        for (auto _demand_it_it = _demand_it -> second.begin(); _demand_it_it != _demand_it -> second.end(); _demand_it_it++) { // bus route
            _veh_to_release = TInt(MNM_Ults::round((_demand_it_it -> second)[assign_interval] * m_flow_scalar));
            if (_veh_to_release == 0) {
                throw std::runtime_error("Error, m_flow_scalar is too small for bus demand, increase it so that m_flow_scalar * original_15min_demand / 15 is at least 1");
            }
            for (int i = 0; i < _veh_to_release; ++i) {
                _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_STATIC, TInt(1), _demand_it_it -> first);

                _veh -> set_destination(_demand_it -> first);
                _veh -> set_origin(this);
                _veh -> m_assign_interval = assign_interval;
                m_origin_node -> m_in_veh_queue.push_back(_veh);
            }
        }
    }
    random_shuffle(m_origin_node -> m_in_veh_queue.begin(), m_origin_node -> m_in_veh_queue.end());
    return 0;
}

int MNM_Origin_Multimodal::release_one_interval_biclass(TInt current_interval,
                                                        MNM_Veh_Factory* veh_factory,
                                                        TInt assign_interval,
                                                        TFlt adaptive_ratio_car,
                                                        TFlt adaptive_ratio_truck)
{
    if (assign_interval < 0) return 0;
    TInt _veh_to_release;
    MNM_Veh_Multimodal *_veh;
    MNM_Veh_Factory_Multimodal *_vfactory = dynamic_cast<MNM_Veh_Factory_Multimodal *>(veh_factory);
    // release all car
    for (auto _demand_it = m_demand_car.begin(); _demand_it != m_demand_car.end(); _demand_it++) {
        _veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
        if (_veh_to_release == 0) {
            throw std::runtime_error("Error, m_flow_scalar is too small for car demand, increase it so that m_flow_scalar * original_15min_demand / 15 is at least 1");
        }
        for (int i = 0; i < _veh_to_release; ++i) {
            if (adaptive_ratio_car == TFlt(0)){
                _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_STATIC, TInt(0));
            }
            else if (adaptive_ratio_car == TFlt(1)){
                _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
            }
            else{
                TFlt _r = MNM_Ults::rand_flt();
                if (_r <= adaptive_ratio_car){
                    _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
                }
                else{
                    _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_STATIC, TInt(0));
                }
            }
            _veh -> set_destination(_demand_it -> first);
            _veh -> set_origin(this);
            _veh -> m_assign_interval = assign_interval;
            m_origin_node -> m_in_veh_queue.push_back(_veh);
        }
    }
    // release all truck
    for (auto _demand_it = m_demand_truck.begin(); _demand_it != m_demand_truck.end(); _demand_it++) {
        _veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
        if (_veh_to_release == 0) {
            throw std::runtime_error("Error, m_flow_scalar is too small for truck demand, increase it so that m_flow_scalar * original_15min_demand / 15 is at least 1");
        }
        for (int i = 0; i < _veh_to_release; ++i) {
            if (adaptive_ratio_truck == TFlt(0)){
                _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_STATIC, TInt(1));
            }
            else if (adaptive_ratio_truck == TFlt(1)){
                _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
            }
            else{
                TFlt _r = MNM_Ults::rand_flt();
                if (_r <= adaptive_ratio_truck){
                    _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
                }
                else{
                    _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_STATIC, TInt(1));
                }
            }
            _veh -> set_destination(_demand_it -> first);
            _veh -> set_origin(this);
            _veh -> m_assign_interval = assign_interval;
            m_origin_node -> m_in_veh_queue.push_back(_veh);
        }
    }
    // release all bus, which have all veh_type = MNM_TYPE_STATIC, vehicle_cls = 1, and different bus_route
    for (auto _demand_it = m_demand_bus.begin(); _demand_it != m_demand_bus.end(); _demand_it++) { // destination
        for (auto _demand_it_it = _demand_it -> second.begin(); _demand_it_it != _demand_it -> second.end(); _demand_it_it++) { // bus route
            _veh_to_release = TInt(MNM_Ults::round((_demand_it_it -> second)[assign_interval] * m_flow_scalar));
            if (_veh_to_release == 0) {
                throw std::runtime_error("Error, m_flow_scalar is too small for bus demand, increase it so that m_flow_scalar * original_15min_demand / 15 is at least 1");
            }
            for (int i = 0; i < _veh_to_release; ++i) {
                _veh = _vfactory -> make_veh_multimodal(current_interval, MNM_TYPE_STATIC, TInt(1), _demand_it_it -> first);

                _veh -> set_destination(_demand_it -> first);
                _veh -> set_origin(this);
                _veh -> m_assign_interval = assign_interval;
                m_origin_node -> m_in_veh_queue.push_back(_veh);
            }
        }
    }
    random_shuffle(m_origin_node -> m_in_veh_queue.begin(), m_origin_node -> m_in_veh_queue.end());

    return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
												OD Factory
*******************************************************************************************************************
******************************************************************************************************************/
MNM_OD_Factory_Multimodal::MNM_OD_Factory_Multimodal()
    : MNM_OD_Factory_Multiclass::MNM_OD_Factory_Multiclass(){
    ;
}

MNM_OD_Factory_Multimodal::~MNM_OD_Factory_Multimodal(){
    ;
}

MNM_Destination_Multimodal *MNM_OD_Factory_Multimodal::make_destination(TInt ID){
    MNM_Destination_Multimodal *_dest;
    _dest = new MNM_Destination_Multimodal(ID);
    m_destination_map.insert({ID, _dest});
    return _dest;
}

MNM_Origin_Multimodal * MNM_OD_Factory_Multimodal::make_origin(TInt ID, TInt max_interval, TFlt flow_scalar,
                                                               TInt frequency) {
    MNM_Origin_Multimodal *_origin;
    _origin = new MNM_Origin_Multimodal(ID, max_interval, flow_scalar, frequency);
    m_origin_map.insert({ID, _origin});
    return _origin;
}

/******************************************************************************************************************
*******************************************************************************************************************
												Node Factory
*******************************************************************************************************************
******************************************************************************************************************/

MNM_Node_Factory_Multimodal::MNM_Node_Factory_Multimodal()
        : MNM_Node_Factory_Multiclass::MNM_Node_Factory_Multiclass()
{
    ;
}

MNM_Node_Factory_Multimodal::~MNM_Node_Factory_Multimodal()
{
    ;
}

MNM_Dnode *MNM_Node_Factory_Multimodal::make_node_multimodal(TInt ID,
                                                             DNode_type_multimodal node_type,
                                                             TFlt flow_scalar,
                                                             TFlt veh_convert_factor)
{
    MNM_Dnode *_node;
    switch (node_type){
        case MNM_TYPE_FWJ_MULTIMODAL:
            _node = new MNM_Dnode_FWJ_Multiclass(ID, flow_scalar, veh_convert_factor);
            break;
        case MNM_TYPE_ORIGIN_MULTIMODAL:
            _node = new MNM_DMOND_Multiclass(ID, flow_scalar, veh_convert_factor);
            break;
        case MNM_TYPE_DEST_MULTIMODAL:
            _node = new MNM_DMDND_Multiclass(ID, flow_scalar, veh_convert_factor);
            break;
        default:
            printf("Wrong node type.\n");
            exit(-1);
    }
    m_node_map.insert({ID, _node});
    return _node;
}

/******************************************************************************************************************
*******************************************************************************************************************
												Link Models
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
					Multimodal Point-Queue Model
**************************************************************************/

MNM_Dlink_Pq_Multimodal::MNM_Dlink_Pq_Multimodal(TInt ID, TInt number_of_lane, TFlt length, TFlt lane_hold_cap_car,
                                                 TFlt lane_hold_cap_truck, TFlt lane_flow_cap_car,
                                                 TFlt lane_flow_cap_truck, TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
                                                 TFlt veh_convert_factor, TFlt flow_scalar)
    : MNM_Dlink_Pq_Multiclass::MNM_Dlink_Pq_Multiclass(ID, number_of_lane, length, lane_hold_cap_car,
                                                       lane_hold_cap_truck, lane_flow_cap_car,
                                                       lane_flow_cap_truck, ffs_car, ffs_truck, unit_time,
                                                       veh_convert_factor, flow_scalar){
    m_link_type = MNM_TYPE_PQ_MULTIMODAL;
    m_busstop_vec = std::vector<MNM_Busstop*>();
}

MNM_Dlink_Pq_Multimodal::~MNM_Dlink_Pq_Multimodal()
{
//    for (MNM_Busstop* _bus_stop : m_busstop_vec){
//        if (_bus_stop != nullptr) delete _bus_stop;
//    }
    m_busstop_vec.clear();
}

int MNM_Dlink_Pq_Multimodal::clear_incoming_array(TInt timestamp) {

    // add zero bus count to cc to ensure the cc has the right length after DNL
    if (!m_busstop_vec.empty()) {
        for (auto _busstop : m_busstop_vec) {
            for (auto _route_ID : _busstop -> m_routeID_vec) {
                _busstop -> m_N_in_bus.find(_route_ID) -> second ->
                        add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(0)/m_flow_scalar));
                _busstop -> m_N_out_bus.find(_route_ID) -> second ->
                        add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(0)/m_flow_scalar));
            }
        }
    }

    MNM_Veh_Multimodal *_veh;
    TFlt _to_be_moved = get_link_supply() * m_flow_scalar;
    while (!m_incoming_array.empty()) {
        if ( _to_be_moved > 0){
            _veh = dynamic_cast<MNM_Veh_Multimodal *>(m_incoming_array.front());
            m_incoming_array.pop_front();
            m_veh_pool.insert({_veh, TInt(0)});
            if (_veh -> m_class == 0) {
                //printf("car\n");
                m_volume_car += 1;
                _to_be_moved -= 1;
            }
            else {
                //printf("truck\n");
                m_volume_truck += 1;
                // _to_be_moved -= m_veh_convert_factor;
                _to_be_moved -= 1;

                if ((_veh -> m_bus_route_ID != TInt(-1)) && (!m_busstop_vec.empty())) {
                    // if it is bus, update cc
                    for (auto *_busstop : m_busstop_vec) {
                        std::vector<TInt>::iterator it;
                        it = std::find(_busstop -> m_routeID_vec.begin(), _busstop -> m_routeID_vec.end(), _veh -> m_bus_route_ID);
                        if (it != _busstop -> m_routeID_vec.end()) {
                            // starting and ending busstops on OD connectors do not hold bus
                            _busstop -> m_N_in_bus.find(_veh -> m_bus_route_ID) -> second -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
                        }
                    }
                }
            }
        }
        else {
            break;
        }
    }
    // printf("car: %d, truck: %d\n", m_volume_car, m_volume_truck);
    return 0;
}

int MNM_Dlink_Pq_Multimodal::evolve(TInt timestamp)
{
    std::unordered_map<MNM_Veh*, TInt>::iterator _que_it = m_veh_pool.begin();
    MNM_Veh_Multimodal* _veh;
    TInt _num_car = 0, _num_truck = 0;

    while (_que_it != m_veh_pool.end()) {
        if (_que_it -> second >= m_max_stamp) {
            m_finished_array.push_back(_que_it -> first);
            _veh = dynamic_cast<MNM_Veh_Multimodal *>(m_finished_array.back());
            if (_veh -> get_class() == 0) {
                _num_car += 1;
            }
            else {
                _num_truck += 1;
                if ((_veh -> m_bus_route_ID != TInt(-1)) && (!m_busstop_vec.empty())) {
                    // if it is bus, update cc
                    for (auto *_busstop : m_busstop_vec) {
                        std::vector<TInt>::iterator it;
                        it = std::find(_busstop -> m_routeID_vec.begin(), _busstop -> m_routeID_vec.end(), _veh -> m_bus_route_ID);
                        if (it != _busstop -> m_routeID_vec.end()) {
                            // starting and ending busstops on OD connectors do not hold bus
                            _busstop -> m_N_out_bus.find(_veh -> m_bus_route_ID) -> second -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
                        }
                    }
                }
            }
            _que_it = m_veh_pool.erase(_que_it); //c++ 11
        }
        else {
            _que_it -> second += 1;
            _que_it ++;
        }
    }
    // printf("car: %d, truck: %d\n", _num_car, _num_truck);
    m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
    return 0;
}

/**************************************************************************
					Multimodal CTM Model
**************************************************************************/

MNM_Dlink_Ctm_Multimodal::MNM_Dlink_Ctm_Multimodal(TInt ID, TInt number_of_lane, TFlt length, TFlt lane_hold_cap_car,
                                                   TFlt lane_hold_cap_truck, TFlt lane_flow_cap_car,
                                                   TFlt lane_flow_cap_truck, TFlt ffs_car, TFlt ffs_truck,
                                                   TFlt unit_time, TFlt veh_convert_factor, TFlt flow_scalar)
     : MNM_Dlink_Ctm_Multiclass::MNM_Dlink_Ctm_Multiclass(ID, number_of_lane, length, lane_hold_cap_car,
                                                          lane_hold_cap_truck, lane_flow_cap_car,
                                                          lane_flow_cap_truck, ffs_car, ffs_truck, unit_time,
                                                          veh_convert_factor, flow_scalar){
    m_link_type = MNM_TYPE_CTM_MULTIMODAL;
    m_cell_busstop_vec = std::unordered_map<TInt, std::vector<MNM_Busstop*>>();  // set in build_busstop_factory
    m_busstop_vec = std::vector<MNM_Busstop*>();  // set in build_busstop_factory
}

MNM_Dlink_Ctm_Multimodal::~MNM_Dlink_Ctm_Multimodal()
{
    for (auto _it : m_cell_busstop_vec){
//        for (MNM_Busstop* _bus_stop : _it.second) {
//            delete _bus_stop;
//        }
        _it.second.clear();
    }
    m_cell_busstop_vec.clear();

//    for (MNM_Busstop* _bus_stop : m_busstop_vec){
//        delete _bus_stop;
//    }
    m_busstop_vec.clear();
}

int MNM_Dlink_Ctm_Multimodal::clear_incoming_array(TInt timestamp)
{
    // if (get_link_supply() * m_flow_scalar < m_incoming_array.size()){
    // 	printf("Wrong incoming array size\n");
    // 	exit(-1);
    // }

    // add zero bus count to cc to ensure the cc has the right length after DNL
    if (!m_busstop_vec.empty()) {
        for (auto _busstop : m_busstop_vec) {
            for (auto _route_ID : _busstop -> m_routeID_vec) {
                _busstop -> m_N_in_bus.find(_route_ID) -> second ->
                        add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(0)/m_flow_scalar));
                _busstop -> m_N_out_bus.find(_route_ID) -> second ->
                        add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(0)/m_flow_scalar));
            }
        }
    }

    MNM_Veh_Multimodal* _veh;
    size_t _cur_size = m_incoming_array.size();
    for (size_t i = 0; i < _cur_size; ++i) {
        _veh = dynamic_cast<MNM_Veh_Multimodal *>(m_incoming_array.front());
        m_incoming_array.pop_front();
        if (_veh -> m_class == TInt(0)) {
            // printf("car\n");
            m_cell_array[0] -> m_veh_queue_car.push_back(_veh);
        }
        else {
            // printf("truck\n");
            m_cell_array[0] -> m_veh_queue_truck.push_back(_veh);

            if ((_veh -> m_bus_route_ID != TInt(-1)) && (!m_busstop_vec.empty())) {
                // check if the bus stop at this cell
                for (auto *_busstop : m_busstop_vec) {
                    if (_busstop->m_cell_ID == 0) {
                        std::vector<TInt>::iterator it;
                        it = std::find(_busstop->m_routeID_vec.begin(), _busstop->m_routeID_vec.end(),
                                       _veh->m_bus_route_ID);
                        if (it != _busstop->m_routeID_vec.end()) {
                            _busstop -> m_N_in_bus.find(_veh -> m_bus_route_ID) -> second ->
                                    add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
                        }
                    }
                }
            }
        }
        _veh -> m_visual_position_on_link = float(1)/float(m_num_cells)/float(2); // initial position at first cell
    }
    m_cell_array[0] -> m_volume_car = m_cell_array[0] -> m_veh_queue_car.size();
    m_cell_array[0] -> m_volume_truck = m_cell_array[0] -> m_veh_queue_truck.size();
    m_cell_array[0] -> update_perceived_density();

    return 0;
}

int MNM_Dlink_Ctm_Multimodal::move_veh_queue_in_cell(std::deque<MNM_Veh*> *from_queue,
                                                     std::deque<MNM_Veh*> *to_queue,
                                                     TInt number_tomove,
                                                     TInt timestamp,
                                                     TInt cell_ID)
{
    MNM_Veh* _veh;
    MNM_Veh_Multimodal* _veh_multimodal;
    bool _held = false;
    TInt _veh_moved_count = 0;
    TInt _veh_held_count = 0;
    std::deque<MNM_Veh*> _held_queue = std::deque<MNM_Veh*>();
    TInt _current_cell = cell_ID;

    // only for CTM link
    if (_current_cell < 0) {
        _current_cell = 0;
    } else if (_current_cell > m_num_cells - 2) {
        // last cell is special, handled in move_last_cell()
        _current_cell = m_num_cells - 2;
    }

    while (_veh_moved_count < number_tomove && !(from_queue->empty())) {
    // for (int i = 0; i < number_tomove; ++i) {
        _veh = from_queue -> front();
        _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *>(_veh);
        _held = false;
        if (_veh_multimodal -> m_bus_route_ID != TInt(-1)) {
            // check if the bus stop at this cell
//            for (auto *_busstop : m_cell_busstop_vec.find(_current_cell) -> second) {
//                std::vector<TInt>::iterator it;
//                it = std::find(_busstop -> m_routeID_vec.begin(), _busstop -> m_routeID_vec.end(), _veh_multimodal -> m_bus_route_ID);
//                if (it != _busstop -> m_routeID_vec.end()) {
//                    // hold bus at bus stop for 2 intervals, 2 x 5s = 10 s
//                    if (_veh_multimodal -> m_stopped_intervals < 2) {
//                        // hold the bus, move the bus to a different location of queue
//                        from_queue -> pop_front();
//                        _veh_moved_count += 1;
//                        std::deque<MNM_Veh *>::iterator pos = from_queue -> begin();
//                        pos = from_queue -> begin() + number_tomove - _veh_moved_count + _veh_held_count;
//                        pos = from_queue -> insert(pos, _veh);
//                        _veh_multimodal -> m_stopped_intervals += 1;
//                        _held = true;
//                        break;
//                    }
//                }
//            }

            for (auto *_busstop : m_busstop_vec) {
                if (_busstop -> m_cell_ID == _current_cell) {
                    std::vector<TInt>::iterator it;
                    it = std::find(_busstop -> m_routeID_vec.begin(), _busstop -> m_routeID_vec.end(), _veh_multimodal -> m_bus_route_ID);
                    if (it != _busstop -> m_routeID_vec.end()) {
                        // bus stop is at cell, originally passing a cell requires at least 1 interval
                        // now hold bus at bus stop for additional n intervals, n x 5s
                        // passing a cell requires at least 1 + n intervals

                        // only stop bus every m_flow_scalar vehicles
                        if ((_busstop -> m_passed_bus_counter.find(_veh_multimodal -> m_bus_route_ID) -> second < int(m_flow_scalar) - 1) &&
                            ((_veh_multimodal -> m_stopped_intervals == 0))) {
                            _busstop -> m_passed_bus_counter.find(_veh_multimodal -> m_bus_route_ID) -> second += 1;
                            break;
                        }
                        // TODO: 2 is arbitrarily set
                        if (_veh_multimodal -> m_stopped_intervals < 2) {
                            // hold the bus, move the bus to a different location of queue
                            from_queue->pop_front();
                            _held_queue.push_back(_veh);
                            // reset bus counter
                            if (_veh_multimodal -> m_stopped_intervals == 0) {
                                IAssert(_busstop -> m_passed_bus_counter.find(_veh_multimodal -> m_bus_route_ID) -> second == int(m_flow_scalar) - 1);
                                _busstop -> m_passed_bus_counter.find(_veh_multimodal -> m_bus_route_ID) -> second = 0;
                            }
                            _veh_multimodal->m_stopped_intervals += 1;
                            _held = true;
                            _veh_held_count += 1;
                            break;
                        }

                        // stop every bus
//                        if (_veh_multimodal -> m_stopped_intervals < 2) {
//                            // hold the bus, move the bus to a different location of queue
//                            from_queue->pop_front();
//                            _held_queue.push_back(_veh);
//                            _veh_multimodal->m_stopped_intervals += 1;
//                            _held = true;
//                            _veh_held_count += 1;
//                            break;
//                        }
                    }
                }
            }
        }

        if (! _held) {
            // bus can move to next cell
            from_queue -> pop_front();
            // update the vehicle position on current link. 0: at the beginning, 1: at the end.
            _veh_multimodal -> m_visual_position_on_link += float(1)/float(m_num_cells);
            if (_veh_multimodal -> m_visual_position_on_link > 0.99)
                _veh_multimodal -> m_visual_position_on_link = 0.99;
            to_queue -> push_back(_veh);
            _veh_moved_count += 1;
            // reset
            _veh_multimodal -> m_stopped_intervals = 0;
            _held = false;

            if ((_veh_multimodal -> m_class == 1) && (_veh_multimodal -> m_bus_route_ID != TInt(-1)) && (!m_busstop_vec.empty())) {
                // check if this cell or next cell has busstops
                for (auto *_busstop : m_busstop_vec) {
                    if (_busstop->m_cell_ID == _current_cell) {
                        std::vector<TInt>::iterator it;
                        it = std::find(_busstop->m_routeID_vec.begin(), _busstop->m_routeID_vec.end(),
                                       _veh_multimodal->m_bus_route_ID);
                        if (it != _busstop->m_routeID_vec.end()) {
                            _busstop -> m_N_out_bus.find(_veh_multimodal -> m_bus_route_ID) -> second ->
                                    add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
                        }
                    } else if (_busstop->m_cell_ID == _current_cell + 1) {
                        std::vector<TInt>::iterator it;
                        it = std::find(_busstop->m_routeID_vec.begin(), _busstop->m_routeID_vec.end(),
                                       _veh_multimodal->m_bus_route_ID);
                        if (it != _busstop->m_routeID_vec.end()) {
                            _busstop -> m_N_in_bus.find(_veh_multimodal -> m_bus_route_ID) -> second ->
                                    add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
                        }
                    }
                }
            }
        }
    }
    IAssert(int(_held_queue.size()) == _veh_held_count);
    if (!_held_queue.empty()) {
        for (int i = 0; i < _veh_held_count; ++i) {
            _veh = _held_queue.back();
            _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal*>(_veh);
            IAssert(_veh_multimodal -> m_class == 1 && _veh_multimodal->m_bus_route_ID != -1);
            _held_queue.pop_back();
            from_queue -> push_front(_veh);
        }
    }
    _held_queue.clear();
    return 0;
}

int MNM_Dlink_Ctm_Multimodal::move_veh_queue_in_last_cell(TInt timestamp)
{
    TInt _num_veh_tomove_car = m_cell_array[m_num_cells - 1] -> m_out_veh_car;
    TInt _num_veh_tomove_truck = m_cell_array[m_num_cells - 1] -> m_out_veh_truck;
    TInt _num_veh_tomove_total = _num_veh_tomove_car + _num_veh_tomove_truck;
    TFlt _pstar = TFlt(_num_veh_tomove_car)/TFlt(_num_veh_tomove_total);
    MNM_Veh* _veh;
    MNM_Veh_Multimodal* _veh_multimodal;
    bool _held = false;
    TFlt _r;  // randomly mix truck and car
    TInt _car_moved_count = 0;
    TInt _truck_moved_count = 0;

    TInt _veh_held_count = 0;
    std::deque<MNM_Veh*> _held_queue = std::deque<MNM_Veh*>();

    while ((_num_veh_tomove_car > _car_moved_count) ||
            ((_num_veh_tomove_truck > _truck_moved_count) && (!(m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.empty())))){
        _held = false;
        _r = MNM_Ults::rand_flt();
        // probability = _pstar to move a car
        if (_r < _pstar){
            // still has car to move
            if (_num_veh_tomove_car > _car_moved_count){
                _veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.front();
                m_cell_array[m_num_cells - 1] -> m_veh_queue_car.pop_front();
                if (_veh -> has_next_link()){
                    m_finished_array.push_back(_veh);
                }
                else {
                    printf("Dlink_CTM_Multimodal::Something wrong!\n");
                    exit(-1);
                }
                _car_moved_count += 1;
            }
            // no car to move, move a truck
            else {
                _veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.front();

                _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal*>(_veh);
                if (_veh_multimodal -> m_bus_route_ID != TInt(-1)) {
                    // check if the bus stop at this cell
                    for (auto *_busstop : m_busstop_vec) {
                        if (_busstop->m_cell_ID == m_num_cells - 1) {
                            std::vector<TInt>::iterator it;
                            it = std::find(_busstop->m_routeID_vec.begin(), _busstop->m_routeID_vec.end(),
                                           _veh_multimodal->m_bus_route_ID);
                            if (it != _busstop->m_routeID_vec.end()) {
                                // hold bus at bus stop for 2 intervals, 2 x 5s = 10 s

                                // only stop bus every m_flow_scalar vehicles
                                if ((_busstop -> m_passed_bus_counter.find(_veh_multimodal -> m_bus_route_ID) -> second < int(m_flow_scalar) - 1) &&
                                    ((_veh_multimodal -> m_stopped_intervals == 0))) {
                                    _busstop -> m_passed_bus_counter.find(_veh_multimodal -> m_bus_route_ID) -> second += 1;
                                    break;
                                }
                                // TODO: 2 is arbitrarily set
                                if (_veh_multimodal -> m_stopped_intervals < 2) {
                                    // hold the bus, move the bus to a different location of queue
                                    m_cell_array[m_num_cells - 1]->m_veh_queue_truck.pop_front();
                                    _held_queue.push_back(_veh);
                                    // reset bus counter
                                    if (_veh_multimodal -> m_stopped_intervals == 0) {
                                        IAssert(_busstop -> m_passed_bus_counter.find(_veh_multimodal -> m_bus_route_ID) -> second == int(m_flow_scalar) - 1);
                                        _busstop -> m_passed_bus_counter.find(_veh_multimodal -> m_bus_route_ID) -> second = 0;
                                    }
                                    _veh_multimodal->m_stopped_intervals += 1;
                                    _held = true;
                                    _veh_held_count += 1;
                                    break;
                                }

                                // stop every bus
//                                if (_veh_multimodal->m_stopped_intervals < 2) {
//                                    // hold the bus, move the bus to a different location of queue
//                                    m_cell_array[m_num_cells - 1]->m_veh_queue_truck.pop_front();
//                                    _held_queue.push_back(_veh);
//                                    _veh_multimodal->m_stopped_intervals += 1;
//                                    _held = true;
//                                    _veh_held_count += 1;
//                                    break;
//                                }
                            }
                        }
                    }
                }

                if (! _held) {
                    m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.pop_front();
                    if (_veh -> has_next_link()){
                        m_finished_array.push_back(_veh);

                        if ((_veh_multimodal -> m_class == 1) && (_veh_multimodal -> m_bus_route_ID != TInt(-1))) {
                            // check if this cell has busstops
                            for (auto *_busstop : m_busstop_vec) {
                                if (_busstop->m_cell_ID == m_num_cells - 1) {
                                    std::vector<TInt>::iterator it;
                                    it = std::find(_busstop->m_routeID_vec.begin(), _busstop->m_routeID_vec.end(),
                                                   _veh_multimodal->m_bus_route_ID);
                                    if (it != _busstop->m_routeID_vec.end()) {
                                        _busstop -> m_N_out_bus.find(_veh_multimodal -> m_bus_route_ID) -> second ->
                                                add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
                                        _veh_multimodal->m_stopped_intervals = 0;
                                        _held = false;
                                    }
                                }
                            }
                        }
                    }
                    else {
                        printf("Dlink_CTM_Multimodal::Something wrong!\n");
                        exit(-1);
                    }
                    _truck_moved_count += 1;
                }

            }
        }
            // probability = 1 - _pstar to move a truck
        else {
            // still has truck to move
            if (_num_veh_tomove_truck > _truck_moved_count){
                _veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.front();

                _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal*>(_veh);
                if (_veh_multimodal -> m_bus_route_ID != TInt(-1)) {
                    // check if the bus stop at this cell
                    for (auto *_busstop : m_busstop_vec) {
                        if (_busstop->m_cell_ID == m_num_cells - 1) {
                            std::vector<TInt>::iterator it;
                            it = std::find(_busstop->m_routeID_vec.begin(), _busstop->m_routeID_vec.end(),
                                           _veh_multimodal->m_bus_route_ID);
                            if (it != _busstop->m_routeID_vec.end()) {
                                // hold bus at bus stop for 2 intervals, 2 x 5s = 10 s

                                // only stop bus every m_flow_scalar vehicles
                                if ((_busstop -> m_passed_bus_counter.find(_veh_multimodal -> m_bus_route_ID) -> second < int(m_flow_scalar) - 1) &&
                                    ((_veh_multimodal -> m_stopped_intervals == 0))) {
                                    _busstop -> m_passed_bus_counter.find(_veh_multimodal -> m_bus_route_ID) -> second += 1;
                                    break;
                                }

                                if (_veh_multimodal -> m_stopped_intervals < 2) {
                                    // hold the bus, move the bus to a different location of queue
                                    m_cell_array[m_num_cells - 1]->m_veh_queue_truck.pop_front();
                                    _held_queue.push_back(_veh);
                                    // reset bus counter
                                    if (_veh_multimodal -> m_stopped_intervals == 0) {
                                        IAssert(_busstop -> m_passed_bus_counter.find(_veh_multimodal -> m_bus_route_ID) -> second == int(m_flow_scalar) - 1);
                                        _busstop -> m_passed_bus_counter.find(_veh_multimodal -> m_bus_route_ID) -> second = 0;
                                    }
                                    _veh_multimodal->m_stopped_intervals += 1;
                                    _held = true;
                                    _veh_held_count += 1;
                                    break;
                                }

                                // stop every bus
//                                if (_veh_multimodal->m_stopped_intervals < 2) {
//                                    // hold the bus, move the bus to a different location of queue
//                                    m_cell_array[m_num_cells - 1]->m_veh_queue_truck.pop_front();
//                                    _held_queue.push_back(_veh);
//                                    _veh_multimodal->m_stopped_intervals += 1;
//                                    _held = true;
//                                    _veh_held_count += 1;
//                                    break;
//                                }
                            }
                        }
                    }
                }

                if (! _held) {
                    m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.pop_front();
                    if (_veh -> has_next_link()){
                        m_finished_array.push_back(_veh);

                        if ((_veh_multimodal -> m_class == 1) && (_veh_multimodal -> m_bus_route_ID != TInt(-1))) {
                            // check if this cell has busstops
                            for (auto *_busstop : m_busstop_vec) {
                                if (_busstop->m_cell_ID == m_num_cells - 1) {
                                    std::vector<TInt>::iterator it;
                                    it = std::find(_busstop->m_routeID_vec.begin(), _busstop->m_routeID_vec.end(),
                                                   _veh_multimodal->m_bus_route_ID);
                                    if (it != _busstop->m_routeID_vec.end()) {
                                        _busstop -> m_N_out_bus.find(_veh_multimodal -> m_bus_route_ID) -> second ->
                                                add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
                                        _veh_multimodal->m_stopped_intervals = 0;
                                        _held = false;
                                    }
                                }
                            }
                        }
                    }
                    else {
                        printf("Dlink_CTM_Multimodal::Something wrong!\n");
                        exit(-1);
                    }
                    _truck_moved_count += 1;
                }
            }
            // no truck to move, move a car
            else {
                _veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.front();
                m_cell_array[m_num_cells - 1] -> m_veh_queue_car.pop_front();
                if (_veh -> has_next_link()){
                    m_finished_array.push_back(_veh);
                }
                else {
                    printf("Dlink_CTM_Multimodal::Something wrong!\n");
                    exit(-1);
                }
                _car_moved_count += 1;
            }
        }
    }
    IAssert(int(_held_queue.size()) == _veh_held_count);
    if (!_held_queue.empty()) {
        for (int i = 0; i < _veh_held_count; ++i) {
            _veh = _held_queue.back();
            _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal*>(_veh);
            IAssert(_veh_multimodal -> m_class == 1 && _veh_multimodal->m_bus_route_ID != -1);
            _held_queue.pop_back();
            m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.push_front(_veh);
        }
    }
    _held_queue.clear();
    return 0;
}

int MNM_Dlink_Ctm_Multimodal::evolve(TInt timestamp) {
    std::deque<MNM_Veh*>::iterator _veh_it;
    TInt _count_car = 0;
    TInt _count_truck = 0;
    TInt _count_tot_vehs = 0;

    // TInt _output_link = 16815;
    // if (m_link_ID == _output_link){
    // 	printf("Link %d volume before: ", int(m_link_ID));
    // 	if (m_num_cells > 1){
    // 		for (int i = 0; i < m_num_cells - 1; ++i)
    // 		{
    // 			printf("(%d, %d) ", int(m_cell_array[i] -> m_veh_queue_car.size()), int(m_cell_array[i] -> m_veh_queue_truck.size()));
    // 		}
    // 	}
    // 	// m_class: 0 - private car, 1 - truck
    // 	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
    // 		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
    // 		if (_veh -> m_class == 0) _count_car += 1;
    // 		if (_veh -> m_class == 1) _count_truck += 1;
    // 	}
    // 	printf("(%d, %d; %d, %d)\n", int(m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size()),
    // 		int(m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size()), int(_count_car), int(_count_truck));
    // }

    /* update volume */
    update_out_veh();

    TInt _num_veh_tomove_car, _num_veh_tomove_truck;
    /* previous cells */
    if (m_num_cells > 1){
        for (int i = 0; i < m_num_cells - 1; ++i)
        {
            // Car
            _num_veh_tomove_car = m_cell_array[i] -> m_out_veh_car;
            move_veh_queue_in_cell( &(m_cell_array[i] -> m_veh_queue_car),
                                    &(m_cell_array[i+1] -> m_veh_queue_car),
                                     _num_veh_tomove_car, timestamp, TInt(i));
            // Truck
            _num_veh_tomove_truck = m_cell_array[i] -> m_out_veh_truck;
            move_veh_queue_in_cell( &(m_cell_array[i] -> m_veh_queue_truck),
                                    &(m_cell_array[i+1] -> m_veh_queue_truck),
                                     _num_veh_tomove_truck, timestamp, TInt(i));
        }
    }

    /* last cell */
    move_veh_queue_in_last_cell(timestamp);
    m_tot_wait_time_at_intersection += TFlt(m_finished_array.size())/m_flow_scalar * m_unit_time;

    // if (m_link_ID == _output_link)
    // 	printf("Link %d volume after: ", int(m_link_ID));

    /* update volume */
    if (m_num_cells > 1){
        for (int i = 0; i < m_num_cells - 1; ++i)
        {
            m_cell_array[i] -> m_volume_car = m_cell_array[i] -> m_veh_queue_car.size();
            m_cell_array[i] -> m_volume_truck = m_cell_array[i] -> m_veh_queue_truck.size();
            // Update perceived density of the i-th cell
            m_cell_array[i] -> update_perceived_density();
            // if (m_link_ID == _output_link)
            // 	printf("(%d, %d) ", int(m_cell_array[i] -> m_volume_car), int(m_cell_array[i] -> m_volume_truck));
        }
    }

    _count_car = 0;
    _count_truck = 0;
    // m_class: 0 - private car, 1 - truck
    for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
        MNM_Veh_Multimodal *_veh = dynamic_cast<MNM_Veh_Multimodal *>(*_veh_it);
        if (_veh -> m_class == 0) _count_car += 1;
        if (_veh -> m_class == 1) _count_truck += 1;
    }
    m_cell_array[m_num_cells - 1] -> m_volume_car =
            m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size() + _count_car;
    m_cell_array[m_num_cells - 1] -> m_volume_truck =
            m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size() + _count_truck;
    m_cell_array[m_num_cells - 1] -> update_perceived_density();

    // if (m_link_ID == _output_link){
    // 	printf("(%d, %d; %d, %d)\n", int(m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size()),
    // 		int(m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size()), int(_count_car), int(_count_truck));
    // 	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
    // 		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
    // 		printf("(%d-%d: %d->%d) ", int(_veh -> m_veh_ID), int(_veh -> m_class), int(_veh -> get_current_link() -> m_link_ID),
    // 			int(_veh -> get_next_link() -> m_link_ID));
    // 	}
    // 	printf("\n");
    // }

    /* compute total volume of link, check if spill back */
    _count_tot_vehs = 0;
    for (int i = 0; i <= m_num_cells - 1; ++i)
    {
        _count_tot_vehs += m_cell_array[i] -> m_volume_car;
        _count_tot_vehs += m_cell_array[i] -> m_volume_truck * m_veh_convert_factor;
    }
    if (TFlt(_count_tot_vehs)/m_flow_scalar/m_length > m_lane_hold_cap_car * m_number_of_lane){
        m_spill_back = true;
    }
    return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
											Link Factory
*******************************************************************************************************************
******************************************************************************************************************/

MNM_Link_Factory_Multimodal::MNM_Link_Factory_Multimodal()
        : MNM_Link_Factory_Multiclass::MNM_Link_Factory_Multiclass()
{
    ;
}

MNM_Link_Factory_Multimodal::~MNM_Link_Factory_Multimodal()
{
    ;
}

MNM_Dlink *MNM_Link_Factory_Multimodal::make_link_multimodal(TInt ID,
                                                             DLink_type_multimodal link_type,
                                                             TInt number_of_lane,
                                                             TFlt length,
                                                             TFlt lane_hold_cap_car,
                                                             TFlt lane_hold_cap_truck,
                                                             TFlt lane_flow_cap_car,
                                                             TFlt lane_flow_cap_truck,
                                                             TFlt ffs_car,
                                                             TFlt ffs_truck,
                                                             TFlt unit_time,
                                                             TFlt veh_convert_factor,
                                                             TFlt flow_scalar)
{
    MNM_Dlink *_link;
    switch (link_type){
        case MNM_TYPE_CTM_MULTIMODAL:
            _link = new MNM_Dlink_Ctm_Multimodal(ID,
                                                 number_of_lane,
                                                 length,
                                                 lane_hold_cap_car,
                                                 lane_hold_cap_truck,
                                                 lane_flow_cap_car,
                                                 lane_flow_cap_truck,
                                                 ffs_car,
                                                 ffs_truck,
                                                 unit_time,
                                                 veh_convert_factor,
                                                 flow_scalar);
            break;
        case MNM_TYPE_PQ_MULTIMODAL:
            _link = new MNM_Dlink_Pq_Multimodal(ID,
                                                number_of_lane,
                                                length,
                                                lane_hold_cap_car,
                                                lane_hold_cap_truck,
                                                lane_flow_cap_car,
                                                lane_flow_cap_truck,
                                                ffs_car,
                                                ffs_truck,
                                                unit_time,
                                                veh_convert_factor,
                                                flow_scalar);
            break;
        default:
            printf("Wrong link type.\n");
            exit(-1);
    }
    m_link_map.insert({ID, _link});
    return _link;
}

/******************************************************************************************************************
*******************************************************************************************************************
												Bus Path
*******************************************************************************************************************
******************************************************************************************************************/
MNM_BusPath::MNM_BusPath(TInt route_ID)
        : MNM_Path::MNM_Path() {
    m_busstop_vec = std::deque<TInt>();
    m_route_ID = route_ID;
}

MNM_BusPath::~MNM_BusPath()
{
    m_link_vec.clear();
    m_node_vec.clear();
    m_busstop_vec.clear();
    if (m_buffer != nullptr) free(m_buffer);
}

TFlt MNM_BusPath::get_busroute_fftt(MNM_Link_Factory *link_factory, MNM_Busstop* start_busstop, MNM_Busstop* end_busstop, TFlt unit_interval) {

    auto *_start_link = dynamic_cast<MNM_Dlink_Multiclass*>(link_factory->get_link(start_busstop->m_link_ID));
    auto *_end_link = dynamic_cast<MNM_Dlink_Multiclass*>(link_factory->get_link(end_busstop->m_link_ID));
    if (_start_link -> m_link_ID == _end_link -> m_link_ID) {  // start and end bus stops on the same link
        return (end_busstop -> m_link_loc - start_busstop -> m_link_loc) / _start_link -> m_ffs_truck / unit_interval;
    }

    TFlt fftt;
    TFlt first_interval = (_start_link -> m_length - start_busstop -> m_link_loc) / _start_link -> m_ffs_truck / unit_interval;
    TFlt last_interval = end_busstop -> m_link_loc / _end_link -> m_ffs_truck / unit_interval;
    auto _start_it = find(m_link_vec.begin(), m_link_vec.end(), start_busstop->m_link_ID);
    auto _end_it = find(m_link_vec.begin(), m_link_vec.end(), end_busstop->m_link_ID);
    if (_start_it == m_link_vec.end()) {
        printf("Error, MNM_BusPath::get_busroute_fftt, start bus stop not on this route");
        exit(-1);
    }
    if (_end_it == m_link_vec.end()) {
        printf("Error, MNM_BusPath::get_busroute_fftt, end bus stop not on this route");
        exit(-1);
    }
    fftt = first_interval + last_interval;
    if (_start_it - m_link_vec.begin() + 1 == _end_it - m_link_vec.begin()) { // start and end bus stops on two consecutive links
        return fftt;
    }

    for (int i = _start_it - m_link_vec.begin() + 1; _end_it - m_link_vec.begin(); ++i) {
        auto _link = dynamic_cast<MNM_Dlink_Multiclass*>(link_factory->get_link(m_link_vec[i]));
        fftt += _link -> m_length / _link -> m_ffs_truck / unit_interval;
    }
    return fftt;
}

TFlt MNM_BusPath::get_busroute_tt(TFlt start_time, MNM_Link_Factory *link_factory,
                                  MNM_Busstop* start_busstop, MNM_Busstop* end_busstop, TFlt unit_interval) {

    TFlt fftt = get_busroute_fftt(link_factory, start_busstop, end_busstop, unit_interval); // intervals

    // _waiting_time should near zero when used in Passenger Path classes, since _waiting_time has been calculated outside
    TFlt _waiting_time = start_busstop ->get_bus_waiting_time(start_time, m_route_ID);
    TFlt _true_start_time = start_time + _waiting_time;

    TFlt _cc_flow = start_busstop -> m_N_in_bus.find(m_route_ID) -> second -> get_result(_true_start_time);
    if (_cc_flow <= DBL_EPSILON) {
        return fftt;  // free flow travel time, # of unit intervals
    }
    TFlt _end_time = end_busstop -> m_N_out_bus.find(m_route_ID) -> second -> get_time(_cc_flow);
    if (_end_time() < 0 || (_end_time - start_time < 0) || (_end_time - start_time < fftt)) {
//        printf("Something wrong in bus cc of busstops\n");
//        exit(-1);
        return fftt;  // # of unit intervals
    } else {
        return _end_time - start_time;  // # of unit intervals
    }
}

TFlt MNM_BusPath::get_whole_busroute_tt(TFlt start_time, MNM_Link_Factory *link_factory,
                                        MNM_Busstop_Factory* busstop_factory, TFlt unit_interval) {

    TInt _start_busstop_ID = m_busstop_vec.front();
    TInt _end_busstop_ID = m_busstop_vec.back();
    MNM_Busstop* _start_busstop = busstop_factory -> get_busstop(_start_busstop_ID);
    MNM_Busstop* _end_busstop = busstop_factory -> get_busstop(_end_busstop_ID);

    TFlt tt = get_busroute_tt(start_time, link_factory, _start_busstop, _end_busstop, unit_interval);

    return tt;
}

/******************************************************************************************************************
*******************************************************************************************************************
												Routing
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                          Bus Fixed routing
**************************************************************************/
MNM_Routing_Bus::MNM_Routing_Bus(PNEGraph &graph, MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
                                 MNM_Link_Factory *link_factory, Bus_Path_Table *bus_path_table, TInt route_frq,
                                 TInt buffer_length, TInt veh_class)
        : MNM_Routing_Biclass_Fixed::MNM_Routing_Biclass_Fixed(graph, od_factory, node_factory,
                                                               link_factory, route_frq, buffer_length, veh_class){
    m_veh_class = 1; // bus is treated as truck
    m_bus_path_table = bus_path_table;
}

MNM_Routing_Bus::~MNM_Routing_Bus() {
    for (auto _map_it : m_tracker){
        _map_it.second -> clear();
        delete _map_it.second;
    }
    m_tracker.clear();

    // clear bus_path_table
    if ((m_bus_path_table != nullptr) && (!m_bus_path_table->empty())){
        // printf("Address of m_bus_path_table is %p\n", (void *)m_bus_path_table);
        // printf("%d\n", m_bus_path_table -> size());
        for (auto _it : *m_bus_path_table){ // origin
            for (auto _it_it : *(_it.second)){ // destination
                for (auto _it_it_it : *(_it_it.second)) { // route ID
                    delete _it_it_it.second; // bus_path
                }
                _it_it.second -> clear();
                delete _it_it.second;
            }
            _it.second -> clear();
            delete _it.second;
        }
        m_bus_path_table -> clear();
        delete m_bus_path_table;
    }
}

// register each vehicle with a route based on the portion of path flow
int MNM_Routing_Bus::register_veh(MNM_Veh* veh)
{
    // printf("%d\n", veh -> get_origin() -> m_origin_node  -> m_node_ID);
    // printf("%d\n", veh -> get_destination() -> m_dest_node  -> m_node_ID);

    MNM_Veh_Multimodal *_veh_multimodal = dynamic_cast<MNM_Veh_Multimodal*>(veh);
    IAssert(_veh_multimodal != nullptr);
    MNM_BusPath *_route_path = m_bus_path_table -> find(_veh_multimodal -> get_origin() -> m_origin_node -> m_node_ID) ->
            second -> find(_veh_multimodal -> get_destination() -> m_dest_node  -> m_node_ID) ->
            second -> find(_veh_multimodal -> m_bus_route_ID) -> second;
    // printf("1\n");
    // note m_path_vec is an ordered vector, not unordered

    // printf("3\n");
    if (_route_path == nullptr){
        printf("Wrong bus route in register_veh!\n");
        exit(-1);
    }
    std::deque<TInt> *_link_queue = new std::deque<TInt>();
    // copy links in the route to _link_queue https://www.cplusplus.com/reference/iterator/back_inserter/
    std::copy(_route_path -> m_link_vec.begin(), _route_path -> m_link_vec.end(), std::back_inserter(*_link_queue));
    // printf("old link q is %d, New link queue is %d\n", _route_path -> m_link_vec.size(), _link_queue -> size());
    m_tracker.insert(std::pair<MNM_Veh*, std::deque<TInt>*>(veh, _link_queue));
    veh -> m_path = _route_path;  // base vehicle class
    return 0;
}

int MNM_Routing_Bus::init_routing(Path_Table *path_table)
{
    return 0;
}

int MNM_Routing_Bus::update_routing(TInt timestamp) {
    MNM_Origin *_origin;
    MNM_DMOND *_origin_node;
    TInt _node_ID, _next_link_ID;
    MNM_Dlink *_next_link;
    MNM_Veh *_veh;
    MNM_Veh_Multimodal *_veh_multimodal;
    TInt _cur_ass_int;

    for (auto _origin_it = m_od_factory -> m_origin_map.begin(); _origin_it != m_od_factory -> m_origin_map.end(); _origin_it++){
        _origin = _origin_it -> second;
        _origin_node = _origin -> m_origin_node;
        _node_ID = _origin_node -> m_node_ID;
        for (auto _veh_it = _origin_node -> m_in_veh_queue.begin(); _veh_it != _origin_node -> m_in_veh_queue.end(); _veh_it++){
            _veh = *_veh_it;
            _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal*>(_veh);
            // if success, change _veh with _veh_multimodal
            IAssert(_veh_multimodal != nullptr);
            // Here is the difference from single-class fixed routing
            if ((_veh -> m_type == MNM_TYPE_STATIC) && (_veh -> get_class() == m_veh_class) && (_veh -> get_bus_route_ID() != TInt(-1))) {
                // Here is the difference from single-class fixed routing

                if (m_tracker.find(_veh) == m_tracker.end()){
                    // printf("Registering!\n");
                    register_veh(_veh);
                    _next_link_ID = m_tracker.find(_veh) -> second -> front();
                    _next_link = m_link_factory -> get_link(_next_link_ID);
                    _veh -> set_next_link(_next_link);
                    m_tracker.find(_veh) -> second -> pop_front();
                }
            }
        }
    }
    // printf("Finished route OD veh\n");

    MNM_Destination *_veh_dest;
    MNM_Dlink *_link;
    for (auto _link_it = m_link_factory -> m_link_map.begin(); _link_it != m_link_factory -> m_link_map.end(); _link_it ++){
        _link = _link_it -> second;
        _node_ID = _link -> m_to_node -> m_node_ID;
        for (auto _veh_it = _link -> m_finished_array.begin(); _veh_it != _link -> m_finished_array.end(); _veh_it++){
            _veh = *_veh_it;

            // Here is the difference from single-class fixed routing
            if ((_veh -> m_type == MNM_TYPE_STATIC) && (_veh -> get_class() == m_veh_class) && (_veh -> get_bus_route_ID() != TInt(-1))){
                // Here is the difference from single-class fixed routing

                _veh_dest = _veh -> get_destination();
                if (_veh_dest -> m_dest_node -> m_node_ID == _node_ID){
                    if (m_tracker.find(_veh) -> second -> size() != 0){
                        printf("Something wrong in fixed bus routing!\n");
                        exit(-1);
                    }
                    _veh -> set_next_link(nullptr);
                }
                else{
                    if (m_tracker.find(_veh) == m_tracker.end()){
                        printf("Vehicle not registered in link, impossible!\n");
                        exit(-1);
                    }
                    if (_veh -> get_current_link() == _veh -> get_next_link()){
                        _next_link_ID = m_tracker.find(_veh) -> second -> front();
                        if (_next_link_ID == -1){
                            printf("Something wrong in routing, wrong next link 2\n");
                            printf("The node is %d, the vehicle should head to %d\n", (int)_node_ID, (int)_veh_dest -> m_dest_node -> m_node_ID);
                            exit(-1);
                        }
                        _next_link = m_link_factory -> get_link(_next_link_ID);
                        _veh -> set_next_link(_next_link);
                        m_tracker.find(_veh) -> second -> pop_front();
                    }
                } //end if-else
            } //end if veh->m_type
        } //end for veh_it
    } //end for link_it

    return 0;
}

/**************************************************************************
                          Biclass_Bus_Hybrid routing
**************************************************************************/
MNM_Routing_Biclass_Bus_Hybrid::MNM_Routing_Biclass_Bus_Hybrid(const std::string& file_folder, PNEGraph &graph,
                                                               MNM_Statistics* statistics, MNM_OD_Factory *od_factory,
                                                               MNM_Node_Factory *node_factory,
                                                               MNM_Link_Factory *link_factory,
                                                               Bus_Path_Table  *bus_path_table,
                                                               TInt route_frq_fixed, TInt buffer_length)
        : MNM_Routing_Biclass_Hybrid::MNM_Routing_Biclass_Hybrid(file_folder, graph, statistics, od_factory,
                                                                 node_factory, link_factory, route_frq_fixed, buffer_length)
{
    m_routing_fixed_bus = new MNM_Routing_Bus(graph, od_factory, node_factory, link_factory, bus_path_table,
                                              route_frq_fixed, buffer_length, TInt(1));
}

MNM_Routing_Biclass_Bus_Hybrid::~MNM_Routing_Biclass_Bus_Hybrid()
{
    delete m_routing_fixed_bus;
    // printf("m_routing_fixed_bus\n");
}

int MNM_Routing_Biclass_Bus_Hybrid::init_routing(Path_Table *path_table)
{
    m_routing_adaptive -> init_routing();
    // printf("Finished init all ADAPTIVE vehicles routing\n");
    m_routing_fixed_car -> init_routing(path_table);
    // printf("Finished init STATIC cars routing\n");
    m_routing_fixed_truck -> init_routing(path_table);
    // printf("Finished init STATIC trucks routing\n");
    m_routing_fixed_bus -> init_routing(path_table);
    // printf("Finished init STATIC buses routing\n");
    return 0;
}

int MNM_Routing_Biclass_Bus_Hybrid::update_routing(TInt timestamp)
{
    m_routing_adaptive -> update_routing(timestamp);
    // printf("Finished update all ADAPTIVE vehicles routing\n");
    m_routing_fixed_car -> update_routing(timestamp);
    // printf("Finished update STATIC cars routing\n");
    m_routing_fixed_truck -> update_routing(timestamp);
    // printf("Finished update STATIC trucks routing\n");
    m_routing_fixed_bus -> update_routing(timestamp);
    // printf("Finished update STATIC buses routing\n");
    return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
												Multimodal IO Functions
*******************************************************************************************************************
******************************************************************************************************************/

int MNM_IO_Multimodal::build_node_factory_multimodal(const std::string &file_folder, MNM_ConfReader *conf_reader,
                                                     MNM_Node_Factory *node_factory, const std::string &file_name) {
    /* find file */
    std::string _node_file_name = file_folder + "/" + file_name;
    std::ifstream _node_file;
    _node_file.open(_node_file_name, std::ios::in);

    /* read config */
    TInt _num_of_node = conf_reader -> get_int("num_of_node");
    TFlt _flow_scalar = conf_reader -> get_float("flow_scalar");

    /* read file */
    std::string _line;
    std::vector<std::string> _words;
    TInt _node_ID;
    std::string _type;
    TFlt _veh_convert_factor;

    MNM_Node_Factory_Multimodal* _node_factory = dynamic_cast<MNM_Node_Factory_Multimodal *>(node_factory);
    IAssert(_node_factory != nullptr);
    if (_node_file.is_open())
    {
        std::getline(_node_file,_line); //skip the first line
        for (int i = 0; i < _num_of_node; ++i){
            std::getline(_node_file,_line);
            // printf("%d\n", i);
            _words = split(_line, ' ');
            if (_words.size() == 3) {
                _node_ID = TInt(std::stoi(_words[0]));
                _type = trim(_words[1]);
                _veh_convert_factor = TFlt(std::stod(_words[2]));
                if (_type == "FWJ"){
                    _node_factory -> make_node_multimodal(_node_ID,
                                                          MNM_TYPE_FWJ_MULTIMODAL,
                                                          _flow_scalar,
                                                          _veh_convert_factor);
                    continue;
                }
                if (_type =="DMOND"){
                    _node_factory -> make_node_multimodal(_node_ID,
                                                          MNM_TYPE_ORIGIN_MULTIMODAL,
                                                          _flow_scalar,
                                                          _veh_convert_factor);
                    continue;
                }
                if (_type =="DMDND"){
                    _node_factory -> make_node_multimodal(_node_ID,
                                                          MNM_TYPE_DEST_MULTIMODAL,
                                                          _flow_scalar,
                                                          _veh_convert_factor);
                    continue;
                }
                printf("Wrong node type, %s\n", _type.c_str());
                exit(-1);
            }
            else {
                printf("MNM_IO_Multimodal::build_node_factory_multimodal: Wrong length of line.\n");
                exit(-1);
            }
        }
        _node_file.close();
    }
    // node_factory -> get_node(TInt(1));
    return 0;
}

int MNM_IO_Multimodal::build_link_factory_multimodal(const std::string &file_folder, MNM_ConfReader *conf_reader,
                                                     MNM_Link_Factory *link_factory, const std::string &file_name) {
    /* find file */
    std::string _link_file_name = file_folder + "/" + file_name;
    std::ifstream _link_file;
    _link_file.open(_link_file_name, std::ios::in);

    /* read config */
    TInt _num_of_link = conf_reader -> get_int("num_of_link");
    TFlt _flow_scalar = conf_reader -> get_float("flow_scalar");
    TFlt _unit_time = conf_reader -> get_float("unit_time");

    /* read file */
    std::string _line;
    std::vector<std::string> _words;
    TInt _link_ID;
    TFlt _lane_hold_cap_car;
    TFlt _lane_flow_cap_car;
    TInt _number_of_lane;
    TFlt _length;
    TFlt _ffs_car;
    std::string _type;
    // new in multiclass vehicle case
    TFlt _lane_hold_cap_truck;
    TFlt _lane_flow_cap_truck;
    TFlt _ffs_truck;
    TFlt _veh_convert_factor;

    MNM_Link_Factory_Multimodal* _link_factory = dynamic_cast<MNM_Link_Factory_Multimodal *>(link_factory);
    IAssert(_link_factory != nullptr);
    if (_link_file.is_open())
    {
        // printf("Start build link factory.\n");
        std::getline(_link_file,_line); //skip the first line
        for (int i = 0; i < _num_of_link; ++i){
            std::getline(_link_file,_line);
            _words = split(_line, ' ');
            if (_words.size() == 11) {
                _link_ID = TInt(std::stoi(_words[0]));
                _type = trim(_words[1]);
                _length = TFlt(std::stod(_words[2]));
                _ffs_car = TFlt(std::stod(_words[3]));
                _lane_flow_cap_car = TFlt(std::stod(_words[4]));  // flow capacity (vehicles/hour/lane)
                _lane_hold_cap_car = TFlt(std::stod(_words[5]));  // jam density (vehicles/mile/lane)
                _number_of_lane = TInt(std::stoi(_words[6]));
                // new in multiclass vehicle case
                _ffs_truck = TFlt(std::stod(_words[7]));
                _lane_flow_cap_truck = TFlt(std::stod(_words[8]));
                _lane_hold_cap_truck = TFlt(std::stod(_words[9]));
                _veh_convert_factor = TFlt(std::stod(_words[10]));

                /* unit conversion */
                // mile -> meter, hour -> second
                _length = _length * TFlt(1600); // m
                _ffs_car = _ffs_car * TFlt(1600) / TFlt(3600); // m/s
                _lane_flow_cap_car = _lane_flow_cap_car / TFlt(3600);  // vehicles/s/lane
                _lane_hold_cap_car = _lane_hold_cap_car / TFlt(1600);  // vehicles/m/lane
                _ffs_truck = _ffs_truck * TFlt(1600) / TFlt(3600);  // m/s
                _lane_flow_cap_truck = _lane_flow_cap_truck / TFlt(3600);  // vehicles/s/lane
                _lane_hold_cap_truck = _lane_hold_cap_truck / TFlt(1600);  // vehicles/m/lane

                /* build */
                if (_type == "PQ"){
                    _link_factory -> make_link_multimodal(_link_ID,
                                                          MNM_TYPE_PQ_MULTIMODAL,
                                                          _number_of_lane,
                                                          _length,
                                                          _lane_hold_cap_car,
                                                          _lane_hold_cap_truck,
                                                          _lane_flow_cap_car,
                                                          _lane_flow_cap_truck,
                                                          _ffs_car,
                                                          _ffs_truck,
                                                          _unit_time,
                                                          _veh_convert_factor,
                                                          _flow_scalar);
                    continue;
                }
                if (_type =="CTM"){
                    _link_factory -> make_link_multimodal(_link_ID,
                                                          MNM_TYPE_CTM_MULTIMODAL,
                                                          _number_of_lane,
                                                          _length,
                                                          _lane_hold_cap_car,
                                                          _lane_hold_cap_truck,
                                                          _lane_flow_cap_car,
                                                          _lane_flow_cap_truck,
                                                          _ffs_car,
                                                          _ffs_truck,
                                                          _unit_time,
                                                          _veh_convert_factor,
                                                          _flow_scalar);
                    continue;
                }
                printf("Wrong link type, %s\n", _type.c_str());
                exit(-1);
            }
            else{
                printf("MNM_IO_Multimodal::build_link_factory_multimodal::Wrong length of line.\n");
                exit(-1);
            }
        }
        _link_file.close();
    }
    return 0;
}

int MNM_IO_Multimodal::build_busstop_factory(const std::string& file_folder,
                                             MNM_ConfReader *conf_reader,
                                             MNM_Busstop_Factory *busstop_factory,
                                             MNM_Link_Factory *link_factory,
                                             const std::string& file_name) {
    /* find file */
    std::string _busstop_file_name = file_folder + "/" + file_name;
    std::ifstream _busstop_file;
    _busstop_file.open(_busstop_file_name, std::ios::in);

    /* read config */
    TInt _num_busstops = conf_reader -> get_int("num_of_busstops");

    TInt _busstop_ID, _link_ID, _route_ID;
    TFlt _link_loc;
    // MNM_Dlink_Ctm_Multimodal *_link;
    std::string _line;
    std::vector<std::string> _words;
    auto *_routeID_vec = new std::vector<TInt>();
    if (_busstop_file.is_open())
    {
        std::getline(_busstop_file,_line); //skip the first line
        for (int i = 0; i < _num_busstops; ++i) {
            std::getline(_busstop_file, _line);
            _words = split(_line, ' ');
            if (_words.size() >= 4) {
                _busstop_ID = TInt(std::stoi(_words[0]));
                _link_ID = TInt(std::stoi(_words[1]));
                _link_loc = TFlt(std::stof(_words[2]));

                for (size_t j = 3; j < _words.size(); ++j) {
                    _routeID_vec->push_back(TInt(std::stoi(_words[j])));
                }

                /* unit conversion */
                // mile -> meter, hour -> second
                _link_loc = _link_loc * TFlt(1600);

                /* make busstop factory */
                // _routeID_vec is reset
                busstop_factory -> make_busstop(_busstop_ID, _link_ID, _link_loc, _routeID_vec);
                if (!_routeID_vec->empty()) _routeID_vec -> clear();

                /* hook busstops with link */
                auto *_link = dynamic_cast<MNM_Dlink_Ctm_Multimodal *>(link_factory -> get_link(_link_ID));
                if (_link != nullptr) { // ctm link
                    TInt cell_ID;
                    if (_link -> m_num_cells == 1) {
                        cell_ID = TInt(0);
                        busstop_factory -> get_busstop(_busstop_ID) -> m_cell_ID = cell_ID;
                    } else {
                        cell_ID = TInt(ceil(_link_loc / _link -> m_cell_array[0] -> m_cell_length)) - 1;
                        if (cell_ID < 0) {
                            cell_ID = TInt(0);
                        } else if (cell_ID + 1 > _link -> m_num_cells) {
                            cell_ID = _link -> m_num_cells - 1;
                        }
                        busstop_factory -> get_busstop(_busstop_ID) -> m_cell_ID = cell_ID;
                    }
                    if (_link -> m_cell_busstop_vec.find(cell_ID) == _link -> m_cell_busstop_vec.end()) {
                        _link -> m_cell_busstop_vec.insert(std::pair<TInt, std::vector<MNM_Busstop*>>(cell_ID, std::vector<MNM_Busstop*>()));
                    }
                    _link -> m_cell_busstop_vec.find(cell_ID) -> second.push_back(busstop_factory -> get_busstop(_busstop_ID));
                    _link -> m_busstop_vec.push_back(busstop_factory -> get_busstop(_busstop_ID));
                } else {
                    auto *_link = dynamic_cast<MNM_Dlink_Pq_Multimodal *>(link_factory -> get_link(_link_ID));
                    _link -> m_busstop_vec.push_back(busstop_factory -> get_busstop(_busstop_ID));
                }


            }
            else {
                printf("Wrong line in bus_stops file!\n");
            }
        }
    }
    else {
        printf("Something wrong in build_busstop!\n");
        _routeID_vec -> clear();
        delete _routeID_vec;
        exit(-1);
    }

    _routeID_vec -> clear();
    delete _routeID_vec;
    _busstop_file.close();
    return 0;
}

int MNM_IO_Multimodal::build_parkinglot_factory(const std::string &file_folder, MNM_ConfReader *conf_reader,
                                                MNM_Parking_Lot_Factory *parkinglot_factory,
                                                MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory,
                                                const std::string &file_name) {
    /* find file */
    std::string _parkinglot_file_name = file_folder + "/" + file_name;
    std::ifstream _parkinglot_file;
    _parkinglot_file.open(_parkinglot_file_name, std::ios::in);

    /* read config */
    TInt _num_parkinglots = conf_reader -> get_int("num_of_parkinglots");

    TInt _parkinglot_ID, _node_ID;
    MNM_Parking_Lot *_parkinglot;
    TFlt _price, _price_surge_coeff, _avg_parking_time, _capacity;
    auto *_in_linkID_vec = new std::vector<TInt>();

    // MNM_Dlink_Ctm_Multimodal *_link;
    std::string _line;
    std::vector<std::string> _words;

    if (_parkinglot_file.is_open())
    {
        std::getline(_parkinglot_file, _line); //skip the first line
        for (int i = 0; i < _num_parkinglots; ++i) {
            std::getline(_parkinglot_file, _line);
            _words = split(_line, ' ');
            if (_words.size() >= 7) {
                _parkinglot_ID = TInt(std::stoi(_words[0]));
                _node_ID = TInt(std::stoi(_words[1]));
                _price = TFlt(std::stof(_words[2]));
                _price_surge_coeff = TFlt(std::stof(_words[3]));
                _avg_parking_time = TFlt(std::stof(_words[4]));
                _capacity = TFlt(std::stof(_words[5]));
                for (size_t j = 6; j < _words.size(); ++j) {
                    _in_linkID_vec->push_back(TInt(std::stoi(_words[j])));
                }

                /* unit conversion */
                // mile -> meter, hour -> second

                /* make parkinglot factory */
                // _in_linkID_vec is reset
                parkinglot_factory -> make_parking_lot(_parkinglot_ID, _node_ID, _in_linkID_vec,
                                                       _price, _price_surge_coeff, _avg_parking_time, _capacity);
                if (!_in_linkID_vec->empty()) _in_linkID_vec -> clear();

                /* hook parking lots with link */
                _parkinglot = parkinglot_factory ->get_parking_lot(_parkinglot_ID);
                _parkinglot -> m_node = node_factory ->get_node(_node_ID);
                for (auto _link_ID : _parkinglot -> m_in_linkID_vec) {
                    _parkinglot -> m_in_link_vec.push_back(link_factory -> get_link(_link_ID));
                }
            }
            else {
                printf("Wrong line in parking_lots file!\n");
            }
        }
    }
    else {
        printf("Something wrong in build_parkinglot!\n");
        _in_linkID_vec -> clear();
        delete _in_linkID_vec;
        exit(-1);
    }

    _in_linkID_vec -> clear();
    delete _in_linkID_vec;
    _parkinglot_file.close();
    return 0;
}

int MNM_IO_Multimodal::build_walkinglink_factory(const std::string& file_folder,
                                                 MNM_ConfReader *conf_reader,
                                                 MNM_Walking_Link_Factory *walkinglink_factory,
                                                 const std::string& file_name) {
    /* find file */
    std::string _walkinglink_file_name = file_folder + "/" + file_name;
    std::ifstream _walkinglink_file;
    _walkinglink_file.open(_walkinglink_file_name, std::ios::in);

    /* read config */
    TInt _num_walkinglinks = conf_reader -> get_int("num_of_walkinglinks");

    TInt _walkinglink_ID, _from_node_ID, _to_node_ID;
    TFlt _walking_time;
    std::string _from_node_type;
    std::string _to_node_type;

    std::string _line;
    std::vector<std::string> _words;

    if (_walkinglink_file.is_open())
    {
        std::getline(_walkinglink_file, _line); //skip the first line
        for (int i = 0; i < _num_walkinglinks; ++i) {
            std::getline(_walkinglink_file, _line);
            _words = split(_line, ' ');
            if (_words.size() == 6) {
                _walkinglink_ID = TInt(std::stoi(_words[0]));
                _from_node_ID = TInt(std::stoi(_words[1]));
                _to_node_ID = TInt(std::stoi(_words[2]));
                _from_node_type = _words[3];
                _to_node_type = _words[4];
                _walking_time = TFlt(std::stof(_words[5]));

                /* unit conversion */
                // mile -> meter, hour -> second

                /* make walkinglink factory */
                walkinglink_factory -> make_walking_link(_walkinglink_ID, _from_node_type, _to_node_type,
                                                         _from_node_ID, _to_node_ID, _walking_time);

            }
            else {
                printf("Wrong line in walking_links file!\n");
            }
        }
    }
    else {
        printf("Something wrong in build_walkinglink!\n");
        exit(-1);
    }

    _walkinglink_file.close();
    return 0;
}

int MNM_IO_Multimodal::build_demand_multimodal(const std::string& file_folder,
                                               MNM_ConfReader *conf_reader,
                                               MNM_OD_Factory *od_factory,
                                               const std::string& file_name_driving,
                                               const std::string& file_name_bus){
    /* find file for car and truck demand */
    std::string _demand_file_name = file_folder + "/" + file_name_driving;
    std::ifstream _demand_file;
    _demand_file.open(_demand_file_name, std::ios::in);

    /* find file for bus demand */
    std::string _bus_demand_file_name = file_folder + "/" + file_name_bus;
    std::ifstream _bus_demand_file;
    _bus_demand_file.open(_bus_demand_file_name, std::ios::in);

    /* read config */
    TInt _unit_time = conf_reader -> get_int("unit_time");
    TInt _num_of_minute = int(conf_reader -> get_int("assign_frq")) / (60 / _unit_time);  // the releasing strategy is assigning vehicles per 1 minute
    TInt _max_interval = conf_reader -> get_int("max_interval");
    // TInt _num_OD = conf_reader -> get_int("OD_pair");
    TInt _num_OD_driving = conf_reader -> get_int("OD_pair_driving");
    // TInt _num_OD_bus = conf_reader -> get_int("OD_pair_bus");
    TInt _num_bus_routes = conf_reader -> get_int("num_bus_routes");

    /* build car and truck demand for origin nodes */
    TInt _Origin_ID, _Dest_ID;
    MNM_Origin_Multimodal *_origin;
    MNM_Destination_Multimodal *_dest;
    std::string _line;
    std::vector<std::string> _words;
    if (_demand_file.is_open())
    {
        // printf("Start build demand profile.\n");
        TFlt *_demand_vector_car = (TFlt*) malloc(sizeof(TFlt) * _max_interval * _num_of_minute);
        TFlt *_demand_vector_truck = (TFlt*) malloc(sizeof(TFlt) * _max_interval * _num_of_minute);
        memset(_demand_vector_car, 0x0, sizeof(TFlt) * _max_interval * _num_of_minute);
        memset(_demand_vector_truck, 0x0, sizeof(TFlt) * _max_interval * _num_of_minute);
        TFlt _demand_car;
        TFlt _demand_truck;

        std::getline(_demand_file,_line); //skip the first line
        for (int i = 0; i < _num_OD_driving; ++i){
            std::getline(_demand_file,_line);
            _words = split(_line, ' ');
            if (TInt(_words.size()) == (_max_interval * 2 + 2)) {
                _Origin_ID = TInt(std::stoi(_words[0]));
                _Dest_ID = TInt(std::stoi(_words[1]));
                // the releasing strategy is assigning vehicles per 1 minute, so disaggregate 15-min demand into 1-min demand
                for (int j = 0; j < _max_interval; ++j) {
                    _demand_car = TFlt(std::stod(_words[j + 2])) / TFlt(_num_of_minute);
                    _demand_truck = TFlt(std::stod(_words[j + _max_interval + 2])) / TFlt(_num_of_minute);
                    for (int k = 0; k < _num_of_minute; ++k){
                        _demand_vector_car[j * _num_of_minute + k] = _demand_car;
                        _demand_vector_truck[j * _num_of_minute + k] = _demand_truck;
                    }
                }
                _origin = dynamic_cast<MNM_Origin_Multimodal *>(od_factory -> get_origin(_Origin_ID));
                _dest = dynamic_cast<MNM_Destination_Multimodal *>(od_factory -> get_destination(_Dest_ID));
                _origin -> add_dest_demand_multiclass(_dest, _demand_vector_car, _demand_vector_truck);
            }
            else{
                printf("Something wrong in build_demand!\n");
                free(_demand_vector_car);
                free(_demand_vector_truck);
                exit(-1);
            }
        }
        free(_demand_vector_car);
        free(_demand_vector_truck);
        _demand_file.close();
    }

    /* build bus demand for origin nodes*/
    TInt _route_ID;
    if (_bus_demand_file.is_open())
    {
        // printf("Start build demand profile.\n");
        TFlt *_demand_vector_bus = (TFlt*) malloc(sizeof(TFlt) * _max_interval * _num_of_minute);
        memset(_demand_vector_bus, 0x0, sizeof(TFlt) * _max_interval * _num_of_minute);
        TFlt _demand_bus;

        std::getline(_bus_demand_file,_line); //skip the first line
        for (int i = 0; i < _num_bus_routes; ++i){
            std::getline(_bus_demand_file,_line);
            _words = split(_line, ' ');
            if (TInt(_words.size()) == (_max_interval + 3)) {
                _Origin_ID = TInt(std::stoi(_words[0]));
                _Dest_ID = TInt(std::stoi(_words[1]));
                _route_ID = TInt(std::stoi(_words[2]));
                // the releasing strategy is assigning vehicles per 1 minute, so disaggregate 15-min demand into 1-min demand
                for (int j = 0; j < _max_interval; ++j) {
                    _demand_bus = TFlt(std::stod(_words[j + 3])) / TFlt(_num_of_minute);
                    for (int k = 0; k < _num_of_minute; ++k){
                        _demand_vector_bus[j * _num_of_minute + k] = _demand_bus;
                    }
                }
                _origin = dynamic_cast<MNM_Origin_Multimodal *>(od_factory -> get_origin(_Origin_ID));
                _dest = dynamic_cast<MNM_Destination_Multimodal *>(od_factory -> get_destination(_Dest_ID));
                _origin -> add_dest_demand_bus(_dest, _route_ID, _demand_vector_bus);
            }
            else{
                printf("Something wrong in build_bus_demand!\n");
                free(_demand_vector_bus);
                exit(-1);
            }
        }
        free(_demand_vector_bus);
        _bus_demand_file.close();
    }
    return 0;
}

int MNM_IO_Multimodal::build_passenger_demand(const std::string& file_folder,
                                              MNM_ConfReader *conf_reader,
                                              std::unordered_map<TInt, std::unordered_map<TInt, TFlt*>> *passenger_demand,
                                              const std::string& file_name) {
    /* find file for passenger demand */
    std::string _demand_file_name = file_folder + "/" + file_name;
    std::ifstream _demand_file;
    _demand_file.open(_demand_file_name, std::ios::in);

    /* read config */
    TInt _max_interval = conf_reader -> get_int("max_interval");
    TInt _num_OD = conf_reader -> get_int("OD_pair_passenger");

    /* build passenger demand for assignment */
    TInt _Origin_ID, _Dest_ID;
    std::string _line;
    std::vector<std::string> _words;
    if (_demand_file.is_open())
    {
        // printf("Start build demand profile.\n");
        std::getline(_demand_file,_line); //skip the first line
        for (int i = 0; i < _num_OD; ++i){
            std::getline(_demand_file,_line);
            _words = split(_line, ' ');
            if (TInt(_words.size()) == (_max_interval + 2)) {
                _Origin_ID = TInt(std::stoi(_words[0]));
                _Dest_ID = TInt(std::stoi(_words[1]));
                if (passenger_demand -> find(_Origin_ID) == passenger_demand -> end()) {
                    passenger_demand -> insert(std::pair<TInt, std::unordered_map<TInt, TFlt*>>(_Origin_ID, std::unordered_map<TInt, TFlt*>()));
                }
                if (passenger_demand -> find(_Origin_ID) -> second.find(_Dest_ID) == passenger_demand -> find(_Origin_ID) -> second.end()) {
                    TFlt *_demand_vector = (TFlt*) malloc(sizeof(TFlt) * _max_interval);
                    memset(_demand_vector, 0x0, sizeof(TFlt) * _max_interval);
                    passenger_demand -> find(_Origin_ID) -> second.insert(std::pair<TInt, TFlt*>(_Dest_ID, _demand_vector));
                }
                for (int j = 0; j < _max_interval; ++j) {
                    passenger_demand -> find(_Origin_ID) -> second.find(_Dest_ID) -> second[j] = TFlt(std::stod(_words[j + 2]));
                }
            }
            else{
                printf("Something wrong in build_passenger_demand!\n");
                exit(-1);
            }
        }
        _demand_file.close();
    }
}

Bus_Path_Table *MNM_IO_Multimodal::load_bus_path_table(const PNEGraph& graph, TInt num_path,
                                                       const std::string& path_file_name,
                                                       const std::string& route_file_name)
{
    printf("Loading Bus Path Table!\n");
    TInt Num_Path = num_path;
    printf("Number of bus paths %d\n", Num_Path());

    std::ifstream _path_table_file;
    std::ifstream _route_file;

    _path_table_file.open(path_file_name, std::ios::in);
    _route_file.open(route_file_name, std::ios::in);
    auto *_path_table = new Bus_Path_Table();

    /* read file */
    std::string _path_line, _route_line;
    std::vector<std::string> _path_words, _route_words;
    TInt _origin_node_ID, _dest_node_ID, _node_ID, _route_ID, _busstop_ID;
    std::unordered_map<TInt, std::unordered_map<TInt, MNM_BusPath*>*> *_new_map_1; // <destID, <routeID, Path>>
    std::unordered_map<TInt, MNM_BusPath*> *_new_map_2; // <routeID, Path>
    MNM_BusPath *_path;
    TInt _from_ID, _to_ID, _link_ID;
    TInt _path_ID_counter = 0;
    if (_path_table_file.is_open() && _route_file.is_open()){
        std::getline(_path_table_file,_path_line);  // skip the first line
        std::getline(_route_file,_route_line);  // skip the first line
        for (int i = 0; i < Num_Path; ++i){
            std::getline(_path_table_file,_path_line);
            std::getline(_route_file, _route_line);
            // std::cout << "Processing: " << _line << "\n";
            _path_words = split(_path_line, ' '); // returns a vector
            _route_words = split(_route_line, ' ');
            if ((_path_words.size() >= 3) && (_route_words.size() >= 3)) {
                _origin_node_ID = TInt(std::stoi(_path_words[0]));
                _dest_node_ID = TInt(std::stoi(_path_words[1]));
                _route_ID = TInt(std::stoi(_path_words[2]));
                if (_path_table -> find(_origin_node_ID) == _path_table -> end()){
                    _new_map_1 = new std::unordered_map<TInt, std::unordered_map<TInt, MNM_BusPath*>*>();
                    _path_table -> insert(std::pair<TInt, std::unordered_map<TInt, std::unordered_map<TInt, MNM_BusPath*>*>*>(_origin_node_ID, _new_map_1));
                }
                if (_path_table -> find(_origin_node_ID) -> second -> find(_dest_node_ID) == _path_table -> find(_origin_node_ID) -> second -> end()){
                    _new_map_2 = new std::unordered_map<TInt, MNM_BusPath*>();
                    _path_table -> find(_origin_node_ID) -> second -> insert(std::pair<TInt, std::unordered_map<TInt, MNM_BusPath*>*>(_dest_node_ID, _new_map_2));
                }
                if (_path_table -> find(_origin_node_ID) -> second -> find(_dest_node_ID) -> second -> find(_route_ID) ==
                    _path_table -> find(_origin_node_ID) -> second -> find(_dest_node_ID) -> second -> end()){

                    _path = new MNM_BusPath(_route_ID);
                    // _path -> m_route_ID = _route_ID;
                    _path -> m_path_ID = _path_ID_counter;
                    _path_ID_counter += 1;

                    for (std::size_t j = 3; j < _path_words.size(); ++j) {
                        // read node sequence
                        _node_ID = TInt(std::stoi(_path_words[j]));
                        _path -> m_node_vec.push_back(_node_ID);
                    }

                    for (std::size_t j = 3; j < _route_words.size(); ++j) {
                        // read busstop sequence
                        _busstop_ID = TInt(std::stoi(_route_words[j]));
                        _path -> m_busstop_vec.push_back(_busstop_ID);
                    }


                    for (size_t j = 0; j < _path -> m_node_vec.size() - 1; ++j){
                        _from_ID = _path -> m_node_vec[j];
                        _to_ID = _path -> m_node_vec[j+1];
                        _link_ID = graph -> GetEI(_from_ID, _to_ID).GetId();
                        _path -> m_link_vec.push_back(_link_ID);
                    }

                    _path_table -> find(_origin_node_ID) -> second -> find(_dest_node_ID) -> second -> insert(std::pair<TInt, MNM_BusPath*>(_route_ID, _path));
                }
            }
        }
        _path_table_file.close();
        _route_file.close();
    }
    else{
        printf("Can't open bus path table file!\n");
        exit(-1);
    }
    printf("Finish Loading Bus Path Table!\n");
    // printf("path table %p\n", _path_table);
    // printf("path table %s\n", _path_table -> find(100283) -> second -> find(150153) -> second
    //                           -> m_path_vec.front() -> node_vec_to_string());
    return _path_table;
}


/******************************************************************************************************************
*******************************************************************************************************************
												Multimodal DTA
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Dta_Multimodal::MNM_Dta_Multimodal(const std::string& file_folder)
    :MNM_Dta_Multiclass::MNM_Dta_Multiclass(file_folder)
{
    initialize();
}

MNM_Dta_Multimodal::~MNM_Dta_Multimodal()
{
    delete m_busstop_factory;
}

int MNM_Dta_Multimodal::initialize()
{
    if (m_veh_factory != nullptr) delete m_veh_factory;
    if (m_node_factory != nullptr) delete m_node_factory;
    if (m_link_factory != nullptr) delete m_link_factory;
    if (m_od_factory != nullptr) delete m_od_factory;
    if (m_busstop_factory != nullptr) delete m_busstop_factory;
    if (m_config != nullptr) delete m_config;

    m_veh_factory = new MNM_Veh_Factory_Multimodal();
    // printf("1\n");
    m_node_factory = new MNM_Node_Factory_Multimodal();
    // printf("2\n");
    m_link_factory = new MNM_Link_Factory_Multimodal();
    // printf("3\n");
    m_od_factory = new MNM_OD_Factory_Multimodal();
    // printf("4\n");
    m_busstop_factory = new MNM_Busstop_Factory();
    // printf("5\n");

    m_config = new MNM_ConfReader(m_file_folder + "/config.conf", "DTA");
    m_unit_time = m_config -> get_int("unit_time");
    m_flow_scalar = m_config -> get_int("flow_scalar");
    // printf("5\n");
    m_emission = new MNM_Cumulative_Emission_Multiclass(TFlt(m_unit_time), 0);

    // the releasing strategy is assigning vehicles per 1 minute, so disaggregate 15-min demand into 1-min demand
    // change assign_freq to 12 (1 minute = 12 x 5 second / 60) and total_assign_interval to max_interval*_num_of_minute
    m_assign_freq = 60 / int(m_unit_time);  // # of unit intervals in 1 min = # of assign freq
    TInt _num_of_minute =  int(m_config -> get_int("assign_frq")) / m_assign_freq;  // 15 min, # of minutes in original assign interval
    m_total_assign_inter = m_config -> get_int("max_interval") * _num_of_minute;  // how many 1-min intervals
    m_start_assign_interval = m_config -> get_int("start_assign_interval");

    return 0;
}

int MNM_Dta_Multimodal::set_routing()
{
    // For Bi-class with bus routing
    // Note here still only one path_table and buffer, but in buffer file each row contains:
    // Row #k : [probabilities choosing route k in all intervals for cars] [probabilities choosing route k in all intervals for trucks]
    // For Bi-class Fixed routing, just set both adaptive_ratio_car=0 & adaptive_ratio_truck=0 in "config.conf"
    if (m_config -> get_string("routing_type") == "Biclass_Bus_Hybrid"){
        MNM_ConfReader* _tmp_conf = new MNM_ConfReader(m_file_folder + "/config.conf", "FIXED");
        Path_Table *_path_table;
        if (_tmp_conf -> get_string("choice_portion") == "Buffer"){
            _path_table = MNM_IO::load_path_table(m_file_folder + "/" + _tmp_conf -> get_string("path_file_name"),
                                                  m_graph, _tmp_conf -> get_int("num_path"), true);
        }
        else{
            _path_table = MNM_IO::load_path_table(m_file_folder + "/" + _tmp_conf -> get_string("path_file_name"),
                                                  m_graph, _tmp_conf -> get_int("num_path"), false);
        }
        TInt _buffer_len = _tmp_conf -> get_int("buffer_length");
        TInt _route_freq_fixed = _tmp_conf -> get_int("route_frq");

        Bus_Path_Table *_bus_path_table;
        _bus_path_table = MNM_IO_Multimodal::load_bus_path_table(
                m_graph, _tmp_conf -> get_int("num_bus_routes"),
                m_file_folder + "/" + _tmp_conf -> get_string("bus_path_file_name"),
                m_file_folder + "/" + _tmp_conf -> get_string("bus_route_file_name"));
        m_routing = new MNM_Routing_Biclass_Bus_Hybrid(m_file_folder, m_graph, m_statistics, m_od_factory,
                                                       m_node_factory, m_link_factory, _bus_path_table,
                                                       _route_freq_fixed, _buffer_len);
        m_routing -> init_routing(_path_table);

        delete _tmp_conf;
    }

    else {
        printf("Wrong routing type for Biclass_Bus_Hybrid");
        exit(111);
//        m_routing = new MNM_Routing_Random(m_graph, m_od_factory, m_node_factory, m_link_factory);
//        m_routing -> init_routing();
    }
    return 0;
}

int MNM_Dta_Multimodal::build_from_files()
{
    MNM_IO_Multimodal::build_node_factory_multimodal(m_file_folder, m_config, m_node_factory);
    MNM_IO_Multimodal::build_link_factory_multimodal(m_file_folder, m_config, m_link_factory);
    // MNM_IO_Multiclass::build_od_factory_multiclass(m_file_folder, m_config, m_od_factory, m_node_factory);
    MNM_IO_Multimodal::build_od_factory(m_file_folder, m_config, m_od_factory, m_node_factory);
    m_graph = MNM_IO_Multimodal::build_graph(m_file_folder, m_config);
    MNM_IO_Multimodal::build_demand_multimodal(m_file_folder, m_config, m_od_factory);
    MNM_IO_Multimodal::build_busstop_factory(m_file_folder, m_config, m_busstop_factory, m_link_factory);
    // build_workzone();
    m_workzone = nullptr;
    set_statistics();
    set_routing();
    return 0;
}


int MNM_Dta_Multimodal::load_once(bool verbose, TInt load_int, TInt assign_int)
{
    MNM_Origin *_origin;
    MNM_Dnode *_node;
    MNM_Dlink *_link;
    MNM_Destination *_dest;
    if (verbose) printf("-------------------------------    Interval %d   ------------------------------ \n", (int)load_int);
    // step 1: Origin release vehicle
    if (verbose) printf("Releasing!\n");
    // for (auto _origin_it = m_od_factory -> m_origin_map.begin(); _origin_it != m_od_factory -> m_origin_map.end(); _origin_it++){
    //   _origin = _origin_it -> second;
    //   _origin -> release(m_veh_factory, _cur_int);
    // }
    if (load_int % m_assign_freq == 0 || load_int==0){
        for (auto _origin_it = m_od_factory -> m_origin_map.begin(); _origin_it != m_od_factory -> m_origin_map.end(); _origin_it++){
            _origin = _origin_it -> second;  // base origin class pointer to multimodal origin object
            if (assign_int >= m_total_assign_inter) {
                _origin -> release_one_interval(load_int, m_veh_factory, -1, TFlt(-1));
            }
            else{
                if((m_config -> get_string("routing_type") == "Biclass_Bus_Hybrid")){
                    TFlt _ad_ratio_car = m_config -> get_float("adaptive_ratio_car");
                    if (_ad_ratio_car > 1) _ad_ratio_car = 1;
                    if (_ad_ratio_car < 0) _ad_ratio_car = 0;

                    TFlt _ad_ratio_truck = m_config -> get_float("adaptive_ratio_truck");
                    if (_ad_ratio_truck > 1) _ad_ratio_truck = 1;
                    if (_ad_ratio_truck < 0) _ad_ratio_truck = 0;
                    // NOTE: in this case the release function is different
                    _origin -> release_one_interval_biclass(load_int, m_veh_factory, assign_int, _ad_ratio_car, _ad_ratio_truck);
                }
                else{
                    printf("WARNING:No assignment!\n");
                }
            }
        }
    }

    if (verbose) printf("Routing!\n");
    // step 2: route the vehicle
    m_routing -> update_routing(load_int);


    if (verbose) printf("Moving through node!\n");
    // step 3: move vehicles through node
    for (auto _node_it = m_node_factory -> m_node_map.begin(); _node_it != m_node_factory -> m_node_map.end(); _node_it++){
        _node = _node_it -> second;
        // printf("node ID is %d\n", _node -> m_node_ID());
        _node -> evolve(load_int);
    }

    // record queuing vehicles after node evolve, which is num of vehicles in finished array
    record_queue_vehicles();
    if (verbose) printf("Moving through link\n");
    // step 4: move vehicles through link
    for (auto _link_it = m_link_factory -> m_link_map.begin(); _link_it != m_link_factory -> m_link_map.end(); _link_it++){
        _link = _link_it -> second;
        // if (_link -> get_link_flow() > 0){
        //   printf("Current Link %d:, traffic flow %.4f, incomming %d, finished %d\n",
        //       _link -> m_link_ID(), _link -> get_link_flow()(), (int)_link -> m_incoming_array.size(),  (int)_link -> m_finished_array.size());
        //   _link -> print_info();
        // }
        // printf("link ID is %d\n", _link-> m_link_ID());
        // _link -> print_info();
        _link -> clear_incoming_array(load_int);
        _link -> evolve(load_int);
    }

    if (verbose) printf("Receiving!\n");
    // step 5: Destination receive vehicle
    for (auto _dest_it = m_od_factory -> m_destination_map.begin(); _dest_it != m_od_factory -> m_destination_map.end(); _dest_it++){
        _dest = _dest_it -> second;
        _dest -> receive(load_int);
    }

    if (verbose) printf("Update record!\n");
    // step 5: update record
    m_statistics -> update_record(load_int);

    record_enroute_vehicles();
    MNM::print_vehicle_statistics(m_veh_factory);
    // test();
    return 0;
}

int MNM_Dta_Multimodal::loading(bool verbose) {
    TInt _current_inter = 0;
    TInt _assign_inter = m_start_assign_interval;

    while (!finished_loading(_current_inter)) {
        printf("\nCurrent loading interval: %d, Current assignment interval: %d\n", _current_inter(), _assign_inter());
        load_once(verbose, _current_inter, _assign_inter);
        if (_current_inter % m_assign_freq == 0 || _current_inter == 0) {
            _assign_inter += 1;
        }
        _current_inter += 1;
        // if (_current_inter > 200) break;
    }
    return _current_inter;  // total_loading_inter [0, _current_inter)
}

/******************************************************************************************************************
*******************************************************************************************************************
												Parking lot
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Parking_Lot::MNM_Parking_Lot(TInt ID, TInt node_ID, std::vector<TInt>* in_linkID_vec,
                                 TFlt base_price, TFlt price_surge_coeff, TFlt avg_parking_time, TFlt capacity) {
    m_ID = ID;
    m_base_price = base_price;
    m_price_surge_coeff = price_surge_coeff;
    m_avg_parking_time = avg_parking_time;  // second
    m_max_parking_time = 10 * m_avg_parking_time;  // second
    m_capacity = capacity;

    m_in_linkID_vec = std::vector<TInt>();
    std::copy(in_linkID_vec -> begin(),
              in_linkID_vec -> end(),
              std::back_inserter(m_in_linkID_vec));
    in_linkID_vec -> clear();
    m_in_link_vec = std::vector<MNM_Dlink*>();

    m_node_ID = node_ID;
    m_node = nullptr;
}

MNM_Parking_Lot::~MNM_Parking_Lot() {
    if (m_node != nullptr) delete m_node;
    m_in_linkID_vec.clear();
    for (auto *_link : m_in_link_vec) {
        if (_link != nullptr) delete _link;
    }
    m_in_link_vec.clear();
}

TFlt MNM_Parking_Lot::get_cruise_time(TFlt timestamp, MNM_Dta_Multimodal *mmdta) {
    TFlt _num_veh_out = TFlt(0);
    for (auto _link_ID : m_in_linkID_vec) {
        MNM_Dlink_Multiclass *_link = dynamic_cast<MNM_Dlink_Multiclass*>(mmdta -> m_link_factory -> get_link(TInt(_link_ID)));
        if (_link -> m_N_out_car == nullptr){
            throw std::runtime_error("Error, Mcdta_Api::get_car_link_out_cc, cc not installed");
        } else {
            // only car will park in parking lot
            _num_veh_out += _link -> m_N_out_car -> get_result(timestamp) / mmdta -> m_flow_scalar;
        }
    }

    if (_num_veh_out >= m_capacity) {
        return m_max_parking_time / mmdta->m_unit_time;
    } else {
        return m_avg_parking_time / (1 - _num_veh_out / m_capacity) / mmdta->m_unit_time;  // interval
    }
}


/******************************************************************************************************************
*******************************************************************************************************************
												Parking lot factory
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Parking_Lot_Factory::MNM_Parking_Lot_Factory()
{
    m_parking_lot_map = std::unordered_map<TInt, MNM_Parking_Lot*>();
}

MNM_Parking_Lot_Factory::~MNM_Parking_Lot_Factory()
{
    for (auto _map_it = m_parking_lot_map.begin(); _map_it!= m_parking_lot_map.end(); _map_it++){
        delete _map_it -> second;
    }
    m_parking_lot_map.clear();
}

MNM_Parking_Lot * MNM_Parking_Lot_Factory::make_parking_lot(TInt ID, TInt node_ID, std::vector<TInt>* in_linkID_vec,
                                                            TFlt base_price, TFlt price_surge_coeff, TFlt avg_parking_time, TFlt capacity) {
    MNM_Parking_Lot *_parking_lot = new MNM_Parking_Lot(ID, node_ID, in_linkID_vec, base_price, price_surge_coeff, avg_parking_time, capacity);
    m_parking_lot_map.insert(std::pair<TInt, MNM_Parking_Lot*>(ID, _parking_lot));
    return _parking_lot;
}

MNM_Parking_Lot *MNM_Parking_Lot_Factory::get_parking_lot(TInt ID) {
    auto _parking_lot_it = m_parking_lot_map.find(ID);
    if (_parking_lot_it == m_parking_lot_map.end()) {
        printf("No such parking lot ID %d\n", (int) ID);
        throw std::runtime_error("Error, MNM_Parking_Lot_Factory::get_parking_lot, parking lot does not exist");
    }
    return _parking_lot_it -> second;
}


/******************************************************************************************************************
*******************************************************************************************************************
												Walking link
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Walking_Link::MNM_Walking_Link(TInt ID, const std::string &from_node_type, const std::string &to_node_type,
                                   TInt from_node_ID, TInt to_node_ID, TFlt walking_time) {
    m_link_ID = ID;
    m_from_node_type = from_node_type;
    m_to_node_type = to_node_type;
    m_from_node_ID = from_node_ID;
    m_to_node_ID = to_node_ID;
    m_walking_time = walking_time; // seconds
}

MNM_Walking_Link::~MNM_Walking_Link(){
    ;
}


/******************************************************************************************************************
*******************************************************************************************************************
												Walking Link Factory
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Walking_Link_Factory::MNM_Walking_Link_Factory() {
    m_walking_link_map = std::unordered_map<TInt, MNM_Walking_Link*>();
}

MNM_Walking_Link_Factory::~MNM_Walking_Link_Factory()
{
    for (auto _map_it = m_walking_link_map.begin(); _map_it!= m_walking_link_map.end(); _map_it++){
        delete _map_it -> second;
    }
    m_walking_link_map.clear();
}

MNM_Walking_Link * MNM_Walking_Link_Factory::make_walking_link(TInt ID, const std::string& from_node_type, const std::string& to_node_type,
                                                               TInt from_node_ID, TInt to_node_ID, TFlt walking_time) {
    MNM_Walking_Link *_walking_link = new MNM_Walking_Link(ID, from_node_type, to_node_type, from_node_ID, to_node_ID, walking_time);
    m_walking_link_map.insert(std::pair<TInt, MNM_Walking_Link*>(ID, _walking_link));
    return _walking_link;
}

MNM_Walking_Link *MNM_Walking_Link_Factory::get_walking_link(TInt ID) {
    auto _walking_link_it = m_walking_link_map.find(ID);
    if (_walking_link_it == m_walking_link_map.end()) {
        printf("No such walking link ID %d\n", (int) ID);
        throw std::runtime_error("Error, MNM_Walking_Link_Factory::get_walking_link, walking link does not exist");
    }
    return _walking_link_it -> second;
}

/******************************************************************************************************************
*******************************************************************************************************************
												Passenger path
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
					           Base
**************************************************************************/
MNM_Passenger_Path_Base::MNM_Passenger_Path_Base(int mode_1, int mode_2, TFlt vot, TFlt early_penalty, TFlt late_penalty, TFlt target_time)
        : MNM_Path::MNM_Path(){
    m_mode_1 = mode_1;
    m_mode_2 = mode_2;
    m_vot = vot;  // money / interval
    m_early_penalty = early_penalty;  // money / interval
    m_late_penalty = late_penalty;  // money / interval
    m_target_time = target_time;  // intervals
}

MNM_Passenger_Path_Base::~MNM_Passenger_Path_Base(){
    ;
}

TFlt MNM_Passenger_Path_Base::get_wrongtime_penalty(TFlt arrival_time){
    return MNM_Ults::max(m_late_penalty * (arrival_time - m_target_time), m_early_penalty * (m_target_time - arrival_time));
}


/**************************************************************************
					           Driving
**************************************************************************/
MNM_Passenger_Path_Driving::MNM_Passenger_Path_Driving(int mode_1, int mode_2, TFlt vot, TFlt early_penalty, TFlt late_penalty, TFlt target_time,
                                                       TInt num_people, TFlt carpool_cost_multiplier, TFlt walking_time_before_driving,
                                                       MNM_Parking_Lot* parking_lot, MNM_Walking_Link *walking_link_after_driving, TFlt walking_time_after_driving)
        : MNM_Passenger_Path_Base::MNM_Passenger_Path_Base(mode_1, mode_2, vot, early_penalty, late_penalty, target_time) {
    IAssert(walking_link_after_driving == nullptr || walking_link_after_driving -> m_from_node_type == "parking_lot");
    IAssert(num_people >= 1);
    m_num_people = num_people;

    m_parking_lot = parking_lot;
    m_walking_link_after_driving = walking_link_after_driving;

    m_carpool_cost_multiplier = carpool_cost_multiplier;
    m_walking_time_before_driving = walking_time_before_driving;  // seconds
    if (m_walking_link_after_driving != nullptr) {
        m_walking_time_after_driving = m_walking_link_after_driving -> m_walking_time;  // seconds
    } else {
        m_walking_time_after_driving = walking_time_after_driving; // seconds
    }
}

MNM_Passenger_Path_Driving::~MNM_Passenger_Path_Driving() {
    if (m_parking_lot != nullptr) delete m_parking_lot;
    if (m_walking_link_after_driving != nullptr) delete m_walking_link_after_driving;
}

TFlt MNM_Passenger_Path_Driving::get_carpool_cost() {
    return TFlt(m_carpool_cost_multiplier * (m_num_people - 1));
}

TFlt MNM_Passenger_Path_Driving::get_amortized_parkingfee() {
    if (m_parking_lot == nullptr) {
        return TFlt(0);
    }
    return TFlt(m_parking_lot->m_base_price * m_parking_lot->m_price_surge_coeff / m_num_people);
}

TFlt MNM_Passenger_Path_Driving::get_travel_time(TFlt start_time, MNM_Dta_Multimodal *mmdta) {
    MNM_Dlink *_link;
    TFlt arrival_time = start_time + m_walking_time_before_driving / mmdta->m_unit_time;
    for (TInt _link_ID : m_link_vec) {
        _link = mmdta -> m_link_factory -> get_link(_link_ID);
        if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
            arrival_time += MNM_DTA_GRADIENT::get_travel_time_car(_mclink, TFlt(arrival_time), mmdta -> m_unit_time);
        } else{
            throw std::runtime_error("Mcdta_Api::register_links: link type is not multiclass");
        }
    }
    if (m_parking_lot != nullptr) {
        arrival_time += m_parking_lot->get_cruise_time(arrival_time, mmdta);
    }
    arrival_time += m_walking_time_after_driving / mmdta->m_unit_time;
    return arrival_time - start_time;  // intervals
}

TFlt MNM_Passenger_Path_Driving::get_travel_cost(TFlt start_time, MNM_Dta_Multimodal *mmdta) {
    TFlt tt = get_travel_time(start_time, mmdta);
    TFlt wrongtime_penalty = get_wrongtime_penalty(start_time + tt);
    return m_vot * tt + wrongtime_penalty + get_carpool_cost() + get_amortized_parkingfee();
}

bool MNM_Passenger_Path_Driving::is_equal(MNM_Passenger_Path_Base* path) {
    auto *_path_driving = dynamic_cast<MNM_Passenger_Path_Driving*>(path);
    if (_path_driving == nullptr) {
        return false;
    }
    else {
        if (m_link_vec.size() != _path_driving -> m_link_vec.size()) return false;
        for (size_t i=0; i<_path_driving -> m_link_vec.size(); ++i){
            if (m_link_vec[i] != _path_driving -> m_link_vec[i])
                return false;
        }

        if (m_walking_link_after_driving != _path_driving -> m_walking_link_after_driving) {
            return false;
        }
        return true;
    }
};


/**************************************************************************
					           Bus
**************************************************************************/
MNM_Passenger_Path_Bus::MNM_Passenger_Path_Bus(int mode_1, int mode_2, TFlt vot, TFlt early_penalty, TFlt late_penalty, TFlt target_time,
                                               TInt route_ID, TInt start_busstop_ID, TInt end_busstop_ID, TFlt bus_fare, TFlt bus_inconvenience,
                                               MNM_Walking_Link *walking_link_before_bus, MNM_Walking_Link *walking_link_after_bus)
        : MNM_Passenger_Path_Base::MNM_Passenger_Path_Base(mode_1, mode_2, vot, early_penalty, late_penalty, target_time){

    IAssert(walking_link_before_bus -> m_to_node_type == "bus_stop");
    IAssert(walking_link_after_bus -> m_from_node_type == "bus_stop");

    m_route_ID = route_ID;
    m_start_busstop_ID = start_busstop_ID;
    m_end_busstop_ID = end_busstop_ID;
    m_bus_fare = bus_fare;
    m_bus_inconvenience = bus_inconvenience;
    //m_waiting_time = waiting_time;

    m_walking_link_before_bus = walking_link_before_bus;
    m_walking_link_after_bus = walking_link_after_bus;
    m_walking_time_before_bus = walking_link_before_bus -> m_walking_time;  // seconds
    m_walking_time_after_bus = walking_link_after_bus -> m_walking_time;  // seconds
}

MNM_Passenger_Path_Bus::~MNM_Passenger_Path_Bus() {
    if (m_walking_link_before_bus != nullptr) delete m_walking_link_before_bus;
    if (m_walking_link_after_bus != nullptr) delete m_walking_link_after_bus;
};

TFlt MNM_Passenger_Path_Bus::get_travel_time(TFlt start_time, MNM_Dta_Multimodal *mmdta) {
    MNM_Dlink *_link;
    MNM_Busstop* _start_busstop = mmdta -> m_busstop_factory ->get_busstop(m_start_busstop_ID);
    MNM_Busstop* _end_busstop = mmdta -> m_busstop_factory ->get_busstop(m_end_busstop_ID);

    TFlt arrival_time = start_time + m_walking_time_before_bus / mmdta->m_unit_time;
    arrival_time += _start_busstop ->get_bus_waiting_time(start_time, m_route_ID);

    auto *_routing = dynamic_cast<MNM_Routing_Biclass_Bus_Hybrid*>(mmdta -> m_routing);
    MNM_BusPath* _buspath = _routing -> m_routing_fixed_bus -> m_bus_path_table -> find(m_node_vec.front()) ->
                                        second -> find(m_node_vec.back()) -> second -> find(m_route_ID) -> second;
    arrival_time += _buspath -> get_busroute_tt(start_time, mmdta -> m_link_factory, _start_busstop, _end_busstop, mmdta -> m_unit_time);

    arrival_time += m_walking_time_after_bus / mmdta->m_unit_time;
    return arrival_time - start_time;  // intervals
}

TFlt MNM_Passenger_Path_Bus::get_travel_cost(TFlt start_time, MNM_Dta_Multimodal *mmdta) {
    TFlt tt = get_travel_time(start_time, mmdta);
    TFlt wrongtime_penalty = get_wrongtime_penalty(start_time + tt);
    return m_vot * tt + wrongtime_penalty + m_bus_fare + m_bus_inconvenience;
}

bool MNM_Passenger_Path_Bus::is_equal(MNM_Passenger_Path_Base* path) {
    auto *_path_bus = dynamic_cast<MNM_Passenger_Path_Bus*>(path);
    if (_path_bus == nullptr) {
        return false;
    }
    else {
        if (m_route_ID != _path_bus -> m_route_ID ||
            m_start_busstop_ID != _path_bus -> m_start_busstop_ID ||
            m_end_busstop_ID != _path_bus -> m_end_busstop_ID ||
            m_walking_link_before_bus != _path_bus -> m_walking_link_before_bus ||
            m_walking_link_after_bus != _path_bus -> m_walking_link_after_bus) {
            return false;
        }
        return true;
    }
};

/**************************************************************************
					           Metro
**************************************************************************/
MNM_Passenger_Path_Metro::MNM_Passenger_Path_Metro(int mode_1, int mode_2, TFlt vot, TFlt early_penalty, TFlt late_penalty, TFlt target_time,
                                                   TInt route_ID, TInt start_metrostop_ID, TInt end_metrostop_ID,
                                                   TFlt metro_fare, TFlt metro_inconvenience, TFlt metro_time, TFlt waiting_time,
                                                   MNM_Walking_Link *walking_link_before_metro, MNM_Walking_Link *walking_link_after_metro)
        : MNM_Passenger_Path_Base::MNM_Passenger_Path_Base(mode_1, mode_2, vot, early_penalty, late_penalty, target_time){

    m_route_ID = route_ID;
    m_start_metrostop_ID = start_metrostop_ID;
    m_end_metrostop_ID = end_metrostop_ID;

    m_metro_fare = metro_fare;
    m_metro_inconvenience = metro_inconvenience;
    m_metro_time = metro_time;  // intervals, need metro network

    m_walking_link_before_metro = walking_link_before_metro;
    m_walking_link_after_metro = walking_link_after_metro;
    m_walking_time_before_metro = m_walking_link_before_metro -> m_walking_time;  // seconds
    m_walking_time_after_metro = m_walking_link_after_metro -> m_walking_time;  // seconds
    m_waiting_time = waiting_time;  // seconds
}

MNM_Passenger_Path_Metro::~MNM_Passenger_Path_Metro() {
    if (m_walking_link_before_metro != nullptr) delete m_walking_link_before_metro;
    if (m_walking_link_after_metro != nullptr) delete m_walking_link_after_metro;
}

TFlt MNM_Passenger_Path_Metro::get_travel_time(TFlt start_time, MNM_Dta_Multimodal *mmdta) {
    return m_metro_time + (m_walking_time_before_metro + m_walking_time_after_metro) / mmdta->m_unit_time;  // interval
}

TFlt MNM_Passenger_Path_Metro::get_travel_cost(TFlt start_time, MNM_Dta_Multimodal *mmdta) {
    TFlt tt = get_travel_time(start_time, mmdta);
    TFlt wrongtime_penalty = get_wrongtime_penalty(start_time + tt);
    return m_vot * tt + wrongtime_penalty + m_metro_fare + m_metro_inconvenience;
}

bool MNM_Passenger_Path_Metro::is_equal(MNM_Passenger_Path_Base* path) {
    auto *_path_metro = dynamic_cast<MNM_Passenger_Path_Metro*>(path);
    if (_path_metro == nullptr) {
        return false;
    }
    else {
        if (m_route_ID != _path_metro -> m_route_ID ||
            m_start_metrostop_ID != _path_metro -> m_start_metrostop_ID ||
            m_end_metrostop_ID != _path_metro -> m_end_metrostop_ID ||
            m_walking_link_before_metro != _path_metro -> m_walking_link_before_metro ||
            m_walking_link_after_metro != _path_metro -> m_walking_link_after_metro) {
            return false;
        }
        return true;
    }
};


/**************************************************************************
					           Park & Ride
**************************************************************************/
MNM_Passenger_Path_PnR::MNM_Passenger_Path_PnR(int mode_1, int mode_2, TFlt vot, TFlt early_penalty, TFlt late_penalty, TFlt target_time,
                                               MNM_Passenger_Path_Driving *driving_part,
                                               MNM_Passenger_Path_Bus *bus_part, TFlt pnr_inconvenience)
        : MNM_Passenger_Path_Base::MNM_Passenger_Path_Base(mode_1, mode_2, vot, early_penalty, late_penalty, target_time){
    IAssert(driving_part->m_num_people == 1);
    IAssert(driving_part -> m_walking_link_after_driving -> m_link_ID == bus_part -> m_walking_link_before_bus -> m_link_ID);
    IAssert(driving_part -> m_walking_link_after_driving -> m_from_node_type == "parking_lot");
    IAssert(bus_part -> m_walking_link_before_bus -> m_from_node_type == "parking_lot");
    IAssert(driving_part -> m_walking_link_after_driving -> m_to_node_type == "bus_stop");
    IAssert(bus_part -> m_walking_link_before_bus -> m_to_node_type == "bus_stop");
    m_driving_part = driving_part;
    m_bus_part = bus_part;
    m_pnr_inconvenience = pnr_inconvenience;
}

MNM_Passenger_Path_PnR::~MNM_Passenger_Path_PnR() {
    if (m_driving_part != nullptr) delete m_driving_part;
    if (m_bus_part != nullptr) delete m_bus_part;
}

TFlt MNM_Passenger_Path_PnR::get_travel_time(TFlt start_time, MNM_Dta_Multimodal *mmdta) {
    TFlt arrival_time = start_time + m_driving_part->get_travel_time(start_time, mmdta);
    arrival_time -= m_driving_part -> m_walking_time_after_driving / mmdta -> m_unit_time; // avoid double counting
    arrival_time += m_bus_part->get_travel_time(arrival_time, mmdta);
    return arrival_time - start_time;  // intervals
}

TFlt MNM_Passenger_Path_PnR::get_travel_cost(TFlt start_time, MNM_Dta_Multimodal *mmdta) {
    TFlt tt = get_travel_time(start_time, mmdta);
    TFlt wrongtime_penalty = get_wrongtime_penalty(start_time + tt);
    return m_vot * tt + wrongtime_penalty + m_driving_part->get_amortized_parkingfee() + m_bus_part->m_bus_fare + m_pnr_inconvenience;
}

bool MNM_Passenger_Path_PnR::is_equal(MNM_Passenger_Path_Base* path) {
    auto *_path_pnr = dynamic_cast<MNM_Passenger_Path_PnR*>(path);
    if (_path_pnr == nullptr) {
        return false;
    }
    else {
        if (m_driving_part != _path_pnr -> m_driving_part ||
            m_bus_part != _path_pnr -> m_bus_part) {
            return false;
        }
        return true;
    }
};


/******************************************************************************************************************
*******************************************************************************************************************
										Passenger Path Set
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Passenger_Pathset::MNM_Passenger_Pathset(MMDue_mode mode) {
    m_mode = mode; // driving, transit, pnr, rh
    m_path_vec = std::vector<MNM_Passenger_Path_Base*>();
}

MNM_Passenger_Pathset::~MNM_Passenger_Pathset() {
    for (auto *_path : m_path_vec) {
        if (_path != nullptr) delete _path;
    }
    m_path_vec.clear();
}

bool MNM_Passenger_Pathset::is_in(MNM_Passenger_Path_Base* path) {
    if (path->m_mode_1 != (int) m_mode) return false;
    // https://stackoverflow.com/questions/92396/why-cant-variables-be-declared-in-a-switch-statement
    switch (m_mode) {
        case driving:
        {
            auto* _path = dynamic_cast<MNM_Passenger_Path_Driving*>(path);
            IAssert(_path != nullptr);
            for (auto *_path_it : m_path_vec) {
                if (_path->is_equal(_path_it)) {
                    return true;
                }
            }
            return false;
//            break;
        }
        case transit:
        {
            // TODO: metro
            auto* _path = dynamic_cast<MNM_Passenger_Path_Bus*>(path);
            IAssert(_path != nullptr);
            for (auto *_path_it : m_path_vec) {
                if (_path->is_equal(_path_it)) {
                    return true;
                }
            }
            return false;
//            break;
        }
        case pnr:
        {
            auto* _path = dynamic_cast<MNM_Passenger_Path_PnR*>(path);
            IAssert(_path != nullptr);
            for (auto *_path_it : m_path_vec) {
                if (_path->is_equal(_path_it)) {
                    return true;
                }
            }
            return false;
//            break;
        }
        case rh:
        {
            throw std::runtime_error("ride hailing passenger path not implemented\n");
            exit(-1);
        }
        default:
        {
            throw std::runtime_error("undefined passenger path!!\n");
            exit(-1);
        }
    }
}

namespace MNM {

Passenger_Path_Table *build_shortest_passenger_pathset(std::vector<MMDue_mode> *mode_vec,
                                                       MNM_MM_Due *mmdue, PNEGraph &graph,
                                                       MNM_OD_Factory *od_factory, MNM_Link_Factory *link_factory,
                                                       MNM_Busstop_Factory *busstop_factory, Bus_Path_Table *bus_path_table,
                                                       MNM_Walking_Link_Factory *walking_link_factory,
                                                       MNM_Parking_Lot_Factory *parking_lot_factory){
    // create an empty passenger path table
    auto *_path_table = new Passenger_Path_Table();
    for (auto _o_it = od_factory -> m_origin_map.begin(); _o_it != od_factory -> m_origin_map.end(); _o_it++){
        auto *_new_map_1 = new std::unordered_map<TInt, std::unordered_map<TInt, MNM_Passenger_Pathset*>*>();
        _path_table -> insert(std::pair<TInt, std::unordered_map<TInt, std::unordered_map<TInt, MNM_Passenger_Pathset*>*>*>(_o_it -> second -> m_origin_node -> m_node_ID, _new_map_1));
        for (auto _d_it = od_factory -> m_destination_map.begin(); _d_it != od_factory -> m_destination_map.end(); _d_it++){
            auto *_new_map_2 = new std::unordered_map<TInt, MNM_Passenger_Pathset*>();
            _new_map_1 -> insert(std::pair<TInt, std::unordered_map<TInt, MNM_Passenger_Pathset*>*>(_d_it -> second -> m_dest_node -> m_node_ID, _new_map_2));
            for (auto _mode_it : *mode_vec){
                auto _pathset = new MNM_Passenger_Pathset(_mode_it);
                _new_map_2 -> insert(std::pair<TInt, MNM_Passenger_Pathset*>(_mode_it, _pathset));
            }
        }
    }

    MNM_Passenger_Path_Driving *_p_path_driving;
    MNM_Passenger_Path_Bus *_p_path_bus;
    MNM_Parking_Lot* _parking_lot;
    std::vector<MNM_Walking_Link*> start_walking_links_vec = std::vector<MNM_Walking_Link*>();
    std::vector<MNM_Walking_Link*> end_walking_links_vec = std::vector<MNM_Walking_Link*>();
    TInt _dest_node_ID, _origin_node_ID;
    std::unordered_map<TInt, TFlt> _free_cost_map = std::unordered_map<TInt, TFlt>();
    std::unordered_map<TInt, TInt> _free_shortest_path_tree;
    MNM_Path *_path;
    for (auto _link_it : link_factory -> m_link_map){
        _free_cost_map.insert(std::pair<TInt, TFlt>(_link_it.first, _link_it.second -> get_link_tt() / mmdue->m_unit_time));
    }
    for (auto _d_it = od_factory -> m_destination_map.begin(); _d_it != od_factory -> m_destination_map.end(); _d_it++){
        _dest_node_ID = _d_it -> second -> m_dest_node -> m_node_ID;
        // destination parking lot
        for (auto _it : parking_lot_factory->m_parking_lot_map) {
            if (_it.second -> m_node_ID == _dest_node_ID) {
                _parking_lot = _it.second;
                break;
            }
        }
        // end walking links for bus
        if (!end_walking_links_vec.empty()) end_walking_links_vec.clear();
        for (auto _it : walking_link_factory->m_walking_link_map) {
            if (_it.second -> m_to_node_ID == _dest_node_ID && _it.second -> m_from_node_type == "bus_stop") {
                end_walking_links_vec.push_back(_it.second);
            }
        }
        MNM_Shortest_Path::all_to_one_FIFO(_dest_node_ID, graph, _free_cost_map, _free_shortest_path_tree);
        for (auto _o_it = od_factory -> m_origin_map.begin(); _o_it != od_factory -> m_origin_map.end(); _o_it++){
            _origin_node_ID = _o_it -> second -> m_origin_node -> m_node_ID;
            // start walking links for bus
            if (!start_walking_links_vec.empty()) start_walking_links_vec.clear();
            for (auto _it : walking_link_factory->m_walking_link_map) {
                if (_it.second -> m_from_node_ID == _origin_node_ID && _it.second -> m_to_node_type == "bus_stop") {
                    start_walking_links_vec.push_back(_it.second);
                }
            }

            // driving
            _path = MNM::extract_path(_origin_node_ID, _dest_node_ID, _free_shortest_path_tree, graph);
            if (_path != nullptr){

                if (_parking_lot -> m_node_ID != _path->m_node_vec.back()) {
                    _parking_lot = nullptr; // no destination parking lot
                }
                _p_path_driving = new MNM_Passenger_Path_Driving(driving, 0, mmdue->m_vot, mmdue->m_early_penalty,
                                                                 mmdue->m_late_penalty, mmdue->m_target_time,
                                                                 1, mmdue->m_carpool_cost_multiplier, 0.0,
                                                                 _parking_lot, nullptr,
                                                                 mmdue->m_parking_lot_to_destination_walking_time);
                // copy
                _p_path_driving -> m_link_vec = _path -> m_link_vec;
                _p_path_driving -> m_node_vec = _path -> m_node_vec;
                delete _path;
                printf("Adding to path table\n");
                std::cout << _p_path_driving -> node_vec_to_string();
                std::cout << _p_path_driving -> link_vec_to_string();
                _path_table -> find(_origin_node_ID) -> second -> find(_dest_node_ID) -> second ->
                               find(driving) -> second -> m_path_vec.push_back(_p_path_driving);
            }

            // bus
            TFlt tt_min = TFlt(std::numeric_limits<double>::max());
            TInt route_ID = -1;
            TInt start_busstop_ID = -1;
            TInt end_busstop_ID = -1;
            MNM_Walking_Link* start_walking_link = nullptr;
            MNM_Walking_Link* end_walking_link = nullptr;
            for (auto _s_link : start_walking_links_vec) {
                TInt _s_busstop_ID = _s_link -> m_to_node_ID;
                MNM_Busstop *_s_busstop = busstop_factory -> get_busstop(_s_busstop_ID);
                for (auto _e_link : end_walking_links_vec) {
                    TInt _e_busstop_ID = _s_link -> m_from_node_ID;
                    MNM_Busstop *_e_busstop = busstop_factory -> get_busstop(_e_busstop_ID);

                    for (auto _it : *bus_path_table) {
                        for (auto _it_it : *_it.second) {
                            for (auto _it_it_it : *_it_it.second) {
                                TInt _route_ID = _it_it_it.first;
                                MNM_BusPath *_bus_path = _it_it_it.second;
                                if (std::find(_bus_path->m_node_vec.begin(), _bus_path->m_node_vec.end(), _s_busstop_ID) != _bus_path->m_node_vec.end() &&
                                    std::find(_bus_path->m_node_vec.begin(), _bus_path->m_node_vec.end(), _e_busstop_ID) != _bus_path->m_node_vec.end()) {
                                    TFlt tt = _bus_path -> get_busroute_fftt(link_factory, _s_busstop, _e_busstop, mmdue->m_unit_time);
                                    if (tt < tt_min) {
                                        tt_min = tt;
                                        route_ID = _route_ID;
                                        start_busstop_ID = _s_busstop_ID;
                                        end_busstop_ID = _e_busstop_ID;
                                        start_walking_link = _s_link;
                                        end_walking_link = _e_link;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (route_ID == -1) {
                printf("No appropriate bus route available\n");
            } else {
                _p_path_bus = new MNM_Passenger_Path_Bus(transit, 0, mmdue->m_vot, mmdue->m_early_penalty,
                                                         mmdue->m_late_penalty, mmdue->m_target_time,
                                                         route_ID, start_busstop_ID, end_busstop_ID,
                                                         mmdue->m_bus_fare, mmdue->m_bus_inconvenience,
                                                         start_walking_link, end_walking_link);
                _path_table -> find(_origin_node_ID) -> second -> find(_dest_node_ID) -> second ->
                               find(transit) -> second -> m_path_vec.push_back(_p_path_bus);
            }

        }
    }
    return _path_table;

}

}

/******************************************************************************************************************
*******************************************************************************************************************
										Multimodal DUE
*******************************************************************************************************************
******************************************************************************************************************/
MNM_MM_Due::MNM_MM_Due(const std::string& file_folder) {
    m_file_folder = file_folder;

    m_mmdta_config = new MNM_ConfReader(m_file_folder + "/config.conf", "DTA");
    IAssert(m_mmdta_config->get_string("routing_type") == "Biclass_Bus_Hybrid");
    IAssert(m_mmdta_config->get_int("total_interval") < 0); // -1

    m_num_modes = m_mmdta_config->get_int("num_of_modes");
    m_unit_time = m_mmdta_config->get_float("unit_time"); // 5s
    m_total_loading_inter = m_mmdta_config->get_int("total_interval"); // -1
    m_total_assign_inter = m_mmdta_config->get_int("max_interval");

    m_mmdue_config = new MNM_ConfReader(m_file_folder + "/config.conf", "MMDUE");

    // money / hour -> money / interval
    m_vot = m_mmdue_config -> get_float("vot") / 3600. * m_unit_time;
    m_early_penalty = m_mmdue_config->get_float("early_penalty") / 3600. * m_unit_time;
    m_late_penalty = m_mmdue_config->get_float("late_penalty") / 3600. * m_unit_time;
    // minute -> interval
    m_target_time = m_mmdue_config->get_float("target_time") * 60 / 5;

    // second
    m_parking_lot_to_destination_walking_time = m_mmdue_config -> get_float("parking_lot_to_destination_walking_time");
    m_carpool_cost_multiplier = m_mmdue_config -> get_float("carpool_cost_multiplier");
    m_bus_fare = m_mmdue_config -> get_float("bus_fare");
    m_metro_fare = m_mmdue_config -> get_float("metro_fare");
    m_pnr_inconvenience = m_mmdue_config -> get_float("pnr_inconvenience");
    m_bus_inconvenience = m_mmdue_config -> get_float("bus_inconvenience");

    m_step_size = m_mmdue_config->get_float("step_size");  //0.01;  // MSA

    m_passenger_demand = std::unordered_map<TInt, std::unordered_map<TInt, TFlt*>> ();

    m_passenger_path_table = nullptr;
    // for car and truck driving
    m_path_table = nullptr;

    m_parkinglot_factory = nullptr;
    m_walkinglink_factory = nullptr;

    m_link_cost_map = std::unordered_map<TInt, TFlt *>();  // time-varying link cost

    m_mmdta = nullptr;
}

MNM_MM_Due::~MNM_MM_Due() {
    if (m_mmdta_config != nullptr) delete m_mmdta_config;
    if (m_mmdue_config != nullptr) delete m_mmdue_config;
    if (m_path_table != nullptr) delete m_path_table;
    if (m_parkinglot_factory != nullptr) delete m_parkinglot_factory;
    if (m_walkinglink_factory != nullptr) delete m_walkinglink_factory;

    for (auto _cost_it: m_link_cost_map) {
        delete _cost_it.second;
    }
    m_link_cost_map.clear();

    for (auto _it: m_passenger_demand) {
        for (auto _it_it : _it.second) {
            free(_it_it.second);
        }
        _it.second.clear();
    }
    m_passenger_demand.clear();

    delete m_mmdta;
}

int MNM_MM_Due::initialize() {
    // no existing path_table and path_table_buffer files
    m_mmdta = new MNM_Dta_Multimodal(m_file_folder);
    m_mmdta -> build_from_files();
    m_mmdta -> hook_up_node_and_link();
    m_mmdta -> is_ok();

    MNM_IO_Multimodal::build_passenger_demand(m_file_folder, m_mmdta_config, &m_passenger_demand, "passenger_demand");

    // for driving
    // create zero buffer, first half for car, last half for truck
    m_path_table = MNM::build_shortest_pathset(m_mmdta->m_graph,
                                               m_mmdta->m_od_factory, m_mmdta->m_link_factory);
    MNM::allocate_path_table_buffer(m_path_table, 2 * m_total_assign_inter);

    // for bus
//    auto* _routing = dynamic_cast<MNM_Routing_Biclass_Bus_Hybrid*>(m_mmdta -> m_routing);
//    _routing -> m_routing_fixed_bus -> m_bus_path_table;


    MNM_IO_Multimodal::build_parkinglot_factory(m_file_folder,
                                                m_mmdta_config,
                                                m_parkinglot_factory,
                                                m_mmdta->m_node_factory,
                                                m_mmdta->m_link_factory,
                                                m_mmdta_config->get_string("parking_lot_file_name"));
    MNM_IO_Multimodal::build_walkinglink_factory(m_file_folder,
                                                 m_mmdta_config,
                                                 m_walkinglink_factory,
                                                 m_mmdta_config->get_string("walking_link_file_name"));

//    m_total_loading_inter = m_mmdta -> loading(false);
    printf("finish initialization\n");
    return 0;
}

// spread the OD demand evenly over the initial paths for each mode
int MNM_MM_Due::init_passenger_path_flow() {

    TFlt _len, _dmd;
    TInt _o_node_ID, _d_node_ID;
    MNM_Origin_Multimodal *_org;
    MNM_Destination_Multimodal *_dest;
    for (auto _it : *m_path_table) {
        // printf("22m\n");
        _o_node_ID = _it.first;
        for (auto _it_it : *(_it.second)) {
            // printf("23m\n");
            _d_node_ID = _it_it.first;
            _len = TFlt(_it_it.second->m_path_vec.size());
            for (MNM_Path *_path : _it_it.second->m_path_vec) {
                // printf("24m\n");
                IAssert(_o_node_ID == _path->m_node_vec.front());
                IAssert(_d_node_ID == _path->m_node_vec.back());
                _org = dynamic_cast<MNM_Origin_Multimodal*>(((MNM_DMOND *) m_mmdta->m_node_factory->get_node(_o_node_ID))->m_origin);
                _dest = dynamic_cast<MNM_Destination_Multimodal*>(((MNM_DMDND *) m_mmdta->m_node_factory->get_node(_d_node_ID))->m_dest);
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

int MNM_MM_Due::passenger_demand_to_vehicle_demand() {
    return 0;
}
