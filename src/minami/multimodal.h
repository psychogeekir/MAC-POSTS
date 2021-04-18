//
// Created by qiling on 2/18/21.
//

#ifndef MINAMI_MULTIMODAL_H
#define MINAMI_MULTIMODAL_H

#include "multiclass.h"
#include "path.h"
#include "due.h"

class MNM_Parking_Lot_Factory;
class MNM_Walking_Link_Factory;

/******************************************************************************************************************
*******************************************************************************************************************
												Bus Stop Models
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Busstop
{
public:
    MNM_Busstop(TInt ID,
                TInt linkID,
                TFlt linkloc,
                std::vector<TInt>* routID_vec);

    ~MNM_Busstop();
    int install_cumulative_curve_multiclass();
    TFlt get_bus_waiting_time(TFlt time, TInt routeID);

    TInt m_busstop_ID;
    TInt m_link_ID;
    TFlt m_link_loc;
    TInt m_cell_ID;
    std::vector<TInt> m_routeID_vec;

    std::unordered_map<TInt, TInt> m_passed_bus_counter;

    // separate N-curves for different bus routes, <route ID, cc curve>
    std::unordered_map<TInt, MNM_Cumulative_Curve*> m_N_in_bus;
    std::unordered_map<TInt, MNM_Cumulative_Curve*> m_N_out_bus;

//    MNM_Cumulative_Curve *m_N_in_car;
//    MNM_Cumulative_Curve *m_N_out_car;
//    MNM_Cumulative_Curve *m_N_in_truck;
//    MNM_Cumulative_Curve *m_N_out_truck;

};

/******************************************************************************************************************
*******************************************************************************************************************
												Bus Stop Factory
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Busstop_Factory
{
public:
    MNM_Busstop_Factory();
    virtual ~MNM_Busstop_Factory();
    MNM_Busstop *make_busstop(TInt ID, TInt linkID, TFlt linkloc, std::vector<TInt>* routID_vec);
    MNM_Busstop *get_busstop(TInt ID);

    std::unordered_map<TInt, MNM_Busstop*> m_busstop_map;
};


/******************************************************************************************************************
*******************************************************************************************************************
											  Multimodal Vehicle
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Veh_Multimodal : public MNM_Veh_Multiclass
{
public:
    MNM_Veh_Multimodal(TInt ID, TInt vehicle_class, TInt start_time, TInt bus_route_ID = TInt(-1));
    virtual ~MNM_Veh_Multimodal() override;

    virtual TInt get_class() override {return m_class;};  // virtual getter
    virtual TInt get_bus_route_ID() override {return m_bus_route_ID;};  // virtual getter

    TInt m_bus_route_ID;
    TInt m_stopped_intervals;
};


/******************************************************************************************************************
*******************************************************************************************************************
												Vehicle Factory
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Veh_Factory_Multimodal : public MNM_Veh_Factory_Multiclass
{
public:
    MNM_Veh_Factory_Multimodal();
    virtual ~MNM_Veh_Factory_Multimodal() override;

    MNM_Veh_Multimodal* make_veh_multimodal(TInt timestamp, Vehicle_type veh_type, TInt vehicle_cls, TInt bus_route=TInt(-1));
};

/******************************************************************************************************************
*******************************************************************************************************************
												Multimodal OD
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Destination_Multimodal : public MNM_Destination_Multiclass
{
public:
    explicit MNM_Destination_Multimodal(TInt ID);
    virtual ~MNM_Destination_Multimodal() override;
};

class MNM_Origin_Multimodal : public MNM_Origin_Multiclass
{
public:
    MNM_Origin_Multimodal(TInt ID, TInt max_interval, TFlt flow_scalar, TInt frequency);
    virtual ~MNM_Origin_Multimodal() override;

    virtual int release_one_interval(TInt current_interval,
                                     MNM_Veh_Factory* veh_factory,
                                     TInt assign_interval,
                                     TFlt adaptive_ratio) override;

    virtual int release_one_interval_biclass(TInt current_interval,
                                             MNM_Veh_Factory* veh_factory,
                                             TInt assign_interval,
                                             TFlt adaptive_ratio_car,
                                             TFlt adaptive_ratio_truck) override;

    int add_dest_demand_bus(MNM_Destination_Multimodal *dest,
                            TInt routeID,
                            TFlt* demand_bus);

    // <Destination node, <routeID, time-varying demand>>
    std::unordered_map<MNM_Destination_Multimodal*, std::unordered_map<TInt, TFlt*>> m_demand_bus;
};

/******************************************************************************************************************
*******************************************************************************************************************
												OD Factory
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_OD_Factory_Multimodal : public MNM_OD_Factory_Multiclass
{
public:
    MNM_OD_Factory_Multimodal();
    virtual ~MNM_OD_Factory_Multimodal() override;
    virtual MNM_Destination_Multimodal* make_destination(TInt ID) override;
    virtual MNM_Origin_Multimodal* make_origin(TInt ID,
                                               TInt max_interval,
                                               TFlt flow_scalar,
                                               TInt frequency) override;

};

/******************************************************************************************************************
*******************************************************************************************************************
												Node Factory
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Node_Factory_Multimodal : public MNM_Node_Factory_Multiclass
{
public:
    MNM_Node_Factory_Multimodal();
    virtual ~MNM_Node_Factory_Multimodal() override;

    // use this one instead of make_node in the base class
    MNM_Dnode *make_node_multimodal(TInt ID, DNode_type_multimodal node_type, TFlt flow_scalar, TFlt veh_convert_factor);
};

/******************************************************************************************************************
*******************************************************************************************************************
												Link Models
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
					Multimodal Point-Queue Model
**************************************************************************/
class MNM_Dlink_Pq_Multimodal : public MNM_Dlink_Pq_Multiclass
{
public:
    MNM_Dlink_Pq_Multimodal(TInt ID,
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
                            TFlt flow_scalar);
    virtual ~MNM_Dlink_Pq_Multimodal() override;
    virtual int clear_incoming_array(TInt timestamp) override;
    virtual int evolve(TInt timestamp) override;

    // for multimodal with busstops on the link
    DLink_type_multimodal m_link_type;
    std::vector<MNM_Busstop*> m_busstop_vec;
};


/**************************************************************************
					Multimodal CTM Model
**************************************************************************/
class MNM_Dlink_Ctm_Multimodal : public MNM_Dlink_Ctm_Multiclass
{
public:
    MNM_Dlink_Ctm_Multimodal(TInt ID,
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
                             TFlt flow_scalar);
    virtual ~MNM_Dlink_Ctm_Multimodal() override;
    virtual int clear_incoming_array(TInt timestamp) override;
    virtual int evolve(TInt timestamp) override;

    int move_veh_queue_in_cell(std::deque<MNM_Veh*> *from_queue,
                               std::deque<MNM_Veh*> *to_queue,
                               TInt number_tomove, TInt timestamp, TInt cell_ID);

    int move_veh_queue_in_last_cell(TInt timestamp);

    DLink_type_multimodal m_link_type;
    // <cell_ID, busstop_vec>
    std::unordered_map<TInt, std::vector<MNM_Busstop*>> m_cell_busstop_vec;
    // for multimodal with busstops on the link
    std::vector<MNM_Busstop*> m_busstop_vec;
};

/******************************************************************************************************************
*******************************************************************************************************************
											Link Factory
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Link_Factory_Multimodal : public MNM_Link_Factory_Multiclass
{
public:
    MNM_Link_Factory_Multimodal();
    virtual ~MNM_Link_Factory_Multimodal() override;

    // use this one instead of make_link in the base class
    MNM_Dlink *make_link_multimodal(TInt ID,
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
                                    TFlt flow_scalar);
};

/******************************************************************************************************************
*******************************************************************************************************************
												Bus Path
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_BusPath : public MNM_Path
{
public:
    MNM_BusPath(TInt route_ID);
    virtual ~MNM_BusPath() override;

    std::deque<TInt> m_busstop_vec;
    TInt m_route_ID;

    TFlt get_busroute_fftt(MNM_Link_Factory *link_factory, MNM_Busstop* start_busstop, MNM_Busstop* end_busstop, TFlt unit_interval);
    TFlt get_busroute_tt(TFlt start_time, MNM_Link_Factory *link_factory,
                         MNM_Busstop* start_busstop, MNM_Busstop* end_busstop, TFlt unit_interval);
    TFlt get_whole_busroute_tt(TFlt start_time, MNM_Link_Factory *link_factory,
                               MNM_Busstop_Factory* busstop_factory, TFlt unit_interval);
};

/******************************************************************************************************************
*******************************************************************************************************************
												Bus Path Table
*******************************************************************************************************************
******************************************************************************************************************/
// <O, <D, <routeID, Path>>>
typedef std::unordered_map<TInt, std::unordered_map<TInt, std::unordered_map<TInt, MNM_BusPath*>*>*> Bus_Path_Table;

/******************************************************************************************************************
*******************************************************************************************************************
												Routing
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                          Bus Fixed routing
**************************************************************************/

class MNM_Routing_Bus : public MNM_Routing_Biclass_Fixed
{
public:
    MNM_Routing_Bus(PNEGraph &graph,
                    MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
                    MNM_Link_Factory *link_factory, Bus_Path_Table *bus_path_table, TInt route_frq = TInt(-1),
                    TInt buffer_length=TInt(-1), TInt veh_class = TInt(1));
    virtual ~MNM_Routing_Bus() override;
    virtual int register_veh(MNM_Veh* veh) override;
    virtual int init_routing(Path_Table *path_table=NULL) override;
    virtual int update_routing(TInt timestamp) override;

    Bus_Path_Table *m_bus_path_table;
};

/**************************************************************************
                          Biclass_Bus_Hybrid routing
**************************************************************************/
class MNM_Routing_Biclass_Bus_Hybrid : public MNM_Routing_Biclass_Hybrid
{
public:
    MNM_Routing_Biclass_Bus_Hybrid(const std::string& file_folder, PNEGraph &graph, MNM_Statistics* statistics, MNM_OD_Factory *od_factory,
                                   MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory, Bus_Path_Table *bus_path_table,
                                   TInt route_frq_fixed = TInt(-1), TInt buffer_length = TInt(-1));
    virtual ~MNM_Routing_Biclass_Bus_Hybrid() override;
    virtual int init_routing(Path_Table *path_table=NULL) override;
    virtual int update_routing(TInt timestamp) override;

    MNM_Routing_Bus *m_routing_fixed_bus;
};

/******************************************************************************************************************
*******************************************************************************************************************
												Multimodal IO Functions
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_IO_Multimodal : public MNM_IO_Multiclass
{
public:

    // static int build_od_factory_multimodal(std::string file_folder,
    // 										MNM_ConfReader *conf_reader,
    // 										MNM_OD_Factory *od_factory,
    // 										MNM_Node_Factory *node_factory) {
    // 	return build_od_factory(file_folder, conf_reader, od_factory, node_factory);
    // };
    static int build_node_factory_multimodal(const std::string& file_folder,
                                             MNM_ConfReader *conf_reader,
                                             MNM_Node_Factory *node_factory,
                                             const std::string& file_name = "MNM_input_node");
    static int build_link_factory_multimodal(const std::string& file_folder,
                                             MNM_ConfReader *conf_reader,
                                             MNM_Link_Factory *link_factory,
                                             const std::string& file_name = "MNM_input_link");
    static int build_busstop_factory(const std::string& file_folder,
                                     MNM_ConfReader *conf_reader,
                                     MNM_Busstop_Factory *busstop_factory,
                                     MNM_Link_Factory *link_factory,
                                     const std::string& file_name = "bus_stops");
    static int build_parkinglot_factory(const std::string& file_folder,
                                        MNM_ConfReader *conf_reader,
                                        MNM_Parking_Lot_Factory *parkinglot_factory,
                                        MNM_Node_Factory *node_factory,
                                        MNM_Link_Factory *link_factory,
                                        const std::string& file_name = "parking_lots");
    static int build_walkinglink_factory(const std::string& file_folder,
                                        MNM_ConfReader *conf_reader,
                                        MNM_Walking_Link_Factory *walkinglink_factory,
                                        const std::string& file_name = "walking_links");
    static int build_demand_multimodal(const std::string& file_folder,
                                       MNM_ConfReader *conf_reader,
                                       MNM_OD_Factory *od_factory,
                                       const std::string& file_name_driving = "MNM_input_demand",
                                       const std::string& file_name_bus = "bus_demand");
    static int build_passenger_demand(const std::string& file_folder,
                                      MNM_ConfReader *conf_reader,
                                      std::unordered_map<TInt, std::unordered_map<TInt, TFlt*>> *passenger_demand,
                                      const std::string& file_name = "passenger_demand");
    static Bus_Path_Table *load_bus_path_table(const PNEGraph& graph, TInt num_path,
                                               const std::string& file_name="bus_path_table",
                                               const std::string& route_file_name="bus_routes");
};


/******************************************************************************************************************
*******************************************************************************************************************
												Multimodal DTA
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Dta_Multimodal : public MNM_Dta_Multiclass
{
public:
    explicit MNM_Dta_Multimodal(const std::string& file_folder);
    virtual ~MNM_Dta_Multimodal() override;
    virtual int initialize() override;
    virtual int set_routing() override;
    virtual int build_from_files() override;
    // virtual int pre_loading() override;
    virtual int load_once(bool verbose, TInt load_int, TInt assign_int) override;
    virtual int loading(bool verbose) override;

//    MNM_Veh_Factory_Multimodal *m_veh_factory;
//    MNM_Node_Factory_Multimodal *m_node_factory;
//    MNM_Link_Factory_Multimodal *m_link_factory;
//    MNM_OD_Factory_Multimodal *m_od_factory;
    MNM_Busstop_Factory *m_busstop_factory;
};


/******************************************************************************************************************
*******************************************************************************************************************
												Parking lot
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Parking_Lot
{
public:
    MNM_Parking_Lot(TInt ID, TInt node_ID, std::vector<TInt>* in_linkID_vec,
                    TFlt base_price, TFlt price_surge_coeff, TFlt avg_parking_time, TFlt capacity);
    virtual ~MNM_Parking_Lot();

    TInt m_ID;
    TFlt m_base_price;
    TFlt m_price_surge_coeff;
    TFlt m_avg_parking_time;
    TFlt m_max_parking_time;
    TFlt m_capacity;
    std::vector<TInt> m_in_linkID_vec;
    std::vector<MNM_Dlink*> m_in_link_vec;
    TInt m_node_ID;
    MNM_Dnode *m_node;

    TFlt get_cruise_time(TFlt timestamp, MNM_Dta_Multimodal *mmdta); // interval
};


/******************************************************************************************************************
*******************************************************************************************************************
												Parking lot factory
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Parking_Lot_Factory
{
public:
    MNM_Parking_Lot_Factory();
    virtual ~MNM_Parking_Lot_Factory();
    MNM_Parking_Lot *make_parking_lot(TInt ID, TInt node_ID, std::vector<TInt>* in_linkID_vec,
                                      TFlt base_price, TFlt price_surge_coeff, TFlt avg_parking_time, TFlt capacity);
    MNM_Parking_Lot *get_parking_lot(TInt ID);
    std::unordered_map<TInt, MNM_Parking_Lot*> m_parking_lot_map;
};


/******************************************************************************************************************
*******************************************************************************************************************
												Walking Link
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Walking_Link
{
public:
    MNM_Walking_Link(TInt ID, const std::string& from_node_type, const std::string& to_node_type,
                     TInt from_node_ID, TInt to_node_ID, TFlt walking_time);
    virtual ~MNM_Walking_Link();

    TInt m_link_ID;
    // possible node-node pair: origin -> bus_stop, parking_lot -> bus_stop, bus_stop -> destination, parking_lot -> destination
    std::string m_from_node_type;
    std::string m_to_node_type;
    TInt m_from_node_ID;
    TInt m_to_node_ID;
    TFlt m_walking_time; //second
};


/******************************************************************************************************************
*******************************************************************************************************************
												Walking Link Factory
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Walking_Link_Factory
{
public:
    MNM_Walking_Link_Factory();
    virtual ~MNM_Walking_Link_Factory();

    MNM_Walking_Link *make_walking_link(TInt ID, const std::string& from_node_type, const std::string& to_node_type,
                                        TInt from_node_ID, TInt to_node_ID, TFlt walking_time);
    MNM_Walking_Link *get_walking_link(TInt ID);
    std::unordered_map<TInt, MNM_Walking_Link*> m_walking_link_map;
};

/******************************************************************************************************************
*******************************************************************************************************************
												Passenger path
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
					           Base
**************************************************************************/
class MNM_Passenger_Path_Base : public MNM_Path
{
public:
    MNM_Passenger_Path_Base(int mode_1, int mode_2, TFlt vot, TFlt early_penalty, TFlt late_penalty, TFlt target_time);
    ~MNM_Passenger_Path_Base();

    int m_mode_1;
    int m_mode_2;
    TFlt m_vot; // money / interval
    TFlt m_early_penalty; // money / interval
    TFlt m_late_penalty; // money / interval
    TFlt m_target_time; // intervals

    virtual TFlt get_travel_time(TFlt start_time, MNM_Dta_Multimodal *mmdta) {return TFlt(-1.0);};  // interval
    virtual TFlt get_travel_cost(TFlt start_time, MNM_Dta_Multimodal *mmdta) {return TFlt(-1.0);};
    TFlt get_wrongtime_penalty(TFlt arrival_time);

    virtual bool is_equal(MNM_Passenger_Path_Base* path) {return false;};
};


/**************************************************************************
					           Driving
**************************************************************************/
class MNM_Passenger_Path_Driving : public MNM_Passenger_Path_Base
{
public:
    MNM_Passenger_Path_Driving(int mode_1, int mode_2, TFlt vot, TFlt early_penalty, TFlt late_penalty, TFlt target_time,
                               TInt num_people, TFlt carpool_cost_multiplier, TFlt walking_time_before_driving,
                               MNM_Parking_Lot* parking_lot, MNM_Walking_Link *walking_link_after_driving);
    ~MNM_Passenger_Path_Driving();

    TInt m_num_people;
    TFlt m_carpool_cost_multiplier;
    TFlt m_walking_time_before_driving;  // seconds
    TFlt m_walking_time_after_driving;  // seconds

    MNM_Parking_Lot *m_parking_lot;
    MNM_Walking_Link *m_walking_link_after_driving;

    virtual TFlt get_travel_time(TFlt start_time, MNM_Dta_Multimodal *mmdta) override;  // interval
    virtual TFlt get_travel_cost(TFlt start_time, MNM_Dta_Multimodal *mmdta) override;
    TFlt get_carpool_cost();
    TFlt get_amortized_parkingfee();

    virtual bool is_equal(MNM_Passenger_Path_Base* path) override;
};


/**************************************************************************
					           Bus
**************************************************************************/
class MNM_Passenger_Path_Bus : public MNM_Passenger_Path_Base
{
public:
    MNM_Passenger_Path_Bus(int mode_1, int mode_2, TFlt vot, TFlt early_penalty, TFlt late_penalty, TFlt target_time,
                           TInt route_ID, TInt start_busstop_ID, TInt end_busstop_ID, TFlt bus_fare, TFlt bus_inconvenience,
                           MNM_Walking_Link *walking_link_before_bus, MNM_Walking_Link *walking_link_after_bus);
    virtual ~MNM_Passenger_Path_Bus();

    TInt m_route_ID;
    TInt m_start_busstop_ID;
    TInt m_end_busstop_ID;
    TFlt m_bus_fare;
    TFlt m_bus_inconvenience;
    TFlt m_walking_time_before_bus;  // seconds
    TFlt m_walking_time_after_bus;  // seconds
    MNM_Walking_Link *m_walking_link_before_bus;
    MNM_Walking_Link *m_walking_link_after_bus;
    // TFlt m_waiting_time;

    virtual TFlt get_travel_time(TFlt start_time, MNM_Dta_Multimodal *mmdta) override;  // intervals
    virtual TFlt get_travel_cost(TFlt start_time, MNM_Dta_Multimodal *mmdta) override;

    virtual bool is_equal(MNM_Passenger_Path_Base* path) override;
};


/**************************************************************************
					           Metro
**************************************************************************/
class MNM_Passenger_Path_Metro : public MNM_Passenger_Path_Base
{
public:
    MNM_Passenger_Path_Metro(int mode_1, int mode_2, TFlt vot, TFlt early_penalty, TFlt late_penalty, TFlt target_time,
                             TInt route_ID, TInt start_metrostop_ID, TInt end_metrostop_ID,
                             TFlt metro_fare, TFlt metro_inconvenience, TFlt metro_time, TFlt waiting_time,
                             MNM_Walking_Link *walking_link_before_metro, MNM_Walking_Link *walking_link_after_metro);
    virtual ~MNM_Passenger_Path_Metro();

    TInt m_route_ID;
    TInt m_start_metrostop_ID;
    TInt m_end_metrostop_ID;
    TFlt m_metro_fare;
    TFlt m_metro_inconvenience;
    TFlt m_metro_time;  // intervals, from metro network (TODO)
    TFlt m_walking_time_before_metro;  // seconds
    TFlt m_walking_time_after_metro;  // second
    TFlt m_waiting_time;  // second

    MNM_Walking_Link *m_walking_link_before_metro;
    MNM_Walking_Link *m_walking_link_after_metro;

    virtual TFlt get_travel_time(TFlt start_time, MNM_Dta_Multimodal *mmdta) override;  // interval
    virtual TFlt get_travel_cost(TFlt start_time, MNM_Dta_Multimodal *mmdta) override;

    virtual bool is_equal(MNM_Passenger_Path_Base* path) override;
};


/**************************************************************************
					           Park & Ride
**************************************************************************/
class MNM_Passenger_Path_PnR : public MNM_Passenger_Path_Base
{
public:
    MNM_Passenger_Path_PnR(int mode_1, int mode_2, TFlt vot, TFlt early_penalty, TFlt late_penalty, TFlt target_time,
                           MNM_Passenger_Path_Driving *driving_part,
                           MNM_Passenger_Path_Bus *bus_part,
                           TFlt pnr_inconvenience);
    virtual ~MNM_Passenger_Path_PnR();

    MNM_Passenger_Path_Driving *m_driving_part;
    MNM_Passenger_Path_Bus *m_bus_part;
    TFlt m_pnr_inconvenience;

    virtual TFlt get_travel_time(TFlt start_time, MNM_Dta_Multimodal *mmdta) override;
    virtual TFlt get_travel_cost(TFlt start_time, MNM_Dta_Multimodal *mmdta) override;

    virtual bool is_equal(MNM_Passenger_Path_Base* path) override;
};


/******************************************************************************************************************
*******************************************************************************************************************
										Passenger Path Set
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Passenger_Pathset
{
public:
    MNM_Passenger_Pathset(MMDue_mode mode);
    ~MNM_Passenger_Pathset();
    MMDue_mode m_mode; // driving, transit, pnr, rh
    std::vector<MNM_Passenger_Path_Base*> m_path_vec;
    bool is_in(MNM_Passenger_Path_Base* path);
};

/******************************************************************************************************************
*******************************************************************************************************************
										Multimodal DUE
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_MM_Due {
public:
    MNM_MM_Due(const std::string& file_folder);

    ~MNM_MM_Due();

    virtual int initialize();

    MNM_Dta_Multimodal *run_mmdta(bool verbose);

    virtual int init_path_flow();

    virtual int update_path_table(MNM_Dta_Multiclass *mcdta, int iter) { return 0; };

    virtual int update_path_table_fixed_departure_time_choice(MNM_Dta_Multiclass *mcdta, int iter) { return 0; };

    virtual int update_path_table_gp_fixed_departure_time_choice(MNM_Dta_Multiclass *mcdta, int iter) { return 0;};

    TFlt compute_merit_function();

    TFlt compute_merit_function_fixed_departure_time_choice();

    TFlt get_disutility(TFlt depart_time, TFlt tt);

    TFlt get_tt(TFlt depart_time, MNM_Path *path);

    int build_cost_map(MNM_Dta_Multiclass *mcdta);

    int update_demand_from_path_table(MNM_Dta_Multimodal *mmdta);

    TFlt compute_total_demand(MNM_Origin *orig, MNM_Destination *dest, TInt total_assign_inter);

    std::string m_file_folder;
    TInt m_num_modes;
    TFlt m_unit_time;
    TInt m_total_loading_inter;

    // <O_node_ID, <D_node_ID, time-varying demand with length of m_total_assign_inter>>
    std::unordered_map<TInt, std::unordered_map<TInt, TFlt*>> m_passenger_demand;
    // <O_node_ID, <D_node_ID, <mode_ID, passenger_pathset>>>
    std::unordered_map<> m_passenger_path_table;
    Path_Table *m_path_table;
    TInt m_total_assign_inter;

    MNM_ConfReader *m_mmdta_config;
    MNM_ConfReader *m_mmdue_config;

    MNM_Parking_Lot_Factory *m_parkinglot_factory;
    MNM_Walking_Link_Factory *m_walkinglink_factory;

    TFlt m_vot;
    TFlt m_early_penalty;
    TFlt m_late_penalty;
    TFlt m_target_time;
    TFlt m_step_size;

    MNM_Dta_Multimodal *m_mmdta;

    // single_level <mode, <passenger path ID, cost>>

    // time-varying link cost
    std::unordered_map<TInt, TFlt *> m_link_cost_map;

};

#endif //MINAMI_MULTIMODAL_H
