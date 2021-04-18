//
// Created by qiling on 4/14/21.
//

#include "io.h"
#include "multimodal.h"
#include "Snap.h"

#include <vector>

int main()
{
    // print cwd
    char buffer[256];
    char *val = getcwd(buffer, sizeof(buffer));
    if (val) {
        std::cout << buffer << std::endl;
    }

    MNM_Dlink *_link;
    MNM_Dlink_Multiclass *_link_m;

    printf("BEGIN multimodal test!\n");

    // On ubuntu (PC)
    // std::string folder = "/home/alanpi/Desktop/MAC-POSTS/data/input_files_SPC_separate_Routing";
    // std::string folder = "/home/lemma/Documents/MAC-POSTS/src/examples/mcDODE/a6e7b31067d2ead8d3725fc0ed587d06c958f63c";
    std::string folder = "../../../data/input_files_7link_multimodal_due";

    // on macOS (Mac air)
    // std::string folder = "/Users/alan-air/Dropbox/MAC-POSTS/data/input_files_MckeesRocks_SPC";
    // std::string folder = "/media/lemma/WD/nhd/experiments/src/temp_input";

    MNM_ConfReader *config = new MNM_ConfReader(folder + "/config.conf", "STAT");
    std::string rec_folder = config -> get_string("rec_folder");


    MNM_Dta_Multimodal *test_dta = new MNM_Dta_Multimodal(folder);

    printf("================================ DTA set! =================================\n");

    test_dta -> build_from_files();
    printf("========================= Finished initialization! ========================\n");

    test_dta -> hook_up_node_and_link();
    printf("====================== Finished node and link hook-up! ====================\n");

    test_dta -> is_ok();
    printf("============================ DTA is OK to run! ============================\n");

    TInt _current_inter = 0;
    TInt _assign_inter = test_dta -> m_start_assign_interval;
    test_dta -> pre_loading();
    printf("========================== Finished pre_loading! ==========================\n");

    printf("\n\n\n====================================== Start loading! =======================================\n");
    bool _verbose = false;
    bool output_link_cong = true; // if true output link congestion level every cong_frequency
    TInt cong_frequency = 180; // 15 minutes
    bool output_veh_locs = true; // if true output veh location every vis_frequency
    TInt vis_frequency = 60; // 5 minutes
    bool output_busroute_tt = true; // if true output busroute tt every vis_frequency
    TInt bustt_frequency = 60; // 5 minutes

    MNM_Routing_Biclass_Bus_Hybrid* _routing;
    MNM_Veh_Multimodal* _veh;
    std::ofstream _vis_file;
    std::string _str;
    if (output_veh_locs){
        _vis_file.open(folder + "/" + rec_folder + "/veh_loc_raw.txt", std::ofstream::out);
        if (! _vis_file.is_open()){
            printf("Error happens when open _vis_file\n");
            exit(-1);
        }
    }

    while (!test_dta -> finished_loading(_current_inter) || _assign_inter < test_dta -> m_total_assign_inter){
        printf("\nCurrent loading interval: %d, Current assignment interval: %d\n", _current_inter(), _assign_inter());
        test_dta -> load_once(_verbose, _current_inter, _assign_inter);
        if (_current_inter % test_dta -> m_assign_freq == 0 || _current_inter == 0){
            _assign_inter += 1;
        }
        if (output_veh_locs && (_current_inter % vis_frequency == 0)){
            for (auto _map_it : test_dta -> m_veh_factory -> m_veh_map){
                if (_map_it.second -> m_finish_time < 0) {
                    _veh = dynamic_cast<MNM_Veh_Multimodal *>(_map_it.second);
                    if (_veh -> m_class == 0){
                        _str = "Car ";
                    }
                    else {
                        _str = "Truck ";
                    }
                    _str += "timestamp (intervals): " + std::to_string(_current_inter) + " ";
                    _str += "link_ID: " + std::to_string(_veh -> get_current_link() -> m_link_ID) + " ";
                    _str += "visual_position: " + std::to_string(_veh -> m_visual_position_on_link);
                    _str += "\n";
                    _vis_file << _str;
                }
            }
        }
        _current_inter += 1;
        // if (_current_inter > 200) break;
    }

    // Output total travels and travel time, before divided by flow_scalar
    TInt _count_car = 0, _count_truck = 0;
    TFlt _tot_tt = 0.0;
    for (auto _map_it : test_dta -> m_veh_factory -> m_veh_map){
        if (_map_it.second -> m_finish_time > 0) {
            _veh = dynamic_cast<MNM_Veh_Multimodal *>(_map_it.second);
            if (_veh -> m_class == 0){
                _count_car += 1;
            }
            else {
                _count_truck += 1;
            }
            _tot_tt += (_veh -> m_finish_time - _veh -> m_start_time) * test_dta -> m_unit_time / 3600.0;
        }
    }
    printf("\n\n\nTotal car: %d, Total truck: %d, Total tt: %.2f hours\n\n\n\n", int(_count_car), int(_count_truck), float(_tot_tt));



    if (output_veh_locs){
        if (_vis_file.is_open()) _vis_file.close();
    }


    std::ofstream _vis_file2;
    if (output_link_cong){
        _vis_file2.open(folder + "/" + rec_folder + "/link_cong_raw.txt", std::ofstream::out);
        if (! _vis_file2.is_open()){
            printf("Error happens when open _vis_file2\n");
            exit(-1);
        }
        TInt _iter = 0;
        while (_iter < _current_inter){
            if (_iter % cong_frequency == 0 || _iter == _current_inter - 1){
                printf("Current loading interval: %d\n", int(_iter));
                for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
                    _link = _link_it -> second;
                    _link_m = dynamic_cast<MNM_Dlink_Multiclass*>(_link);
                    _str = "\ntimestamp (intervals): " + std::to_string(int(_iter)) + " ";
                    _str += "link_ID: " + std::to_string(_link -> m_link_ID()) + " ";
                    _str += "car_inflow: " + std::to_string(MNM_DTA_GRADIENT::get_link_inflow_car(_link_m, _iter, _iter + 1)) + " ";
                    _str += "truck_inflow: " + std::to_string(MNM_DTA_GRADIENT::get_link_inflow_truck(_link_m, _iter, _iter + 1)) + " ";
                    _str += "car_tt (s): " + std::to_string(MNM_DTA_GRADIENT::get_travel_time_car(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time) * test_dta -> m_unit_time) + " ";
                    _str += "truck_tt (s): " + std::to_string(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time) * test_dta -> m_unit_time) + " ";
                    _str += "car_fftt (s): " + std::to_string(_link_m -> get_link_freeflow_tt_car()) + " ";
                    _str += "truck_fftt (s): " + std::to_string(_link_m -> get_link_freeflow_tt_truck()) + " ";
                    _str += "car_speed (mph): " + std::to_string(_link_m -> m_length/(MNM_DTA_GRADIENT::get_travel_time_car(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time) * test_dta -> m_unit_time) * 3600 / 1600) + " ";
                    _str += "truck_speed (mph): " + std::to_string(_link_m -> m_length/(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time) * test_dta -> m_unit_time) * 3600 / 1600) + "\n";
                    _vis_file2 << _str;
                }
            }
            _iter += 1;
        }
        if (_vis_file2.is_open()) _vis_file2.close();
    }

    // // output tt of some special links
    // for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
    // 		_link = _link_it -> second;
    // 		if (_link -> m_link_ID() == 7186) {
    // 			TInt _iter = 0;
    // 			while (_iter < _current_inter){
    // 				// if (_iter == 984){
    // 					_link_m = dynamic_cast<MNM_Dlink_Multiclass*>(_link);
    // 					printf("%d,%.2f,%.2f\n", int(_iter),
    // 						double(MNM_DTA_GRADIENT::get_travel_time_car(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time)),
    // 						double(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time)));
    // 				// }
    // 				_iter += 1;
    // 			}
    // 		}
    // }

    // output CC of some special links
    std::cout << "\n\n **************************** link cc ****************************" << std::endl;
    for (auto _link_it = test_dta->m_link_factory->m_link_map.begin();
         _link_it != test_dta->m_link_factory->m_link_map.end(); _link_it++) {
        _link = _link_it->second;
        if (_link->m_link_ID() == 1) {
            _link_m = dynamic_cast<MNM_Dlink_Multiclass *>(_link);
            std::cout << "\nlink_ID: " << _link->m_link_ID << std::endl;
            printf("m_N_in_car: \n");
            std::cout << _link_m->m_N_in_car->to_string() << std::endl;
            printf("m_N_out_car: \n");
            std::cout << _link_m->m_N_out_car->to_string() << std::endl;
            printf("m_N_in_truck: \n");
            std::cout << _link_m->m_N_in_truck->to_string() << std::endl;
            printf("m_N_out_truck: \n");
            std::cout << _link_m->m_N_out_truck->to_string() << std::endl;
        }
    }

    std::ofstream _vis_file3;
    if (output_busroute_tt){
        _vis_file3.open(folder + "/" + rec_folder + "/busroute_tt_raw.txt", std::ofstream::out);
        if (! _vis_file3.is_open()){
            printf("Error happens when open _vis_file3\n");
            exit(-1);
        }
        _routing = dynamic_cast<MNM_Routing_Biclass_Bus_Hybrid*>(test_dta -> m_routing);
        TInt _iter = 0;
        while (_iter < _current_inter){
            if (_iter % bustt_frequency == 0 || _iter == _current_inter - 1){
                printf("\nCurrent loading interval: %d\n", int(_iter));

                // output CC of some busstops
                if (_iter == _current_inter - 1) {
                    std::cout << "\n\n **************************** busstop cc ****************************" << std::endl;
                    for (auto _busstop_it : test_dta -> m_busstop_factory -> m_busstop_map){
                        auto* _busstop = _busstop_it.second;
                        if ((_busstop -> m_busstop_ID() == 101) || (_busstop -> m_busstop_ID() == 106) || (_busstop -> m_busstop_ID() == 4)){
                            for (auto _route_ID : _busstop -> m_routeID_vec) {
                                std::cout << "\nbusstop ID: " << _busstop -> m_busstop_ID << std::endl;
                                std::cout << "route ID: " << _route_ID << std::endl;
                                printf("m_N_in_bus: \n");
                                std::cout <<_busstop -> m_N_in_bus.find(_route_ID) -> second -> to_string() << std::endl;
                                printf("m_N_out_bus: \n");
                                std::cout <<_busstop -> m_N_out_bus.find(_route_ID) -> second -> to_string() << std::endl;
                            }
                        }
                    }
                }

                for (auto _it : *(_routing -> m_routing_fixed_bus -> m_bus_path_table)) {
                    for (auto _it_it : *(_it.second)) {
                        for (auto _it_it_it : *(_it_it.second)) {
                            _str = "\ntimestamp (intervals) " + std::to_string(int(_iter)) + " ";
                            _str += "Origin: " + std::to_string(_it.first) + " ";
                            _str += "Destination: " + std::to_string(_it_it.first) + " ";
                            _str += "route ID: " + std::to_string(_it_it_it.first) + " ";
                            _str += "travel time (intervals): " +
                                    std::to_string(_it_it_it.second -> get_whole_busroute_tt(TFlt(_iter), test_dta -> m_link_factory,
                                                                                             test_dta -> m_busstop_factory, test_dta -> m_unit_time)) +
                                    " ";
                            _vis_file3 << _str;
                        }
                    }
                }
            }
            _iter += 1;
        }
        if (_vis_file3.is_open()) _vis_file3.close();
    }

    delete config;
    delete test_dta;
    printf("\n\nFinished delete test_dta!\n");

    return 0;
}
