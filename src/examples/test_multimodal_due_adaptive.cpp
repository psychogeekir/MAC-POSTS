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

    printf("BEGIN multimodal DUE test!\n");

    // std::string folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_due_columngeneration";
    std::string folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_due_fixedpath";

    // std::string folder = "/srv/data/qiling/Projects/CentralOhio_Honda_Project/Multimodal/scenarios/mobility_service/input_files_CentralOhio_mmdue_AM";
    // std::string folder = "/home/qiling/Documents/CentralOhio_Honda_Project/Multimodal/scenarios/no_mobility_service/input_files_CentralOhio_mmdue_AM";

    // std::string folder = "/home/qiling/Documents/CentralOhio_Honda_Project/Multimodal/scenarios/mobility_service/input_files_CentralOhio_mmdue_AM_1.1";
    // std::string folder = "/home/qiling/Documents/CentralOhio_Honda_Project/Multimodal/scenarios/no_mobility_service/input_files_CentralOhio_mmdue_AM_1.1";

    // std::string folder = "/home/qiling/Documents/CentralOhio_Honda_Project/Multimodal/scenarios/mobility_service/input_files_CentralOhio_mmdue_AM_1.2";
    // std::string folder = "/home/qiling/Documents/CentralOhio_Honda_Project/Multimodal/scenarios/no_mobility_service/input_files_CentralOhio_mmdue_AM_1.2";

    // std::string folder = "/home/qiling/Documents/CentralOhio_Honda_Project/Multimodal/scenarios/mobility_service/input_files_CentralOhio_mmdue_AM_1.5";
    // std::string folder = "/home/qiling/Documents/CentralOhio_Honda_Project/Multimodal/scenarios/no_mobility_service/input_files_CentralOhio_mmdue_AM_1.5";

    // std::string folder = "/home/qiling/Documents/CentralOhio_Honda_Project/Multimodal/scenarios/mobility_service/input_files_CentralOhio_mmdue_AM_2";
    // std::string folder = "/home/qiling/Documents/CentralOhio_Honda_Project/Multimodal/scenarios/no_mobility_service/input_files_CentralOhio_mmdue_AM_2";

    // std::string folder = "/home/qiling/Documents/CentralOhio_Honda_Project/Multimodal/scenarios/mobility_service/input_files_CentralOhio_mmdue_AM_50";
    // std::string folder = "/home/qiling/Documents/CentralOhio_Honda_Project/Multimodal/scenarios/no_mobility_service/input_files_CentralOhio_mmdue_AM_50";

    // std::string folder = "/home/qiling/Documents/CentralOhio_Honda_Project/Multimodal/scenarios/mobility_service/input_files_CentralOhio_mmdue_AM_100";
    // std::string folder = "/home/qiling/Documents/CentralOhio_Honda_Project/Multimodal/scenarios/no_mobility_service/input_files_CentralOhio_mmdue_AM_100";


    // on macOS (Mac air)
    // std::string folder = "/Users/alan-air/Dropbox/MAC-POSTS/data/input_files_MckeesRocks_SPC";
    // std::string folder = "/media/lemma/WD/nhd/experiments/src/temp_input";

    MNM_ConfReader *config = new MNM_ConfReader(folder + "/config.conf", "STAT");
    std::string rec_folder = config -> get_string("rec_folder");


    bool output_link_cong = true; // if true output link congestion level every cong_frequency
    TInt cong_frequency = 1; // 5 s, 15 minutes


    MNM_MM_Due *test_due = new MNM_MM_Due(folder);
    MNM_Dta_Multimodal *mmdta;

    printf("================================ DUE set! =================================\n");

    test_due -> initialize();
    printf("========================= Finished initialization! ========================\n");

    mmdta = test_due -> run_mmdta_adaptive(true);

    print_vehicle_statistics(dynamic_cast<MNM_Veh_Factory_Multimodal*>(mmdta -> m_veh_factory));
    print_passenger_statistics(mmdta -> m_passenger_factory);

    // TInt _count_car, _count_car_pnr, _count_truck, _count_bus, _count_passenger, _count_passenger_pnr;
    // TFlt _tot_tt_car, _tot_tt_truck, _tot_tt_bus, _tot_tt_passenger;
    // _count_car = dynamic_cast<MNM_Veh_Factory_Multimodal*>(mmdta -> m_veh_factory) -> m_finished_car;
    // _count_car_pnr = dynamic_cast<MNM_Veh_Factory_Multimodal*>(mmdta -> m_veh_factory) -> m_finished_car_pnr;
    // _count_truck = dynamic_cast<MNM_Veh_Factory_Multimodal*>(mmdta -> m_veh_factory) -> m_finished_truck;
    // _count_bus = dynamic_cast<MNM_Veh_Factory_Multimodal*>(mmdta -> m_veh_factory) -> m_finished_bus;
    // _tot_tt_car = dynamic_cast<MNM_Veh_Factory_Multimodal*>(mmdta -> m_veh_factory) -> m_total_time_car * mmdta -> m_unit_time / 3600.0;
    // _tot_tt_truck = dynamic_cast<MNM_Veh_Factory_Multimodal*>(mmdta -> m_veh_factory) -> m_total_time_truck * mmdta -> m_unit_time / 3600.0;
    // _tot_tt_bus = dynamic_cast<MNM_Veh_Factory_Multimodal*>(mmdta -> m_veh_factory) -> m_total_time_bus * mmdta -> m_unit_time / 3600.0;
    // printf("\nTotal driving car: %d, Total pnr car: %d, Total truck: %d, Total bus: %d, Total car tt: %.2f hours, Total truck tt: %.2f hours, Total bus tt: %.2f hours\n", 
    //         int(_count_car), int(_count_car_pnr), int(_count_truck), int(_count_bus), float(_tot_tt_car), float(_tot_tt_truck), float(_tot_tt_bus));
    // _count_passenger = mmdta -> m_passenger_factory -> m_finished_passenger;
    // _count_passenger_pnr = mmdta -> m_passenger_factory -> m_finished_passenger_pnr;
    // _tot_tt_passenger = mmdta -> m_passenger_factory -> m_total_time_passenger * mmdta -> m_unit_time / 3600.0;
    // printf("Total passenger: %d, Total pnr passenger: %d, Total Total tt: %.2f hours\n", 
    //         int(_count_passenger), int(_count_passenger_pnr), float(_tot_tt_passenger));

    // test_due -> build_link_cost_map(mmdta);

    // MNM::save_driving_path_table(folder, test_due -> m_driving_path_table,
    //                                 "driving_path_table", "driving_path_table_buffer", true);
    // MNM::save_bustransit_path_table(folder, test_due -> m_bustransit_path_table,
    //                                 "bustransit_path_table", "bustransit_path_table_buffer", true);
    // MNM::save_pnr_path_table(folder, test_due -> m_pnr_path_table,
    //                         "pnr_path_table", "pnr_path_table_buffer", true);

    MNM_Dlink *_link;
    MNM_Dlink_Multiclass *_link_m;
    MNM_Transit_Link *_transit_link;
    // MNM_Walking_Link *_walking_link;
    std::string _str1;
    std::string _str2;
    TInt _current_inter = mmdta -> m_current_loading_interval;
    std::ofstream _vis_file2;
    std::ofstream _vis_file3;
    if (output_link_cong){
        _vis_file2.open(folder + "/" + rec_folder + "/driving_link_cong_raw.txt", std::ofstream::out);
        if (! _vis_file2.is_open()){
            printf("Error happens when open _vis_file2\n");
            exit(-1);
        }
        _vis_file3.open(folder + "/" + rec_folder + "/transit_link_cong_raw.txt", std::ofstream::out);
        if (! _vis_file3.is_open()){
            printf("Error happens when open _vis_file3\n");
            exit(-1);
        }

        _str1 = "timestamp (intervals), driving_link_ID, car_inflow, truck_inflow, car_tt (s), truck_tt (s), car_fftt (s), truck_fftt (s), car_speed (mph), truck_speed (mph)\n";
        _str2 = "timestamp (intervals), bus_transit_link_ID, bus_transit_link_type, passenger_inflow, tt (s), fftt (s)\n";
        _vis_file2 << _str1;
        _vis_file3 << _str2;

        TInt _iter = 0;
        while (_iter + cong_frequency <= _current_inter){
            if (_iter % cong_frequency == 0 || _iter == _current_inter - 1){
                printf("Current loading interval: %d\n", int(_iter));
                for (auto _link_it : mmdta -> m_link_factory -> m_link_map){
                    _link = _link_it.second;
                    _link_m = dynamic_cast<MNM_Dlink_Multiclass*>(_link);
                    _str1 = std::to_string(int(_iter)) + " ";
                    _str1 += std::to_string(_link -> m_link_ID()) + " ";
                    _str1 += std::to_string(MNM_DTA_GRADIENT::get_link_inflow_car(_link_m, _iter, _iter+cong_frequency)) + " ";
                    _str1 += std::to_string(MNM_DTA_GRADIENT::get_link_inflow_truck(_link_m, _iter, _iter+cong_frequency)) + " ";
                    _str1 += std::to_string(MNM_DTA_GRADIENT::get_travel_time_car(_link_m, TFlt(_iter), mmdta -> m_unit_time) * mmdta -> m_unit_time) + " ";
                    _str1 += std::to_string(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m, TFlt(_iter), mmdta -> m_unit_time) * mmdta -> m_unit_time) + " ";
                    _str1 += std::to_string(_link_m -> get_link_freeflow_tt_car()) + " ";
                    _str1 += std::to_string(_link_m -> get_link_freeflow_tt_truck()) + " ";
                    _str1 += std::to_string(_link_m -> m_length/(MNM_DTA_GRADIENT::get_travel_time_car(_link_m, TFlt(_iter), mmdta -> m_unit_time) * mmdta -> m_unit_time) * 3600 / 1600) + " ";
                    _str1 += std::to_string(_link_m -> m_length/(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m, TFlt(_iter), mmdta -> m_unit_time) * mmdta -> m_unit_time) * 3600 / 1600) + "\n";
                    _vis_file2 << _str1;
                }
                for (auto _link_it : mmdta -> m_transitlink_factory -> m_transit_link_map){
                    _transit_link = _link_it.second;
                    _str2 = std::to_string(int(_iter)) + " ";
                    _str2 += std::to_string(_transit_link -> m_link_ID()) + " ";
                    _str2 += std::to_string(_transit_link -> m_link_type) + " ";
                    _str2 += std::to_string(MNM_DTA_GRADIENT::get_link_inflow_passenger(_transit_link, TFlt(_iter), TFlt(_iter+cong_frequency))) + " ";
                    if (_transit_link -> m_link_type == MNM_TYPE_WALKING_MULTIMODAL) {
                        _str2 += std::to_string(MNM_DTA_GRADIENT::get_travel_time_walking(dynamic_cast<MNM_Walking_Link*>(_transit_link), TFlt(_iter), mmdta -> m_unit_time) * mmdta -> m_unit_time) + " ";
                        _str2 += std::to_string(dynamic_cast<MNM_Walking_Link*>(_transit_link) -> m_fftt) + "\n";
                    }
                    else {
                        _str2 += std::to_string(MNM_DTA_GRADIENT::get_travel_time_bus(dynamic_cast<MNM_Bus_Link*>(_transit_link), TFlt(_iter), mmdta -> m_unit_time) * mmdta -> m_unit_time) + " ";
                        _str2 += std::to_string(dynamic_cast<MNM_Bus_Link*>(_transit_link) -> m_fftt) + "\n";
                    }
                    _vis_file3 << _str2;
                }
            }
            _iter += 1;
        }

        // // save cc of some links
        // _str = "\n\n **************************** driving link cc ****************************";
        // for (auto _link_it : mmdta->m_link_factory->m_link_map) {
        //     _link = _link_it.second;
        //     if (_link->m_link_ID() == 4) {
        //         _link_m = dynamic_cast<MNM_Dlink_Multiclass *>(_link);
        //         _str += "\nlink_ID: " + std::to_string(_link->m_link_ID());
        //         _str +="\nm_N_in_car: \n";
        //         _str += _link_m->m_N_in_car->to_string();
        //         _str +="\nm_N_out_car: \n";
        //         _str += _link_m->m_N_out_car->to_string();
        //         _str +="\nm_N_in_truck: \n";
        //         _str += _link_m->m_N_in_truck->to_string();
        //         _str +="\nm_N_out_truck: \n";
        //         _str += _link_m->m_N_out_truck->to_string();
        //         _vis_file2 << _str;
        //     }
        // }

        if (_vis_file2.is_open()) _vis_file2.close();
        if (_vis_file3.is_open()) _vis_file3.close();
    }

    TFlt _tot_demand = 0;
    for (auto _o_it : test_due -> m_passenger_demand) {
        for (auto _d_it : _o_it.second) {
            for (int i = 0; i < test_due -> m_total_assign_inter; ++i) {
                _tot_demand += _d_it.second[i];
            }
        }
    }
    for (auto _it : test_due -> m_mode_share) {
        test_due -> m_mode_share.find(_it.first) -> second = _it.second / _tot_demand;
    }

    std::string _str;
    std::ofstream _vis_file4;
    _vis_file4.open(folder + "/" + rec_folder + "/mode_share.txt", std::ofstream::out);
    if (! _vis_file4.is_open()){
        printf("Error happens when open _vis_file4\n");
        exit(-1);
    }
    _str = "driving, bus_transit, pnr\n";
    if (test_due -> m_mode_share.find(driving) != test_due -> m_mode_share.end()) {
        _str += std::to_string(test_due -> m_mode_share.find(driving) -> second) + " ";
    }
    else {
        _str += "0 ";
    }
    if (test_due -> m_mode_share.find(transit) != test_due -> m_mode_share.end()) {
        _str += std::to_string(test_due -> m_mode_share.find(transit) -> second) + " ";
    }
    else {
        _str += "0 ";
    }
    if (test_due -> m_mode_share.find(pnr) != test_due -> m_mode_share.end()) {
        _str += std::to_string(test_due -> m_mode_share.find(pnr) -> second) + "\n";
    }
    else {
        _str += "0\n";
    }
  
    _vis_file4 << _str;
    if (_vis_file4.is_open()) _vis_file4.close();

    std::string emission_file_name = folder + "/" + rec_folder + "/emission";
    std::ofstream emission_file;
    emission_file.open(emission_file_name, std::ofstream::out);
    if (!emission_file.is_open()){
        printf("Error happens when open emission_file\n");
        exit(-1);
    }
    // freopen((folder + "/" + rec_folder + "/emission_output.log").c_str(), "w", stdout);
    emission_file << mmdta -> m_emission -> output();
    emission_file.close();

    // delete mmdta;
    delete config;
    delete test_due;
    printf("====================== Finished delete test_due! ====================\n");

    return 0;
}

