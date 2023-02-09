#include "dso.h"
#include "Snap.h"

#include <string>

int main() {
    // https://stackoverflow.com/questions/29441631/cannot-open-file-with-relative-path-c-ifstream
    // print current working directory
    char buffer[256];
    char *val = getcwd(buffer, sizeof(buffer));
    if (val) {
        std::cout << buffer << std::endl;
    }

    // the file path is relative to the current working directory (if launch.json exists, refer to "cwd"; otherwise it can be executable directory instead of source directory)
    std::string file_folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_7link_dso";
    // std::string file_folder = "../../../data/input_files_new_philly";

    MNM_ConfReader *config = new MNM_ConfReader(file_folder + "/config.conf", "STAT");
    std::string rec_folder = config -> get_string("rec_folder");
    delete config;

    MNM_Dso *dso = new MNM_Dso(file_folder);
    dso->initialize();  // create and set m_buffer[i] = 0
    dso->init_path_flow();

    MNM_Dta *dta;

    std::string gap_file_name = file_folder + "/" + rec_folder + "/gap_iteration";
    std::ofstream gap_file;
    gap_file.open(gap_file_name, std::ofstream::out);
    if (!gap_file.is_open()){
        printf("Error happens when open gap_file\n");
        exit(-1);
    }

    TFlt gap, total_tt;
    for (int i = 0; i < 100; ++i) {
        printf("---------- Iteration %d ----------\n", i);

        // DNL using dta.cpp, new dta is built from scratch
        dta = dso->run_dta(false);

        // time-dependent link cost
        dso -> build_link_cost_map(dta);
        total_tt = dso -> compute_total_travel_time();

        // time-dependent link marginal cost in m_link_tt_map
        dso -> get_link_marginal_cost(dta);

        dso -> update_path_table_cost(dta);

        // save path table
        MNM::print_path_table(dso -> m_path_table, dta->m_od_factory, true, true);
        MNM::save_path_table(file_folder + "/" + rec_folder, dso -> m_path_table, dta->m_od_factory, true, true);

        // calculate gap
        // with departure time choice
        // gap = dso -> compute_merit_function();
        // fixed departure time choice
        gap = dso -> compute_merit_function_fixed_departure_time_choice();

        printf("GAP = %lf, total tt = %lf\n", (float) gap, (float)total_tt);
        gap_file << std::to_string(gap) + " " + std::to_string(total_tt) + "\n";

        // search for the lowest disutility route and update path flow
        // with departure time choice
        // dso->update_path_table(dta, i);
        // fixed departure time choice
        // dso->update_path_table_fixed_departure_time_choice(dta, i);
        // gradient projection
        dso->update_path_table_gp_fixed_departure_time_choice(dta, i);

        dynamic_cast<MNM_Routing_Fixed*>(dta -> m_routing) -> m_path_table = nullptr;
        delete dta;
    }

    gap_file.close();

    delete dso;
    printf("Finished\n");
    return 0;
}