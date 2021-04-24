#include "due.h"
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
    std::string file_folder = "../../../data/input_files_7link_due";
    // std::string file_folder = "../../../data/input_files_new_philly";

    MNM_ConfReader *config = new MNM_ConfReader(file_folder + "/config.conf", "STAT");
    std::string rec_folder = config -> get_string("rec_folder");

    MNM_Due *due = new MNM_Due_Msa(file_folder);
    due->initialize();  // create and set m_buffer[i] = 0
    due->init_path_flow();

    MNM_Dta *dta;

    std::string gap_file_name = file_folder + "/" + rec_folder + "/gap_iteration";
    std::ofstream gap_file;
    gap_file.open(gap_file_name, std::ofstream::out);
    if (!gap_file.is_open()){
        printf("Error happens when open gap_file\n");
        exit(-1);
    }

    TFlt gap;
    for (int i = 0; i < 100; ++i) {
        printf("---------- Iteration %d ----------\n", i);

        // DNL using dta.cpp, new dta is built from scratch
        dta = due->run_dta(false);

        // search for the lowest disutility route and update path flow
        // with departure time choice
        // due->update_path_table(dta, i);
        // fixed departure time choice
        // due->update_path_table_fixed_departure_time_choice(dta, i);
        // gradient projection
        due->update_path_table_gp_fixed_departure_time_choice(dta, i);

        // calculate gap
        // with departure time choice
        // gap = due -> compute_merit_function();
        // fixed departure time choice
        gap = due -> compute_merit_function_fixed_departure_time_choice();
        printf("GAP = %lf\n", (float) gap);
        gap_file << std::to_string(gap) + "\n";
        // delete dta;
    }

    gap_file.close();

    delete config;
    delete dta;
    delete due;
    printf("Finished\n");
    return 0;
}