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

    // On ubuntu (PC)
    // std::string folder = "/home/alanpi/Desktop/MAC-POSTS/data/input_files_SPC_separate_Routing";
    // std::string folder = "/home/lemma/Documents/MAC-POSTS/src/examples/mcDODE/a6e7b31067d2ead8d3725fc0ed587d06c958f63c";

//    std::string folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_due_columngeneration";
    std::string folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_7link_multimodal_due_fixedpath";

    // on macOS (Mac air)
    // std::string folder = "/Users/alan-air/Dropbox/MAC-POSTS/data/input_files_MckeesRocks_SPC";
    // std::string folder = "/media/lemma/WD/nhd/experiments/src/temp_input";

    MNM_ConfReader *config = new MNM_ConfReader(folder + "/config.conf", "STAT");
    std::string rec_folder = config -> get_string("rec_folder");


    MNM_MM_Due *test_due = new MNM_MM_Due(folder);
    MNM_Dta_Multimodal *mmdta;

    printf("================================ DUE set! =================================\n");

    test_due -> initialize();
    printf("========================= Finished initialization! ========================\n");

    test_due -> init_passenger_path_flow();
    printf("====================== Finished initialize passenger path flow! ====================\n");

    std::string gap_file_name = folder + "/" + rec_folder + "/gap_iteration";
    std::ofstream gap_file;
    gap_file.open(gap_file_name, std::ofstream::out);
    if (!gap_file.is_open()){
        printf("Error happens when open gap_file\n");
        exit(-1);
    }

    TFlt gap;
    for (int i = 0; i < test_due -> m_max_iter; ++i) {
        printf("---------- Iteration %d ----------\n", i);

        // DNL using dta, new dta is built from scratch
        mmdta = test_due->run_mmdta(false);

        // update time dependent cost and save existing path table
        test_due -> update_path_table_cost(mmdta);

        // calculate gap
        // with departure time choice
        // gap = test_due -> compute_merit_function(mmdta);
        // fixed departure time choice
        gap = test_due -> compute_merit_function_fixed_departure_time_choice(mmdta);
        printf("\n\n*******************GAP = %lf*******************\n\n", (float) gap);
        gap_file << std::to_string(gap) + "\n";

        // search for the lowest disutility route and update path flow
        // with departure time choice
        // test_due->update_path_table(mmdta, i);
        // fixed departure time choice
        // test_due->update_path_table_fixed_departure_time_choice(mmdta, i);
        // gradient projection
        test_due->update_path_table_gp_fixed_departure_time_choice(mmdta, i);

        delete mmdta;
    }

    gap_file.close();

    delete config;
    // delete mmdta;
    delete test_due;
    printf("====================== Finished delete test_due! ====================\n");

    return 0;
}

