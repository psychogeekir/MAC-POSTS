#include "io.h"
#include "Snap.h"
#include "multimodal.h"

int main() {
    // print cwd
    char buffer[256];
    char *val = getcwd(buffer, sizeof(buffer));
    if (val) {
        std::cout << buffer << std::endl;
    }

    std::string folder = "/srv/data/qiling/Projects/CentralOhio_Honda_Project/Multimodal/input_files_CentralOhio_multimodal_AM";
    double min_path_tt = 0;
    int max_iter_driving = 0;
    int max_iter_bustransit = 1;
    int max_iter_pnr = 0;
    double mid_scale = 3;
    double heavy_scale = 6;

    // ******************************************************
    // Mmdta_Api::generate_shortest_pathsets
    // ******************************************************
    // no driving_demand, bustransit_demand, and pnr_demand files, no corresponding path_table and path_table_buffer files
    auto *m_mmdue = new MNM_MM_Due(folder);
    m_mmdue -> initialize();
    m_mmdue -> m_mmdta -> is_ok();

    if (std::find(m_mmdue -> m_mode_vec.begin(), m_mmdue -> m_mode_vec.end(), driving) != m_mmdue -> m_mode_vec.end()) {
        Path_Table *_driving_path_table = \
            MNM::build_shortest_driving_pathset(m_mmdue -> m_mmdta -> m_graph, m_mmdue -> m_mmdta -> m_od_factory, m_mmdue -> m_od_mode_connectivity,
                                                m_mmdue -> m_mmdta -> m_link_factory, min_path_tt, max_iter_driving, mid_scale, heavy_scale, 2 * m_mmdue -> m_total_assign_inter);
        printf("driving pathset generated\n");
        MNM::save_driving_path_table(folder, _driving_path_table, "driving_path_table", "driving_path_table_buffer", true);
        printf("driving pathset saved\n");

        for (auto _it: *_driving_path_table) {
            for (auto _it_it : *_it.second) {
                delete _it_it.second;
            }
            _it.second -> clear();
            delete _it.second;
        }
        _driving_path_table -> clear();
        delete _driving_path_table;
    }
    if (std::find(m_mmdue -> m_mode_vec.begin(), m_mmdue -> m_mode_vec.end(), transit) != m_mmdue -> m_mode_vec.end()) {
        Path_Table *_bustransit_path_table = \
            MNM::build_shortest_bustransit_pathset(m_mmdue -> m_mmdta -> m_bus_transit_graph, m_mmdue -> m_mmdta -> m_od_factory, m_mmdue -> m_od_mode_connectivity,
                                                m_mmdue -> m_mmdta -> m_transitlink_factory, min_path_tt, max_iter_bustransit, mid_scale, heavy_scale, m_mmdue -> m_total_assign_inter);
        printf("bus transit pathset generated\n");
        MNM::save_bustransit_path_table(folder, _bustransit_path_table, "bustransit_path_table", "bustransit_path_table_buffer", true);
        printf("bus transit pathset saved\n");

        for (auto _it: *_bustransit_path_table) {
            for (auto _it_it : *_it.second) {
                delete _it_it.second;
            }
            _it.second -> clear();
            delete _it.second;
        }
        _bustransit_path_table -> clear();
        delete _bustransit_path_table;
    }
    if (std::find(m_mmdue -> m_mode_vec.begin(), m_mmdue -> m_mode_vec.end(), pnr) != m_mmdue -> m_mode_vec.end()) {
        PnR_Path_Table *_pnr_path_table = \
            MNM::build_shortest_pnr_pathset(m_mmdue -> m_mmdta -> m_graph, m_mmdue -> m_mmdta -> m_bus_transit_graph, m_mmdue -> m_mmdta -> m_od_factory,
                                            m_mmdue -> m_od_mode_connectivity, m_mmdue -> m_mmdta -> m_link_factory, m_mmdue -> m_mmdta -> m_transitlink_factory,
                                            min_path_tt, max_iter_pnr, mid_scale, heavy_scale, m_mmdue -> m_total_assign_inter);
        printf("pnr pathset generated\n");
        MNM::save_pnr_path_table(folder, _pnr_path_table, "pnr_path_table", "pnr_path_table_buffer", true);
        printf("pnr pathset saved\n");

        for (auto _it: *_pnr_path_table) {
            for (auto _it_it : *_it.second) {
                delete _it_it.second;
            }
            _it.second -> clear();
            delete _it.second;
        }
        _pnr_path_table -> clear();
        delete _pnr_path_table;
    }

    MNM::generate_init_mode_demand_file(m_mmdue, folder);
    return 0;
}