#include "Snap.h"

#include "path.h"
#include "io.h"
#include "shortest_path.h"
#include "ults.h"

#include <string>

int main()
{

    // print cwd
    char buffer[256];
    char *val = getcwd(buffer, sizeof(buffer));
    if (val) {
        std::cout << buffer << std::endl;
    }

    printf("BEGIN TDSP with node cost test!\n");

    std::string file_folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_16link_tdsp_node_cost";
    // std::string file_folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_PGH_tdsp_node_cost";
    MNM_ConfReader *conf_reader = new MNM_ConfReader(file_folder + "/config.conf", "Network");
    // printf("1\n");
    PNEGraph graph = MNM_IO::build_graph(file_folder, conf_reader);
    delete conf_reader;
    // printf("2\n");
    // TInt dest_node_ID = graph -> GetRndNId();

    // 16 link
    TInt dest_node_ID = 13;
    TInt ori_node_ID = 1;
    
    TInt num_rows_link_file = 23;
    TInt num_rows_node_file = 10;
    TInt max_interval = 100;

    // PGH
    // TInt dest_node_ID = 150361;
    // TInt ori_node_ID = 100264;
    
    // TInt num_rows_link_file = 12976;
    // TInt num_rows_node_file = 10000;
    // TInt max_interval = 5000;

    std::unordered_map<TInt, TFlt*> td_link_cost = std::unordered_map<TInt, TFlt*> ();
    std::unordered_map<TInt, std::unordered_map<TInt, TFlt*>> td_node_cost = std::unordered_map<TInt, std::unordered_map<TInt, TFlt*>> ();
    std::unordered_map<TInt, TFlt*> td_link_tt = std::unordered_map<TInt, TFlt*> ();
    std::unordered_map<TInt, std::unordered_map<TInt, TFlt*>> td_node_tt = std::unordered_map<TInt, std::unordered_map<TInt, TFlt*>> ();

    // external input
    MNM_IO::read_td_link_cost(file_folder, td_link_cost, num_rows_link_file, max_interval, "td_link_cost");
    MNM_IO::read_td_node_cost(file_folder, td_node_cost, num_rows_node_file, max_interval, "td_node_cost");

    MNM_IO::read_td_link_cost(file_folder, td_link_tt, num_rows_link_file, max_interval, "td_link_tt");
    MNM_IO::read_td_node_cost(file_folder, td_node_tt, num_rows_node_file, max_interval, "td_node_tt");

    MNM_TDSP_Tree *tdsp_tree = new MNM_TDSP_Tree(dest_node_ID, graph, max_interval);
    printf("Init TDSP tree\n");
    tdsp_tree -> initialize();
    printf("Update tree\n");
    tdsp_tree -> update_tree(td_link_cost, td_node_cost, td_link_tt, td_node_tt);
    TFlt tmp_cost;
    MNM_Path *_path;
    std::string _str;
    for (int i=0; i<max_interval; ++i){
        printf("get distance to dest\n");
        tmp_cost = tdsp_tree -> m_dist[ori_node_ID][i];
        printf("At time %d, minimum cost is %f\n", i, tmp_cost());
        _path = new MNM_Path();
        tdsp_tree -> get_tdsp(ori_node_ID, i, td_link_tt, td_node_tt, _path);
        printf("number of nodes %d\n", int(_path -> m_node_vec.size()));
        _str = _path -> node_vec_to_string();
        std::cout << "path: " << _str << "\n";
        delete _path;
    }
    
    delete tdsp_tree;
    for (auto _it : td_link_cost) {
        free(_it.second);
    }
    for (auto _it: td_node_cost) {
        for (auto _it_it : _it.second) {
            free(_it_it.second);
        }
    }
    for (auto _it : td_link_tt) {
        free(_it.second);
    }
    for (auto _it: td_node_tt) {
        for (auto _it_it : _it.second) {
            free(_it_it.second);
        }
    }

    return 0;
}