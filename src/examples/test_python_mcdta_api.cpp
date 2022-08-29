#include "io.h"
#include "Snap.h"
#include "multiclass.h"

#include <vector>

//#include "../pybinder/pybind11/include/pybind11/pybind11.h"
//#include "../pybinder/pybind11/include/pybind11/numpy.h"
//namespace py = pybind11;

int main() {
    // print cwd
    char buffer[256];
    char *val = getcwd(buffer, sizeof(buffer));
    if (val) {
        std::cout << buffer << std::endl;
    }

    printf("BEGIN MNMAPI test!\n");

    std::string folder = "/srv/data/qiling/Projects/Philly/input_files_philly";

    MNM_ConfReader *config = new MNM_ConfReader(folder + "/config.conf", "STAT");
    std::string rec_folder = config -> get_string("rec_folder");

    TInt m_num_path = TInt(0);

    MNM_Dta_Multiclass *m_mcdta;
    std::vector<MNM_Dlink_Multiclass*> m_link_vec;
    std::vector<MNM_Path*> m_path_vec;
    std::set<MNM_Path*> m_path_set; 
    std::unordered_map<TInt, MNM_Path*> m_ID_path_mapping;

    // time-varying link cost
    std::unordered_map<TInt, TFlt *> m_link_cost_map;
    std::unordered_map<TInt, TFlt *> m_link_cost_map_truck;

    // time-varying indicator
    std::unordered_map<TInt, bool *> m_link_congested_car;
    std::unordered_map<TInt, bool *> m_link_congested_truck;

    // ******************************************************
    // Mcdta_Api::initialize()
    // ******************************************************
    m_mcdta = new MNM_Dta_Multiclass(folder);
    m_mcdta -> build_from_files();
    m_mcdta -> hook_up_node_and_link();
    m_mcdta -> is_ok();
    if (MNM_Routing_Fixed *_routing = dynamic_cast<MNM_Routing_Fixed *>(m_mcdta -> m_routing)){
        MNM::get_ID_path_mapping(m_ID_path_mapping, _routing -> m_path_table);
        // return 0;
    }
    if (MNM_Routing_Hybrid *_routing = dynamic_cast<MNM_Routing_Hybrid *>(m_mcdta -> m_routing)){
        // printf("start load ID path mapping\n");
        MNM::get_ID_path_mapping(m_ID_path_mapping, _routing -> m_routing_fixed -> m_path_table);
        // return 0;
        // printf("mapping size %d\n", m_ID_path_mapping.size());
    }
    if (MNM_Routing_Biclass_Hybrid *_routing = dynamic_cast<MNM_Routing_Biclass_Hybrid *>(m_mcdta -> m_routing)){
        printf("MNM_Routing_Biclass_Hybrid start load ID path mapping\n");
        MNM::get_ID_path_mapping(m_ID_path_mapping, _routing -> m_routing_fixed_car -> m_path_table);
        printf("MNM_Routing_Biclass_Hybrid mapping size %d\n", (int)m_ID_path_mapping.size());
        // return 0;
    }
    printf("xxx\n");
    std::runtime_error("Mcdta_Api:: Routing type not implemented in API");

    // ******************************************************
    // Mcdta_Api::register_links
    // ******************************************************
    if (m_link_vec.size() > 0){
        printf("Warning, Mcdta_Api::register_links, link exists\n");
        m_link_vec.clear();
    }
    std::vector<int> links_ptr = std::vector<int>();
    int _count = 0;

    // from file
    /* find file */
	std::string _file_name = "/srv/data/qiling/Projects/Philly/training_data_AM/observed_link_driving_list";
	std::ifstream _file;
	_file.open(_file_name, std::ios::in);

    std::string _line;
	std::vector<std::string> _words;
	if (_file.is_open()) {
		/* read config */
		TInt _num_of_lines = 1563;

		// printf("Processing Origin node.\n");
		for (int i=0; i < _num_of_lines; ++i){
			std::getline(_file, _line);
            _words = MNM_IO::split(_line, ' ');
			if ((int)_words.size() == 1) {   // check 
				// std::cout << "Processing: " << _line << "\n";
				links_ptr.push_back(TInt(std::stoi(_words[0])));
                _count++;
			}
		}
	}
	else {
		printf("No observed_link_driving_list\n");
	}
	_file.close();
    // from file

    // all links
    // for (auto it: m_mcdta -> m_link_factory -> m_link_map) {
    //     links_ptr.push_back((int)it.first);
    //     _count++;
    // }

    // example links
    // std::vector<int> links_ID = {2, 3, 4, 5};
    // for (auto it: links_ID) {
    //     links_ptr.push_back(it);
    //     _count++;
    // }

    MNM_Dlink *_link;
    for (int i = 0; i < _count; i++){
        _link = m_mcdta -> m_link_factory -> get_link(TInt(links_ptr[i]));
        // printf("%d\n", links_ptr[i]);
        if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
            if(std::find(m_link_vec.begin(), m_link_vec.end(), _link) != m_link_vec.end()) {
                throw std::runtime_error("Error, Mcdta_Api::register_links, link does not exist");
            } 
            else {
                m_link_vec.push_back(_mclink);
            }
        }
        else{
            throw std::runtime_error("Mcdta_Api::register_links: link type is not multiclass");
        }
    }

    // ******************************************************
    // Mcdta_Api::register_paths
    // ******************************************************
    if (m_path_vec.size() > 0){
        printf("Warning, Mmdta_Api::register_paths, path exists\n");
        m_path_vec.clear();
        m_path_set.clear();
    }

    std::vector<int> paths_ptr = std::vector<int>(); // = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    for (int i = 0; i < (int)m_ID_path_mapping.size(); i++){
        paths_ptr.push_back(i);
    }
    TInt _path_ID;
    for (int i = 0; i < (int)paths_ptr.size(); i++){
        _path_ID = TInt(paths_ptr[i]);
        // printf("registering path %d, %d\n", _path_ID(), (int)m_ID_path_mapping.size());
        if (m_ID_path_mapping.find(_path_ID) == m_ID_path_mapping.end()){
        throw std::runtime_error("Mcdta_Api::register_paths: No such path");
        }
        else {
        m_path_vec.push_back(m_ID_path_mapping[_path_ID]);
        }
    }
    m_path_set = std::set<MNM_Path*> (m_path_vec.begin(), m_path_vec.end());

    // ******************************************************
    // Mcdta_Api::check_registered_links_in_registered_paths
    // Mcdta_Api::generate_paths_to_cover_registered_links
    // ******************************************************
    {
        // Mcdta_Api::check_registered_links_in_registered_paths
        std::vector<bool> _link_existing = std::vector<bool> ();
        if (m_link_vec.empty()){
            printf("Warning, Mcdta_Api::check_registered_links_in_registered_paths, no link registered\n");
            // return _link_existing;
            exit(-1);
        }
        for (size_t k=0; k < m_link_vec.size(); ++k) {
            _link_existing.push_back(false);
        }
        if (m_path_vec.empty()){
            printf("Warning, Mcdta_Api::check_registered_links_in_registered_paths, no path registered\n");
            // return _link_existing;
            exit(-1);
        }
        for (auto* _path : m_path_vec) {
            for (size_t i = 0; i < m_link_vec.size(); ++i) {
                if (!_link_existing[i]) {
                    _link_existing[i] = _path -> is_link_in(m_link_vec[i] -> m_link_ID);
                }
            }
            if (std::all_of(_link_existing.cbegin(), _link_existing.cend(), [](bool v){return v;})) {
                break;
            }
        }
        if (std::any_of(_link_existing.cbegin(), _link_existing.cend(), [](bool v){return !v;})) {
            printf("Warning: some observed driving links in m_link_vec are not covered by generated paths in m_path_vec!\n");
        }

        // Mcdta_Api::generate_paths_to_cover_registered_links
        PNEGraph reversed_graph = MNM_Ults::reverse_graph(m_mcdta -> m_graph);
        std::unordered_map<TInt, TFlt> _cost_map = std::unordered_map<TInt, TFlt> ();
        for (auto _link_it : m_mcdta -> m_link_factory -> m_link_map) {
            _cost_map.insert(std::pair<TInt, TFlt>(_link_it.first, dynamic_cast<MNM_Dlink_Multiclass*>(_link_it.second) -> get_link_freeflow_tt_car()));
        }
        std::unordered_map<TInt, TInt> _shortest_path_tree = std::unordered_map<TInt, TInt>();
        std::unordered_map<TInt, TInt> _shortest_path_tree_reversed = std::unordered_map<TInt, TInt>();
        TInt _from_node_ID, _to_node_ID;
        MNM_Origin_Multiclass *_origin;
        MNM_Destination_Multiclass *_dest;
        MNM_Path *_path_1, *_path_2, *_path;
        std::random_device rng; // random sequence
        std::vector<std::pair<TInt, MNM_Origin*>> pair_ptrs_1 = std::vector<std::pair<TInt, MNM_Origin*>> ();
        std::vector<std::pair<MNM_Destination*, TFlt*>> pair_ptrs_2 = std::vector<std::pair<MNM_Destination*, TFlt*>> ();


        for (size_t i = 0; i < m_link_vec.size(); ++i) {
            if (!_link_existing[i]) {
                // generate new path including this link
                _from_node_ID = m_mcdta -> m_graph -> GetEI(m_link_vec[i] -> m_link_ID).GetSrcNId(); 
                _to_node_ID = m_mcdta -> m_graph -> GetEI(m_link_vec[i] -> m_link_ID).GetDstNId();

                // path from origin to from_node_ID
                if (!_shortest_path_tree.empty()) {
                    _shortest_path_tree.clear();
                }
                if (dynamic_cast<MNM_DMOND_Multiclass*>(m_mcdta -> m_node_factory -> get_node(_from_node_ID)) != nullptr) {
                    _path_1 = new MNM_Path();
                    _path_1->m_node_vec.push_back(_from_node_ID);
                }
                else {
                    MNM_Shortest_Path::all_to_one_FIFO(_from_node_ID, m_mcdta -> m_graph, _cost_map, _shortest_path_tree);
                }
                
                // path from to_node_ID to destination
                if (!_shortest_path_tree_reversed.empty()) {
                    _shortest_path_tree_reversed.clear();
                }
                if (dynamic_cast<MNM_DMDND_Multiclass*>(m_mcdta -> m_node_factory -> get_node(_to_node_ID)) != nullptr) {
                    _path_2 = new MNM_Path();
                    _path_2 -> m_node_vec.push_back(_to_node_ID);
                }
                else {
                    MNM_Shortest_Path::all_to_one_FIFO(_to_node_ID, reversed_graph, _cost_map, _shortest_path_tree_reversed);
                }

                _origin = nullptr;
                _dest = nullptr;
                bool _flg = false;

                if (!pair_ptrs_1.empty()) {
                    pair_ptrs_1.clear();
                }
                for(const auto& p : m_mcdta -> m_od_factory -> m_origin_map) 
                {
                    pair_ptrs_1.emplace_back(p);
                }
                std::shuffle(std::begin(pair_ptrs_1), std::end(pair_ptrs_1), rng);
                for (auto _it : pair_ptrs_1) {
                    _origin = dynamic_cast<MNM_Origin_Multiclass*>(_it.second);
                    if (_origin -> m_demand_car.empty()) {
                        continue;
                    }

                    if (!pair_ptrs_2.empty()) {
                        pair_ptrs_2.clear();
                    }
                    for(const auto& p : _origin -> m_demand_car) 
                    {
                        pair_ptrs_2.emplace_back(p);
                    }
                    std::shuffle(std::begin(pair_ptrs_2), std::end(pair_ptrs_2), rng);
                    for (auto _it_it : pair_ptrs_2) {
                        _dest = dynamic_cast<MNM_Destination_Multiclass*>(_it_it.first);
                        if (_shortest_path_tree.find(_origin -> m_origin_node -> m_node_ID) -> second != -1 &&
                            _shortest_path_tree_reversed.find(_dest -> m_dest_node -> m_node_ID) -> second != -1) {
                            _flg = true;
                            break;
                        }
                    }
                    if (_flg) {
                        break;
                    }
                }

                if (!_flg) {
                    printf("Cannot generate path covering this link\n");
                    // exit(-1);
                    continue;
                }
                IAssert(_origin != nullptr && _dest != nullptr);

                if (!_shortest_path_tree.empty()) {
                    _path_1 = MNM::extract_path(_origin -> m_origin_node -> m_node_ID, _from_node_ID, 
                                                _shortest_path_tree, m_mcdta -> m_graph); 
                }
                if (!_shortest_path_tree_reversed.empty()) {
                    _path_2 = MNM::extract_path(_dest -> m_dest_node -> m_node_ID, _to_node_ID,
                                                _shortest_path_tree_reversed, reversed_graph); 
                }
                
                // merge the paths to a complete path
                _path = new MNM_Path();
                _path -> m_link_vec = _path_1 -> m_link_vec;
                _path -> m_link_vec.push_back(m_link_vec[i] -> m_link_ID);
                _path -> m_link_vec.insert(_path -> m_link_vec.end(), _path_2 -> m_link_vec.rbegin(), _path_2 -> m_link_vec.rend());
                _path -> m_node_vec = _path_1 -> m_node_vec;
                _path -> m_node_vec.insert(_path -> m_node_vec.end(), _path_2 -> m_node_vec.rbegin(), _path_2 -> m_node_vec.rend());
                _path -> allocate_buffer(2 * m_mcdta -> m_config -> get_int("max_interval"));
                delete _path_1;
                delete _path_2;
                // add this new path to path table
                dynamic_cast<MNM_Routing_Biclass_Hybrid*>(m_mcdta -> m_routing) -> m_routing_fixed_car -> m_path_table->find(_origin -> m_origin_node -> m_node_ID) -> second->find(_dest -> m_dest_node -> m_node_ID) -> second -> m_path_vec.push_back(_path);
                m_path_vec.push_back(_path);
                _link_existing[i] = true;

                // check if this new path cover other links
                for (size_t j = 0; j < m_link_vec.size(); ++j) {
                    if (!_link_existing[j]) {
                        _link_existing[j] = _path -> is_link_in(m_link_vec[j] -> m_link_ID);
                    }
                }
                if (std::all_of(_link_existing.cbegin(), _link_existing.cend(), [](bool v){return v;})) {
                    printf("All links in m_link_vec are covered by paths in m_path_vec!\n");
                    break;
                }
            }
        }

        if (std::all_of(_link_existing.cbegin(), _link_existing.cend(), [](bool v){return v;})) {
            printf("All links in m_link_vec are covered by paths in m_path_vec!\n");
        }
        else {
            printf("Mcdta_Api::generate_paths_to_cover_registered_links, NOT all links in m_link_vec are covered by paths in m_path_vec!\n");
            // exit(-1);
        }

        MNM::save_path_table(m_mcdta -> m_file_folder, dynamic_cast<MNM_Routing_Biclass_Hybrid*>(m_mcdta -> m_routing) -> m_routing_fixed_car -> m_path_table, m_mcdta -> m_od_factory, true);
        
        _link_existing.clear();
        _cost_map.clear();
        _shortest_path_tree.clear();
        _shortest_path_tree_reversed.clear();
        reversed_graph.Clr();
        pair_ptrs_1.clear();
        pair_ptrs_2.clear();
    }

    return 0;
}