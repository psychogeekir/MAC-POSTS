#include "dnode.h"

#include "path.h"

#include <algorithm>

MNM_Dnode::MNM_Dnode(TInt ID, TFlt flow_scalar)
{
  m_node_ID = ID;
  m_flow_scalar = flow_scalar;
  m_out_link_array = std::vector<MNM_Dlink*>();
  m_in_link_array = std::vector<MNM_Dlink*>();
}

MNM_Dnode::~MNM_Dnode()
{
  m_out_link_array.clear();
  m_in_link_array.clear();
}

/**************************************************************************
                          Origin node
**************************************************************************/

MNM_DMOND::MNM_DMOND(TInt ID, TFlt flow_scalar)
  : MNM_Dnode::MNM_Dnode(ID, flow_scalar)
{
  m_origin = nullptr;
  m_out_volume = std::unordered_map<MNM_Dlink*, TInt>();
  m_in_veh_queue = std::deque<MNM_Veh *>();
}

MNM_DMOND::~MNM_DMOND()
{
  m_in_veh_queue.clear();
  m_out_volume.clear();
}


int MNM_DMOND::evolve(TInt timestamp)
{
  // printf("MNM_DMOND node evolve\n");
  MNM_Dlink *_link, *_to_link;

  for (unsigned i=0; i<m_out_link_array.size(); ++i){
    _link = m_out_link_array[i];
    m_out_volume.find(_link) -> second = 0;
  }  

  /* compute out flow */
  std::deque<MNM_Veh*>::iterator _que_it = m_in_veh_queue.begin();
  while (_que_it != m_in_veh_queue.end()) {
    _link = (*_que_it) -> get_next_link();
    m_out_volume.find(_link) -> second += 1;
    _que_it++;
  }
  /* compare outflow with the capacity */
  for (unsigned i=0; i<m_out_link_array.size(); ++i){
    _link = m_out_link_array[i];
    if ((_link -> get_link_supply() * m_flow_scalar) < TFlt(m_out_volume.find(_link) -> second)){
      m_out_volume.find(_link) -> second = TInt(MNM_Ults::round(_link -> get_link_supply() * m_flow_scalar));
    }
  }
  /* move vehicle */
  MNM_Veh *_veh;
  for (unsigned i=0; i<m_out_link_array.size(); ++i){
    _link = m_out_link_array[i];
    if (_link -> m_N_in != nullptr && m_out_volume.find(_link) -> second > 0) {
      _link -> m_N_in -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(m_out_volume.find(_link) -> second)/m_flow_scalar));
    }
//    if (!m_in_veh_queue.empty()){
//        printf("In node %d, %d veh to move to link %d, total veh %d \n", m_node_ID, m_out_volume.find(_link) -> second, _link -> m_link_ID, m_in_veh_queue.size());
//    }
    _que_it = m_in_veh_queue.begin();
    while (_que_it != m_in_veh_queue.end()) {
      if (m_out_volume.find(_link) -> second > 0){
        _veh = *_que_it;
        _to_link = _veh -> get_next_link();
        if (_to_link == _link){
           _to_link -> m_incoming_array.push_back(_veh);
          (_veh) -> set_current_link(_to_link);
          _que_it = m_in_veh_queue.erase(_que_it); //c++ 11, erase during loop, https://en.cppreference.com/w/cpp/container/deque/erase
          m_out_volume.find(_to_link) -> second -= 1;
        }
        else{
          _que_it++;
        }
      }
      else{
        break; //break while loop
      }

    }
  }

  // for (unsigned i=0; i<m_out_link_array.size(); ++i){
  //   _link = m_out_link_array[i];
  //   if (m_out_volume.find(_link) -> second != 0){
  //     printf("Something wrong in moving vehicles on DMOND\n");
  //     printf("Remaining out volume in link %d is %d\n", (int)_link -> m_link_ID, (int)m_out_volume.find(_link) -> second);
  //     exit(-1);
  //   }
  // }  
  // printf("Finish MNM_DMOND evolve\n");
  return 0;
}


int MNM_DMOND::add_out_link(MNM_Dlink* out_link)
{
  m_out_link_array.push_back(out_link);
  m_out_volume.insert(std::pair<MNM_Dlink*, TInt>(out_link, TInt(0)));
  return 0;
}

void MNM_DMOND::print_info()
{
  ;
}

int MNM_DMOND::hook_up_origin(MNM_Origin *origin)
{
  m_origin = origin;
  return 0;
}

/**************************************************************************
                          Destination node
**************************************************************************/

MNM_DMDND::MNM_DMDND(TInt ID, TFlt flow_scalar)
  : MNM_Dnode::MNM_Dnode(ID, flow_scalar)
{
  m_dest = nullptr;
}

MNM_DMDND::~MNM_DMDND()
{
  m_out_veh_queue.clear();
}

int MNM_DMDND::add_in_link(MNM_Dlink *link)
{
  m_in_link_array.push_back(link);
  return 0;
}

int MNM_DMDND::evolve(TInt timestamp)
{
  // printf("MNM_DMDND node evomlve\n");
  MNM_Dlink *_link;
  MNM_Veh *_veh;
  size_t _size;
  // printf("in link size:%d\n", m_in_link_array.size());
  for (size_t i = 0; i<m_in_link_array.size(); ++i){
    _link = m_in_link_array[i];
    _size = _link->m_finished_array.size();
    if (_link -> m_N_out != nullptr && _size > 0) {
      _link -> m_N_out -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_size)/m_flow_scalar));
    }
    for (size_t j=0; j<_size; ++j){
      _veh = _link->m_finished_array.front();
      if (_veh -> get_next_link() != nullptr){
        printf("Something wrong in DMDND evolve\n");
        exit(-1);
      }
      m_out_veh_queue.push_back(_veh);
      _veh -> set_current_link(nullptr);
      _link -> m_finished_array.pop_front();
    }
  }
  return 0;
}

void MNM_DMDND::print_info()
{
  ;
}

int MNM_DMDND::hook_up_destination(MNM_Destination *dest)
{
  m_dest = dest;
  return 0;
}


/**************************************************************************
                              In-out node
**************************************************************************/
MNM_Dnode_Inout::MNM_Dnode_Inout(TInt ID, TFlt flow_scalar)
  : MNM_Dnode::MNM_Dnode(ID, flow_scalar)
{
  m_demand = nullptr;
  m_supply = nullptr;
  m_veh_flow = nullptr;
  m_veh_tomove = nullptr;
}

MNM_Dnode_Inout::~MNM_Dnode_Inout()
{
  if (m_demand != nullptr) free(m_demand);
  if (m_supply != nullptr) free(m_supply);
  if (m_veh_flow != nullptr) free(m_veh_flow);
  if (m_veh_tomove != nullptr) free(m_veh_tomove);
}

int MNM_Dnode_Inout::prepare_loading()
{
  TInt _num_in = m_in_link_array.size();
  TInt _num_out = m_out_link_array.size();
  // printf("node id %d, num_in: %d, num_out: %d\n",m_node_ID(), _num_in(), _num_out());
  m_demand = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out);  // create m_demand, maybe use static_cast<TFlt*>
  memset(m_demand, 0x0, sizeof(TFlt) * _num_in * _num_out);  // initialize m_demand
  m_supply = (TFlt*) malloc(sizeof(TFlt) * _num_out);
  memset(m_supply, 0x0, sizeof(TFlt) * _num_out);
  m_veh_flow = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out);
  memset(m_veh_flow, 0x0, sizeof(TFlt) * _num_in * _num_out);
  m_veh_tomove = (TInt*) malloc(sizeof(TInt) * _num_in * _num_out);
  memset(m_veh_tomove, 0x0, sizeof(TInt) * _num_in * _num_out);
  return 0;
}

int MNM_Dnode_Inout::prepare_supplyANDdemand()
{
  // printf("MNM_Dnode_Inout::prepare_supplyANDdemand\n");
   /* calculate demand */
  size_t _offset = m_out_link_array.size();
  TInt _count;
  std::deque <MNM_Veh*>::iterator _veh_it;
  MNM_Dlink *_in_link, *_out_link;


  for (size_t i=0; i < m_in_link_array.size(); ++i){
    _in_link = m_in_link_array[i];
    for (_veh_it = _in_link -> m_finished_array.begin(); _veh_it != _in_link -> m_finished_array.end(); _veh_it++){
      if (std::find(m_out_link_array.begin(), m_out_link_array.end(), (*_veh_it) -> get_next_link()) == m_out_link_array.end()) {
        printf("Vehicle in the wrong node, no exit!\n");
        printf("Vehicle is on link %d, node %d, next link ID is: %d\n", _in_link -> m_link_ID(), m_node_ID(), (*_veh_it) -> get_next_link() -> m_link_ID());
        exit(-1);
      }
    }
    for (size_t j=0; j< m_out_link_array.size(); ++j){
      _out_link = m_out_link_array[j];
      // printf("Current out link is %d\n", _out_link -> m_link_ID);
      _count = 0;
      
      for (_veh_it = _in_link -> m_finished_array.begin(); _veh_it != _in_link -> m_finished_array.end(); _veh_it++){
        if ((*_veh_it) -> get_next_link() == _out_link) _count += 1;
      }
      m_demand[_offset*i + j] = TFlt(_count) / m_flow_scalar;
    }
  }
  // printf("Finished\n");
  /* calculated supply */
  for (size_t j=0; j< m_out_link_array.size(); ++j){
    _out_link = m_out_link_array[j];
    // printf("Get link s\n");
    // printf("The out link is %d\n", _out_link -> m_link_ID);
    m_supply[j] = _out_link -> get_link_supply();
    // printf(" get link s fin\n");
    // printf("Link %d, supply is %.4f\n", _out_link -> m_link_ID, m_supply[j]);
  } 

  return 0;
}

int MNM_Dnode_Inout::round_flow_to_vehicle()
{
  // printf("MNM_Dnode_Inout::round_flow_to_vehicle\n");
  // the rounding mechanism may cause the lack of vehicle in m_finished_array
  // but demand is always a integer and only supply can be float, so don't need to worry about it
  size_t _offset = m_out_link_array.size();
  MNM_Dlink *_out_link;
  TInt _to_move;
  size_t _rand_idx;
  for (size_t j=0; j< m_out_link_array.size(); ++j){
    _to_move = 0;
    _out_link = m_out_link_array[j];
    for (size_t i=0; i< m_in_link_array.size(); ++i){
      m_veh_tomove[i * _offset + j] = MNM_Ults::round(m_veh_flow[i * _offset + j] * m_flow_scalar);
      _to_move += m_veh_tomove[i * _offset + j];
      // printf("Rounding %d, %d the value %f to %d\n", i, j, m_veh_flow[i * _offset + j] * m_flow_scalar, m_veh_tomove[i * _offset + j]);
    }
    // printf("Going to loop %d vs supply %lf\n", _to_move, _out_link -> get_link_supply());
    while (TFlt(_to_move) > (_out_link -> get_link_supply() * m_flow_scalar)){
      _rand_idx = rand() % m_in_link_array.size();
      if (m_veh_tomove[_rand_idx * _offset + j] >= 1){
        m_veh_tomove[_rand_idx * _offset + j] -= 1;
        _to_move -= 1;
      }
    }
    // printf("Rounding %d, %d the value %f to %d\n", i, j, m_veh_flow[i * _offset + j] * m_flow_scalar, m_veh_tomove[i * _offset + j]);
  }
  return 0;
}


int MNM_Dnode_Inout::record_cumulative_curve(TInt timestamp)
{
  TInt _num_to_move;
  TInt _temp_sum;
  MNM_Dlink *_in_link, *_out_link;
  size_t _offset = m_out_link_array.size();

  for (size_t j=0; j<m_out_link_array.size(); ++j){
    _temp_sum = 0;
    _out_link = m_out_link_array[j];
    for (size_t i=0; i<m_in_link_array.size(); ++i) {
      _in_link = m_in_link_array[i];
      _num_to_move = m_veh_tomove[i * _offset + j];
      _temp_sum += _num_to_move;
    }
    if (_out_link -> m_N_out != nullptr && _temp_sum > 0) {
      // printf("record out link cc: link ID %d, time %d, value %f\n", _out_link -> m_link_ID(), timestamp()+1, (float) TFlt(_temp_sum)/m_flow_scalar);
      _out_link -> m_N_in -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum)/m_flow_scalar));
    }
  }

  for (size_t i=0; i<m_in_link_array.size(); ++i){
    _temp_sum = 0;
    _in_link = m_in_link_array[i];
    for (size_t j=0; j<m_out_link_array.size(); ++j) {
      _out_link = m_out_link_array[j];
      _num_to_move = m_veh_tomove[i * _offset + j];
      _temp_sum += _num_to_move;
    }
    if (_in_link -> m_N_in != nullptr && _temp_sum > 0) {
      // printf("record in link cc: link ID %d, time %d, value %f\n", _in_link -> m_link_ID(), timestamp()+1, (float) TFlt(_temp_sum)/m_flow_scalar);
      _in_link -> m_N_out -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum)/m_flow_scalar));
    }
  }
  return 0;
}


int MNM_Dnode_Inout::move_vehicle(TInt timestamp)
{
  // printf("MNM_Dnode_Inout::move_vehicle\n");
  MNM_Dlink *_in_link, *_out_link;
  MNM_Veh *_veh;
  size_t _offset = m_out_link_array.size();
  TInt _num_to_move;

  std::vector<size_t> _in_link_ind_array = std::vector<size_t>();
  for (size_t i=0; i<m_in_link_array.size(); ++i){
    _in_link_ind_array.push_back(i);
  }

  for (size_t j=0; j<m_out_link_array.size(); ++j){
    _out_link = m_out_link_array[j];

    // shuffle the in links, reserve the FIFO
    std::random_device rng; // random sequence
    std::shuffle(_in_link_ind_array.begin(), _in_link_ind_array.end(), rng);
    for (size_t i : _in_link_ind_array) {
    // for (size_t i=0; i<m_in_link_array.size(); ++i) {
      _in_link = m_in_link_array[i];
      _num_to_move = m_veh_tomove[i * _offset + j];
      // printf("In node %d, from link %d to link %d, %d to move\n", m_node_ID, _in_link ->m_link_ID, _out_link->m_link_ID, _num_to_move);
      // for (size_t k=0; k<_size; ++k){
      //   if (_num_to_move > 0){
      //     _veh = _in_link->m_finished_array[k];
      //     if (_veh -> get_next_link() == _out_link){
      //       _out_link ->m_incoming_array.push_back(_veh);
      //       _veh -> set_current_link(_out_link);
      //       // _in_link -> m_finished_array.pop_front();

      //       _num_to_move -= 1;            
      //     }
      //   }
      //   else{
      //     break; // break the inner most structure
      //   }
      // }
      auto _veh_it = _in_link->m_finished_array.begin();
      while (_veh_it != _in_link->m_finished_array.end()) {
        if (_num_to_move > 0) {
          _veh = *_veh_it;
          if (_veh -> get_next_link() == _out_link){
            _out_link ->m_incoming_array.push_back(_veh);
            _veh -> set_current_link(_out_link);
            _veh_it = _in_link->m_finished_array.erase(_veh_it); //c++ 11
            _num_to_move -= 1;
            if (_out_link -> m_N_in_tree != nullptr) {
              // printf("record out link cc tree: link ID %d, time %d, path id %d, assign interval %d\n", _out_link -> m_link_ID(), timestamp()+1, _veh -> m_path -> m_path_ID(), _veh -> m_assign_interval());
              _out_link -> m_N_in_tree -> add_flow(TFlt(timestamp + 1), TFlt(1)/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
            }
            if (_in_link -> m_N_out_tree != nullptr) {
              // printf("record in link cc tree: link ID %d, time %d, path id %d, assign interval %d\n", _in_link -> m_link_ID(), timestamp()+1, _veh -> m_path -> m_path_ID(), _veh -> m_assign_interval());
              _in_link -> m_N_out_tree -> add_flow(TFlt(timestamp + 1), TFlt(1)/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
            }
          }
          else {
            _veh_it++;
          }
        }
        else{
          break; // break the inner most structure
        }
      }
      if (_num_to_move != 0){
        printf("Something wrong during the vehicle moving, remaining to move %d\n", (int)_num_to_move);
        printf("The finished veh queue is now size %d\n", (int)_in_link->m_finished_array.size());
        printf("But it is heading to %d\n", (int)_in_link->m_finished_array.front() -> get_next_link() -> m_link_ID);
        exit(-1);
      }
    }
    // make the queue randomly perturbed, may not be true in signal controlled intersection, violate FIFO
    // random_shuffle(_out_link -> m_incoming_array.begin(), _out_link -> m_incoming_array.end());
  }
  _in_link_ind_array.clear();
  return 0;
}

void MNM_Dnode_Inout::print_info()
{
  ;
}

int MNM_Dnode_Inout::add_out_link(MNM_Dlink* out_link)
{
  m_out_link_array.push_back(out_link);
  return 0;
}

int MNM_Dnode_Inout::add_in_link(MNM_Dlink *in_link)
{
  m_in_link_array.push_back(in_link);
  return 0;
}


int MNM_Dnode_Inout::evolve(TInt timestamp)
{
  // printf("Inout node evolve\n");
  // printf("1\n");
  prepare_supplyANDdemand();
  // printf("2\n"); 
  compute_flow();
  // printf("3\n");
  round_flow_to_vehicle();
  // printf("4\n");
  record_cumulative_curve(timestamp);
  // printf("4.1\n");
  move_vehicle(timestamp);
  // printf("5\n");
  return 0;
}
/**************************************************************************
                              FWJ node
**************************************************************************/

MNM_Dnode_FWJ::MNM_Dnode_FWJ(TInt ID, TFlt flow_scalar)
  : MNM_Dnode_Inout::MNM_Dnode_Inout(ID, flow_scalar)
{
}

MNM_Dnode_FWJ::~MNM_Dnode_FWJ()
{
    ;
}

void MNM_Dnode_FWJ::print_info()
{
  ;
}

int MNM_Dnode_FWJ::compute_flow()
{
  // printf("MNM_Dnode_FWJ::compute_flow\n");
  size_t _offset = m_out_link_array.size();
  TFlt _sum_in_flow, _portion;
  for (size_t j=0; j< m_out_link_array.size(); ++j){
    _sum_in_flow = TFlt(0);
    for (size_t i=0; i< m_in_link_array.size(); ++i){
      _sum_in_flow += m_demand[i * _offset + j];
    }
    for (size_t i=0; i< m_in_link_array.size(); ++i){
      _portion = MNM_Ults::divide(m_demand[i * _offset + j], _sum_in_flow);
      // printf("Portion is %.4f, sum in flow is %.4f, demand is %.4f\n", _portion, _sum_in_flow, m_demand[i * _offset + j]);
      m_veh_flow[i * _offset + j] = MNM_Ults::min(m_demand[i * _offset + j], _portion * m_supply[j]);
      // printf("to link %d the flow is %.4f\n", m_out_link_array[j] -> m_link_ID, m_veh_flow[i * _offset + j]);
    }
  }
  return 0;
}

/**************************************************************************
                   General Road Junction node
**************************************************************************/
MNM_Dnode_GRJ::MNM_Dnode_GRJ(TInt ID, TFlt flow_scalar)
  : MNM_Dnode_Inout::MNM_Dnode_Inout(ID, flow_scalar)
{
  m_d_a = NULL;
  m_C_a = NULL;
}

MNM_Dnode_GRJ::~MNM_Dnode_GRJ()
{
  if (m_d_a != NULL) free(m_d_a);
  if (m_C_a != NULL) free(m_C_a);
}

int MNM_Dnode_GRJ::prepare_loading()
{
  MNM_Dnode_Inout::prepare_loading();
  TInt _num_in = m_in_link_array.size();
  m_d_a = (TFlt*) malloc(sizeof(TFlt) * _num_in);
  memset(m_d_a, 0x0, sizeof(TFlt) * _num_in);
  m_C_a = (TFlt*) malloc(sizeof(TFlt) * _num_in);
  memset(m_C_a, 0x0, sizeof(TFlt) * _num_in);
  return 0;
}

void MNM_Dnode_GRJ::print_info()
{
  ;
}

int MNM_Dnode_GRJ::compute_flow()
{
  if (m_in_link_array.size() == 0 || m_out_link_array.size() == 0){
    return 0;
  }
  // printf("MNM_Dnode_GRJ ID %d::compute_flow()\n", m_node_ID());
  TFlt _theta = get_theta();
  size_t _offset = m_out_link_array.size();
  TFlt _f_a;
  for (size_t i=0; i< m_in_link_array.size(); ++i){
    // printf("3\n");
    _f_a = MNM_Ults::min(m_d_a[i], _theta * m_C_a[i]);
    // printf("f_a is %lf\n", _f_a);
    for (size_t j=0; j< m_out_link_array.size(); ++j){
      m_veh_flow[i * _offset + j] = _f_a * MNM_Ults::divide(m_demand[i * _offset + j], m_supply[j]);
      // printf("to link %d the flow is %.4f\n", m_out_link_array[j] -> m_link_ID, m_veh_flow[i * _offset + j]);
    }
  }
  // printf("return\n");
  return 0;
}

int MNM_Dnode_GRJ::prepare_outflux()
{
  MNM_Dlink *_link;
  for (size_t i=0; i<m_in_link_array.size(); ++i){
    _link = m_in_link_array[i];
    m_d_a[i] = TFlt(_link -> m_finished_array.size()) / m_flow_scalar;
    m_C_a[i] = MNM_Ults::max(m_d_a[i], _link -> get_link_supply());
    // printf("mda is %lf, mca is %lf\n", m_d_a[i](), m_C_a[i]());
  }
  return 0;
}


std::vector<int> MNM_Dnode_GRJ::getOnLocations(int a) 
{
  std::vector<int> result;
  int place = 0;
  while (a != 0) {
    if (a & 1) {
      result.push_back(place);
    }
    ++place;
    a >>= 1;
  }
  return result;
}

template<typename T>
std::vector<std::vector<T> > MNM_Dnode_GRJ::powerSet(const std::vector<T>& set) {
  std::vector<std::vector<T> > result;
  int numPowerSets = static_cast<int>(pow(2.0, static_cast<double>(set.size())));
  for (int i = 0; i < numPowerSets; ++i) {
    std::vector<int> onLocations = getOnLocations(i);
    std::vector<T> subSet;
    for (size_t j = 0; j < onLocations.size(); ++j) {
      subSet.push_back(set.at(onLocations.at(j)));
    }
    result.push_back(subSet);
  }
  return result;
}

TFlt MNM_Dnode_GRJ::get_theta()
{
  if (m_pow.size() == 0) {
    m_pow = powerSet(m_in_link_array);
  }
  // printf("s1\n");
  prepare_outflux();
  size_t _offset = m_out_link_array.size();
  // part 1: max d_a/C_a
  std::vector<TFlt> _temp1 = std::vector<TFlt>();
  for (size_t i=0; i< m_in_link_array.size(); ++i){
    _temp1.push_back(MNM_Ults::divide(m_d_a[i], m_C_a[i]));
  }

  TFlt _e1 = *max_element(_temp1.begin(), _temp1.end());

  
  // printf("s2\n");
  //part 2: min max
  std::vector<TFlt> _temp2 = std::vector<TFlt>();
  for (size_t j=0; j<m_out_link_array.size(); ++j){
    std::vector<TFlt> _temp3 = std::vector<TFlt>();
    for (std::vector<MNM_Dlink*> v : m_pow){
      if (v.size() == 0) continue;
      // for (MNM_Dlink *a : v){
      //   printf("a -> m_link_ID is %d\n", a->m_link_ID());
      // }
      TFlt _up = TFlt(0);
      TFlt _down = TFlt(0);
      for (size_t i=0; i< m_in_link_array.size(); ++i){
        if (std::find(v.begin(), v.end(), m_in_link_array[i]) != v.end()){
          _down += m_C_a[i] * MNM_Ults::divide(m_demand[i * _offset + j], m_supply[j]);
        }
        else{
          _up += m_d_a[i] * MNM_Ults::divide(m_demand[i * _offset + j], m_supply[j]);
        }
      }
      // printf("for j is %d, the value of cal is %lf, s is %lf, up is %lf, down is %lf\n", 
          // j, (m_supply[j] - _up)/_down ,m_supply[j](), _up(), _down());
      _temp3.push_back((m_supply[j] - _up)/_down);
    }
    _temp2.push_back(*max_element(_temp3.begin(), _temp3.end()));
  }
  // printf("s3\n");
  // printf("_temp2 size %d\n", _temp2.size());
  TFlt _e2 = *max_element(_temp2.begin(), _temp2.end());


  // printf("e1 is %lf, e2 is %lf\n", _e1(), _e2());
  // total
  TFlt _theta = MNM_Ults::min(_e1, _e2);
  // printf("return\n");
  return _theta;
}