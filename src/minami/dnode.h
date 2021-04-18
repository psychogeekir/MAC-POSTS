#ifndef DNODE_H
#define DNODE_H

#include "Snap.h"
#include "dlink.h"
#include "od.h"
#include "vehicle.h"
#include "ults.h"

#include <vector>
#include <deque>
#include <unordered_map>

class MNM_Dlink;
class MNM_Destination;
class MNM_Origin;
class MNM_Veh;

class MNM_Dnode
{
public:
  MNM_Dnode(TInt ID, TFlt flow_scalar);
  virtual ~MNM_Dnode();
  TInt m_node_ID;
  virtual int evolve(TInt timestamp){return 0;};
  virtual void print_info(){};
  virtual int prepare_loading(){return 0;};
  virtual int add_out_link(MNM_Dlink* out_link){printf("Error!\n"); return 0;};
  virtual int add_in_link(MNM_Dlink* in_link){printf("Error!\n"); return 0;};
  std::vector<MNM_Dlink*> m_out_link_array;
  std::vector<MNM_Dlink*> m_in_link_array;
protected:
  TFlt m_flow_scalar;
};

class MNM_DMOND : public MNM_Dnode
{
public:
  MNM_DMOND(TInt ID, TFlt flow_scalar);
  virtual ~MNM_DMOND() override;
  virtual int evolve(TInt timestamp) override;
  virtual void print_info() override;
  virtual int add_out_link(MNM_Dlink* out_link) override;
  int hook_up_origin(MNM_Origin *origin);
  std::deque<MNM_Veh *> m_in_veh_queue;
// private:
  MNM_Origin *m_origin;
  std::unordered_map<MNM_Dlink*, TInt> m_out_volume;
};

class MNM_DMDND : public MNM_Dnode
{
public:
  MNM_DMDND(TInt ID, TFlt flow_scalar);
  virtual ~MNM_DMDND() override;
  virtual int evolve(TInt timestamp) override;
  virtual void print_info() override;
  virtual int add_in_link(MNM_Dlink *link) override;
  int hook_up_destination(MNM_Destination *dest);
  std::deque<MNM_Veh *> m_out_veh_queue;
// private:
  MNM_Destination *m_dest;
};

/**************************************************************************
                              In-out node
**************************************************************************/

class MNM_Dnode_Inout : public MNM_Dnode
{
public:
  MNM_Dnode_Inout(TInt ID, TFlt flow_scalar);
  virtual ~MNM_Dnode_Inout() override;
  virtual int evolve(TInt timestamp) override;
  virtual void print_info() override;
  virtual int prepare_loading() override;
  virtual int add_out_link(MNM_Dlink* out_link) override;
  virtual int add_in_link(MNM_Dlink* in_link) override;
protected:
  int prepare_supplyANDdemand();
  virtual int compute_flow(){return 0;};
  int round_flow_to_vehicle();
  int move_vehicle(TInt timestamp);
  int record_cumulative_curve(TInt timestamp);
  TFlt *m_demand; //2d array
  TFlt *m_supply; //1d array
  TFlt *m_veh_flow; //2d array
  TInt *m_veh_tomove; //2d array
};

/**************************************************************************
                              FWJ node
**************************************************************************/
class MNM_Dnode_FWJ : public MNM_Dnode_Inout
{
public:
  MNM_Dnode_FWJ(TInt ID, TFlt flow_scalar);
  virtual ~MNM_Dnode_FWJ() override;
  virtual void print_info() override;
  virtual int compute_flow() override;
};

/**************************************************************************
                   General Road Junction node
**************************************************************************/
class MNM_Dnode_GRJ : public MNM_Dnode_Inout
{
public:
  MNM_Dnode_GRJ(TInt ID, TFlt flow_scalar);
  virtual ~MNM_Dnode_GRJ() override;
  virtual void print_info() override;
  virtual int compute_flow() override;
  virtual int prepare_loading() override;
private:
  std::vector<std::vector<MNM_Dlink*>> m_pow;
  TFlt get_theta();
  int prepare_outflux();
  TFlt *m_d_a; //1d array
  TFlt *m_C_a; //1d array
  template<typename T> std::vector<std::vector<T> > powerSet(const std::vector<T>& set);
  std::vector<int> getOnLocations(int a);
};
#endif
