#include "limits.h"
#include "multiclass.h"

#include "limits.h"

#include <algorithm>
#include <fstream>
#include <iostream>

/******************************************************************************************************************
*******************************************************************************************************************
												Link Models
*******************************************************************************************************************
******************************************************************************************************************/

MNM_Dlink_Multiclass::MNM_Dlink_Multiclass(TInt ID,
										TInt number_of_lane,
										TFlt length, // meters
										TFlt ffs_car, // Free-flow speed (m/s)
										TFlt ffs_truck)
	: MNM_Dlink::MNM_Dlink(ID, number_of_lane, length, ffs_car) // Note: although m_ffs is not used in child class, let it be ffs_car
{
	m_ffs_car = ffs_car;
	m_ffs_truck = ffs_truck;

	m_N_in_car = nullptr;
	m_N_out_car = nullptr;
	m_N_in_truck = nullptr;
	m_N_out_truck = nullptr;

	m_N_in_tree_car = nullptr;
	m_N_out_tree_car = nullptr;
	m_N_in_tree_truck = nullptr;
	m_N_out_tree_truck = nullptr;

	// average waiting time per vehicle = tot_wait_time/(tot_num_car + tot_num_truck)
	m_tot_wait_time_at_intersection = 0; // seconds

	// flag of spill back on this link
	m_spill_back = false; // if spill back happens during simulation, then set to true

	install_cumulative_curve_multiclass();

	// !!! Close cc_tree if only doing loading to save a lot of memory !!!
//	install_cumulative_curve_tree_multiclass();

}

MNM_Dlink_Multiclass::~MNM_Dlink_Multiclass()
{
	if (m_N_out_car != nullptr) delete m_N_out_car;
  	if (m_N_in_car != nullptr) delete m_N_in_car;
  	if (m_N_out_truck != nullptr) delete m_N_out_truck;
  	if (m_N_in_truck != nullptr) delete m_N_in_truck;
  	if (m_N_out_tree_car != nullptr) delete m_N_out_tree_car;
  	if (m_N_in_tree_car != nullptr) delete m_N_in_tree_car;
  	if (m_N_out_tree_truck != nullptr) delete m_N_out_tree_truck;
  	if (m_N_in_tree_truck != nullptr) delete m_N_in_tree_truck;

}

int MNM_Dlink_Multiclass::install_cumulative_curve_multiclass()
{
	if (m_N_out_car != nullptr) delete m_N_out_car;
  	if (m_N_in_car != nullptr) delete m_N_in_car;
  	if (m_N_out_truck != nullptr) delete m_N_out_truck;
  	if (m_N_in_truck != nullptr) delete m_N_in_truck;
	m_N_in_car = new MNM_Cumulative_Curve();
  	m_N_out_car = new MNM_Cumulative_Curve();
  	m_N_in_truck = new MNM_Cumulative_Curve();
  	m_N_out_truck = new MNM_Cumulative_Curve();
  	m_N_in_car -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
  	m_N_out_car -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
  	m_N_in_truck -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
  	m_N_out_truck -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
  	return 0;
}

int MNM_Dlink_Multiclass::install_cumulative_curve_tree_multiclass()
{
	if (m_N_out_tree_car != nullptr) delete m_N_out_tree_car;
  	if (m_N_in_tree_car != nullptr) delete m_N_in_tree_car;
  	if (m_N_out_tree_truck != nullptr) delete m_N_out_tree_truck;
  	if (m_N_in_tree_truck != nullptr) delete m_N_in_tree_truck;


  	// !!! Close all cc_tree if only doing loading to save a lot of memory !!!
	m_N_in_tree_car = new MNM_Tree_Cumulative_Curve();
  	// m_N_out_tree_car = new MNM_Tree_Cumulative_Curve();
	m_N_in_tree_truck = new MNM_Tree_Cumulative_Curve();
  	// m_N_out_tree_truck = new MNM_Tree_Cumulative_Curve();

  	return 0;
}

TFlt MNM_Dlink_Multiclass::get_link_freeflow_tt_car()
{
	return m_length/m_ffs_car;  // seconds, absolute tt
}

TFlt MNM_Dlink_Multiclass::get_link_freeflow_tt_truck()
{
	return m_length/m_ffs_truck;  // seconds, absolute tt
}


/*************************************************************************					
						Multiclass CTM Functions
			(currently only for car & truck two classes)
	(see: Z. (Sean) Qian et al./Trans. Res. Part B 99 (2017) 183-204)			
**************************************************************************/
MNM_Dlink_Ctm_Multiclass::MNM_Dlink_Ctm_Multiclass(TInt ID,
												   TInt number_of_lane,
												   TFlt length, // (m)
												   TFlt lane_hold_cap_car, // Jam density (veh/m/lane)
												   TFlt lane_hold_cap_truck,
												   TFlt lane_flow_cap_car, // Max flux (veh/s/lane)
												   TFlt lane_flow_cap_truck,
												   TFlt ffs_car, // Free-flow speed (m/s)
												   TFlt ffs_truck, 
												   TFlt unit_time, // (s)
												   TFlt veh_convert_factor, // 1 * truck = c * private cars 
												   							// when compute node demand
												   TFlt flow_scalar) // flow_scalar can be 2.0, 5.0, 10.0, etc.
	: MNM_Dlink_Multiclass::MNM_Dlink_Multiclass(ID, number_of_lane, length, ffs_car, ffs_truck)
{
    m_link_type = MNM_TYPE_CTM_MULTICLASS;
	// Jam density for private cars and trucks cannot be negative
	if ((lane_hold_cap_car < 0) || (lane_hold_cap_truck < 0)){
		printf("lane_hold_cap can't be negative, current link ID is %d\n", m_link_ID());
		exit(-1);
	}
	// Jam density for private cars cannot be too large
	if (lane_hold_cap_car > TFlt(400) / TFlt(1600)){
		// "lane_hold_cap is too large, set to 300 veh/mile
		lane_hold_cap_car = TFlt(400) / TFlt(1600);
	}
	// Jam density for trucks cannot be too large
	if (lane_hold_cap_truck > TFlt(400) / TFlt(1600)){
		// "lane_hold_cap is too large, set to 300 veh/mile
		lane_hold_cap_truck = TFlt(400) / TFlt(1600);
	}

	// Maximum flux for private cars and trucks cannot be negative
	if ((lane_flow_cap_car < 0) || (lane_flow_cap_truck < 0)){
		printf("lane_flow_cap can't be less than zero, current link ID is %d\n", m_link_ID());
		exit(-1);
	}
	// Maximum flux for private cars cannot be too large
	if (lane_flow_cap_car > TFlt(3500) / TFlt(3600)){
		// lane_flow_cap is too large, set to 3500 veh/hour
		lane_flow_cap_car = TFlt(3500) / TFlt(3600);
	}
	// Maximum flux for trucks cannot be too large
	if (lane_flow_cap_truck > TFlt(3500) / TFlt(3600)){
		// lane_flow_cap is too large, set to 3500 veh/hour
		lane_flow_cap_truck = TFlt(3500) / TFlt(3600);
	}

	if ((ffs_car < 0) || (ffs_truck < 0)){
		printf("free-flow speed can't be less than zero, current link ID is %d\n", m_link_ID());
		exit(-1);
	}

	if (veh_convert_factor < 1){
		printf("veh_convert_factor can't be less than 1, current link ID is %d\n", m_link_ID());
		exit(-1);
	}

	if (flow_scalar < 1){
		printf("flow_scalar can't be less than 1, current link ID is %d\n", m_link_ID());
		exit(-1);
	}

	if (unit_time <= 0){
		printf("unit_time should be positive, current link ID is %d\n", m_link_ID());
		exit(-1);
	}
	m_unit_time = unit_time;
	m_lane_flow_cap_car = lane_flow_cap_car;
	m_lane_flow_cap_truck = lane_flow_cap_truck;	
	m_lane_hold_cap_car = lane_hold_cap_car;
	m_lane_hold_cap_truck = lane_hold_cap_truck;
	m_veh_convert_factor = veh_convert_factor;
	m_flow_scalar = flow_scalar;

	m_cell_array = std::vector<Ctm_Cell_Multiclass*>();

	// Note m_ffs_car > m_ffs_truck, use ffs_car to define the standard cell length
	TFlt _std_cell_length = m_ffs_car * unit_time;
	m_num_cells = TInt(floor(m_length / _std_cell_length));
	if (m_num_cells == 0){
		m_num_cells = 1;
		m_length = _std_cell_length;
	}
	TFlt _last_cell_length = m_length - TFlt(m_num_cells - 1) * _std_cell_length;

	m_lane_critical_density_car = m_lane_flow_cap_car / m_ffs_car;
	m_lane_critical_density_truck = m_lane_flow_cap_truck / m_ffs_truck;

	if (m_lane_hold_cap_car <= m_lane_critical_density_car){
		printf("Wrong private car parameters, current link ID is %d\n", m_link_ID());
		exit(-1);
	}
	m_wave_speed_car = m_lane_flow_cap_car / (m_lane_hold_cap_car - m_lane_critical_density_car);

	if (m_lane_hold_cap_truck <= m_lane_critical_density_truck){
		printf("Wrong truck parameters, current link ID is %d\n", m_link_ID());
		exit(-1);
	}
	m_wave_speed_truck = m_lane_flow_cap_truck / (m_lane_hold_cap_truck - m_lane_critical_density_truck);

	// see the reference paper for definition
	// m_lane_rho_1_N > m_lane_critical_density_car and m_lane_critical_density_truck
	m_lane_rho_1_N = m_lane_hold_cap_car * (m_wave_speed_car / (m_ffs_truck + m_wave_speed_car));

	init_cell_array(unit_time, _std_cell_length, _last_cell_length);
}

MNM_Dlink_Ctm_Multiclass::~MNM_Dlink_Ctm_Multiclass()
{
	for (Ctm_Cell_Multiclass* _cell : m_cell_array){
		delete _cell;
	}
	m_cell_array.clear();
}

int MNM_Dlink_Ctm_Multiclass::move_veh_queue(std::deque<MNM_Veh*> *from_queue,
                                		std::deque<MNM_Veh*> *to_queue, 
                                		TInt number_tomove)
{
	MNM_Veh* _veh;
	MNM_Veh_Multiclass* _veh_multiclass;
	for (int i = 0; i < number_tomove; ++i) {
		_veh = from_queue -> front();
		from_queue -> pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);
		// update the vehicle position on current link. 0: at the beginning, 1: at the end.
		_veh_multiclass -> m_visual_position_on_link += float(1)/float(m_num_cells);
		if (_veh_multiclass -> m_visual_position_on_link > 0.99) 
			_veh_multiclass -> m_visual_position_on_link = 0.99;
		to_queue -> push_back(_veh);
	}
	return 0;
}

int MNM_Dlink_Ctm_Multiclass::init_cell_array(TFlt unit_time, 
											  TFlt std_cell_length, 
											  TFlt last_cell_length)
{
	// All previous cells
	Ctm_Cell_Multiclass *cell = NULL;
	for (int i = 0; i < m_num_cells - 1; ++i){
		cell = new Ctm_Cell_Multiclass(TInt(i),
                                       std_cell_length,
									   unit_time,
									   // Convert lane parameters to cell (link) parameters by multiplying # of lanes
									   TFlt(m_number_of_lane) * m_lane_hold_cap_car,
									   TFlt(m_number_of_lane) * m_lane_hold_cap_truck,
									   TFlt(m_number_of_lane) * m_lane_critical_density_car,
									   TFlt(m_number_of_lane) * m_lane_critical_density_truck,
									   TFlt(m_number_of_lane) * m_lane_rho_1_N,
									   TFlt(m_number_of_lane) * m_lane_flow_cap_car,
									   TFlt(m_number_of_lane) * m_lane_flow_cap_truck,
									   m_ffs_car,
									   m_ffs_truck,
									   m_wave_speed_car,
									   m_wave_speed_truck,
									   m_flow_scalar);
		if (cell == NULL) {
			printf("Fail to initialize some standard cell.\n");
			exit(-1);
		}
		m_cell_array.push_back(cell);
	}

	// The last cell
	// last cell must exist as long as link length > 0, see definition above
	if (m_length > 0.0) {
		cell = new Ctm_Cell_Multiclass(m_num_cells - 1,
		                               last_cell_length, // Note last cell length is longer but < 2X
									   unit_time,
									   TFlt(m_number_of_lane) * m_lane_hold_cap_car,
									   TFlt(m_number_of_lane) * m_lane_hold_cap_truck,
									   TFlt(m_number_of_lane) * m_lane_critical_density_car,
									   TFlt(m_number_of_lane) * m_lane_critical_density_truck,
									   TFlt(m_number_of_lane) * m_lane_rho_1_N,
									   TFlt(m_number_of_lane) * m_lane_flow_cap_car,
									   TFlt(m_number_of_lane) * m_lane_flow_cap_truck,
									   m_ffs_car,
									   m_ffs_truck,
									   m_wave_speed_car,
									   m_wave_speed_truck,
									   m_flow_scalar);
		if (cell == NULL) {
			printf("Fail to initialize the last cell.\n");
			exit(-1);
		}
		m_cell_array.push_back(cell);
	}

	// compress the cell_array to reduce space
	m_cell_array.shrink_to_fit();

	return 0;
}

void MNM_Dlink_Ctm_Multiclass::print_info()
{
	printf("Total number of cell: \t%d\n Flow scalar: \t%.4f\n", int(m_num_cells), double(m_flow_scalar));

	printf("Car volume for each cell is:\n");
	for (int i = 0; i < m_num_cells - 1; ++i){
		printf("%d, ", int(m_cell_array[i] -> m_volume_car));
	}
	printf("%d\n", int(m_cell_array[m_num_cells - 1] -> m_volume_car));

	printf("Truck volume for each cell is:\n");
	for (int i = 0; i < m_num_cells - 1; ++i){
		printf("%d, ", int(m_cell_array[i] -> m_volume_truck));
	}
	printf("%d\n", int(m_cell_array[m_num_cells - 1] -> m_volume_truck));
}

int MNM_Dlink_Ctm_Multiclass::update_out_veh()
{
	TFlt _temp_out_flux_car, _supply_car, _demand_car;
	TFlt _temp_out_flux_truck, _supply_truck, _demand_truck;

	// TInt _output_link = 16815;
	// if (m_link_ID == _output_link)
	// 	printf("Link %d to be moved: ", int(m_link_ID));
	// no update is needed if only one cell
	if (m_num_cells > 1){
		for (int i = 0; i < m_num_cells - 1; ++i){
			// car, veh_type = TInt(0)
			_demand_car = m_cell_array[i] -> get_perceived_demand(TInt(0));
			_supply_car = m_cell_array[i + 1] -> get_perceived_supply(TInt(0));
			_temp_out_flux_car = m_cell_array[i] -> m_space_fraction_car * MNM_Ults::min(_demand_car, _supply_car);
			m_cell_array[i] -> m_out_veh_car = MNM_Ults::round(_temp_out_flux_car * m_flow_scalar);

			// truck, veh_type = TInt(1)
			_demand_truck = m_cell_array[i] -> get_perceived_demand(TInt(1));
			_supply_truck = m_cell_array[i + 1] -> get_perceived_supply(TInt(1));
			_temp_out_flux_truck = m_cell_array[i] -> m_space_fraction_truck * MNM_Ults::min(_demand_truck, _supply_truck);
			m_cell_array[i] -> m_out_veh_truck = MNM_Ults::round(_temp_out_flux_truck * m_flow_scalar);

			// if (m_link_ID == _output_link){
			// 	printf("(%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %d; %.2f, %.2f, %.2f, %.2f, %d) ", 
			// 		_demand_car, m_cell_array[i] -> m_cell_length, m_cell_array[i] -> m_flow_cap_car, m_cell_array[i] -> m_ffs_car, _supply_car, _temp_out_flux_car, m_cell_array[i] -> m_space_fraction_car, m_cell_array[i] -> m_out_veh_car,
			// 		_demand_truck, _supply_truck, _temp_out_flux_truck, m_cell_array[i] -> m_space_fraction_truck, m_cell_array[i] -> m_out_veh_truck);
			// 	printf("(%d, %d) ",  int(m_cell_array[i] -> m_out_veh_car), int(m_cell_array[i] -> m_out_veh_truck));
			// }
		}
	}
	m_cell_array[m_num_cells - 1] -> m_out_veh_car = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size();
	m_cell_array[m_num_cells - 1] -> m_out_veh_truck = m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size();
	// if (m_link_ID == _output_link) printf("(%d, %d)\n", int(m_cell_array[m_num_cells - 1] -> m_out_veh_car), int(m_cell_array[m_num_cells - 1] -> m_out_veh_truck));
	return 0;
}

int MNM_Dlink_Ctm_Multiclass::evolve(TInt timestamp)
{
	std::deque<MNM_Veh*>::iterator _veh_it;
	TInt _count_car = 0;
	TInt _count_truck = 0;
	TInt _count_tot_vehs = 0;

	// TInt _output_link = 16815;
	// if (m_link_ID == _output_link){
	// 	printf("Link %d volume before: ", int(m_link_ID));
	// 	if (m_num_cells > 1){
	// 		for (int i = 0; i < m_num_cells - 1; ++i)
	// 		{
	// 			printf("(%d, %d) ", int(m_cell_array[i] -> m_veh_queue_car.size()), int(m_cell_array[i] -> m_veh_queue_truck.size()));
	// 		}
	// 	}
	// 	// m_class: 0 - private car, 1 - truck
	// 	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
	// 		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
	// 		if (_veh -> m_class == 0) _count_car += 1;
	// 		if (_veh -> m_class == 1) _count_truck += 1;
	// 	}
	// 	printf("(%d, %d; %d, %d)\n", int(m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size()), 
	// 		int(m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size()), int(_count_car), int(_count_truck));
	// }
	
	/* update volume */
	update_out_veh();
	
	TInt _num_veh_tomove_car, _num_veh_tomove_truck;
	/* previous cells */
	if (m_num_cells > 1){
		for (int i = 0; i < m_num_cells - 1; ++i)
		{
			// Car
			_num_veh_tomove_car = m_cell_array[i] -> m_out_veh_car;
			move_veh_queue( &(m_cell_array[i] -> m_veh_queue_car),
							&(m_cell_array[i+1] -> m_veh_queue_car),
							_num_veh_tomove_car);
			// Truck
			_num_veh_tomove_truck = m_cell_array[i] -> m_out_veh_truck;
			move_veh_queue( &(m_cell_array[i] -> m_veh_queue_truck),
							&(m_cell_array[i+1] -> m_veh_queue_truck),
							_num_veh_tomove_truck);
		}
	}

	/* last cell */
	move_last_cell();
	m_tot_wait_time_at_intersection += TFlt(m_finished_array.size())/m_flow_scalar * m_unit_time;

	// if (m_link_ID == _output_link)
	// 	printf("Link %d volume after: ", int(m_link_ID));

	/* update volume */
	if (m_num_cells > 1){
		for (int i = 0; i < m_num_cells - 1; ++i)
		{
			m_cell_array[i] -> m_volume_car = m_cell_array[i] -> m_veh_queue_car.size();
			m_cell_array[i] -> m_volume_truck = m_cell_array[i] -> m_veh_queue_truck.size();
			// Update perceived density of the i-th cell
			m_cell_array[i] -> update_perceived_density();
			// if (m_link_ID == _output_link)
			// 	printf("(%d, %d) ", int(m_cell_array[i] -> m_volume_car), int(m_cell_array[i] -> m_volume_truck));
		}
	}

	_count_car = 0;
	_count_truck = 0;
	// m_class: 0 - private car, 1 - truck
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 0) _count_car += 1;
		if (_veh -> m_class == 1) _count_truck += 1;
	}
	m_cell_array[m_num_cells - 1] -> m_volume_car = 
		m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size() + _count_car;
	m_cell_array[m_num_cells - 1] -> m_volume_truck = 
		m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size() + _count_truck;	
	m_cell_array[m_num_cells - 1] -> update_perceived_density();

	// if (m_link_ID == _output_link){
	// 	printf("(%d, %d; %d, %d)\n", int(m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size()), 
	// 		int(m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size()), int(_count_car), int(_count_truck));
	// 	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
	// 		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
	// 		printf("(%d-%d: %d->%d) ", int(_veh -> m_veh_ID), int(_veh -> m_class), int(_veh -> get_current_link() -> m_link_ID), 
	// 			int(_veh -> get_next_link() -> m_link_ID));
	// 	}
	// 	printf("\n");
	// }

	/* compute total volume of link, check if spill back */
	_count_tot_vehs = 0;
	for (int i = 0; i <= m_num_cells - 1; ++i)
	{
		_count_tot_vehs += m_cell_array[i] -> m_volume_car;
		_count_tot_vehs += m_cell_array[i] -> m_volume_truck * m_veh_convert_factor;
	}
	if (TFlt(_count_tot_vehs)/m_flow_scalar/m_length > m_lane_hold_cap_car * m_number_of_lane){
		m_spill_back = true;
	}
	return 0;
}


int MNM_Dlink_Ctm_Multiclass::move_last_cell() 
{
	TInt _num_veh_tomove_car = m_cell_array[m_num_cells - 1] -> m_out_veh_car;
	TInt _num_veh_tomove_truck = m_cell_array[m_num_cells - 1] -> m_out_veh_truck;
	TFlt _pstar = TFlt(_num_veh_tomove_car)/TFlt(_num_veh_tomove_car + _num_veh_tomove_truck);
	MNM_Veh* _veh;
	TFlt _r;
	while ((_num_veh_tomove_car > 0) || (_num_veh_tomove_truck > 0)){
		_r = MNM_Ults::rand_flt();
		// probability = _pstar to move a car
		if (_r < _pstar){
			// still has car to move
			if (_num_veh_tomove_car > 0){
				_veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.front();
				m_cell_array[m_num_cells - 1] -> m_veh_queue_car.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_car--;
			}
			// no car to move, move a truck
			else {
				_veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.front();
				m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_truck--;
			}
		}
		// probability = 1 - _pstar to move a truck
		else {
			// still has truck to move
			if (_num_veh_tomove_truck > 0){
				_veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.front();
				m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_truck--;
			}
			// no truck to move, move a car
			else {
				_veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.front();
				m_cell_array[m_num_cells - 1] -> m_veh_queue_car.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_car--;
			}
		}
	}
	return 0;
}

// TFlt MNM_Dlink_Ctm_Multiclass::get_link_supply()
// {
// 	m_cell_array[0] -> update_perceived_density();
// 	TFlt _supply_car = m_cell_array[0] -> m_space_fraction_car * std::min(m_cell_array[0] -> m_flow_cap_car, 
// 					TFlt(m_wave_speed_car * (m_cell_array[0] -> m_hold_cap_car - m_cell_array[0] -> m_perceived_density_car)));
// 	TFlt _supply_truck = m_cell_array[0] -> m_space_fraction_truck * std::min(m_cell_array[0] -> m_flow_cap_truck, 
// 					TFlt(m_wave_speed_truck * (m_cell_array[0] -> m_hold_cap_truck - m_cell_array[0] -> m_perceived_density_truck)));
// 	TFlt _supply = std::max(TFlt(0.0), _supply_car) + m_veh_convert_factor * std::max(TFlt(0.0), _supply_truck);

// 	return _supply * (m_cell_array[0] -> m_unit_time);
// }

TFlt MNM_Dlink_Ctm_Multiclass::get_link_supply()
{
	TFlt _real_volume_both = ( TFlt(m_cell_array[0] -> m_volume_truck) * m_veh_convert_factor + 
							   TFlt(m_cell_array[0] -> m_volume_car) ) / m_flow_scalar;
	// TFlt _real_volume_both = ( TFlt(m_cell_array[0] -> m_volume_truck) * 1 + 
	// 						   TFlt(m_cell_array[0] -> m_volume_car) ) / m_flow_scalar;

	// m_cell_length can't be 0 according to implementation above
	TFlt _density = _real_volume_both / (m_cell_array[0] -> m_cell_length);
	double _tmp = std::min(double(m_cell_array[0] -> m_flow_cap_car), m_wave_speed_car * (m_cell_array[0] -> m_hold_cap_car - _density));

	// only use when network is too large and complex and no other ways solving gridlock.
	_tmp = std::max(_tmp, m_wave_speed_car * 0.25 * (m_cell_array[0] -> m_hold_cap_car - _density));

	return std::max(0.0, _tmp) * (m_cell_array[0] -> m_unit_time);
}
	
int MNM_Dlink_Ctm_Multiclass::clear_incoming_array(TInt timestamp)
{
	// if (get_link_supply() * m_flow_scalar < m_incoming_array.size()){
	// 	printf("Wrong incoming array size\n");
	// 	exit(-1);
	// }
	
	//if (m_link_ID == 3963)
		//printf("Link %d incoming array size: %d\n", int(m_link_ID), int(m_incoming_array.size()));

	MNM_Veh_Multiclass* _veh;
	size_t _cur_size = m_incoming_array.size();
	for (size_t i = 0; i < _cur_size; ++i) {
		_veh = dynamic_cast<MNM_Veh_Multiclass *>(m_incoming_array.front());
		m_incoming_array.pop_front();
		if (_veh -> m_class == TInt(0)) {
			// printf("car\n");
			m_cell_array[0] -> m_veh_queue_car.push_back(_veh);
		}
		else {
			// printf("truck\n");
			m_cell_array[0] -> m_veh_queue_truck.push_back(_veh);
		}
		_veh -> m_visual_position_on_link = float(1)/float(m_num_cells)/float(2); // initial position at first cell
	}
	m_cell_array[0] -> m_volume_car = m_cell_array[0] -> m_veh_queue_car.size();
	m_cell_array[0] -> m_volume_truck = m_cell_array[0] -> m_veh_queue_truck.size();
	m_cell_array[0] -> update_perceived_density();
	
	return 0;
}

TFlt MNM_Dlink_Ctm_Multiclass::get_link_flow_car()
{
	TInt _total_volume_car = 0;
	for (int i = 0; i < m_num_cells; ++i){
		_total_volume_car += m_cell_array[i] -> m_volume_car;
	}
	std::deque<MNM_Veh*>::iterator _veh_it;
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 0) _total_volume_car += 1;
	}
	return TFlt(_total_volume_car) / m_flow_scalar;
}

TFlt MNM_Dlink_Ctm_Multiclass::get_link_flow_truck()
{
	TInt _total_volume_truck = 0;
	for (int i = 0; i < m_num_cells; ++i){
		_total_volume_truck += m_cell_array[i] -> m_volume_truck;
	}
	std::deque<MNM_Veh*>::iterator _veh_it;
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 1) _total_volume_truck += 1;
	}
	return TFlt(_total_volume_truck) / m_flow_scalar;
}

TFlt MNM_Dlink_Ctm_Multiclass::get_link_flow()
{
	// For get_link_tt in adaptive routing
	TInt _total_volume_car = 0;
	TInt _total_volume_truck = 0;
	for (int i = 0; i < m_num_cells; ++i){
		_total_volume_car += m_cell_array[i] -> m_volume_car;
		_total_volume_truck += m_cell_array[i] -> m_volume_truck;
	}
	return TFlt(_total_volume_car + _total_volume_truck + m_finished_array.size()) / m_flow_scalar;
}

TFlt MNM_Dlink_Ctm_Multiclass::get_link_tt()
{
	// For adaptive routing and emissions, need modification for multiclass cases
	TFlt _cost, _spd;
	// get the density in veh/mile/lane
	TFlt _rho = get_link_flow()/m_number_of_lane/m_length;
	// get the jam density
	TFlt _rhoj = m_lane_hold_cap_car;
	// get the critical density
	TFlt _rhok = m_lane_flow_cap_car/m_ffs_car;

	if (_rho >= _rhoj){
		_cost = MNM_Ults::max_link_cost();
	}
	else {
		if (_rho <= _rhok){
			_spd = m_ffs_car;
		}
		else {
			_spd = MNM_Ults::max(0.001 * m_ffs_car, 
					m_lane_flow_cap_car * (_rhoj - _rho) / (_rhoj - _rhok) / _rho);
		}
		_cost = m_length / _spd;
	}
	return _cost;
}

TFlt MNM_Dlink_Ctm_Multiclass::get_link_tt_from_flow_car(TFlt flow)
{
    TFlt _cost, _spd;
    // get the density in veh/mile/lane
    TFlt _rho = flow/m_number_of_lane/m_length;
    // get the jam density
    TFlt _rhoj = m_lane_hold_cap_car;
    // get the critical density
    TFlt _rhok = m_lane_flow_cap_car/m_ffs_car;

    if (_rho >= _rhoj){
        _cost = MNM_Ults::max_link_cost();
    }
    else {
        if (_rho <= _rhok){
            _spd = m_ffs_car;
        }
        else {
            _spd = MNM_Ults::max(0.001 * m_ffs_car,
                                 m_lane_flow_cap_car * (_rhoj - _rho) / (_rhoj - _rhok) / _rho);
        }
        _cost = m_length / _spd;
    }
    return _cost;
}

TFlt MNM_Dlink_Ctm_Multiclass::get_link_tt_from_flow_truck(TFlt flow)
{
    TFlt _cost, _spd;
    // get the density in veh/mile/lane
    TFlt _rho = flow/m_number_of_lane/m_length;
    // get the jam density
    TFlt _rhoj = m_lane_hold_cap_truck;
    // get the critical density
    TFlt _rhok = m_lane_flow_cap_truck/m_ffs_truck;

    if (_rho >= _rhoj){
        _cost = MNM_Ults::max_link_cost();
    }
    else {
        if (_rho <= _rhok){
            _spd = m_ffs_truck;
        }
        else {
            _spd = MNM_Ults::max(0.001 * m_ffs_truck,
                                 m_lane_flow_cap_truck * (_rhoj - _rho) / (_rhoj - _rhok) / _rho);
        }
        _cost = m_length / _spd;
    }
    return _cost;
}

/*							Multiclass CTM Cells
**************************************************************************/
MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::Ctm_Cell_Multiclass(TInt cell_ID, TFlt cell_length,
														TFlt unit_time,
														TFlt hold_cap_car,
														TFlt hold_cap_truck,
														TFlt critical_density_car,
														TFlt critical_density_truck, 
														TFlt rho_1_N,
														TFlt flow_cap_car,
														TFlt flow_cap_truck,
														TFlt ffs_car,
														TFlt ffs_truck,
														TFlt wave_speed_car,
														TFlt wave_speed_truck,
														TFlt flow_scalar)
{
    m_cell_ID = cell_ID;
	m_cell_length = cell_length;
	m_unit_time = unit_time;
	m_flow_scalar = flow_scalar;

	m_hold_cap_car = hold_cap_car; // Veh/m
	m_hold_cap_truck = hold_cap_truck; // Veh/m
	m_critical_density_car = critical_density_car; // Veh/m
	m_critical_density_truck = critical_density_truck; // Veh/m
	m_rho_1_N = rho_1_N; // Veh/m
	m_flow_cap_car = flow_cap_car; // Veh/s
	m_flow_cap_truck = flow_cap_truck; // Veh/s
	m_ffs_car = ffs_car;
	m_ffs_truck = ffs_truck;
	m_wave_speed_car = wave_speed_car;
	m_wave_speed_truck = wave_speed_truck;

	// initialized as car=1, truck=0
	m_space_fraction_car = TFlt(1);
	m_space_fraction_truck = TFlt(0);

	m_volume_car = TInt(0);
	m_volume_truck = TInt(0);
	m_out_veh_car = TInt(0);
	m_out_veh_truck = TInt(0);
	m_veh_queue_car = std::deque<MNM_Veh*>();
	m_veh_queue_truck = std::deque<MNM_Veh*>();
}

MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::~Ctm_Cell_Multiclass()
{
	m_veh_queue_car.clear();
	m_veh_queue_truck.clear();
}

int MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::update_perceived_density()
{
	TFlt _real_volume_car = TFlt(m_volume_car) / m_flow_scalar;
	TFlt _real_volume_truck = TFlt(m_volume_truck) / m_flow_scalar;

	TFlt _density_car = _real_volume_car / m_cell_length;
	TFlt _density_truck = _real_volume_truck / m_cell_length;

	TFlt _space_fraction_car, _space_fraction_truck;
	// printf("0");
	// Free-flow traffic (free-flow for both car and truck classes)
	if (_density_car/m_critical_density_car + _density_truck/m_critical_density_truck <= 1) {
		_space_fraction_car = _density_car/m_critical_density_car;
		_space_fraction_truck = _density_truck/m_critical_density_truck;
		m_perceived_density_car = _density_car + m_critical_density_car * _space_fraction_truck;
		m_perceived_density_truck = _density_truck + m_critical_density_truck * _space_fraction_car;
		if (_space_fraction_car + _space_fraction_truck == 0){
			// same to initial values car=1, truck=0
			m_space_fraction_car = 1;
			m_space_fraction_truck = 0;
		}
		else {
			m_space_fraction_car = _space_fraction_car / (_space_fraction_car + _space_fraction_truck);
			m_space_fraction_truck = _space_fraction_truck / (_space_fraction_car + _space_fraction_truck);
		}
		// printf("-1, %.4f, %.4f", m_space_fraction_car, m_space_fraction_truck);
	}
	// Semi-congested traffic (truck free-flow but car not)
	else if ((_density_truck / m_critical_density_truck < 1) && 
			 (_density_car / (1 - _density_truck/m_critical_density_truck) <= m_rho_1_N)) {
		_space_fraction_truck = _density_truck/m_critical_density_truck;
		_space_fraction_car = 1 - _space_fraction_truck;
		m_perceived_density_car = _density_car / _space_fraction_car;
		m_perceived_density_truck = m_critical_density_truck;
		m_space_fraction_car = _space_fraction_car;
		m_space_fraction_truck = _space_fraction_truck;
		// printf("-2, %.4f, %.4f", m_space_fraction_car, m_space_fraction_truck);
	}
	// Fully congested traffic (both car and truck not free-flow)
	// this case should satisfy: 1. m_perceived_density_car > m_rho_1_N
	// 							 2. m_perceived_density_truck > m_critical_density_truck
	else {
		// _density_truck (m_volume_truck) could still be 0
		if (m_volume_truck == 0) {
			m_perceived_density_car = _density_car;
			_space_fraction_car = 1;
			_space_fraction_truck = 0;
			// this case same speed (u) for both private cars and trucks
			TFlt _u = (m_hold_cap_car - _density_car) * m_wave_speed_car / _density_car;
			m_perceived_density_truck = (m_hold_cap_truck * m_wave_speed_truck) / (_u + m_wave_speed_truck);
		}
		// _density_car (m_volume_car) could still be 0 in some extreme case
		else if (m_volume_car == 0) {
			m_perceived_density_truck = _density_truck;
			_space_fraction_car = 0;
			_space_fraction_truck = 1;
			// this case same speed (u) for both private cars and trucks
			TFlt _u = (m_hold_cap_truck - _density_truck) * m_wave_speed_truck / _density_truck;
			m_perceived_density_car = (m_hold_cap_car * m_wave_speed_car) / (_u + m_wave_speed_car);
		}
		else {
			TFlt _tmp_1 = m_hold_cap_car * m_wave_speed_car * _density_truck;
			TFlt _tmp_2 = m_hold_cap_truck * m_wave_speed_truck * _density_car;
			_space_fraction_car = ( _density_car * _density_truck * (m_wave_speed_car - m_wave_speed_truck) 
									 + _tmp_2 ) / ( _tmp_2 + _tmp_1 );
			_space_fraction_truck = ( _density_car * _density_truck * (m_wave_speed_truck - m_wave_speed_car)
									   + _tmp_1 ) / ( _tmp_2 + _tmp_1 );
			m_perceived_density_car = _density_car / _space_fraction_car;
			m_perceived_density_truck = _density_truck / _space_fraction_truck;
		}
		m_space_fraction_car = _space_fraction_car;
		m_space_fraction_truck = _space_fraction_truck;
		// printf("-3, %.4f, %.4f", m_space_fraction_car, m_space_fraction_truck);
	}
	// printf("\n");
	return 0;
}

TFlt MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::get_perceived_demand(TInt veh_type)
{	
	// car
	if (veh_type == TInt(0)) {
		return std::min(m_flow_cap_car, TFlt(m_ffs_car * m_perceived_density_car)) * m_unit_time;
	}
	// truck
	else {
		return std::min(m_flow_cap_truck, TFlt(m_ffs_truck * m_perceived_density_truck)) * m_unit_time;
	}
}

TFlt MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::get_perceived_supply(TInt veh_type)
{
	TFlt _tmp;
	// car
	if (veh_type == TInt(0)) {
		_tmp = std::min(m_flow_cap_car, TFlt(m_wave_speed_car * (m_hold_cap_car - m_perceived_density_car)));
	}
	// truck
	else {
		_tmp = std::min(m_flow_cap_truck, TFlt(m_wave_speed_truck * (m_hold_cap_truck - m_perceived_density_truck)));
	}
	return std::max(TFlt(0.0), _tmp) * m_unit_time;
}


/**************************************************************************
							Multiclass Link-Queue Model
**************************************************************************/
MNM_Dlink_Lq_Multiclass::MNM_Dlink_Lq_Multiclass(TInt ID,
												TInt number_of_lane,
												TFlt length,
												TFlt lane_hold_cap_car,
												TFlt lane_hold_cap_truck,
												TFlt lane_flow_cap_car,
												TFlt lane_flow_cap_truck,
												TFlt ffs_car,
												TFlt ffs_truck,
												TFlt unit_time,
												TFlt veh_convert_factor,
												TFlt flow_scalar)
  : MNM_Dlink_Multiclass::MNM_Dlink_Multiclass(ID, number_of_lane, length, ffs_car, ffs_truck)
{
    m_link_type = MNM_TYPE_LQ_MULTICLASS;
	m_k_j_car = lane_hold_cap_car * number_of_lane;
	m_k_j_truck = lane_hold_cap_truck * number_of_lane;
	m_C_car = lane_flow_cap_car * number_of_lane;
	m_C_truck = lane_flow_cap_truck * number_of_lane;
	m_k_C_car = m_C_car / ffs_car;
	m_k_C_truck = m_C_truck / ffs_truck;
	m_w_car = m_C_car / (m_k_j_car - m_k_C_car);
	m_w_truck = m_C_truck / (m_k_j_truck - m_k_C_truck);
	m_rho_1_N = m_k_j_car * (m_w_car / (m_ffs_truck + m_w_car));
	
	m_veh_queue_car = std::deque<MNM_Veh*>();
	m_veh_queue_truck = std::deque<MNM_Veh*>();
	m_veh_out_buffer_car = std::deque<MNM_Veh*>();
	m_veh_out_buffer_truck = std::deque<MNM_Veh*>();
	m_volume_car = TInt(0);
	m_volume_truck = TInt(0);

	// initialized as car=1, truck=0
	m_space_fraction_car = TFlt(1);
	m_space_fraction_truck = TFlt(0);

	m_flow_scalar = flow_scalar;
	m_unit_time = unit_time;
	m_veh_convert_factor = veh_convert_factor;
}

MNM_Dlink_Lq_Multiclass::~MNM_Dlink_Lq_Multiclass()
{
	m_veh_queue_car.clear();
	m_veh_queue_truck.clear();
	m_veh_out_buffer_car.clear();
	m_veh_out_buffer_truck.clear();
}

int MNM_Dlink_Lq_Multiclass::update_perceived_density()
{
	TFlt _real_volume_car = TFlt(m_volume_car) / m_flow_scalar;
	TFlt _real_volume_truck = TFlt(m_volume_truck) / m_flow_scalar;

	TFlt _density_car = _real_volume_car / m_length;
	TFlt _density_truck = _real_volume_truck / m_length;

	TFlt _space_fraction_car, _space_fraction_truck;
	// printf("0");
	// Free-flow traffic (free-flow for both car and truck classes)
	if (_density_car/m_k_C_car + _density_truck/m_k_C_truck <= 1) {
		_space_fraction_car = _density_car/m_k_C_car;
		_space_fraction_truck = _density_truck/m_k_C_truck;
		m_perceived_density_car = _density_car + m_k_C_car * _space_fraction_truck;
		m_perceived_density_truck = _density_truck + m_k_C_truck * _space_fraction_car;
		if (_space_fraction_car + _space_fraction_truck == 0){
			// same to initial values: car=1, truck=0
			m_space_fraction_car = 1;
			m_space_fraction_truck = 0;
		}
		else {
			m_space_fraction_car = _space_fraction_car / (_space_fraction_car + _space_fraction_truck);
			m_space_fraction_truck = _space_fraction_truck / (_space_fraction_car + _space_fraction_truck);
		}
		// printf("-1, %.4f, %.4f", m_space_fraction_car, m_space_fraction_truck);
	}
	// Semi-congested traffic (truck free-flow but car not)
	else if ((_density_truck / m_k_C_truck < 1) && 
			 (_density_car / (1 - _density_truck/m_k_C_truck) <= m_rho_1_N)) {
		_space_fraction_truck = _density_truck/m_k_C_truck;
		_space_fraction_car = 1 - _space_fraction_truck;
		m_perceived_density_car = _density_car / _space_fraction_car;
		m_perceived_density_truck = m_k_C_truck;
		m_space_fraction_car = _space_fraction_car;
		m_space_fraction_truck = _space_fraction_truck;
		// printf("-2, %.4f, %.4f", m_space_fraction_car, m_space_fraction_truck);
	}
	// Fully congested traffic (both car and truck not free-flow)
	// this case should satisfy: 1. m_perceived_density_car > m_rho_1_N
	// 							 2. m_perceived_density_truck > m_k_C_truck
	else {
		// _density_truck (m_volume_truck) could still be 0
		if (m_volume_truck == 0) {
			m_perceived_density_car = _density_car;
			_space_fraction_car = 1;
			_space_fraction_truck = 0;
			// this case same speed (u) for both private cars and trucks
			TFlt _u = (m_k_j_car - _density_car) * m_w_car / _density_car;
			m_perceived_density_truck = (m_k_j_truck * m_w_truck) / (_u + m_w_truck);
		}
		// _density_car (m_volume_car) could still be 0 in some extreme case
		else if (m_volume_car == 0) {
			m_perceived_density_truck = _density_truck;
			_space_fraction_car = 0;
			_space_fraction_truck = 1;
			// this case same speed (u) for both private cars and trucks
			TFlt _u = (m_k_j_truck - _density_truck) * m_w_truck / _density_truck;
			m_perceived_density_car = (m_k_j_car * m_w_car) / (_u + m_w_car);
		}
		else {
			TFlt _tmp_1 = m_k_j_car * m_w_car * _density_truck;
			TFlt _tmp_2 = m_k_j_truck * m_w_truck * _density_car;
			_space_fraction_car = ( _density_car * _density_truck * (m_w_car - m_w_truck) + _tmp_2 ) / ( _tmp_2 + _tmp_1 );
			_space_fraction_truck = ( _density_car * _density_truck * (m_w_truck - m_w_car) + _tmp_1 ) / ( _tmp_2 + _tmp_1 );
			m_perceived_density_car = _density_car / _space_fraction_car;
			m_perceived_density_truck = _density_truck / _space_fraction_truck;
		}
		m_space_fraction_car = _space_fraction_car;
		m_space_fraction_truck = _space_fraction_truck;
		// printf("-3, %.4f, %.4f", m_space_fraction_car, m_space_fraction_truck);
	}
	// printf("\n");
	return 0;
}

int MNM_Dlink_Lq_Multiclass::evolve(TInt timestamp)
{
	// Update volume, perceived density, space fraction, and demand/supply
	// printf("1\n");
	std::deque<MNM_Veh*>::iterator _veh_it;
	TInt _count_car = 0;
	TInt _count_truck = 0;
	TInt _count_tot_vehs = 0;
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 0) _count_car += 1;
		if (_veh -> m_class == 1) _count_truck += 1;
	}
	m_volume_car = m_veh_queue_car.size() + _count_car;
	m_volume_truck = m_veh_queue_truck.size() + _count_truck;

	/* compute total volume of link, check if spill back */
	_count_tot_vehs = m_volume_car + m_volume_truck * m_veh_convert_factor;
	if (TFlt(_count_tot_vehs)/m_flow_scalar/m_length > m_k_j_car){
		m_spill_back = true;
	}

	update_perceived_density();

	TFlt _demand_car = m_space_fraction_car * std::min(m_C_car, TFlt(m_ffs_car * m_perceived_density_car)) * m_unit_time;
	TFlt _demand_truck = m_space_fraction_truck * std::min(m_C_truck, TFlt(m_ffs_truck * m_perceived_density_truck)) * m_unit_time;
	TFlt _demand = _demand_car + m_veh_convert_factor * _demand_truck;
	TFlt _veh_to_move = _demand * m_flow_scalar - TInt(m_finished_array.size());

	// Move vehicle from queue to buffer
	MNM_Veh *_v;
	TInt _veh_to_move_car = MNM_Ults::round(_veh_to_move * (_demand_car / _demand));
	_veh_to_move_car = std::min(_veh_to_move_car, TInt(m_veh_queue_car.size()));

	// TFlt _raw = _veh_to_move * (m_veh_convert_factor * _demand_truck / _demand) / m_veh_convert_factor;
	// TInt _tmp = std::floor(_raw);
	// _raw = _raw - TFlt(_tmp);
	// TInt _tmp2 = 0;
	// if (_raw < 0.2){
	// 	_tmp2 = 0;
	// }
	// else if (_raw > 0.8){
	// 	_tmp2 = 1;
	// }
	// else {
	// 	_tmp2 = MNM_Ults::round(_raw);
	// }
	// TInt _veh_to_move_truck = _tmp + _tmp2; 
	TInt _veh_to_move_truck = MNM_Ults::round(_veh_to_move * (m_veh_convert_factor * _demand_truck / _demand) / m_veh_convert_factor);

	_veh_to_move_truck = std::min(_veh_to_move_truck, TInt(m_veh_queue_truck.size()));
	// printf("demand %f, Veh queue size %d, finished size %d, to move %d \n", (float) _demand(), (int) m_veh_queue.size(), (int)m_finished_array.size(), _veh_to_move());
	for (int i = 0; i < _veh_to_move_car; ++i){
		_v = m_veh_queue_car.front();
		m_veh_out_buffer_car.push_back(_v);
		m_veh_queue_car.pop_front();
	}
	for (int i = 0; i < _veh_to_move_truck; ++i){
		_v = m_veh_queue_truck.front();
		m_veh_out_buffer_truck.push_back(_v);
		m_veh_queue_truck.pop_front();
	}

	// Empty buffers, nothing to move to finished array
	if ((m_veh_out_buffer_car.size() == 0) && (m_veh_out_buffer_car.size() == 0)){
		m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
		return 0;
	}

	// Move vehicles from buffer to finished array
	TInt _num_veh_tomove_car = m_veh_out_buffer_car.size();
	TInt _num_veh_tomove_truck = m_veh_out_buffer_truck.size();
	TFlt _pstar = TFlt(_num_veh_tomove_car)/TFlt(_num_veh_tomove_car + _num_veh_tomove_truck);
	MNM_Veh* _veh;
	TFlt _r;
	while ((_num_veh_tomove_car > 0) || (_num_veh_tomove_truck > 0)){
		_r = MNM_Ults::rand_flt();
		// probability = _pstar to move a car
		if (_r < _pstar){
			// still has car to move
			if (_num_veh_tomove_car > 0){
				_veh = m_veh_out_buffer_car.front();
				m_veh_out_buffer_car.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_car--;
			}
			// no car to move, move a truck
			else {
				_veh = m_veh_out_buffer_truck.front();
				m_veh_out_buffer_truck.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_truck--;
			}
		}
		// probability = 1 - _pstar to move a truck
		else {
			// still has truck to move
			if (_num_veh_tomove_truck > 0){
				_veh = m_veh_out_buffer_truck.front();
				m_veh_out_buffer_truck.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_truck--;
			}
			// no truck to move, move a car
			else {
				_veh = m_veh_out_buffer_car.front();
				m_veh_out_buffer_car.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_car--;
			}
		}
	}

	if ((m_veh_out_buffer_car.size() != 0) || (m_veh_out_buffer_car.size() != 0)){
		printf("Something wrong with our buffer, not empty!\n");
		exit(-1);
	}
	m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
	return 0;
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_supply()
{
	// std::deque<MNM_Veh*>::iterator _veh_it;
	// TInt _count_car = 0;
	// TInt _count_truck = 0;
	// for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
	// 	MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
	// 	if (_veh -> m_class == 0) _count_car += 1;
	// 	if (_veh -> m_class == 1) _count_truck += 1;
	// }
	// m_volume_car = m_veh_queue_car.size() + _count_car;
	// m_volume_truck = m_veh_queue_truck.size() + _count_truck;
	// update_perceived_density();

	TFlt _supply_car = m_space_fraction_car * std::min(m_C_car, TFlt(m_w_car * (m_k_j_car - m_perceived_density_car))) * m_unit_time;
	TFlt _supply_truck = m_space_fraction_truck * std::min(m_C_truck, TFlt(m_w_truck * (m_k_j_truck - m_perceived_density_truck))) * m_unit_time;

	// Only for short links, change the FD shape around rhoj:
    _supply_car = std::max(_supply_car, TFlt(m_space_fraction_car * m_w_car * 0.30 * (m_k_j_car - m_k_C_car) * m_unit_time));
    _supply_truck = std::max(_supply_truck, TFlt(m_space_fraction_truck * m_w_truck * 0.30 * (m_k_j_truck - m_k_C_truck) * m_unit_time));

	TFlt _supply = std::max(TFlt(0.0), _supply_car) + m_veh_convert_factor * std::max(TFlt(0.0), _supply_truck);
	return _supply;
}

int MNM_Dlink_Lq_Multiclass::clear_incoming_array(TInt timestamp) {
  	MNM_Veh_Multiclass* _veh;
	size_t _cur_size = m_incoming_array.size();
	for (size_t i = 0; i < _cur_size; ++i) {
		_veh = dynamic_cast<MNM_Veh_Multiclass *>(m_incoming_array.front());
		m_incoming_array.pop_front();
		if (_veh -> m_class == TInt(0)) {
			m_veh_queue_car.push_back(_veh);
		}
		else {
			m_veh_queue_truck.push_back(_veh);
		}
		_veh -> m_visual_position_on_link = 0.5;
	}
	return 0;
}

void MNM_Dlink_Lq_Multiclass::print_info()
{
	printf("Link Dynamic model: Multiclass Link Queue\n");
	printf("Total car volume in the link: %.4f\n", (float)(m_volume_car/m_flow_scalar));
	printf("Total truck volume in the link: %.4f\n", (float)(m_volume_truck/m_flow_scalar));
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_flow_car()
{
	return TFlt(m_volume_car) / m_flow_scalar;
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_flow_truck()
{
	return TFlt(m_volume_truck) / m_flow_scalar;
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_flow()
{
	// For get_link_tt in adaptive routing
	return TFlt(m_volume_car + m_volume_truck) / m_flow_scalar;
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_tt()
{
	// For adaptive routing and emissions, need modification for multiclass cases
	TFlt _cost, _spd;
	TFlt _rho  = get_link_flow() / m_number_of_lane / m_length; // get the density in veh/mile
	TFlt _rhoj = m_k_j_car; //get the jam density
	TFlt _rhok = m_k_C_car; //get the critical density
	//  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
	if (_rho >= _rhoj) {
		_cost = MNM_Ults::max_link_cost(); //sean: i think we should use rhoj, not rhok
	} 
	else {
		if (_rho <= _rhok) {
			_spd = m_ffs_car;
		}
		else {
			_spd = MNM_Ults::max(DBL_EPSILON * m_ffs_car, 
					m_C_car * (_rhoj - _rho) / ((_rhoj - _rhok) * _rho));
		}
		_cost = m_length / _spd;
	} 
	return _cost;
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_tt_from_flow_car(TFlt flow)
{
    TFlt _cost, _spd;
    TFlt _rho  = flow / m_number_of_lane / m_length; // get the density in veh/mile
    TFlt _rhoj = m_k_j_car; //get the jam density
    TFlt _rhok = m_k_C_car; //get the critical density
    //  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
    if (_rho >= _rhoj) {
        _cost = MNM_Ults::max_link_cost(); //sean: i think we should use rhoj, not rhok
    }
    else {
        if (_rho <= _rhok) {
            _spd = m_ffs_car;
        }
        else {
            _spd = MNM_Ults::max(DBL_EPSILON * m_ffs_car,
                                 m_C_car * (_rhoj - _rho) / ((_rhoj - _rhok) * _rho));
        }
        _cost = m_length / _spd;
    }
    return _cost;
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_tt_from_flow_truck(TFlt flow)
{
    TFlt _cost, _spd;
    TFlt _rho  = flow / m_number_of_lane / m_length; // get the density in veh/mile
    TFlt _rhoj = m_k_j_truck; //get the jam density
    TFlt _rhok = m_k_C_truck; //get the critical density
    //  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
    if (_rho >= _rhoj) {
        _cost = MNM_Ults::max_link_cost(); //sean: i think we should use rhoj, not rhok
    }
    else {
        if (_rho <= _rhok) {
            _spd = m_ffs_truck;
        }
        else {
            _spd = MNM_Ults::max(DBL_EPSILON * m_ffs_truck,
                                 m_C_truck * (_rhoj - _rho) / ((_rhoj - _rhok) * _rho));
        }
        _cost = m_length / _spd;
    }
    return _cost;
}

/**************************************************************************
							Multiclass Point-Queue Model
**************************************************************************/
MNM_Dlink_Pq_Multiclass::MNM_Dlink_Pq_Multiclass(TInt ID,
												TInt number_of_lane,
												TFlt length,
												TFlt lane_hold_cap_car,
												TFlt lane_hold_cap_truck,
												TFlt lane_flow_cap_car,
												TFlt lane_flow_cap_truck,
												TFlt ffs_car,
												TFlt ffs_truck,
												TFlt unit_time,
												TFlt veh_convert_factor,
												TFlt flow_scalar)
  : MNM_Dlink_Multiclass::MNM_Dlink_Multiclass(ID, number_of_lane, length, ffs_car, ffs_truck)
{
    m_link_type = MNM_TYPE_PQ_MULTICLASS;
	// PQ only used for OD connectors, cap/rhoj are all 99999 
	// so no need to use truck parameters
	m_lane_hold_cap = lane_hold_cap_car;
	m_lane_flow_cap = lane_flow_cap_car;
	m_flow_scalar = flow_scalar;
	m_hold_cap = m_lane_hold_cap * TFlt(number_of_lane) * m_length;
	m_max_stamp = MNM_Ults::round(m_length/(ffs_car * unit_time));
	// printf("m_max_stamp = %d\n", m_max_stamp);
	m_veh_pool = std::unordered_map<MNM_Veh*, TInt>();
	m_volume_car = TInt(0);
	m_volume_truck = TInt(0);
	m_unit_time = unit_time;
	m_veh_convert_factor = veh_convert_factor;
}

MNM_Dlink_Pq_Multiclass::~MNM_Dlink_Pq_Multiclass()
{
	m_veh_pool.clear();
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_supply()
{
	return m_lane_flow_cap * TFlt(m_number_of_lane) * m_unit_time;
	// return 9999999;
}

int MNM_Dlink_Pq_Multiclass::clear_incoming_array(TInt timestamp) {
	MNM_Veh_Multiclass *_veh;
	TFlt _to_be_moved = get_link_supply() * m_flow_scalar;
	while (!m_incoming_array.empty()) {
		if ( _to_be_moved > 0){
			_veh = dynamic_cast<MNM_Veh_Multiclass *>(m_incoming_array.front());
			m_incoming_array.pop_front();
			m_veh_pool.insert({_veh, TInt(0)});
			if (_veh -> m_class == 0) {
				//printf("car\n");
				// m_volume_car += 1;
				_to_be_moved -= 1;
			}
			else {
				//printf("truck\n");
				// m_volume_truck += 1;
				// _to_be_moved -= m_veh_convert_factor;
				_to_be_moved -= 1;
			}
		}
		else {
			break;
		}
	}

    m_volume_car = 0;
    m_volume_truck = 0;
    for (auto _veh_it : m_veh_pool){
        auto *_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh_it.first);
        if (_veh_multiclass -> m_class == 0) m_volume_car += 1;
        if (_veh_multiclass -> m_class == 1) m_volume_truck += 1;
    }
    for (auto _veh_it : m_finished_array){
        auto *_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh_it);
        if (_veh_multiclass -> m_class == 0) m_volume_car += 1;
        if (_veh_multiclass -> m_class == 1) m_volume_truck += 1;
    }
	// printf("car: %d, truck: %d\n", m_volume_car, m_volume_truck);
	return 0;
}

void MNM_Dlink_Pq_Multiclass::print_info()
{
	printf("Link Dynamic model: Multiclass Point Queue\n");
	printf("Total car volume in the link: %.4f\n", (float)(m_volume_car/m_flow_scalar));
	printf("Total truck volume in the link: %.4f\n", (float)(m_volume_truck/m_flow_scalar));
}

int MNM_Dlink_Pq_Multiclass::evolve(TInt timestamp)
{
	std::unordered_map<MNM_Veh*, TInt>::iterator _que_it = m_veh_pool.begin();
	MNM_Veh_Multiclass* _veh;
	TInt _num_car = 0, _num_truck = 0;
	while (_que_it != m_veh_pool.end()) {
		if (_que_it -> second >= m_max_stamp) {
			m_finished_array.push_back(_que_it -> first);
			_veh = dynamic_cast<MNM_Veh_Multiclass *>(m_finished_array.back());
			if (_veh -> m_class == 0) {
				_num_car += 1;
			}
			else {
				_num_truck += 1;
			}
			_que_it = m_veh_pool.erase(_que_it); //c++ 11
		}
		else {
			_que_it -> second += 1;
			_que_it ++;
		}
	}
	// printf("car: %d, truck: %d\n", _num_car, _num_truck);
	m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
	return 0;
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_flow_car()
{
	return TFlt(m_volume_car) / m_flow_scalar;
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_flow_truck()
{
	return TFlt(m_volume_truck) / m_flow_scalar;
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_flow()
{
	// For adaptive routing, need modification for multiclass case
	return TFlt(m_volume_car + m_volume_truck) / m_flow_scalar;
	// return 0;
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_tt()
{
	// For adaptive routing, need modification for multiclass case

	// TFlt _cost, _spd;
	// // get the density in veh/mile
	// TFlt _rho = get_link_flow()/m_number_of_lane/m_length;
	// // get the jam density
	// TFlt _rhoj = m_lane_hold_cap;
	// // get the critical density
	// TFlt _rhok = m_lane_flow_cap/m_ffs;

	// if (_rho >= _rhoj){
	// 	_cost = MNM_Ults::max_link_cost();
	// }
	// else {
	// 	if (_rho <= _rhok){
	// 		_spd = m_ffs;
	// 	}
	// 	else {
	// 		_spd = MNM_Ults::max(0.001 * m_ffs, 
	// 				m_lane_flow_cap * (_rhoj - _rho) / (_rhoj - _rhok) / _rho);
	// 	}
	// 	_cost = m_length / _spd;
	// }
	// return _cost;
  	
  	// FOR DEBUG ONLY RETURN FREE-FLOW TT
	return m_length/m_ffs_car;
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_tt_from_flow_car(TFlt flow)
{
//     TFlt _cost, _spd;
//     // get the density in veh/mile
//     TFlt _rho = flow/m_number_of_lane/m_length;
//     // get the jam density
//     TFlt _rhoj = m_lane_hold_cap;
//     // get the critical density
//     TFlt _rhok = m_lane_flow_cap/m_ffs_car;
//
//     if (_rho >= _rhoj){
//     	_cost = MNM_Ults::max_link_cost();
//     }
//     else {
//     	if (_rho <= _rhok){
//     		_spd = m_ffs_car;
//     	}
//     	else {
//     		_spd = MNM_Ults::max(0.001 * m_ffs_car,
//     				m_lane_flow_cap * (_rhoj - _rho) / (_rhoj - _rhok) / _rho);
//     	}
//     	_cost = m_length / _spd;
//     }
//     return _cost;

    return m_length/m_ffs_car;
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_tt_from_flow_truck(TFlt flow)
{
//    TFlt _cost, _spd;
//    // get the density in veh/mile
//    TFlt _rho = flow/m_number_of_lane/m_length;
//    // get the jam density
//    TFlt _rhoj = m_lane_hold_cap;
//    // get the critical density
//    TFlt _rhok = m_lane_flow_cap/m_ffs_truck;
//
//    if (_rho >= _rhoj){
//        _cost = MNM_Ults::max_link_cost();
//    }
//    else {
//        if (_rho <= _rhok){
//            _spd = m_ffs_truck;
//        }
//        else {
//            _spd = MNM_Ults::max(0.001 * m_ffs_truck,
//                                 m_lane_flow_cap * (_rhoj - _rho) / (_rhoj - _rhok) / _rho);
//        }
//        _cost = m_length / _spd;
//    }
//    return _cost;

    return m_length/m_ffs_car;
}

/******************************************************************************************************************
*******************************************************************************************************************
												Node Models
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                              Origin node
**************************************************************************/
MNM_DMOND_Multiclass::MNM_DMOND_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor)
	: MNM_DMOND::MNM_DMOND(ID, flow_scalar)
{
	m_veh_convert_factor = veh_convert_factor;
}

MNM_DMOND_Multiclass::~MNM_DMOND_Multiclass()
{
	;
}

int MNM_DMOND_Multiclass::evolve(TInt timestamp)
{
  	MNM_Dlink *_link;
  	MNM_Veh_Multiclass *_veh;
  	MNM_Dlink_Pq_Multiclass *_next_link;

  	for (unsigned i = 0; i < m_out_link_array.size(); ++i){
    	_link = m_out_link_array[i];
   		m_out_volume[_link] = 0;
  	}

  	/* compute out flow */
  	std::deque<MNM_Veh*>::iterator _que_it = m_in_veh_queue.begin();
  	while (_que_it != m_in_veh_queue.end()) {
  		_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_que_it);
    	_link = _veh -> get_next_link();
    	if (_veh -> m_class == 0){
    		m_out_volume[_link] += 1;
    	}
    	else {
    		_next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
    		// m_out_volume[_link] += _next_link -> m_veh_convert_factor;
    		m_out_volume[_link] += 1;
    	}
    	_que_it++;
  	}
  	for (unsigned i = 0; i < m_out_link_array.size(); ++i){
	    _link = m_out_link_array[i];
	    if ((_link -> get_link_supply() * m_flow_scalar) < TFlt(m_out_volume[_link])){
	      	m_out_volume[_link] = TInt(MNM_Ults::round(_link -> get_link_supply() * m_flow_scalar));
	    }
  	}

  	/* move vehicle */
  	TInt _moved_car, _moved_truck;
  	for (unsigned i = 0; i < m_out_link_array.size(); ++i){
	    _link = m_out_link_array[i];
	    _moved_car = 0;
	    _moved_truck = 0;	    
	    _que_it = m_in_veh_queue.begin();
	    while (_que_it != m_in_veh_queue.end()) {
	      	if (m_out_volume[_link] > 0){
		        _veh = dynamic_cast<MNM_Veh_Multiclass *>(*_que_it);
		        if (_veh -> get_next_link() == _link){
					_link -> m_incoming_array.push_back(_veh);
					_veh -> set_current_link(_link);
					if (_veh -> m_class == 0){
						m_out_volume[_link] -= 1;
						_moved_car += 1;
					}
					else {
						_next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
						// m_out_volume[_link] -= _next_link -> m_veh_convert_factor;
						m_out_volume[_link] -= 1;
						_moved_truck += 1;
					}
					_que_it = m_in_veh_queue.erase(_que_it); //c++ 11
		        }
		        else{
		        	_que_it++;
		        }
	      	}
	      	else{
	        	break; //break while loop
	      	}
	    }
	    // record cc for both classes
	    _next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
	    if (_next_link -> m_N_in_car != nullptr && _moved_car > 0) {
	      	_next_link -> m_N_in_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car)/m_flow_scalar));
	    }
	    if (_next_link -> m_N_in_truck != nullptr && _moved_truck > 0) {
	      	_next_link -> m_N_in_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_truck)/m_flow_scalar));
	    }
	    // printf("car: %d, truck: %d\n", _moved_car, _moved_truck);
  	}
  	return 0;
}


/**************************************************************************
                              Destination node
**************************************************************************/
MNM_DMDND_Multiclass::MNM_DMDND_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor)
	: MNM_DMDND::MNM_DMDND(ID, flow_scalar)
{
	m_veh_convert_factor = veh_convert_factor;
}

MNM_DMDND_Multiclass::~MNM_DMDND_Multiclass()
{
	;
}

int MNM_DMDND_Multiclass::evolve(TInt timestamp)
{
  	MNM_Dlink *_link;
  	MNM_Dlink_Pq_Multiclass *_from_link;
  	MNM_Veh_Multiclass *_veh;
  	size_t _size;
  	TInt _moved_car, _moved_truck;
  	for (size_t i = 0; i < m_in_link_array.size(); ++i){
  		_moved_car = 0;
	    _moved_truck = 0;
	    _link = m_in_link_array[i];
	    _size = _link -> m_finished_array.size();
	    for (size_t j = 0; j < _size; ++j){
			_veh = dynamic_cast<MNM_Veh_Multiclass *>(_link -> m_finished_array.front());
			if (_veh -> get_next_link() != nullptr){
				printf("Something wrong in DMDND evolve\n");
				exit(-1);
			}
			m_out_veh_queue.push_back(_veh);
			_veh -> set_current_link(nullptr);
			if (_veh -> m_class == 0){
				_moved_car += 1;
			}
			else {
				_moved_truck += 1;
			}
			_link -> m_finished_array.pop_front();
	    }
	    // record cc for both classes
	    _from_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
	    if (_from_link -> m_N_out_car != nullptr && _moved_car > 0) {
	      	_from_link -> m_N_out_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car)/m_flow_scalar));
	    }
	    if (_from_link -> m_N_out_truck != nullptr && _moved_truck > 0) {
	      	_from_link -> m_N_out_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_truck)/m_flow_scalar));
	    }
  	}
  	return 0;
}


/**************************************************************************
                   				In-out node
**************************************************************************/
MNM_Dnode_Inout_Multiclass::MNM_Dnode_Inout_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor)
	: MNM_Dnode::MNM_Dnode(ID, flow_scalar)
{
	m_demand = NULL;
	m_supply = NULL;
	m_veh_flow = NULL;
	m_veh_moved_car = NULL;
	m_veh_moved_truck = NULL;
	m_veh_convert_factor = veh_convert_factor;
}

MNM_Dnode_Inout_Multiclass::~MNM_Dnode_Inout_Multiclass()
{
  	if (m_demand != NULL) free(m_demand);
  	if (m_supply != NULL) free(m_supply);
  	if (m_veh_flow != NULL) free(m_veh_flow);
  	if (m_veh_moved_car != NULL) free(m_veh_moved_car);
  	if (m_veh_moved_truck != NULL) free(m_veh_moved_truck);
}

int MNM_Dnode_Inout_Multiclass::prepare_loading()
{
	TInt _num_in = m_in_link_array.size();
	TInt _num_out = m_out_link_array.size();
	m_demand = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out); // real-world vehicles
	memset(m_demand, 0x0, sizeof(TFlt) * _num_in * _num_out);
	m_supply = (TFlt*) malloc(sizeof(TFlt) * _num_out); // real-world vehicles
	memset(m_supply, 0x0, sizeof(TFlt) * _num_out);
	m_veh_flow = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out); // real-world vehicles
	memset(m_veh_flow, 0x0, sizeof(TFlt) * _num_in * _num_out);
	m_veh_moved_car = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out); // simulation vehicles = real-world vehicles * flow scalar
	memset(m_veh_moved_car, 0x0, sizeof(TFlt) * _num_in * _num_out);
	m_veh_moved_truck = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out); // simulation vehicles = real-world vehicles * flow scalar
	memset(m_veh_moved_truck, 0x0, sizeof(TFlt) * _num_in * _num_out);
	return 0;
}

int MNM_Dnode_Inout_Multiclass::prepare_supplyANDdemand()
{
	size_t _num_in = m_in_link_array.size();
	size_t _num_out = m_out_link_array.size();
	size_t _offset = m_out_link_array.size();
	TFlt _equiv_count;
	std::deque<MNM_Veh*>::iterator _veh_it;
	MNM_Dlink *_in_link, *_out_link;

	/* zerolize num of vehicle moved */
	memset(m_veh_moved_car, 0x0, sizeof(TFlt) * _num_in * _num_out);
	memset(m_veh_moved_truck, 0x0, sizeof(TFlt) * _num_in * _num_out);

	/* calculate demand */
	for (size_t i = 0; i < _num_in; ++i){
		_in_link = m_in_link_array[i];
		for (_veh_it = _in_link -> m_finished_array.begin(); _veh_it != _in_link -> m_finished_array.end(); _veh_it++){
			if (std::find(m_out_link_array.begin(), m_out_link_array.end(), (*_veh_it) -> get_next_link()) == m_out_link_array.end()){
				printf("Vehicle in the wrong node, no exit!\n");
        		printf("Vehicle is on link %d, node %d, next link ID is: %d\n", _in_link -> m_link_ID(), m_node_ID(), 
        			   (*_veh_it) -> get_next_link() -> m_link_ID());
        		exit(-1);
			}
		}
		for (size_t j = 0; j < _num_out; ++j){
			_out_link = m_out_link_array[j];
			_equiv_count = 0;
			for (_veh_it = _in_link -> m_finished_array.begin(); _veh_it != _in_link -> m_finished_array.end(); _veh_it++){
        		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
        		if (_veh -> get_next_link() == _out_link) {
        			if (_veh -> m_class == 0) {
        				// private car
        				_equiv_count += 1;
        			}
        			else { 
        				// truck
        				_equiv_count += m_veh_convert_factor;
        				// _equiv_count += 1;
        			}
        		}
      		}
      		m_demand[_offset * i + j] = _equiv_count / m_flow_scalar;
		}
	}

	/* calculated supply */
  	for (size_t j = 0; j < _num_out; ++j){
  		// printf("%d, %d\n", j, m_out_link_array[j] -> m_link_ID);
	    m_supply[j] = m_out_link_array[j] -> get_link_supply();
  	}

  	return 0;
}

int MNM_Dnode_Inout_Multiclass::move_vehicle(TInt timestamp)
{
	MNM_Dlink *_in_link, *_out_link;
	MNM_Dlink_Multiclass *_ilink, *_olink;
	size_t _offset = m_out_link_array.size();
	TFlt _to_move;
	TFlt _equiv_num;
	TFlt _r;

    std::vector<size_t> _in_link_ind_array = std::vector<size_t>();
    for (size_t i=0; i<m_in_link_array.size(); ++i){
        _in_link_ind_array.push_back(i);
    }

	for (size_t j = 0; j < m_out_link_array.size(); ++j){
		_out_link = m_out_link_array[j];

        // shuffle the in links, reserve the FIFO
        std::random_device rng; // random sequence
        std::shuffle(_in_link_ind_array.begin(), _in_link_ind_array.end(), rng);
        for (size_t i : _in_link_ind_array) {
		// for (size_t i = 0; i < m_in_link_array.size(); ++i){
			_in_link = m_in_link_array[i];
			_to_move = m_veh_flow[i * _offset + j] * m_flow_scalar;
			// printf("from %d to %d: %.4f\n", int(_in_link -> m_link_ID), int(_out_link -> m_link_ID), double(_to_move));
			auto _veh_it = _in_link -> m_finished_array.begin();

			// if (_to_move > 0){
			// 	auto _veh_it2 = _in_link -> m_finished_array.begin();
			// 	while (_veh_it2 != _in_link -> m_finished_array.end()){
			// 		MNM_Veh_Multiclass *_veh2 = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it2);
			// 		printf("%d ", _veh2 -> m_class);
			// 		_veh_it2++;
			// 	}
			// 	printf("\n");
			// }

			while (_veh_it != _in_link -> m_finished_array.end()){
				if (_to_move > 0){
					MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
					if (_veh -> get_next_link() == _out_link){
						// printf("%d ", _veh -> m_class);
						if (_veh -> m_class == 0) {
							// private car
							_equiv_num = 1;
						}
						else { 
							// truck
							_equiv_num = m_veh_convert_factor;
							// _equiv_num = 1;
						}
						if (_to_move < _equiv_num) {
							// Randomly decide to move or not in this case base on the probability = _to_move/_equiv_num < 1
							// Will result in WRONG INCOMING ARRAY SIZE if the beginning check in function
							// MNM_Dlink_Ctm_Multiclass::clear_incoming_array() was not commented out (@_@)!
							//_r = MNM_Ults::rand_flt();
							
							// Always move 1 more vehicle
							_r = 0;
							if (_r <= _to_move/_equiv_num){
								_out_link -> m_incoming_array.push_back(_veh);
								_veh -> set_current_link(_out_link);
								if (_veh -> m_class == 0){
									m_veh_moved_car[i * _offset + j] += 1;
									_olink = dynamic_cast<MNM_Dlink_Multiclass *>(_out_link);
									_ilink = dynamic_cast<MNM_Dlink_Multiclass *>(_in_link);
									if (_olink -> m_N_in_tree_car != nullptr) {
									 	_olink -> m_N_in_tree_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
//									if (_ilink -> m_N_out_tree_car != nullptr) {
//										_ilink -> m_N_out_tree_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
//									}
								}
								else {
									IAssert(_veh -> m_class == 1);
									m_veh_moved_truck[i * _offset + j] += 1;
									_olink = dynamic_cast<MNM_Dlink_Multiclass *>(_out_link);
									_ilink = dynamic_cast<MNM_Dlink_Multiclass *>(_in_link);
									if (_olink -> m_N_in_tree_truck != nullptr) {
									 	_olink -> m_N_in_tree_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
//									if (_ilink -> m_N_out_tree_truck != nullptr) {
//										_ilink -> m_N_out_tree_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
//									}
								}
								_veh_it = _in_link -> m_finished_array.erase(_veh_it);
							}
						}
						else {
							_out_link -> m_incoming_array.push_back(_veh);
							_veh -> set_current_link(_out_link);
							if (_veh -> m_class == 0){
								m_veh_moved_car[i * _offset + j] += 1;
								_olink = dynamic_cast<MNM_Dlink_Multiclass *>(_out_link);
								_ilink = dynamic_cast<MNM_Dlink_Multiclass *>(_in_link);
								if (_olink -> m_N_in_tree_car != nullptr) {
								 	_olink -> m_N_in_tree_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
								}
//								if (_ilink -> m_N_out_tree_car != nullptr) {
//									_ilink -> m_N_out_tree_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
//								}
							}
							else {
								IAssert(_veh -> m_class == 1);
								m_veh_moved_truck[i * _offset + j] += 1;
								_olink = dynamic_cast<MNM_Dlink_Multiclass *>(_out_link);
								_ilink = dynamic_cast<MNM_Dlink_Multiclass *>(_in_link);
								if (_olink -> m_N_in_tree_truck != nullptr) {
								 	_olink -> m_N_in_tree_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
								}
//								if (_ilink -> m_N_out_tree_truck != nullptr) {
//									_ilink -> m_N_out_tree_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
//								}
							}
							_veh_it = _in_link -> m_finished_array.erase(_veh_it);
						}
						_to_move -= _equiv_num;
					}
					else {
						_veh_it++;
					}
				}
				else {
					break;
				}
			}
			// printf("\n");
			if (_to_move > 0.001){
				printf("Something wrong during the vehicle moving, remaining to move %.16f\n", (float)_to_move);
				// printf("The finished veh queue is now size %d\n", (int)_in_link->m_finished_array.size());
				// printf("But it is heading to %d\n", (int)_in_link->m_finished_array.front() -> get_next_link() -> m_link_ID);
				exit(-1);
			}
		}
        // make the queue randomly perturbed, may not be true in signal controlled intersection, violate FIFO
        // random_shuffle(_out_link -> m_incoming_array.begin(), _out_link -> m_incoming_array.end());
	}
    _in_link_ind_array.clear();
	return 0;
}

int MNM_Dnode_Inout_Multiclass::record_cumulative_curve(TInt timestamp)
{
  	TInt _temp_sum_car, _temp_sum_truck;
  	MNM_Dlink_Multiclass *_in_link, *_out_link;
  	size_t _offset = m_out_link_array.size();

  	for (size_t j = 0; j < m_out_link_array.size(); ++j){
    	_temp_sum_car = 0;
    	_temp_sum_truck = 0;
    	_out_link = dynamic_cast<MNM_Dlink_Multiclass *>(m_out_link_array[j]);
    	for (size_t i = 0; i < m_in_link_array.size(); ++i) {
    		// _in_link = dynamic_cast<MNM_Dlink_Multiclass *>(m_in_link_array[i]);
       		_temp_sum_car += m_veh_moved_car[i * _offset + j];
      		_temp_sum_truck += m_veh_moved_truck[i * _offset + j];
    	}
    	if (_out_link -> m_N_in_car != nullptr && _temp_sum_car > 0) {
      		_out_link -> m_N_in_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_car)/m_flow_scalar));
    	}
    	if (_out_link -> m_N_in_truck != nullptr && _temp_sum_truck > 0) {
      		_out_link -> m_N_in_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_truck)/m_flow_scalar));
    	}
  	}

  	for (size_t i = 0; i < m_in_link_array.size(); ++i){
    	_temp_sum_car = 0;
    	_temp_sum_truck = 0;
    	_in_link = dynamic_cast<MNM_Dlink_Multiclass *>(m_in_link_array[i]);
    	for (size_t j = 0; j < m_out_link_array.size(); ++j) {
      		// _out_link = dynamic_cast<MNM_Dlink_Multiclass *>(m_out_link_array[j]);
      		_temp_sum_car += m_veh_moved_car[i * _offset + j];
      		_temp_sum_truck += m_veh_moved_truck[i * _offset + j];
    	}
    	if (_in_link -> m_N_out_car != nullptr && _temp_sum_car > 0) {
      		_in_link -> m_N_out_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_car)/m_flow_scalar));
    	}
    	if (_in_link -> m_N_out_truck != nullptr && _temp_sum_truck > 0) {
      		_in_link -> m_N_out_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_truck)/m_flow_scalar));
    	}
  	}

  	return 0;
}

int MNM_Dnode_Inout_Multiclass::add_out_link(MNM_Dlink* out_link)
{
  	m_out_link_array.push_back(out_link);
  	return 0;
}

int MNM_Dnode_Inout_Multiclass::add_in_link(MNM_Dlink *in_link)
{
  	m_in_link_array.push_back(in_link);
  	return 0;
}

int MNM_Dnode_Inout_Multiclass::evolve(TInt timestamp)
{
	// printf("Inout node evolve\n");
	// printf("1\n");
	prepare_supplyANDdemand();
	// printf("2\n"); 
	compute_flow();
	// printf("3\n");
	move_vehicle(timestamp);
	// printf("4\n");
	record_cumulative_curve(timestamp);
	// printf("5\n");
	return 0;
}

/*                          FWJ node
**************************************************************************/
MNM_Dnode_FWJ_Multiclass::MNM_Dnode_FWJ_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor)
  : MNM_Dnode_Inout_Multiclass::MNM_Dnode_Inout_Multiclass(ID, flow_scalar, veh_convert_factor)
{
}

MNM_Dnode_FWJ_Multiclass::~MNM_Dnode_FWJ_Multiclass()
{

}

int MNM_Dnode_FWJ_Multiclass::compute_flow()
{
	size_t _offset = m_out_link_array.size();
	TFlt _sum_in_flow, _portion;
	for (size_t j = 0; j < m_out_link_array.size(); ++j){
		_sum_in_flow = TFlt(0);
		for (size_t i = 0; i < m_in_link_array.size(); ++i){
	  		_sum_in_flow += m_demand[i * _offset + j];
		}
		for (size_t i = 0; i < m_in_link_array.size(); ++i){
	  		_portion = MNM_Ults::divide(m_demand[i * _offset + j], _sum_in_flow);
		  	m_veh_flow[i * _offset + j] = MNM_Ults::min(m_demand[i * _offset + j], _portion * m_supply[j]);
		}
	}

	return 0;
}

/*               General Road Junction node
**************************************************************************/
MNM_Dnode_GRJ_Multiclass::MNM_Dnode_GRJ_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor)
  : MNM_Dnode_Inout_Multiclass::MNM_Dnode_Inout_Multiclass(ID, flow_scalar, veh_convert_factor)
{
	m_d_a = nullptr;
	m_C_a = nullptr;
}

MNM_Dnode_GRJ_Multiclass::~MNM_Dnode_GRJ_Multiclass()
{
	if (m_d_a != nullptr) free(m_d_a);
	if (m_C_a != nullptr) free(m_C_a);
}

int MNM_Dnode_GRJ_Multiclass::prepare_loading()
{
	MNM_Dnode_Inout_Multiclass::prepare_loading();
	TInt _num_in = m_in_link_array.size();
	m_d_a = (TFlt*) malloc(sizeof(TFlt) * _num_in);
	memset(m_d_a, 0x0, sizeof(TFlt) * _num_in);
	m_C_a = (TFlt*) malloc(sizeof(TFlt) * _num_in);
	memset(m_C_a, 0x0, sizeof(TFlt) * _num_in);
	return 0;
}

int MNM_Dnode_GRJ_Multiclass::compute_flow()
{
	// to be implemented...
	return 0;
}




/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass OD
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                          		Origin
**************************************************************************/
MNM_Origin_Multiclass::MNM_Origin_Multiclass(TInt ID, 
											 TInt max_interval,
											 TFlt flow_scalar,
											 TInt frequency)
	: MNM_Origin::MNM_Origin(ID, max_interval, flow_scalar, frequency)
{
	m_demand_car = std::unordered_map<MNM_Destination_Multiclass*, TFlt*>();
	m_demand_truck = std::unordered_map<MNM_Destination_Multiclass*, TFlt*>();
}

MNM_Origin_Multiclass::~MNM_Origin_Multiclass()
{
	for (auto _demand_it : m_demand_car) {
		free(_demand_it.second);
	}
	m_demand_car.clear();

	for (auto _demand_it : m_demand_truck) {
		free(_demand_it.second);
	}
	m_demand_truck.clear();
}

int MNM_Origin_Multiclass::add_dest_demand_multiclass(MNM_Destination_Multiclass *dest, 
													TFlt* demand_car, 
													TFlt* demand_truck)
{
	// split (15-mins demand) to (15 * 1-minute demand)
  	TFlt* _demand_car = (TFlt*) malloc(sizeof(TFlt) * m_max_assign_interval * 15);
  	for (int i = 0; i < m_max_assign_interval * 15; ++i) {
  		_demand_car[i] =  TFlt(demand_car[i]);
  	}
  	m_demand_car.insert({dest, _demand_car});

  	TFlt* _demand_truck = (TFlt*) malloc(sizeof(TFlt) * m_max_assign_interval * 15);
  	for (int i = 0; i < m_max_assign_interval * 15; ++i) {
  		_demand_truck[i] =  TFlt(demand_truck[i]);
  	}
  	m_demand_truck.insert({dest, _demand_truck});
  	
  	return 0;
}

int MNM_Origin_Multiclass::release(MNM_Veh_Factory* veh_factory, TInt current_interval)
{
  	// if ((m_current_assign_interval < m_max_assign_interval) && (current_interval % m_frequency == 0)){
   //  	TInt _veh_to_release;
   //  	MNM_Veh_Multiclass *_veh;
   //  	MNM_Veh_Factory_Multiclass *_vfactory = dynamic_cast<MNM_Veh_Factory_Multiclass *>(veh_factory);
   //  	// release all car
	  //   for (auto _demand_it = m_demand_car.begin(); _demand_it != m_demand_car.end(); _demand_it++) {
	  //   	_veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[m_current_assign_interval] * m_flow_scalar));
	  //     	for (int i = 0; i < _veh_to_release; ++i) {
		 //        _veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
		 //        _veh -> set_destination(_demand_it -> first);
		 //        _veh -> set_origin(this);
		 //        m_origin_node -> m_in_veh_queue.push_back(_veh);
	  //     	}
	  //   }
	  //   // release all truck
	  //   for (auto _demand_it = m_demand_truck.begin(); _demand_it != m_demand_truck.end(); _demand_it++) {
	  //   	_veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[m_current_assign_interval] * m_flow_scalar));
	  //     	for (int i = 0; i < _veh_to_release; ++i) {
		 //        _veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
		 //        _veh -> set_destination(_demand_it -> first);
		 //        _veh -> set_origin(this);
		 //        m_origin_node -> m_in_veh_queue.push_back(_veh);
	  //     	}
	  //   }
	  //   m_current_assign_interval++;
  	// }
  	// random_shuffle(m_origin_node -> m_in_veh_queue.begin(), m_origin_node -> m_in_veh_queue.end());
  	return 0;
}

int MNM_Origin_Multiclass::release_one_interval(TInt current_interval, 
												MNM_Veh_Factory* veh_factory, 
												TInt assign_interval, 
												TFlt adaptive_ratio)
{
	if (assign_interval < 0) return 0;
	TInt _veh_to_release;
	MNM_Veh_Multiclass *_veh;
	MNM_Veh_Factory_Multiclass *_vfactory = dynamic_cast<MNM_Veh_Factory_Multiclass *>(veh_factory);
	// release all car
	for (auto _demand_it = m_demand_car.begin(); _demand_it != m_demand_car.end(); _demand_it++) {
		_veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
		for (int i = 0; i < _veh_to_release; ++i) {
			if (adaptive_ratio == TFlt(0)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(0));
			}
			else if (adaptive_ratio == TFlt(1)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
			}
			else{
				TFlt _r = MNM_Ults::rand_flt();
				if (_r <= adaptive_ratio){
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
				}
				else{
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(0));
				}
			}
			_veh -> set_destination(_demand_it -> first);
			_veh -> set_origin(this);
			_veh -> m_assign_interval = assign_interval;
			m_origin_node -> m_in_veh_queue.push_back(_veh);
		}
	}
	// release all truck
	for (auto _demand_it = m_demand_truck.begin(); _demand_it != m_demand_truck.end(); _demand_it++) {
		_veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
		for (int i = 0; i < _veh_to_release; ++i) {
			if (adaptive_ratio == TFlt(0)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(1));
			}
			else if (adaptive_ratio == TFlt(1)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
			}
			else{
				TFlt _r = MNM_Ults::rand_flt();
				if (_r <= adaptive_ratio){
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
				}
				else{
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(1));
				}
			}
			_veh -> set_destination(_demand_it -> first);
			_veh -> set_origin(this);
			_veh -> m_assign_interval = assign_interval;
			m_origin_node -> m_in_veh_queue.push_back(_veh);
		}
	}
	random_shuffle(m_origin_node -> m_in_veh_queue.begin(), m_origin_node -> m_in_veh_queue.end());
 	return 0;
}

int MNM_Origin_Multiclass::release_one_interval_biclass(TInt current_interval, 
														MNM_Veh_Factory* veh_factory, 
														TInt assign_interval, 
														TFlt adaptive_ratio_car,
														TFlt adaptive_ratio_truck)
{
	if (assign_interval < 0) return 0;
	TInt _veh_to_release;
	MNM_Veh_Multiclass *_veh;
	MNM_Veh_Factory_Multiclass *_vfactory = dynamic_cast<MNM_Veh_Factory_Multiclass *>(veh_factory);
	// release all car
	for (auto _demand_it = m_demand_car.begin(); _demand_it != m_demand_car.end(); _demand_it++) {
		_veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
		for (int i = 0; i < _veh_to_release; ++i) {
			if (adaptive_ratio_car == TFlt(0)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(0));
			}
			else if (adaptive_ratio_car == TFlt(1)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
			}
			else{
				TFlt _r = MNM_Ults::rand_flt();
				if (_r <= adaptive_ratio_car){
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
				}
				else{
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(0));
				}
			}
			_veh -> set_destination(_demand_it -> first);
			_veh -> set_origin(this);
			_veh -> m_assign_interval = assign_interval;
			m_origin_node -> m_in_veh_queue.push_back(_veh);
		}
	}
	// release all truck
	for (auto _demand_it = m_demand_truck.begin(); _demand_it != m_demand_truck.end(); _demand_it++) {
		_veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
		for (int i = 0; i < _veh_to_release; ++i) {
			if (adaptive_ratio_truck == TFlt(0)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(1));
			}
			else if (adaptive_ratio_truck == TFlt(1)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
			}
			else{
				TFlt _r = MNM_Ults::rand_flt();
				if (_r <= adaptive_ratio_truck){
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
				}
				else{
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(1));
				}
			}
			_veh -> set_destination(_demand_it -> first);
			_veh -> set_origin(this);
			_veh -> m_assign_interval = assign_interval;
			m_origin_node -> m_in_veh_queue.push_back(_veh);
		}
	}
	random_shuffle(m_origin_node -> m_in_veh_queue.begin(), m_origin_node -> m_in_veh_queue.end());
 	return 0;
}



/**************************************************************************
                          		Destination
**************************************************************************/
MNM_Destination_Multiclass::MNM_Destination_Multiclass(TInt ID)
	: MNM_Destination::MNM_Destination(ID)
{
	;
}


MNM_Destination_Multiclass::~MNM_Destination_Multiclass()
{
	;
}




/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass Vehicle
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Veh_Multiclass::MNM_Veh_Multiclass(TInt ID, TInt vehicle_class, TInt start_time)
	: MNM_Veh::MNM_Veh(ID, start_time)
{
	m_class = vehicle_class;  // 0: car, 1: truck
	m_visual_position_on_link = 0.5; // default: visualize veh as at the middle point of link
}

MNM_Veh_Multiclass::~MNM_Veh_Multiclass()
{
	;
}



/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass Factory
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                          Vehicle Factory
**************************************************************************/
MNM_Veh_Factory_Multiclass::MNM_Veh_Factory_Multiclass()
	: MNM_Veh_Factory::MNM_Veh_Factory()
{
	m_num_car = TInt(0);
	m_num_truck = TInt(0);
	m_enroute_car = TInt(0);
	m_enroute_truck = TInt(0);
	m_finished_car = TInt(0);
	m_finished_truck = TInt(0);
	m_total_time_car = TFlt(0);
	m_total_time_truck = TFlt(0);
}

MNM_Veh_Factory_Multiclass::~MNM_Veh_Factory_Multiclass()
{
	;
}

MNM_Veh_Multiclass* MNM_Veh_Factory_Multiclass::make_veh_multiclass(TInt timestamp, 
														 			Vehicle_type veh_type,
														 			TInt vehicle_cls)
{
	// printf("A vehicle is produce at time %d, ID is %d\n", (int)timestamp, (int)m_num_veh + 1);
	MNM_Veh_Multiclass *_veh = new MNM_Veh_Multiclass(m_num_veh + 1, vehicle_cls, timestamp);
	_veh -> m_type = veh_type;
	m_veh_map.insert({m_num_veh + 1, _veh});

	m_num_veh += 1;
	m_enroute += 1;
	if (vehicle_cls == 0) {
		m_num_car += 1;
		m_enroute_car += 1;
	}
	else if (vehicle_cls == 1) {
		m_num_truck += 1;
		m_enroute_truck += 1;
	}
	return _veh;
}

int MNM_Veh_Factory_Multiclass::remove_finished_veh(MNM_Veh *veh, bool del)
{
	MNM_Veh_Multiclass *_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass*>(veh);
	IAssert(_veh_multiclass != nullptr);
	IAssert(veh -> m_finish_time > veh -> m_start_time);
	if (_veh_multiclass -> m_class == 0) {
		m_finished_car += 1;
		m_enroute_car -= 1;
  		m_total_time_car += (veh -> m_finish_time - veh -> m_start_time);
	}
	else if (_veh_multiclass -> m_class == 1) {
		m_finished_truck += 1;
		m_enroute_truck -= 1;
		m_total_time_truck += (veh -> m_finish_time - veh -> m_start_time);
	}
	MNM_Veh_Factory::remove_finished_veh(veh, del);
	return 0;
}

/**************************************************************************
                          Node factory
**************************************************************************/
MNM_Node_Factory_Multiclass::MNM_Node_Factory_Multiclass()
	: MNM_Node_Factory::MNM_Node_Factory()
{
	;
}

MNM_Node_Factory_Multiclass::~MNM_Node_Factory_Multiclass()
{
	;
}

MNM_Dnode *MNM_Node_Factory_Multiclass::make_node_multiclass(TInt ID, 
												  			DNode_type_multiclass node_type, 
												  			TFlt flow_scalar,
												  			TFlt veh_convert_factor)
{
	MNM_Dnode *_node;
	switch (node_type){
    	case MNM_TYPE_FWJ_MULTICLASS:
			_node = new MNM_Dnode_FWJ_Multiclass(ID, flow_scalar, veh_convert_factor);
			break;
    	case MNM_TYPE_ORIGIN_MULTICLASS:
			_node = new MNM_DMOND_Multiclass(ID, flow_scalar, veh_convert_factor);
			break;
    	case MNM_TYPE_DEST_MULTICLASS:
			_node = new MNM_DMDND_Multiclass(ID, flow_scalar, veh_convert_factor);
			break;
    	default:
			printf("Wrong node type.\n");
			exit(-1);
	}
	m_node_map.insert({ID, _node});
	return _node;
}

/**************************************************************************
                          Link factory
**************************************************************************/
MNM_Link_Factory_Multiclass::MNM_Link_Factory_Multiclass()
	: MNM_Link_Factory::MNM_Link_Factory()
{
	;
}

MNM_Link_Factory_Multiclass::~MNM_Link_Factory_Multiclass()
{
	;
}

MNM_Dlink *MNM_Link_Factory_Multiclass::make_link_multiclass(TInt ID,
									                        DLink_type_multiclass link_type,
															TInt number_of_lane,
															TFlt length,
															TFlt lane_hold_cap_car,
															TFlt lane_hold_cap_truck,
															TFlt lane_flow_cap_car,
															TFlt lane_flow_cap_truck,
															TFlt ffs_car,
															TFlt ffs_truck,
															TFlt unit_time,
															TFlt veh_convert_factor,
															TFlt flow_scalar)
{
	MNM_Dlink *_link;
	switch (link_type){
    	case MNM_TYPE_CTM_MULTICLASS:
			_link = new MNM_Dlink_Ctm_Multiclass(ID,
												number_of_lane,
												length,
												lane_hold_cap_car,
												lane_hold_cap_truck,
												lane_flow_cap_car,
												lane_flow_cap_truck,
												ffs_car,
												ffs_truck,
												unit_time,
												veh_convert_factor,
												flow_scalar);
			break;
		case MNM_TYPE_LQ_MULTICLASS:
			_link = new MNM_Dlink_Lq_Multiclass(ID,
												number_of_lane,
												length,
												lane_hold_cap_car,
												lane_hold_cap_truck,
												lane_flow_cap_car,
												lane_flow_cap_truck,
												ffs_car,
												ffs_truck,
												unit_time,
												veh_convert_factor,
												flow_scalar);
			break;
    	case MNM_TYPE_PQ_MULTICLASS:
			_link = new MNM_Dlink_Pq_Multiclass(ID,
												number_of_lane,
												length,
												lane_hold_cap_car,
												lane_hold_cap_truck,
												lane_flow_cap_car,
												lane_flow_cap_truck,
												ffs_car,
												ffs_truck,
												unit_time,
												veh_convert_factor,
												flow_scalar);
			break;
    	default:
			printf("Wrong link type.\n");
			exit(-1);
	}
	m_link_map.insert({ID, _link});
	return _link;
}

/**************************************************************************
                          OD factory
**************************************************************************/
MNM_OD_Factory_Multiclass::MNM_OD_Factory_Multiclass()
	: MNM_OD_Factory::MNM_OD_Factory()
{
	;
}

MNM_OD_Factory_Multiclass::~MNM_OD_Factory_Multiclass()
{
	;
}

MNM_Destination_Multiclass *MNM_OD_Factory_Multiclass::make_destination(TInt ID)
{
	MNM_Destination_Multiclass *_dest;
	_dest = new MNM_Destination_Multiclass(ID);
	m_destination_map.insert({ID, _dest});
	return _dest;
}

MNM_Origin_Multiclass *MNM_OD_Factory_Multiclass::make_origin(TInt ID, 
												TInt max_interval, 
												TFlt flow_scalar, 
												TInt frequency)
{
	MNM_Origin_Multiclass *_origin;
	_origin = new MNM_Origin_Multiclass(ID, max_interval, flow_scalar, frequency);
	m_origin_map.insert({ID, _origin});
	return _origin;
}




/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass IO Functions
*******************************************************************************************************************
******************************************************************************************************************/
int MNM_IO_Multiclass::build_node_factory_multiclass(const std::string& file_folder,
											         MNM_ConfReader *conf_reader,
											         MNM_Node_Factory *node_factory,
                                                     const std::string& file_name)
{
	/* find file */
	std::string _node_file_name = file_folder + "/" + file_name;
	std::ifstream _node_file;
	_node_file.open(_node_file_name, std::ios::in);

	/* read config */
	TInt _num_of_node = conf_reader -> get_int("num_of_node");
	TFlt _flow_scalar = conf_reader -> get_float("flow_scalar");

	/* read file */
	std::string _line;
	std::vector<std::string> _words;
	TInt _node_ID;
	std::string _type;
	TFlt _veh_convert_factor;

	MNM_Node_Factory_Multiclass* _node_factory = dynamic_cast<MNM_Node_Factory_Multiclass *>(node_factory);
	if (_node_file.is_open())
	{
		std::getline(_node_file,_line); //skip the first line
		for (int i = 0; i < _num_of_node; ++i){
			std::getline(_node_file,_line);
			// printf("%d\n", i);
			_words = split(_line, ' ');
			if (_words.size() == 3) {
				_node_ID = TInt(std::stoi(_words[0]));
				_type = trim(_words[1]);
				_veh_convert_factor = TFlt(std::stod(_words[2]));
				if (_type == "FWJ"){
					_node_factory -> make_node_multiclass(_node_ID, 
														MNM_TYPE_FWJ_MULTICLASS, 
														_flow_scalar,
														_veh_convert_factor);
					continue;
				}				
				if (_type =="DMOND"){
					_node_factory -> make_node_multiclass(_node_ID, 
														MNM_TYPE_ORIGIN_MULTICLASS, 
														_flow_scalar,
														_veh_convert_factor);
					continue;
				}
				if (_type =="DMDND"){
					_node_factory -> make_node_multiclass(_node_ID, 
														MNM_TYPE_DEST_MULTICLASS, 
														_flow_scalar,
														_veh_convert_factor);
					continue;
				}
				printf("Wrong node type, %s\n", _type.c_str());
				exit(-1);
			}
			else {
				printf("MNM_IO_Multiclass::build_node_factory_Multiclass: Wrong length of line.\n");
				exit(-1);
			}
		}
		_node_file.close();
	}
	return 0;
}

int MNM_IO_Multiclass::build_link_factory_multiclass(const std::string& file_folder,
                                                     MNM_ConfReader *conf_reader,
                                                     MNM_Link_Factory *link_factory,
                                                     const std::string& file_name)
{
	/* find file */
	std::string _link_file_name = file_folder + "/" + file_name;
	std::ifstream _link_file;
	_link_file.open(_link_file_name, std::ios::in);

	/* read config */
	TInt _num_of_link = conf_reader -> get_int("num_of_link");
	TFlt _flow_scalar = conf_reader -> get_float("flow_scalar");
	TFlt _unit_time = conf_reader -> get_float("unit_time");

	/* read file */
	std::string _line;
	std::vector<std::string> _words;
	TInt _link_ID;
	TFlt _lane_hold_cap_car;
	TFlt _lane_flow_cap_car;
	TInt _number_of_lane;
	TFlt _length;
	TFlt _ffs_car;
	std::string _type;
	// new in multiclass vehicle case
	TFlt _lane_hold_cap_truck;
	TFlt _lane_flow_cap_truck;
	TFlt _ffs_truck;
	TFlt _veh_convert_factor;

	MNM_Link_Factory_Multiclass* _link_factory = dynamic_cast<MNM_Link_Factory_Multiclass *>(link_factory);

	if (_link_file.is_open())
	{
		// printf("Start build link factory.\n");
		std::getline(_link_file,_line); //skip the first line
		for (int i = 0; i < _num_of_link; ++i){
			std::getline(_link_file,_line);
			_words = split(_line, ' ');
			if (_words.size() == 11) {
				_link_ID = TInt(std::stoi(_words[0]));
				_type = trim(_words[1]);
				_length = TFlt(std::stod(_words[2]));
				_ffs_car = TFlt(std::stod(_words[3]));
				_lane_flow_cap_car = TFlt(std::stod(_words[4]));  // flow capacity (vehicles/hour/lane)
				_lane_hold_cap_car = TFlt(std::stod(_words[5]));  // jam density (vehicles/mile/lane)
				_number_of_lane = TInt(std::stoi(_words[6]));
				// new in multiclass vehicle case
				_ffs_truck = TFlt(std::stod(_words[7]));
				_lane_flow_cap_truck = TFlt(std::stod(_words[8]));
				_lane_hold_cap_truck = TFlt(std::stod(_words[9]));
				_veh_convert_factor = TFlt(std::stod(_words[10]));

				/* unit conversion */
				// mile -> meter, hour -> second
				_length = _length * TFlt(1600); // m
				_ffs_car = _ffs_car * TFlt(1600) / TFlt(3600); // m/s
				_lane_flow_cap_car = _lane_flow_cap_car / TFlt(3600);  // vehicles/s/lane
				_lane_hold_cap_car = _lane_hold_cap_car / TFlt(1600);  // vehicles/m/lane
				_ffs_truck = _ffs_truck * TFlt(1600) / TFlt(3600);  // m/s
				_lane_flow_cap_truck = _lane_flow_cap_truck / TFlt(3600);  // vehicles/s/lane
				_lane_hold_cap_truck = _lane_hold_cap_truck / TFlt(1600);  // vehicles/m/lane

				/* build */
				if (_type == "PQ"){
					_link_factory -> make_link_multiclass(_link_ID,
														MNM_TYPE_PQ_MULTICLASS,
														_number_of_lane,
														_length,
														_lane_hold_cap_car,
														_lane_hold_cap_truck,
														_lane_flow_cap_car,
														_lane_flow_cap_truck,
														_ffs_car,
														_ffs_truck,
														_unit_time,
														_veh_convert_factor,
														_flow_scalar);
					continue;
				}
				if (_type == "LQ"){
					_link_factory -> make_link_multiclass(_link_ID,
														MNM_TYPE_LQ_MULTICLASS,
														_number_of_lane,
														_length,
														_lane_hold_cap_car,
														_lane_hold_cap_truck,
														_lane_flow_cap_car,
														_lane_flow_cap_truck,
														_ffs_car,
														_ffs_truck,
														_unit_time,
														_veh_convert_factor,
														_flow_scalar);
					continue;
				}
				if (_type =="CTM"){
					_link_factory -> make_link_multiclass(_link_ID,
														MNM_TYPE_CTM_MULTICLASS,
														_number_of_lane,
														_length,
														_lane_hold_cap_car,
														_lane_hold_cap_truck,
														_lane_flow_cap_car,
														_lane_flow_cap_truck,
														_ffs_car,
														_ffs_truck,
														_unit_time,
														_veh_convert_factor,
														_flow_scalar);
					continue;
				}
				printf("Wrong link type, %s\n", _type.c_str());
				exit(-1);
			}
			else{
				printf("MNM_IO::build_link_factory::Wrong length of line.\n");
				exit(-1);
			}
		}
		_link_file.close();
	}
	return 0;
}


int MNM_IO_Multiclass::build_demand_multiclass(const std::string& file_folder,
 											   MNM_ConfReader *conf_reader,
 											   MNM_OD_Factory *od_factory,
                                               const std::string& file_name)
{
	/* find file */
	std::string _demand_file_name = file_folder + "/" + file_name;
	std::ifstream _demand_file;
	_demand_file.open(_demand_file_name, std::ios::in);

	/* read config */
	TFlt _flow_scalar = conf_reader -> get_float("flow_scalar");
	TInt _unit_time = conf_reader -> get_int("unit_time");
	TInt _num_of_minute =  int(conf_reader -> get_int("assign_frq")) / (60 / _unit_time);  // the releasing strategy is assigning vehicles per 1 minute
	TInt _max_interval = conf_reader -> get_int("max_interval"); 
	TInt _num_OD = conf_reader -> get_int("OD_pair");
	TInt _init_demand_split = conf_reader -> get_int("init_demand_split");

	/* build */
	TInt _O_ID, _D_ID;
	MNM_Origin_Multiclass *_origin;
	MNM_Destination_Multiclass *_dest;
	std::string _line;
	std::vector<std::string> _words;
	if (_demand_file.is_open())
	{
		// printf("Start build demand profile.\n");
		TFlt *_demand_vector_car = (TFlt*) malloc(sizeof(TFlt) * _max_interval * _num_of_minute);
		TFlt *_demand_vector_truck = (TFlt*) malloc(sizeof(TFlt) * _max_interval * _num_of_minute);
		TFlt _demand_car;
		TFlt _demand_truck;

		std::getline(_demand_file,_line); //skip the first line
		for (int i = 0; i < _num_OD; ++i){
			std::getline(_demand_file,_line);
			_words = split(_line, ' ');
			if (TInt(_words.size()) == (_max_interval * 2 + 2)) {
				_O_ID = TInt(std::stoi(_words[0]));
				_D_ID = TInt(std::stoi(_words[1]));
				memset(_demand_vector_car, 0x0, sizeof(TFlt) * _max_interval * _num_of_minute);
				memset(_demand_vector_truck, 0x0, sizeof(TFlt) * _max_interval * _num_of_minute);
				// the releasing strategy is assigning vehicles per 1 minute, so disaggregate 15-min demand into 1-min demand
				for (int j = 0; j < _max_interval; ++j) {
					// _demand_car = TFlt(std::stod(_words[j + 2])) / TFlt(_num_of_minute);  
					// _demand_truck = TFlt(std::stod(_words[j + _max_interval + 2])) / TFlt(_num_of_minute);
					// for (int k = 0; k < _num_of_minute; ++k){
					// 	_demand_vector_car[j * _num_of_minute + k] = _demand_car;
					// 	_demand_vector_truck[j * _num_of_minute + k] = _demand_truck;
					// }

					if (_init_demand_split == 0) {
                        _demand_car = TFlt(std::stod(_words[j + 2]));
                        _demand_truck = TFlt(std::stod(_words[j + _max_interval + 2]));
                        _demand_vector_car[j * _num_of_minute] = _demand_car;
                        _demand_vector_truck[j * _num_of_minute] = _demand_truck;
                    }
                    else if (_init_demand_split == 1) {
                        // find suitable releasing interval so that the agent-based DNL is feasible
                        for (int p = 0; p < _num_of_minute; ++p) {
                            _demand_car = TFlt(std::stod(_words[j + 2])) / TFlt(_num_of_minute - p);
                            // if (round(_demand_car * _flow_scalar) >= 1){
                            if (floor(_demand_car * _flow_scalar) >= 1){
                                for (int k = 0; k < _num_of_minute - p; ++k){
                                    _demand_vector_car[j * _num_of_minute + k] = _demand_car;
                                }
                                break;
                            }
                        }
                        for (int p = 0; p < _num_of_minute; ++p) {
                            _demand_truck = TFlt(std::stod(_words[j + _max_interval + 2])) / TFlt(_num_of_minute - p);
                            // if (round(_demand_truck * _flow_scalar) >= 1){
                            if (floor(_demand_truck * _flow_scalar) >= 1){
                                for (int k = 0; k < _num_of_minute - p; ++k){
                                    _demand_vector_truck[j * _num_of_minute + k] = _demand_truck;
                                }
                                break;
                            }
                        }
                    }
                    else {
                        printf("Wrong init_demand_split\n");
                        exit(-1);
                    }

				}
				_origin = dynamic_cast<MNM_Origin_Multiclass *>(od_factory -> get_origin(_O_ID));
				_dest = dynamic_cast<MNM_Destination_Multiclass *>(od_factory -> get_destination(_D_ID));
				_origin -> add_dest_demand_multiclass(_dest, _demand_vector_car, _demand_vector_truck);
			}
			else{
				printf("Something wrong in build_demand!\n");
				free(_demand_vector_car);
				free(_demand_vector_truck);
				exit(-1);
			}
		}
		free(_demand_vector_car);
		free(_demand_vector_truck);
		_demand_file.close();
	}
	return 0;
}




/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass DTA
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Dta_Multiclass::MNM_Dta_Multiclass(const std::string& file_folder)
	: MNM_Dta::MNM_Dta(file_folder)
{
	// Re-run the multiclass version of initialize();
	initialize();
}

MNM_Dta_Multiclass::~MNM_Dta_Multiclass()
{
	;
}

int MNM_Dta_Multiclass::initialize()
{
	if (m_veh_factory != nullptr) delete m_veh_factory;
  	if (m_node_factory != nullptr) delete m_node_factory;
  	if (m_link_factory != nullptr) delete m_link_factory;
  	if (m_od_factory != nullptr) delete m_od_factory;
  	if (m_config != nullptr) delete m_config;
	m_veh_factory = new MNM_Veh_Factory_Multiclass();
	// printf("1\n");
	m_node_factory = new MNM_Node_Factory_Multiclass();
	// printf("2\n");
	m_link_factory = new MNM_Link_Factory_Multiclass();
	// printf("3\n");
	m_od_factory = new MNM_OD_Factory_Multiclass();
	// printf("4\n");
	m_config = new MNM_ConfReader(m_file_folder + "/config.conf", "DTA");
	m_unit_time = m_config -> get_int("unit_time");
	m_flow_scalar = m_config -> get_int("flow_scalar");
	// printf("5\n");
	m_emission = new MNM_Cumulative_Emission_Multiclass(TFlt(m_unit_time), 0);
    
	// the releasing strategy is assigning vehicles per 1 minute, so disaggregate 15-min demand into 1-min demand
	// change assign_freq to 12 (1 minute = 12 x 5 second / 60) and total_assign_interval to max_interval*_num_of_minute
	m_assign_freq = 60 / int(m_unit_time);  // # of unit intervals in 1 min = # of assign freq
	TInt _num_of_minute =  int(m_config -> get_int("assign_frq")) / m_assign_freq;  // 15 min, # of minutes in original assign interval
	m_total_assign_inter = m_config ->  get_int("max_interval") * _num_of_minute;  // how many 1-min intervals
	m_start_assign_interval = m_config -> get_int("start_assign_interval");

	return 0;
}

int MNM_Dta_Multiclass::build_from_files()
{
	MNM_IO_Multiclass::build_node_factory_multiclass(m_file_folder, m_config, m_node_factory);
	MNM_IO_Multiclass::build_link_factory_multiclass(m_file_folder, m_config, m_link_factory);
	// MNM_IO_Multiclass::build_od_factory_multiclass(m_file_folder, m_config, m_od_factory, m_node_factory);
	MNM_IO_Multiclass::build_od_factory(m_file_folder, m_config, m_od_factory, m_node_factory);
	m_graph = MNM_IO_Multiclass::build_graph(m_file_folder, m_config);
	MNM_IO_Multiclass::build_demand_multiclass(m_file_folder, m_config, m_od_factory);
	// build_workzone();
	m_workzone = nullptr;
	set_statistics();
	set_routing();
	return 0;
}

int MNM_Dta_Multiclass::pre_loading()
{
	MNM_Dnode *_node;
	// printf("MNM: Prepare loading!\n");
	m_statistics -> init_record();
	for (auto _node_it = m_node_factory -> m_node_map.begin(); _node_it != m_node_factory -> m_node_map.end(); _node_it++){
		_node = _node_it -> second;
		_node -> prepare_loading();
	}

    // https://stackoverflow.com/questions/7443787/using-c-ifstream-extraction-operator-to-read-formatted-data-from-a-file
	std::ifstream _emission_file(m_file_folder + "/MNM_input_emission_linkID");
	int _link_ID;
	std::unordered_map<int, int> _emission_links = {};
	while (_emission_file >> _link_ID)
	{
	    _emission_links.insert({_link_ID, 0});
	}
	_emission_file.close();

	std::deque<TInt> *_rec;
  	for (auto _map_it : m_link_factory -> m_link_map)
  	{
    	_rec = new std::deque<TInt>();
    	m_queue_veh_map.insert({_map_it.second -> m_link_ID, _rec});
    	if (_emission_links.find(int(_map_it.second -> m_link_ID)) != _emission_links.end()) 
    		m_emission -> register_link(_map_it.second);
  	}
  	
	printf("Exiting MNM: Prepare loading!\n");
	return 0;
}




/******************************************************************************************************************
*******************************************************************************************************************
										Multiclass DTA Gradient Utils
*******************************************************************************************************************
******************************************************************************************************************/

// All functions/API to python should be coded under this namespace
namespace MNM_DTA_GRADIENT
{
TFlt get_link_inflow_car(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_link_inflow_car link is null");
	}
	if (link -> m_N_in_car == nullptr){
		throw std::runtime_error("Error, get_link_inflow_car link cumulative curve is not installed");
	}
	return link -> m_N_in_car -> get_result(end_time) - link -> m_N_in_car -> get_result(start_time);
}

TFlt get_link_inflow_car(MNM_Dlink_Multiclass* link, 
                    	TInt start_time, TInt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_link_inflow_car link is null");
	}
	if (link -> m_N_in_car == nullptr){
		throw std::runtime_error("Error, get_link_inflow_car link cumulative curve is not installed");
	}
	return link -> m_N_in_car -> get_result(TFlt(end_time)) - link -> m_N_in_car -> get_result(TFlt(start_time));
}

TFlt get_link_inflow_truck(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_link_inflow_truck link is null");
	}
	if (link -> m_N_in_truck == nullptr){
		throw std::runtime_error("Error, get_link_inflow_truck link cumulative curve is not installed");
	}
	return link -> m_N_in_truck -> get_result(end_time) - link -> m_N_in_truck -> get_result(start_time);
}

TFlt get_link_inflow_truck(MNM_Dlink_Multiclass* link, 
                           TInt start_time, TInt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_link_inflow_truck link is null");
	}
	if (link -> m_N_in_truck == nullptr){
		throw std::runtime_error("Error, get_link_inflow_truck link cumulative curve is not installed");
	}
	return link -> m_N_in_truck -> get_result(TFlt(end_time)) - link -> m_N_in_truck -> get_result(TFlt(start_time));
}

TFlt get_average_waiting_time_at_intersection(MNM_Dlink_Multiclass* link)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_average_waiting_time_at_intersection link is null");
	}
	if (link -> m_N_in_car == nullptr){
		throw std::runtime_error("Error, get_average_waiting_time_at_intersection link car in cumulative curve is not installed");
	}
	if (link -> m_N_in_truck == nullptr){
		throw std::runtime_error("Error, get_average_waiting_time_at_intersection link truck in cumulative curve is not installed");
	}
	TFlt _tot_vehs = 0;
	_tot_vehs = link -> m_N_in_car -> m_recorder.back().second + link -> m_N_in_truck -> m_recorder.back().second;

	return link -> m_tot_wait_time_at_intersection / _tot_vehs;
}

TInt get_is_spillback(MNM_Dlink_Multiclass* link) // 0 - no spillback, 1 - spillback
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_is_spillback link is null");
	}
	if (link -> m_spill_back){
		return 1;
	}
	else {
		return 0;
	}
}

TFlt get_travel_time_from_FD_car(MNM_Dlink_Multiclass *link, TFlt start_time, TFlt unit_interval)
{
    TFlt _flow = link -> m_N_in_car -> get_result(start_time) - link -> m_N_out_car -> get_result(start_time);
    if (_flow < 0.) _flow = 0.;
    TFlt _tt = link ->get_link_tt_from_flow_car(_flow);
    return _tt / unit_interval;
}

TFlt get_travel_time_from_FD_truck(MNM_Dlink_Multiclass *link, TFlt start_time, TFlt unit_interval)
{
    TFlt _flow = link -> m_N_in_truck -> get_result(start_time) - link -> m_N_out_truck -> get_result(start_time);
    if (_flow < 0.) _flow = 0.;
    TFlt _tt = link ->get_link_tt_from_flow_truck(_flow);
    return _tt / unit_interval;
}

TFlt get_travel_time_car(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt unit_interval)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_travel_time_car link is null");
	}
	if (link -> m_N_in_car == nullptr){
		throw std::runtime_error("Error, get_travel_time_car link cumulative curve is not installed");
	}

	TFlt fftt = link -> get_link_freeflow_tt_car() / unit_interval;

	if (link -> m_last_valid_time < 0) {
		link -> m_last_valid_time = get_last_valid_time(link -> m_N_in_car, link -> m_N_out_car);
	}
	IAssert(link -> m_last_valid_time >= 0);

	return get_travel_time_from_cc(start_time, link -> m_N_in_car, link -> m_N_out_car, link -> m_last_valid_time, fftt);
}


TFlt get_travel_time_car_robust(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt end_time, TFlt unit_interval, TInt num_trials)
{
	TFlt _delta = (end_time - start_time) / TFlt(num_trials);
	TFlt _ave_tt = TFlt(0);
	for (int i=0; i < num_trials(); ++i){
		_ave_tt += get_travel_time_car(link, start_time + TFlt(i) * _delta, unit_interval);
	}
	return _ave_tt / TFlt(num_trials);
}


TFlt get_travel_time_truck(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt unit_interval)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_travel_time_truck link is null");
	}
	if (link -> m_N_in_truck == nullptr){
		throw std::runtime_error("Error, get_travel_time_truck link cumulative curve is not installed");
	}
	// printf("%.2f\n", start_time);

    TFlt fftt = link -> get_link_freeflow_tt_truck() / unit_interval;

	if (link -> m_last_valid_time_truck < 0) {
		link -> m_last_valid_time_truck = get_last_valid_time(link -> m_N_in_truck, link -> m_N_out_truck);
	}
	IAssert(link -> m_last_valid_time_truck >= 0);

	return get_travel_time_from_cc(start_time, link -> m_N_in_truck, link -> m_N_out_truck, link -> m_last_valid_time_truck, fftt);
}

int add_dar_records_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                        std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
  if (link == nullptr){
    throw std::runtime_error("Error, add_dar_records_car link is null");
  }
  if (link -> m_N_in_tree_car == nullptr){
    throw std::runtime_error("Error, add_dar_records_car link cumulative curve tree is not installed");
  }
  MNM_Path* _path;
  for (auto path_it : link -> m_N_in_tree_car -> m_record){
    _path = path_it.first;
	if (pathset.find(_path) != pathset.end()) {
      for (auto depart_it : path_it.second){
        TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
        if (tmp_flow > DBL_EPSILON){
          auto new_record = new dar_record();
          new_record -> path_ID = path_it.first -> m_path_ID;
          // the count of 1 min intervals, the vehicles record this assign_int
          new_record -> assign_int = depart_it.first;
          new_record -> link_ID = link -> m_link_ID;
          // the count of unit time interval (5s)
          new_record -> link_start_int = start_time;
          new_record -> flow = tmp_flow;
          // printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
          //     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
          record.push_back(new_record);
        }
      }
    }
  }
  return 0;
}

int add_dar_records_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                          std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
  if (link == nullptr){
    throw std::runtime_error("Error, add_dar_records_truck link is null");
  }
  if (link -> m_N_in_tree_truck == nullptr){
    throw std::runtime_error("Error, add_dar_records_truck link cumulative curve tree is not installed");
  }
  MNM_Path* _path;
  for (auto path_it : link -> m_N_in_tree_truck -> m_record){
    _path = path_it.first;
	if (pathset.find(_path) != pathset.end()) {
      for (auto depart_it : path_it.second){
        TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
        if (tmp_flow > DBL_EPSILON){
          auto new_record = new dar_record();
          new_record -> path_ID = path_it.first -> m_path_ID;
          // the count of 1 min intervals, the vehicles record this assign_int
          new_record -> assign_int = depart_it.first;
          new_record -> link_ID = link -> m_link_ID;
          // the count of unit time interval (5s)
          new_record -> link_start_int = start_time;
          new_record -> flow = tmp_flow;
          // printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
          //     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
          record.push_back(new_record);
        }
      }
    }
  }
  return 0;
}

int add_dar_records_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                        std::set<TInt> pathID_set, TFlt start_time, TFlt end_time)
{
  if (link == nullptr){
    throw std::runtime_error("Error, add_dar_records_car link is null");
  }
  if (link -> m_N_in_tree_car == nullptr){
    throw std::runtime_error("Error, add_dar_records_car link cumulative curve tree is not installed");
  }

  MNM_Path* _path;
  for (auto path_it : link -> m_N_in_tree_car -> m_record){
    _path = path_it.first;
	if (pathID_set.find(_path -> m_path_ID) != pathID_set.end()) {
      for (auto depart_it : path_it.second){
        TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
        if (tmp_flow > DBL_EPSILON){
          auto new_record = new dar_record();
          new_record -> path_ID = path_it.first -> m_path_ID;
          // the count of 1 min intervals, the vehicles record this assign_int
          new_record -> assign_int = depart_it.first;
          new_record -> link_ID = link -> m_link_ID;
          // the count of unit time interval (5s)
          new_record -> link_start_int = start_time;
          new_record -> flow = tmp_flow;
          // printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
          //     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
          record.push_back(new_record);
        }
      }
    }
  }
  return 0;
}

int add_dar_records_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                          std::set<TInt> pathID_set, TFlt start_time, TFlt end_time)
{
  if (link == nullptr){
    throw std::runtime_error("Error, add_dar_records_truck link is null");
  }
  if (link -> m_N_in_tree_truck == nullptr){
    throw std::runtime_error("Error, add_dar_records_truck link cumulative curve tree is not installed");
  }

  MNM_Path* _path;
  for (auto path_it : link -> m_N_in_tree_truck -> m_record){
    _path = path_it.first;
	if (pathID_set.find(_path -> m_path_ID) != pathID_set.end()) {
      for (auto depart_it : path_it.second){
        TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
        if (tmp_flow > DBL_EPSILON){
          auto new_record = new dar_record();
          new_record -> path_ID = path_it.first -> m_path_ID;
          // the count of 1 min intervals, the vehicles record this assign_int
          new_record -> assign_int = depart_it.first;
          new_record -> link_ID = link -> m_link_ID;
          // the count of unit time interval (5s)
          new_record -> link_start_int = start_time;
          new_record -> flow = tmp_flow;
          // printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
          //     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
          record.push_back(new_record);
        }
      }
    }
  }
  return 0;
}

}//end namespace MNM_DTA_GRADIENT




/******************************************************************************************************************
*******************************************************************************************************************
											Multiclass Emissions
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Cumulative_Emission_Multiclass::MNM_Cumulative_Emission_Multiclass(TFlt unit_time, TInt freq)
	: MNM_Cumulative_Emission::MNM_Cumulative_Emission(unit_time, freq)
{
	m_fuel_truck = TFlt(0);
	m_CO2_truck = TFlt(0);
	m_HC_truck = TFlt(0);
	m_CO_truck = TFlt(0);
	m_NOX_truck = TFlt(0);
	m_VMT_truck = TFlt(0);

	m_VHT_car = TFlt(0);
	m_VHT_truck = TFlt(0);

	m_car_set = std::unordered_set<MNM_Veh*>();
	m_truck_set = std::unordered_set<MNM_Veh*>();
}

MNM_Cumulative_Emission_Multiclass::~MNM_Cumulative_Emission_Multiclass()
{
	;
}

// All convert_factors from MOVES
// Reference: MOVES default database - class 2b trucks with 4 tires
TFlt MNM_Cumulative_Emission_Multiclass::calculate_fuel_rate_truck(TFlt v)
{
	TFlt _convert_factor = 1.0;
	if (v < 25){
		_convert_factor = 1.53;
	}
	else if (v < 55){
		_convert_factor = 1.50;
	}
	else {
		_convert_factor = 1.55;
	}
	TFlt _fuel_rate_car = calculate_fuel_rate(v);
	TFlt _fuel_rate_truck = _fuel_rate_car * _convert_factor;
	return _fuel_rate_truck;
}

TFlt MNM_Cumulative_Emission_Multiclass::calculate_CO2_rate_truck(TFlt v)
{
	TFlt _convert_factor = 1.0;
	if (v < 25){
		_convert_factor = 1.53;
	}
	else if (v < 55){
		_convert_factor = 1.50;
	}
	else {
		_convert_factor = 1.55;
	}
	TFlt _CO2_rate_car = calculate_CO2_rate(v);
	TFlt _CO2_rate_truck = _CO2_rate_car * _convert_factor;
	return _CO2_rate_truck;
}

TFlt MNM_Cumulative_Emission_Multiclass::calculate_HC_rate_truck(TFlt v)
{
	TFlt _convert_factor = 1.0;
	if (v < 25){
		_convert_factor = 1.87;
	}
	else if (v < 55){
		_convert_factor = 2.41;
	}
	else {
		_convert_factor = 2.01;
	}
	TFlt _HC_rate_car = calculate_HC_rate(v);
	TFlt _HC_rate_truck = _HC_rate_car * _convert_factor;
	return _HC_rate_truck;
}

TFlt MNM_Cumulative_Emission_Multiclass::calculate_CO_rate_truck(TFlt v)
{
	TFlt _convert_factor = 1.0;
	if (v < 25){
		_convert_factor = 3.97;
	}
	else if (v < 55){
		_convert_factor = 2.67;
	}
	else {
		_convert_factor = 5.01;
	}
	TFlt _CO_rate_car = calculate_CO_rate(v);
	TFlt _CO_rate_truck = _CO_rate_car * _convert_factor;
	return _CO_rate_truck;
}

TFlt MNM_Cumulative_Emission_Multiclass::calculate_NOX_rate_truck(TFlt v)
{
	TFlt _convert_factor = 1.0;
	if (v < 25){
		_convert_factor = 7.32;
	}
	else if (v < 55){
		_convert_factor = 6.03;
	}
	else {
		_convert_factor = 5.75;
	}
	TFlt _NOX_rate_car = calculate_NOX_rate(v);
	TFlt _NOX_rate_truck = _NOX_rate_car * _convert_factor;
	return _NOX_rate_truck;
}

int MNM_Cumulative_Emission_Multiclass::update(MNM_Veh_Factory* veh_factory)
{
	// assume car truck the same speed on the same link when computing the emissions
	// possible to change to more accurate speeds for cars and trucks
	TFlt _v;
	TFlt _v_converted;
	for (MNM_Dlink *link : m_link_vector){
		MNM_Dlink_Multiclass *_mlink = dynamic_cast<MNM_Dlink_Multiclass *>(link);
		IAssert(_mlink != nullptr);
		_v = _mlink -> m_length / _mlink -> get_link_tt(); // m/s
		_v_converted = _v * TFlt(3600) / TFlt(1600); // mile / hour
		_v_converted = MNM_Ults::max(_v_converted, TFlt(5));
		_v_converted = MNM_Ults::min(_v_converted, TFlt(65));

		// cars
		m_fuel += calculate_fuel_rate(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_car();
		m_CO2 += calculate_CO2_rate(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_car();
		m_HC += calculate_HC_rate(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_car();
		m_CO += calculate_CO_rate(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_car();
		m_NOX += calculate_NOX_rate(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_car();
		m_VMT += (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_car();

		// trucks
		m_fuel_truck += calculate_fuel_rate_truck(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_truck();
		m_CO2_truck += calculate_CO2_rate_truck(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_truck();
		m_HC_truck += calculate_HC_rate_truck(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_truck();
		m_CO_truck += calculate_CO_rate_truck(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_truck();
		m_NOX_truck += calculate_NOX_rate_truck(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_truck();
		m_VMT_truck += (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_truck();

		// VHT (hours)
		m_VHT_car += m_unit_time * _mlink -> get_link_flow_car() / 3600;
		m_VHT_truck += m_unit_time * _mlink -> get_link_flow_truck() / 3600;

	}
	// trips
	MNM_Veh * _veh;
	MNM_Veh_Multiclass * _veh_multiclass;
	for (auto pair_it : veh_factory -> m_veh_map){
		_veh =  pair_it.second;
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);
		if (m_link_set.find(_veh_multiclass -> m_current_link) != m_link_set.end()){
			if (_veh_multiclass -> m_class == 0){
				m_car_set.insert(_veh_multiclass);
			}
			if (_veh_multiclass -> m_class == 1){
				m_truck_set.insert(_veh_multiclass);
			}		
		}
	}
	
	return 0;
}

std::string MNM_Cumulative_Emission_Multiclass::output()
{
	std::string _s = "";

	_s += "The emission stats for cars are: \n";
	_s += "fuel: " + std::to_string(m_fuel()) + " gallons, ";
	_s += "CO2: " + std::to_string(m_CO2()) + " g, ";
	_s += "HC: " + std::to_string(m_HC()) + " g, ";
	_s += "CO: " + std::to_string(m_CO()) + " g, ";
	_s += "NOX: " + std::to_string(m_NOX()) + " g, ";
	_s += "VMT: " + std::to_string(m_VMT()) + " miles, ";
	_s += "VHT: " + std::to_string(m_VHT_car()) + " hours, ";
	_s += "number of trips: " + std::to_string(int(m_car_set.size())) + " trips\n";

	_s += "The emission stats for trucks are: \n";
	_s += "fuel: " + std::to_string(m_fuel_truck()) + " gallons, ";
	_s += "CO2: " + std::to_string(m_CO2_truck()) + " g, ";
	_s += "HC: " + std::to_string(m_HC_truck()) + " g, ";
	_s += "CO: " + std::to_string(m_CO_truck()) + " g, ";
	_s += "NOX: " + std::to_string(m_NOX_truck()) + " g, ";
	_s += "VMT: " + std::to_string(m_VMT_truck()) + " miles, ";
	_s += "VHT: " + std::to_string(m_VHT_truck()) + " hours, ";
	_s += "number of trips: " + std::to_string(int(m_truck_set.size())) + " trips\n";

	printf("The emission stats for cars are: ");
	printf("fuel: %lf gallons, CO2: %lf g, HC: %lf g, CO: %lf g, NOX: %lf g, VMT: %lf miles, VHT: %lf hours, %d trips\n", 
		   m_fuel(), m_CO2(), m_HC(), m_CO(), m_NOX(), m_VMT(), m_VHT_car(), int(m_car_set.size()));

	printf("The emission stats for trucks are: ");
	printf("fuel: %lf gallons, CO2: %lf g, HC: %lf g, CO: %lf g, NOX: %lf g, VMT: %lf miles, VHT: %lf hours, %d trips\n", 
		   m_fuel_truck(), m_CO2_truck(), m_HC_truck(), m_CO_truck(), m_NOX_truck(), m_VMT_truck(), m_VHT_truck(), int(m_truck_set.size()));
	return _s;
}