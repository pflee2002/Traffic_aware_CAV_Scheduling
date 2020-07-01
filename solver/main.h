#ifndef _HD_DTA_H_
#define _HD_DTA_H_
#define _MAX_LABEL_COST 99999
#define _MAX_NUMBER_OF_NODES 1000
#define _MAX_NUMBER_OF_LINKS 20000
#define _MAX_NUM_OF_LANES 10 //at most 10 lanes for a link
#define _MAX_NUMBER_OF_LINKS_IN_PATH 999
#define _MAX_NUMBER_OF_NODES_IN_PATH 999
#define _MAX_NUMBER_OF_TIME_INTERVALS 120
#define _MAX_NUMBER_OF_PERIODS 1440 // Every minutes, 24 hours
#define _MAX_NUMBER_OF_ZONES 400
#define _MAX_NUMBER_OF_OUTBOUND_NODES 200
#define _MAX_NUMBER_OF_PROCESSORS 64
#define _MAX_NUM_OF_INTERSECTIONS 1
#define _MAX_NUMBER_OF_INTERSECTIONS 10
#define _MAX_NUMBER_OF_TIME_BIN 10//assume the shortest interval is 20 sec
#define _MAX_NUMBER_OF_VEHICLES 20
#define _MAX_NUM_OF_PATH_LINKS 999
#define _MAX_NUM_OF_QUEUES 200
#define _MAX_NUM_OF_PROCESSORS 64
#define _MAX_NUMBER_OF_PHASES 8
#define _M_ALPHA 0.95
#define _MIN_HEADWAY 1.5
#define _JAM_DENSITY 150
#define _MAX_NUMBER_OF_DETECTOR 9999
#define _MAX_NUMBER_OF_GENERALIZED_PHASES 16
#pragma comment(lib, "Ws2_32.lib")
using namespace std;
struct sig_attributes
{
	char ip[16];
	int port_no;
	int controller_id;
};
class sig_link_mapping
{
public:
	int int_id;
	int phase_id;
	std::string ip_address;
	int port_no;
	int stop_bar_link_id;
	sig_link_mapping()
	{
		ip_address.reserve(20);
	}
};
struct node_to_pt
{
	int node_id;
	int phase;
	int time;
};
struct static_path_link
{
	int link_id;
	int from_node_id;
	float link_travel_time;
	int to_node_id;
};
#include <winsock.h>
#include <map>
#include <vector>
#include <list>
#include <map>
#include <iostream>
#include "CSVParser.h"
#include <random>
#include <list>

using namespace std;
struct veh_position //relative_pos is from 0 to 1, standing for the distance from the from_node
{
	int veh_id;
	float relative_pos;
	int lane_id;
};
struct turning_counts
{
	int from_time_in_second;
	int to_time_in_second;
	int LT;
	int TH;
	int RT;
	map<string, long> turning_next_link_mapping;
};
struct turning_ratio
{
	int int_id;
	list<int> turning_nodes;
	list<turning_counts> turning_counts;
};
struct hist_link_counts
{
	int from_time_in_second;
	int to_time_in_second;
	int counts;
};
struct hist_TD_link_counts
{
	int link_id;
	float dist_to_link_end;
	list<hist_link_counts> TD_counts;
};
struct det_attributes
{
	int det_id;
	int local_det_id;
	int link_id;
	int tt_from_node;
	int sc_id;
};
struct sc_det_list
{
	int sc_id;
	vector<det_attributes> det_list;
};
//////////////////////////////////////////////////////////////////////////
//Variables for DTA module
template <typename T>
T **AllocateDynamicArray(int nRows, int nCols)
{
	T **dynamicArray;

	dynamicArray = new (std::nothrow) T*[nRows]();

	if (dynamicArray == NULL)
	{
		exit(1);
	}

	for (int i = 0; i < nRows; i++)
	{
		dynamicArray[i] = new (std::nothrow) T[nCols]();

		if (dynamicArray[i] == NULL)
		{
			exit(1);
		}
	}

	return dynamicArray;
}
template <typename T>
void DeallocateDynamicArray(T** dArray, int nRows, int nCols)
{
	if (!dArray)
		return;

	for (int x = 0; x < nRows; x++)
	{
		delete[] dArray[x];
	}

	delete[] dArray;

}
template <typename T>
T *AllocateDynamicVector(int nRows)
{
	T *dynamicVector;

	dynamicVector = new (std::nothrow) T[nRows]();

	if (dynamicVector == NULL)
	{
		exit(1);

	}
	return dynamicVector;
}
template <typename T>
void DeallocateDynamicVector(T* dVector, int nRows)
{
	if (!dVector)
		return;
	delete[] dVector;
}
template <typename T>
T ***Allocate3DDynamicArray(int nX, int nY, int nZ)
{
	T ***dynamicArray;

	dynamicArray = new (std::nothrow) T**[nX]();

	if (dynamicArray == NULL)
	{
		exit(1);
	}

	for (int x = 0; x < nX; x++)
	{
		//if (x % 1000 == 0)
		//{
		//	cout << "allocating 3D memory for " << x << endl;
		//}


		dynamicArray[x] = new (std::nothrow) T*[nY]();

		if (dynamicArray[x] == NULL)
		{
			exit(1);
		}

		for (int y = 0; y < nY; y++)
		{
			dynamicArray[x][y] = new (std::nothrow) T[nZ]();
			if (dynamicArray[x][y] == NULL)
			{
				exit(1);
			}
		}
	}

	return dynamicArray;

}
template <typename T>
void Deallocate3DDynamicArray(T*** dArray, int nX, int nY)
{
	if (!dArray)
		return;
	for (int x = 0; x < nX; x++)
	{
		for (int y = 0; y < nY; y++)
		{
			delete[] dArray[x][y];
		}

		delete[] dArray[x];
	}

	delete[] dArray;

}
template <typename T>
T ****Allocate4DDynamicArray(int nM, int nX, int nY, int nZ)
{
	T ****dynamicArray;

	dynamicArray = new (std::nothrow) T***[nX]();

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		exit(1);
	}
	for (int m = 0; m < nM; m++)
	{
		if (m % 100 == 0)
			//cout << "allocating 4D memory for " << m << endl;

		dynamicArray[m] = new (std::nothrow) T**[nX]();

		if (dynamicArray[m] == NULL)
		{
			cout << "Error: insufficient memory.";
			exit(1);
		}

		for (int x = 0; x < nX; x++)
		{
			dynamicArray[m][x] = new (std::nothrow) T*[nY]();

			if (dynamicArray[m][x] == NULL)
			{
				cout << "Error: insufficient memory.";
				exit(1);
			}

			for (int y = 0; y < nY; y++)
			{
				dynamicArray[m][x][y] = new (std::nothrow) T[nZ]();
				if (dynamicArray[m][x][y] == NULL)
				{
					cout << "Error: insufficient memory.";
					exit(1);
				}
			}
		}
	}
	return dynamicArray;

}
template <typename T>
void Deallocate4DDynamicArray(T**** dArray, int nM, int nX, int nY)
{
	if (!dArray)
		return;
	for (int m = 0; m < nM; m++)
	{
		for (int x = 0; x < nX; x++)
		{
			for (int y = 0; y < nY; y++)
			{
				delete[] dArray[m][x][y];
			}
			cout << x << endl;
			delete[] dArray[m][x];
		}
		delete[] dArray[m];
	}
	delete[] dArray;

}

  int sim_time_duration;
  int microsim_flag ;
  std::vector <sig_attributes> sig_list;
  std::map <int, sig_link_mapping> g_sig_link_map, g_int_link_map;
  std::map<int, int> g_signal_phase_map;
  std::map<int, int> g_signal_RT_status_map;
  std::vector<int> g_intersection_id_list;
  std::vector<sig_link_mapping> g_intersection_list;
  int g_sim_horizon;
  int total_counts;
  int sig_frequency;
  int flow_frequency ;
  std::map <string, int> g_node_name_to_id_map;//Taylor: reserved for future
  std::map <string, int> g_link_name_to_id_map;//Taylor: reserved for future
  std::map <string, int> g_int_name_to_id_map;//Taylor: reserved for future
  std::map <string, int> g_det_name_to_id_map;
  string g_from_link_id_to_get_link_name[_MAX_NUMBER_OF_LINKS];//Taylor: reserved for future
  string g_from_node_id_to_get_node_name[_MAX_NUMBER_OF_NODES];//Taylor: reserved for future
  string g_from_int_id_to_get_int_name[_MAX_NUMBER_OF_NODES];//Taylor: reserved for future
  string g_from_det_id_to_get_det_name[_MAX_NUMBER_OF_NODES];//Taylor: reserved for future
  map<int, node_to_pt> node_to_pt_map;
  int g_from_sig_link_to_stopbar_link[_MAX_NUMBER_OF_LINKS];//Taylor: reserved for future
  vector<int> g_stop_bar_link_list;
  float **total_time_link_travel_time;
  float *g_node_x;//Taylor added: Feb-2015
  float *g_node_y;//Taylor added: Feb-2015
  int *g_outbound_node_size;
  int *g_inbound_node_size;
  int **g_outbound_node_id;
  int **g_outbound_link_no;
  int *g_waiting_node_flag;
  int *g_waiting_node_ending_time;
  int **g_inbound_node_id;
  int **g_inbound_link_no;
  float g_link_free_flow_travel_time[_MAX_NUMBER_OF_LINKS];
  float g_link_cost[_MAX_NUMBER_OF_LINKS];
  float **g_external_link_time_dependent_toll;
  float g_link_link_length[_MAX_NUMBER_OF_LINKS];
  int g_link_number_of_lanes[_MAX_NUMBER_OF_LINKS];
  int g_link_mode_code[_MAX_NUMBER_OF_LINKS];
  int g_link_sim_mode[_MAX_NUMBER_OF_LINKS];//This array is to identify whether we will need to try lane-based location info of vehicles
  float **g_link_capacity_per_time_interval;
  float **g_link_capacity_per_time_interval_backup;//Taylor: time-dependent link capacity
  int g_link_jam_density[_MAX_NUMBER_OF_LINKS];
  float g_link_speed[_MAX_NUMBER_OF_LINKS];
  int g_link_from_node_number[_MAX_NUMBER_OF_LINKS];
  int g_link_to_node_number[_MAX_NUMBER_OF_LINKS];
  int g_vehicle_path_number_of_nodes[_MAX_NUMBER_OF_VEHICLES];
  int g_from_zone_id_to_production_attraction[_MAX_NUMBER_OF_ZONES][2];//Taylor added: [][0] is production; [][1] is attraction
  int g_vehicle_origin_node[_MAX_NUMBER_OF_VEHICLES];  // for vehcile routings
  int g_vehicle_origin_link[_MAX_NUMBER_OF_VEHICLES];  // for vehcile routings
  int g_vehicle_departure_time_beginning[_MAX_NUMBER_OF_VEHICLES];
  int g_vehicle_departure_time_ending[_MAX_NUMBER_OF_VEHICLES];
  int g_vehicle_destination_node[_MAX_NUMBER_OF_VEHICLES];
  int g_vehicle_destination_link[_MAX_NUMBER_OF_VEHICLES];
  int g_vehicle_arrival_time_beginning[_MAX_NUMBER_OF_VEHICLES];
  int g_vehicle_arrival_time_ending[_MAX_NUMBER_OF_VEHICLES];
  int g_path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
  int g_path_link_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
  int g_path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
  int g_from_node_id_to_get_zone_id[_MAX_NUMBER_OF_NODES];
  int g_from_node_id_to_get_x_id[_MAX_NUMBER_OF_NODES];
  int g_from_node_id_to_get_y_id[_MAX_NUMBER_OF_NODES];
  int g_if_a_node_within_scope[_MAX_NUMBER_OF_NODES] ;
  int g_path_number_of_nodes;
  float g_path_travel_time;
  int g_number_of_links=0 ;
  int g_number_of_nodes=0 ;
  int g_number_of_dets;
  int g_maximum_node_number ;
  int g_number_of_zones ;
  int g_number_of_activities ;
  int g_number_of_vehicles ;
  int g_number_of_toll_records ;
  int g_shortest_path_debugging_flag ;
  int **LinkCumulativeArrivalCount;
  int **LinkCumulativeDepartureCount;
  int LinkCumulativeArrivalCount_static[_MAX_NUMBER_OF_LINKS];
  int LinkCumulativeDepartureCount_static[_MAX_NUMBER_OF_LINKS];
  int Link_InFlowCapacity_static[_MAX_NUMBER_OF_LINKS];
  int Veh_current_link_index[_MAX_NUMBER_OF_VEHICLES];
  float g_node_x_coordinate[_MAX_NUMBER_OF_NODES] ;
  float g_node_y_coordinate[_MAX_NUMBER_OF_NODES] ;
  int g_number_of_intersections;
  int g_int_historical_turning_ratio_interval;
  int g_link_historical_counts_interval;
  int g_show_debug_info;
  //int **TD_queue_length;
  int TD_queue_length[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];
  float** LinkKjam;
  int** temp_LinkInFlowCapacity_by_processor;
  int** temp_LinkCumulativeArrivalCount_by_processor;
  SOCKET* ntcip_socket;
  struct sockaddr_in agent_addr;
  int flag_of_vehicle_reached_destination[_MAX_NUMBER_OF_VEHICLES] ;
  int flag_of_vehicle_enter_network[_MAX_NUMBER_OF_VEHICLES] ;
  int flag_of_vehicle_on_the_way[_MAX_NUMBER_OF_VEHICLES] ;
  int g_sim_vehicle_enter_time[_MAX_NUMBER_OF_VEHICLES] ;
  int g_sim_vehicle_finish_time[_MAX_NUMBER_OF_VEHICLES] ;
  int g_sim_vehicle_complete_flag[_MAX_NUMBER_OF_VEHICLES];
  vector<int> *vehicle_link_matrix_list;
  vector<int> *vehicle_node_matrix_list;
  vector<int> *vehicle_time_matrix_list;
  vector<int> *vehicle_link_matrix_backup_list;
  vector<int> *vehicle_node_matrix_backup_list;
  vector<int> *vehicle_time_matrix_backup_list;
  vector<int> *g_vehicle_path_link_sequence_list;
  vector<int> *g_vehicle_path_time_sequence_list;
  vector<int> *g_vehicle_path_node_sequence_list;
  int **LinkCumulativeArrival;
  int **LinkCumulativeDeparture;
  vector<int> *Veh_TA_list;
  vector<int> *Veh_TD_list;
  vector<int> *Veh_TV_list;
  map<int,int> *g_veh_reached_list;
  int g_from_node_of_link[_MAX_NUMBER_OF_LINKS] ;
  int g_to_node_of_link[_MAX_NUMBER_OF_LINKS] ;
  float **LinkOutFlowCapacity;
  //float **LinkOutFlowCapacity_AllLane;
  int **LinkInFlowCapacity;
  //int LinkInFlowCapacity[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];
  int Temp_LinkInFlowCapacity_increase[_MAX_NUMBER_OF_LINKS];
  int **link_open_status; //signal link status
  int **link_from_node_open_status; //signal link from node status
  int total_number_of_vehicles_in_the_system ;
  int g_free_flow_path_travel_time[_MAX_NUMBER_OF_VEHICLES] ;
  float g_vehicle_travel_distance[_MAX_NUMBER_OF_VEHICLES];
  int g_experienced_path_travel_time[_MAX_NUMBER_OF_VEHICLES] ;
  int g_non_stop_path_travel_time[_MAX_NUMBER_OF_VEHICLES] ;
  int g_free_flow_path_travel_time_backup[_MAX_NUMBER_OF_VEHICLES] ;
  int g_non_stop_path_travel_time_backup[_MAX_NUMBER_OF_VEHICLES] ;
  int g_experienced_path_travel_time_backup[_MAX_NUMBER_OF_VEHICLES] ;
  int g_min_travel_time ;
  int g_int_id_of_links[_MAX_NUMBER_OF_LINKS] ; //Taylor: each links's intersection if it is a signal link;
  float **g_link_resource_price;
  float **g_link_resource_vehicle_usage;
  int g_sig_link[_MAX_NUMBER_OF_LINKS];
  int g_number_of_sig_links;
  int g_regular_link[_MAX_NUMBER_OF_LINKS];
  int g_number_of_regular_links ;//Taylor: how many travel links in space-time network
  int **g_nodes_to_link_map;//You can get the link id using from node to to node
  int g_number_of_siglinks ;
  int g_sig_link_flag[_MAX_NUMBER_OF_LINKS] ;
  std::vector<int> g_sig_link_vector;
  int old_next_link_no[_MAX_NUMBER_OF_VEHICLES] ;
  unsigned long g_seed ;
  int sig_path_travel_time ;
  std::list<int> g_origin_dummy_list[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_PERIODS];
  std::vector<int> g_orgin_nodes_list;
  std::vector<int> g_destination_link_list;
  std::map<int, int> g_orgin_node_enter_link_map;
  std::map<string, int> g_config_map;
  std::multimap<string, vector<static_path_link>>g_static_path_map;
  int g_real_time_signal_call[_MAX_NUMBER_OF_INTERSECTIONS][17] ;//at each intersection, at most 16 phases
  int g_real_time_signal_status[_MAX_NUMBER_OF_INTERSECTIONS][17];//at each intersection, at most 16 phases 1. green; 0, otherwise. 
  int **g_sample_num_of_link;
  int node_id_flag[_MAX_NUMBER_OF_NODES] ;
  vector<int> g_vehicle_list;
   //time dependent turning ratios;
  map<int, turning_ratio> g_int_turning_counts;
	//time dependent link counts;
  map<int, hist_TD_link_counts> g_hist_link_counts;
  //inbound mapping
  map<int, vector<int>> g_link_merging_links_map;

  //time-dependent turning ratio;
  map<string, map<int, turning_counts>> g_TD_turning_counts;

  //detectors
  vector<int> g_sc_list;
  vector<int> g_link_with_dets_list;
  std::map<int, sc_det_list> g_sc_det_map_list;
  std::map<int, sc_det_list> g_link_det_map_list;
  std::map<int, sc_det_list> g_det_link_map_list;
  int g_link_with_dets[_MAX_NUMBER_OF_LINKS];
  //int **g_dets_counts;
  int g_dets_counts[_MAX_NUMBER_OF_DETECTOR][_MAX_NUMBER_OF_TIME_INTERVALS];

/*======================================================*/
void g_ReadInputNode(std::string FileName)
{
	CCSVParser parser;
	if (parser.OpenCSVFile(FileName, false))
	{
		std::map<int, int> node_id_map;
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			string name;
			int node_type;
			int node_id;
			double X;
			double Y;
			if (parser.GetValueByFieldName("node_id", node_id) == false)
				continue;
			g_number_of_nodes++;
			node_id_flag[g_number_of_nodes] = 1;
			parser.GetValueByFieldName("node_type", node_type);
			parser.GetValueByFieldName("p", X);
			g_node_x_coordinate[g_number_of_nodes] = X;
			parser.GetValueByFieldName("t", Y);
			g_node_y_coordinate[g_number_of_nodes] = Y;
			parser.GetValueByFieldName("waiting_flag", g_waiting_node_flag[node_id]);
			node_to_pt temp;
			temp.node_id = g_number_of_nodes;
			temp.phase = X;
			temp.time = Y;
			node_to_pt_map.insert(pair<int, node_to_pt>(g_number_of_nodes, temp));
			g_node_name_to_id_map.insert(pair<string,int>(to_string(node_id),g_number_of_nodes));//Taylor: reserved for future
			g_from_node_id_to_get_node_name[g_number_of_nodes]=to_string(node_id);//Taylor: reserved for future
			if (g_number_of_nodes % 1000 == 0)
			{
				cout << floor((g_number_of_nodes + 1) / 1000) << "k nodes have been loaded...\n";
			}
		}

		parser.CloseCSVFile();
	}
	else
	{
		exit(1);
	}
}
void g_ReadInputLink(std::string FileName)
{
	// initialization
	for (int i = 0; i < _MAX_NUMBER_OF_NODES; i++)
	{
		g_outbound_node_size[i] = 0;
		g_inbound_node_size[i] = 0;
	}
	CCSVParser parser;
	if (parser.OpenCSVFile(FileName, false))
	{
		std::map<int, int> node_id_map;
		parser.m_bDataHubSingleCSVFile = true;
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int from_node_id = 0;
			int to_node_id = 0;
			int sig_link_flag = 0;
			if (parser.GetValueByFieldName("from_node_id", from_node_id) == false)
				continue;
			if (parser.GetValueByFieldName("to_node_id", to_node_id) == false)
				continue;
			if (g_node_name_to_id_map[to_string(from_node_id)] <= 0 || g_node_name_to_id_map[to_string(from_node_id)] >= _MAX_NUMBER_OF_NODES)
			{
				cout << "from_node_id " << from_node_id << " is out of range" << endl;
				exit(1);
			}
			if (g_node_name_to_id_map[to_string(to_node_id)] <= 0 || g_node_name_to_id_map[to_string(to_node_id)] >= _MAX_NUMBER_OF_NODES)
			{
				cout << "to_node_id " << to_node_id << " is out of range" << endl;
				exit(1);
			}

			if (node_id_flag[g_node_name_to_id_map[to_string(from_node_id)]] != 1)
			{
				cout << "from_node_id " << from_node_id << " has not been defined in node block" << endl;
				exit(1);
			}

			if (node_id_flag[g_node_name_to_id_map[to_string(from_node_id)]] != 1)
			{
				cout << "to_node_id " << to_node_id << " has not been defined in node block" << endl;
				exit(1);
			}
			int direction = 1;
			parser.GetValueByFieldName("direction", direction);

			if (direction <= -2 || direction >= 2)
			{
				cout << "direction " << direction << " is out of range" << endl;
				exit(1);
			}
			for (int link_direction = -1; link_direction <= 1; link_direction += 2)  // called twice; -1 direction , 1 direction 
			{
				if (direction == -1 && link_direction == 1)
					continue; // skip
				if (direction == 1 && link_direction == -1)
					continue; // skip

				int directional_from_node_id = g_node_name_to_id_map[to_string(from_node_id)];
				int directional_to_node_id = g_node_name_to_id_map[to_string(to_node_id)];
				if (link_direction == -1) // reverse direction;
				{
					directional_from_node_id = g_node_name_to_id_map[to_string(from_node_id)];
					directional_to_node_id = g_node_name_to_id_map[to_string(to_node_id)];
				}
				float link_length = 1;
				int number_of_lanes = 1;
				int mode_code = 0;
				float capacity_per_time_interval = 1;
				float travel_time = 1;
				float speed = 1;
				int jam_density = 200;
				int siglinkflag = 0;
				int link_id;
				int sim_mode=0;
				parser.GetValueByFieldName("cost", link_length);
				parser.GetValueByFieldName("number_of_lanes", number_of_lanes);
				parser.GetValueByFieldName("mode_code", mode_code);
				parser.GetValueByFieldName("lane_capacity_in_vhc_per_hour", capacity_per_time_interval);
				parser.GetValueByFieldName("speed_limit", speed);
				if (g_number_of_links == 32)
					int x = 1;
				if (parser.GetValueByFieldName("travel_time", travel_time) == false)
					travel_time = (link_length * 5280) / (speed * 5280 / 3600); //second
				parser.GetValueByFieldName("jam_density", jam_density);
				parser.GetValueByFieldName("sig_link_flag", siglinkflag);
				parser.GetValueByFieldName("link_id", link_id);
				parser.GetValueByFieldName("sim_mode", sim_mode);
				// increase the link counter by 1
				g_number_of_links++;
				g_link_cost[g_number_of_links] = link_length;
				g_from_node_of_link[g_number_of_links] = g_node_name_to_id_map[to_string(from_node_id)];//Taylor added: Dec-2014
				g_to_node_of_link[g_number_of_links] = g_node_name_to_id_map[to_string(to_node_id)];//Taylor added: Dec-2014;//Taylor added: Dec-2014
				g_link_sim_mode[g_number_of_links] = sim_mode;
				g_outbound_node_id[directional_from_node_id][g_outbound_node_size[directional_from_node_id]] = directional_to_node_id;
				//g_outbound_link_no[directional_from_node_id][g_outbound_node_size[directional_from_node_id]] = g_number_of_links;
				g_outbound_link_no[directional_from_node_id][g_outbound_node_size[directional_from_node_id]] = g_number_of_links;//Taylor revised: Dec-2014
				g_outbound_node_size[directional_from_node_id]++;
				g_inbound_node_id[directional_to_node_id][g_inbound_node_size[directional_to_node_id]] = directional_from_node_id;
				//g_inbound_link_no[directional_to_node_id][g_inbound_node_size[directional_to_node_id]] = g_number_of_links;
				g_inbound_link_no[directional_to_node_id][g_inbound_node_size[directional_to_node_id]] = g_number_of_links;//Taylor revised: Dec-2014
				g_inbound_node_size[directional_to_node_id]++;
				if (siglinkflag)
				{
					g_sig_link_flag[g_number_of_links] = 1;
				}
				//g_sig_link_vector.push_back(link_id);
				g_link_from_node_number[g_number_of_links] = directional_from_node_id;
				g_link_to_node_number[g_number_of_links] = directional_to_node_id;
				
				g_link_link_length[g_number_of_links] = link_length;
				g_link_jam_density[g_number_of_links] = jam_density;
				g_link_number_of_lanes[g_number_of_links] = number_of_lanes;
				g_link_mode_code[g_number_of_links] = mode_code;
				g_link_speed[g_number_of_links] = speed;
			}
			if (g_number_of_links % 1000 == 0)
			{
				cout << floor((g_number_of_links + 1) / 1000) << "k links have been loaded...\n";
			}
		}
		parser.CloseCSVFile();
	}
	else
	{
		exit(1);
	}
}
void initialization();

#endif // !_HD_DTA_H_