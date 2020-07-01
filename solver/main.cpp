#ifndef HD_DTA_H_
#define HD_DTA_H_
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define HORIZON 120
#include <WinSock2.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm> 
#include <random>
#include <cstdlib>      // std::rand, std::srand
#include <fstream>
#include <math.h>
#include <random>
#include <cmath>
#include <bitset>
#include <stdio.h>
#include <map>
#include "main.h"
#include "CSVParser.h"
#include <ctime>
#include <omp.h>
#include <time.h>
#include <fstream>
#include <direct.h>
#include <bitset>
#include <time.h>
#include <list>
//#include "mysql.h"
#include <stdexcept>   // for exception, runtime_error, out_of_range
#include <stdio.h>
#include <string>
#include <iostream>
//#pragma comment(lib, "ws2_32.lib")
void g_Read_PT_config(std::string FileName);
using namespace std;
struct tsp_info
{
	int tsp_id;
	int phase_id;
	int start_time;
	int end_time;
};
vector<int> phase_list;
int max_green, min_green;
vector<tsp_info> tsp_list;
int clone_arc_cost;
int tsp_service_arc_cost;
int tsp_special_service_arc_cost;
int phase_volume[4];
class traffic_state //#for constructing A-D curve in phase time network
{
public:
	int horizon;
	int A[4][HORIZON];
	int D[4][HORIZON];
	int total_delay;
	traffic_state::traffic_state()
	{
		horizon = 120;
		for (int p = 0; p < 4; ++p)
		{
			for (int idx = 0; idx < HORIZON; idx++)
			{
				A[p][idx] = 0;
				D[p][idx] = 0;
				total_delay = 0;
			}

		}

	}
	int delay_calculation(int tau)
	{
		int total_delay_A = 0;
		int total_delay_D = 0;
		for (int p = 0; p < 4; p++)
		{
			for (int idx = 1; idx <= min(horizon-1,tau); idx++)
			{
				int N = D[p][idx];
				total_delay_D = total_delay_D + N;
			}
				
			for (int idx1 = 1; idx1 <= min(horizon - 1, tau); idx1++)
			{
				int N = A[p][idx1];
				total_delay_A = total_delay_A + N;
			}
			
		}
		return total_delay_A-total_delay_D;
	}
	void AD_propagation(int link_no)
	{
		int from_node = g_from_node_of_link[link_no];
		int to_node = g_to_node_of_link[link_no];
		int p = node_to_pt_map[from_node].phase;
		int t = node_to_pt_map[from_node].time;
		int pp = node_to_pt_map[to_node].phase;
		int h = node_to_pt_map[to_node].time;
		int dt = h - t;
		float lamda[4];
		lamda[0] = (float)phase_volume[0] / 3600.0;
		lamda[1] = (float)phase_volume[1] / 3600.0;
		lamda[2] = (float)phase_volume[2] / 3600.0;
		lamda[3] = (float)phase_volume[3] / 3600.0;
		unsigned long old_seed;
		old_seed = g_seed;
		//A curve update
		for (int phase = 0; phase < 4; ++phase)
		{
			for (int tt = t+1; tt <= min(horizon-1,h); ++tt) //2 is inter-green
			{
				A[phase][tt] = A[phase][tt-1];
				D[phase][tt] = D[phase][tt - 1];//at this moment, no vehicles are released yet. 
				//lamda[phase] = (float)phase_volume[phase] / 3600.0;
				while (lamda[phase] > 1)
				{
					A[phase][tt] = A[phase][tt] + 1;
					lamda[phase]--;
				}
				if (lamda[phase] > 0 && lamda[phase] < 1)
				{
					g_seed = (g_seed * 16807) % 2147483647;// 
					//float u = g_seed / 2147483647.0; //random number between 0 to 1
					float u = ((double)rand() / (RAND_MAX));
					if (u < lamda[phase])
					{
						A[phase][tt]++;
						lamda[phase]= (float)phase_volume[phase] / 3600.0; //reset
					}
					else
					{
						A[phase][tt] = A[phase][tt - 1];
						lamda[phase] = lamda[phase] + 0.5*phase_volume[phase] / 3600.0; //uniform. hazard function calculation
					}
				}
			}
		}
		//D curve update, only phase p is updated
		g_seed = old_seed;
		for (int tt = t + 1; tt <= min(horizon-1, h); ++tt)
		{
			float dept_veh = 0.5;
			D[p-1][tt] = D[p-1][tt - 1];
			if (D[p - 1][tt - 1] == 0)
				int x = 1;
			while (dept_veh >= 1)
			{
				D[p-1][tt] = min(A[p-1][tt], D[p-1][tt] + 1);
				dept_veh--;
			}
			if (dept_veh > 0 && dept_veh < 1)
			{
				g_seed = (g_seed * 16807) % 2147483647;// 
				//float u = g_seed / 2147483647.0; //random number between 0 to 1
				float u = ((double)rand() / (RAND_MAX));
				if (u < dept_veh)
				{
					D[p - 1][tt] = min(A[p - 1][tt], D[p - 1][tt - 1] + 1);
					dept_veh = 1800/3600+0.1;
				}
				else
				{
					dept_veh = dept_veh + 1800/3600+0.1;
				}
				
			}			
		}
	}
};
int arc_delay(traffic_state new_state, int t2,traffic_state old_state, int t1)
{
	return new_state.delay_calculation(t2) - old_state.delay_calculation(t1);
};
float g_phase_time_DP(int origin);
int main()
{
	phase_volume[0] = 150;
	phase_volume[1] = 700;
	phase_volume[2] = 200;
	phase_volume[3] = 1100;
	initialization();
	g_phase_time_DP(1);
	return 0;

}

void g_Read_PT_config(std::string FileName)
{
	string line;
	int row = 1;
	//std::ifstream file_input(FileName);
	std::ifstream file_input;
	file_input.open(FileName.c_str(), std::ifstream::in);
	std::string token;
	while (file_input) //It is different from visual studio
	{
		string s;
		std::istringstream ss(s);
		while (ss)
		{
			if (!getline(file_input, s, '\n'))//reach the file end
				break;
			std::istringstream ss1(s);
			std::string token;
			string section_name;
			string m_value;
			for (int i = 0; i <= 1; i++)
			{
				getline(ss1, token, '=');
				if (i == 0)
					section_name = token;
				else
					m_value = token.c_str();
			}
			if (section_name == "phasing")
			{
				std::istringstream ss2(m_value);
				std::string token;
				while (1)
				{
					getline(ss2, token, ';');
					if (token == "")
					{
						break;
					}
					phase_list.push_back(atoi(token.c_str()));
				}
			}
			else if (section_name == "min_green")
			{
				std::istringstream ss2(m_value);
				std::string token;
				getline(ss2, token, ';');
				min_green = atoi(token.c_str());
			}
			else if (section_name == "max_green")
			{
				std::istringstream ss2(m_value);
				std::string token;
				getline(ss2, token, ';');
				max_green = atoi(token.c_str());
			}
			else if (section_name == "TSP_list")
			{
				std::istringstream ss2(m_value);
				std::string token;
				while (1)
				{
					getline(ss2, token, ';');
					if (token == "")
					{
						break;
					}
					std::istringstream ss3(token);
					std::string token1;
					tsp_info temp;
					for (int i = 0; i <= 3; i++)
					{
						getline(ss3, token1, ',');
						if (i == 0)
							temp.tsp_id = atoi(token1.c_str());
						else if (i == 1)
							temp.phase_id = atoi(token1.c_str());
						else if (i == 2)
							temp.start_time = atoi(token1.c_str());
						else if (i == 3)
							temp.end_time = atoi(token1.c_str());
					}
					tsp_list.push_back(temp);
				}
			}
			else if (section_name == "TSP_service_arc_cost")
			{
				std::istringstream ss2(m_value);
				std::string token;
				getline(ss2, token, ';');
				tsp_service_arc_cost = atoi(token.c_str());

			}
			else if (section_name == "clone_arc_cost")
			{
				std::istringstream ss2(m_value);
				std::string token;
				getline(ss2, token, ';');
				clone_arc_cost = atoi(token.c_str());
			}
			else if (section_name == "TSP_special_service_arc_cost")
			{
				std::istringstream ss2(m_value);
				std::string token;
				getline(ss2, token, ';');
				tsp_special_service_arc_cost = atoi(token.c_str());
			}
		}
	}
	file_input.close();
}
void initialization()
{
	//Read Input files
	g_seed = 1;
	const char* a_cwd = _getcwd(NULL, 0);
	std::string file_path(a_cwd);
	std::string file_path1(a_cwd);
	std::string file_path2(a_cwd);
	std::string file_path3(a_cwd);
	std::string file_path4(a_cwd);
	std::string file_path5(a_cwd);
	std::string file_path6(a_cwd);
	std::string file_path7(a_cwd);
	std::string file_path8(a_cwd);
	file_path = file_path  + "\\input_link.csv";
	file_path1 = file_path1 + "\\input_node.csv";
	a_cwd = file_path.c_str();
	//memory initialization for global variables
	g_inbound_node_size = AllocateDynamicVector<int>(_MAX_NUMBER_OF_NODES);
	g_outbound_node_size = AllocateDynamicVector<int>(_MAX_NUMBER_OF_NODES);
	g_outbound_node_id = AllocateDynamicArray<int>(_MAX_NUMBER_OF_NODES, _MAX_NUMBER_OF_OUTBOUND_NODES);
	g_outbound_link_no = AllocateDynamicArray<int>(_MAX_NUMBER_OF_NODES, _MAX_NUMBER_OF_OUTBOUND_NODES);
	g_waiting_node_flag = AllocateDynamicVector<int>(_MAX_NUMBER_OF_NODES);
	g_waiting_node_ending_time = AllocateDynamicVector<int>(_MAX_NUMBER_OF_NODES);
	g_inbound_node_id = AllocateDynamicArray<int>(_MAX_NUMBER_OF_NODES,_MAX_NUMBER_OF_OUTBOUND_NODES); 
	//TD_queue_length = AllocateDynamicArray<int>(_MAX_NUMBER_OF_LINKS, _MAX_NUMBER_OF_TIME_INTERVALS);
	g_inbound_link_no=AllocateDynamicArray<int>(_MAX_NUMBER_OF_NODES,_MAX_NUMBER_OF_OUTBOUND_NODES);


	g_ReadInputNode(file_path1);
	g_ReadInputLink(file_path);
}

float g_phase_time_DP(int origin)
{
	int destination = 941;
	float travel_time_return_value = 0;
	float arc_cost[_MAX_NUMBER_OF_LINKS];
	int path_node_sequence[_MAX_NUMBER_OF_NODES];
	float node_label_cost[_MAX_NUMBER_OF_NODES] = { 0 };
	int node_predecessor[_MAX_NUMBER_OF_NODES] = { 0 };
	int time_predecessor[_MAX_NUMBER_OF_NODES] = { 0 };
	traffic_state node_state[1000];
	for (int i = 0; i <= g_number_of_nodes; i++) //Initialization for all nodes
	{
		node_label_cost[i] = _MAX_LABEL_COST;
		node_predecessor[i] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
		time_predecessor[i] = -1;
	}
	for (int l = 1; l <= g_number_of_links; l++)//initial cost without considering the background delay cost
	{
		arc_cost[l] = g_link_cost[l];
	}
	node_label_cost[origin] = 0;
	list<int> open_list;
	open_list.push_back(origin);
	int last_update_node = 0;
	while (!open_list.empty())
	{
		int from_node = open_list.front();
		open_list.pop_front();
		unsigned long common_seed = g_seed;
		for (int i = 0; i < g_outbound_node_size[from_node]; ++i)
		{

			int link_no = g_outbound_link_no[from_node][i];
			int to_node = g_outbound_node_id[from_node][i];

			traffic_state new_state = node_state[from_node];
			int t = node_to_pt_map[from_node].time;
			int p = node_to_pt_map[from_node].phase;
			int h = node_to_pt_map[to_node].time;
			int pp = node_to_pt_map[to_node].phase;
			g_seed = common_seed;
			if (to_node == 941)
				int x = 1;
			new_state.AD_propagation(link_no);

			int delay_cost = arc_delay(new_state,h, node_state[from_node],t);
			//delay_cost = 1;
			int total_arc_cost = delay_cost + g_link_cost[link_no];
			//total_arc_cost = 1;
			int temp_label_cost = node_label_cost[from_node]+total_arc_cost;
			if (temp_label_cost < (int)node_label_cost[to_node]) //a better solution
			{
				node_label_cost[to_node] = temp_label_cost;
				node_predecessor[to_node] = from_node;
				node_state[to_node] = new_state;
				last_update_node = to_node;
				if (g_outbound_node_size[to_node] > 0 && to_node != destination)
				{
					last_update_node = to_node;
					if (last_update_node == 452)
						int x = 1;
					if (to_node == 941 || h==120)
						int x = 1;
					open_list.push_back(to_node);
				}
			}
		}
	}
	int reversed_path_node_sequence[_MAX_NUMBER_OF_NODES];
	int	node_size = 0;
	reversed_path_node_sequence[node_size] = destination;//record the first node backward, destination node
	node_size++;
	int pred_node = node_predecessor[destination];
	int pred_time = time_predecessor[destination];
	while (pred_node != -1 && node_size < _MAX_NUMBER_OF_NODES)
	{
		reversed_path_node_sequence[node_size] = pred_node;
		node_size++;
		//record current values of node and time predecessors, and update PredNode and PredTime
		int pred_node_record = pred_node;
		int pred_time_record = pred_time;
		pred_node = node_predecessor[pred_node_record];
	}
	ofstream myfile;
	myfile.open("example.txt");
	for (int n = 0; n < node_size; n++)
	{
		path_node_sequence[n] = reversed_path_node_sequence[node_size - n - 1];
		int node= reversed_path_node_sequence[node_size - n - 1];
		int p = node_to_pt_map[node].phase;
		int t = node_to_pt_map[node].time;
		myfile << p << "," << t << endl;
	}
	myfile.close();
	myfile.open("ad_curves.csv");
	for (int p = 0; p < 4; p++)
	{
		myfile << "phase " << p + 1 << endl;
		for (int idx = 0; idx < HORIZON; ++idx)
		{
			myfile << node_state[941].A[p][idx]<<",";
		}
		myfile << endl;
		for (int idx = 0; idx < HORIZON; ++idx)
		{
			myfile << node_state[941].D[p][idx] << ",";
		}
		myfile << endl;
	}
	myfile.close();
	return 0;
}

#endif