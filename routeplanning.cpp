// -*- C++ -*-
/*!
 * @file  routeplanning.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "routeplanning.h"
#include <stdio.h>
#include <utility>
#include <queue>
#include <vector>
#include <list>
#include <windows.h>
#include <iostream>

// Module specification
// <rtc-template block="module_spec">
static const char* routeplanning_spec[] =
  {
    "implementation_id", "routeplanning",
    "type_name",         "routeplanning",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "KK",
    "category",          "routeplanning",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
routeplanning::routeplanning(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_mapIn("map", m_map),
    m_current_velocityIn("current_velocity", m_current_velocity),
    m_positionIn("position", m_position),
    m_velocityOut("velocity", m_velocity)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
routeplanning::~routeplanning()
{
}



RTC::ReturnCode_t routeplanning::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("map", m_mapIn);
  addInPort("current_velocity", m_current_velocityIn);
  addInPort("position", m_positionIn);
  
  // Set OutPort buffer
  addOutPort("velocity", m_velocityOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t routeplanning::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t routeplanning::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t routeplanning::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t routeplanning::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t routeplanning::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

struct Edge
{
    Edge(int v, float c)
    {
        ToNodeId = v;
        Cost = c;
    }

    int ToNodeId;	// 曈偺廔揰捀揰Id
    float Cost;		// 廳傒
};

// Pair偺first偲second偺堄枴傪朰傟偑偪偵側傞偺偱崱夞偼typedef偱堄枴傪愝掕
typedef int ShortestDistance;
typedef int NodeId;
typedef std::pair<ShortestDistance, NodeId> Pair;	// first:嵟抁嫍棧 second:捀揰斣崋

const int Infinity = 100000;	// 嵟抁嫍棧偺嵟戝抣
const int NodeNum = 4800;			// 捀揰悢

std::vector<Edge> g_Nodes[NodeNum];	// 捀揰攝楍

void InitEdge(double map[80][60][3])
{
	// (first, second) => (捀揰斣崋丄嵟抁嫍棧)
	//四个顶点
	//(0,0)顶点
	if (map[0][0][2] == 1 && map[0][1][2] == 1)
	{
		g_Nodes[0].emplace_back(81, 1);
	}
	if (map[0][0][2] == 1 && map[1][0][2] == 1)
	{
		g_Nodes[0].emplace_back(1, 1);
	}
	if (map[0][0][2] == 1 && map[1][1][2] == 1)
	{
		g_Nodes[0].emplace_back(82, 1.4);
	}
	//(0,59)顶点
	if (map[0][59][2] == 1 && map[0][58][2] == 1)
	{
		g_Nodes[4720].emplace_back(4640, 1);
	}
	if (map[0][59][2] == 1 && map[1][59][2] == 1)
	{
		g_Nodes[4720].emplace_back(4721, 1);
	}
	if (map[0][59][2] == 1 && map[1][58][2] == 1)
	{
		g_Nodes[4720].emplace_back(4641, 1.4);
	}
	//(79,0)顶点
	if (map[79][0][2] == 1 && map[78][0][2] == 1)
	{
		g_Nodes[79].emplace_back(78, 1);
	}
	if (map[79][0][2] == 1 && map[79][1][2] == 1)
	{
		g_Nodes[79].emplace_back(159, 1);
	}
	if (map[79][0][2] == 1 && map[78][1][2] == 1)
	{
		g_Nodes[79].emplace_back(158, 1.4);
	}
	//(79,59)顶点
	if (map[79][59][2] == 1 && map[79][58][2] == 1)
	{
		g_Nodes[4799].emplace_back(4719, 1);
	}
	if (map[79][59][2] == 1 && map[78][59][2] == 1)
	{
		g_Nodes[4799].emplace_back(4798, 1);
	}
	if (map[79][59][2] == 1 && map[78][58][2] == 1)
	{
		g_Nodes[4799].emplace_back(4718, 1.4);
	}
	//左边界
	for (int i = 0; i < 1; i++)
	{
		for (int j = 1; j < 59; j++)
		{
			if (map[i][j][2] == 1 && map[i + 1][j][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + j * 80 + 1, 1);
			}
			if (map[i][j][2] == 1 && map[i + 1][j - 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j - 1) * 80 + 1, 1.4);
			}
			if (map[i][j][2] == 1 && map[i + 1][j + 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j + 1) * 80 + 1, 1.4);
			}
		}
	}
	//右边界
	for (int i = 79; i < 80; i++)
	{
		for (int j = 1; j < 59; j++)
		{
			if (map[i][j][2] == 1 && map[i - 1][j][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + j * 80 - 1, 1);
			}
			if (map[i][j][2] == 1 && map[i - 1][j - 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j - 1) * 80 - 1, 1.4);
			}
			if (map[i][j][2] == 1 && map[i - 1][j + 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j + 1) * 80 - 1, 1.4);
			}
		}
	}
	//上边界
	for (int i = 1; i < 79; i++)
	{
		for (int j = 59; j < 60; j++)
		{
			if (map[i][j][2] == 1 && map[i][j - 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j - 1) * 80, 1);
			}
			if (map[i][j][2] == 1 && map[i - 1][j - 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j - 1) * 80 - 1, 1.4);
			}
			if (map[i][j][2] == 1 && map[i + 1][j - 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j - 1) * 80 + 1, 1.4);
			}
		}
	}
	//下边界
	for (int i = 1; i < 79; i++)
	{
		for (int j = 0; j < 1; j++)
		{
			if (map[i][j][2] == 1 && map[i][j + 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j + 1) * 80, 1);
			}
			if (map[i][j][2] == 1 && map[i - 1][j + 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j + 1) * 80 - 1, 1.4);
			}
			if (map[i][j][2] == 1 && map[i + 1][j + 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j + 1) * 80 + 1, 1.4);
			}
		}
	}
	//其他
	for (int i = 1; i < 79; i++)
	{
		for (int j = 1; j < 59; j++)
		{
			if (map[i][j][2] == 1 && map[i - 1][j][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + j * 80 - 1, 1);
			}
			if (map[i][j][2] == 1 && map[i - 1][j - 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j - 1) * 80 - 1, 1.4);
			}
			if (map[i][j][2] == 1 && map[i - 1][j + 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j + 1) * 80 - 1, 1.4);
			}
			if (map[i][j][2] == 1 && map[i][j + 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j + 1) * 80, 1);
			}
			if (map[i][j][2] == 1 && map[i][j - 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j - 1) * 80, 1);
			}
			if (map[i][j][2] == 1 && map[i + 1][j][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + j * 80 + 1, 1);
			}
			if (map[i][j][2] == 1 && map[i + 1][j + 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j + 1) * 80 + 1, 1.4);
			}
			if (map[i][j][2] == 1 && map[i + 1][j - 1][2] == 1)
			{
				g_Nodes[i + j * 80].emplace_back(i + (j - 1) * 80 + 1, 1.4);
			}
		}
	}
}
//移动函数
void move(int nodenum[4800] , double target_velocity_in[2] )
{
	for (int i = 1; i < 4800; i++)
	{
		if (nodenum[i] == nodenum[i - 1])
		{
			break;
		}
		else
		{
			if (nodenum[i] - nodenum[i - 1] == 1)//向右
			{
				target_velocity_in[0] = 1;
				target_velocity_in[1] = 0;
				Sleep(10);
			}
			if (nodenum[i] - nodenum[i - 1] == -1)//向左
			{
				target_velocity_in[0] = -1;
				target_velocity_in[1] = 0;
				Sleep(10);
			}
			if (nodenum[i] - nodenum[i - 1] == 80)//向上
			{
				target_velocity_in[0] = 0;
				target_velocity_in[1] = 1;
				Sleep(10);
			}
			if (nodenum[i] - nodenum[i - 1] == -80)//向下
			{
				target_velocity_in[0] = 0;
				target_velocity_in[1] = -1;
				Sleep(10);
			}
			if (nodenum[i] - nodenum[i - 1] == 79)//向左上
			{
				target_velocity_in[0] = -1;
				target_velocity_in[1] = 1;
				Sleep(10);
			}
			if (nodenum[i] - nodenum[i - 1] == 81)//向右上
			{
				target_velocity_in[0] = 1;
				target_velocity_in[1] = 1;
				Sleep(10);
			}
			if (nodenum[i] - nodenum[i - 1] == -79)//向右下
			{
				target_velocity_in[0] = 1;
				target_velocity_in[1] = -1;
				Sleep(10);
			}
			if (nodenum[i] - nodenum[i - 1] == -81)//向左下
			{
				target_velocity_in[0] = -1;
				target_velocity_in[1] = -1;
				Sleep(10);
			}
		}
	}
}

void Dijkstra(int start, int goal, double velocity[2] )
{
	int last_update_node_ids[NodeNum];											// 奺捀揰嫍棧偺嵟屻偵曄峏偟偨捀揰ID曐懚梡
	std::fill(last_update_node_ids, last_update_node_ids + NodeNum, Infinity);	// 枹應検忬懺偲偟偰弶婜壔

	int distances[NodeNum];									// 奺捀揰偺嵟抁嫍棧曐懚梡攝楍
	std::fill(distances, distances + NodeNum, Infinity);	// 枹寁應忬懺偲偟偰弶婜壔
	distances[start] = 0;									// 扵嶕奐巒埵抲偲偟偰弶婜壔

	/*
		寁應岓曗捀揰曐懚梡偺僉儏乕
			桪愭弴埵偼嵟抁嫍棧偑嵟傕崅偔丄摨偠応崌偼捀揰斣崋偺彫偝偄曽偲偡傞

		桪愭弴埵晅偒僉儏乕
			Pair傪巊偭偨応崌僜乕僩偼first傪婎弨偵峴偆
			first偑摨偠抣偺応崌偼second偱僜乕僩偝傟傞
	*/
	std::priority_queue<
		Pair,					// 梫慺偼Pair<int, int>
		std::vector<Pair>,		// 僐儞僥僫偺宆 vector傪巊梡
		std::greater<Pair>		// 徃弴偱僜乕僩偡傞(僜乕僩曽朄偺帺嶌壜擻)
	> survey_node_que;

	// 應検岓曗捀揰偲偟偰巜掕偝傟偨捀揰傪巜掕
	survey_node_que.push(Pair(0, start));

	while (survey_node_que.empty() == false)
	{
		/*
			寁應岓曗僉儏乕偺愭摢偐傜捀揰傪庢摼
			偙偺捀揰偑巒揰丄曐懚偝傟偰傞奺曈偵搊榐偝傟偰偄傞
			捀揰偑廔揰偲偟偰應検傪峴偆
		*/
		Pair node = survey_node_que.top();
		// 愭摢偺嶍彍
		survey_node_que.pop();

		ShortestDistance distance = node.first;
		NodeId node_id = node.second;

		/*
			寁應懳徾偺捀揰傑偱偺嵟抁嫍棧(p.first)偑
			偦偺捀揰偵帄傞傑偱偺嵟抁嫍棧(distances[node_id])傛傝傕
			墦偄側傜師偺捀揰傪扵偝側偄
		*/
		if (distances[node_id] < distance)
		{
			continue;
		}

		// 應検梡捀揰偺曈偺悢偩偗挷傋傞
		for (int i = 0; i < g_Nodes[node_id].size(); i++)
		{
			Edge edge = g_Nodes[node_id][i];
			/*
				崱偺捀揰傑偱偺嫍棧 + 曈偺僐僗僩偑
				曐懚偝傟偰偄傞師偺捀揰傑偱偺嵟抁嫍棧傛傝傕嬤偗傟偽峏怴偡傞
			*/
			if (distances[node_id] + edge.Cost < distances[edge.ToNodeId])
			{
				// 峏怴偟偨宱楬偺巒揰懁偺捀揰ID傪婰榐偡傞
				last_update_node_ids[edge.ToNodeId] = node_id;

				// 嵟抁嫍棧傪峏怴偡傞
				distances[edge.ToNodeId] = distances[node_id] + edge.Cost;
				// 師偺應検岓曗偲偟偰搊榐偡傞
				survey_node_que.push(Pair(distances[edge.ToNodeId], edge.ToNodeId));
			}
		}
	}

	// 嵟抁儖乕僩傪庢摼
	int current_route_id = goal;		// 崱僠僃僢僋拞偺宱楬ID
	std::list<int> shortest_route;		// 宱楬曐懚梡
	shortest_route.push_front(goal);

	while (true)
	{
		// 捀揰攝楍偐傜師偺捀揰ID傪庢摼偡傞
		int next_id = last_update_node_ids[current_route_id];

		// 巒揰偠傖側偐偭偨傜捛壛
		if (current_route_id != start)
		{
			shortest_route.push_front(next_id);
			current_route_id = next_id;
		}
		else
		{
			break;
		}
	}
	int sr[4800];
	for (int i = 0; i < 4800; i++)
	{
		sr[i] = 0;
	}
	int k = 0;
	printf("最短路径");
	for (int id : shortest_route)
	{
		printf("%d ", id);
		sr[k] = id;
		k++;
	}
	move(sr, velocity);
	printf("\n");
	printf("花费%d\n", distances[goal]);
}

RTC::ReturnCode_t routeplanning::onExecute(RTC::UniqueId ec_id)
{
	double v;
	double map[80][60][3];
	double v_judge[100];
	double v_out[2];
	double position[2];
	if (m_current_velocityIn.isNew())
	{
		m_current_velocityIn.read();
		v = m_current_velocity.data.vx * m_current_velocity.data.vx + m_current_velocity.data.vy * m_current_velocity.data.vy;
	}
	if (m_mapIn.isNew())
	{
		m_mapIn.read();
		for (int i = 0; i < 80; i++)
		{
			for (int j = 0; j < 60; j++)
			{
				for (int k = 0; k < 3; k++)
				{
					map[i][60-j][k] = m_map.data[j*80+i*3+k];
				}
			}
		}
	}
	if (m_positionIn.isNew())
	{
		m_positionIn.read();
		position[0] = m_position.data.position.x;
		position[1] = m_position.data.position.y;
	}
	while (1)
	{
		for (int i = 1; i < 100; i++)
		{
	    	v_judge[i] = v_judge[i - 1];
		}
		if (v_judge[0] == 0 && v_judge[49] == 0 && v_judge[99] == 0)
		{
			break;
		}
	}
	int x = floor(position[0]);
	int y = floor(position[1]);
	InitEdge(map);
	Dijkstra(y*80+x, 82, v_out);
	m_velocity.data.vx = v_out[0];
	m_velocity.data.vy = v_out[1];
	m_velocityOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t routeplanning::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t routeplanning::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t routeplanning::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t routeplanning::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t routeplanning::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void routeplanningInit(RTC::Manager* manager)
  {
    coil::Properties profile(routeplanning_spec);
    manager->registerFactory(profile,
                             RTC::Create<routeplanning>,
                             RTC::Delete<routeplanning>);
  }
  
};


