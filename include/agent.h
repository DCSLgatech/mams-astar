/*******************************************************************************
*
* mams_astar
* Copyright (C) 2020  Georgia Institute of Technology
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* any later version.

* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.

* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*******************************************************************************/

/* Author: Jaein Lim */

#ifndef __AGENT_H
#define __AGENT_H

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
#include "kshortestpaths/Graph.h"
#include "kshortestpaths/GraphElements.h"
#include "Tree.h"
#include "Node.h"
#include "State.h"
#include "Key.h"



#include "mams_astar/Cuckoo.h"

using namespace std;
using namespace kshortestpaths;

template<unsigned int DIM>
class Agent {

  public:

  	Agent();

    Agent(Tree<DIM>* original_tree);

    Agent(Tree<DIM>* original_tree, State<DIM>* sensor_state, State<DIM>* init_state, State<DIM>* goal_state);

    Agent(Tree<DIM>* original_tree, Key<DIM> sensor_key, Key<DIM>  init_key,  Key<DIM>  goal_key );

    ~Agent();

    // Setters and Getters
    void SetAgentID(int id){agent_ID_ = id;};

    int  GetAgentID(){return agent_ID_;};

    void Activate(){b_active_ = true; };

    bool IsActive(){	return b_active_ ; };

    void MuteOn(){mute_ = true;};

    bool IsMute(){return mute_;};

    void SetStartKey(const Key<DIM>& new_start_key){start_key_ = new_start_key; };


    // Graph construction

  	void ConstructReducedGraph(); // Construct from seleted nodes of tree

    // Search setups
    void PrepareSearch();

    void PushEgg(const mams_astar::CuckooConstPtr& cuckoo_egg_msg);

    void ProcessMessage();

    void ExpandToMessage(mams_astar::Cuckoo& new_msg);

    void ClearMessageQue();


    // DEBUGERS and Visualization

    void PrintGraph2File(std::string name,bool print_ID);

    void PrintPath2File(std::string name,bool print_ID);

    void PrintPathInfo();

    void PrintCommResult2File(std::string name,bool print_ID);

    void PrintINFO();

    void PrintOpenList();

    void PrintChosenNodes();

    Key<DIM> GetEndKey(){return end_key_;};



  private:

    //====================   1. Data members
  	//1.1 Constants for graph

  	double m_alpha;

  	double m_lambda1;

  	double m_lambda2;

  	//1.2 Graph initializer

  	kshortestpaths::Graph* pt_graph_;

  	Tree<DIM>* pt_tree_;

    int agent_ID_;

  	bool b_active_ = true;

    long end_vertex_ID_;

    long start_vertex_ID_;

    bool end_removed_ =false;

    bool mute_ = false;

    Key<DIM> start_key_;

  	Key<DIM> end_key_;

  	Key<DIM> sensor_key_;

  	int snsr_key_sz_;


  	//1.3 Data storage for Graph

  	std::vector<std::pair<Key<DIM>,int>> vec_chosen_nodes_; //node coord and size

  	std::vector<double> vec_node_costs_; //stores traversing cost for candidates //TODO: depreciate

    std::vector<double> vec_node_value_; //information abstracted in node

    std::vector<int> vec_nest_neigbors_; // stores neighbors ids of nest for fast edge construction

    std::vector<mams_astar::CuckooConstPtr> vec_msg_que_;

    //1.4 Data storage for Search

    std::map<BaseVertex*, double> m_mpStartDistanceIndex; 		//cost to come

  	std::map<BaseVertex*, BaseVertex*> m_mpPredecessorVertex; // back pointer

  	std::set<long> m_stDeterminedVertices; //closed list

  	std::multiset<BaseVertex*, WeightLess<BaseVertex> > m_quCandidateVertices; // open list


    //1.5 Counters

    int number_processed_msg_;

    int number_expanded_vtx_;



  	//====================   2. Data functions

    //2.1 Setup agent

    void initializeAgent();

    void clearCurrentExpansionResult();

    //2.2 Cost functions

  	double getCost(Node<DIM>* n);

    double getCost(const int size, const double raw_val);

    BasePath* getPath(bool& path_found);

    //2.3 Graph Manipulation

  	void chooseNodes4Vertices(Node<DIM>* node, Key<DIM> coord, int size);//Choose nodes from tree for graph

    bool checkNeighbor(const std::pair<Key<DIM>,int> &na, const std::pair<Key<DIM>,int> &nb);

    bool searchCuckooNest(const Key<DIM>& cuckoo_egg_key, long& node_index);

    void updateGraph(const std::pair<Key<DIM>,int>& cuckoo_node,
                const double& cuckoo_value, const std::pair<double,double>& cost,
                const long nest_id, Key<DIM>& pred_key );

    void localUpdate( const std::pair<Key<DIM>,int>& cuckoo_node,
                const double& cuckoo_value, const std::pair<double,double>& cost,
                Key<DIM>& pred_key);

    void inconsistencyUpdate(const std::pair<Key<DIM>,int>& cuckoo_node,
                const std::pair<double,double>& cost, const long node_id,
                Key<DIM>& pred_key);


  	//2.4 Search relevant methods

    void removePathInfo(BaseVertex* vertex_pt);

    void addPathInfo(BaseVertex* vertex_pt, double cost_to_come, double cost_to_go );

    bool addPredecessor(BaseVertex* vertex_pt, const Key<DIM>& pred_key);



};
#include "agent.hpp"

#endif //__AGENT_H
