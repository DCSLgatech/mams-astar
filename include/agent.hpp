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


#include "agent.h"
#include "kshortestpaths/Graph.h"

template <unsigned int DIM> Agent<DIM>::Agent(){}


template <unsigned int DIM> Agent<DIM>::Agent(Tree<DIM>* original_tree):pt_tree_(original_tree){}


template <unsigned int DIM> Agent<DIM>::Agent(Tree<DIM>* original_tree, State<DIM>* sensor_state,
	  																					State<DIM>* init_state, State<DIM>* goal_state ){
	 pt_tree_= original_tree;
	 pt_tree_->getKey(*sensor_state, sensor_key_, false);
	 snsr_key_sz_ =  pt_tree_ ->getKeySize(sensor_key_);
	 pt_tree_->getKey(*goal_state, end_key_, false);
	 pt_tree_->getKey(*init_state, start_key_, false);

	 this->initializeAgent();
}


template <unsigned int DIM> Agent<DIM>::Agent(Tree<DIM>* original_tree, Key<DIM> sensor_key,
	  																					 Key<DIM>  init_key,  Key<DIM>  goal_key ){
	 pt_tree_= original_tree;

	 sensor_key_ = sensor_key;
	 start_key_ = init_key;
	 end_key_ = goal_key;

	 snsr_key_sz_ =  pt_tree_ ->getKeySize(sensor_key_);

	 this->initializeAgent();
}


template <unsigned int DIM> Agent<DIM>::~Agent(){}


/* ========================== PUBLIC FUNCTIONS =========================== */

template <unsigned int DIM> void Agent<DIM>::ConstructReducedGraph(){

	//1. clear graph
	pt_graph_->clear();
	vec_chosen_nodes_.clear();
	vec_node_value_.clear();
	vec_node_costs_.clear();
	vec_nest_neigbors_.clear();

	//2. from root choose nodes for vertices
	// ROS_INFO("Construct Graph: %d",  pt_tree_-> getKeySize( pt_tree_->getRootKey() ));
	chooseNodes4Vertices(pt_tree_->getRoot(), pt_tree_->getRootKey(), pt_tree_-> getKeySize( pt_tree_->getRootKey() )  );

	//3. add chosen nodes to the graph
	int l=vec_chosen_nodes_.size();
	for(int i=0;i<l;++i)
	{
		// assign each vertex herusticis to goal
		 pt_graph_->add_vertex(i,m_lambda2*(vec_chosen_nodes_[i].first -end_key_).norm());
	}//for

	//4. Add edges to the graph
	for(int i=0;i<l;++i){
	    for(int j=i+1;j<l;++j){
	        if(checkNeighbor(vec_chosen_nodes_[i],vec_chosen_nodes_[j])){
				    pt_graph_->add_edge(i,j, vec_node_costs_[j]);
				    pt_graph_->add_edge(j,i, vec_node_costs_[i]);
					}
	    }
	} //4

} //ConstructReducedGraph()


template <unsigned int DIM> void Agent<DIM>::PrepareSearch()
{
		// Clear out any results
		this->clearCurrentExpansionResult();

		// Retreive start vertex ID and end vertex ID
		this->searchCuckooNest (start_key_, start_vertex_ID_);

		this->searchCuckooNest (end_key_, end_vertex_ID_ );

		bool start_vertex_missing = start_vertex_ID_ < 0 ;

		bool end_vertex_missing = end_vertex_ID_ < 0 ;

		if (start_vertex_missing ||  end_vertex_missing )
		{
				if (start_vertex_missing)
				{
					ROS_ERROR("AGENT %d: PREPARE_SEARCH() START KEY MISSING (%ld, %ld)", agent_ID_, start_key_[0],start_key_[1]);
				}
				else {
					ROS_INFO("AGENT %d: START KEY FOUND @ %ld", agent_ID_,start_vertex_ID_) ;
				}
				if (end_vertex_missing)
				{
					ROS_ERROR("AGENT %d:  PREPARE_SEARCH() END KEY MISSING (%ld, %ld)", agent_ID_, end_key_[0],end_key_[1]);
				}
				else {
					ROS_INFO("AGENT %d: END KEY FOUND @ %ld", agent_ID_,end_vertex_ID_) ;
				}
				exit(1);
		}

		BaseVertex* start_vertex =  pt_graph_-> get_vertex(start_vertex_ID_);

		// Make offset cost to compensate for abstraction on start vertex
 	  double offset_cost = 0;

		if (vec_chosen_nodes_[start_vertex_ID_].second !=2)

		offset_cost= getCost( vec_chosen_nodes_[start_vertex_ID_].second ,vec_node_value_[start_vertex_ID_]);

		m_mpStartDistanceIndex[start_vertex] = offset_cost;

		start_vertex->Weight( offset_cost);

		m_quCandidateVertices.insert(start_vertex);

		if(! mute_ ) {
			ROS_INFO("AGENT %d: PREPARE_SEARCH() START_ID: %ld @ KEY (%ld, %ld) FOUND?: %s",
								agent_ID_, start_vertex_ID_,  start_key_[0], start_key_[1],
								!start_vertex_missing ? "true" : "false");
			ROS_INFO("AGENT %d: PREPARE_SEARCH() OFFSET COST: %f",
								agent_ID_, offset_cost);
			ROS_INFO("AGENT %d: PREPARE_SEARCH() END_ID: %ld @ KEY (%ld, %ld) FOUND?: %s",
								agent_ID_,  end_vertex_ID_,  end_key_[0], end_key_[1],
								!end_vertex_missing ? "true" : "false");

		  ROS_INFO("AGENT %d: PREPARE_SEARCH() SENSOR_KEY (%ld, %ld) << ", agent_ID_, sensor_key_[0],sensor_key_[1]);
		}

}



template <unsigned int DIM> void Agent<DIM>::PushEgg(const mams_astar::CuckooConstPtr& cuckoo_egg_msg)
{
	vec_msg_que_.push_back(cuckoo_egg_msg);

}


template <unsigned int DIM> void Agent<DIM>::ProcessMessage()
{
	if(! mute_ )
		ROS_INFO("AGENT %d: PROCESS_MESSAGE() #MSGS IN QUE ------------------- %d << ", agent_ID_, vec_msg_que_.size());

	// 0. go through all messages in the que
	for(int msg_pos = 0; msg_pos < vec_msg_que_.size();msg_pos++)
	{

		Key<DIM> egg_key(vec_msg_que_[msg_pos]->node_key);
		std::pair<Key<DIM>,int> egg_node(egg_key, vec_msg_que_[msg_pos]->node_size);
		std::pair<double,double> egg_g_h (vec_msg_que_[msg_pos] -> cost_to_come, vec_msg_que_[msg_pos]-> cost_to_go);
		double egg_val =vec_msg_que_[msg_pos] ->node_value;
		Key<DIM> egg_pred_key(vec_msg_que_[msg_pos]-> predecessor_key);
		int egg_pred_size = vec_msg_que_[msg_pos]-> predecessor_size;

		long node_id;
		BaseVertex* vertex_pt;

		if(this->searchCuckooNest(egg_key, node_id)) // CASE 1 nest was found
		{
			if(! mute_ ) ROS_INFO("AGENT %d  PROCESS_MESSAGE() CASE1 UPDATE GRAPH: NEST ID %d ", agent_ID_, node_id);

			this->updateGraph(egg_node, egg_val, egg_g_h, node_id, egg_pred_key);

			//UPDATING START AND END IDs
			if (node_id == start_vertex_ID_)
			{
				start_vertex_ID_ =vec_chosen_nodes_.size()-1 ;
					if(! mute_ ) ROS_INFO("AGENT %d:  PROCESS_MESSAGE() start updated %ld", agent_ID_, vec_chosen_nodes_.size()-1);
			}

			if (vec_msg_que_[msg_pos]-> terminated)
			{
				end_vertex_ID_ = vec_chosen_nodes_.size()-1;
			}

			number_processed_msg_++;
			continue;

		}
		else // CASE 2 either exact vertex found or smaller
		{
				if(node_id == -2) //  CASE 2 IGNORE: smaller vertex exist
				{
						if(! mute_ )ROS_INFO("AGENT %d: PROCESS_MESSAGE() CASE2 IGNORE: (%ld, %ld)", agent_ID_ ,egg_key[0],egg_key[1]);
						continue;
				}
				else if (node_id == -1) // CASE 3 AFFIRMITVE CASE: new vertex without nest
				{
						if(! mute_ ) ROS_INFO("AGENT %d: PROCESS_MESSAGE() CASE3 LOCAL UPDATE: (%ld, %ld)", agent_ID_ ,egg_key[0],egg_key[1]);

						this->localUpdate(egg_node, egg_val, egg_g_h,  egg_pred_key);

						if (vec_msg_que_[msg_pos]-> terminated)
						{
							end_vertex_ID_ = vec_chosen_nodes_.size()-1;
						}

						number_processed_msg_++;
						continue;
				}
				else //  CASE 4: INVESTIGATE exact vertex exists, needs further investigation
				{
						if (vec_chosen_nodes_[node_id].second == -1)
						{
							if(! mute_ ) 	ROS_ERROR("AGENT %d: PROCESS_MESSAGE() CASE4 trying to reach removed node %ld", agent_ID_, node_id);
						}
						else
						{
							if(! mute_ ) ROS_INFO("AGENT %d: PROCESS_MESSAGE() CASE4 INCONSISTENCY UPDATE: %ld", agent_ID_, node_id);

							this->inconsistencyUpdate(egg_node, egg_g_h, node_id,  egg_pred_key);

							number_processed_msg_++;
							continue;
						}
				} // exact vertex or smaller
			} // CASE 2

	}//for message que


	// 4. clear the message que
	vec_msg_que_.clear();

	if(! mute_ ) ROS_INFO("AGENT %d: PROCESS_MESSAGE() ----------------- CLEARS MESSEAGE QUE " ,agent_ID_);
} //ProcessMessage


template <unsigned int DIM> void Agent<DIM>::ExpandToMessage(mams_astar::Cuckoo& new_msg)
{

	// 0. JUST IN CASE OF INACTIVE, BROADCAST INVALID MSG
	if ( !b_active_ || m_quCandidateVertices.empty())
	{
		new_msg.agent_ID = 0;
		return;
	}

	// 1. POP FRONT VERTEX IN OPEN
	multiset<BaseVertex*, WeightLess<BaseVertex> >::const_iterator pos = m_quCandidateVertices.begin();
	BaseVertex* cur_vertex_pt = *pos;
	m_quCandidateVertices.erase(pos);

	// 2. CHECK IF THE VERTEX IS END-VERTEX
	new_msg.terminated = false;

	if (cur_vertex_pt-> getID() == end_vertex_ID_ )
	{

		if(! mute_)
		{
			std::map<BaseVertex*, double>::const_iterator g_pos = m_mpStartDistanceIndex.find(cur_vertex_pt);
			std::cout << std::endl <<std::endl;
			ROS_ERROR("\n!!!!! AGENT %d: EXPAND() FOUND GOAL %ld KEY (%ld, %ld) ~ with cost %f !!!!! \n"
					,agent_ID_, cur_vertex_pt->getID(),
					vec_chosen_nodes_[ cur_vertex_pt->getID()].first[0],
					vec_chosen_nodes_[ cur_vertex_pt->getID()].first[1],
					g_pos -> second);
		}

		// DEACTIVATE AND CLEAR OPEN LIST
		b_active_ = false;
		m_quCandidateVertices.clear();

		new_msg.terminated = true;
	}

	// 3. MOVE FRONT_VERTEX TO CLOSED
	m_stDeterminedVertices.insert(cur_vertex_pt->getID());
	number_expanded_vtx_++;

	// 3.1 GET COST_TO_COME OF FRONT_VERTEX
	map<BaseVertex*, double>::const_iterator cur_pos = m_mpStartDistanceIndex.find(cur_vertex_pt);

	double cur_distance =  cur_pos ->second;



	// 4. GET ADJACENT VERTICES OF THE FRONT_VERTEX
	set<BaseVertex*>* neighbor_vertex_list_pt = new set<BaseVertex*>();

	pt_graph_->get_adjacent_vertices(cur_vertex_pt, *neighbor_vertex_list_pt);

	for(set<BaseVertex*>::iterator cur_neighbor_pos=neighbor_vertex_list_pt->begin();
		cur_neighbor_pos!=neighbor_vertex_list_pt->end(); ++cur_neighbor_pos)
	{

		// 4.1 SKIP IF IT IS CLOSED
		if (m_stDeterminedVertices.find((*cur_neighbor_pos)->getID())!=m_stDeterminedVertices.end())
		{
			continue;
		}

		// 4.2 EVALUATE COST TO ADJACENT
		double distance = cur_distance + getCost(vec_chosen_nodes_[(*cur_neighbor_pos)->getID()].second,
																						 vec_node_value_[(*cur_neighbor_pos)->getID()] );


		// 4.3 UPDATE NEW DISTANCE (IF NOT VISITED OR BETTER COST)
		cur_pos = m_mpStartDistanceIndex.find(*cur_neighbor_pos);
		if (cur_pos == m_mpStartDistanceIndex.end() || cur_pos->second > distance)
		{

			// 4.3.1 update g of the adj vertex
			m_mpStartDistanceIndex[*cur_neighbor_pos] = distance;
			// 4.3.2 update the predecessor of the adj vertex
			m_mpPredecessorVertex[*cur_neighbor_pos] = cur_vertex_pt;

			//4.3.3 update the open list accordingly
			(*cur_neighbor_pos)->Weight(distance);
			multiset<BaseVertex*, WeightLess<BaseVertex> >::const_iterator pos = m_quCandidateVertices.find((*cur_neighbor_pos));

			if (pos != m_quCandidateVertices.end() )
			m_quCandidateVertices.erase(pos);

			m_quCandidateVertices.insert(*cur_neighbor_pos);
			if(! mute_)
			ROS_INFO("AGENT %d: EXPAND() %ld PUTS IN THE QUE %ld KEY (%ld, %ld) @ f: %f !! "
			    ,agent_ID_, cur_vertex_pt->getID(), (*cur_neighbor_pos)->getID(),
			    vec_chosen_nodes_[ 	(*cur_neighbor_pos)->getID() ].first[0],
			    vec_chosen_nodes_[ 	(*cur_neighbor_pos)->getID() ].first[1],
			    distance + (*cur_neighbor_pos)->h() );

		}// 4.3 if
	} //4. for all



	// 5. MAKE NEW MESSAGE
	new_msg.agent_ID = agent_ID_;
	new_msg.node_key = vec_chosen_nodes_[cur_vertex_pt->getID()].first.vectorize();
	new_msg.node_size = vec_chosen_nodes_[cur_vertex_pt->getID()].second;
	new_msg.cost_to_come = cur_distance;
	new_msg.cost_to_go = cur_vertex_pt->h();
	new_msg.node_value = vec_node_value_[cur_vertex_pt->getID()];
	new_msg.node_ID = cur_vertex_pt->getID();

	std::map<BaseVertex*, BaseVertex*>::const_iterator pre_pos = m_mpPredecessorVertex.find(cur_vertex_pt);
	if(pre_pos != m_mpPredecessorVertex.end() && vec_chosen_nodes_[(pre_pos->second)->getID()].second !=-1 )
	{
		new_msg.predecessor_key = vec_chosen_nodes_[(pre_pos->second)->getID()].first.vectorize();
		new_msg.predecessor_size = vec_chosen_nodes_[(pre_pos->second)->getID()].second;
	}


	if(! mute_ ) {
		ROS_INFO("AGENT %d: EXPAND() BROADCAST %ld KEY (%ld, %ld) @ g: %f ------>> "
				,agent_ID_, cur_vertex_pt->getID(), new_msg.node_key[0],new_msg.node_key[1],new_msg.cost_to_come );
		PrintOpenList();
	}

	//6. IF OPEN LIST EMPTY
	if (m_quCandidateVertices.empty())
	{
		b_active_ = false;
		if(! mute_)
		ROS_INFO("AGENT %d: EMPTIED THE QUE - DEACTIVATE " ,agent_ID_);
	}

} //expandToMessage()

template <unsigned int DIM> void Agent<DIM>::ClearMessageQue()
{
	vec_msg_que_.clear();

}










/*================= PRIVATE FUNCTIONS ======================= */





template <unsigned int DIM> void Agent<DIM>::initializeAgent(){

	m_alpha=1;

	m_lambda1=0.999;

	m_lambda2=0.001;

	pt_graph_ = new kshortestpaths::Graph();

	Node<DIM>* nstart=pt_tree_->getNode( start_key_);

	Node<DIM>* ngoal =pt_tree_->getNode(end_key_);

} // Initialize Agent


template <unsigned int DIM> void Agent<DIM>::clearCurrentExpansionResult()
{
	m_stDeterminedVertices.clear();

	m_mpPredecessorVertex.clear();

	m_mpStartDistanceIndex.clear();

	m_quCandidateVertices.clear();

	number_processed_msg_=0;

	number_expanded_vtx_=0;

	end_removed_ =false;
}


template <unsigned int DIM> double Agent<DIM>::getCost(Node<DIM>* n){

	return (m_lambda1*n->getValue()+m_lambda2)*pt_tree_->getVolume(n->getDepth());
}

template <unsigned int DIM> double Agent<DIM>::getCost(const int size, const double raw_val)
{
	return (m_lambda1*raw_val + m_lambda2) * size *(1<<DIM);

}


template <unsigned int DIM> BasePath* Agent<DIM>::getPath(bool& path_found)
{
	path_found =true;
	//	PrintGraph2File("graph");
	if(! mute_ )
	ROS_INFO("AGENT %d: retreiving END %ld KEY (%ld, %ld) "
	,agent_ID_, end_vertex_ID_,
	vec_chosen_nodes_[end_vertex_ID_].first[0],
	vec_chosen_nodes_[end_vertex_ID_].first[1]);
	if(! mute_ )
	ROS_INFO("AGENT %d: retreiving START %ld KEY (%ld, %ld) "
	,agent_ID_, end_vertex_ID_,
	vec_chosen_nodes_[start_vertex_ID_].first[0],
	vec_chosen_nodes_[start_vertex_ID_].first[1]);

  //std::cin.ignore();
	BaseVertex* sink = pt_graph_->get_vertex(end_vertex_ID_);
	BaseVertex* source = pt_graph_->get_vertex(start_vertex_ID_);
	std::vector<BaseVertex*> vertex_list;
	std::map<BaseVertex*, double>::const_iterator pos = m_mpStartDistanceIndex.find(sink);
	double weight = pos != m_mpStartDistanceIndex.end() ? pos->second : Graph::DISCONNECT;

	if (weight < Graph::DISCONNECT)
	{
		BaseVertex* cur_vertex_pt = sink;
		BaseVertex* base_vertex_pt = cur_vertex_pt;
		do
		{
			vertex_list.insert(vertex_list.begin(), cur_vertex_pt);

			std::map<BaseVertex*, BaseVertex*>::const_iterator pre_pos =
				m_mpPredecessorVertex.find(cur_vertex_pt);

			if (pre_pos == m_mpPredecessorVertex.end()) // predecessor doesn't exist
			{
				path_found = false;
				break;
			}


			if(! mute_ )
			ROS_INFO("AGENT %d: tracks %ld KEY (%ld, %ld) --> %ld KEY (%ld, %ld)"
				,agent_ID_,  cur_vertex_pt-> getID(),
				vec_chosen_nodes_[ cur_vertex_pt-> getID()].first[0],
				vec_chosen_nodes_[ cur_vertex_pt-> getID()].first[1],
				(pre_pos->second)->getID(),
				vec_chosen_nodes_[ (pre_pos->second)->getID()].first[0],
				vec_chosen_nodes_[ (pre_pos->second)->getID() ].first[1]);

			if (pre_pos->second == base_vertex_pt) // predecessor is its own... somethings wrong
			{
				path_found = false;
				break;
			}
			else
			{
					base_vertex_pt = cur_vertex_pt;
			}
			cur_vertex_pt = pre_pos->second;

		} while (cur_vertex_pt != source);

		vertex_list.insert(vertex_list.begin(), source);
	}
	else {
		path_found = false;
	}

	return new BasePath(vertex_list, weight);


}


/* This version of chooseNodes4Vertices follows Florian's where in a top-down fashion, recursively
  leaf nodes that are m_alpha distance away from current agent's sensor state
initialized from ConstructReducedGraph() with root_key and root_key size */
template <unsigned int DIM> void Agent<DIM>::chooseNodes4Vertices (Node<DIM>* node, Key<DIM> coord, int size){

	//1. check if reached to leaf
	//std::cout << "ChooseNodes : " << coord  <<"size " << size << std::endl;
	//TODO: (coord-sensor_key_).normSq()> (m_alpha*(size<<1)+sqrt(DIM)*size) || node-> isLeaf()

	 if( (coord-sensor_key_).normSq()>
	  	 		(m_alpha*(size)+sqrt(DIM)*(snsr_key_sz_))*(m_alpha*(size)+sqrt(DIM)*(snsr_key_sz_))
	   		|| node-> isLeaf()
		 )
	 {
	    vec_chosen_nodes_.push_back(std::pair<Key<DIM>,int> (coord,size));
			vec_node_costs_.push_back(getCost(size, node->getValue()) );
			vec_node_value_.push_back(node->getValue());

	//2. recursive down
	 }
	 else
	 {

		int s=size>>1; // s= size/2

 		if(!node->isLeaf() )
		{
		// 2.1 for all children
	        for(int i=0;i<TwoPow<DIM>::value;++i){
				    Key<DIM> next_key = coord+( *(pt_tree_->getDirections()) )[i]* (s>>1);
				    chooseNodes4Vertices(node->getChild(i), next_key , s);
					} //for childrens

	   }// if not leaf nor epsilonObs

	} //Top if

} // chooseNodes4Vertices



template <unsigned int DIM> bool Agent<DIM>::checkNeighbor(const std::pair<Key<DIM>,int> &na, const std::pair<Key<DIM>,int> &nb)
{
	int l=(na.second>>1)+(nb.second>>1);
	Key<DIM> diff=(na.first-nb.first).abs();
	std::partial_sort(diff.begin(),diff.begin()+2,diff.end(),std::greater<int>()); // only interested in first two coordinates
	if(diff[0]==l && diff[1]!=l) // second clause prevents 8 connectivity
		return true;
	return false;
}// checkNeighbor



/*
TRUE: if nest found
FALSE: else
	1. exact size vertex exists => node_index = vertex_id   INVESTIGATE CASE
	2. smaller size vertex exists => node_index = -2        IGNORE CASE
	3. if new vertex => node_index =-1											AFFIRMITIVE CASE
*/

template <unsigned int DIM> bool Agent<DIM>:: searchCuckooNest(const Key<DIM>& cuckoo_egg_key, long& node_index){

	//1. default output values
	bool nest_found;
	long index_it;

	//2. intialize with root key and tree height
	Key<DIM> k = pt_tree_-> getRootKey();	//root key of the tree
	int height=pt_tree_ -> getMaxDepth()-1;	//top-down search preparation
	Key<DIM> k2;				//key distance holder

	//3. top down iterate
	while(height>=0)
	{

		//3.1 compute next key
		k2=cuckoo_egg_key-k;		//key distance
		k= k+ (pt_tree_ -> getNextKey(k2) <<height); // get to next key in tree child direction
		height--;

		//Renew index_it
		index_it = -1;

		//3.2 Already failed to find a nest
		if(k == cuckoo_egg_key)
		{
			nest_found = false;
			height++;
			double shift_dist = (pt_tree_ -> getNextKey(k2) << height).normSq();
			for(typename std::vector<std::pair<Key<DIM>,int>>::const_iterator pos =
			    vec_chosen_nodes_.begin(); pos != vec_chosen_nodes_.end(); ++pos, ++index_it)
			{
			  Key<DIM> cur_dist = (*pos).first - cuckoo_egg_key;
			  if ( cur_dist.normInf() ==0 )
			  {
			    node_index = index_it+1;
			    return nest_found;	//  INVESTIGATE CASE
			  }
			  if ( shift_dist > cur_dist.normInf()*cur_dist.normInf()*2)
			  {
			    node_index = -2;	// IGNORE CASE
			    return nest_found;
			  }
			}


			// AFFIRMITIVE CASE
			node_index = -1;
			return nest_found;
		}

		//3.3 iterate through chosen vertices to find the nest
		for (typename std::vector<std::pair<Key<DIM>,int>>::const_iterator pos= vec_chosen_nodes_.begin();
							 pos != vec_chosen_nodes_.end(); ++pos, ++index_it)
		{
			if(  (*pos).second != -1 && (*pos).first == k  )
			{
				node_index = index_it+1;
				nest_found =true;
				return nest_found;
		 	}//not removed && same key
		} //for candidates

	 } //while reached to the bottom of the tree but not found

	 return nest_found;
} //SearchCuckooNest


template <unsigned int DIM> void Agent<DIM>::updateGraph(const std::pair<Key<DIM>,int>& cuckoo_node,
		const double& cuckoo_value,	const std::pair<double,double>& cost, const long nest_id, Key<DIM>& pred_key)
{

	//CREATE CUCKOO VERTEX
	int cuckoo_id = vec_chosen_nodes_.size();
	vec_chosen_nodes_.push_back(cuckoo_node);
	vec_node_value_.push_back(cuckoo_value);
	vec_node_costs_.push_back(getCost( cuckoo_node.second , cuckoo_value));
	pt_graph_ -> add_vertex(cuckoo_id);

	// END-VERTEX UPDATE TODO: REMOVE
	// end_vertex_ID_  = end_removed_? cuckoo_id : end_vertex_ID_;

	//RETRIEVE NEST NEIGHBORS
	BaseVertex* nest_vertex_pt= pt_graph_ -> get_vertex(nest_id);
	set<BaseVertex*>* neighbor_vertex_list_pt = new set<BaseVertex*>();
	pt_graph_->get_adjacent_vertices(nest_vertex_pt, *neighbor_vertex_list_pt);


	//REMOVE NEST VERTEX
	vec_chosen_nodes_[nest_id].second = -1 ;
	pt_graph_ -> remove_vertex(nest_id);
	if(! mute_ )ROS_INFO("AGENT %d: UPDATE_GRAPH() REMOVES NODE ID %ld", agent_ID_, nest_id );


	//EDGE MODIFICATION AMONG NEST NEIGHBORS AND CUCKOO
	for(set<BaseVertex*>::iterator cur_neighbor_pos=neighbor_vertex_list_pt->begin();
			cur_neighbor_pos!=neighbor_vertex_list_pt->end(); ++cur_neighbor_pos)
	{

		//REMOVE NEST EDGES
		pt_graph_->remove_edge(std::pair<long,long> (nest_id, (*cur_neighbor_pos)->getID()) );
		pt_graph_->remove_edge(std::pair<long,long> ( (*cur_neighbor_pos)->getID() , nest_id) );
		if(! mute_ )ROS_INFO("AGENT %d: UPDATE_GRAPH() REMOVES EDGE  %ld -- %ld", agent_ID_, nest_id, (*cur_neighbor_pos)->getID()  );


		//ADD EDGES WITH CUCKOO
		if ( vec_chosen_nodes_[(*cur_neighbor_pos)->getID()].second!=-1 &&
					checkNeighbor(cuckoo_node, vec_chosen_nodes_[(*cur_neighbor_pos)->getID()])   )
		{
			pt_graph_->add_edge(cuckoo_id, (*cur_neighbor_pos)->getID(),
			  			getCost( vec_chosen_nodes_[(*cur_neighbor_pos)->getID() ].second ,
											 vec_node_value_[(*cur_neighbor_pos)->getID() ])  );
			pt_graph_->add_edge((*cur_neighbor_pos)->getID() , cuckoo_id,  getCost( cuckoo_node.second , cuckoo_value) );

			 if(! mute_ )ROS_INFO("AGENT %d: UPDATE_GRAPH() NEW EDGE  %ld -- %ld", agent_ID_, cuckoo_id, (*cur_neighbor_pos)->getID() );
		}


		//SAVE NEST NEIGHBORS FOR FAST LOCAL UPDATE
		vec_nest_neigbors_.push_back((*cur_neighbor_pos)->getID());

	}


	// REMOVE NEST VERTEX IN CURRENT PATH (OPEN, CLOSED, PREDECESSOR)
	this->removePathInfo(nest_vertex_pt);


	// ADD CUCKOO VERTEX IN CURRENT PATH (g, h, OPEN, CLOSED, PREDECSSOR)
	BaseVertex* cuckoo_vertex_pt = pt_graph_ -> get_vertex(cuckoo_id) ;
	if (this->addPredecessor(cuckoo_vertex_pt, pred_key))
	{
		this->addPathInfo(cuckoo_vertex_pt, cost.first, cost.second);
	}
	else {
		if(! mute_ ) ROS_ERROR("AGENT %d: UPDATE_GRAPH() FAILS TO FIND PREDECESSOR OF (%ld, %ld)",
		 agent_ID_ ,cuckoo_node.first[0], cuckoo_node.first[1]);
	}

	if(! mute_ ) ROS_INFO("AGENT %d: UPDATE_GRAPH() PUT IN THE QUE (%ld, %ld), @ f: %f", agent_ID_ ,
		 cuckoo_node.first[0],cuckoo_node.first[1], cost.first + cost.second);

  // SAVE CUCKOO VERTEX FOR FAST LOCAL UPDATE
	vec_nest_neigbors_.push_back(cuckoo_id);
} // UpdateGraph




template <unsigned int DIM> void Agent<DIM>::localUpdate( const std::pair<Key<DIM>,int>& cuckoo_node,
		const double& cuckoo_value, const std::pair<double,double>& cost, Key<DIM>& pred_key)
{
	//CREATE CUCKOO VERTEX
	int cuckoo_id = vec_chosen_nodes_.size();
	vec_chosen_nodes_.push_back(cuckoo_node);
	vec_node_value_.push_back(cuckoo_value);
	vec_node_costs_.push_back(getCost( cuckoo_node.second , cuckoo_value));
	pt_graph_ -> add_vertex(cuckoo_id);

	// ADD NEW EDGES AMONG CANDIDATES
		for(int i =0; i< vec_nest_neigbors_.size(); i++)
		{

			int neighbor_candidate = vec_nest_neigbors_[i];
			if (vec_chosen_nodes_[neighbor_candidate].second!=-1 &&  checkNeighbor(cuckoo_node, vec_chosen_nodes_[neighbor_candidate]) )
			{

				 if(! mute_ )ROS_INFO("AGENT %d: LOCAL_UPDATE() NEW EDGE %ld-%ld @ ->%f & <-%f", agent_ID_, cuckoo_id, neighbor_candidate,
															getCost( vec_chosen_nodes_[neighbor_candidate].second ,vec_node_value_[neighbor_candidate]),
															getCost( cuckoo_node.second , cuckoo_value)	);
			 pt_graph_->add_edge(cuckoo_id, neighbor_candidate, getCost( vec_chosen_nodes_[neighbor_candidate].second ,vec_node_value_[neighbor_candidate]) ); //TODO: find edge_cost
			 pt_graph_->add_edge(neighbor_candidate, cuckoo_id, getCost( cuckoo_node.second , cuckoo_value));

			} //if checkNeighbor
		}


	// ADD CUCKOO TO CURRENT PATH (g,h, OPEN, CLOSED, PREDECSSOR)
	BaseVertex* cuckoo_vertex_pt = pt_graph_ -> get_vertex(cuckoo_id) ;
	if (addPredecessor(cuckoo_vertex_pt, pred_key))
	{
		addPathInfo(cuckoo_vertex_pt, cost.first, cost.second);
	}
	else {
		if(! mute_ ) ROS_ERROR("AGENT %d: LOCAL_UPDATE() FAILS TO FIND PREDECESSOR OF (%ld, %ld)",
		 agent_ID_ ,cuckoo_node.first[0], cuckoo_node.first[1]);
	}

	// ADD TO FAST NEIGHBOR CANDIDATES
	vec_nest_neigbors_.push_back(cuckoo_id);

}// LocalUpdate





template <unsigned int DIM> void Agent<DIM>::inconsistencyUpdate(const std::pair<Key<DIM>,int>& cuckoo_node,
			const std::pair<double,double>& cost, const long node_id, Key<DIM>& pred_key)
{

	 //RETRIEVE EXISITING VERTEX
	 BaseVertex* vertex_pt = pt_graph_->get_vertex(node_id);

	 // CHECK IF THE VERTEX HAS BEEN VISITED (IN CLOSED LIST)
	 std::map<BaseVertex*, double>::const_iterator pos = m_mpStartDistanceIndex.find(vertex_pt);
	 if( pos != m_mpStartDistanceIndex.end() ) //if g(s) has been examined
	 {

		   // IN CLOSED + INCONSISTENT
	     if (pos->second > cost.first)
			 {
				  if(! mute_ ) ROS_INFO("AGENT %d: INCONSISTENCY_UPDATE() SPOTS (%ld, %ld) old g %f > new g %f", agent_ID_ ,
				 		 cuckoo_node.first[0],cuckoo_node.first[1], pos->second, cost.first);

					// UPDATE PATH INFO
					removePathInfo(vertex_pt);
					if (addPredecessor(vertex_pt, pred_key) )
					{
						addPathInfo(vertex_pt, cost.first, cost.second);
						number_processed_msg_++;
						// b_active_ = true;
					}
					else
					{
						if(! mute_ )
						{
							ROS_ERROR("AGENT %d: INCONSISTENCY_UPDATE() ABORTS (%ld, %ld) : failure to find predecessor", agent_ID_ ,cuckoo_node.first[0], cuckoo_node.first[1]);
							//exit(1);
						}

						return;
					}

			 }
			 else
			 {
				 if(! mute_ ) ROS_INFO("AGENT %d: INCONSISTENCY_UPDATE() ABORTS (%ld, %ld) : worse g(s)", agent_ID_ ,cuckoo_node.first[0], cuckoo_node.first[1]);
				 return;
			 }

	 }
	 else // it has not been expanded yet
	 {
		 addPathInfo(vertex_pt, cost.first, cost.second);
		 addPredecessor(vertex_pt, pred_key);
		 number_processed_msg_++;
		 // b_active_ = true;
		 if(! mute_ ) ROS_ERROR("AGENT %d: INCONSISTENCY_UPDATE() PUT IN THE QUE : %ld has not expanded", agent_ID_, node_id);
	 }

}// InconsistencyUpdate



template <unsigned int DIM> void Agent<DIM>::removePathInfo(BaseVertex* vertex_pt)
{

	//ERASE FROM OPEN LIST
	multiset<BaseVertex*, WeightLess<BaseVertex> >::const_iterator pos = m_quCandidateVertices.find(vertex_pt);
	if (pos != m_quCandidateVertices.end() )
	m_quCandidateVertices.erase(pos);

	//ERASE FROM COST_TO_COME
	map<BaseVertex*, double>::const_iterator g_pos = m_mpStartDistanceIndex.find(vertex_pt);
	if (g_pos != m_mpStartDistanceIndex.end() )
	m_mpStartDistanceIndex.erase(g_pos);

	//ERASE PREDECESSOR
	map<BaseVertex*, BaseVertex*> ::const_iterator pred_pos = m_mpPredecessorVertex.find(vertex_pt);
	if (pred_pos != m_mpPredecessorVertex.end() )
	m_mpPredecessorVertex.erase(pred_pos);

	//ERASE FROM CLOSED LIST
	set<long>::const_iterator closed_pos =  m_stDeterminedVertices.find(vertex_pt -> getID());
	if ( closed_pos != m_stDeterminedVertices.end() )
	m_stDeterminedVertices.erase(closed_pos);


}

template <unsigned int DIM> void Agent<DIM>::addPathInfo(BaseVertex* vertex_pt, double cost_to_come, double cost_to_go )
{

	vertex_pt -> Weight(cost_to_come);
	vertex_pt -> h(cost_to_go);
	m_quCandidateVertices.insert( vertex_pt );
	m_mpStartDistanceIndex[vertex_pt] = cost_to_come ;
	b_active_ = true;
}

template <unsigned int DIM> bool Agent<DIM>::addPredecessor(BaseVertex* vertex_pt, const Key<DIM>& pred_key)
{

	int node_it = 0;
	for (; node_it != vec_chosen_nodes_.size(); node_it++)
	{
	  if (  vec_chosen_nodes_[node_it].second != -1 &&  vec_chosen_nodes_[node_it].first == pred_key)
		{
			m_mpPredecessorVertex[ vertex_pt ] = pt_graph_ ->get_vertex(node_it);
			return true;
		}

	}

	  if(! mute_ ) ROS_ERROR("AGENT %d: FAILS TO FIND PREDECESSOR FOR KEY (%ld, %ld)",  agent_ID_, pred_key[0], pred_key[1]);
		return false;


}


/*================================= DEBUGGERS =========================== */


template <unsigned int DIM> void Agent<DIM>::PrintGraph2File(std::string name, bool print_ID)
{

		std::string dir_name = ros::package::getPath("mams_astar") + "/data/";
		std::string fmt_name = ".txt";
		std::string full_name;
		if (print_ID)
		{
			full_name = dir_name +	name + std::to_string(agent_ID_)+ fmt_name;
		}else {
			full_name = dir_name +	name + fmt_name;
		}
    std::ofstream graph_file(full_name);
    //std::cout << "number of vertices alive : " << vec_chosen_nodes_.size()-st_removed_nest_id_.size() <<"/" << vec_chosen_nodes_.size() << std::endl;
   // PrintChosenNodes();
    if( graph_file.is_open() )
		{
        for(int i=0; i<vec_chosen_nodes_.size() ; i++)
				{
	    		if( vec_chosen_nodes_.at(i).second != -1 ) //removed vertex check
					{
	    			std::pair<Key<DIM>,int> node = vec_chosen_nodes_.at(i);
						graph_file << node.first << " " << node.second <<" " ;
						graph_file << vec_node_value_.at(i) << " ";
	    		} else { //if removed, insert with 0 to make the indexing consistent
						for(int i=0; i< vec_chosen_nodes_.at(i).first.size(); i++)
						{
							graph_file << "0 ";
						}
						graph_file <<"0 0 "; //for node size and node value
	    		}

			    std::vector<long> neighbors;
			    pt_graph_->get_neighbor_vertices(i, neighbors);
			    for (int j=0; j< (vec_chosen_nodes_.size() ) ; j++)
					{
						if(j<neighbors.size() ){
							graph_file << neighbors[j]+1 << " " ; //+1 for matlab indexing
						}else{
							graph_file << 0 << " " ;
						}
	    		}// for neighbors of current node

	    		graph_file <<std::endl;
				}//for entire nodes
    }// if graph_file open
    graph_file.close();
}// PrintGraph2File


template <unsigned int DIM> void Agent<DIM>::PrintPath2File(std::string name,bool print_ID)
{
	std::string dir_name = ros::package::getPath("mams_astar") + "/data/";
	std::string fmt_name = ".txt";
  std::string full_name;
	if (print_ID)
	{
		full_name = dir_name +	name + std::to_string(agent_ID_)+ fmt_name;
	}else {
		full_name = dir_name +	name + fmt_name;
	}

  std::ofstream path_file(full_name);
   // PrintChosenNodes();
	bool path_found;
	BasePath* path_pt = getPath(path_found);
	//if(path_found) 	ROS_INFO("AGENT %d: FOUND PATH AT COST : %f" ,agent_ID_, path_pt->Weight());
  if( path_file.is_open() )
	{
      for(int i=0; i<path_pt->length() ; i++)
			{
				path_file << (path_pt->GetVertex(i)) ->getID() << std::endl;
			}

  }// if path_file open
  path_file.close();
}// PrintPath2Files


template <unsigned int DIM> void Agent<DIM>::PrintPathInfo()
{
	   // PrintChosenNodes();
	bool path_found;
	BasePath* path_pt = getPath(path_found);
	if(path_found) 	ROS_INFO("AGENT %d: FOUND PATH AT COST : %f" ,agent_ID_, path_pt->Weight());

}// PrintGraph2File


template <unsigned int DIM> void Agent<DIM>::PrintCommResult2File(std::string name,bool print_ID)
{
	std::string dir_name = ros::package::getPath("mams_astar") + "/data/";
	std::string fmt_name = ".txt";

	std::string full_name;
	if (print_ID)
	{
		full_name = dir_name +	name + std::to_string(agent_ID_)+ fmt_name;
	}else {
		full_name = dir_name +	name + fmt_name;
	}

    std::ofstream path_file(full_name);

		if( path_file.is_open() )
		{
				path_file << number_processed_msg_ <<std::endl;
				path_file << number_expanded_vtx_ << std::endl;
		}

    path_file.close();
}// PrintGraph2File


template <unsigned int DIM> void Agent<DIM>::PrintINFO()
{
	ROS_INFO("AGENT %d: num_vertices: %ld , num_msg_que : %ld",agent_ID_, vec_chosen_nodes_.size() , vec_msg_que_.size() ) ;
}// PrintGraph2File



template <unsigned int DIM> void Agent<DIM>::PrintOpenList()
{
		if(! mute_ )
	ROS_INFO("AGENT %d: QUE LIST ------------------------------------------ ", agent_ID_);
	multiset<BaseVertex*, WeightLess<BaseVertex> >::const_iterator pos = m_quCandidateVertices.begin();
	for(; pos != m_quCandidateVertices.end(); ++pos)
	{

		std:: cout << " " <<(*pos)->getID() <<":" << "(" << vec_chosen_nodes_[(*pos)->getID()].first << ")" << ", " ;

	}
	std::cout<<endl;


}

/* For debugging purpose print all the nodes inserted */
template <unsigned int DIM> void Agent<DIM>::PrintChosenNodes()
{
	for (typename std::vector<std::pair<Key<DIM>,int>>::iterator it = vec_chosen_nodes_.begin() ; it != vec_chosen_nodes_.end(); ++it){
		std::pair<Key<DIM>,int> node = *(it);
		std::cout <<  node.first <<" " << node.second <<std::endl;
	}
}


//////////////////////////////////// END LINE  ///////////////////////////////////////////
