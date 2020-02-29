/*******************************************************************************
*
* Copyright (C) 2020, Jaein Lim and Panagiotis Tsiotras
*
*
* This library is free software; you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free
* Software Foundation; either version 2.1 of the License, or any later version.
*
* This library is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY;  without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
* for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this library; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*
*******************************************************************************/

#include "std_msgs/String.h"
#include "agent_node.h"
#include <time.h>

AgentNode::AgentNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) :nh_(nh), private_nh_(private_nh)
{
  //subscribers
  //tree_sub_ = nh_.subscribe(nh_.resolveName("raw_tree"),1,&AgentNode::TreeCallback, this);
  cuckoo_egg_sub_ = nh_.subscribe(nh_.resolveName("cuckoo_egg"),1000, &AgentNode::n_CuckooEggCallback, this);
  active_agent_sub = nh_.subscribe(nh_.resolveName("active_agent"),100, &AgentNode::n_ActiveAgentCallback, this);

  //publishers
  cuckoo_egg_pub_ = nh_.advertise<mams_astar::Cuckoo>(nh_.resolveName("cuckoo_egg"),1);
  active_agent_pub_ = nh_.advertise<std_msgs::String>(nh_.resolveName("active_agent"),1);

}

AgentNode::~AgentNode() { }

void AgentNode::n_PrepareAgent(Agent<2>* agent)
 {
  agent_pt_ = agent;
  agent_pt_ -> Activate();
  agent_pt_ -> ConstructReducedGraph();
  agent_pt_ -> PrepareSearch();
}



void AgentNode::n_ClearSearch()
{

  agent_pt_ -> Activate();
  agent_pt_ -> ConstructReducedGraph();
  agent_pt_ -> PrepareSearch();
  agent_pt_ -> ClearMessageQue();

}



void AgentNode::n_ClearQue()
{
  agent_pt_->ClearMessageQue();
}


void AgentNode::n_CuckooEggCallback(const mams_astar::CuckooConstPtr& cuckoo_egg_msg)
{

  if( (cuckoo_egg_msg->agent_ID != agent_pt_->GetAgentID()) && (cuckoo_egg_msg->agent_ID!=0) )
  {
    if(  !(agent_pt_->IsMute()) )
    {
    std::cout << std::endl <<std::endl;
    ROS_INFO("AGENT %d: egg pushed KEY (%ld, %ld) from Agent%d NODE %ld @ %f",
      agent_pt_->GetAgentID(),cuckoo_egg_msg->node_key[0], cuckoo_egg_msg->node_key[1],
       cuckoo_egg_msg->agent_ID, cuckoo_egg_msg->node_ID,  cuckoo_egg_msg->cost_to_come);
     }
    agent_pt_->PushEgg(cuckoo_egg_msg);
  }
}

void AgentNode::n_ActiveAgentCallback(const std_msgs::StringConstPtr& active_agent_msg)
{
  ROS_INFO("active agent exists");

}

void AgentNode::n_Search()
{

  agent_pt_->ProcessMessage();

  mams_astar::Cuckoo egg;

  agent_pt_->ExpandToMessage(egg);

  cuckoo_egg_pub_.publish(egg);


}

bool AgentNode::n_ActivityCheck()
{
  return agent_pt_->IsActive();
}

void AgentNode::n_PrintPath(std::string name, bool print_ID){

  agent_pt_ -> PrintPath2File(name, print_ID);

}

void AgentNode::n_PrintCommResult(std::string name, bool print_ID){

  agent_pt_ -> PrintCommResult2File(name, print_ID);
}

void AgentNode::n_PrintGraph(std::string name, bool print_ID){

  agent_pt_ -> PrintGraph2File(name, print_ID);
}

void AgentNode::n_INFO(){

    agent_pt_ -> PrintINFO();

    agent_pt_ -> PrintPathInfo();
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "agent_node");

  // True linearSearch will make hop communication from one agent to another
  // False linearSearch will make all agent search concurrently
  bool linearSearch = false;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  AgentNode agent_node(nh,private_nh);
  AgentNode agent_node2(nh,private_nh);
  AgentNode agent_node3(nh,private_nh);
  AgentNode agent_node4(nh,private_nh);



  ROS_INFO("agent_nodeS created");
  clock_t graph_gen_time;
  clock_t tic_toc;
  tic_toc = clock();

  // Create a quadtree
  Tree<2>* original_tree=new Tree<2>();
	//Set Search Space Bounds
	State<2> minState={-1,-1};
	State<2> maxState={1,1};
	original_tree->setStateBounds(minState,maxState);

	//Set Tree Max Depth
	int depth=7;
	original_tree->setMaxDepth(depth);

  // Copy the tree from external source
  original_tree->copyWaveletTransform("cost_map/map_wavelet");

  tic_toc = clock() - tic_toc;
  printf ("Tree generation took %d clicks (%f seconds).\n",tic_toc,((float)tic_toc)/CLOCKS_PER_SEC);

  // This is the start and goal states of THE path planning problem
	State<2> init_state ={-0.68, -0.2};
	State<2> goal_state ={0.9, 0.5};

  // The agent will be located at the sensor state
  State<2> sensor_state = init_state;
  State<2> sensor_state2 = {0.7, 0.8};
  State<2> sensor_state3 = {0.35, -0.5};
  State<2> sensor_state4 = {-0.3, -0.5};


  // Configure multiple agents, //DO NOT ASSIGN AGENT ID 0
  Agent<2> agent(original_tree, &sensor_state, &init_state, &goal_state);
  agent.SetAgentID(1); //DO NOT ASSIGN AGENT ID 0
  agent.MuteOn();
  Agent<2> agent2(original_tree, &sensor_state2, &init_state, &goal_state);
  agent2.SetAgentID(2);
  agent2.MuteOn();
  Agent<2> agent3(original_tree, &sensor_state3, &init_state, &goal_state);
  agent3.SetAgentID(3);
  agent3.MuteOn();
  Agent<2> agent4(original_tree, &sensor_state4, &init_state, &goal_state);
  agent4.SetAgentID(4);
  agent4.MuteOn();


  graph_gen_time = clock();
  // This Construct first abstract graph based on its current sensor_state
  agent_node.n_PrepareAgent(&agent);
  agent_node2.n_PrepareAgent(&agent2);
  agent_node3.n_PrepareAgent(&agent3);
  agent_node4.n_PrepareAgent(&agent4);

  graph_gen_time = clock() - graph_gen_time;
  printf ("Reduced generation took %d clicks (%f seconds).\n",graph_gen_time,((float)graph_gen_time)/CLOCKS_PER_SEC);

  // Now put agents in the vector.
  std::vector<AgentNode> vec_agent_node;
  vec_agent_node.push_back(agent_node);
  vec_agent_node.push_back(agent_node2);
  vec_agent_node.push_back(agent_node3);
  vec_agent_node.push_back(agent_node4);

  ROS_INFO("agentS assigned to the node");


  ROS_INFO("agentS start state assigned, ready to search !");
  ROS_INFO("=================================================================");
  std::cout << std::endl <<std::endl;


  const int count_max = 10000;
  bool at_least_one_active ;
  int simple_counter;


  // If you wish, save the graph before search...
  for (std::vector<AgentNode>::iterator agent_node = vec_agent_node.begin();
        agent_node != vec_agent_node.end(); ++agent_node)
  {
            agent_node->n_PrintGraph("instance/pre_graph",true);
  }

  at_least_one_active = true;
  simple_counter=0;

  clock_t search_time;
  tic_toc = clock();

  /* ====================== LINEAR SEARCH ========================= */
  if (linearSearch)
  {
    std::vector<AgentNode>::reverse_iterator agent_node = vec_agent_node.rbegin();
    for (; agent_node != vec_agent_node.rend(); ++agent_node)
    {
      // First collect all subscribed queues.
      ros::spinOnce();

      // For each agent, search
      while(ros::ok()&& at_least_one_active)
      {
        at_least_one_active = agent_node->n_ActivityCheck();
        agent_node-> n_Search();
        simple_counter++;
        if(simple_counter > count_max){
          ROS_ERROR("Reached termination threshold! helper");
          exit(1);
        }
      }
      at_least_one_active= true;
      simple_counter = 0;
    }

  }
  /* ====================== CONCURRENT SEARCH ========================= */
  else
  {
    while (ros::ok() && at_least_one_active)
    {
      ros::spinOnce(); //collects all subscribed ques

      // Check if there exists at least one active agent
      std::vector<AgentNode>::iterator agent_node = vec_agent_node.begin();
      for (; agent_node != vec_agent_node.end(); ++agent_node)
      {
        if (agent_node->n_ActivityCheck())
        {
          at_least_one_active = true;
          break;
        }
      }
      if(agent_node == vec_agent_node.end())
        at_least_one_active = false;


      // Now for each agent, do search
      for (std::vector<AgentNode>::iterator agent_node = vec_agent_node.begin();
            agent_node != vec_agent_node.end(); ++agent_node)
      {
        agent_node->n_Search();
      }

      // Check if the maximum termination threshold has exceeded!
      simple_counter++;
      if(simple_counter > count_max){
        ROS_ERROR("Reached termination threshold!");
        exit(1);
      }
    }
  }


  search_time = ( clock()-tic_toc );

  ROS_INFO("coarse search is over with iteration %d ", simple_counter);

  // Save the result into txt files
  for (std::vector<AgentNode>::iterator agent_node = vec_agent_node.begin();
        agent_node != vec_agent_node.end(); ++agent_node)
  {
    agent_node->n_INFO();
    agent_node->n_PrintGraph("instance/graph",true);
    agent_node->n_PrintPath("instance/path",true);
  }



  printf ("Search took %d clicks (%f seconds).\n",search_time,((float)search_time)/CLOCKS_PER_SEC);
  printf ("Total time %d clicks (%f seconds).\n",graph_gen_time+search_time,((float)(graph_gen_time+search_time))/CLOCKS_PER_SEC);

  return 0;
}
