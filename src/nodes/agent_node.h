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


#ifndef MULTISCALE_AGENT_NODE_H
#define MULTISCALE_AGENT_NODE_H

#include "mams_astar/Cuckoo.h"
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "agent.h"
#include "Key.h"

class AgentNode {
 public:
   AgentNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~ AgentNode();

  void n_PrepareAgent(Agent<2>* agent);

  void n_Search();
  bool n_ActivityCheck();
  void n_ClearSearch();
  void n_ClearQue();

  void n_PrintGraph(std::string name,bool print_ID);
  void n_PrintPath(std::string name, bool print_ID);
  void n_PrintCommResult(std::string name, bool print_ID);
  void n_INFO();

 private:

  Agent<2>* agent_pt_;

  bool all_agent_inactive_;

  //ros stuff
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  //subscribers
  ros::Subscriber cuckoo_egg_sub_;
  ros::Subscriber active_agent_sub;
  //publishers
  ros::Publisher cuckoo_egg_pub_;
  ros::Publisher active_agent_pub_;

  //Callbacks

  void n_CuckooEggCallback(const mams_astar::CuckooConstPtr& cuckoo_egg_msg);
  void n_ActiveAgentCallback(const std_msgs::StringConstPtr& active_agent_msg);



};


#endif //MULTISCALE_AGENT_NODE_H
