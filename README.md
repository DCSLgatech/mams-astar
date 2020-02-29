# Multi-Agent Multi-Scale A*
Multi-Agent Multi-Scale A* is a shortest path planning algorithm for multi agents in multi scale graphs.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=KJ0cPXKepI0
" target="_blank"><img src="http://img.youtube.com/vi/KJ0cPXKepI0/0.jpg" 
alt="http://www.youtube.com/watch?feature=player_embedded&v=KJ0cPXKepI0" width="240" height="180" border="10" /></a>

## Install
``` bash
$ cd catkin_ws/src
$ catkin_init_workspace
$ git clone https://github.gatech.edu/DCSL/mams_astar.git
$ catkin build mams_astar
```

## Quick START
1. Open agent_node.cpp located in /src/nodes.
  In main, configure tree, initial and goal states, number of agents, location of sensors for graph construction. Save the change.
2. Compile again with catkin build mams_astar
``` bash
$ cd catkin_ws/src
$ catkin build mams_astar
```
3. Running
``` bash
$ roscore
$ rosrun mams_astar multiscale
```
4. Observe result:
Open data_observer.m in /data
Change the number of agents accordingly and Run


## Algorithm Overview
1. In main of agent_node.cpp, each agent is assigned with the same tree, but different sensor locations will be used to construct different graphs for each agent in PrepareAgent()
2. Each agent constructs a reduced graph using ConstructReducedGraph() : the construction rule is based on top-down recursive investigation of every node in the given tree. Each agent chooses specific nodes from the common tree using ChooseNodes4Vertices(): finer near the sensor and coarser farther away.
3. n_Search() does 1) process message, 2) expand the highest index vertex in que, 3) publish the expanded vertex via ROS messaging
4. Algorithm terminates when the finest goal vertex is processed by all agents


## Important object and data structure
### Data Elements:
1. State: Actual coordinates of the world, this will be normalized to positive integers depending on the depth of tree and boundary of the world.
2. Key: Key is an array of size DIM (dimension of search problem) of positive integers that assigns a unique ID to each node of tree. Each integer represents the spatial coordinate in each axis of the center of the hypercube. Key will be used to check the neighbor & nest relationship of two vertices.
3. Tree: Tree object will be created in main() by passing the name of the file that stores preprocessed wavelet coefficients of 2D map. See "data/cost_map/map_wavelet.txt". The tree object will be assigned to all agents for their graph construction.  

### Graph Elements:
1. Each agent stores a Graph instance that manages the vertices & edges.
2. Each agent alongside this Graph instance, stores following lists to carry vertices information:
   - a. vec< pair<Key, int> > *vec_chosen_nodes_* : the location and size of the vertex. size=-1 for removed vertex
   - b. vec< double > *vec_node_value_*: the information (risk, cost) contained in the vertex
  The index of these vecs corresponds to the vertex ID stored in Graph instance.

### Path Planning Elements (from kshortestpaths):
1. cost to come: 	map<BaseVertex*, double> *m_mpStartDistanceIndex*
2. predecessor vertex:  map<BaseVertex*, BaseVertex*> *m_mpPredecessorVertex*; //pointer to pointer
3. closed list: set<long> *m_stDeterminedVertices*; //closed vertices ID
4. open list: multiset<BaseVertex*, WeightLess<BaseVertex> > *m_quCandidateVertices*;
