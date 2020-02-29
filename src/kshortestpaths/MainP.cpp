/************************************************************************/
/* $Id: MainP.cpp 65 2010-09-08 06:48:36Z yan.qi.asu $                                                                 */
/************************************************************************/

#include <limits>
#include <set>
#include <map>
#include <queue>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include "GraphElements.h"
#include "Graph.h"
#include "DijkstraShortestPathAlg.h"
#include "YenTopKShortestPathsAlg.h"

using namespace std;
using namespace kshortestpaths;


void testDijkstraGraph()
{
	kshortestpaths::Graph my_graph;
	for(int i=0;i<10;++i){
		for(int j=0;j<10;++j){
			my_graph.add_vertex(10*i+j,sqrt(i*i+j*j));
		}
	}
	for(int i=9;i>0;--i){
		for(int j=9;j>0;--j){
			my_graph.add_edge(10*i+j,10*i+j-1,1.0);
			my_graph.add_edge(10*i+j,10*(i-1)+j,1.0);
			my_graph.add_edge(10*i+j,10*(i-1)+j-1,1.5);
		}
	}

	//Graph* my_graph_pt = new Graph("../data/test_1");
	DijkstraShortestPathAlg shortest_path_alg(&my_graph);
	BasePath* result =
		shortest_path_alg.get_shortest_path(
				my_graph.get_vertex(99), my_graph.get_vertex(0));
	result->PrintOut(cout);
//	std::map<BaseVertex*, double> updated_nodes=shortest_path_alg.m_mpStartDistanceIndex;
//	for(int i=0;i<10;++i){
//		for(int j=0;j<10;++j){
//				if(updated_nodes.find(my_graph.get_vertex(10*i+j))!=updated_nodes.end()){
//					std::cout<< "X";
//				}else{
//					std::cout<< "O";
//				}
//		}
//		std::cout<< std::endl;
//	}
}

void testDijkstraGraphDisconnectedGraph()
{
	cout << "Test disconnected graph" << endl;
	Graph graph;
	graph.add_vertex(0);
	graph.add_vertex(1);
	DijkstraShortestPathAlg shortest_path_alg(&graph);
	BasePath* result =
		shortest_path_alg.get_shortest_path(
				graph.get_vertex(0), graph.get_vertex(1));
	result->PrintOut(cout);
}

void testYenAlg()
{
	Graph my_graph("/home/florian/workspace/3D_MSP/3rdparty/kshortestpaths/data/test_1");

	YenTopKShortestPathsAlg yenAlg(my_graph, my_graph.get_vertex(0),
		my_graph.get_vertex(5));

	int i=0;
	while(yenAlg.has_next())
	{
		++i;
		yenAlg.next()->PrintOut(cout);
	}

// 	System.out.println("Result # :"+i);
// 	System.out.println("Candidate # :"+yenAlg.get_cadidate_size());
// 	System.out.println("All generated : "+yenAlg.get_generated_path_size());

}
void testYenAlg2()
{
	kshortestpaths::Graph my_graph;
	for(int i=0;i<8;++i){
		my_graph.add_vertex(i);
	}
	for(int i=0;i<5;++i){
		my_graph.add_edge(i,i+1,0.4);
		my_graph.add_edge(i,i+2,0.4);
		my_graph.add_edge(i,i+3,0.4);
	}
	my_graph.add_edge(5,5+1,0.4);
	my_graph.add_edge(5,5+2,0.4);
	my_graph.add_edge(5,0,0.4);
	my_graph.add_edge(6,6+1,0.4);
	my_graph.add_edge(6,0,0.4);
	my_graph.add_edge(6,1,0.4);
	my_graph.add_edge(7,0,0.4);
	my_graph.add_edge(7,1,0.4);
	my_graph.add_edge(7,2,0.4);

	YenTopKShortestPathsAlg yenAlg(my_graph, my_graph.get_vertex(6),
		my_graph.get_vertex(7));

	int i=0;
	while(yenAlg.has_next())
	{
		++i;
		yenAlg.next()->PrintOut(cout);
	}

// 	System.out.println("Result # :"+i);
// 	System.out.println("Candidate # :"+yenAlg.get_cadidate_size());
// 	System.out.println("All generated : "+yenAlg.get_generated_path_size());

}

int main(...)
{
//	cout << "Welcome to the real world!" << endl;

	testDijkstraGraph();
//	cout << "Yen" << endl;
//	testYenAlg();
//	cout << "Yen 2" << endl;
//	testYenAlg2();
//	testDijkstraGraphDisconnectedGraph();
}
