///////////////////////////////////////////////////////////////////////////////
///  Graph.cpp
///  <TODO: insert file description here>
///
///  @remarks <TODO: insert remarks here>
///
///  @author Yan Qi @date 8/18/2010
///
///  $Id: Graph.cpp 65 2010-09-08 06:48:36Z yan.qi.asu $
///////////////////////////////////////////////////////////////////////////////

#include <limits>
#include <set>
#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>
#include "kshortestpaths/GraphElements.h"
#include "kshortestpaths/Graph.h"


namespace kshortestpaths{
const double Graph::DISCONNECT = (numeric_limits<double>::max)();


Graph::Graph( const string& file_name )
{
	_import_from_file(file_name);
}

Graph::Graph()
{

}

Graph::Graph( const Graph& graph )
{
	m_nVertexNum = graph.m_nVertexNum;
	m_nEdgeNum = graph.m_nEdgeNum;
	m_vtVertices.assign(graph.m_vtVertices.begin(),graph.m_vtVertices.end());
	m_mpFaninVertices.insert(graph.m_mpFaninVertices.begin(),graph.m_mpFaninVertices.end());
	m_mpFanoutVertices.insert(graph.m_mpFanoutVertices.begin(),graph.m_mpFanoutVertices.end());
	m_mpEdgeCodeWeight.insert(graph.m_mpEdgeCodeWeight.begin(),graph.m_mpEdgeCodeWeight.end());
	m_mpVertexIndex.insert(graph.m_mpVertexIndex.begin(),graph.m_mpVertexIndex.end());
}

Graph::~Graph(void)
{
	clear();
}

///////////////////////////////////////////////////////////////////////////////
///  public  _import_from_file
///  Construct the graph by importing the edges from the input file.
///
///  @param [in]       file_name const std::string &    The input graph file
///
///  This function doesn't return a value
///
///  @remarks The format of the file is as follows:
///   1. The first line has an integer as the number of vertices of the graph
///   2. Each line afterwards contains a directed edge in the graph:
///		     starting point, ending point and the weight of the edge.
///		 These values are separated by 'white space'.
///
///  @see <TODO: insert text here>
///
///  @author Yan Qi @date 5/29/2010
///////////////////////////////////////////////////////////////////////////////
void Graph::_import_from_file( const string& input_file_name )
{
	const char* file_name = input_file_name.c_str();

	//1. Check the validity of the file
	ifstream ifs(file_name);
	if (!ifs)
	{
		cerr << "The file " << file_name << " can not be opened!" << endl;
		exit(1);
	}

	//2. Reset the members of the class
	clear();

	//3. Start to read information from the input file.
	/// Note the format of the data in the graph file.
	//3.1 The first line has an integer as the number of vertices of the graph
	ifs >> m_nVertexNum;

	//3.2 In the following lines, each line contains a directed edge in the graph:
	///   the id of starting point, the id of ending point, the weight of the edge.
	///   These values are separated by 'white space'.
	int start_vertex, end_vertex;
	double edge_weight;
	int vertex_id = 0;

	while(ifs >> start_vertex)
	{
		if (start_vertex == -1)
		{
			break;
		}
		ifs >> end_vertex;
		ifs >> edge_weight;

		///3.2.1 construct the vertices
		BaseVertex* start_vertex_pt = get_vertex(start_vertex);
		BaseVertex* end_vertex_pt = get_vertex(end_vertex);

		///3.2.2 add the edge weight
		//// note that the duplicate edge would overwrite the one occurring before.
		m_mpEdgeCodeWeight[get_edge_code(start_vertex_pt, end_vertex_pt)] = edge_weight;

		///3.2.3 update the fan-in or fan-out variables
		//// Fan-in
		get_vertex_set_pt(end_vertex_pt, m_mpFaninVertices)->insert(start_vertex_pt);

		//// Fan-out
		get_vertex_set_pt(start_vertex_pt, m_mpFanoutVertices)->insert(end_vertex_pt);

	}

	if(m_nVertexNum != m_vtVertices.size())
	{
		cerr << "The number of nodes in the graph is "<<  m_vtVertices.size() << " instead of " << m_nVertexNum << endl;
		exit(1);
	}

	m_nVertexNum = m_vtVertices.size();
	m_nEdgeNum = m_mpEdgeCodeWeight.size();

	ifs.close();
}

void Graph::add_vertex(long node_id){
	BaseVertex* vertex_pt = NULL;
	vertex_pt = new BaseVertex();
	vertex_pt->setID(node_id);
	m_mpVertexIndex[node_id] = vertex_pt;
	m_vtVertices.push_back(vertex_pt);
	m_nVertexNum = m_vtVertices.size();
	//std::cout<<"vertex added: " << node_id<<std::endl;
}

void Graph::add_vertex(long node_id,double h){
	BaseVertex* vertex_pt = NULL;
	vertex_pt = new BaseVertex();
	vertex_pt->setID(node_id);
	vertex_pt->h(h);
	m_mpVertexIndex[node_id] = vertex_pt;
	m_vtVertices.push_back(vertex_pt);
	m_nVertexNum = m_vtVertices.size();
}

void Graph::add_edge(long start_vertex, long end_vertex, double edge_weight){
//	std::cout<< "add edge: "<< start_vertex << "," << end_vertex << "," << edge_weight << std::endl;
	///3.2.1 construct the vertices
	BaseVertex* start_vertex_pt = get_vertex(start_vertex);
	BaseVertex* end_vertex_pt = get_vertex(end_vertex);

	///3.2.2 add the edge weight
	//// note that the duplicate edge would overwrite the one occurring before.
	m_mpEdgeCodeWeight[get_edge_code(start_vertex_pt, end_vertex_pt)] = edge_weight;

	///3.2.3 update the fan-in or fan-out variables
	//// Fan-in
	get_vertex_set_pt(end_vertex_pt, m_mpFaninVertices)->insert(start_vertex_pt);

	//// Fan-out
	get_vertex_set_pt(start_vertex_pt, m_mpFanoutVertices)->insert(end_vertex_pt);
	m_nEdgeNum = m_mpEdgeCodeWeight.size();
}

BaseVertex* Graph::get_vertex( long node_id )
{
	if (m_stRemovedVertexIds.find(node_id) != m_stRemovedVertexIds.end())
	{
		return NULL;
	}else
	{
		BaseVertex* vertex_pt = NULL;
		const map<long, BaseVertex*>::iterator pos = m_mpVertexIndex.find(node_id);
		if (pos == m_mpVertexIndex.end())
		{
			long vertex_id = m_vtVertices.size();
			vertex_pt = new BaseVertex();
			vertex_pt->setID(node_id);
			m_mpVertexIndex[node_id] = vertex_pt;

			m_vtVertices.push_back(vertex_pt);
		}else
		{
			vertex_pt = pos->second;
		}

		return vertex_pt;
	}
}

void Graph::clear()
{
	m_nEdgeNum = 0;
	m_nVertexNum = 0;

	for(map<BaseVertex*, set<BaseVertex*>*>::const_iterator pos=m_mpFaninVertices.begin();
		pos!=m_mpFaninVertices.end(); ++pos)
	{
		delete pos->second;
	}
	m_mpFaninVertices.clear();

	for(map<BaseVertex*, set<BaseVertex*>*>::const_iterator pos=m_mpFanoutVertices.begin();
		pos!=m_mpFanoutVertices.end(); ++pos)
	{
		delete pos->second;
	}
	m_mpFanoutVertices.clear();


	m_mpEdgeCodeWeight.clear();

	//clear the list of vertices objects
	for_each(m_vtVertices.begin(), m_vtVertices.end(), DeleteFunc<BaseVertex>());
	m_vtVertices.clear();
	m_mpVertexIndex.clear();

	m_stRemovedVertexIds.clear();
	m_stRemovedEdge.clear();
}

long Graph::get_edge_code( const BaseVertex* start_vertex_pt, const BaseVertex* end_vertex_pt ) const
{
	/// Note that the computation below works only if
	/// the result is smaller than the maximum of an integer!
	return start_vertex_pt->getID()*m_nVertexNum+end_vertex_pt->getID();
}


set<BaseVertex*>* Graph::get_vertex_set_pt( BaseVertex* vertex_, map<BaseVertex*, set<BaseVertex*>*>& vertex_container_index )
{
	BaseVertexPt2SetMapIterator pos = vertex_container_index.find(vertex_);

	if(pos == vertex_container_index.end())
	{
		set<BaseVertex*>* vertex_set = new set<BaseVertex*>();
		pair<BaseVertexPt2SetMapIterator,bool> ins_pos =
			vertex_container_index.insert(make_pair(vertex_, vertex_set));

		pos = ins_pos.first;
	}

	return pos->second;
}


double Graph::get_edge_weight( const BaseVertex* source, const BaseVertex* sink )
{
	long source_id = source->getID();
	long sink_id = sink->getID();

	if (m_stRemovedVertexIds.find(source_id) != m_stRemovedVertexIds.end()
		|| m_stRemovedVertexIds.find(sink_id) != m_stRemovedVertexIds.end()
		|| m_stRemovedEdge.find(make_pair(source_id, sink_id)) != m_stRemovedEdge.end())
	{
		return DISCONNECT;
	}else
	{
		return get_original_edge_weight(source, sink);
	}
}


void Graph::get_adjacent_vertices( BaseVertex* vertex, set<BaseVertex*>& vertex_set )
{
	long starting_vt_id = vertex->getID();

	if (m_stRemovedVertexIds.find(starting_vt_id) == m_stRemovedVertexIds.end())
	{
		set<BaseVertex*>* vertex_pt_set = get_vertex_set_pt(vertex, m_mpFanoutVertices);
		for(set<BaseVertex*>::const_iterator pos=(*vertex_pt_set).begin();
			pos != (*vertex_pt_set).end(); ++pos)
		{
			long ending_vt_id = (*pos)->getID();
			if (m_stRemovedVertexIds.find(ending_vt_id) != m_stRemovedVertexIds.end()
				|| m_stRemovedEdge.find(make_pair(starting_vt_id, ending_vt_id)) != m_stRemovedEdge.end())
			{
				continue;
			}
			//
			vertex_set.insert(*pos);
		}
	}
}

void Graph::get_precedent_vertices( BaseVertex* vertex, set<BaseVertex*>& vertex_set )
{
	if (m_stRemovedVertexIds.find(vertex->getID()) == m_stRemovedVertexIds.end())
	{
		long ending_vt_id = vertex->getID();
		set<BaseVertex*>* pre_vertex_set = get_vertex_set_pt(vertex, m_mpFaninVertices);
		for(set<BaseVertex*>::const_iterator pos=(*pre_vertex_set).begin();
			pos != (*pre_vertex_set).end(); ++pos)
		{
			long starting_vt_id = (*pos)->getID();
			if (m_stRemovedVertexIds.find(starting_vt_id) != m_stRemovedVertexIds.end()
				|| m_stRemovedEdge.find(make_pair(starting_vt_id, ending_vt_id)) != m_stRemovedEdge.end())
			{
				continue;
			}
			//
			vertex_set.insert(*pos);
		}
	}
}

//Modifed by Jaein: 20190305
/* receives a node_id and returns a vector of neighbor_ids*/
void Graph::get_neighbor_vertices(long node_id, vector<long>& neighbor_ids){
	//if the node has not been removed
	if (m_stRemovedVertexIds.find(node_id) == m_stRemovedVertexIds.end())
	{
		BaseVertex* vertex = get_vertex(node_id);
		set<BaseVertex*> adjacent_vertices_set;
		set<BaseVertex*> precedent_vertices_set;
		get_adjacent_vertices(vertex, adjacent_vertices_set);
		get_precedent_vertices(vertex, precedent_vertices_set);
		for (set<BaseVertex*>::const_iterator pos=adjacent_vertices_set.begin();
			pos != adjacent_vertices_set.end(); ++pos){
			if( m_stRemovedEdge.find(std::pair<long, long>(node_id,(*pos)-> getID())) == m_stRemovedEdge.end() ){
				neighbor_ids.push_back((*pos)-> getID());

			}
		} // for adjacent
		for (set<BaseVertex*>::const_iterator pos=precedent_vertices_set.begin();
			pos != precedent_vertices_set.end(); ++pos){
			if(m_stRemovedEdge.find(std::pair<long, long>((*pos)-> getID(), node_id)) == m_stRemovedEdge.end() ) {
				neighbor_ids.push_back((*pos)-> getID());
			}
		}//for precedent
	}
}


double Graph::get_original_edge_weight( const BaseVertex* source, const BaseVertex* sink )
{
	map<long, double>::const_iterator pos =
		m_mpEdgeCodeWeight.find(get_edge_code(source, sink));

	if (pos != m_mpEdgeCodeWeight.end())
	{
		return pos->second;
	}else
	{
		return DISCONNECT;
	}
}

}
