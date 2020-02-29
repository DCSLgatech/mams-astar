///////////////////////////////////////////////////////////////////////////////
///  Graph.h
///  <TODO: insert file description here>
///
///  @remarks <TODO: insert remarks here>
///
///  @author Yan Qi @date 8/18/2010
///
///  $Id: Graph.h 65 2010-09-08 06:48:36Z yan.qi.asu $
///////////////////////////////////////////////////////////////////////////////


#pragma once
#include "kshortestpaths/GraphElements.h"
#include <set>
#include <map>

using namespace std;
namespace kshortestpaths{

class Path : public BasePath
{
public:

	Path(const std::vector<BaseVertex*>& vertex_list, double weight):BasePath(vertex_list,weight){}

	// display the content
	void PrintOut(std::ostream& out_stream) const
	{
		out_stream << "Cost: " << m_dWeight << " Length: " << m_vtVertexList.size() << std::endl;
		for(std::vector<BaseVertex*>::const_iterator pos=m_vtVertexList.begin(); pos!=m_vtVertexList.end();++pos)
		{
			out_stream << (*pos)->getID() << " ";
		}
		out_stream << std::endl <<  "*********************************************" << std::endl;
	}
};

class Graph
{
public: // members

	const static double DISCONNECT;

	typedef set<BaseVertex*>::iterator VertexPtSetIterator;
	typedef map<BaseVertex*, set<BaseVertex*>*>::iterator BaseVertexPt2SetMapIterator;

protected: // members

	// Basic information
	map<BaseVertex*, set<BaseVertex*>*> m_mpFanoutVertices;
	map<BaseVertex*, set<BaseVertex*>*> m_mpFaninVertices;
	map<long, double> m_mpEdgeCodeWeight;
	vector<BaseVertex*> m_vtVertices;
	long m_nEdgeNum;
	long m_nVertexNum;

	map<long, BaseVertex*> m_mpVertexIndex;

	// Members for graph modification
	set<long> m_stRemovedVertexIds;
	set<pair<long,long> > m_stRemovedEdge;

public:

	// Constructors and Destructor
	Graph(const string& file_name);
	Graph();
	Graph(const Graph& rGraph);
	~Graph(void);

	void clear();

	void add_vertex(long node_id);
	void add_vertex(long node_id,double h);
	void add_edge(long start_vertex, long end_vertex, double edge_weight);

	BaseVertex* get_vertex(long node_id);

	long get_edge_code(const BaseVertex* start_vertex_pt, const BaseVertex* end_vertex_pt) const;
	set<BaseVertex*>* get_vertex_set_pt(BaseVertex* vertex_, map<BaseVertex*, set<BaseVertex*>*>& vertex_container_index);

	double get_original_edge_weight(const BaseVertex* source, const BaseVertex* sink);

	double get_edge_weight(const BaseVertex* source, const BaseVertex* sink);
	void get_adjacent_vertices(BaseVertex* vertex, set<BaseVertex*>& vertex_set);
	void get_precedent_vertices(BaseVertex* vertex, set<BaseVertex*>& vertex_set);

	//Modifed Jaein: 20190305
	void get_neighbor_vertices(long node_id, vector<long>& neighbor_ids);

	/// Methods for changing graph
	void remove_edge(const pair<long,long> edge)
	{
		m_stRemovedEdge.insert(edge);
	}

	void remove_vertex(const long vertex_id)
	{
		m_stRemovedVertexIds.insert(vertex_id);
	}

	void recover_removed_edges()
	{
		m_stRemovedEdge.clear();
	}

	void recover_removed_vertices()
	{
		m_stRemovedVertexIds.clear();
	}

	void recover_removed_edge(const pair<long,long> edge)
	{
		m_stRemovedEdge.erase(m_stRemovedEdge.find(edge));
	}

	void recover_removed_vertex(long vertex_id)
	{
		m_stRemovedVertexIds.erase(m_stRemovedVertexIds.find(vertex_id));
	}

private:
	void _import_from_file(const std::string& file_name);

};
}
