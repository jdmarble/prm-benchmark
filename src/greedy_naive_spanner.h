/* 
 * File:   greedy_naive_spanner.h
 * Author: jdmarble
 *
 * Created on December 13, 2011
 */

#ifndef GREEDY_NAIVE_SPANNER_H
#define	GREEDY_NAIVE_SPANNER_H

#define BOOST_NO_HASH
#include <boost/config.hpp>

#include <boost/bind.hpp>

#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/named_function_params.hpp>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/ConnectionFilter.h>


using namespace boost;

#include <algorithm>
#include <vector>

using namespace std;


typedef ompl::geometric::PRM::Graph Graph;
typedef ompl::geometric::PRM::Vertex Vertex;
typedef ompl::geometric::PRM::Edge Edge;



class GreedyNaiveSpanner
{
    struct EdgeInfo
    {
      Vertex u, v;
      double weight;
  
      EdgeInfo(const Vertex u, const Vertex v, const double weight) : u(u), v(v), weight(weight) {}
    };

    bool ascending_weight(const EdgeInfo& a, const EdgeInfo& b)
    {
      return a.weight < b.weight;
    }

    Graph& G;

    std::vector<EdgeInfo> edge_queue;

    const ompl::geometric::IRSFilter filter;
    
public:

 GreedyNaiveSpanner(Graph& G, const double t, 
		    boost::function<double (const Vertex, const Vertex)>& d)
   : G(G), filter(G, t, d)
    {
    }

    void calculateSpanner()
    {
      // Put all edges in a queue
      foreach(const Edge& e, edges(G))
	edge_queue.push_back(EdgeInfo(source(e, G), target(e, G), G[e].weight));

      // Sort edges by weight.
      // We want the smallest edges processed first, so sort in ascending order.
      sort(edge_queue.begin(), edge_queue.end(),
	   bind(&GreedyNaiveSpanner::ascending_weight, this, _1, _2));

      // Remove all edges from the spanner
      foreach(const Vertex& v, vertices(G))
	clear_vertex(v, G);

      foreach(const EdgeInfo& e, edge_queue)
	{

	  if (filter(e.u, e.v))
	  {
	    Edge e_new;
	    bool added;
	    boost::tie(e_new, added) = add_edge(e.u, e.v, G);
	    if (!added)
	    {
	      std::cerr << "Can't add and edge!" << std::endl;
	      exit(1);
	    }
	    G[e_new].weight = e.weight;
	  }
	}
    }
};

#endif

