/* 
 * File:   remove_random_edges.h
 * Author: jdmarble
 *
 * Created on December 13, 2011
 */

#ifndef REMOVE_RANDOM_EDGES_H
#define	REMOVE_RANDOM_EDGES_H

#define BOOST_NO_HASH
#include <boost/config.hpp>

#include <boost/bind.hpp>

#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/named_function_params.hpp>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/variate_generator.hpp>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/ConnectionFilter.h>


using namespace boost;

#include <algorithm>
#include <vector>

using namespace std;


typedef ompl::geometric::PRM::Graph Graph;
typedef ompl::geometric::PRM::Vertex Vertex;
typedef ompl::geometric::PRM::Edge Edge;



class RemoveRandomEdges
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

    std::vector<EdgeInfo> kept_edges;

    variate_generator<rand48, uniform_01<double> > uniform01;

    const double rate;

public:

 RemoveRandomEdges(Graph& G, const int seed, const double rate)
   : G(G), rate(rate), uniform01(rand48(seed), uniform_01<double>())
    {
    }

    void calculateSpanner()
    {
      // Don't put an edge in the new graph with probability `rate` (0-1). 
      foreach(const Edge& e, edges(G))
        {
          if (uniform01() > rate)
            {
	      kept_edges.push_back(EdgeInfo(source(e, G), target(e, G), G[e].weight));
            }
        }

      // Remove all edges from the spanner
      foreach(const Vertex& v, vertices(G))
	clear_vertex(v, G);

      foreach(const EdgeInfo& e, kept_edges)
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
};

#endif

