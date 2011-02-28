/* 
 * File:   RandomizedSpanner.h
 * Author: jdmarble
 *
 * Created on February 25, 2011, 10:26 PM
 */

#ifndef BASWANA_RANDOMIZED_SPANNER_H
#define	BASWANA_RANDOMIZED_SPANNER_H

#include <ompl/util/RandomNumbers.h>

#define BOOST_NO_HASH
#include <boost/config.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <set>

namespace boost
{

namespace detail
{

    template <class Graph, class Weight, class EdgeParity, class VertexColor>
    void baswana_randomized_3_spanner_impl(const Graph& G,
            Weight weight, EdgeParity spanner_edge, VertexColor cluster,
            ompl::RNG rand)
    {
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        typename graph_traits<Graph>::vertices_size_type n = num_vertices(G);

        typedef typename graph_traits<Graph>::edge_descriptor Edge;
        typename graph_traits<Graph>::edges_size_type m = num_edges(G);
\
        function_requires<IncidenceGraphConcept<Graph> >();

        // Weights are edge properties and are comparable
        function_requires<ReadablePropertyMapConcept<Weight, Edge> >();
        typedef typename property_traits<Weight>::value_type W_value;
        function_requires<ComparableConcept<W_value> >();

        function_requires<WritablePropertyMapConcept<EdgeParity, Edge> >();
        typedef typename property_traits<EdgeParity>::value_type EP_value;
        function_requires<Convertible<EP_value, bool> >();
        function_requires<Convertible<bool, EP_value> >();

        function_requires<ReadWritePropertyMapConcept<VertexColor, Vertex> >();
        typedef typename property_traits<VertexColor>::value_type VC_value;
        function_requires<EqualityComparableConcept<VC_value> >();
\
        
        // Make a vertex a cluster center with a probability p_center
        unordered_set<Vertex> cluster_center;
        const double p_center = 1.0 / sqrt((double)n);
        foreach(Vertex v, vertices(G))
            if (rand.uniform01() <= p_center)
                cluster_center.insert(v);

        // Add each vertex to a cluster, or just keep all edges.
        std::vector<Edge> center_neighbors;
        foreach(Vertex v, vertices(G))
        {
            // Only process vertices that were not sampled.
            if (cluster_center.find(v) == cluster_center.end())
            {
                // Find out if any neighbors are cluster centers.
                center_neighbors.clear();
                foreach(Edge e, out_edges(v, G))
                {
                    if (cluster_center.find(target(e, G)) != cluster_center.end())
                        center_neighbors.push_back(e);
                }

                if (center_neighbors.empty())
                {
                    // No cluster center neighbors. Keep all edges.
                    foreach(Edge e, out_edges(v, G))
                        put(spanner_edge, e, EP_value(1));
                }
                else
                {
                    // Find nearest neighbor in R.
                    Edge closest = center_neighbors.front();
                    double smallest_weight = get(weight, closest);
                    foreach(Edge e, center_neighbors)
                    {
                        double w = get(weight, e);
                        if (w < smallest_weight)
                        {
                            closest = e;
                            smallest_weight = w;
                        }
                    }
                    put(spanner_edge, closest, true);
                    put(cluster, v, target(closest, G));

                    // Add edges with smaller weights to the nearest center.
                    foreach(Edge e, out_edges(v, G))
                    {
                        double w = get(weight, e);
                        if (w < smallest_weight)
                            put(spanner_edge, e, true);
                    }

                }
            }
        }

        // Discard all edges in E' that aren't connected to a center
        // and are in the same cluster.
        std::set<Edge> e_prime;
        std::set<Vertex> v_prime;
        foreach(Edge e, edges(G))
        {
            const Vertex v1 = source(e, G);
            const bool v1_is_center =
                cluster_center.find(v1) != cluster_center.end();
            
            const Vertex v2 = target(e, G);
            const bool v2_is_center =
                cluster_center.find(v2) != cluster_center.end();

            if(v1_is_center || v2_is_center
                    || get(cluster, v1) != get(cluster, v2))
            {
                e_prime.insert(e);
                v_prime.insert(v1);
                v_prime.insert(v2);
            }
        }

        // End of first phase.
        foreach(Vertex v, v_prime)
        {
            foreach(Vertex c, cluster_center)
            {
                Edge smallest_edge;
                W_value smallest_weight = 1000000; //TODO: LOL

                foreach(Edge e, out_edges(v, G))
                {
                    if(e_prime.find(e) != e_prime.end() &&
                            get(cluster, target(e, G)) == c &&
                            get(weight, e) < smallest_weight)
                    {
                        smallest_weight = get(weight, e);
                        smallest_edge = e;
                    }
                }
                put(spanner_edge, smallest_edge, true);
            }
        }
    }

} // namespace detail


template <class Graph, class OutputIterator>
void baswana_randomized_3_spanner(const Graph& G, OutputIterator spanner_edges)
{
    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
    typename graph_traits<Graph>::vertices_size_type n = num_vertices(G);

    typedef typename graph_traits<Graph>::edge_descriptor Edge;
    typename graph_traits<Graph>::edges_size_type m = num_edges(G);

    typename boost::property_map<Graph, vertex_index_t>::const_type
            vertex_id = get(vertex_index, G);
    typename boost::property_map<Graph, edge_weight_t>::const_type
            weight = get(edge_weight, G);

    ompl::RNG rand;
    
    // Initialize each vertex to its own color.
    std::vector<Vertex> cluster(n);
    typename graph_traits<Graph>::vertex_iterator v_begin, v_end;
    tie(v_begin, v_end) = vertices(G);
    std::copy(v_begin, v_end, cluster.begin());

    // Intialize the spanner edges to empty and create a property map
    std::map<Edge, bool> spanner_map;
    foreach(Edge e, edges(G))
        spanner_map[e] = false;
    associative_property_map<std::map<Edge, bool> > spanner_edge(spanner_map);

    detail::baswana_randomized_3_spanner_impl(G,
        weight, spanner_edge, &cluster[0], rand);

    foreach(Edge e, edges(G))
        if (get(spanner_edge, e) == true)
            *spanner_edges++ = e;
}

}

#endif

