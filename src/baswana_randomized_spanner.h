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
#include "set_member_predicate_property_map.hpp"

namespace boost
{

namespace detail
{

    // The second phase works only on G'=(V',E'), the subgraph of G where
    // vertices in V' is either a sampled vertex (in R) or adjacent to one.
    // E' are inter-cluster edges.
    template <class Graph, class Weight, class SpannerEdge,
        class Center, class Cluster, class Auxiliary>
    void baswana_link_clusters(const Graph& G, const Weight weight,
            const Center center, const Cluster N,
            SpannerEdge spanner_edge, Auxiliary A)
    {
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        typename graph_traits<Graph>::vertices_size_type n = num_vertices(G);

        typedef typename graph_traits<Graph>::edge_descriptor Edge;
        typename graph_traits<Graph>::edges_size_type m = num_edges(G);

        std::set<Vertex> null_edge;

        unsigned int phase2 = 0;
        foreach(Vertex v, vertices(G))
        {
            // Initialize each element of A to null.
            null_edge.clear();

            // Find the shortest edge between this node and other clusters.
            foreach(Edge e, out_edges(v, G))
            {
                const Vertex w = target(e, G);
                const Vertex x = N[w];
                if (null_edge.find(x) == null_edge.end() ||
                        weight[e] < weight[A[x]])
                {
                    A[x] = e;
                    null_edge.insert(x);
                }
            }

            // Add the shortest edges to the spanner.
            foreach(Vertex u, vertices(G))
            {
                if (center[u] == true)
                {                                        
                    if (null_edge.find(u) != null_edge.end())
                    {
                        if (!spanner_edge[A[u]]) ++phase2;
                        spanner_edge[A[u]] = true;
                    }
                }
            }
        }
        std::cout << "Intercluster edges: " << phase2 << std::endl;
    }

    template <class Graph, class Weight, class SpannerEdge, class Cluster>
    void baswana_randomized_3_spanner_impl(const Graph& G,
            Weight weight, SpannerEdge spanner_edge, Cluster N,
            ompl::RNG rand)
    {
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        typename graph_traits<Graph>::vertices_size_type n = num_vertices(G);

        typedef typename graph_traits<Graph>::edge_descriptor Edge;
        typename graph_traits<Graph>::edges_size_type m = num_edges(G);
        function_requires<IncidenceGraphConcept<Graph> >();

        // Weights are edge properties and are comparable
        function_requires<ReadablePropertyMapConcept<Weight, Edge> >();
        typedef typename property_traits<Weight>::value_type W_value;
        function_requires<ComparableConcept<W_value> >();

        function_requires<WritablePropertyMapConcept<SpannerEdge, Edge> >();
        typedef typename property_traits<SpannerEdge>::value_type EP_value;
        function_requires<Convertible<EP_value, bool> >();
        function_requires<Convertible<bool, EP_value> >();

        function_requires<ReadWritePropertyMapConcept<Cluster, Vertex> >();
        typedef typename property_traits<Cluster>::value_type VC_value;
        function_requires<EqualityComparableConcept<VC_value> >();
        
        // Make a vertex a cluster center with a probability p_center
        unordered_set<Vertex> cluster_center;
        const double p_center = 12 * 1.0 / sqrt((double)n);
        foreach(Vertex v, vertices(G))
            if (rand.uniform01() <= p_center)
                cluster_center.insert(v);
        std::cout << "Sampled cluster centers:" << cluster_center.size() << std::endl;

        // Where do the edges come from?
        unsigned int phase1a = 0;
        unsigned int phase1b = 0;
        unsigned int phase1c = 0;

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
                    if (cluster_center.find(target(e, G)) != cluster_center.end())
                        center_neighbors.push_back(e);

                if (center_neighbors.empty())
                {
                    // No cluster center neighbors. Keep all edges.
                    foreach(Edge e, out_edges(v, G))
                    {
                        if (!spanner_edge[e]) ++phase1a;
                        spanner_edge[e] = true;
                    }
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
                    if (!spanner_edge[closest]) ++phase1b;
                    put(spanner_edge, closest, true);
                    put(N, v, target(closest, G));

                    // Add edges with smaller weights to the nearest center.
                    foreach(Edge e, out_edges(v, G))
                    {
                        double w = get(weight, e);
                        if (w < smallest_weight)
                        {
                            if (!spanner_edge[e]) ++phase1c;
                            put(spanner_edge, e, true);
                        }
                    }

                }
            }
        }
        std::cout << "Noncluster node edges: " << phase1a << std::endl;
        std::cout << "Node to cluster center edges: " << phase1b << std::endl;
        std::cout << "Other cluster edges: " << phase1c << std::endl;

        // Discard all edges in E' that aren't connected to a center
        // and are in the same cluster.
        Graph G_prime(n);
        foreach(Edge e, edges(G))
        {
            const Vertex v1 = source(e, G);
            const bool v1_is_center =
                cluster_center.find(v1) != cluster_center.end();
            
            const Vertex v2 = target(e, G);
            const bool v2_is_center =
                cluster_center.find(v2) != cluster_center.end();

            if(v1_is_center || v2_is_center
                    || get(N, v1) != get(N, v2))
            {
                add_edge(source(e, G), target(e, G), G_prime);
                // TODO: Use copy.
            }
        }

        // End of first phase.

        std::vector<Edge> A(n);
        const_set_member_predicate_property_map<unordered_set<Vertex> >
            center(cluster_center);
        baswana_link_clusters(G_prime, weight, center, N, spanner_edge, &A[0]);
    }

} // namespace detail


template <class Graph, class SpannerEdge>
void baswana_randomized_3_spanner(const Graph& G, SpannerEdge spanner_edge)
{
    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
    const typename graph_traits<Graph>::vertices_size_type n = num_vertices(G);

    typename boost::property_map<Graph, edge_weight_t>::const_type
            weight = get(edge_weight, G);

    ompl::RNG rand;
    
    // Initialize each vertex to its own color.
    std::vector<Vertex> cluster(n);
    typename graph_traits<Graph>::vertex_iterator v_begin, v_end;
    tie(v_begin, v_end) = vertices(G);
    std::copy(v_begin, v_end, cluster.begin());

    detail::baswana_randomized_3_spanner_impl(G,
        weight, spanner_edge, &cluster[0], rand);
}

}

#endif

