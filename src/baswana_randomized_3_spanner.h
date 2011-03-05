/* 
 * File:   baswana_randomized_3_spanner.h
 * Author: jdmarble
 *
 * Created on February 25, 2011, 10:26 PM
 */

#ifndef BASWANA_RANDOMIZED_3_SPANNER_H
#define	BASWANA_RANDOMIZED_3_SPANNER_H

#include <ompl/util/RandomNumbers.h>

#define BOOST_NO_HASH
#include <boost/config.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace boost
{

namespace detail
{
    template <typename Graph, typename ClusterMap>
    struct intercluster_subgraph {
        intercluster_subgraph() { }
        intercluster_subgraph(const Graph& G, const ClusterMap& N)
            : G(&G), N(&N) { }

        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        typedef typename graph_traits<Graph>::edge_descriptor Edge;

        bool operator()(const Edge& e) const {
            const Vertex v1 = source(e, *G);
            //if ((*N)[v1] == v1) return false;

            const Vertex v2 = target(e, *G);
            //if ((*N)[v2] == v2) return false;

            return (*N)[v1] != (*N)[v2];
        }

        const Graph* G;
        const ClusterMap* N;
    };

    // The second phase works only on G'=(V',E'), the subgraph of G where
    // vertices in V' is either a sampled vertex (in R) or adjacent to one.
    // E' are inter-cluster edges.
    template <class Graph, class Weight, class SpannerEdge,
        class Cluster, class Auxiliary>
    void baswana_link_clusters_impl(const Graph& G, const Weight weight,
            const Cluster N, SpannerEdge spanner_edge, Auxiliary A)
    {
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        typename graph_traits<Graph>::vertices_size_type n = num_vertices(G);

        typedef typename graph_traits<Graph>::edge_descriptor Edge;
        typename graph_traits<Graph>::edges_size_type m = num_edges(G);

        std::set<Vertex> non_null_edge;

        unsigned int phase2 = 0;
        foreach(Vertex v, vertices(G))
        {
            // Initialize each element of A to null.
            non_null_edge.clear();

            // Find the shortest edge between this node and other clusters.
            foreach(Edge e, out_edges(v, G))
            {
                const Vertex w = target(e, G);
                const Vertex x = N[w];
                if (non_null_edge.find(x) == non_null_edge.end() ||
                        weight[e] < weight[A[x]])
                {
                    A[x] = e;
                    non_null_edge.insert(x);
                }
            }

            // Add the shortest edges to the spanner.
            foreach(Vertex u, non_null_edge)
            {
                *spanner_edge++ = A[u];
                ++phase2;
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
        Vertex null_vertex = graph_traits<Graph>::null_vertex();

        typedef typename graph_traits<Graph>::edge_descriptor Edge;
        typename graph_traits<Graph>::edges_size_type m = num_edges(G);

        {   // Type contraints.
            function_requires<IncidenceGraphConcept<Graph> >();

            // Weights are edge properties and are comparable
            function_requires<ReadablePropertyMapConcept<Weight, Edge> >();
            typedef typename property_traits<Weight>::value_type W_value;
            function_requires<ComparableConcept<W_value> >();

            /* Mke this an output iterator
            function_requires<WritablePropertyMapConcept<SpannerEdge, Edge> >();
            typedef typename property_traits<SpannerEdge>::value_type EP_value;
            function_requires<Convertible<EP_value, bool> >();
            function_requires<Convertible<bool, EP_value> >();
             */

            function_requires<ReadWritePropertyMapConcept<Cluster, Vertex> >();
            typedef typename property_traits<Cluster>::value_type VC_value;
            function_requires<EqualityComparableConcept<VC_value> >();
        }        

        // Where do the edges come from?
        unsigned int phase1a = 0;
        unsigned int phase1b = 0;
        unsigned int phase1c = 0;

        // Add each vertex to a cluster, or just keep all edges.
        std::vector<Edge> center_neighbors;
        foreach(Vertex v, vertices(G))
        {
            // Only process vertices that were not sampled.
            if (N[v] == null_vertex)
            {
                // Find out if any neighbors are cluster centers.
                center_neighbors.clear();
                foreach(Edge e, out_edges(v, G))
                    if (N[target(e, G)] == target(e, G))
                        center_neighbors.push_back(e);

                if (center_neighbors.empty())
                {
                    // No cluster center neighbors. Keep all edges.
                    foreach(Edge e, out_edges(v, G))
                    {
                        *spanner_edge++ = e;
                        ++phase1a;
                    }
                }
                else
                {
                    // Find nearest neighbor in R.
                    Edge closest = center_neighbors.front();
                    double smallest_weight = weight[closest];
                    foreach(Edge e, center_neighbors)
                    {
                        double w = weight[e];
                        if (w < smallest_weight)
                        {
                            closest = e;
                            smallest_weight = w;
                        }
                    }                    
                    *spanner_edge++ = closest;
                    ++phase1b;
                    N[v] = target(closest, G);

                    // Add edges with smaller weights to the nearest center.
                    foreach(Edge e, out_edges(v, G))
                    {
                        double w = weight[e];
                        if (w < smallest_weight)
                        {
                            *spanner_edge++ = e;
                            ++phase1c;
                        }
                    }

                }
            }
        }
        std::cout << "Noncluster node edges: " << phase1a << std::endl;
        std::cout << "Node to cluster center edges: " << phase1b << std::endl;
        std::cout << "Other cluster edges: " << phase1c << std::endl;

    }

} // namespace detail

template <typename Graph>
struct compare_degree
{
    const Graph& G;
    compare_degree(const Graph& G) : G(G) {}

    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
    bool operator()(const Vertex& v1, const Vertex& v2)
    {
        return out_degree(v1, G) < out_degree(v2, G);
    }
};

template <class Graph, class SpannerEdge>
void baswana_randomized_3_spanner(const Graph& G, SpannerEdge spanner_edge,
        bool cluster_heur = false)
{
    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
    typedef typename graph_traits<Graph>::vertices_size_type vertices_size_t;
    const vertices_size_t n = num_vertices(G);
    Vertex null_vertex = graph_traits<Graph>::null_vertex();

    typedef typename graph_traits<Graph>::edge_descriptor Edge;
    const typename graph_traits<Graph>::edges_size_type m = num_edges(G);

    typename boost::property_map<Graph, edge_weight_t>::const_type
            weight = get(edge_weight, G);



    ompl::RNG rand;
    
    // Initialize each vertex cluster to itself.
    typedef vector_property_map<Vertex> ClusterMap;
    ClusterMap N(n);
    foreach(Vertex v, vertices(G)) N[v] = v;

    if (cluster_heur)
    {
        // Sort vertices, highest degree to lowest
        std::vector<Vertex> degree_array;
        boost::unordered_set<Vertex> cluster_adjacent;
        degree_array.reserve(n);
        foreach(Vertex v, vertices(G))
            degree_array.push_back(v);

        std::sort(degree_array.begin(), degree_array.end(), compare_degree<Graph>(G));
        
        unsigned int i = 0;
        foreach(Vertex v, degree_array)
            if(cluster_adjacent.find(v) != cluster_adjacent.end())
                N[v] = null_vertex;
            else
            {
                N[v] = v; // Make a cluster center
                // Mark all connected nodes
                cluster_adjacent.insert(v);
                foreach(Edge e, out_edges(v, G))
                    cluster_adjacent.insert(target(e, G));
            }
    }
    else
    {   // Make a vertex a cluster center with a probability p_center
        const double p_center = 1.0 / sqrt((double)n);
        unsigned int samples = 0;
        foreach(Vertex v, vertices(G))
            if (rand.uniform01() <= p_center)
            {
                N[v] = v;
                ++samples;
            }
            else
            {
                N[v] = null_vertex;
            }

        std::cout << "Sampled cluster centers:" << samples << std::endl;
    }

    detail::baswana_randomized_3_spanner_impl(G, weight, spanner_edge, N, rand);

    std::vector<Edge> A(n);

    // Discard all edges in E' that aren't connected to a center
    // and are in the same cluster.
    typedef typename detail::intercluster_subgraph< Graph, ClusterMap > ClusterFilter;
    filtered_graph<Graph, ClusterFilter> G_prime(G, ClusterFilter(G, N));

    detail::baswana_link_clusters_impl(G_prime, weight, N, spanner_edge, A);

}

}

#endif

