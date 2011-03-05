/* 
 * File:   baswana_randomized_spanner.h
 * Author: jdmarble
 *
 * Created on March 3, 2011, 12:01 PM
 */

#ifndef BASWANA_RANDOMIZED_SPANNER_H
#define	BASWANA_RANDOMIZED_SPANNER_H

#define BOOST_NO_HASH
#include <boost/config.hpp>

#include <boost/bind.hpp>

#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/filtered_graph.hpp>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/variate_generator.hpp>

#include <boost/unordered_set.hpp>

using namespace boost;
using namespace std;


template<class Graph>
class BaswanaSpanner
{
    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
    typedef typename graph_traits<Graph>::edge_descriptor Edge;

    const Graph& G;
    const unsigned int k;
    typedef typename graph_traits<Graph>::vertices_size_type VertexSize;
    VertexSize n;
    typedef typename graph_traits<Graph>::edges_size_type EdgeSize;
    EdgeSize m;

    const Vertex null_vertex;
    const typename property_map<Graph, edge_weight_t>::const_type weight;
    
    variate_generator<rand48, uniform_01<double> > uniform01;
    typedef unordered_map<Vertex, Vertex> ClusterMap;
    typedef typename ClusterMap::value_type Cluster;
    ClusterMap clusters;
    ClusterMap last_clusters;
    unordered_set<Vertex> cluster_centers;
    unordered_set<Vertex> last_cluster_centers;
    unordered_set<Vertex> unsampled_vertices;
    unordered_map<Vertex, Edge> lightest_cluster_edge;
    list<Edge> spanner_edges;

    struct EdgeHash : std::unary_function<Edge, size_t>
    {
        const Graph& G;
        EdgeHash(const Graph& G) : G(G) {}

        size_t operator()(const Edge& e) const
        {
            size_t seed = 0;
            hash_combine(seed, min(source(e, G), target(e, G)));
            hash_combine(seed, max(source(e, G), target(e, G)));
            return seed;
        }
    };
    
    unordered_set<Edge, EdgeHash> E_;

public:

    BaswanaSpanner(const Graph& G, const unsigned int k) : G(G), k(k),
    n(num_vertices(G)), m(num_edges(G)),
    null_vertex(graph_traits<Graph>::null_vertex()),
    weight(get(edge_weight, G)),
    E_(m, EdgeHash(G)),
    uniform01(rand48(), uniform_01<double>())
    {
        // Add all edges in E to E'
        foreach(Edge e, edges(G))
            E_.insert(e);

        // Initialize subgraphs to singletons
        foreach(Vertex v, vertices(G))
        {
            clusters[v] = v;
            cluster_centers.insert(v);
        }
    }

    const list<Edge>& calculateSpanner()
    {
        spanner_edges.clear();

        for (unsigned int i = 1; i < k; ++i) // Perform k-1 iterations.
        {
            sampleClusters();
            connectToSampled();
            addEdgesToSpanner();
        }

        return spanner_edges;
    }

private:

    void sampleClusters()
    {
        last_cluster_centers = cluster_centers;
        cluster_centers.clear();

        last_clusters = clusters;
        clusters.clear();

        unsampled_vertices.clear();

        foreach (Vertex c, last_cluster_centers)
            if (uniform01() > pow((double)n, -1.0/(double)k))
                cluster_centers.insert(c);

        foreach (Cluster vc, last_clusters)
            if (cluster_centers.find(vc.second) != cluster_centers.end())
                clusters[vc.first] = vc.second;
            else
                unsampled_vertices.insert(vc.first);
    }

    void connectToSampled()
    {
        lightest_cluster_edge.clear();
        foreach(Vertex v, unsampled_vertices)
        {
            foreach(Edge e, out_edges(v, G))
            {
                const Vertex u = target(e, G);
                // If this edge connects to a vertex in a sampled cluster.
                if (unsampled_vertices.find(u) != unsampled_vertices.end() &&
                        (lightest_cluster_edge.find(v) == lightest_cluster_edge.end()
                        || weight[e] < weight[lightest_cluster_edge[v]])
                   )
                {
                    lightest_cluster_edge[v] = e;
                }
            }
        }
    }

    void moveAllE_vc(const Vertex& v)
    {
        unordered_map<Vertex, Edge> lightest;
        foreach(Edge e, out_edges(v, G))
        {
            const Vertex c = last_clusters[target(e, G)];
            if (E_.find(e) != E_.end() &&
                (lightest.find(c) == lightest.end() ||
                    weight[e] < weight[lightest[c]]))
            {
                lightest[c] = e;
            }
            E_.erase(e);
        }
        typedef pair<Vertex, Edge> VertexEdge;
        foreach(VertexEdge ve, lightest)
            spanner_edges.push_front(ve.second);
    }

    void addEdgesToSpanner()
    {
        foreach(Vertex v, unsampled_vertices)
        {
            if (lightest_cluster_edge.find(v) == lightest_cluster_edge.end())
            {
                moveAllE_vc(v);
            }
            else
            {

            }
        }
    }
};

#endif

