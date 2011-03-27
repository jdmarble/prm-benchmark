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

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include <boost/bind.hpp>

using namespace boost;

#include <algorithm>

using namespace std;


template<typename Container>
bool in(const Container& c, const typename Container::key_type& v)
{
    if (c.empty()) return false;
    return c.find(v) != c.end();
}

template<class Graph>
class BaswanaSpanner
{
    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
    typedef typename graph_traits<Graph>::edge_descriptor Edge;

    const Graph& G;
    const unsigned int k;
    const bool cluster_heur;
    typedef typename graph_traits<Graph>::vertices_size_type VertexSize;
    VertexSize n;
    typedef typename graph_traits<Graph>::edges_size_type EdgeSize;
    EdgeSize m;

    const Vertex null_vertex;
    typedef typename property_map<Graph, edge_index_t>::const_type EdgeIndexMap;
    const EdgeIndexMap index;
    const typename property_map<Graph, edge_weight_t>::const_type weight;
    
    variate_generator<rand48, uniform_01<double> > uniform01;
    typedef unordered_map<Vertex, Vertex> ClusterMap;
    typedef typename ClusterMap::value_type Cluster;
    ClusterMap clusters;
    ClusterMap last_clusters;
    unordered_set<Vertex> cluster_centers;
    unordered_set<Vertex> last_cluster_centers;
    unordered_set<Vertex> sampled_vertices;
    unordered_map<Vertex, Edge> lightest_cluster_edge;
    list<Edge> spanner_edges;

    // Stats
    unsigned int step3a, step3b;

    struct EdgeEq : std::binary_function<Edge, Edge, size_t>
    {
        const EdgeIndexMap& indexMap;
        EdgeEq(const EdgeIndexMap& indexMap) : indexMap(indexMap) {}

        size_t operator()(const Edge& e1, const Edge& e2) const
        {
            return indexMap[e1] == indexMap[e2];
        }
    };

    struct EdgeHash : std::unary_function<Edge, size_t>
    {
        const EdgeIndexMap& indexMap;
        EdgeHash(const EdgeIndexMap& indexMap) : indexMap(indexMap) {}

        size_t operator()(const Edge& e) const
        {
            size_t seed = 0;
            hash_combine(seed, indexMap[e]);
            return seed;
        }
    };
    
    typedef unordered_set<Edge, EdgeHash, EdgeEq> EdgeSet;
    EdgeSet E_;

public:

    BaswanaSpanner(const Graph& G, const int seed,
	const unsigned int k, const bool cluster_heur = false)
    : G(G), k(k), cluster_heur(cluster_heur),
    n(num_vertices(G)), m(num_edges(G)),
    null_vertex(graph_traits<Graph>::null_vertex()),
    index(get(edge_index, G)), weight(get(edge_weight, G)),
    E_(m, EdgeHash(index), EdgeEq(index)),
    uniform01(rand48(seed), uniform_01<double>())
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

        for (unsigned int i = 0; i < k/2; ++i) // Perform floor(k/2) iterations.
        {
            step3a = step3b = 0;

            sampleClusters();
            connectToSampled();
            addEdgesToSpanner();
            removeInterclusterEdges();
        }

        clusterClusterJoining();

        return spanner_edges;
    }

private:

    void calc_degrees(unordered_map<Vertex, unsigned int>& degree)
    {
        unordered_set<Vertex> covered;
        foreach(Edge e, E_)
        {
            const Vertex& v = source(e, G);
            const Vertex& u = target(e, G);

            if (in(sampled_vertices, v) || in(sampled_vertices, u))
            {
                covered.insert(u);
                covered.insert(v);
            }
        }

        typedef unordered_map<Vertex, unordered_set<Vertex> > NMap;
        typedef typename NMap::value_type N;
        NMap neighborMap;

        foreach(Edge e, E_)
        {
            const Vertex& v = source(e, G);
            const Vertex& u = target(e, G);

            if (in(last_clusters, v) &&
                !in(sampled_vertices, v) && !in(covered, u) )
                neighborMap[last_clusters[v]].insert(u);

            if (in(last_clusters, u) &&
                !in(sampled_vertices, u) && !in(covered, v) )
                neighborMap[last_clusters[u]].insert(v);
        }

        degree.clear();
        foreach(N& neighbors, neighborMap)
            degree[neighbors.first] = neighbors.second.size();
    }

    void sampleClusters()
    {
        const double sample_prob = pow((double)n, -1.0/(double)k);
        const unsigned int expected_clusters =
            cluster_centers.size() * sample_prob;

        last_cluster_centers = cluster_centers;
        cluster_centers.clear();

        last_clusters = clusters;
        clusters.clear();

        sampled_vertices.clear();

        if (cluster_heur)
        {
            typedef unordered_map<Vertex, unsigned int> Map;
            typedef typename Map::value_type MapPair;
            Map degree;
                    
            while(cluster_centers.size() < expected_clusters)
            {
                calc_degrees(degree);
                if (degree.empty()) break;

                // Max by second (degree)                
                MapPair c = *max_element(degree.begin(), degree.end(),
                    bind(&MapPair::second, _1) < bind(&MapPair::second, _2));

                make_sampled_cluster(c.first);
            }
        }
        else
        {
            foreach (Vertex c, last_cluster_centers)
                if (uniform01() < sample_prob)
                    make_sampled_cluster(c);
        }  
    }

    void make_sampled_cluster(const Vertex& c)
    {   
        cluster_centers.insert(c);

        typedef pair<Vertex, Vertex> ChildParent;
        foreach(ChildParent cp, last_clusters)
        {
            const Vertex v = cp.first;
            const Vertex x = cp.second; // cluster center
            if (c == x)
            {
                clusters[v] = c;
                sampled_vertices.insert(v);
            }
        }
    }

    void connectToSampled()
    {
        lightest_cluster_edge.clear();
        foreach(Vertex v, vertices(G))
            if (!in(sampled_vertices, v))
                foreach(Edge e, out_edges(v, G))
                {
                    const Vertex u = target(e, G);
                    // If this edge connects to a vertex in a sampled cluster.
                    if (in(E_, e) && in(clusters, u) &&
                            (!in(lightest_cluster_edge, v) ||
                             weight[e] < weight[lightest_cluster_edge[v]])
                       )
                       lightest_cluster_edge[v] = e;
                }
    }

    void addEdgesToSpanner()
    {
        foreach(Vertex v, vertices(G))
            if (!in(sampled_vertices, v))
                if (!in(lightest_cluster_edge,v))
                    moveAllE_vc(v);
                else
                    connectCluster(v);
    }

    void moveAllE_vc(const Vertex& v)
    {
        unordered_map<Vertex, Edge> lightest;
        foreach(Edge e, out_edges(v, G))
        {
            const Vertex c = last_clusters[target(e, G)];
            if (in(E_, e) &&
                (!in(lightest, c) ||
                    weight[e] < weight[lightest[c]]))
            {
                lightest[c] = e;
            }
            E_.erase(e);
        }
        typedef pair<Vertex, Edge> VertexEdge;
        foreach(VertexEdge ve, lightest)
        {
            ++step3a;
            spanner_edges.push_front(ve.second);
        }
    }

    void connectCluster(const Vertex& v)
    {
        // Step 3. b.

        // "Let c in R_i be the cluster that is adjacent to v with edge e_v
        // that is the least weight amoung all the clusters incident on v from
        // R_i."
        BOOST_ASSERT(in(lightest_cluster_edge, v));
        const Edge e_v = lightest_cluster_edge[v];
        
        const Vertex w = target(e_v, G);
        BOOST_ASSERT(in(clusters, w));
        const Vertex c = clusters[w];

        // "Add the edge e_v to E_S and E_i."
        spanner_edges.push_front(e_v);
        clusters[v] = c;

        // "Remove from E' all edges in E'(v,c)."
        foreach(Edge e, out_edges(v, G))
        {
            const Vertex u = target(e, G);
            if(in(clusters, u) && clusters[u] == c)
                E_.erase(e);
        }

        // "For each cluster c' in C_{i-1} adjacent to v with an edge weight
        // less than e_v..."
        unordered_map<Vertex, Edge> least_cluster_edge;
        foreach (Edge e, out_edges(v, G))
        {
            const Vertex u = target(e, G);
            if (weight[e] < weight[e_v] && in(E_, e) && in(last_clusters, u))
            {
                const Vertex c_ = last_clusters[u];
                if (c_ != null_vertex &&
                    (!in(least_cluster_edge, c_) ||
                        weight[e] < weight[least_cluster_edge[c_]]))
                {
                    least_cluster_edge[c_] = e;
                }
            }
        }
        // "... add the least edge from E'(v,c') to E_S and remove
        // E'(v,c') from E'."
        typedef pair<Vertex, Edge> VertexEdge;
        foreach (VertexEdge ve, least_cluster_edge)
        {
            spanner_edges.push_front(ve.second);
            ++step3b;

            foreach (Edge e, out_edges(v, G))
            {
                const Vertex v1 = target(e, G);
                const Vertex v2 = ve.first;
                if (in(last_clusters, v1) && in(last_clusters, v2) &&
                        last_clusters[v1] == last_clusters[v2])
                    E_.erase(e);
            }
        }

    }

    bool removeInterclusterEdges()
    {
        list<Edge> toRemove;
        foreach(Edge e, E_)
        {
            const Vertex v1 = source(e, G);
            const Vertex v2 = target(e, G);
            if (in(clusters, v1) && in(clusters, v2) && 
                    clusters[v1] == clusters[v2])
                toRemove.push_front(e);
        }
        foreach(Edge e, toRemove)
            E_.erase(e);
    }

    void clusterClusterJoining()
    {
        if(k % 2) // k is odd
        {
            oddClusterClusterJoining();
        }
        else
        {
            evenClusterClusterJoining();
        }
    }

    void oddClusterClusterJoining()
    {
        typedef pair<Vertex, Vertex> ClusterPair;
        unordered_map<ClusterPair, Edge> lightest;
        foreach(Edge e, E_)
        {
            const Vertex v1 = min(source(e, G), target(e, G));
            const Vertex v2 = max(source(e, G), target(e, G));
            if (in(clusters, v1) && in(last_clusters, v2))
            {
                const Vertex c = clusters[v1];
                const Vertex c_ = clusters[v2];
                const ClusterPair p = make_pair(c, c_);
                if (c != c_ &&
                    (lightest.find(p) == lightest.end() ||
                        weight[e] < weight[lightest[p]]))
                {
                    lightest[p] = e;
                }
            }
        }

        typedef pair<ClusterPair, Edge> ClusterPairEdge;
        foreach(ClusterPairEdge cpe, lightest)
            spanner_edges.push_front(cpe.second);
    }
    
    void evenClusterClusterJoining()
    {
        typedef pair<Vertex, Vertex> ClusterPair;
        typedef pair<ClusterPair, Edge> ClusterPairEdge;
        unordered_map<ClusterPair, Edge> lightest;

        foreach(Edge e, E_)
        {
            const Vertex v1 = source(e, G);
            const Vertex v2 = target(e, G);
            if (in(clusters, v1) && in(last_clusters, v2))
                {
                const Vertex c = clusters[v1];
                const Vertex c_ = last_clusters[v2];
                const ClusterPair p = make_pair(c, c_);
                if (c != c_ &&
                    (lightest.find(p) == lightest.end() ||
                        weight[e] < weight[lightest[p]]))
                {
                    lightest[p] = e;
                }
            }
        }
        foreach(ClusterPairEdge cpe, lightest)
            spanner_edges.push_front(cpe.second);

        lightest.clear();
        foreach(Edge e, E_)
        {
            const Vertex v1 = target(e, G);
            const Vertex v2 = source(e, G);
            if (in(clusters, v1) && in(last_clusters, v2))
                {
                const Vertex c = clusters[v1];
                const Vertex c_ = last_clusters[v2];
                const ClusterPair p = make_pair(c, c_);
                if (c != c_ &&
                    (lightest.find(p) == lightest.end() ||
                        weight[e] < weight[lightest[p]]))
                {
                    lightest[p] = e;
                }
            }
        }
        foreach(ClusterPairEdge cpe, lightest)
            spanner_edges.push_front(cpe.second);
    }
};

#endif

