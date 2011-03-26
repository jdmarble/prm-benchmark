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

using namespace boost;
using namespace std;


template<typename Container>
bool in(const Container& c, const typename Container::key_type& v)
{
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
    unordered_set<Vertex> unsampled_vertices;
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

    struct ClusterDegree
    {
        unordered_map<Vertex, unsigned int> degree;

        ClusterDegree(const Graph& G,
            const ClusterMap& clusters, const EdgeSet& E_)
        {
            // Count all (vertex) neighbors of all clusters.
            typedef pair<Vertex, Vertex> ChildParent;
            foreach(ChildParent cp, clusters)
            {
                const Vertex v = cp.first;
                const Vertex x = cp.second;
                foreach(Edge e, out_edges(v, G))
                {
                    const Vertex u = target(e, G);
                    typename ClusterMap::const_iterator cluster = clusters.find(u);
                    // Neighbor is not in a cluster or in a different cluster
                    if (in(E_, e) && 
                            (cluster == clusters.end() ||
                            (*cluster).second != x))
                        ++degree[x];
                }
            }


        }

        bool operator()(const Vertex& c1, const Vertex& c2)
        {
            // Will default to 0 if no neighbors.
            return degree[c1] < degree[c2];
        }
    };

    void sampleClusters()
    {
        last_cluster_centers = cluster_centers;
        cluster_centers.clear();

        last_clusters = clusters;
        clusters.clear();

        unsampled_vertices.clear();

        if (cluster_heur)
        {
            // Sort by degree
            vector<Vertex> sorted_clusters(
                    last_cluster_centers.begin(), last_cluster_centers.end());
            cout << "Sorted clusters:" << sorted_clusters.size() << endl;
            sort(sorted_clusters.begin(), sorted_clusters.end(),
                ClusterDegree(G, last_clusters, E_));

            // Find all neighbors of all clusters.
            unordered_map<Vertex, unordered_set<Vertex> > clusterNeighbors;
            typedef pair<Vertex, Vertex> ChildParent;
            foreach(ChildParent cp, last_clusters)
            {
                const Vertex v = cp.first;
                const Vertex x = cp.second;
                foreach(Edge e, out_edges(v, G))
                {
                    const Vertex u = target(e, G);                    
                    // Neighbor is not in a cluster or in a different cluster
                    if (in(E_, e) && 
                            in(last_clusters, u) && last_clusters[u] != x)
                        clusterNeighbors[x].insert(last_clusters[u]);
                }
            }

            // Start with all clusters available
            unordered_set<Vertex> unavailable_clusters;

            // Add clusters with no added neighbors
            foreach(Vertex c, sorted_clusters)
            {
                if (!in(unavailable_clusters, c))
                {
                    cluster_centers.insert(c);
                    
                    foreach(Vertex c_, clusterNeighbors[c])
                        unavailable_clusters.insert(c_);
                }
            }
        }
        else
        {
            foreach (Vertex c, last_cluster_centers)
                if (uniform01() < pow((double)n, -1.0/(double)k))
                    cluster_centers.insert(c);         
        }

        // Copy the sampled clusters from the last clusters
        foreach (Cluster vc, last_clusters)
            if (in(cluster_centers, vc.second))
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
                if (in(E_, e) && in(clusters, u) &&
                        (!in(lightest_cluster_edge, v) ||
                         weight[e] < weight[lightest_cluster_edge[v]])
                   )
                {
                    lightest_cluster_edge[v] = e;
                }
            }
        }
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
                connectCluster(v);
            }
        }
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

