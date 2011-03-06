/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: James Marble */

#include <gtest/gtest.h>

#define BOOST_NO_HASH
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/make_connected.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>

#include <iterator>
#include <list>

#include "baswana_randomized_3_spanner.h"
#include "baswana_randomized_spanner.h"
#include "set_member_predicate_property_map.hpp"

using namespace ompl;

class TestGraphs : public ::testing::Test
{
public:

    typedef boost::adjacency_list <
        boost::setS, // Use a set for edges
        boost::vecS, // Use a vector for the vertices
        boost::undirectedS,
        boost::no_property,
        boost::property <boost::edge_index_t, unsigned int,
            boost::property <boost::edge_weight_t, float> > > Graph;

    typedef boost::graph_traits<Graph>::edge_descriptor Edge;
        
    struct GraphData
    {
        static boost::rand48 rand;

        const std::string name;
        Graph graph;
        std::vector<Graph> spanners;

        GraphData(const std::string& name, const Graph& in_graph,
            const bool complete = false, const unsigned int random = 0) :
            name(name), graph(in_graph)
        {
            if (complete)
                make_complete(graph);

            {
                boost::rand48 rand_gen;
                boost::generate_random_graph(graph, 0, random, rand_gen);
            }

            boost::make_connected(graph);

            // Set edge indices
            boost::property_map<Graph, boost::edge_index_t>::type
                indices = get(boost::edge_index, graph);
            unsigned int i = 0;
            foreach(Edge e, boost::edges(graph))
                indices[e] = i++;

            // Randomize weights
            boost::property_map<Graph, boost::edge_weight_t>::type
                weight = get(boost::edge_weight, graph);
            boost::uniform_real<double> weight_distribution(0.001, 1.0);
            boost::variate_generator<boost::rand48, boost::uniform_real<double> >
                random_weights(rand, weight_distribution);
            boost::randomize_property<boost::edge_weight_t>(graph, random_weights);

            // Find spanner edges
            for(int cluster_heur = 0; cluster_heur <= 1; ++cluster_heur)
            {
                Graph spanner(boost::num_vertices(graph));
                std::cout << "\nRunning 3-spanner algorithm ";
                std::cout << "(with" << (cluster_heur ? "" : "out") << " cluster heuristic) ";
                std::cout << "on: " << name << std::endl;
                
                std::vector<Edge> edge_data;
                baswana_randomized_3_spanner(graph, std::back_inserter(edge_data), cluster_heur);
                foreach(Edge e, edge_data)
                    boost::add_edge(boost::source(e, graph), boost::target(e, graph),
                        Graph::edge_property_type(indices[e], weight[e]),
                        spanner);
                spanners.push_back(spanner);
            }

            for (unsigned int k = 2; k <= 5; ++k)
            {
                std::cout << "\nRunning 2(" << k << ")-1 spanner algorithm ";
                std::cout << " on: " << name << std::endl;

                Graph spanner(boost::num_vertices(graph));
                BaswanaSpanner<Graph> S(graph, k, true);
                foreach(Edge e, S.calculateSpanner())
                    boost::add_edge(boost::source(e, graph), boost::target(e, graph),
                        Graph::edge_property_type(indices[e], weight[e]),
                        spanner);
                spanners.push_back(spanner);

            }

        }
    };
    static std::list<GraphData> graphs;

    static void make_complete(Graph& G)
    {
        Graph::vertex_iterator v1, v2, end;
        for(boost::tie(v1, end) = boost::vertices(G); v1 != end; ++v1)
            for(v2 = v1; v2 != end; ++v2)
            {
                boost::add_edge(*v1, *v2, G);
            }
    }

    static void SetUpTestCase()
    {
        // Empty graph
        graphs.push_back(GraphData("empty", Graph(0)));
        
        // Small, complete graph
        graphs.push_back(GraphData("complete10", Graph(10), true));
        
        // Large, complete graph
        graphs.push_back(GraphData("complete1h", Graph(100), true));
        
        // Large, very sparse graph
        graphs.push_back(GraphData("sparse1h", Graph(1000)));
        
        // Small, random graph            
        graphs.push_back(GraphData("random10", Graph(10), false, 50));
        
        // Large, random graph
        graphs.push_back(GraphData("random1h", Graph(1000), false, 5000));
    }
};
std::list<TestGraphs::GraphData> TestGraphs::graphs;
boost::rand48 TestGraphs::GraphData::rand;

TEST_F(TestGraphs, graph_edges_have_nonzero_weight) {
    foreach(const GraphData G, graphs) { SCOPED_TRACE(G.name);
        foreach(const Edge e, boost::edges(G.graph)) {
            
            boost::property_map<Graph, boost::edge_weight_t>::const_type
                weight = boost::get(boost::edge_weight, G.graph);

            ASSERT_GT(weight[e], 0);
        }
    }
}

TEST_F(TestGraphs, spanner_edges_have_nonzero_weight) {
    foreach(const GraphData G, graphs) { SCOPED_TRACE(G.name);
        unsigned int k = 0;
        foreach(const Graph spanner, G.spanners) { SCOPED_TRACE(k++);
            foreach(const Edge e, boost::edges(spanner)) {

                boost::property_map<Graph, boost::edge_weight_t>::const_type
                    weight = boost::get(boost::edge_weight, spanner);

                ASSERT_GT(weight[e], 0);
            }
        }
    }
}

TEST_F(TestGraphs, spanner_has_fewer_or_equal_edges) {
    foreach(const GraphData G, graphs) { SCOPED_TRACE(G.name);
        unsigned int k = 0;
        foreach(const Graph spanner, G.spanners) { SCOPED_TRACE(k++);
            EXPECT_LE(boost::num_edges(spanner), boost::num_edges(G.graph));
        }
    }
}

TEST_F(TestGraphs, spanner_has_one_component) {
    foreach(const GraphData G, graphs) { SCOPED_TRACE(G.name);
        unsigned int k = 0;
        foreach(const Graph spanner, G.spanners) { SCOPED_TRACE(k++);

        std::vector<int> component(num_vertices(spanner));
        const int num_components =
            boost::connected_components(spanner, &component[0]);

        EXPECT_LE(num_components, 1);
        }
    }
}

// Use Dijkstra's shortests paths from a random point in both the original
// graph and the spanner. Then ensure that no path is lengthened further than
// expected.
TEST_F(TestGraphs, spanner_property) {
    foreach(const GraphData G, graphs) { SCOPED_TRACE(G.name);
        if (boost::num_vertices(G.graph) > 0) {

            const unsigned int n = boost::num_vertices(G.graph);
            const Graph::vertex_descriptor start =
                boost::random_vertex(G.graph, GraphData::rand);
            std::vector<Graph::vertex_descriptor> predecessor(n);

            // Run Dijkstra's on the original graph.
            std::vector<float> d_g(n);
            dijkstra_shortest_paths(G.graph, start,
                boost::predecessor_map(&predecessor[0]).distance_map(&d_g[0]));

            unsigned int k = 0;
            foreach(const Graph spanner, G.spanners) {
                const float stretch = k<2 ? 3.0 : 2*k-1;
                SCOPED_TRACE(testing::Message() <<
                        "k: " << k << " stretch:" << stretch);

                // Run Dijkstra's on the spanner.
                std::vector<float> d_s(n);
                dijkstra_shortest_paths(spanner, start,
                    boost::predecessor_map(&predecessor[0]).distance_map(&d_s[0]));

                foreach(Graph::vertex_descriptor v, boost::vertices(G.graph))
                {
                    const float spanner_distance = d_s[v];
                    const float original_distance = d_g[v];
                    ASSERT_LE(spanner_distance, original_distance * stretch);
                    ASSERT_GE(spanner_distance, original_distance);
                }
                ++k;
            }
        }
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
