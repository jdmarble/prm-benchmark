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

#include <list>

#include "baswana_randomized_spanner.h"

using namespace ompl;

class TestGraphs : public ::testing::Test
{
public:

    typedef boost::adjacency_matrix <
        boost::undirectedS,
        boost::no_property,
        boost::property <boost::edge_weight_t, float> > Graph;

    typedef boost::graph_traits<Graph>::edge_descriptor Edge;

    struct GraphData
    {
        static boost::rand48 rand;

        const std::string name;
        Graph graph;
        Graph spanner;

        GraphData(const std::string& name, const Graph& in_graph) :
            name(name), graph(in_graph), spanner(boost::num_vertices(graph))
        {
            // Randomize weights
            boost::uniform_real<double> weight_distribution(0.001, 1.0);
            boost::variate_generator<boost::rand48, boost::uniform_real<double> >
                random_weights(rand, weight_distribution);
            boost::randomize_property<boost::edge_weight_t>(graph, random_weights);

            // Find spanner edges
            std::cout << "\nRunning spanner algorithm on: " << name << std::endl;
            std::map<Edge, bool> spanner_map;
            typedef boost::associative_property_map<std::map<Edge, bool> > spanner_t;
            spanner_t spanner_edge(spanner_map);
            baswana_randomized_3_spanner(graph, spanner_edge);

            // Copy the edges to the (empty) spanner graph.
            foreach(Edge e, boost::edges(graph))
                boost::add_edge(boost::source(e, graph),
                                boost::target(e, graph), spanner);
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
        {// Empty graph
            Graph empty(0);
            graphs.push_back(GraphData("empty", empty));
        }

        {// Small, complete graph
            Graph complete10(10);
            make_complete(complete10);
            graphs.push_back(GraphData("complete10", complete10));
        }

        {// Large, complete graph
            Graph complete1k(1000);
            make_complete(complete1k);
            graphs.push_back(GraphData("complete1k", complete1k));
        }

        {// Large, very sparse graph
            Graph sparse1k(1000);
            boost::make_connected(sparse1k);
            graphs.push_back(GraphData("sparse1k", sparse1k));
        }

        boost::rand48 rand_gen;

        {// Small, random graph
            Graph random10(10);
            boost::generate_random_graph(random10, 0, 50, rand_gen);
            boost::make_connected(random10);
            graphs.push_back(GraphData("random10", random10));
        }

        {// Large, random graph
            Graph random1k(1000);
            boost::generate_random_graph(random1k, 0, 50000, rand_gen);
            boost::make_connected(random1k);
            graphs.push_back(GraphData("random1k", random1k));
        }

    }
};
std::list<TestGraphs::GraphData> TestGraphs::graphs;
boost::rand48 TestGraphs::GraphData::rand;

TEST_F(TestGraphs, spanner_has_fewer_or_equal_edges)
{
    foreach(const GraphData G, graphs)
    {
        EXPECT_LE(boost::num_edges(G.spanner), boost::num_edges(G.graph))
                << "Failed for graph: " << G.name;
    }
}

TEST_F(TestGraphs, spanner_has_one_component)
{
    foreach(const GraphData G, graphs)
    {        
        std::vector<int> component(num_vertices(G.spanner));
        const int num_components =
            boost::connected_components(G.spanner, &component[0]);

        EXPECT_LE(num_components, 1) << "Failed for graph: " << G.name;
    }
}

// Use Dijkstra's shortests paths from a random point in both the original
// graph and the spanner. Then ensure that no path is lengthened further than
// expected.
TEST_F(TestGraphs, spanner_property)
{
    const float stretch = 3.0;
    
    foreach(const GraphData G, graphs)
    {
        if (boost::num_vertices(G.graph) > 0)
        {
            const unsigned int n = boost::num_vertices(G.graph);
            const Graph::vertex_descriptor start =
                boost::random_vertex(G.graph, GraphData::rand);

            // Run Dijkstra's on the original graph.
            std::vector<Graph::vertex_descriptor> p_g(n);
            std::vector<float> d_g(n);
            dijkstra_shortest_paths(G.graph, start,
                boost::predecessor_map(&p_g[0]).distance_map(&d_g[0]));

            // Run Dijkstra's on the spanner.
            std::vector<Graph::vertex_descriptor> p_s(n);
            std::vector<float> d_s(n);
            dijkstra_shortest_paths(G.spanner, start,
                boost::predecessor_map(&p_s[0]).distance_map(&d_s[0]));

            foreach(Graph::vertex_descriptor v, boost::vertices(G.graph))
            {
                const float spanner_distance = d_s[v];
                const float original_distance = d_g[v];
                EXPECT_LE(spanner_distance, original_distance * stretch)
                        << "Failed for graph: " << G.name;
            }
        }
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
