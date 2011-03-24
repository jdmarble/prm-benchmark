#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/RealVectorStateManifold.h>
#include <ompl/geometric/planners/prm/BGL_PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>

#include <boost/program_options.hpp>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace b = boost;
namespace ba = boost::accumulators;
namespace po = boost::program_options;

#include "baswana_randomized_spanner.h"
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/multi_array.hpp>
#include <boost/graph/adjacency_matrix.hpp>

typedef ob::RealVectorStateManifold::StateType R2State;
typedef og::BGL_PRM::Graph Graph;
typedef og::BGL_PRM::Vertex Vertex;
typedef og::BGL_PRM::Edge Edge;
typedef og::BGL_PRM::vertex_state_t vertex_state_t;


bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect    
    const R2State* r2state = state->as<R2State>();
    const double x = (*r2state)[0];
    const double y = (*r2state)[1];
    
    // check validity of state
    const bool inSlat = (int)(y*10)%2;
    return !(x > -0.25 && x < 0.25 && inSlat);
}

double meanPathLength(const Graph& g)
{
    const unsigned int n = b::num_vertices(g);
    b::multi_array<double, 2> d(b::extents[n][n]);
    bool success = b::johnson_all_pairs_shortest_paths(g, d);

    ba::accumulator_set<double, ba::stats<ba::tag::mean> > acc;
    for(unsigned int x = 0; x < n; ++x)
        for(unsigned int y = 0; y < n; ++y)
            acc(d[x][y]);

    return ba::mean(acc);
}

void plan(const unsigned int target_n)
{
    // construct the manifold we are planning in
    ob::StateManifoldPtr manifold(new ob::RealVectorStateManifold(2));

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    manifold->as<ob::RealVectorStateManifold>()->setBounds(bounds);

    // construct an instance of  space information from this manifold
    ob::SpaceInformationPtr si(new ob::SpaceInformation(manifold));

    // setting collision checking resolution to 1% of the space extent
    si->setStateValidityCheckingResolution(0.0001);

    si->setup();

    // set state validity checking for this space
    si->setStateValidityChecker(boost::bind(&isStateValid, _1));

    // create a planner for the defined space
    ob::PlannerPtr planner(new og::BGL_PRM(si));

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cerr);

    const Graph& g = planner->as<og::BGL_PRM>()->getGraph();

    // Repeat for at least target_n milestones have been added.
    while (b::num_vertices(g) < target_n)
    {
        planner->as<og::BGL_PRM>()->growRoadmap(0.01);
        std::cerr << b::num_vertices(g) << ',' << b::num_edges(g) << std::endl;
    }

    BaswanaSpanner<Graph> spannerCalc(g, 9, false);
    const std::list<Edge> s = spannerCalc.calculateSpanner();

    std::cout << "index,spanner,x0,y0,x1,y1" << std::endl;
    b::property_map<Graph, vertex_state_t>::const_type
        state = boost::get(vertex_state_t(), g);
    b::property_map<Graph, b::edge_index_t>::const_type
        index = boost::get(b::edge_index, g);
    foreach(Edge e, b::edges(g))
    {
        std::cout << b::lexical_cast<std::string>(index[e]) << ',';

        char subgraph = '0';
        if (std::find(s.begin(), s.end(), e) != s.end())
            subgraph = '1';
        std::cout << subgraph << ',';

        R2State* s1 = state[b::source(e, g)]->as<R2State>();
        std::cout << b::lexical_cast<std::string>((*s1)[0]) << ',';
        std::cout << b::lexical_cast<std::string>((*s1)[1]) << ',';

        R2State* s2 = state[b::target(e, g)]->as<R2State>();
        std::cout << b::lexical_cast<std::string>((*s2)[0]) << ',';
        std::cout << b::lexical_cast<std::string>((*s2)[1]) << std::endl;
    }

    std::cerr << "Graph mean path length:" << meanPathLength(g) << std::endl;
    std::cerr << "Graph edges:" << b::num_edges(g) << std::endl;

    b::property_map<Graph, b::edge_weight_t>::const_type
        weight = boost::get(b::edge_weight, g);
    Graph g_s(b::num_vertices(g));
    foreach(Edge e, s)
    {
        Vertex v1 = b::source(e, g);
        Vertex v2 = b::target(e, g);
        const Graph::edge_property_type properties(weight[e], index[e]);
        
        b::add_edge(v1, v2, properties, g_s);
    }

    std::cerr << "Spanner mean path length:" << meanPathLength(g_s) << std::endl;
    std::cerr << "Spanner edges:" << b::num_edges(g_s) << std::endl;
}

int main(int argc, char* argv[])
{
    // Turn off logging to std::out. Need the stream for file output.
    ompl::msg::noOutputHandler();

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("n", po::value<unsigned int>(), "number of nodes in graph")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cerr << desc << "\n";
        return 1;
    }

    plan(vm["n"].as<unsigned int>());

    return 0;
}