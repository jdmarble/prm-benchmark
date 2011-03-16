#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <ompl/geometric/planners/prm/BGL_PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <omplapp/SE3RigidBodyPlanning.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
namespace bacc = boost::accumulators;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/random.hpp>

#include <iostream>
#include <limits>
#include <map>
#include <numeric>

//#include "baswana_randomized_3_spanner.h"
#include "baswana_randomized_spanner.h"

using namespace ompl;

// For counting the size of a nested vector.
template <typename T>
unsigned int accumSizes(unsigned int acc, const std::vector<T>& vec)
{
    return vec.size() + acc;
}

struct Experiment {

    app::SE3RigidBodyPlanning setup;
    boost::shared_ptr<geometric::BGL_PRM> planner;
    base::PlannerData plannerData;

    typedef geometric::BGL_PRM::Graph Graph;
    typedef geometric::BGL_PRM::Vertex Vertex;
    typedef geometric::BGL_PRM::Edge Edge;

    /** Current graph in the planner */
    const Graph& G;

    /** Copy of graph as it was before spanner */
    Graph originalGraph;

    /** For picking random vertices */
    boost::rand48 randGen;

    Experiment(const std::string& environment) :
        planner(new geometric::BGL_PRM(setup.getSpaceInformation())),
        G(planner->getGraph())
    {
        setup.setPlanner(planner);
        
        // load the robot and the environment
        std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR)
                + "/" + environment + "_robot.dae";
        std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR)
                + "/" + environment + "_env.dae";
        setup.setRobotMesh(robot_fname.c_str());
        setup.setEnvironmentMesh(env_fname.c_str());

        // setting collision checking resolution to 1% of the space extent
        setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
        setup.setup();
    }

    void explore_space(const unsigned int target_n)
    {
        // Repeat for at least target_n milestones have been added.
        while (plannerData.states.size() < target_n)
        {
            planner->as<geometric::BGL_PRM>()->growRoadmap(1.0);
            planner->getPlannerData(plannerData);

            unsigned int verts = plannerData.states.size();
            unsigned int edges =
                std::accumulate(plannerData.edges.begin(), plannerData.edges.end(),
                    0, accumSizes<unsigned int>) / 2;

            std::cerr << verts << ',' << edges << std::endl;
        }

        originalGraph = planner->getGraph();
    }

    void make_spanner(const unsigned int k, const bool heuristic)
    {
        BaswanaSpanner<Graph> S(G, k, heuristic);
        const std::list<Edge> spannerEdges = S.calculateSpanner();
        planner->edgeSetIntersect(spannerEdges);
    }

    void print_stats()
    {
        const Graph::vertices_size_type n = boost::num_vertices(G);
        std::vector<double> d_graph(n);
        std::vector<double> d_spanner(n);

        std::cout << "Graph vertices: " << n << std::endl;
        std::cout << "Graph edges: " << boost::num_edges(originalGraph) << std::endl;
        std::cout << "Graph stats..." << std::endl;
        pathStats(originalGraph, d_graph);

        std::cout << "Spanner stats..." << std::endl;
        std::cout << "Spanner edges: " << boost::num_edges(G) << std::endl;
        pathStats(G, d_spanner);

        for (int i = 0; i < 10; ++i)
        {
            std::cout << "Difference stats..." << std::endl;
            foreach(Vertex v, boost::vertices(G))
            {
                std::cout << d_graph[v] << ',' << d_spanner[v]/d_graph[v] << std::endl;
            }
        }
    }

    void pathStats(const Graph& G, std::vector<double>& d)
    {
        Graph::vertices_size_type n = boost::num_vertices(G);

        Vertex start = boost::random_vertex(G, randGen);

        // Need scratch vector for Dijkstra's
        std::vector<Vertex> predecessor(n);
         
        dijkstra_shortest_paths(G, start,
            boost::predecessor_map(&predecessor[0]).distance_map(&d[0]));
    }

};

int main(int argc, char* argv[])
{
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("environment", po::value<std::string>(), "environment name")
        ("n", po::value<unsigned int>(), "number of nodes in graph")
        ("k", po::value<unsigned int>(), "create a 2k-1 spanner")
        ("heuristic", po::value<bool>(), "use cluster heuristic")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    Experiment exp(vm["environment"].as<std::string>());
    exp.explore_space(vm["n"].as<unsigned int>());
    exp.make_spanner(vm["k"].as<unsigned int>(), vm["heuristic"].as<bool>());
    exp.print_stats();
   
}