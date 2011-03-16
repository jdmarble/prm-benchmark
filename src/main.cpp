#include <ompl/config.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <ompl/geometric/planners/prm/BGL_PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/benchmark/Benchmark.h>

#include <omplapp/SE3RigidBodyPlanning.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/vector_property_map.hpp>

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

    typedef geometric::BGL_PRM::Graph Graph;
    typedef geometric::BGL_PRM::Vertex Vertex;
    typedef geometric::BGL_PRM::Edge Edge;

    app::SE3RigidBodyPlanning setup;
    
    boost::shared_ptr<geometric::BGL_PRM> graphPlanner;
    base::PlannerData graphPlannerData;
    /** Original graph */
    const Graph& G;

    Graph::vertices_size_type n;

    boost::shared_ptr<geometric::BGL_PRM> spannerPlanner;
    base::PlannerData spannerPlannerData;
    /** Spanner subgraph */
    const Graph& S;
       
    /** For picking random vertices */
    RNG rng_;

    const std::string environment_;

    const unsigned int k_;

    const bool heuristic_;

    Experiment(const std::string& environment,
               const unsigned int k,
               const bool heuristic) :
        graphPlanner(new geometric::BGL_PRM(setup.getSpaceInformation())),
        G(graphPlanner->getGraph()),
        spannerPlanner(new geometric::BGL_PRM(setup.getSpaceInformation())),
        S(spannerPlanner->getGraph()),
        environment_(environment), k_(k), heuristic_(heuristic)
    {
        setup.setPlanner(graphPlanner);
        
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
        while (graphPlannerData.states.size() < target_n)
        {
            graphPlanner->growRoadmap(1.0);
            graphPlanner->getPlannerData(graphPlannerData);

            unsigned int verts = graphPlannerData.states.size();
            unsigned int edges =
                std::accumulate(graphPlannerData.edges.begin(), graphPlannerData.edges.end(),
                    0, accumSizes<unsigned int>) / 2;

            std::cerr << verts << ',' << edges << std::endl;
        }
        n = boost::num_vertices(G);
    }

    void make_spanner()
    {
        spannerPlanner->setGraph(graphPlanner->getGraph());
        BaswanaSpanner<Graph> spannerCalc(S, k_, heuristic_);
        const std::list<Edge> spannerEdges = spannerCalc.calculateSpanner();
        spannerPlanner->edgeSetIntersect(spannerEdges);
        cerr << "spanner edges:" << boost::num_edges(S) << endl;
    }

    void print_stats()
    {
        for (int i = 0; i < 10; ++i)
        {
            Vertex start = rng_.uniformInt(*(boost::vertices(G).first), n);

            boost::vector_property_map<double> d_graph(n);
            pathStats(G, d_graph, start);

            boost::vector_property_map<double> d_spanner(n);
            pathStats(S, d_spanner, start);

            foreach(Vertex v, boost::vertices(G))
            {
                std::cout << k_ << ',' << heuristic_ << ',' << i << ',';
                std::cout << d_graph[v] << ',' << d_spanner[v];
                std::cout << std::endl;
            }
        }
    }

    template<class DistanceMap>
    void pathStats(const Graph& g, DistanceMap& d, const Vertex start)
    {        
        // Need scratch vector for Dijkstra's
        boost::vector_property_map<double> predecessor(n);
         
        dijkstra_shortest_paths(g, start,
            boost::predecessor_map(predecessor).distance_map(d));
    }

    void benchmark()
    {
        // create the benchmark object and add all the planners we'd like to run
        const std::string benchmark_name = environment_;
        Benchmark b(setup, benchmark_name);
        b.addPlanner(graphPlanner);
        b.addPlanner(spannerPlanner);

        b.benchmark(1, 10000, 50, false);
        b.saveResultsToFile();
    }
};

int main(int argc, char* argv[])
{
    // Turn off logging to std::out. Need the stream for file output.
    msg::noOutputHandler();

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

    Experiment exp(
        vm["environment"].as<std::string>(),
        vm["k"].as<unsigned int>(),
        vm["heuristic"].as<bool>()
    );
    exp.explore_space(vm["n"].as<unsigned int>());
    exp.make_spanner();
    exp.print_stats();
    
    //exp.benchmark();
}