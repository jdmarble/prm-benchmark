#include <ompl/config.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
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

    struct KSpanner
    {
        boost::shared_ptr<geometric::BGL_PRM> planner_;
        const Graph& g_;
        const unsigned int k_;
        const bool heuristic_;

        KSpanner(const unsigned int k, const bool heuristic,
                 boost::shared_ptr<geometric::BGL_PRM> planner):
            planner_(new geometric::BGL_PRM(planner->getSpaceInformation())),
            g_(planner_->getGraph()),
            k_(k), heuristic_(heuristic)
        {
            planner_->copyPRMGraph(*planner);
            BaswanaSpanner<Graph> spannerCalc(g_, k, heuristic);
            const std::list<Edge> spannerEdges = spannerCalc.calculateSpanner();
            planner_->edgeSetIntersect(spannerEdges);
            cerr << "k:" << k << " heur:" << heuristic;
            cerr << " - edges:" << boost::num_edges(g_) << endl;

            planner_->setName(
                std::string("spanner_") +
                "k" + boost::lexical_cast<std::string>(k) +
                "_h" + boost::lexical_cast<std::string>(heuristic) +
                "_e" + boost::lexical_cast<std::string>(boost::num_edges(g_))
            );
        }
    };

    boost::shared_ptr<geometric::BGL_PRM> planner_;
    base::PlannerData plannerData_;
    /** Original graph */
    const Graph& g_;

    Graph::vertices_size_type n;

    std::list<KSpanner> spanners_;

    /** For picking random vertices */
    RNG rng_;

    const std::string environment_;

    Experiment(const std::string& environment) :
        planner_(new geometric::BGL_PRM(setup.getSpaceInformation())),
        g_(planner_->getGraph()),
        environment_(environment)
    {
        setup.setPlanner(planner_);
        
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

        planner_->setName("original");
    }

    void explore_space(const unsigned int target_n)
    {
        // Repeat for at least target_n milestones have been added.
        while (plannerData_.states.size() < target_n)
        {
            planner_->growRoadmap(1.0);
            planner_->getPlannerData(plannerData_);

            unsigned int verts = plannerData_.states.size();
            unsigned int edges =
                std::accumulate(plannerData_.edges.begin(), plannerData_.edges.end(),
                    0, accumSizes<unsigned int>) / 2;

            std::cerr << verts << ',' << edges << std::endl;
        }
        n = boost::num_vertices(g_);
    }

    void make_spanner(const unsigned int k, const bool heuristic = false)
    {
        spanners_.push_back(KSpanner(k, heuristic, planner_));
    }

    void print_stats()
    {
        for (int i = 0; i <= 1; ++i)
        {
            Vertex start = rng_.uniformInt(*(boost::vertices(g_).first), n);

            boost::vector_property_map<double> d_graph(n);
            pathStats(g_, d_graph, start);

            foreach (KSpanner& s, spanners_)
            {
                boost::vector_property_map<double> d_spanner(n);
                pathStats(s.g_, d_spanner, start);

                foreach(Vertex v, boost::vertices(g_))
                {
                    std::cout << s.k_ << ',' << s.heuristic_ << ',';
                    std::cout << d_graph[v] << ',' << d_spanner[v];
                    std::cout << std::endl;
                }
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
        b.addPlanner(planner_);
        foreach (KSpanner& s, spanners_)
            b.addPlanner(s.planner_);        

        // Find some valid start/goal pairs and add them to the benchmark.
        base::UniformValidStateSampler sampler(&*setup.getSpaceInformation());
        for (int i = 0; i < 100; ++i)
        {
            base::State* start = setup.getStateManifold()->allocState();
            sampler.sample(start);

            base::State* goal = setup.getStateManifold()->allocState();
            sampler.sample(goal);

            b.addProblem(start, goal);            
        }

        b.benchmark(0.01, 10000, -1, true, false);
        b.saveResultsToFile((benchmark_name + ".log").c_str());
    }
};

int main(int argc, char* argv[])
{
    // Turn off logging to std::out. Need the stream for file output.
    //msg::noOutputHandler();

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("environment", po::value<std::string>(), "environment name")
        ("n", po::value<unsigned int>(), "number of nodes in graph")
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

    for(int i = 0; i < 10; ++i)
        for(int k = 2; k <=9; ++k)
            exp.make_spanner(k);

    //exp.print_stats();
    
    exp.benchmark();
}