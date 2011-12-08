#include <ompl/config.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Time.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/geometric/planners/prm/IRS2.h>
#include <ompl/geometric/planners/prm/ConnectionFilter.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/geometric/SimpleSetup.h>

#include <omplapp/config.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/apps/SE2RigidBodyPlanning.h>

#include <boost/graph/astar_search.hpp>
#include <boost/program_options.hpp>
#include <iostream>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "baswana_randomized_spanner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace b = boost;
namespace po = boost::program_options;

typedef ob::RealVectorStateSpace::StateType R2State;
typedef og::PRM::Graph Graph;
typedef og::PRM::Vertex Vertex;
typedef og::PRM::Edge Edge;
typedef std::pair<ob::State*, ob::State*> Witness;


struct Edge_not_in_set
{
    Edge_not_in_set(const EdgeSet& edge_set) : edge_set(edge_set) {}

    bool operator()(const Edge& e)
    {
        return edge_set.find(e) == edge_set.end();
    }

    const EdgeSet& edge_set;
};

ob::PlannerPtr setupPlanner (const ob::SpaceInformationPtr si,
    const double stretch, const double epsilon, const bool baswana)
{
    ob::PlannerPtr planner;
    if (stretch > 1.0 && epsilon > 0.0)
    {
        if (baswana)
        {
            std::cerr << "Can't run Baswana algorithm with additive term.";
            exit(1);
        }
        planner.reset(new og::IRS2(si, stretch, epsilon));
    }
    else if (stretch > 1.0 && !baswana)
    {
        planner.reset(new og::PRM(si, true));
        planner->setName("IRS");

        og::PRM* prm = planner->as<og::PRM>();

        prm->setConnectionFilter(
            og::IRSFilter(prm->getRoadmap(), stretch,
                boost::bind(&og::PRM::distanceFunction, prm, _1, _2)));
    }
    else
    {
        planner.reset(new og::PRM(si, true));

	if (stretch > 1.0)
	  planner->setName("SRS");
	else
	  planner->setName("kPRM*");
    }

    return planner;
}

void outputWitnessQuality (og::PRM* const prm,
    const std::vector<Witness>& witnesses)
{
    og::PRM::Graph& g = prm->getRoadmap();
    boost::vector_property_map<double> distance_map(boost::num_vertices(g) + 2);

    const ompl::time::point start_time = ompl::time::now();
    double search_time = 0;

    std::cout << ":costs [";
    foreach (const Witness& witness, witnesses)
    {
        Vertex start = prm->addFakeMilestone(witness.first);
        Vertex goal = prm->addFakeMilestone(witness.second);

        const ompl::time::point search_start = ompl::time::now();

        b::astar_search(g, start,
            b::bind(&og::PRM::distanceFunction, prm, _1, goal),
            b::weight_map(boost::get(&og::PRM::EdgeProperties::weight, g))
                    .distance_map(distance_map));

        search_time += ompl::time::seconds(ompl::time::now() - search_start);

        std::cout << distance_map[goal] << ' ';

        b::clear_vertex(goal, g);
        b::remove_vertex(goal, g);
        b::clear_vertex(start, g);
        b::remove_vertex(start, g);
    }
    std::cout << ']';

    const double query_time = ompl::time::seconds(ompl::time::now() - start_time);

    std::cout << " :query_time " << query_time / witnesses.size();
    std::cout << " :search_time " << search_time / witnesses.size();
}

void generateWitnesses (const unsigned int n, ob::PlannerPtr planner,
    std::vector<Witness>& output)
{
    assert(output.empty());
    ob::SpaceInformationPtr si = planner->getSpaceInformation();
    ob::ValidStateSamplerPtr sampler = si->allocValidStateSampler();

    std::vector<ob::State*> samples;
    for (unsigned int i = 0; i < n; ++i)
    {
        ob::State* state = si->allocState();
        sampler->sample(state);

        foreach(ob::State* const goal, samples)
            output.push_back(std::make_pair(state, goal));

        samples.push_back(state);
    }

    assert(output.size() == (n*n - n)/2);
}

void runExperiment (ob::PlannerPtr planner, const std::vector<Witness>& witnesses,
    const unsigned int max_size, const double max_time, const unsigned int max_space,
    const double time_step,
    const std::string& environment, const double stretch, const double epsilon,
    const bool baswana, const boost::uint32_t seed)
{
    og::PRM* prm = dynamic_cast<og::PRM*>(planner.get());
    assert(prm != NULL);
    prm->resetSampler();

    const unsigned int vertex_size = sizeof(og::PRM::VertexProperties) +
        planner->getSpaceInformation()->getStateDimension() * sizeof(double);

    const unsigned int edge_size = sizeof(og::PRM::EdgeProperties) +
        2 * sizeof(Vertex);

    Graph& g = planner->as<og::PRM>()->getRoadmap();

    std::cout << "{:environment \"" << environment <<
      "\" :planner \"" << planner->getName() << '"';

    std::cout << " :stretch " << stretch;
    std::cout << " :epsilon " << epsilon;

    unsigned int k;
    if (baswana)
    {
        // Stretch must be of the form 2k+1 for some integer k > 0.
        assert(std::floor((stretch+1)/2) == (stretch+1)/2);
        k = (stretch+1)/2;
        assert(k > 1);
        std::cout << " :k " << k;
    }

    std::cout << " :data [" << std::endl;
    unsigned int space = 0;
    unsigned int n = 0;
    double time = 0;
    while ((max_size == 0 || n < max_size) && (max_space == 0 || space < max_space) && time < max_time)
    {
        planner->as<og::PRM>()->growRoadmap(time_step);
        time += time_step;
        n = b::num_vertices(g);
        const unsigned int m = b::num_edges(g);
        space = (n * vertex_size) + (m * edge_size);

        std::cout << '{';
        std::cout << ":n " << n << ' ';
        std::cout << ":m " << m << ' ';
        std::cout << ":time " << time << ' ';
        std::cout << ":size " << space << ' ';

        const og::IRS2* const irs2 = dynamic_cast<const og::IRS2*>(planner.get());
        if (irs2 != NULL)
        {
            std::cout << ":max_failures " << irs2->getMaxFailures() << ' ';
            const Graph& g_dense = irs2->getDenseRoadmap();
            const unsigned int n_dense = b::num_vertices(g_dense);
            const unsigned int m_dense = b::num_edges(g_dense);
            const unsigned int size_dense = (n_dense * vertex_size) + (m_dense * edge_size);

            std::cout << ":n_dense " << n_dense << ' ';
            std::cout << ":m_dense " << m_dense << ' ';
            std::cout << ":size_dense " << size_dense << ' ';
        }

        outputWitnessQuality(prm, witnesses);

        std::cout << '}' << std::endl;
    }

    if (baswana)
    {
        BaswanaSpanner spannerCalc(g, seed, k, false);

        const ompl::time::point spanner_start = ompl::time::now();
        const list<Edge>& spanner_edges = spannerCalc.calculateSpanner();

	const double additional_time = ompl::time::seconds(ompl::time::now() - spanner_start);
        time += additional_time;

        // Remove edges not in the set of spanner edges.
        EdgeSet spanner_set(spanner_edges.begin(), spanner_edges.end(),
                            4, PRM_EdgeHash(g), PRM_EdgeEq(g));
        boost::remove_edge_if(Edge_not_in_set(spanner_set), g);

        const unsigned int n = b::num_vertices(g);
        const unsigned int m = b::num_edges(g);
        const unsigned int space = (n * vertex_size) + (m * edge_size);

        std::cout << '{';
        std::cout << ":n " << n << ' ';
        std::cout << ":m " << m << ' ';
        std::cout << ":time " << time << ' ';
	std::cout << ":additional_time " << additional_time << ' ';
        std::cout << ":size " << space << ' ';
        outputWitnessQuality(prm, witnesses);
        std::cout << '}' << std::endl;

    }

    std::cout << ']' << std::endl;
    std::cout << '}' << std::endl;
}

bool isR2StateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const R2State* r2state = state->as<R2State>();
    const double x = (*r2state)[0];
    const double y = (*r2state)[1];

    // check validity of state
    const bool inSlat = (int)(y*10)%2;
    return !(inSlat && x > -0.75 && x < 0.75);
}

og::SimpleSetup* setupR2()
{
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    og::SimpleSetup* setup = new og::SimpleSetup(space);

    // setting collision checking resolution to 1% of the space extent
    setup->getSpaceInformation()->setStateValidityChecker(
        boost::bind(&isR2StateValid, _1));

    return setup;
}


og::SimpleSetup* setupSE3(const std::string environment)
{
    ompl::app::SE3RigidBodyPlanning* setup
        = new ompl::app::SE3RigidBodyPlanning();

    // load the robot and the environment
    const std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR)
            + "/3D/" + environment + "_robot.dae";
    const std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR)
            + "/3D/" + environment + "_env.dae";
    setup->setRobotMesh(robot_fname.c_str());
    setup->setEnvironmentMesh(env_fname.c_str());

    return setup;
}

og::SimpleSetup* setupSE2()
{
    ompl::app::SE2RigidBodyPlanning* setup
        = new ompl::app::SE2RigidBodyPlanning();

    // load the robot and the environment
    const std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR)
            + "/2D/UniqueSolutionMaze_robot.dae";
    const std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR)
            + "/2D/UniqueSolutionMaze_env.dae";
    setup->setRobotMesh(robot_fname.c_str());
    setup->setEnvironmentMesh(env_fname.c_str());

    return setup;
}

int main(int argc, char* argv[])
{
    // Turn off logging to std::out. Need the stream for file output.
    ompl::msg::noOutputHandler();

    // Use the same seed for each run to get the same witnesses.
    // Later, change the seed to the one specified on the command line.
    ompl::RNG::setSeed(1);

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("environment", po::value<std::string>()->default_value("point"), "environment name")
        ("stretch",     po::value<double>()->default_value(1.0),          "stretch factor")
        ("error",       po::value<double>()->default_value(0.0),          "additive error term")
        ("space",       po::value<unsigned int>()->default_value(0),      "maximum size of roadmap (bytes)")
        ("size",        po::value<unsigned int>()->default_value(0),      "maximum size of roadmap (nodes)")
        ("time",        po::value<double>()->default_value(10.0),         "maximum time (seconds)")
        ("witness",     po::value<unsigned int>()->default_value(2),      "generate (w^2 * w)/2 queries")
        ("seed",        po::value<boost::uint32_t>()->default_value(1),   "PRNG seed")
        ("baswana",     po::value<bool>()->default_value(false),          "run spanner after")
        ("step",        po::value<double>()->default_value(1.0),          "time step")
     ;


    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    }
    catch (boost::program_options::error& e)
    {
        std::cerr << "Can't process command line arguments: " << e.what() << std::endl;
        return 1;
    }

    if (vm.count("help")) {
        std::cerr << desc << "\n";
        return 1;
    }
    const std::string environment = vm["environment"].as<std::string>();
    const double stretch = vm["stretch"].as<double>();
    const double epsilon = vm["error"].as<double>();
    const unsigned int max_size = vm["size"].as<unsigned int>();
    const unsigned int max_space = vm["space"].as<unsigned int>();
    const double max_time = vm["time"].as<double>();
    const unsigned int witness_samples = vm["witness"].as<unsigned int>();
    const boost::uint32_t seed = vm["seed"].as<boost::uint32_t>();
    const bool baswana = vm["baswana"].as<bool>();
    const double time_step = vm["step"].as<double>();

    assert(time_step <= max_time);
    // =========================================================================

    og::SimpleSetup* setup;
    if (environment == "point")
        setup = setupR2();
    else if (environment == "maze")
        setup = setupSE2();
    else
        setup = setupSE3(environment);

    // setting collision checking resolution to 1% of the space extent
    setup->getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    ob::PlannerPtr planner = setupPlanner(setup->getSpaceInformation(), stretch, epsilon, baswana);
    setup->setPlanner(planner);
    setup->setup();

    std::vector<Witness> witnesses;
    generateWitnesses(witness_samples, planner, witnesses);
    // Set real seed -after- so each run has the same witnesses but different
    // results.
    for(boost::uint32_t i; i < seed; ++i)
        ompl::RNG discard(); // Hack to advance the seeds...
    
    // =========================================================================

    runExperiment(planner, witnesses, max_size, max_time, max_space, time_step,
        environment, stretch, epsilon, baswana, seed);

}
