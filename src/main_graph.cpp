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

#include <boost/program_options.hpp>
#include <iostream>

#include <boost/graph/graphviz.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace b = boost;
namespace po = boost::program_options;

typedef ob::RealVectorStateSpace::StateType R2State;
typedef ob::SE2StateSpace::StateType SE2State;
typedef og::PRM::Graph Graph;
typedef og::PRM::Vertex Vertex;
typedef og::PRM::Edge Edge;
typedef std::pair<ob::State*, ob::State*> Witness;

/*
    // cast the abstract state type to the type we expect

 */

class state_writer {
    public:
    state_writer(const Graph& g) : g(g) {}

    void operator()(std::ostream& out, const Vertex& v) const
    {
        const ob::State* state = g[v].state;
        const SE2State* se2state = state->as<SE2State>();
        const double x = se2state->getX() * 5;
        const double y = se2state->getY() * 5;

        out << "[ label=\"\" shape=point pos=\"" << x << ',' << y << "\"]";
    }
private:
    const Graph& g;
};

ob::PlannerPtr setupPlanner (const ob::SpaceInformationPtr si,
    const double stretch, const double epsilon)
{
    ob::PlannerPtr planner;
    if (stretch > 1.0 && epsilon > 0.0)
        planner.reset(new og::IRS2(si, stretch, epsilon));
    else if (stretch > 1.0)
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
        planner->setName("kPRM_star");
    }

    return planner;
}

void runExperiment (ob::PlannerPtr planner,
    const unsigned int max_size, const double max_time,
    const std::string& environment, const double stretch, const double epsilon)
{
    og::PRM* prm = dynamic_cast<og::PRM*>(planner.get());
    assert(prm != NULL);
    prm->resetSampler();

    const unsigned int vertex_size = sizeof(og::PRM::VertexProperties) +
        planner->getSpaceInformation()->getStateDimension() * sizeof(double);

    const unsigned int edge_size = sizeof(og::PRM::EdgeProperties) +
        2 * sizeof(Vertex);

    const Graph& g = planner->as<og::PRM>()->getRoadmap();

    unsigned int size = 0;
    double time = 0;
    const double time_step = 1.0;
    while (size < max_size && time < max_time)
    {
        planner->as<og::PRM>()->growRoadmap(time_step);
        time += time_step;
    }

    b::write_graphviz(std::cout, g, state_writer(g));
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

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("environment", po::value<std::string>(), "environment name")
        ("t", po::value<double>(), "stretch factor")
        ("e", po::value<double>(), "additive error term")
        ("size", po::value<unsigned int>(), "maximum size of roadmap (bytes)")
        ("time", po::value<double>(), "maximum time (seconds)")
        ("witness", po::value<unsigned int>(), "generate (w^2 * w)/2 queries")
        ("seed", po::value<boost::uint32_t>(), "PRNG seed")
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
    const double stretch = vm["t"].as<double>();
    const double epsilon = vm["e"].as<double>();
    const unsigned int max_size = vm["size"].as<unsigned int>();
    const double max_time = vm["time"].as<double>();
    const unsigned int witness_samples = vm["witness"].as<unsigned int>();
    const boost::uint32_t seed = vm["seed"].as<boost::uint32_t>();
    
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

    ob::PlannerPtr planner = setupPlanner(setup->getSpaceInformation(), stretch, epsilon);
    setup->setPlanner(planner);
    setup->setup();
    
    // =========================================================================

    runExperiment(planner, max_size, max_time, environment, stretch, epsilon);
}
