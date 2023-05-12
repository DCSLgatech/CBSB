
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

#include "ECBS.h"
#include "CBSB.h"

/* Main function */
int main(int argc, char **argv)
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        // help message
        ("help", "produce help message")
        // map file
        ("map,m", po::value<std::string>()->required(), "input file for map")
        // agent file
        ("agents,a", po::value<std::string>()->required(), "input file for agents")
        // output file
        ("output,o", po::value<std::string>(), "output file for statistics")
        // output path
        ("outputPaths", po::value<std::string>(), "output file for paths")
        // number of agents
        ("agentNum,k", po::value<int>()->default_value(0), "number of agents")
        // max solve time
        ("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
        // print result
        ("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
        // params for CBS node selection strategies
        ("solver", po::value<std::string>()->default_value("CBS"),
         "mapf solver (CBS, ECBS, EECBS, CBSB)")
        // suboptimality
        ("suboptimality", po::value<double>()->default_value(1.0), "suboptimality bound")
        // problem type
        ("problemType", po::value<std::string>()->default_value("FLOWTIME"), "(FLOWTIME, MAKESPAN)")
        // prioritize conflict            
        ("prioritizingConflicts", po::value<bool>()->default_value(false), "conflict prioirtization. If true, conflictSelection is used as a tie-breaking rule.")
        // params for CBS improvement
        ("bypass", po::value<bool>()->default_value(false), "bypass conflict")
        // disjoint
        ("disjointSplitting", po::value<bool>()->default_value(false), "disjoint splitting")
        // rect reasoning
        ("rectangleReasoning", po::value<bool>()->default_value(false), "rectangle reasoning")
        // corridor reasoning
        ("corridorReasoning", po::value<bool>()->default_value(false), "corridor reasoning")
        // target reasoning
        ("targetReasoning", po::value<bool>()->default_value(false), "target reasoning");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);
    if (vm["suboptimality"].as<double>() < 1)
    {
        std::cerr << "Suboptimal bound should be at least 1!" << std::endl;
        return -1;
    }

    if (vm["solver"].as<std::string>() == "CBS" && vm["suboptimality"].as<double>() > 1)
    {
        std::cerr << "CBS cannot perform suboptimal search!" << std::endl;
        return -1;
    }

    // ProblemType problem_type;
    // if (vm["problemType"].as<std::string>() == "FLOWTIME")
    // {
    //     problem_type = ProblemType::FLOWTIME;
    // }
    // else if (vm["problemType"].as<std::string>() == "MAKESPAN")
    // {
    //     problem_type = ProblemType::MAKESPAN;
    // }
    // else
    // {
    //     std::cerr << "Invalid problem type! - Choose FLOWTIME or MAKESPAN" << std::endl;
    //     return -1;
    // }
    conflict_selection conflict = conflict_selection::EARLIEST;
    node_selection n = node_selection::NODE_CONFLICTPAIRS;

    Instance instance(vm["map"].as<std::string>(), vm["agents"].as<std::string>(),
                      vm["agentNum"].as<int>());
    int runs = 1;
    if (vm["solver"].as<std::string>() == "CBS")
    {
        CBS cbs(instance, false, vm["screen"].as<int>());
        cbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
        cbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
        cbs.setBypass(vm["bypass"].as<bool>());
        cbs.setRectangleReasoning(vm["rectangleReasoning"].as<bool>());
        cbs.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
        cbs.setHeuristicType(heuristics_type::ZERO, heuristics_type::ZERO);
        cbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
        cbs.setMutexReasoning(false);
        cbs.setConflictSelectionRule(conflict);
        cbs.setNodeSelectionRule(n);
        cbs.setHighLevelSolver(high_level_solver_type::ASTAR, 1.0);
        //////////////////////////////////////////////////////////////////////
        // run
        double runtime = 0;
        int lowerbound = 0;
        for (int i = 0; i < runs; i++)
        {
            cbs.clear();
            cbs.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
            runtime += cbs.runtime;
            if (cbs.solution_found)
                break;
            lowerbound = cbs.getLowerBound();
            cbs.randomRoot = true;
            cout << "Failed to find solutions in Run " << i << endl;
        }
        cbs.runtime = runtime;
        if (vm.count("output"))
            cbs.saveSummary(vm["output"].as<string>(), vm["agents"].as<string>());
        if (cbs.solution_found && vm.count("outputPaths"))
            // cbs.savePaths(vm["outputPaths"].as<string>());
            cbs.savePathsTable(vm["outputPaths"].as<string>());
        cbs.clearSearchEngines();
    }
    else if (vm["solver"].as<std::string>() == "ECBS")
    {
        ECBS ecbs(instance, false, vm["screen"].as<int>());
        ecbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
        ecbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
        ecbs.setBypass(vm["bypass"].as<bool>());
        ecbs.setRectangleReasoning(vm["rectangleReasoning"].as<bool>());
        ecbs.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
        ecbs.setHeuristicType(heuristics_type::ZERO, heuristics_type::ZERO);
        ecbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
        ecbs.setMutexReasoning(false);
        ecbs.setConflictSelectionRule(conflict);
        ecbs.setNodeSelectionRule(n);
        ecbs.setHighLevelSolver(high_level_solver_type::ASTAREPS, vm["suboptimality"].as<double>());
        //////////////////////////////////////////////////////////////////////
        // run
        double runtime = 0;
        int lowerbound = 0;
        for (int i = 0; i < runs; i++)
        {
            ecbs.clear();
            ecbs.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
            runtime += ecbs.runtime;
            if (ecbs.solution_found)
                break;
            lowerbound = ecbs.getLowerBound();
            ecbs.randomRoot = true;
            cout << "Failed to find solutions in Run " << i << endl;
        }
        ecbs.runtime = runtime;
        if (vm.count("output"))
            ecbs.saveSummary(vm["output"].as<string>(), vm["agents"].as<string>());
        if (ecbs.solution_found && vm.count("outputPaths"))
            ecbs.savePaths(vm["outputPaths"].as<string>());
        ecbs.clearSearchEngines();
    }
    else if (vm["solver"].as<std::string>() == "EECBS")
    {
        ECBS ecbs(instance, false, vm["screen"].as<int>());
        ecbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
        ecbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
        ecbs.setBypass(vm["bypass"].as<bool>());
        ecbs.setRectangleReasoning(vm["rectangleReasoning"].as<bool>());
        ecbs.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
        ecbs.setHeuristicType(heuristics_type::ZERO, heuristics_type::GLOBAL);
        ecbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
        ecbs.setMutexReasoning(false);
        ecbs.setConflictSelectionRule(conflict);
        ecbs.setNodeSelectionRule(n);
        ecbs.setHighLevelSolver(high_level_solver_type::EES, vm["suboptimality"].as<double>());
        //////////////////////////////////////////////////////////////////////
        // run
        double runtime = 0;
        int lowerbound = 0;
        for (int i = 0; i < runs; i++)
        {
            ecbs.clear();
            ecbs.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
            runtime += ecbs.runtime;
            if (ecbs.solution_found)
                break;
            lowerbound = ecbs.getLowerBound();
            ecbs.randomRoot = true;
            cout << "Failed to find solutions in Run " << i << endl;
        }
        ecbs.runtime = runtime;
        if (vm.count("output"))
            ecbs.saveSummary(vm["output"].as<string>(), vm["agents"].as<string>());
        if (ecbs.solution_found && vm.count("outputPaths"))
            ecbs.savePaths(vm["outputPaths"].as<string>());
        ecbs.clearSearchEngines();
    }
    else if (vm["solver"].as<std::string>() == "CBSB")
    {
        CBSB cbsb(instance, true, vm["screen"].as<int>());
        cbsb.setPrioritizeConflicts(false);
        cbsb.setDisjointSplitting(false);
        cbsb.setBypass(vm["bypass"].as<bool>());
        cbsb.setRectangleReasoning(false);
        cbsb.setCorridorReasoning(false);
        cbsb.setHeuristicType(heuristics_type::ZERO, heuristics_type::ZERO);
        cbsb.setTargetReasoning(false);
        cbsb.setMutexReasoning(false);
        cbsb.setConflictSelectionRule(conflict);
        cbsb.setNodeSelectionRule(n);
        cbsb.setHighLevelSolver(high_level_solver_type::COASTAR, vm["suboptimality"].as<double>());
        //////////////////////////////////////////////////////////////////////
        // run
        double runtime = 0;
        int lowerbound = 0;
        for (int i = 0; i < runs; i++)
        {
            cbsb.clear();
            cbsb.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
            runtime += cbsb.runtime;
            if (cbsb.solution_found)
                break;
            lowerbound = cbsb.getLowerBound();
            cbsb.randomRoot = true;
            cout << "Failed to find solutions in Run " << i << endl;
        }
        cbsb.runtime = runtime;
        if (vm.count("output"))
            cbsb.saveSummary(vm["output"].as<string>(), vm["agents"].as<string>());
        if (cbsb.solution_found && vm.count("outputPaths"))
            // cbsb.savePaths(vm["outputPaths"].as<string>());
            cbsb.savePathsTable(vm["outputPaths"].as<string>());
        cbsb.clearSearchEngines();
    }

    return 0;
}