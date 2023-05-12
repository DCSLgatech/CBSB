#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include "CBS.h"
// #include "SIPP.h"
#include "SpaceTimeAStar.h"
#include "SpaceTimeCOAStar.h"

//=============================================================================
CBS::CBS(vector<SingleAgentSolver*>& search_engines,
	const vector<ConstraintTable>& initial_constraints,
	vector<Path>& paths_found_initially, int screen) :
	screen(screen), suboptimality(1),
	initial_constraints(initial_constraints), paths_found_initially(paths_found_initially),
	search_engines(search_engines),
	mdd_helper(initial_constraints, search_engines),
	rectangle_helper(search_engines[0]->instance),
	mutex_helper(search_engines[0]->instance, initial_constraints),
	corridor_helper(search_engines, initial_constraints),
	heuristic_helper(search_engines.size(), paths, search_engines, initial_constraints, mdd_helper)
{
	num_of_agents = (int) search_engines.size();
	mutex_helper.search_engines = search_engines;
}

//=============================================================================
CBS::CBS(const Instance& instance, bool sipp, int screen) :
	screen(screen), suboptimality(1),
	num_of_agents(instance.getDefaultNumberOfAgents()),
	mdd_helper(initial_constraints, search_engines),
	rectangle_helper(instance),
	mutex_helper(instance, initial_constraints),
	corridor_helper(search_engines, initial_constraints),
	heuristic_helper(instance.getDefaultNumberOfAgents(), paths, search_engines, initial_constraints, mdd_helper)
{
	clock_t t = clock();
	initial_constraints.resize(num_of_agents,
		ConstraintTable(instance.num_of_cols, instance.map_size));

	search_engines.resize(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (sipp)
			search_engines[i] = new SpaceTimeCOAStar(instance, i);
		else
			search_engines[i] = new SpaceTimeAStar(instance, i);

		initial_constraints[i].goal_location = search_engines[i]->goal_location;
	}
	runtime_preprocessing = (double)(clock() - t) / CLOCKS_PER_SEC;

	mutex_helper.search_engines = search_engines;

	if (screen >= 2) // print start and goals
	{
		instance.printAgents();
	}
}

//=============================================================================
CBS::~CBS()
{
	releaseNodes();
	mdd_helper.clear();
}

//=============================================================================
// used for rapid random  restart
void CBS::clear()
{
	mdd_helper.clear();
	heuristic_helper.clear();
	releaseNodes();
	paths.clear();
	paths_found_initially.clear();
	dummy_start = nullptr;
	goal_node = nullptr;
	solution_found = false;
	solution_cost = -2;
}

//=============================================================================
inline void CBS::releaseNodes()
{
	open_list.clear();
	cleanup_list.clear();
	focal_list.clear();
	bcoa_list.clear();
	for (auto& node : allNodes_table)
		delete node;
	allNodes_table.clear();
}

//=============================================================================
void CBS::clearSearchEngines()
{
	for (auto s : search_engines)
		delete s;
	search_engines.clear();
}

//=============================================================================
bool CBS::solve(double _time_limit, int _cost_lowerbound, int _cost_upperbound)
{
	this->cost_lowerbound = _cost_lowerbound;
	this->inadmissible_cost_lowerbound = 0;
	this->cost_upperbound = _cost_upperbound;
	this->time_limit = _time_limit;

	if (screen > 0) // 1 or 2
	{
		string name = getSolverName();
		name.resize(35, ' ');
		cout << name << ": ";
	}
	// set timer
	start = clock();

	// Generate root by planning individual shortest path planning problem
	generateRoot();

	// Main high-level search loop
	while (!cleanup_list.empty() && !solution_found)
	{
		// pop front of the OPEN, choose CT node for expansion
		auto curr = selectNode();

		// Are we done?
		if (terminate(curr))
			return solution_found;

		// No, let's continue, but do we want to prioritize conflicts?
		if (PC) // priortize conflicts
			classifyConflicts(*curr);

		// Let's compute the heuristic if we have not.
		if (!curr->h_computed) // heuristics has not been computed yet
		{
			runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
			bool succ = heuristic_helper.computeInformedHeuristics(*curr, time_limit - runtime);
			runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
            heuristic_helper.updateOnlineHeuristicErrors(*curr);
            heuristic_helper.updateInadmissibleHeuristics(*curr); // compute inadmissible heuristics
			/*if (runtime > time_limit)
			{  // timeout
				solution_cost = -1;
				solution_found = false;
				break;
			}*/
			if (!succ) // no solution, so prune this node
			{
				curr->clear();
				continue;
			}

			if (reinsertNode(curr))
			{
				continue;
			}
		}

		//Expand the node
		num_HL_expanded++;
		curr->time_expanded = num_HL_expanded;

		// Split the node if no bypass is found
		// While bypass exists, keep bypassing the node.
		bool foundBypass = true;
		while (foundBypass)
		{
			// Are we still good to go?
			if(terminate(curr))
				return solution_found;

			// Assume no more bypass for now.
			foundBypass = false;

			// Yes, let's branch out.
			CBSNode* child[2] = { new CBSNode() , new CBSNode() };

			// Let's pick a conflict to resolve, if not PC, then this is at random
			curr->conflict = chooseConflict(*curr);

			// Let's split the conflict to assign constraints to each child
			addConstraints(curr, child[0], child[1]);

			if (screen > 1)
				cout << "	Expand " << *curr << endl <<
				"	on " << *(curr->conflict) << endl;

			// Can we generate child? i.e., does a new path exist with additional constraints?
			// If not, then that child is a deadend.
			bool solved[2] = { false, false };
			vector<vector<PathEntry>*> copy(paths);

			for (int i = 0; i < 2; i++)
			{
				if (i > 0)
					paths = copy;

				// Let's replan for each child node
				solved[i] = generateChild(child[i], curr);
				if (!solved[i])
				{
					// This child is a dead end, no valid solution exists, delete it.
					delete (child[i]);
					continue;
				}
				// Do we want to bypass the conflict?
				else if (bypass && child[i]->g_val == curr->g_val && child[i]->distance_to_go < curr->distance_to_go) // Bypass1
				{
					// Yes, since g_val is the same, and we have more promising cost-to-go.
					// But if the first child was a deadend, then let's not bypass it.
					if (i == 1 && !solved[0])
						continue;

					// Yes, we can bypass, and let's do it.
					foundBypass = true;
					num_adopt_bypass++;

					// Adopt this child values to the current node
					curr->conflicts = child[i]->conflicts;
					curr->unknownConf = child[i]->unknownConf;
					curr->distance_to_go = child[i]->distance_to_go;
					curr->conflict = nullptr;
					for (const auto& path : child[i]->paths) // update paths
					{
						auto p = curr->paths.begin();
						while (p != curr->paths.end())
						{
							if (path.first == p->first) // path.first = agent_id, path.second = path
							{
								p->second = path.second;
								paths[p->first] = &p->second;
								break;
							}
							++p;
						}
						if (p == curr->paths.end())
						{
							curr->paths.emplace_back(path);
							paths[path.first] = &curr->paths.back().second;
						}
					}
					if (screen > 1)
					{
						cout << "	Update " << *curr << endl;
					}
					// Yes, we have successfully found a bypass node, and replaced the parent with it.
					break;
				} //end if bypassing child found
				// No we don't bypass.
			} // end for each child

			// Have we found a bypass?
			if (foundBypass)
			{
				// Yes, then delete all child
				for (auto & i : child)
				{
					delete i;
					i = nullptr;
				}
				// Let's re prioritize conflicts
				if (PC) // priortize conflicts
					classifyConflicts(*curr);
			}
			else
			{
				// No we have not found a bypass.
				for (int i = 0; i < 2; i++)
				{
					// Did this child have a solution?
					if (solved[i])
					{
						// Yes, then put this node in the queue
						pushNode(child[i]);
						// Also, save this in children
						curr->children.push_back(child[i]);

						if (screen > 1)
						{
							cout << "		Generate " << *child[i] << endl;
						}
					}
				}

				// Let's count the numbers (only the chosen conflict)
				switch (curr->conflict->type)
				{
				case conflict_type::RECTANGLE:
					num_rectangle_conflicts++;
					break;
				case conflict_type::CORRIDOR:
					num_corridor_conflicts++;
					break;
				case  conflict_type::TARGET:
					num_target_conflicts++;
					break;
				case conflict_type::STANDARD:
					num_standard_conflicts++;
					break;
				case conflict_type::MUTEX:
					num_mutex_conflicts++;
					break;
				default:
					break;
				}
				if (curr->chosen_from == "cleanup")
					num_cleanup++;
				else if (curr->chosen_from == "open")
					num_open++;
				else if (curr->chosen_from == "focal")
					num_focal++;
				if (curr->conflict->priority == conflict_priority::CARDINAL)
					num_cardinal_conflicts++;

				// Let's clear out the conflicts for memory save
				curr->clear();
			}
		} // end while found bypass
	}  // end of while loop
	return solution_found;
}

//=============================================================================
bool CBS::generateRoot()
{
	auto root = new CBSNode();
	root->g_val = 0;
	paths.resize(num_of_agents, nullptr);

	mdd_helper.init(num_of_agents);
	heuristic_helper.init();

	// initialize paths_found_initially
	if (paths_found_initially.empty())
	{
		paths_found_initially.resize(num_of_agents);

		//generate random permuattion of agent indices
		vector<int> agents(num_of_agents);
		for (int i = 0; i < num_of_agents; i++)
		{
			agents[i] = i;
		}

		if (randomRoot)
		{
			std::random_device rd;
			std::mt19937 g(rd());
			std::shuffle(std::begin(agents), std::end(agents), g);
		}

		// Now solve shortest path problem for each agent
		for (auto i : agents)
		{
			// Find initial solution
			paths_found_initially[i] = search_engines[i]->findOptimalPath(*root, initial_constraints[i], paths, i, 0);
			if (paths_found_initially[i].empty())
			{
				cout << "No path exists for agent " << i << endl;
				return false;
			}
			// collect the results
			paths[i] = &paths_found_initially[i];
			root->makespan = max(root->makespan, paths_found_initially[i].size() - 1);
			root->g_val += (int)paths_found_initially[i].size() - 1;
			num_LL_expanded += search_engines[i]->num_expanded;
			num_LL_generated += search_engines[i]->num_generated;
			num_LL_conflicts += search_engines[i]->num_conflicts;
		}
	}
	else
	{
		// We already have initial paths
		for (int i = 0; i < num_of_agents; i++)
		{
			paths[i] = &paths_found_initially[i];
			root->makespan = max(root->makespan, paths_found_initially[i].size() - 1);
			root->g_val += (int) paths_found_initially[i].size() - 1;
		}
	}

	// mark this root
	root->h_val = 0;
	root->depth = 0;

	// Find conflicts within this root, this assigns constraints to conflicting agents
	findConflicts(*root);

	// Now compute the heuristic for this root
	heuristic_helper.computeQuickHeuristics(*root);

	// Put this node in the queue
	pushNode(root);

	dummy_start = root;
	if (screen >= 2) // print start and goals
	{
		printPaths();
	}

	return true;
}


//=============================================================================
// Finds all conflict in the current curr, call findConflicts for every pair of agents
void CBS::findConflicts(HLNode& curr)
{
	clock_t t = clock();
	// Do we have a parent CT node to copy any unresolved conflicts?
	if (curr.parent != nullptr)
	{
		// Note that each CT node contains a set of paths only for replanning agents.
		auto new_agents = curr.getReplannedAgents();

		// Copy from parent CT node, but do not copy conflicts of new_agents.
		copyConflicts(curr.parent->conflicts, curr.conflicts, new_agents);
		copyConflicts(curr.parent->unknownConf, curr.unknownConf, new_agents);

		// detect new conflicts
		for (auto it = new_agents.begin(); it != new_agents.end(); ++it)
		{
			int a1 = *it;
			for (int a2 = 0; a2 < num_of_agents; a2++)
			{
				// No self conflict!
				if (a1 == a2)
					continue;
				bool skip = false;

				// Do not need this part...
				// They were trying to skip conflict between new-agents, but decided not to.
				for (auto it2 = new_agents.begin(); it2 != it; ++it2)
				{
					if (*it2 == a2)
					{
						skip = true;
						break;
					}
				}
				// Yes, a2 belongs to new_agents, check conflict
				// if (skip) // This was intended by original authors, but no need anymore.
				findConflicts(curr, a1, a2);

			} // end for second agent
		} // end for new_agents
	} // end if not root CT node
	else
	{
		// This is a root CT node, so check all-to-all conflicts
		for (int a1 = 0; a1 < num_of_agents; a1++)
		{
			for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
			{
				findConflicts(curr, a1, a2);
			}
		}
	}
	// curr.distance_to_go = (int)(curr.unknownConf.size() + curr.conflicts.size());
	runtime_detect_conflicts += (double)(clock() - t) / CLOCKS_PER_SEC;
}

//=============================================================================
// Find all conflicts between two agents and determine whether a conflict is
// vertex conflict (target conflict) or edge conflict,
// saving constraints to for the conflicting agents to resolve this conflict
void CBS::findConflicts(HLNode& curr, int a1, int a2)
{

	// First, check any conflict while two agents are moving.
	int min_path_length = (int) (paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size());
	for (int timestep = 0; timestep < min_path_length; timestep++)
	{

		// Iterate through the timesteps of a shorter path and check the locations
		int loc1 = paths[a1]->at(timestep).location;
		int loc2 = paths[a2]->at(timestep).location;

		// is it on the same location?
		if (loc1 == loc2)
		{
			// Yes, it's a vertex conflict, now check if this is a target conflict
			shared_ptr<Conflict> conflict(new Conflict());
			// Are we using target reasoning?
			if (target_reasoning && paths[a1]->size() == timestep + 1)
			{
				// a1 is at its target (shorter)
				// either one must hold true: a1 has to reach its goal at or before timestep
				// 														a1 has to reach its goal after timestep
				conflict->targetConflict(a1, a2, loc1, timestep);
			}
			else if (target_reasoning && paths[a2]->size() == timestep + 1)
			{
				// a2 is at its target
				// either one must hold true: a2 has to reach its goal at or before timestep
				// 														a2 has to reach its goal after timestep
				conflict->targetConflict(a2, a1, loc1, timestep);
			}
			else
			{
				// This is not a target conflict nor do we care
				conflict->vertexConflict(a1, a2, loc1, timestep);
			}
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			curr.unknownConf.push_back(conflict); // unknown of its priority (cardinal, etc)
		}
		// No, check if it is an edge conflict
		else if (timestep < min_path_length - 1
			&& loc1 == paths[a2]->at(timestep + 1).location
			&& loc2 == paths[a1]->at(timestep + 1).location)
		{
			shared_ptr<Conflict> conflict(new Conflict());
			conflict->edgeConflict(a1, a2, loc1, loc2, timestep + 1);
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			curr.unknownConf.push_back(conflict); // edge conflict
		}
		// No else, there is no conflict at this timestep
	}

	// Now check any conflict after one agent stops moving
	if (paths[a1]->size() != paths[a2]->size())
	{
		// call the agent with a shorter path as a1_ and with a longer path a2_
		int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
		int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
		int loc1 = paths[a1_]->back().location;

		// time between the shorter and the longer
		for (int timestep = min_path_length; timestep < (int)paths[a2_]->size(); timestep++)
		{
			int loc2 = paths[a2_]->at(timestep).location;
			if (loc1 == loc2)
			{
				// This is a target conflict, since a1_ never moves.
				shared_ptr<Conflict> conflict(new Conflict());
				// But do we reason about target?
				if (target_reasoning)
					conflict->targetConflict(a1_, a2_, loc1, timestep);
				else
					conflict->vertexConflict(a1_, a2_, loc1, timestep);
				assert(!conflict->constraint1.empty());
				assert(!conflict->constraint2.empty());
				curr.unknownConf.push_front(conflict); // It's at least a semi conflict
			}
		}
	} // end if path lengths not equal
}
//=============================================================================
// deep copy of all conflicts except ones that involve the particular agent
// used for copying conflicts from the parent node to the child nodes
void CBS::copyConflicts(const list<shared_ptr<Conflict >>& conflicts,
	list<shared_ptr<Conflict>>& copy, const list<int>& excluded_agents)
{
	// Go through each conflict
	for (auto& conflict : conflicts)
	{
		// check if this conflict involves an excluded agent
		bool found = false;
		for (auto a : excluded_agents)
		{
			if (conflict->a1 == a || conflict->a2 == a)
			{
				found = true;
				break;
			}
		}
		// No this conflict does not include an excluded agent, so copy it
		if (!found)
		{
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			copy.push_back(conflict);
		}
	}
}


//=============================================================================
// Compute the conflict priority of unknownConfs, and also classifies
// mutex, corridor, rectangle conflicts
void CBS::classifyConflicts(CBSNode &node)
{
	// Classify all conflicts in unknownConf (unknown of type) and store them to conflicts
	while (!node.unknownConf.empty())
	{
		// PopFront unknownConf for classfication
		// Extract conflicting agents, timestep, and the type ()
		shared_ptr<Conflict> con = node.unknownConf.front();

		int a1 = con->a1, a2 = con->a2;

		int timestep = get<3>(con->constraint1.back());

		constraint_type type = get<4>(con->constraint1.back());

		// Get rid of from unknownConf list (pop)
		node.unknownConf.pop_front();

		// Determines cardinal, pseudo_cardinal, semi, non, or unknown conflicts of priority.
		computeConflictPriority(con, node);

		if (con->priority == conflict_priority::CARDINAL && heuristic_helper.type == heuristics_type::ZERO)
		{
			// If this is cardinal, and heuristic type is zero,
			// Then no need to further classify the left unknownConfs, just return
			// by assigning its secondary priority
			computeSecondPriorityForConflict(*con, node);
			node.conflicts.push_back(con);
			return;
		}

		// Mutex reasoning
		if (mutex_reasoning)
		{
			// TODO mutex reasoning is per agent pair, don't do duplicated work...
			auto mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size());
			auto mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());

			auto mutex_conflict = mutex_helper.run(a1, a2, node, mdd1, mdd2);

			if (mutex_conflict != nullptr)
			{
				computeSecondPriorityForConflict(*mutex_conflict, node);
				node.conflicts.push_back(mutex_conflict);
				continue;
			}
		}

		// Target Reasoning
		if (con->type == conflict_type::TARGET)
		{
			computeSecondPriorityForConflict(*con, node);
			node.conflicts.push_back(con);
			continue;
		}

		// Corridor reasoning
		if (corridor_reasoning)
		{
			auto corridor = corridor_helper.run(con, paths, node);
			if (corridor != nullptr)
			{
				corridor->priority = con->priority;
				computeSecondPriorityForConflict(*corridor, node);
				node.conflicts.push_back(corridor);
				continue;
			}
		}


		// Rectangle reasoning
		if (rectangle_reasoning &&
			(int)paths[con->a1]->size() > timestep &&
			(int)paths[con->a2]->size() > timestep && //conflict happens before both agents reach their goal locations
			type == constraint_type::VERTEX) // vertex conflict
		{
			auto mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size());
			auto mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());
			auto rectangle = rectangle_helper.run(paths, timestep, a1, a2, mdd1, mdd2);
			if (rectangle != nullptr)
			{
				computeSecondPriorityForConflict(*rectangle, node);
				node.conflicts.push_back(rectangle);
				continue;
			}
		}

		computeSecondPriorityForConflict(*con, node);
		node.conflicts.push_back(con);
	} // end while unknownConf

	// remove conflicts that cannot be chosen, to save some memory
	removeLowPriorityConflicts(node.conflicts);
}


//=============================================================================
// This assigns a primary priority based on cardinality
void CBS::computeConflictPriority(shared_ptr<Conflict>& con, CBSNode& node)
{
	// Learn about this conflict
	int a1 = con->a1, a2 = con->a2;
	int timestep = get<3>(con->constraint1.back());
	constraint_type type = get<4>(con->constraint1.back());

	// Determine the cardinality of this conflict
	bool cardinal1 = false, cardinal2 = false;
	MDD *mdd1 = nullptr, *mdd2 = nullptr;

	// First, all target conflicts are cardinal for at least for stopping agent
	if (timestep >= (int)paths[a1]->size())
		cardinal1 = true;
	else //if (!paths[a1]->at(0).is_single())
	{
		// If not, then construct its mdd
		mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size());
	}

	// Is a2 a stopping agent?
	if (timestep >= (int)paths[a2]->size())
		cardinal2 = true;
	else //if (!paths[a2]->at(0).is_single())
	{
		// If not, then construct its mdd
		mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());
	}

	if (type == constraint_type::EDGE) // Edge conflict
	{
		cardinal1 = mdd1->levels[timestep].size() == 1 && mdd1->levels[timestep - 1].size() == 1;
		cardinal2 = mdd2->levels[timestep].size() == 1 && mdd2->levels[timestep - 1].size() == 1;
	}
	else // vertex conflict or target conflict
	{
		if (!cardinal1)
			cardinal1 = mdd1->levels[timestep].size() == 1;
		if (!cardinal2)
			cardinal2 = mdd2->levels[timestep].size() == 1;
	}

	/*int width_1 = 1, width_2 = 1;

	if (paths[a1]->size() > timestep){
	width_1 = paths[a1]->at(timestep).mdd_width;
	}

	if (paths[a2]->size() > timestep){
	width_2 = paths[a2]->at(timestep).mdd_width;
	}
	con -> mdd_width = width_1 * width_2;*/


	// Now see if the conflict is cardinal, semi, or none.
	if (cardinal1 && cardinal2)
	{
		con->priority = conflict_priority::CARDINAL;
	}
	else if (cardinal1 || cardinal2)
	{
		con->priority = conflict_priority::SEMI;
	}
	else
	{
		con->priority = conflict_priority::NON;
	}
}

//=============================================================================
// This assigns a secondary priority key for tie-breaking among the same priority and type
void CBS::computeSecondPriorityForConflict(Conflict& conflict, const HLNode& node)
{
	int count[2] = {0, 0};

	// What is a given selection rule
	switch (conflict_seletion_rule)
	{

	// Don't care, pick at random
	case conflict_selection::RANDOM:
		conflict.secondary_priority = 0;
		break;

	// Assign according to earliest time
	case conflict_selection::EARLIEST:
		switch (conflict.type)
		{
		case conflict_type::STANDARD:
		case conflict_type::RECTANGLE:
		case conflict_type::TARGET:
		case conflict_type::MUTEX:
			conflict.secondary_priority = get<3>(conflict.constraint1.front());
			break;
		case conflict_type::CORRIDOR:
			conflict.secondary_priority = min(get<2>(conflict.constraint1.front()),
											  get<3>(conflict.constraint1.front()));
			break;
		default:
			break;
		}
		break;

	// Assign the number of conflicting agents with these two
	case conflict_selection::CONFLICTS:
		for (const auto& c : node.conflicts)
		{
			if (c->a1 == conflict.a1 || c->a2 == conflict.a1)
				count[0]++;
			if (c->a1 == conflict.a2 || c->a2 == conflict.a2)
				count[1]++;
		}
		conflict.secondary_priority = count[0] + count[1];
		break;
	default:
		break;
	}
}


//=============================================================================
// If there exists multiple conflicts between the same agents,
// then delete the ones with lower priority.
void CBS::removeLowPriorityConflicts(list<shared_ptr<Conflict>>& conflicts) const
{
	// nothing to remove
	if (conflicts.empty())
		return;

	// The conflicts to keep, map by its index
	unordered_map<int, shared_ptr<Conflict> > keep;

	// The conflicts to be deleted
	list<shared_ptr<Conflict>> to_delete;

	// Going through each conflict
	for (const auto& conflict : conflicts)
	{
		// Compute a unique index for conflict
		int a1 = min(conflict->a1, conflict->a2), a2 = max(conflict->a1, conflict->a2);
		int key = a1 * num_of_agents + a2;

		// Do we have already a keeper for the conflict between the same agents?
		auto p = keep.find(key);
		if (p == keep.end())
		{
			// No, let's keep it for now
			keep[key] = conflict;
		}
		else if (*(p->second) < *conflict)
		{
			// Yes, and this conflict has a higher priority, so delete the other
			to_delete.push_back(p->second);
			keep[key] = conflict;
		}
		else
		{
			// Yes, but this conflict has lower priority, delete it.
			to_delete.push_back(conflict);
		}
	}

	// Now, let's remove them!
	for (const auto& conflict : to_delete)
	{
		conflicts.remove(conflict);
	}
}


//=============================================================================
// Choose which conflict to branch
shared_ptr<Conflict> CBS::chooseConflict(const HLNode &node) const
{
	// Debug?
	if (screen == 3)
		printConflicts(node);

	// This is what we will output
	shared_ptr<Conflict> choose;

	// Does this node have a conflict? if not, return nullptr
	if (node.conflicts.empty() && node.unknownConf.empty())
		return nullptr;

	// Yes, we have a conflict with priority known
	else if (!node.conflicts.empty())
	{
		// choose the conflict with the highest priority
		choose = node.conflicts.back();
		for (const auto& conflict : node.conflicts)
		{
			// pick a conflict of higher priority
			if (*choose < *conflict)
				choose = conflict;
		}
	}

	// Yes, and this is an unknownConf
	else
	{
		// Choose the conflict with the highest priority
		choose = node.unknownConf.back();
		for (const auto& conflict : node.unknownConf)
		{
			if (*choose < *conflict)
				choose = conflict;
		}
	}
	return choose;
}


//=============================================================================
// Assigns a set of constraints to each agent (splitting the chosen conflict)
void CBS::addConstraints(const HLNode* curr, HLNode* child1, HLNode* child2) const
{
	if (disjoint_splitting && curr->conflict->type == conflict_type::STANDARD)
	{
		int first = (bool)(rand() % 2);
		if (first) // disjoint splitting on the first agent
		{
			child1->constraints = curr->conflict->constraint1;
			int a, x, y, t;
			constraint_type type;
			tie(a, x, y, t, type) = curr->conflict->constraint1.back();
			if (type == constraint_type::VERTEX)
			{
				child2->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_VERTEX);
			}
			else
			{
				assert(type == constraint_type::EDGE);
				child2->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_EDGE);
			}
		}
		else // disjoint splitting on the second agent
		{
			child2->constraints = curr->conflict->constraint2;
			int a, x, y, t;
			constraint_type type;
			tie(a, x, y, t, type) = curr->conflict->constraint2.back();
			if (type == constraint_type::VERTEX)
			{
				child1->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_VERTEX);
			}
			else
			{
				assert(type == constraint_type::EDGE);
				child1->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_EDGE);
			}
		}
	}
	else
	{
		child1->constraints = curr->conflict->constraint1;
		child2->constraints = curr->conflict->constraint2;
	}
}

//=============================================================================
// Replan for newly constrained agents, then find new conflicts and compute heuristic
bool CBS::generateChild(CBSNode*  node, CBSNode* parent)
{
	clock_t t1 = clock();

	// Assign the parent and inherit the node values
	node->parent = parent; // CBSNode parent
	node->HLNode::parent = parent; // HLNode parent
	node->g_val = parent->g_val;
	node->makespan = parent->makespan;
	node->depth = parent->depth + 1;

	// Find all agents that are conflicting at the parent node,
	// by examining the constraints imposed in reverse.
	auto agents = getInvalidAgents(node->constraints);
	assert(!agents.empty());

	// Find a new path for those individuals at least as long as the previous one.
	for (auto agent : agents)
	{
		int lowerbound = (int)paths[agent]->size() - 1;
		if (!findPathForSingleAgent(node, agent, lowerbound))
		{
			// there does not exist a new path, this node is a deadend.
			runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
			return false;
		}
	}

	// Find new conflicts
	findConflicts(*node);

	// Compute heuristic cost-to-go of this node
	heuristic_helper.computeQuickHeuristics(*node);

	// Timing
	runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
	return true;
}



//=============================================================================
// return agents that violates the constraints
set<int> CBS::getInvalidAgents(const list<Constraint>& constraints)
{
	set<int> agents;
	int agent, x, y, t;
	constraint_type type;
	assert(!constraints.empty());
	tie(agent, x, y, t, type) = constraints.front();

	// examine constraint by each type
	if (type == constraint_type::LEQLENGTH)
	{
		// path of agent should be of lenth at most t,
		// and any other agent cannot be at loc at or after t
		assert(constraints.size() == 1);

		//going through all agents, find a conflicting agent at the target
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == agent)
				continue;
			for (int i = t; i < (int)paths[ag]->size(); i++)
			{
				if (paths[ag]->at(i).location == x)
				{
					agents.insert(ag);
					break;
				}
			}
		}
	}
	else if (type == constraint_type::POSITIVE_VERTEX)
	{
		// find a conflicting agnet
		assert(constraints.size() == 1);
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == agent)
				continue;
			if (getAgentLocation(ag, t) == x)
			{
				agents.insert(ag);
			}
		}
	}
	else if (type == constraint_type::POSITIVE_EDGE)
	{
		assert(constraints.size() == 1);
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == agent)
				continue;
			int curr = getAgentLocation(ag, t);
			int prev = getAgentLocation(ag, t - 1);
			if (prev == x || curr == y ||
				(prev == y && curr == x))
			{
				agents.insert(ag);
			}
		}

	}
	else
	{
		// at least this agent is in currently conflict
		agents.insert(agent);
	}
	return agents;
}


//=============================================================================
bool CBS::terminate(HLNode* curr)
{
	if (cost_lowerbound >= cost_upperbound)
	{
		solution_cost = cost_lowerbound;
		solution_found = false;
        if (screen > 0) // 1 or 2
            printResults();
		return true;
	}
	runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
	if (curr->conflicts.empty() && curr->unknownConf.empty()) //no conflicts
	{// found a solution
		solution_found = true;
		goal_node = curr;
		solution_cost = goal_node->getFHatVal() - goal_node->cost_to_go;
		if (!validateSolution())
		{
			cout << "Solution invalid!!!" << endl;
			printPaths();
			exit(-1);
		}
		if (screen > 0) // 1 or 2
			printResults();
		return true;
	}
	if (runtime > time_limit || num_HL_expanded > node_limit)
	{   // time/node out
		solution_cost = -1;
		solution_found = false;
        if (screen > 0) // 1 or 2
            printResults();
		return true;
	}
	return false;
}

//=============================================================================
CBSNode* CBS::selectNode()
{
	CBSNode* curr = nullptr;
	switch (solver_type)
	{
		// Regular Astar
	  case high_level_solver_type::ASTAR:
	      cost_lowerbound = max(cost_lowerbound, cleanup_list.top()->getFVal());
	      curr = cleanup_list.top();
	      curr->chosen_from = "cleanup";
	      /*curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
	      curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
	      curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;*/
	      cleanup_list.pop();
	      break;
		// Focal Search
	  case high_level_solver_type::ASTAREPS:
	      // update the focal list if necessary
	      if (cleanup_list.top()->getFVal() > cost_lowerbound)
	      {
	          if (screen == 3)
	          {
	              cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << cleanup_list.size() << " to |FOCAL|=";
	          }
	          double old_focal_list_threshold = suboptimality * cost_lowerbound;
	          cost_lowerbound = max(cost_lowerbound, cleanup_list.top()->getFVal());
	          double new_focal_list_threshold = suboptimality * cost_lowerbound;
	          for (auto n : cleanup_list)
	          {
	              if (n->getFVal() > old_focal_list_threshold && n->getFVal() <= new_focal_list_threshold)
	                  n->focal_handle = focal_list.push(n);
	          }
	          if (screen == 3)
	          {
	              cout << focal_list.size() << endl;
	          }
	      }

	      // choose best d in the focal list
	      curr = focal_list.top();
	      curr->chosen_from = "focal";
	      /*curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
	      curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
	      curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
	      curr->f_of_best_in_focal = focal_list.top()->getFVal();
	      curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
	      curr->d_of_best_in_focal = focal_list.top()->distance_to_go;*/
	      focal_list.pop();
	      cleanup_list.erase(curr->cleanup_handle);
	      break;

		// Explicit Estimate Search
	  case high_level_solver_type::EES:
	      // update the focal list if necessary
	      if (open_list.top()->getFHatVal() > inadmissible_cost_lowerbound)
	      {
	          if (screen == 3)
	          {
	              cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
	          }
	          double old_focal_list_threshold = suboptimality * inadmissible_cost_lowerbound;
	          inadmissible_cost_lowerbound = open_list.top()->getFHatVal();
	          double new_focal_list_threshold = suboptimality * inadmissible_cost_lowerbound;
	          for (auto n : open_list)
	          {
	              if (n->getFHatVal() > old_focal_list_threshold &&
	                  n->getFHatVal() <= new_focal_list_threshold)
	                  n->focal_handle = focal_list.push(n);
	          }
	          if (screen == 3)
	          {
	              cout << focal_list.size() << endl;
	          }
	      }

	      // choose the best node
	      cost_lowerbound = max(cleanup_list.top()->getFVal(), cost_lowerbound);
	      if (focal_list.top()->getFVal() <= suboptimality * cost_lowerbound)
	      { // return best d
	          curr = focal_list.top();
	          /* for debug */
	          curr->chosen_from = "focal";
	          // curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
	          // curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
	          // curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
	          // curr->f_of_best_in_open = open_list.top()->getFVal();
	          // curr->f_hat_of_best_in_open = open_list.top()->getFHatVal();
	          // curr->d_of_best_in_open = open_list.top()->distance_to_go;
	          // curr->f_of_best_in_focal = focal_list.top()->getFVal();
	          // curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
	          // curr->d_of_best_in_focal = focal_list.top()->distance_to_go;
	          /* end for debug */
	          focal_list.pop();
	          cleanup_list.erase(curr->cleanup_handle);
	          open_list.erase(curr->open_handle);
	      }
	      else if (open_list.top()->getFVal() <= suboptimality * cost_lowerbound)
	      { // return best f_hat
	          curr = open_list.top();
	          /* for debug */
	          curr->chosen_from = "open";
	          // curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
	          // curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
	          // curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
	          // curr->f_of_best_in_open = open_list.top()->getFVal();
	          // curr->f_hat_of_best_in_open = open_list.top()->getFHatVal();
	          // curr->d_of_best_in_open = open_list.top()->distance_to_go;
	          // curr->f_of_best_in_focal = focal_list.top()->getFVal();
	          // curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
	          // curr->d_of_best_in_focal = focal_list.top()->distance_to_go;
	          /* end for debug */
	          open_list.pop();
	          cleanup_list.erase(curr->cleanup_handle);
	          focal_list.erase(curr->focal_handle);
	      }
	      else
	      { // return best f
	          curr = cleanup_list.top();
	          /* for debug */
	          curr->chosen_from = "cleanup";
	          // curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
	          // curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
	          // curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
	          // curr->f_of_best_in_open = open_list.top()->getFVal();
	          // curr->f_hat_of_best_in_open = open_list.top()->getFHatVal();
	          // curr->d_of_best_in_open = open_list.top()->distance_to_go;
	          // curr->f_of_best_in_focal = focal_list.top()->getFVal();
	          // curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
	          // curr->d_of_best_in_focal = focal_list.top()->distance_to_go;
	          /* end for debug */
	          cleanup_list.pop();
	          open_list.erase(curr->open_handle);
	          if (curr->getFHatVal() <= suboptimality * inadmissible_cost_lowerbound)
	              focal_list.erase(curr->focal_handle);
	      }
	      break;
	  case high_level_solver_type::NEW:

          // choose best d in the focal list
          curr = focal_list.top();
          /*curr->chosen_from = "focal";
          curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
          curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
          curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
          curr->f_of_best_in_focal = focal_list.top()->getFVal();
          curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
          curr->d_of_best_in_focal = focal_list.top()->distance_to_go;*/
          focal_list.pop();
          cleanup_list.erase(curr->cleanup_handle);
	      break;
	} // end switch solver type

	// takes the paths_found_initially and UPDATE all constrained paths found
	// for agents from curr to dummy_start (and lower-bounds)
	updatePaths(curr);

	if (screen > 1)
		cout << endl << "Pop " << *curr << endl;
	return curr;
}

//=============================================================================
// Bottom-up updates on paths (so that paths have the paths of curr node).
// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
inline void CBS::updatePaths(CBSNode* curr)
{
	for (int i = 0; i < num_of_agents; i++)
		paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_agents, false);  // initialized for false

	while (curr != nullptr)
	{
		for (auto & path : curr->paths)
		{
			// path is a pair of <agent_id, path>
			// is this agent updated?
			if (!updated[path.first])
			{
				// No, let's update
				// paths is a vector of num_of_agents length, each entry is a pointer path
				paths[path.first] = &(path.second);
				updated[path.first] = true;
			}
		}
		curr = curr->parent;
	}
}

//=============================================================================
inline bool CBS::reinsertNode(CBSNode* node)
{
    node->cleanup_handle = cleanup_list.push(node);
	switch (solver_type)
	{
	case high_level_solver_type::ASTAR:
		break;
	case high_level_solver_type::ASTAREPS:
		if (node->getFVal() <= suboptimality * cost_lowerbound)
			node->focal_handle = focal_list.push(node);
		break;
	case high_level_solver_type::NEW:
        // if (node->getFHatVal() <= suboptimality * cost_lowerbound)
            node->focal_handle = focal_list.push(node);
		break;
	case high_level_solver_type::EES:
		node->open_handle = open_list.push(node);
		if (node->getFHatVal() <= suboptimality * inadmissible_cost_lowerbound)
			node->focal_handle = focal_list.push(node);
		break;
	}
	if (screen == 2)
	{
		cout << "	Reinsert " << *node << endl;
	}
	return true;
}
//=============================================================================
// push high-level CTnode to the OPEN
inline void CBS::pushNode(CBSNode* node)
{
	num_HL_generated++;
	node->time_generated = num_HL_generated;
	// update handles
    node->cleanup_handle = cleanup_list.push(node);
	switch (solver_type)
	{
		case high_level_solver_type::ASTAR:
			break;
		case high_level_solver_type::ASTAREPS: // cleanup_list is called open_list in ECBS
			if (node->getFVal()<= suboptimality * cost_lowerbound)
				node->focal_handle = focal_list.push(node);
			break;
		case high_level_solver_type::NEW:
            // if (node->getFHatVal() <= suboptimality * cost_lowerbound)
                node->focal_handle = focal_list.push(node);
			break;
		case high_level_solver_type::EES:
			node->open_handle = open_list.push(node);
			if (node->getFHatVal() <= suboptimality * inadmissible_cost_lowerbound)
				node->focal_handle = focal_list.push(node);
			break;
	}
	allNodes_table.push_back(node);
}



//=============================================================================
// This is a wrapper to call search_engine of individual agent
bool CBS::findPathForSingleAgent(CBSNode*  node, int ag, int lowerbound)
{
	clock_t t = clock();
	// build reservation table
	// CAT cat(node->makespan + 1);  // initialized to false
	// updateReservationTable(cat, ag, *node);

	// find a path, and report the results
	Path new_path = search_engines[ag]->findOptimalPath(*node, initial_constraints[ag], paths, ag, lowerbound);
	num_LL_expanded += search_engines[ag]->num_expanded;
	num_LL_generated += search_engines[ag]->num_generated;
	num_LL_conflicts += search_engines[ag]->num_conflicts;
	runtime_build_CT += search_engines[ag]->runtime_build_CT;
	runtime_build_CAT += search_engines[ag]->runtime_build_CAT;
	runtime_path_finding += (double)(clock() - t) / CLOCKS_PER_SEC;

	// Did we find a new path?
	if (!new_path.empty())
	{
		// Yes, are you sure?
		assert(!isSamePath(*paths[ag], new_path));

		// Let's save this new path, and update the node values
		node->paths.emplace_back(ag, new_path);
		node->g_val = node->g_val - (int)paths[ag]->size() + (int)new_path.size();
		paths[ag] = &node->paths.back().second;
		node->makespan = max(node->makespan, new_path.size() - 1);

		// Yes, we found it and updated the node values, return true
		return true;
	}
	else
	{
		// No, there was no new path.
		return false;
	}
}


//=============================================================================
//generate random permuattion of agent indices
vector<int> CBS::shuffleAgents() const
{
	vector<int> agents(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
	{
		agents[i] = i;
	}

	if (randomRoot)
	{
		std::random_device rd;
		std::mt19937 g(rd());
		std::shuffle(std::begin(agents), std::end(agents), g);
	}
	return agents;
}


/*inline void CBS::releaseOpenListNodes()
{
	while (!open_list.empty())
	{
		CBSNode* curr = open_list.top();
		open_list.pop();
		delete curr;
	}
}*/


//=============================================================================
bool CBS::validateSolution() const
{
	// check whether the solution cost is within the bound
	if (solution_cost > cost_lowerbound * suboptimality)
    {
	    cout << "Solution cost exceeds the sub-optimality bound!" << endl;
        return false;
    }

	// check whether the paths are feasible
	size_t soc = 0;
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		soc += paths[a1]->size() - 1;
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
			size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
			for (size_t timestep = 0; timestep < min_path_length; timestep++)
			{
				int loc1 = paths[a1]->at(timestep).location;
				int loc2 = paths[a2]->at(timestep).location;
				if (loc1 == loc2)
				{
					cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
					return false;
				}
				else if (timestep < min_path_length - 1
					&& loc1 == paths[a2]->at(timestep + 1).location
					&& loc2 == paths[a1]->at(timestep + 1).location)
				{
					cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
						loc1 << "-->" << loc2 << ") at timestep " << timestep << endl;
					return false;
				}
			}
			if (paths[a1]->size() != paths[a2]->size())
			{
				int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
				int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
				int loc1 = paths[a1_]->back().location;
				for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
				{
					int loc2 = paths[a2_]->at(timestep).location;
					if (loc1 == loc2)
					{
						cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
						return false; // It's at least a semi conflict
					}
				}
			}
		}
	}
	if ((int)soc != solution_cost)
	{
		cout << "The solution cost is wrong!" << endl;
		return false;
	}
	return true;
}

//=============================================================================
inline int CBS::getAgentLocation(int agent_id, size_t timestep) const
{
	size_t t = max(min(timestep, paths[agent_id]->size() - 1), (size_t)0);
	return paths[agent_id]->at(t).location;
}



//=============================================================================
void CBS::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
			paths[i]->size() - 1 << "): ";
		for (const auto & t : *paths[i])
			cout << t.location << "->";
		cout << endl;
	}
}


//=============================================================================
void CBS::printResults() const
{
	if (solution_cost >= 0) // solved
		cout << "Succeed,";
	else if (solution_cost == -1) // time_out
		cout << "Timeout,";
	else if (solution_cost == -2) // no solution
		cout << "No solutions,";
	else if (solution_cost == -3) // nodes out
		cout << "Nodesout,";

	cout << solution_cost << "," << runtime << "," <<
		num_HL_expanded << "," << num_LL_expanded << "," << // HL_num_generated << "," << LL_num_generated << "," <<
		cost_lowerbound << "," << dummy_start->g_val << "," << dummy_start->g_val + dummy_start->h_val << "," <<
		endl;
    /*if (solution_cost >= 0) // solved
    {
        cout << "fhat = [";
        auto curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->getFHatVal() << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "hhat = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->cost_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "d = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->distance_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "soc = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->getFHatVal() - curr->cost_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
    }*/
}

//=============================================================================
void CBS::saveResults(const string &fileName, const string &instanceName) const
{
	std::ifstream infile(fileName);
	bool exist = infile.good();
	infile.close();
	if (!exist)
	{
		ofstream addHeads(fileName);
		addHeads << "runtime,#high-level expanded,#high-level generated,#low-level expanded,#low-level generated," <<
			"solution cost,min f value,root g value, root f value," <<
			"#adopt bypasses," <<
			"cardinal conflicts," <<
			"standard conflicts,rectangle conflicts,corridor conflicts,target conflicts,mutex conflicts," <<
			"chosen from cleanup,chosen from open,chosen from focal," <<
			"#solve MVCs,#merge MDDs,#solve 2 agents,#memoization," <<
			"cost error,distance error," <<
			"runtime of building heuristic graph,runtime of solving MVC," <<
			"runtime of detecting conflicts," <<
			"runtime of rectangle conflicts,runtime of corridor conflicts,runtime of mutex conflicts," <<
			"runtime of building MDDs,runtime of building constraint tables,runtime of building CATs," <<
			"runtime of path finding,runtime of generating child nodes," <<
			"preprocessing runtime,solver name,instance name" << endl;
		addHeads.close();
	}
	ofstream stats(fileName, std::ios::app);
	stats << runtime << "," <<
		num_HL_expanded << "," << num_HL_generated << "," <<
		num_LL_expanded << "," << num_LL_generated << "," <<

		solution_cost << "," << cost_lowerbound << "," << dummy_start->g_val << "," <<
		dummy_start->g_val + dummy_start->h_val << "," <<

		num_adopt_bypass << "," <<
		num_cardinal_conflicts << "," <<
		num_standard_conflicts << "," << num_rectangle_conflicts << "," << num_corridor_conflicts << "," << num_target_conflicts << "," << num_mutex_conflicts << "," <<

		num_cleanup << "," << num_open << "," << num_focal << "," <<

		heuristic_helper.num_solve_MVC << "," <<
		heuristic_helper.num_merge_MDDs << "," <<
		heuristic_helper.num_solve_2agent_problems << "," <<
		heuristic_helper.num_memoization << "," <<
		heuristic_helper.getCostError() << "," << heuristic_helper.getDistanceError() << "," <<
		heuristic_helper.runtime_build_dependency_graph << "," <<
		heuristic_helper.runtime_solve_MVC << "," <<



		runtime_detect_conflicts << "," <<
		rectangle_helper.accumulated_runtime << "," << corridor_helper.accumulated_runtime << "," << mutex_helper.accumulated_runtime << "," <<
		mdd_helper.accumulated_runtime << "," << runtime_build_CT << "," << runtime_build_CAT << "," <<
		runtime_path_finding << "," << runtime_generate_child << "," <<

		runtime_preprocessing << "," << getSolverName() << "," << instanceName << endl;
	stats.close();
}

//=============================================================================
void CBS::saveSummary(const std::string& file_name,
                                   const std::string& instance_name) const {
  std::ifstream infile(file_name);
  bool exist = infile.good();
  infile.close();
  if (!exist) {
    std::ofstream addHeads(file_name);
    addHeads << "solver name, runtime, solution cost, suboptimality,"
             << "#high-level expanded, #high-level generated,"
             << "#low-level expanded, #low-level generated,"
             << "#adopt bypasses,"
             << "runtime of detecting conflicts,"
             << "runtime of building constraint tables,"
             << "runtime of building confilct tables,"
             << "runtime of path finding,"
             << "runtime of generating child high-level nodes,"
             << "preprocessing runtime,"
             << "number of conflicts in low-level paths,"
             << "instance name" << std::endl;
    addHeads.close();
  }
  std::ofstream stats(file_name, std::ios::app);
  stats << getSolverName() << "," << runtime << "," << solution_cost << "," << suboptimality << ","
        << num_HL_expanded << "," << num_HL_generated << "," << num_LL_expanded << ","
        << num_LL_generated << "," << num_adopt_bypass << "," << runtime_detect_conflicts << ","
        << runtime_build_CT << "," << runtime_build_CAT << ","
        << runtime_path_finding << "," << runtime_generate_child << "," << runtime_preprocessing
        << "," << num_LL_conflicts << "," << instance_name << std::endl;

  stats.close();
}

//=============================================================================
void CBS::saveStats(const string &fileName, const string &instanceName)
{
	/*cout << "writing logs..." << endl;
	ofstream stats(fileName+ ".txt", std::ios::app);
	stats << instanceName << endl;
	stats << "agent 1,agent 2,node id,#expanded nodes, h" << endl;
	for (auto ins : heuristic_helper.sub_instances)
	{
			stats << get<0>(ins) << "," << get<1>(ins) << "," << get<2>(ins)->time_generated << "," << get<3>(ins) << "," << get<4>(ins) << endl;
	}
	stats.close();*/
	/*if (solution_found)
	{
		std::ifstream infile(fileName + ".heuristic");
		bool exist = infile.good();
		infile.close();
		if (!exist)
		{
			ofstream addHeads(fileName + ".heuristic");
			addHeads << "h*,solution depth,WDG,greedy WDG,DG,MVC on all conflicts,#conflicts,#CT nodes" << endl;
			addHeads.close();
		}
		ofstream out(fileName + ".heuristic", std::ios::app);
		out << solution_cost - dummy_start->g_val << "," // h*
			<< goal_node->depth << "," // depth
			<< dummy_start->h_val << ","; // WDG
		updatePaths(dummy_start);
		findConflicts(*dummy_start);
		classifyConflicts(*dummy_start);
		// int greedy = heuristic_helper.greedyWDG(*dummy_start, time_limit);
		int greedy = 0;
		heuristic_helper.type = heuristics_type::DG;
		int dg = heuristic_helper.computeInformedHeuristics(*dummy_start, time_limit);
		// int mvc = heuristic_helper.MVConAllConflicts(*dummy_start);
		int mvc = 0;
		out << greedy << "," << dg << "," << mvc << "," << dummy_start->conflicts.size() << "," <<
			dummy_start->distance_to_go << endl;
		out.close();
	}*/
}

//=============================================================================
void CBS::saveCT(const string &fileName) const // write the CT to a file
{
	// Write the tree graph in dot language to a file
	{
		std::ofstream output;
		output.open(fileName + ".tree", std::ios::out);
		output << "digraph G {" << endl;
		output << "size = \"5,5\";" << endl;
		output << "center = true;" << endl;
		set<HLNode*> path_to_goal;
		auto curr = goal_node;
		while (curr != nullptr)
		{
			path_to_goal.insert(curr);
			curr = curr->parent;
		}
		for (const auto& node : allNodes_table)
		{
			output << node->time_generated << " [label=\"g=" << node->g_val << ", h=" << node->h_val
				<< "\nd=" << node->distance_to_go << ", h^=" << node->getFHatVal() - node->g_val;
			if (node->time_expanded > 0) // the node has been expanded
			{
				output << "\n #" << node->time_expanded << " from " << node->chosen_from;
				output << "\", color=";
				if (node->chosen_from == "focal")
					output << "blue]" << endl;
				else if (node->chosen_from == "cleanup")
					output << "green]" << endl;
				else if (node->chosen_from == "open")
					output << "orange]" << endl;
			}
			else
			{
				output << "\"]" << endl;
			}

			if (node == dummy_start)
				continue;
			if (path_to_goal.find(node) == path_to_goal.end())
			{
				output << node->parent->time_generated << " -> " << node->time_generated << endl;
			}
			else
			{
				output << node->parent->time_generated << " -> " << node->time_generated << " [color=red]" << endl;
			}
		}
		output << "}" << endl;
		output.close();
	}

	// Write the stats of the tree to a CSV file
	{
		std::ofstream output;
		output.open(fileName + "-tree.csv", std::ios::out);
		// header
		output << "time generated,g value,h value,h^ value,d value,depth,time expanded,chosen from,h computed,"
			<< "f of best in cleanup,f^ of best in cleanup,d of best in cleanup,"
			<< "f of best in open,f^ of best in open,d of best in open,"
			<< "f of best in focal,f^ of best in focal,d of best in focal,"
			<< "praent,goal node" << endl;
		for (auto& node : allNodes_table)
		{
			output << node->time_generated << ","
				<< node->g_val << "," << node->h_val << "," << node->getFHatVal() - node->g_val << "," <<  node->distance_to_go << ","
				<< node->depth << ","
				<< node->time_expanded << "," << node->chosen_from << "," << node->h_computed << ","
				<< node->f_of_best_in_cleanup << "," << node->f_hat_of_best_in_cleanup << "," << node->d_of_best_in_cleanup << ","
				<< node->f_of_best_in_open << "," << node->f_hat_of_best_in_open << "," << node->d_of_best_in_open << ","
				<< node->f_of_best_in_focal << "," << node->f_hat_of_best_in_focal << "," << node->d_of_best_in_focal << ",";
			if (node->parent == nullptr)
				output << "0,";
			else
				output << node->parent->time_generated << ",";
			if (node == goal_node)
				output << "1" << endl;
			else
				output << "0" << endl;
		}
		output.close();
	}

}

//=============================================================================
void CBS::savePaths(const string &fileName) const
{
    std::ofstream output;
    output.open(fileName, std::ios::out);
    for (int i = 0; i < num_of_agents; i++)
    {
        output << "Agent " << i << ": ";
        for (const auto & t : *paths[i])
            output << "(" << search_engines[0]->instance.getRowCoordinate(t.location)
                   << "," << search_engines[0]->instance.getColCoordinate(t.location) << ")->";
        output << endl;
    }
    output.close();
}

//=============================================================================
void CBS::savePathsTable(const string &fileName) const
{
    std::ofstream output_x;
    output_x.open(fileName+"_x", std::ios::out);

		std::ofstream output_y;
		output_y.open(fileName+"_y", std::ios::out);

		// Going through each agent, mark its location 'makespan_' long.
		for (int i = 0; i < num_of_agents; i++)
		{
			int pathLength=0;
			for (const auto & t : *paths[i])
			{
				pathLength++;
				output_x << search_engines[0]->instance.getRowCoordinate(t.location)<< ",";
				output_y << search_engines[0]->instance.getColCoordinate(t.location)<< ",";
			}
			auto goalLoc = (*paths[i]).back();
			for (pathLength; pathLength<makespan_+1; pathLength++)
			{
				output_x << search_engines[0]->instance.getRowCoordinate(goalLoc.location)<< ",";
				output_y << search_engines[0]->instance.getColCoordinate(goalLoc.location)<< ",";
			}
			output_x << endl;
			output_y << endl;
		}
    output_x.close();
		output_y.close();

}


//=============================================================================
void CBS::printConflicts(const HLNode &curr)
{
	for (const auto& conflict : curr.conflicts)
	{
		cout << *conflict << endl;
	}
	for (const auto& conflict : curr.unknownConf)
	{
		cout << *conflict << endl;
	}
}

//=============================================================================
string CBS::getSolverName() const
{
	string name;
	if (disjoint_splitting)
		name += "Disjoint ";
	switch (heuristic_helper.type)
	{
	case heuristics_type::ZERO:
		if (PC)
			name += "+PC";
		break;
	case heuristics_type::CG:
		name += "+CG";
		break;
	case heuristics_type::DG:
		name += "+DG";
		break;
	case heuristics_type::WDG:
		name += "+WDG";
		break;
	default:
		break;
	}
	if (rectangle_reasoning)
		name += "+R";
	if (corridor_reasoning)
		name += "+C";
	if (target_reasoning)
		name += "+T";
	if (mutex_reasoning)
		name += "+MP";
	if (bypass)
		name += "+BP";
	name += " with " + search_engines[0]->getName();
	return name;
}
