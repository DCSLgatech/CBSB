#include "CBSB.h"

bool CBSB::solve(double _time_limit, int _cost_lowerbound, int _cost_upperbound)
{
	this->cost_lowerbound = MAX_COST - 1;
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
	while (!bcoa_list.empty() && !solution_found)
	{
		// pop front of the OPEN, choose CT node for expansion
		auto curr = selectNode();

		// Are we done?
		if (terminate(curr))
			return solution_found;

		// No, let's continue, but do we want to prioritize conflicts?
		if (PC) // priortize conflicts
			classifyConflicts(*curr);

		// Expand the node
		num_HL_expanded++;
		curr->time_expanded = num_HL_expanded;

		// Split the node if no bypass is found
		// While bypass exists, keep bypassing the node.
		bool foundBypass = true;
		while (foundBypass)
		{
			// Are we still good to go?
			if (terminate(curr))
				return solution_found;

			// Assume no more bypass for now.
			foundBypass = false;

			// Yes, let's branch out.
			CBSNode *child[2] = {new CBSNode(), new CBSNode()};

			// Let's pick a conflict to resolve, if not PC, then this is at random
			curr->conflict = chooseConflict(*curr);

			// Let's split the conflict to assign constraints to each child
			addConstraints(curr, child[0], child[1]);

			if (screen > 1)
				cout << "	Expand " << *curr << endl
					 << "	on " << *(curr->conflict) << endl;

			// Can we generate child? i.e., does a new path exist with additional constraints?
			// If not, then that child is a deadend.
			bool solved[2] = {false, false};
			vector<vector<PathEntry> *> copy(paths);

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
				// But if the first child was a deadend, then let's not bypass it.
				else if (i == 1 && !solved[0])
					continue;
				// Do we want to bypass the conflict?
				else if (bypass && child[i]->g_val <= allowable_budget && child[i]->distance_to_go < curr->distance_to_go) // Bypass1
				{
					foundBypass = true;
					for (const auto &path : child[i]->paths)
					{
						if (curr->budgets[path.first] > curr->budgets[path.first])
						{
							foundBypass = false;
							break;
						}
					}
					// Yes, we can bypass, and let's do it.
					if (foundBypass)
					{
						num_adopt_bypass++;

						// Adopt this child values to the current node
						curr->g_val = child[i]->g_val;
						curr->distance_to_go = child[i]->distance_to_go;
						curr->conflicts = child[i]->conflicts;
						curr->unknownConf = child[i]->unknownConf;
						curr->conflict = nullptr;
						curr->makespan = child[i]->makespan;
						for (const auto &path : child[i]->paths) // update paths
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
					}

				} // end if bypassing child found
				  // No we don't bypass.
			}	  // end for each child

			// Have we found a bypass?
			if (foundBypass)
			{
				// Yes, then delete all child
				for (auto &i : child)
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
				case conflict_type::TARGET:
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
	}	  // end of while loop
	return solution_found;
}

//=============================================================================
bool CBSB::generateRoot()
{
	auto root = new CBSNode();
	root->g_val = 0;
	paths.resize(num_of_agents, nullptr);

	mdd_helper.init(num_of_agents);
	heuristic_helper.init();

	root->budgets.resize(num_of_agents);

	// initialize paths_found_initially
	if (paths_found_initially.empty())
	{
		paths_found_initially.resize(num_of_agents);

		// generate random permuattion of agent indices
		vector<int> agents(num_of_agents);
		for (int i = 0; i < num_of_agents; i++)
		{
			agents[i] = i;
		}

		if (randomRoot)
		{
			agents = shuffleAgents();
		}

		// Now solve shortest path problem for each agent
		for (auto i : agents)
		{
			// Find an initial budget based on admissible heuristic cost-to-go
			root->budgets[i] = suboptimality * search_engines[i]->my_heuristic[search_engines[i]->start_location];

			// Find initial solution with a given budget
			paths_found_initially[i] = search_engines[i]->findSuboptimalPath(*root, initial_constraints[i], paths, i, 0, root->budgets[i]).first;

			// Update the budget value
			if (root->budgets[i] < paths_found_initially[i].size() - 1)
				root->budgets[i] = (int)(suboptimality * (paths_found_initially[i].size() - 1));

			// root->budgets[i] = max(root->budgets[i], (int) (suboptimality*(paths_found_initially[i].size()-1)) );

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
			root->g_val += (int)paths_found_initially[i].size() - 1;
		}
	}

	// mark this root
	root->h_val = 0;
	root->depth = 0;

	// initialize b_val
	int b_val = 0;
	for (auto &b : root->budgets)
		b_val += b;
	root->b_val = b_val; // sum up the budgets

	allowable_budget = b_val;

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
// Replan for newly constrained agents, then find new conflicts and compute heuristic
bool CBSB::generateChild(CBSNode *node, CBSNode *parent)
{
	num_HL_generated++;
	node->time_generated = num_HL_generated;

	clock_t t1 = clock();

	// Assign the parent and inherit the node values
	node->parent = parent;		   // CBSNode parent
	node->HLNode::parent = parent; // HLNode parent
	node->g_val = parent->g_val;
	node->makespan = parent->makespan;
	node->depth = parent->depth + 1;

	// inherit budget
	node->b_val = parent->b_val;	 // cumulative budget
	node->budgets = parent->budgets; // individual budgets

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

	// Compute c_val
	// if (node->distance_to_go > 0 || node->getFVal() > budget) // we have conflict or exceeding budget
	// 	node->c_val = true;

	// Timing
	runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
	return true;
}

//=============================================================================
inline void CBSB::pushNode(CBSNode *node)
{
	// num_HL_generated++;
	// node->time_generated = num_HL_generated;
	// update handles
	node->bcoa_handle = bcoa_list.push(node);

	if (node->getFVal() <= allowable_budget)
		node->focal_handle = focal_list.push(node);

	allNodes_table.push_back(node);
}

//=============================================================================
// This is a wrapper to call search_engine of individual agent
bool CBSB::findPathForSingleAgent(CBSNode *node, int ag, int lowerbound)
{
	clock_t t = clock();
	// build reservation table
	// CAT cat(node->makespan + 1);  // initialized to false
	// updateReservationTable(cat, ag, *node);

	// find a path, and report the results
	Path new_path = search_engines[ag]->findSuboptimalPath(*node, initial_constraints[ag], paths, ag, lowerbound, node->budgets[ag]).first;
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

		if (node->budgets[ag] < new_path.size() - 1)
		{
			node->b_val = node->b_val - node->budgets[ag];					  // subtract old budget
			node->budgets[ag] = (int)(suboptimality * (new_path.size() - 1)); // compute new budget
			node->b_val = node->b_val + node->budgets[ag];					  // add new budget
		}

		// node->b_val =	node->b_val - node->budgets[ag]; // subtract old budget
		// node->budgets[ag] = max(node->budgets[ag], (int) (suboptimality*(new_path.size()-1)) ); // compute new budget
		// node->b_val = node->b_val + node->budgets[ag]; // add new budget

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
inline bool CBSB::reinsertNode(CBSNode *node)
{

	// switch (solver_type)
	// {
	// case high_level_solver_type::ASTAREPS:
	//       if (node->sum_of_costs <= suboptimality * cost_lowerbound)
	//           return false;
	//       node->bcoa_handle = bcoa_list.push(node);
	//       break;
	// case high_level_solver_type::NEW:
	//       if (node->getFHatVal() <= suboptimality * cost_lowerbound)
	//           return false;
	//       node->bcoa_handle = bcoa_list.push(node);
	// 	break;
	// case high_level_solver_type::EES:
	//       node->bcoa_handle = bcoa_list.push(node);
	// 			node->open_handle = open_list.push(node);
	// 			if (node->getFHatVal() <= suboptimality * inadmissible_cost_lowerbound)
	// 				node->focal_handle = focal_list.push(node);
	// 	break;
	// default:
	// 	break;
	// }
	node->bcoa_handle = bcoa_list.push(node);
	if (screen == 2)
	{
		cout << "	Reinsert " << *node << endl;
	}
	return true;
}

//=============================================================================
CBSNode *CBSB::selectNode()
{
	CBSNode *curr = nullptr;

	// update the focal list if necessary
	// if (bcoa_list.top()->b_val != allowable_budget)
	// {
	// 		allowable_budget = bcoa_list.top()->b_val;
	// 		focal_list.clear();
	// 		for (auto n : bcoa_list)
	// 		{
	// 			if (n->getFVal() <= allowable_budget)
	// 				n->focal_handle = focal_list.push(n);
	// 		}
	// }

	if (bcoa_list.top()->b_val > allowable_budget)
	{
		int old_allowable_budget = allowable_budget;
		allowable_budget = bcoa_list.top()->b_val;
		// focal_list.clear();
		for (auto n : bcoa_list)
		{
			if (n->getFVal() > old_allowable_budget && n->getFVal() <= allowable_budget)
				n->focal_handle = focal_list.push(n);
		}
	}

	// Choose the node from focal
	curr = focal_list.top();
	curr->chosen_from = "focal";
	focal_list.pop();
	bcoa_list.erase(curr->bcoa_handle);

	// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
	updatePaths(curr);

	if (screen > 1)
		cout << endl
			 << "Pop " << *curr << endl;
	return curr;
}

//=============================================================================
// Bottom-up updates on paths (so that paths have the paths of curr node).
// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
inline void CBSB::updatePaths(CBSNode *curr)
{
	for (int i = 0; i < num_of_agents; i++)
		paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_agents, false); // initialized for false

	while (curr != nullptr)
	{
		for (auto &path : curr->paths)
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
// Compute the conflict priority of unknownConfs, and also classifies
// mutex, corridor, rectangle conflicts
void CBSB::classifyConflicts(CBSNode &node)
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
			(int)paths[con->a2]->size() > timestep && // conflict happens before both agents reach their goal locations
			type == constraint_type::VERTEX)		  // vertex conflict
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
void CBSB::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" << paths[i]->size() - 1 << "): ";
		for (const auto &t : *paths[i])
			cout << t.location << "->";
		cout << endl;
	}
}
//=============================================================================
string CBSB::getSolverName() const
{
	string name;
	if (disjoint_splitting)
		name += "Disjoint ";

	name += "CBSB";

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
