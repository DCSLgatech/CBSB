#include "SpaceTimeCOAStar.h"


void SpaceTimeCOAStar::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
	num_conflicts = goal->num_of_conflicts;
	path.reserve(goal->g_val + 1);
	const LLNode* curr = goal;
	while (curr != nullptr)
	{
		path.emplace_back(curr->location);
		curr = curr->parent;
	}
    std::reverse(path.begin(),path.end());
}


Path SpaceTimeCOAStar::findOptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
	const vector<Path*>& paths, int agent, int lowerbound)
{
	return findSuboptimalPath(node, initial_constraints, paths, agent, lowerbound, 1).first;
}


// find path by time-space A* search
// Returns a bounded-suboptimal path that satisfies the constraints of the give node  while
// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
// lowerbound is an underestimation of the length of the path in order to speed up the search.
pair<Path, int> SpaceTimeCOAStar::findSuboptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
	const vector<Path*>& paths, int agent, int lowerbound, double w)
{
	// this->w = w;
	Path path;
	num_expanded = 0;
	num_generated = 0;

	int budget = w;

	// build constraint table
	auto t = clock();
	constraint_table.init(initial_constraints); // copy initial constraints
	constraint_table.build(node, agent);
	runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;
	if (constraint_table.constrained(start_location, 0))
	{
		return {path, 0};
	}

	int holding_time = constraint_table.getHoldingTime(); // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
	t = clock();
	constraint_table.buildCAT(agent, paths, node.makespan + 1);
	runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;

  lowerbound =  max(holding_time, lowerbound);

	// generate start and add it to the OPEN & FOCAL list
	// (loc, g_val, h_val, parent, timestep, num_of_conflicts, in_openlist)
	auto start = new COAStarNode(start_location, 0, max(lowerbound, my_heuristic[start_location]), nullptr, 0, 0, false);

	num_generated++;
	start->open_handle = open_list.push(start);
	start->in_openlist = true;
	allNodes_table.insert(start);
	min_f_val = (int) start->getFVal();
	// lower_bound = int(w * min_f_val));

	while (!open_list.empty())
	{
		// Pop from focal
		auto* curr = popNode();
        assert(curr->location >= 0);

		min_f_val = (int) curr->getFVal();

		// check if the popped node is a goal
		if (curr->location == goal_location && // arrive at the goal location
			!curr->wait_at_goal && // not wait at the goal location (action is complete)
			curr->timestep >= holding_time) // the agent can hold the goal location afterward
		{
			// retrieve path by tracing parents
			updatePath(curr, path);
			break;
		}

		// if the agent must reach its goal before the length_max (target_constraint)
		if (curr->timestep >= constraint_table.length_max)
			continue;

		// Now find neighbors
		auto next_locations = instance.getNeighbors(curr->location);
		// Also add this current location, for wait action
		next_locations.emplace_back(curr->location);
		// For each neighbor + myself
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;
			// Is the current timestep the longest? Then everyone else finished moving.
			if (max((int)node.makespan, constraint_table.latest_timestep) + 1 < curr->timestep)
			{ // now everything is static, so switch to space A* where we always use the same timestep
				// Exclude wait action, since there is no point not moving.
				if (next_location == curr->location)
				{
					continue;
				}
				next_timestep--;
			}

			// skip if the next_location is constrained at the next_timestep
			// Check vertex or edge constraints
			if (constraint_table.constrained(next_location, next_timestep) ||
				constraint_table.constrained(curr->location, next_location, next_timestep))
				continue;

			// compute cost to next_id via curr node
			int next_g_val = curr->g_val + 1; // every action costs 1
			int next_h_val = max(lowerbound - next_g_val, my_heuristic[next_location]);

			// Can it possbibly be reaching the goal before the length_max?
			if (next_g_val + next_h_val > constraint_table.length_max) // No, skip it then.
				continue;

			// int next_internal_conflicts = (next_g_val + next_h_val > budget || next_timestep > budget)?
			// 															 curr->num_of_conflicts + 1 :
			// 															 curr->num_of_conflicts +
			// 																 constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);

			int next_internal_conflicts = curr->num_of_conflicts + constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);
			bool next_c_val = curr->c_val;
			if (!next_c_val) // still possibly be in class1
			{
				// next_c_val becomes true if either budget exceed, or conflict exists
				next_c_val = (next_g_val + next_h_val > budget) || (next_timestep > budget)
										 || next_internal_conflicts != 0;
			}

			// generate (maybe temporary) node, we will decide whether to put this in the queue or not
			auto next = new COAStarNode(next_location, next_g_val, next_h_val,
				curr, next_timestep, next_internal_conflicts, next_c_val, false);

			// mark waith_at_goal true, if the action is to wait at the goal (not done yet)
			if (next_location == goal_location && curr->location == goal_location)
				next->wait_at_goal = true;

			// try to retrieve it from the hash table if exists (hash is for key)
			auto it = allNodes_table.find(next);
			// if not found
			if (it == allNodes_table.end())
			{
				// put this node in the OPEN, and also in FOCAL if within the bound
				pushNode(next);
				allNodes_table.insert(next);
				continue;
			}
			// update existing node's if needed (only in the open_list)
			// Found in allNodes_table
			auto existing_next = *it;
			// if (existing_next->num_of_conflicts > next->num_of_conflicts &&
			// 		existing_next->getFVal() > next->getFVal()) // or it remains the same but there's fewer conflicts
			// if ((existing_next->c_val && !next->c_val) || // existing->class2 & next->class1
			// 		(existing_next->c_val == next->c_val && existing_next->num_of_conflicts > next->num_of_conflicts) )
			if ((existing_next->c_val && !next->c_val) || // existing->class2 & next->class1
					(existing_next->c_val == next->c_val && existing_next->getFVal() > next->getFVal()) )
			{
				if (!existing_next->in_openlist) // if its in the closed list (reopen)
				{
					existing_next->copy(*next);
					pushNode(existing_next);
				}
				else
				{
					// Okay, now update existing node with the newly evaluated node.
					existing_next->copy(*next);	// update existing node
					open_list.increase(existing_next->open_handle);  // increase because f-val improved

				}
			}
			// No, we already have a better path to the node.

			// Now we don't need this node, since it has been put in the queue.
			delete(next);
		}  // end for loop that generates successors
	}  // end while loop

	// We are done, let's clear the mess.
	releaseNodes();
	constraint_table.clear();
	return {path, min_f_val};
}


int SpaceTimeCOAStar::getTravelTime(int start, int end, const ConstraintTable& constraint_table, int upper_bound)
{
	int length = MAX_TIMESTEP;
	auto root = new COAStarNode(start, 0, compute_heuristic(start, end), nullptr, 0);
	root->open_handle = open_list.push(root);  // add root to heap
	allNodes_table.insert(root);       // add root to hash_table (nodes)
	COAStarNode* curr = nullptr;
	while (!open_list.empty())
	{
		curr = open_list.top(); open_list.pop();
		if (curr->location == end)
		{
			length = curr->g_val;
			break;
		}
		list<int> next_locations = instance.getNeighbors(curr->location);
		next_locations.emplace_back(curr->location);
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;
			int next_g_val = curr->g_val + 1;
			if (constraint_table.latest_timestep <= curr->timestep)
			{
				if (curr->location == next_location)
				{
					continue;
				}
				next_timestep--;
			}
			if (!constraint_table.constrained(next_location, next_timestep) &&
				!constraint_table.constrained(curr->location, next_location, next_timestep))
			{  // if that grid is not blocked
				int next_h_val = compute_heuristic(next_location, end);
				if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
					continue;
				auto next = new COAStarNode(next_location, next_g_val, next_h_val, nullptr, next_timestep);
				auto it = allNodes_table.find(next);
				if (it == allNodes_table.end())
				{  // add the newly generated node to heap and hash table
					next->open_handle = open_list.push(next);
					allNodes_table.insert(next);
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					auto existing_next = *it;
					if (existing_next->g_val > next_g_val)
					{
						existing_next->g_val = next_g_val;
						existing_next->timestep = next_timestep;
						open_list.increase(existing_next->open_handle);
					}
				}
			}
		}
	}
	releaseNodes();
	return length;
}

inline COAStarNode* SpaceTimeCOAStar::popNode()
{
	auto node = open_list.top(); open_list.pop();
	node->in_openlist = false;
	num_expanded++;
	return node;
}


inline void SpaceTimeCOAStar::pushNode(COAStarNode* node)
{
	node->open_handle = open_list.push(node);
	node->in_openlist = true;
	num_generated++;
}


void SpaceTimeCOAStar::releaseNodes()
{
	open_list.clear();

	for (auto node: allNodes_table)
		delete node;
	allNodes_table.clear();
}
