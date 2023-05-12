#pragma once
#include "CBS.h"

class CBSB : public CBS
{
public:
	CBSB(const Instance& instance, bool sipp, int screen) : CBS(instance, sipp, screen) {}

	// CBSBNode* dummy_start = nullptr;
	// CBSBNode* goal_node = nullptr;

	////////////////////////////////////////////////////////////////////////////////////////////
	// Runs the algorithm until the problem is solved or time is exhausted
	bool solve(double time_limit, int cost_lowerbound = 0, int cost_upperbound = MAX_COST);

private:

	int allowable_budget;
	
	pairing_heap< CBSNode*, compare<CBSNode::compare_node_by_b> > bcoa_list;
	pairing_heap< CBSNode*, compare<CBSNode::compare_node_by_d> > focal_list;

	void pushNode(CBSNode* node);
	CBSNode* selectNode();
	bool reinsertNode(CBSNode* node);

	bool generateChild(CBSNode* child, CBSNode* curr);
	bool generateRoot();
	bool findPathForSingleAgent(CBSNode*  node, int ag, int lower_bound = 0);
	void classifyConflicts(CBSNode &parent);
	void updatePaths(CBSNode* curr);

	void printPaths() const;
	string getSolverName() const;

};
