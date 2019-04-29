#pragma once

#include <core/Godot.hpp>
#include <core/PoolArrays.hpp>
#include <ProgressBar.hpp>
#include "../../Map.h"
#include <vector>
#include "../AStar/AStar.h"

using namespace godot;

class GreedySearch
{
public:
	GreedySearch();
	~GreedySearch();
	
	struct NodeItem
	{
		Vector2 node;
		int nrOfConnections = 0;
		NodeItem* connectedNode1 = nullptr;
		NodeItem* connectedNode2 = nullptr;
		int specialConnection = 0; // 0 = none      1 = start        2 = end

		NodeItem(Vector2 p_node)
		{
			node = p_node;
		}

		bool equals(NodeItem* other)
		{
			if (node == other->node)
				return true;
			return false;
		}

		bool operator==(const NodeItem& B)
		{
			if (node == B.node)
				return true;
			return false;
		}
	};

	struct PathItem 
	{
		int length;
		NodeItem* node1;
		NodeItem* node2;
		PoolVector2Array path;

		PathItem(int p_length, NodeItem* p_node1, NodeItem* p_node2, PoolVector2Array p_path)
		{
			length = p_length;
			node1 = p_node1;
			node2 = p_node2;
			path = p_path;
		}

		bool equals(PathItem& other)
		{
			if (length == other.length && node1->equals(other.node1) && node2->equals(other.node2))
				return true;
			return false;
		}

		bool operator==(const PathItem& B)
		{
			if (length == B.length && node1->equals(B.node1) && node2->equals(B.node2))
				return true;
			return false;
		}
		

	};




	static PoolVector2Array calculatePath(Map& map, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points);

private:
	static int pathLength(NodeItem* startNode);
};

