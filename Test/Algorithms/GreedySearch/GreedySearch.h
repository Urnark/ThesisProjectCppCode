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
	//static PoolVector2Array calculatePath2(Map& map, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points);

	struct GPath;

	struct GNode
	{
		Vector2 pos;
		GNode* prev = nullptr;
		GNode* next = nullptr;
		int connections = 0;
		GPath* parent1 = nullptr;
		GPath* parent2 = nullptr;

		GNode(Vector2 pos)
		{
			this->pos = pos;
		}

		void addConnectionPrev(GNode* prev)
		{
			if (this->prev == prev)
				return;

			if (connections == 2 || prev->connections == 2)
				Godot::print("ERROR: [GNode:addConnectionPrev] connections == 2");

			this->prev = prev;
			connections += 1;

			prev->next = this;
			prev->connections += 1;
		}
		void addConnectionNext(GNode* next)
		{
			if (this->next == next)
				return;

			if (connections == 2 || next->connections == 2)
				Godot::print("ERROR: [GNode:addConnectionNext] connections == 2");

			this->next = next;
			connections += 1;

			next->prev = this;
			next->connections += 1;
		}

		void addConnections(GNode* prev, GNode* next)
		{
			if (this->next != next || this->prev != prev)
				if (next->connections == 2 || prev->connections == 2)
					Godot::print("ERROR: [GNode:addConnections] connections == 2");
			if (this->next != next)
			{
				next->prev = this;
				next->connections += 1;
			}
			if (this->prev != prev)
			{
				prev->next = this;
				prev->connections += 1;
			}

			this->prev = prev;
			this->next = next;
			connections = 2;
		}

		void addConnection(GNode* node)
		{
			if (connections < 2)
			{
				if (this->prev == nullptr)
				{
					connections += 1;
					this->prev = node;
				}
				if (this->next == nullptr)
				{
					connections += 1;
					this->next = node;
				}
			}
		}

		bool isFull()
		{
			return connections == 2;
		}

		std::vector<GPath*> getCircularConnectionPaths(GNode* other)
		{
			std::vector<GPath*> list;
			GPath* path = this->parent1;
			if (path->start != other && path->end != other)
				path = this->parent2;
			GPath* oldPath = path;
			list.push_back(path);

			GNode* old = this;
			GNode* n = (next != nullptr ? next : prev);
			while (n != other && n != nullptr)
			{
				if (n->parent1 != oldPath)
				{
					list.push_back(n->parent1);
					oldPath = n->parent1;
				}
				else
				{
					list.push_back(n->parent2);
					oldPath = n->parent2;
				}
				GNode* nn = n;
				n = (n->next != old ? n->next : n->prev);
				old = nn;
			}
			if (n == other)
			{
				if (n->parent1 != oldPath && n->parent1 != path)
				{
					list.push_back(n->parent1);
					oldPath = n->parent1;
				}
				else if (n->parent2 != path)
				{
					list.push_back(n->parent2);
					oldPath = n->parent2;
				}
			}
			return list;
		}

		PoolVector2Array getCircularConnection(GNode* other)
		{
			PoolVector2Array list;
			list.push_back(pos);
			GNode* old = this;
			GNode* n = (next != nullptr ? next : prev);
			while (n != other && n != nullptr)
			{
				list.push_back(n->pos);
				GNode* nn = n;
				n = (n->next != old ? n->next : n->prev);
				old = nn;
			}
			if (n == other)
				list.push_back(n->pos);
			return list;
		}

		PoolVector2Array getConnectionsToEnd(bool _next)
		{
			PoolVector2Array list;
			GNode* n = this;
			while (n != nullptr)
			{
				list.push_back(n->pos);
				n = (_next ? n->next : n->prev);
			}
			return list;
		}

		bool isConnectedTo(Vector2 pos)
		{
			if (connections != 1)
			{
				Godot::print("ERROR: [GNode:isConnectedTo] connections != 1");
				return false;
			}

			PoolVector2Array list = getConnectionsToEnd((prev == nullptr?true:false));
			if (list[list.size() - 1] == pos)
				return true;
			return false;
		}

		bool operator==(const GNode& other)
		{
			if (pos == other.pos)
				return true;
			return false;
		}
		bool operator!=(const GNode& other)
		{
			if ((*this) == other)
				return false;
			return true;
		}
	};

	struct GPath
	{
		GNode* start = nullptr;
		GNode* end = nullptr;
		PoolVector2Array path;
		bool reverse = false;
		bool used = false;

		GPath(GNode* start, GNode* end, PoolVector2Array path)
		{
			this->start = start;
			this->end = end;
			this->path = path;

			if (this->start->parent1 == nullptr)
				this->start->parent1 = this;
			else
				this->start->parent2 = this;

			if (this->end->parent1 == nullptr)
				this->end->parent1 = this;
			else
				this->end->parent2 = this;
		}

		bool operator==(const GPath& other)
		{
			if (start == other.start && end == other.end)
				return true;
			return false;
		}
		bool operator!=(const GPath& other)
		{
			if ((*this) == other)
				return false;
			return true;
		}
	};

	static PoolVector2Array calculatePath2(Map& map, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points);

private:
	static int pathLength(NodeItem* startNode);
};

