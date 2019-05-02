#pragma once

#include <core/Godot.hpp>
#include <core/PoolArrays.hpp>
#include <ProgressBar.hpp>
#include "../../Map.h"
#include <vector>
#include "../AStar/AStar.h"
#include <string>

using namespace godot;

class GreedySearch
{
public:
	GreedySearch();
	~GreedySearch();

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
		float length = 0.0f;
		bool reverse = false;
		bool used = false;
		static int ID;
		int id = 0;

		GPath(GNode* start, GNode* end, PoolVector2Array path)
		{
			this->start = start;
			this->end = end;
			this->path = path;

			ID++;
			id = ID;
		}

		void clear_parents()
		{
			this->start->parent1 = nullptr;
			this->start->parent2 = nullptr;
			this->end->parent1 = nullptr;
			this->end->parent2 = nullptr;

		}

		void set_parents()
		{
			if (this->start->parent1 == nullptr)
				this->start->parent1 = this;
			else
				this->start->parent2 = this;

			if (this->end->parent1 == nullptr)
				this->end->parent1 = this;
			else
				this->end->parent2 = this;
		}

		String id_str()
		{
			return String(std::to_string(id).c_str());
		}

		String to_string()
		{
			String line = this->id_str() + ", length: " + String(std::to_string(length).c_str()) + ", " + " {(" + (String)this->start->pos + "), (" + (String)this->end->pos + ")}";
			line += ", Start { parent1: " + (this->start->parent1 == nullptr ? "null" : this->start->parent1->id_str()) + ", parent2: " +
				(this->start->parent2 == nullptr ? "null" : this->start->parent2->id_str()) + " }";
			line += ", End { parent1: " + (this->end->parent1 == nullptr ? "null" : this->end->parent1->id_str()) + ", parent2: " +
				(this->end->parent2 == nullptr ? "null" : this->end->parent2->id_str()) + " }";

			return line;
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

	static PoolVector2Array calculatePath(Map& map, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points);

private:
	static void increaseProgress(Map& map, float bar_step, bool reset = false)
	{
		static float current_bar_step = 0.0f;
		if (reset)
		{
			current_bar_step = 0.0f;
			return;
		}

		current_bar_step += bar_step;
		if (current_bar_step > 1.0f) current_bar_step = 1.0f;
		map.progress_bar_value = current_bar_step;
	}
};

