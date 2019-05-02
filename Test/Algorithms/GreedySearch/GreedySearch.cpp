#include "GreedySearch.h"



GreedySearch::GreedySearch()
{
}


GreedySearch::~GreedySearch()
{
}

int GreedySearch::GPath::ID = 0;

PoolVector2Array GreedySearch::calculatePath(Map & map, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points)
{
	Godot::print("GreedySearch2");

	if (start_pos == end_pos)
	{
		Godot::print("Error: start_pos == end_pos");
		return PoolVector2Array();
	}
	if (!AStar::isValid(map, start_pos))
	{
		Godot::print("Error: start_pos is not valid");
		return PoolVector2Array();
	}
	if (!AStar::isValid(map, end_pos))
	{
		Godot::print("Error: end_pos is not valid");
		return PoolVector2Array();
	}

	map.progress_bar_value = 0.0f;
	map.progress_bar_visible = true;
	increaseProgress(map, 0, true);
	float nr_of_nodes = goal_points.size() + 2.0f;
	float edges = ((nr_of_nodes)*((nr_of_nodes - 1))) / 2.0f;
	float total = (float)(nr_of_nodes * 7 - 4 + edges * 2);
	float bar_step = 1.0f / total;

	PoolVector2Array final_path;

	std::vector<GNode*> goalNodes;
	goalNodes.push_back(new GNode(start_pos));
	increaseProgress(map, bar_step);
	goalNodes.push_back(new GNode(end_pos));
	increaseProgress(map, bar_step);
	for (int i = 0; i < goal_points.size(); i++)
	{
		goalNodes.push_back(new GNode(goal_points[i]));
		increaseProgress(map, bar_step);
	}
	GNode* startNode = goalNodes[0];
	GNode* endNode = goalNodes[1];

	Godot::print("Before adding all possible paths");
	//Add all possible paths to allPaths
	std::vector<GPath*> all_paths;
	std::vector<GPath*> copy_for_clearing_memory_of_all_paths;
	bool exists = false;
	for (GNode* node1 : goalNodes)
	{
		for (GNode* node2 : goalNodes)
		{
			if (node1 != node2)
			{
				exists = false;
				for (GPath* path : all_paths)
				{
					if (path->start == node1 && path->end == node2)
					{
						exists = true;
					}
					else if (path->start == node2 && path->end == node1)
					{
						exists = true;
					}
				}
				if (!exists)
				{
					all_paths.push_back(new GPath(node1, node2, AStar::calculatePath(map, node1->pos, node2->pos)));
					all_paths.back()->length = map.tile(node2->pos)->get_g();
					all_paths.back()->set_parents();
					copy_for_clearing_memory_of_all_paths.push_back(all_paths.back());
					increaseProgress(map, bar_step);
				}
			}
		}
	}

	Godot::print("Before sorting all paths");
	std::qsort(&all_paths[0], all_paths.size(), sizeof(GPath*), [](const void* A, const void* B)
	{
		const GPath* arg1 = *(const GPath**)(A);
		const GPath* arg2 = *(const GPath**)(B);

		if (arg1->length < arg2->length) return -1;
		if (arg1->length > arg2->length) return 1;
		return 0;
	});

	Godot::print("Before connecting start to end");
	std::vector<GPath*> selected_paths;
	startNode->next = endNode;
	endNode->prev = startNode;
	GPath* path = startNode->parent1;
	if (path->start != endNode && path->end != endNode)
		path = startNode->parent2;

	for (auto it = all_paths.begin(); it != all_paths.end(); it++)
	{
		if ((*it) == path)
		{
			increaseProgress(map, bar_step);

			selected_paths.push_back((*it));
			all_paths.erase(it);
			break;
		}
	}

	Godot::print("Before picking shortest paths");
	while (!all_paths.empty())
	{
		GNode* p1 = all_paths[0]->start;
		GNode* p2 = all_paths[0]->end;
		for (auto it = all_paths.begin(); it != all_paths.end();)
		{
			p1 = (*it)->start;
			p2 = (*it)->end;

			PoolVector2Array temp = p1->getCircularConnection(p2);
			if (temp[temp.size() - 1] != p2->pos || selected_paths.size() == goalNodes.size() - 1)
			{
				increaseProgress(map, bar_step);

				selected_paths.push_back((*it));
				if (p1->next == nullptr)
					p1->next = p2;
				else
					p1->prev = p2;

				if (p2->prev == nullptr)
					p2->prev = p1;
				else
					p2->next = p1;
				all_paths.erase(it);
				break;
			}
			else
			{
				it++;
			}
		}

		// Remove all the paths that is connected to a node that has two connections
		if (p1->next != nullptr && p1->prev != nullptr)
		{
			for (auto it = all_paths.begin(); it != all_paths.end();)
			{
				if ((*it)->start->pos == p1->pos || (*it)->end->pos == p1->pos)
				{
					it = all_paths.erase(it);
				}
				else
				{
					it++;
				}
			}
		}
		// Remove all the paths that is connected to a node that has two connections
		if (p2->next != nullptr && p2->prev != nullptr)
		{
			for (auto it = all_paths.begin(); it != all_paths.end();)
			{
				if ((*it)->start->pos == p2->pos || (*it)->end->pos == p2->pos)
				{
					it = all_paths.erase(it);
				}
				else
				{
					it++;
				}
			}
		}
	}

	// Remove connection from start to end
	selected_paths.erase(selected_paths.begin());

	// Update the parent pointers in every Node
	for (int i = 0; i < selected_paths.size(); i++)
	{
		selected_paths[i]->clear_parents();
		increaseProgress(map, bar_step);
	}
	for (int i = 0; i < selected_paths.size(); i++)
	{
		selected_paths[i]->set_parents();
		increaseProgress(map, bar_step);
	}

	Godot::print("Sorted paths");
	// Sort the paths so they go from start node to end node
	startNode->next = nullptr;
	endNode->prev = nullptr;
	std::vector<GPath*> sorted_paths;
	for (int i = 0; i < selected_paths.size(); i++)
	{
		for (GPath* inner_path : selected_paths)
		{
			if (sorted_paths.size() == 0)
			{
				if (inner_path->start == startNode)
				{
					inner_path->used = true;
					sorted_paths.push_back(inner_path);
					sorted_paths.back()->reverse = true;
					break;
				}
				else if (inner_path->end == startNode)
				{
					inner_path->used = true;
					sorted_paths.push_back(inner_path);
					break;
				}
			}
			else if (inner_path->used == false)
			{
				GNode* next = sorted_paths.back()->end->next;
				if (next == sorted_paths.back()->start)
					next = sorted_paths.back()->end->prev;

				bool use_end = true;
				if (sorted_paths.size() > 1)
				{
					GPath* twoBack = sorted_paths[sorted_paths.size() - 2];
					if (next->parent1 == twoBack || next->parent2 == twoBack)
					{
						next = sorted_paths.back()->start->next;
						if (next == sorted_paths.back()->end)
							next = sorted_paths.back()->start->prev;
						use_end = false;
					}
				}

				if (next == inner_path->end && (use_end?sorted_paths.back()->end: sorted_paths.back()->start) == inner_path->start)
				{
					inner_path->used = true;
					sorted_paths.push_back(inner_path);
					sorted_paths.back()->reverse = true;
					break;
				}
				else if (next == inner_path->start && (use_end ? sorted_paths.back()->end : sorted_paths.back()->start) == inner_path->end)
				{
					inner_path->used = true;
					sorted_paths.push_back(inner_path);
					break;
				}
			}
		}

		increaseProgress(map, bar_step);
	}

	/*Godot::print("Testing:");
	for (int i = 0; i < selected_paths.size(); i++)
	{
		Godot::print(selected_paths[i]->to_string());
	}*/

	// Place all the nodes in all the selected sorted paths in the final path
	for (GPath* gpath : sorted_paths)
	{
		if (gpath->reverse == false)
		{
			for (int i = (gpath == sorted_paths[0] ? 1 : 0); i < gpath->path.size() - 1; i++)
			{
				final_path.append(gpath->path[i]);
			}
		}
		else
		{
			for (int i = (gpath == sorted_paths[0] ? 1 : 0); i < gpath->path.size() - 1; i++)
			{
				final_path.append(gpath->path[gpath->path.size() - 1 - i]);
			}
		}

		increaseProgress(map, bar_step);
	}

	// Delete memorry that has been allocated
	for (GNode* node : goalNodes)
	{
		delete node;
		increaseProgress(map, bar_step);
	}
	for (GPath* path : copy_for_clearing_memory_of_all_paths)
	{
		delete path;
		increaseProgress(map, bar_step);
	}


	map.progress_bar_value = 1.0f;
	map.progress_bar_visible = false;

	return final_path;
}
