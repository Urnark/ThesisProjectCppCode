#include "GreedySearch.h"



GreedySearch::GreedySearch()
{
}


GreedySearch::~GreedySearch()
{
}

int GreedySearch::pathLength(NodeItem* startNode)
{
	int result = 0;
	NodeItem* currentNode = startNode;
	NodeItem* prevNode = nullptr;

	while (currentNode != nullptr)
	{
		result += 1;
		if (currentNode->connectedNode1 != prevNode)
		{
			prevNode = currentNode;
			currentNode = currentNode->connectedNode1;
		}
		else
		{
			prevNode = currentNode;
			currentNode = currentNode->connectedNode2;
		}
	}

	return result;
}

PoolVector2Array GreedySearch::calculatePath(Map & map, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points)
{
	Godot::print("GreedySearch");

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
	float bar_step = 1.0f / ((float)goal_points.size() + 1.0f);
	float current_bar_step = 0.0f;

	PoolVector2Array path;


	NodeItem* startNode = new NodeItem(start_pos);
	NodeItem* endNode = new NodeItem(end_pos);
	std::vector<NodeItem*> goalNodes;
	for (int i = 0; i < goal_points.size(); i++)
	{
		goalNodes.push_back(new NodeItem(goal_points[i]));
	}

	std::vector<PathItem> allPaths;
	bool exists = false;

	Godot::print("Before adding all possible paths");
	//Add all possible paths to allPaths
	for (NodeItem* node1 : goalNodes)
	{
		for (NodeItem* node2 : goalNodes)
		{
			if (!(node1->equals(node2)))
			{
				for (PathItem item : allPaths)
				{
					if (item.node1->equals(node1) && item.node2->equals(node2))
					{
						exists = true;
					}
					else if (item.node2->equals(node1) && item.node1->equals(node2))
					{
						exists = true;
					}
				}
				if (!exists)
				{
					PoolVector2Array path_between_nodes = AStar::calculatePath(map, node1->node, node2->node);
					allPaths.push_back(PathItem(path_between_nodes.size(), node1, node2, path_between_nodes));
				}
				exists = false;
			}
		}
	}
	Godot::print("Before sorting all paths");
	std::qsort(&allPaths[0], allPaths.size(), sizeof(PathItem), [](const void* A, const void* B)
	{
		PathItem arg1 = *static_cast<const PathItem*>(A);
		PathItem arg2 = *static_cast<const PathItem*>(B);

		if (arg1.length < arg2.length) return -1;
		if (arg1.length > arg2.length) return 1;
		return 0;
	});

	Godot::print("Before connecting start node to closest node");
	//Connect start node to closest node
	PoolVector2Array startNodeShortestPath;
	startNodeShortestPath = AStar::calculatePath(map, startNode->node, goalNodes[0]->node);
	NodeItem* startNodeNextNode = goalNodes[0];

	for (int i = 1; i < goalNodes.size(); i++)
	{
		NodeItem* item = goalNodes[i];
		PoolVector2Array tempStartPath = AStar::calculatePath(map, startNode->node, item->node);
		if (tempStartPath.size() < startNodeShortestPath.size())
		{
			startNodeShortestPath = tempStartPath;
			startNodeNextNode = item;
		}
	}
	startNodeNextNode->nrOfConnections += 1;
	startNode->nrOfConnections += 1;
	startNodeNextNode->connectedNode1 = startNode;
	startNodeNextNode->specialConnection = 1;
	startNode->connectedNode1 = startNodeNextNode;


	Godot::print("Before connecting end node to closest node");
	//Connect end node to closest node
	PoolVector2Array endNodeShortestPath;
	endNodeShortestPath = AStar::calculatePath(map, endNode->node, goalNodes[0]->node);
	NodeItem* endNodeNextNode = goalNodes[0];

	for (int i = 1; i < goalNodes.size(); i++)
	{
		NodeItem* item = goalNodes[i];
		PoolVector2Array tempEndPath = AStar::calculatePath(map, endNode->node, item->node);
		if (tempEndPath.size() < endNodeShortestPath.size())
		{
			endNodeShortestPath = tempEndPath;
			endNodeNextNode = item;
		}
	}
	endNodeNextNode->nrOfConnections += 1;
	endNode->nrOfConnections += 1;
	endNodeNextNode->connectedNode1 = endNode;
	endNodeNextNode->specialConnection = 2;
	endNode->connectedNode1 = endNodeNextNode;


	Godot::print("Before picking shortest paths");
	//Pick shortest paths
	std::vector<PathItem> selectedPaths;
	bool flag = false;
	bool found = false;
	while (pathLength(startNode) < (goalNodes.size() + 2))
	{
		for (PathItem path : allPaths)
		{
			found = false;
			if (path.node1->nrOfConnections < 2 && path.node2->nrOfConnections < 2)
			{
				if (path.node1->specialConnection == 1 || path.node2->specialConnection == 1)
				{
					if (path.node1->specialConnection == 2 || path.node2->specialConnection == 2)
					{
						flag = true;
						if ((pathLength(path.node1) + pathLength(path.node2)) >= (goalNodes.size() + 2))
						{
							selectedPaths.push_back(path);
							allPaths.erase(std::find(allPaths.begin(), allPaths.end(), path));
							found = true;
							if (path.node1->connectedNode1 != nullptr)
							{
								path.node1->connectedNode2 = path.node2;
							}
							else
							{
								path.node1->connectedNode1 = path.node2;
							}

							if (path.node2->connectedNode1 != nullptr)
							{
								path.node2->connectedNode2 = path.node1;
							}
							else
							{
								path.node2->connectedNode1 = path.node1;
							}
							path.node1->nrOfConnections += 1;
							path.node2->nrOfConnections += 1;
						}
					}
				}
				if (!flag)
				{
					selectedPaths.push_back(path);
					allPaths.erase(std::find(allPaths.begin(), allPaths.end(), path));
					found = true;

					if (path.node1->specialConnection == 1)
					{
						path.node2->specialConnection = 1;
					}
					else if (path.node2->specialConnection == 1)
					{
						path.node1->specialConnection = 1;
					}

					if (path.node1->specialConnection == 2)
					{
						path.node2->specialConnection = 2;
					}
					else if (path.node2->specialConnection == 2)
					{
						path.node1->specialConnection = 2;
					}

					if (path.node1->connectedNode1 != nullptr)
					{
						path.node1->connectedNode2 = path.node2;
					}
					else
					{
						path.node1->connectedNode1 = path.node2;
					}

					if (path.node2->connectedNode1 != nullptr)
					{
						path.node2->connectedNode2 = path.node1;
					}
					else
					{
						path.node2->connectedNode1 = path.node1;
					}
					path.node1->nrOfConnections += 1;
					path.node2->nrOfConnections += 1;
				}
			}
			flag = false;
		}
		if (found)
		{
			break;
		}
	}

	Godot::print("Before sorting nodes");
	//Sort nodes
	std::vector<PathItem> sortedPaths;
	int arrayLength = selectedPaths.size();
	NodeItem* currentNode = startNodeNextNode;
	while (sortedPaths.size() < arrayLength)
	{
		for (PathItem item : selectedPaths)
		{
			if (item.node1->equals(currentNode))
			{
				sortedPaths.push_back(item);
				currentNode = item.node2;
				selectedPaths.erase(std::find(selectedPaths.begin(), selectedPaths.end(), item));
			}
			else if (item.node2->equals(currentNode))
			{
				sortedPaths.push_back(item);
				currentNode = item.node1;
				selectedPaths.erase(std::find(selectedPaths.begin(), selectedPaths.end(), item));
			}
		}
	}

	Godot::print("Before creating final path");
	//Create final path
	for (int i = 0; i < startNodeShortestPath.size() - 2; i++)
	{
		path.append(startNodeShortestPath[startNodeShortestPath.size() - 2 - i]);
	}

	Vector2 lastNode = startNodeShortestPath[0];

	for (PathItem item : sortedPaths)
	{
		if (item.path[0] == lastNode)
		{
			for (int i = 0; i < item.path.size(); i++)
			{
				path.append(item.path[i]);
				lastNode = item.path[item.path.size() - 1];
			}
		}
		else
		{
			for (int i = 0; i < item.path.size(); i++)
			{
				path.append(item.path[item.path.size() - 1 - i]);
				lastNode = item.path[0];
			}
		}
	}

	for (int i = 0; i < endNodeShortestPath.size(); i++)
	{
		path.append(endNodeShortestPath[i]);
	}

	for (int i = 0; i < goalNodes.size(); i++)
	{
		delete goalNodes[i];
	}

	return path;
}

PoolVector2Array GreedySearch::calculatePath2(Map & map, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points)
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
	float bar_step = 1.0f / ((float)goal_points.size() + 2.0f);
	float current_bar_step = 0.0f;

	PoolVector2Array final_path;

	std::vector<GNode*> goalNodes;
	goalNodes.push_back(new GNode(start_pos));
	goalNodes.push_back(new GNode(end_pos));
	for (int i = 0; i < goal_points.size(); i++)
	{
		goalNodes.push_back(new GNode(goal_points[i]));
	}
	GNode* startNode = goalNodes[0];
	GNode* endNode = goalNodes[1];

	Godot::print("Before adding all possible paths");
	//Add all possible paths to allPaths
	std::vector<GPath*> all_paths;
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
				}
			}
		}
	}

	Godot::print("Before sorting all paths");
	std::qsort(&all_paths[0], all_paths.size(), sizeof(GPath*), [](const void* A, const void* B)
	{
		const GPath* arg1 = *(const GPath**)(A);
		const GPath* arg2 = *(const GPath**)(B);

		if (arg1->path.size() < arg2->path.size()) return -1;
		if (arg1->path.size() > arg2->path.size()) return 1;
		return 0;
	});

	Godot::print("Sorted:");
	for (int i = 0; i < all_paths.size(); i++)
	{
		Godot::print((Variant)all_paths[i]->path.size());
	}

	Godot::print("Before connecting start to end");
	std::vector<GPath*> selected_paths;
	startNode->next = endNode;
	endNode->prev = startNode;
	Godot::print((Variant)startNode->pos);
	Godot::print((Variant)endNode->pos);
	GPath* path = startNode->parent1;
	Godot::print((Variant)startNode->parent1->start->pos);
	Godot::print((Variant)startNode->parent1->end->pos);
	Godot::print((Variant)startNode->parent2->start->pos);
	Godot::print((Variant)startNode->parent2->end->pos);
	if (path->start == endNode || path->end == endNode) {}
	else
		path = startNode->parent2;

	for (auto it = all_paths.begin(); it != all_paths.end(); it++)
	{
		if ((*it) == path)
		{
			Godot::print((Variant)(*it)->start->pos);
			Godot::print((Variant)(*it)->end->pos);
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
				current_bar_step += bar_step;
				map.progress_bar_value = current_bar_step;

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
					delete (*it);
					it = all_paths.erase(it);
				}
				else
				{
					it++;
				}
			}
		}
		if (p2->next != nullptr && p2->prev != nullptr)
		{
			for (auto it = all_paths.begin(); it != all_paths.end();)
			{
				if ((*it)->start->pos == p2->pos || (*it)->end->pos == p2->pos)
				{
					delete (*it);
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

	Godot::print("Testing2:");
	for (int i = 0; i < selected_paths.size(); i++)
	{
		Godot::print((Variant)selected_paths[i]->start->pos);
		Godot::print((Variant)selected_paths[i]->end->pos);
	}

	//std::vector<GPath> sorted_paths;
	//sorted_paths.push_back(selected_paths[0]);

	/*std::vector<GPath> sorted_paths;
	sorted_paths.push_back(selected_paths[0]);
	GPath& oldPath = selected_paths[0];
	GNode* old = selected_paths[0].end;
	GNode* n = sorted_paths.back().end->next;
	if (n == sorted_paths.back().start)
		n = sorted_paths.back().end->prev;
	while (n != selected_paths[0].start)
	{
		if (n->parent1 == &oldPath)
		{
			sorted_paths.push_back(*n->parent2);
			oldPath = *n->parent2;
		}
		else
		{
			sorted_paths.push_back(*n->parent1);
			oldPath = *n->parent1;
		}
		//GNode* nn = n;
		//n = (n->next != old ? n->next : n->prev);
		//old = nn;
		n = sorted_paths.back().end->next;
		if (n == sorted_paths.back().start)
			n = sorted_paths.back().end->prev;
	}*/

	startNode->next = nullptr;
	endNode->prev = nullptr;
	Godot::print("Sorted paths2");
	/*std::vector<GPath*> sorted_paths = startNode->getCircularConnectionPaths(endNode);
	for (GPath* gpath : sorted_paths)
	{
		Godot::print((Variant)gpath->start->pos);
		Godot::print((Variant)gpath->end->pos);
	}*/
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
	}

	for (GPath* gpath : sorted_paths)
	{
		if (gpath->reverse == false)
		{
			for (int i = 0; i < gpath->path.size(); i++)
			{
				final_path.append(gpath->path[i]);
			}
		}
		else
		{
			for (int i = 0; i < gpath->path.size(); i++)
			{
				final_path.append(gpath->path[gpath->path.size() - 1 - i]);
			}
		}
	}

	/*
	Vector2 lastNode = selected_paths[0].path[0];
	for (GPath& item : selected_paths)
	{
		if (item.path[0] == lastNode)
		{
			for (int i = 0; i < item.path.size(); i++)
			{
				final_path.append(item.path[i]);
				lastNode = item.path[item.path.size() - 1];
			}
		}
		else
		{
			for (int i = 0; i < item.path.size(); i++)
			{
				final_path.append(item.path[item.path.size() - 1 - i]);
				lastNode = item.path[0];
			}
		}
	}*/

	Godot::print((Variant)final_path);

	for (GNode* node : goalNodes)
	{
		delete node;
	}
	for (GPath* path : selected_paths)
	{
		delete path;
	}

	return final_path;
}

/*
#include <map>

PoolVector2Array GreedySearch::calculatePath2(Map & map, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points)
{
	Godot::print("GreedySearch");

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
	float bar_step = 1.0f / ((float)goal_points.size() + 1.0f);
	float current_bar_step = 0.0f;

	PoolVector2Array final_path;
	typedef std::vector<Vector2> Edge;
	std::vector<std::vector<Edge>> groups;

	Godot::print("Before adding all possible paths");
	std::vector<std::vector<PoolVector2Array>> all_paths;
	//std::vector<Vector2> checker;
	for (int i = 0; i < goal_points.size() + 2; i++)
	{
		Vector2 current = (i == 0 ? start_pos : (i == 1 ? end_pos : goal_points[i - 2]));
		all_paths.push_back(std::vector<PoolVector2Array>());
		groups.push_back(std::vector<Edge>());
		for (int j = 0; j < goal_points.size() + 2; j++)
		{
			if (i != j)
			{
				//if (std::find(checker.begin(), checker.end(), Vector2(i, j)) == checker.end())
				{
					Vector2 second = (j == 0 ? start_pos : (j == 1 ? end_pos : goal_points[j - 2]));
				//	checker.push_back(Vector2(i, j));
					all_paths[i].push_back(AStar::calculatePath(map, current, second));

					Edge edge;
					edge.push_back(current);
					edge.push_back(second);
					groups[i].push_back(edge);
				}
			}
		}
	}
	Godot::print("Before sorting all paths");
	for (int i = 0; i < all_paths.size(); i++)
	{
		std::qsort(&all_paths[i][0], all_paths[i].size(), sizeof(PoolVector2Array), [](const void* A, const void* B)
		{
			PoolVector2Array arg1 = *static_cast<const PoolVector2Array*>(A);
			PoolVector2Array arg2 = *static_cast<const PoolVector2Array*>(B);

			if (arg1.size() < arg2.size()) return -1;
			if (arg1.size() > arg2.size()) return 1;
			return 0;
		});
	}
	std::qsort(&all_paths[0], all_paths.size(), sizeof(std::vector<PoolVector2Array>), [](const void* A, const void* B)
	{
		std::vector<PoolVector2Array> arg1 = *static_cast<const std::vector<PoolVector2Array>*>(A);
		std::vector<PoolVector2Array> arg2 = *static_cast<const std::vector<PoolVector2Array>*>(B);

		if (arg1[0].size() < arg2[0].size()) return -1;
		if (arg1[0].size() > arg2[0].size()) return 1;
		return 0;
	});

	Godot::print("Before picking shortest paths");
	int count = 0;
	std::vector<std::vector<PoolVector2Array>> test;
	std::map<Vector2, int> vmap;
	std::vector<int> counter;
	for (int i = 0; i < all_paths.size(); i++)
	{
		test.push_back(std::vector<PoolVector2Array>());
		vmap.insert(std::make_pair(all_paths[i][0][0], i));
		counter.push_back(0);
	}

	while (count < goal_points.size() + 2)
	{
		bool out = false;
		for (int i = count; i < all_paths.size() && !out; i++)
		{
			for (int j = 0; j < all_paths[i].size(); j++)
			{
				if ()
				Vector2 pos1 = all_paths[i][j][0];
				Vector2 pos2 = all_paths[i][j][all_paths[i][j].size() - 1];
				if (vmap[pos1] == i)
				{

				}

				if (pos1 == start_pos)
				{
					test[i].push_back(all_paths[i][j]);
					test[i].insert(test[i].begin(), all_paths[i][j]);

					count = i + 1;
					out = true;
					break;
				}
			}
		}
	}
	Godot::print("Before sorting nodes");
	Godot::print("Before creating final path");

	return final_path;
}*/
