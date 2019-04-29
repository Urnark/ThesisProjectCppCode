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
