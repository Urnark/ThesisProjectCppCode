#include "NearestNeighbor.h"



NearestNeighbor::NearestNeighbor()
{
}


NearestNeighbor::~NearestNeighbor()
{
}

PoolVector2Array NearestNeighbor::calculatePath(Map & map, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points)
{
	Godot::print("Nearest Neighbor");

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

	Vector2 currentNode = start_pos;
	Vector2 closestNode;

	Godot::print("Before while");
	
	while (goal_points.size() > 0)
	{
		int index = -1;
		int closestDistance = std::numeric_limits<int>::max();
		for (int i = 0; i < goal_points.size(); i++)
		{
			if (currentNode != end_pos && currentNode != goal_points[i])
			{
				int dist = currentNode.distance_to(goal_points[i]);
				if (dist < closestDistance)
				{
					closestDistance = dist;
					closestNode = goal_points[i];
					index = i;
				}
			}
		}
		PoolVector2Array currentPath = AStar::calculatePath(map, currentNode, closestNode);
		for (int j = 0; j < currentPath.size() - 1; j++)
		{
			path.append(currentPath[currentPath.size() - 2 - j]);
		}

		currentNode = closestNode;
		

		if (index >= 0)
		{
			goal_points.remove(index);
		}
	}

	Godot::print("After while");

	PoolVector2Array finalPath = AStar::calculatePath(map, currentNode, end_pos);

	for (int i = 0; i < finalPath.size() - 2; i++)
	{
		path.append(finalPath[finalPath.size() - 2 - i]);
	}

	Godot::print((Variant)path);

	return path;
}
