#include "DNN.h"

Vector2 DNN::nn(PoolVector2Array & all, Vector2 pos, int& index)
{
	float distance_to_nn = AStar::calculateH(pos, all[0]);
	Vector2 goal_pos = all[0];
	for (int i = 0; i < all.size(); i++)
	{
		float temp_dist = AStar::calculateH(pos, all[i]);
		if (distance_to_nn > temp_dist)
		{
			distance_to_nn = temp_dist;
			goal_pos = all[i];
			index = i;
		}
	}
	return goal_pos;
}

PoolVector2Array DNN::calculatePath(Map & map, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points)
{
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
		Godot::print("Error: start_pos is not valid");
		return PoolVector2Array();
	}

	map.progress_bar_value = 0.0f;
	map.progress_bar_visible = true;
	float bar_step = 1.0f / ((float)goal_points.size() + 1.0f);
	float current_bar_step = 0.0f;

	PoolVector2Array path;

	Vector2 current_pos = start_pos;
	// Loop through all goals
	while (goal_points.size() != 0)
	{
		int index = 0;
		// Get the closest goal from the current position
		Vector2 goal = nn(goal_points, current_pos, index);
		float distance_to_nn = AStar::calculateH(current_pos, goal);

		// Calculate a path from the current position to the goal
		PoolVector2Array path_to_goal = AStar::calculatePath(map, current_pos, goal);
		float length = map.tile(goal)->get_g();

		// See if a goal that is closer exists and if so use it instead
		int step = 10;
		for (int i = 0; i < (path_to_goal.size() - 1) / step; i++)
		{
			int temp_index = 0;
			// Find a new goal that is closer to the position in the current path to the goal
			Vector2 temp_goal = nn(goal_points, path_to_goal[(path_to_goal.size() - 2) - (i * step)], temp_index);
			if (temp_goal != goal)
			{
				// If the path to the new goal from the current position is shorter use it instead
				PoolVector2Array temp_path_to_goal = AStar::calculatePath(map, current_pos, temp_goal);
				if (map.tile(temp_goal)->get_g() < length)
				{
					path_to_goal = temp_path_to_goal;
					index = temp_index;
					break;
				}
			}
		}

		// Add all the positions in the path to the goal to the final path
		for (int i = 0; i < path_to_goal.size() - 1; i++)
			path.append(path_to_goal[(path_to_goal.size() - 2) - i]);

		// Set the current position to the goal that the path leads to
		current_pos = path_to_goal[0];
		// Remove the goal from the list of goals
		goal_points.remove(index);

		current_bar_step += bar_step;
		map.progress_bar_value = current_bar_step;
	}
	// Add the path that leads to the end position to the final path
	PoolVector2Array path_to_goal = AStar::calculatePath(map, current_pos, end_pos);
	for (int i = 1; i < path_to_goal.size() - 1; i++)
		path.append(path_to_goal[(path_to_goal.size() - 1) - i]);

	map.progress_bar_value = 1.0f;
	map.progress_bar_visible = false;

	return path;
}
