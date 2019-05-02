#include "AStarSearch.h"

PoolVector2Array AStarSearch::calculatePath(Map & map, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points)
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
	while (goal_points.size() != 0)
	{
		int index = 0;
		// Get the path from the current position to the closest goal
		PoolVector2Array path_to_goal = AStar::calculatePath(map, current_pos, goal_points[0]);
		float length = map.tile(goal_points[0])->get_g();
		for (int i = 0; i < goal_points.size(); i++)
		{
			PoolVector2Array temp_path = AStar::calculatePath(map, current_pos, goal_points[i]);
			if (map.tile(goal_points[i])->get_g() < length)
			{
				path_to_goal = temp_path;
				index = i;
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
