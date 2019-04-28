#include "AStar.h"

float AStar::calculateH(Vector2 pos1, Vector2 pos2)
{
	return pos1.distance_to(pos2);
}

bool AStar::isValid(Map & map, Vector2 pos)
{
	if (pos.x >= 0.0f && pos.x < map.width && pos.y >= 0.0f && pos.y < map.height)
	{
		return map.tiles[(int)pos.x + (int)pos.y * map.width]->get_tileIndex() != 0;
	}
	return false;
}

PoolVector2Array AStar::makePath(Map & map, Vector2 start_pos, Vector2 end_pos)
{
	PoolVector2Array path;
	path.append(end_pos);
	Vector2 pos = end_pos;
	while (pos != start_pos)
	{
		pos = map.tile(pos)->get_parent()->get_pos();
		path.append(pos);
	}
	return path;
}

PoolVector2Array AStar::calculatePath(Map& map, Vector2 start_pos, Vector2 end_pos)
{
	if (start_pos == end_pos)
	{
		Godot::print("Error: start_pos == end_pos");
		return PoolVector2Array();
	}
	if (!isValid(map, start_pos))
	{
		Godot::print("Error: start_pos is not valid");
		return PoolVector2Array();
	}
	if (!isValid(map, end_pos))
	{
		Godot::print("Error: start_pos is not valid");
		return PoolVector2Array();
	}

	map.clearForAlgorithm();

	std::vector<MyTile*> open;
	MyTile* tile = map.tile(start_pos);
	tile->set_f(0);
	tile->set_g(0);
	tile->set_h(0);
	tile->set_visited(false);
	open.push_back(tile);

	while (!open.empty())
	{
		float min = open[0]->get_f();
		int index = 0;
		for (int i = 0; i < open.size(); i++)
		{
			if (open[i]->get_f() == min)
			{
				index = i;
			}
			else if (open[i]->get_f() < min)
			{
				min = open[i]->get_f();
				index = i;
			}
		}

		tile = open[index];
		open.erase(open.begin() + index);
		tile->set_visited(true);

		for (int x = -1; x <= 1; x++)
		{
			for (int y = -1; y <= 1; y++)
			{
				if (!(x == 0 && y == 0))
				{
					Vector2 p = Vector2(x, y) + tile->get_pos();
					if (isValid(map, p))
					{
						MyTile* cell = map.tile(p);
						if (p == end_pos)
						{
							cell->set_parent(tile);
							return makePath(map, start_pos, end_pos);
						}
						else if (cell->get_visited() == false)
						{
							float g = tile->get_g() + ((x != 0 && y != 0) ? 10.0f : 5.0f);
							float h = calculateH(p, end_pos);
							float f = g + h;
							// Check if this path is better than the one already present
							if (cell->get_f() > f)
							{
								// Update the details of this neighbour node
								cell->set_f(f);
								cell->set_h(h);
								cell->set_g(g);
								cell->set_parent(tile);
								open.push_back(cell);
							}
						}
					}
				}
			}
		}
	}

	return PoolVector2Array();
}
