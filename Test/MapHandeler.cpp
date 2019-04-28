#include "MapHandeler.h"

using namespace godot;

void godot::MapHandeler::_register_methods()
{
	register_method((char*)"init", &MapHandeler::init);
	register_method((char*)"cleanup", &MapHandeler::cleanup);
	register_method((char*)"calculatePath", &MapHandeler::calculatePath);
	register_method((char*)"getProgressBarVisible", &MapHandeler::getProgressBarVisible);
	register_method((char*)"getProgressBarValue", &MapHandeler::getProgressBarValue);
}

void godot::MapHandeler::_init()
{
}

MapHandeler::MapHandeler()
{
}

MapHandeler::~MapHandeler()
{
}

void godot::MapHandeler::init(int width, int height, PoolIntArray var_intArray)
{
	Godot::print((Variant)width);
	Godot::print((Variant)height);
	Godot::print((Variant)var_intArray.size());
	this->map.init(width, height, var_intArray);
	/*for (int i = 0; i < var_intArray.size(); i++)
	{
		Variant var = var_intArray[i];
		Godot::print("iterate");
		Godot::print(var);
	}*/
}

void godot::MapHandeler::cleanup()
{
	this->map.cleanup();
}

PoolVector2Array godot::MapHandeler::calculatePath(int algorithm, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points)
{
	PoolVector2Array path;
	switch (algorithm)
	{
	case 0:
		path = AStar::calculatePath(this->map, start_pos, end_pos);
		break;
	case 1:
		path = AStarSearch::calculatePath(this->map, start_pos, end_pos, goal_points);
		break;
	case 2:
		path = DNN::calculatePath(this->map, start_pos, end_pos, goal_points);
		break;
	default:
		break;
	}

	return path;
}
