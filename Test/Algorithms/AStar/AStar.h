#pragma once

#include <core/Godot.hpp>
#include <core/PoolArrays.hpp>
#include <ProgressBar.hpp>
#include "../../Map.h"
#include <vector>

using namespace godot;

class AStar
{
public:
	AStar() {};
	~AStar() {};

	static float calculateH(Vector2 pos1, Vector2 pos2);
	static bool isValid(Map& map, Vector2 pos);
	static PoolVector2Array makePath(Map& map, Vector2 start_pos, Vector2 end_pos);
	static PoolVector2Array calculatePath(Map& map, Vector2 start_pos, Vector2 end_pos);
};
