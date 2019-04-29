#pragma once

#include <core/Godot.hpp>
#include <core/PoolArrays.hpp>
#include <ProgressBar.hpp>
#include "../../Map.h"
#include <vector>
#include "../AStar/AStar.h"

using namespace godot;

class NearestNeighbor
{
public:
	NearestNeighbor();
	~NearestNeighbor();

	static PoolVector2Array calculatePath(Map& map, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points);
};
