#pragma once

#include <core/Godot.hpp>
#include <core/PoolArrays.hpp>
#include <ProgressBar.hpp>
#include "Map.h"
#include "Algorithms/AStar/AStar.h"
#include "Algorithms/AStarSearch/AStarSearch.h"
#include "Algorithms/DNN/DNN.h"

namespace godot
{

	class MapHandeler : public Reference
	{
		GODOT_CLASS(MapHandeler, Reference)
	private:
		Map map;

	public:
		static void _register_methods();
		void _init();

		MapHandeler();
		~MapHandeler();

		void init(int width, int height, PoolIntArray var_intArray);
		void cleanup();

		PoolVector2Array calculatePath(int algorithm, Vector2 start_pos, Vector2 end_pos, PoolVector2Array goal_points);

		bool getProgressBarVisible() { return this->map.progress_bar_visible; };
		float getProgressBarValue() { return this->map.progress_bar_value; };
	};

}