#pragma once

#include <core/Godot.hpp>
#include <Reference.hpp>
#include "Tile.h"
#include <core/PoolArrays.hpp>

namespace godot {

	class Map : public Reference
	{
		//GODOT_CLASS(Map, Reference)
	private:

	public:
		int width;
		int height;
		MyTile** tiles;

		float progress_bar_value = 0.0f;
		bool progress_bar_visible = false;

		//static void _register_methods();
		//void _init();
		//void _process(float delta);

		Map();
		~Map();

		MyTile* tile(int x, int y);
		MyTile* tile(Vector2 v);
		void init(int width, int height, PoolIntArray& var_intArray);
		void cleanup();
		void clearForAlgorithm();
	};
}

