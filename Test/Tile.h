#pragma once

#include <core/Godot.hpp>
#include <Reference.hpp>
#include <limits>

namespace godot {

	class MyTile
	{
	private:
		int tile_index = 0;
		Vector2 pos;
		float f = std::numeric_limits<float>::max();
		float h = std::numeric_limits<float>::max();
		float g = std::numeric_limits<float>::max();
		bool visited = false;
		MyTile* parent = nullptr;

	public:
		MyTile();
		~MyTile();

		Vector2 get_pos() { return this->pos; };
		void set_pos(Vector2 pos) { this->pos = pos; };
		int get_tileIndex() { return this->tile_index; };
		void set_tileIndex(int tile_index) { this->tile_index = tile_index; };
		float get_h() { return this->h; };
		void set_h(float h) { this->h = h; };
		float get_f() { return this->f; };
		void set_f(float f) { this->f = f; };
		float get_g() { return this->g; };
		void set_g(float g) { this->g = g; };
		bool get_visited() { return this->visited; };
		void set_visited(bool visited) { this->visited = visited; };
		MyTile* get_parent() { return this->parent; };
		void set_parent(MyTile* parent) { this->parent = parent; };

		void init(int tile_index, Vector2 pos);
	};
}
