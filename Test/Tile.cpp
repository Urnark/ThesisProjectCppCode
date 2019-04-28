#include "Tile.h"

using namespace godot;

godot::MyTile::MyTile()
{
}


godot::MyTile::~MyTile()
{
}

void godot::MyTile::init(int tile_index, Vector2 pos)
{
	this->tile_index = tile_index;
	this->pos = pos;
	this->f = std::numeric_limits<float>::max();
	this->h = std::numeric_limits<float>::max();
	this->g = std::numeric_limits<float>::max();
	this->visited = false;
	this->parent = nullptr;
}
