#include "Map.h"

using namespace godot;

/*void godot::Map::_register_methods()
{
	
}

void godot::Map::_init()
{
}

void godot::Map::_process(float delta)
{
}*/

godot::Map::Map()
{
}

godot::Map::~Map()
{
	cleanup();
}

MyTile * godot::Map::tile(int x, int y)
{
	return this->tiles[x + y * width];
}

MyTile * godot::Map::tile(Vector2 v)
{
	return this->tiles[(int)v.x + (int)v.y * width];
}

void godot::Map::init(int width, int height, PoolIntArray & var_intArray)
{
	this->width = width;
	this->height = height;
	this->tiles = new MyTile*[width * height];
	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			int index = x + y * width;
			this->tiles[index] = new MyTile();
			this->tiles[index]->init(var_intArray[index], Vector2(x, y));
		}
	}
}

void godot::Map::cleanup()
{
	if (this->tiles)
	{
		for (int i = 0; i < width * height; i++)
		{
			delete this->tiles[i];
		}
		delete this->tiles;
	}
}

void godot::Map::clearForAlgorithm()
{
	for (int i = 0; i < width*height; i++)
	{
		this->tiles[i]->set_f(std::numeric_limits<float>::max());
		this->tiles[i]->set_g(std::numeric_limits<float>::max());
		this->tiles[i]->set_h(std::numeric_limits<float>::max());
		this->tiles[i]->set_visited(false);
		this->tiles[i]->set_parent(nullptr);
	}
}
