#pragma once

struct SceneVec2
{
	float x;
	float y;
};

struct ScenePoly
{
	SceneVec2 *vertices;
	unsigned numVertices;
};

struct Scene
{
	ScenePoly *polys;
	unsigned numPolys;

	SceneVec2 ballPos;
	float ballRadius;

	SceneVec2 goalMin;
	SceneVec2 goalMax;

	int screenWidth;
	int screenHeight;
};

