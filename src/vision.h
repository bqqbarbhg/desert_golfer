#pragma once

struct ViewVec2
{
	float x;
	float y;
};

struct ViewInfo
{
	const void *viewData;
	unsigned viewWidth;
	unsigned viewHeight;

	unsigned screenWidth;
	unsigned screenHeight;
};

struct VisionScene;

struct Scene;
VisionScene *visionReadScene(Scene *scene, const ViewInfo *info);
