#define NANOVG_GL2_IMPLEMENTATION
#define GLEW_STATIC

#include <GL/glew.h>
#include <GL/GL.h>
#include <GLFW/glfw3.h>
#include "../external/nanovg/nanovg.h"
#include "../external/nanovg/nanovg_gl.h"
#include "../external/nanovg/stb_image.h"
#include <stddef.h>
#include "physics.h"
#include "vision.h"
#include "scene.h"

int main(int argc, char **argv)
{
	glfwInit();

	GLFWwindow *window = glfwCreateWindow(1440, 750, "Desert golfer", NULL, NULL);
	glfwMakeContextCurrent(window);
	glewInit();

	Scene scene;

	ViewInfo viewInfo;
	int width, height, comp;
	stbi_uc *pixels = stbi_load("test5.png", &width, &height, &comp, 3);

	viewInfo.viewData = pixels;
	viewInfo.viewWidth = width;
	viewInfo.viewHeight = height;
	viewInfo.screenWidth = 1334;
	viewInfo.screenHeight = 750;

	VisionScene *vision = visionReadScene(&scene, &viewInfo);

	scene.ballPos.x = 473.0f;
	scene.ballPos.y = 505.0f;
	scene.ballRadius = 5.0f;

	scene.goalMin.x = 1220.0f;
	scene.goalMin.y = 526.0f;
	scene.goalMax.x = 1256.0f;
	scene.goalMax.y = 550.0f;

	PhysicsScene *physics = physicsCreateScene(&scene);

	NVGcontext *nvg = nvgCreateGL2(0);

	while (!glfwWindowShouldClose(window))
	{
		glfwPollEvents();

		physicsUpdate(physics, 50);

		int windowWidth, windowHeight;
		glfwGetWindowSize(window, &windowWidth, &windowHeight);

		glClearColor(0x64/255.0f, 0x95/255.0f, 0xED/255.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		nvgBeginFrame(nvg, windowWidth, windowHeight, 1.0f);
		physicsRender(physics, nvg);
		nvgEndFrame(nvg);

		glfwSwapBuffers(window);
	}

	return 0;
}
