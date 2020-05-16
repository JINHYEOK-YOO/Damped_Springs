#ifndef __GL_SETUP_H_
#define __GL_SETUP_H_

#include <GL/glew.h>
#include <GLFW/glfw3.h>             // OpenGL Extension Wrangler Library

extern float  screenScale;          // Portion of the screen when not using full screen
extern int    screenW, screenH;     // screenScale portion of the screen
extern int    windowW, windowH;     // Framebuffer size
extern float  aspect;				// Aspect ratio = windowW/windowH

extern bool   perspective;          // Perspective or orthographic viewing
extern float  fovy;                 // Field of view in the y direction

GLFWwindow* initializeOpenGL(int argc, char* argv[], GLfloat bgColor[4]);
void        reshape(GLFWwindow* window, int w, int h);

void    drawAxes(float l, float w);

#endif  // __GL_SETUP_H_
