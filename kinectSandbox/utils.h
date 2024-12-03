#pragma once
#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <GL/gl.h>

// Tamaño de la ventana y resolución del Kinect
const int width = 640;
const int height = 480;

// Buffer para datos de profundidad
std::vector<GLfloat> elevationData(width* height);


#endif
