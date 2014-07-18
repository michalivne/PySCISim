//  BodyGeometryRenderer.h
//
//  Created by Breannan Smith on 03/30/2013

#ifndef __BODY_GEOMETRY_RENDERER__
#define __BODY_GEOMETRY_RENDERER__

#include "SDIC/Math/MathDefines.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

class BodyGeometryRenderer
{

public:

  virtual ~BodyGeometryRenderer();

  virtual void renderBody( const Eigen::Matrix<GLfloat,3,1>& color ) = 0;

};

#endif
