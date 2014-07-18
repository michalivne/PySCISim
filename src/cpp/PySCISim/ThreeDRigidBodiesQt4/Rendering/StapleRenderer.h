// StapleRenderer.h
//
// Created by Breannan Smith on 09/06/2013

#ifndef __STAPLE_RENDERER_H__
#define __STAPLE_RENDERER_H__

#include "BodyGeometryRenderer.h"

#include "OpenGL3DSphereRenderer.h"

class StapleRenderer : public BodyGeometryRenderer
{

public:

  StapleRenderer( const int num_subdivs, const std::vector<Vector3s>& points, const scalar& r );
  virtual ~StapleRenderer();

  virtual void renderBody( const Eigen::Matrix<GLfloat,3,1>& color );

private:

  void drawCylinder();

  int computeNumSamples( const int num_subdivs ) const;

  void initializeCylinderMemory();

  const int m_num_samples;
  const std::vector<Vector3s> m_points;
  const scalar m_r;

  OpenGL3DSphereRenderer m_sphere_renderer;

  Eigen::Matrix<GLfloat,Eigen::Dynamic,1> m_cylinder_verts;
  Eigen::Matrix<GLfloat,Eigen::Dynamic,1> m_cylinder_normals;

};

#endif
