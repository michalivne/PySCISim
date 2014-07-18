//  StaticCylinderRenderer.h
//
// Breannan Smith
// Last updated: 06/10/2013

#ifndef __STATIC_CYLINDER_RENDERER__
#define __STATIC_CYLINDER_RENDERER__

#include "ThreeDRigidBodies/StaticGeometry/StaticCylinder.h"

#include <QtOpenGL>

class StaticCylinderRenderer
{

public:

  StaticCylinderRenderer( const int idx, const scalar& length );

  void draw( const StaticCylinder& cylinder ) const;

  inline int idx() const
  {
    return m_idx;
  }
  
private:

  int m_idx;
  GLfloat m_L;

  Eigen::Matrix<GLfloat,Eigen::Dynamic,1> m_crds;

};

#endif
