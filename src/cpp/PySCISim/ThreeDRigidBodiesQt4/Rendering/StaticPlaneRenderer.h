// StaticPlaneRenderer.h
//
// Breannan Smith
// Last updated: 03/16/2014

#ifndef __STATIC_PLANE_RENDERER__
#define __STATIC_PLANE_RENDERER__

#include "SDIC/Math/MathDefines.h"

class StaticPlane;

class StaticPlaneRenderer
{

public:

  StaticPlaneRenderer( const unsigned idx, const Array2s& half_width );

  unsigned idx() const;

  void draw( const StaticPlane& plane ) const;

private:

  unsigned m_idx;
  Array2s m_half_width;

};

#endif
