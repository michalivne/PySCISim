// StaticPlaneRenderer.h
//
// Breannan Smith
// Last updated: 08/30/2014

#ifndef __STATIC_PLANE_RENDERER__
#define __STATIC_PLANE_RENDERER__

#include "SCISim/Math/MathDefines.h"

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
