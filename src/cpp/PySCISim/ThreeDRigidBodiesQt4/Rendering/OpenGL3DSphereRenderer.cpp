// OpenGL3DSphereRenderer.cpp
//
// Created by Breannan Smith on 09/06/2013

#include "OpenGL3DSphereRenderer.h"

//const GLfloat OpenGL3DSphereRenderer::X_CONST = 0.525731112119133606;

//const GLfloat OpenGL3DSphereRenderer::Z_CONST = 0.850650808352039932;

//const GLfloat OpenGL3DSphereRenderer::m_vdata[12][3] =
//{
//  {-X_CONST, 0.0, Z_CONST}, {X_CONST, 0.0, Z_CONST}, {-X_CONST, 0.0, -Z_CONST}, {X_CONST, 0.0, -Z_CONST},
//  {0.0, Z_CONST, X_CONST}, {0.0, Z_CONST, -X_CONST}, {0.0, -Z_CONST, X_CONST}, {0.0, -Z_CONST, -X_CONST},
//  {Z_CONST, X_CONST, 0.0}, {-Z_CONST, X_CONST, 0.0}, {Z_CONST, -X_CONST, 0.0}, {-Z_CONST, -X_CONST, 0.0}
//};
//
//const GLint OpenGL3DSphereRenderer::m_tindices[20][3] =
//{
//  {1,4,0}, {4,9,0}, {4,5,9}, {8,5,4}, {1,8,4},
//  {1,10,8}, {10,3,8}, {8,3,5}, {3,2,5}, {3,7,2},
//  {3,10,7}, {10,6,7}, {6,11,7}, {6,0,11}, {6,1,0},
//  {10,1,6}, {11,0,9}, {2,11,9}, {5,2,9}, {11,2,7}
//};

const GLfloat OpenGL3DSphereRenderer::m_vdata[4][3] =
{
  {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}
};

const GLint OpenGL3DSphereRenderer::m_tindices[2][3] =
{
  {0,1,2}, {3,2,1}
};


OpenGL3DSphereRenderer::OpenGL3DSphereRenderer( GLfloat radius, int num_subdivisions )
: num_sphere_refinements(num_subdivisions)
, sphere_radius(radius)
, sphere_verts(NULL)
, sphere_normals(NULL)
, m_side_a()
{
  generateSphere();
}

OpenGL3DSphereRenderer::~OpenGL3DSphereRenderer()
{
	deallocateSphereMemory();
}

OpenGL3DSphereRenderer::OpenGL3DSphereRenderer( const OpenGL3DSphereRenderer& sphere_renderer )
{
  num_sphere_refinements = sphere_renderer.num_sphere_refinements;
  sphere_radius = sphere_renderer.sphere_radius;
  sphere_verts = NULL;
  sphere_normals = NULL;
  generateSphere();
}

OpenGL3DSphereRenderer& OpenGL3DSphereRenderer::operator=( const OpenGL3DSphereRenderer& sphere_renderer )
{
  num_sphere_refinements = sphere_renderer.num_sphere_refinements;
  sphere_radius = sphere_renderer.sphere_radius;
  deallocateSphereMemory();
  sphere_verts = NULL;
  sphere_normals = NULL;
  generateSphere();

  return *this;
}

void OpenGL3DSphereRenderer::drawFixedFunction( const Eigen::Matrix<GLfloat,3,1>& primary_color, const Eigen::Matrix<GLfloat,3,1>& secondary_color )
{
  assert( false );

  assert( sphere_radius == 1.0 );
  int num_mesh_verts = computeNumVerts();

  Eigen::Map<Eigen::Matrix<GLfloat,Eigen::Dynamic,1>,0> verts(sphere_verts,3*num_mesh_verts);

  for( int i = 0; i < num_mesh_verts/3; ++i )
  {
    const Eigen::Matrix<GLfloat,3,1> v0 = verts.segment<3>(9*i+3*0);
    const Eigen::Matrix<GLfloat,3,1> v1 = verts.segment<3>(9*i+3*1);
    const Eigen::Matrix<GLfloat,3,1> v2 = verts.segment<3>(9*i+3*2);

    if( i == 0 )
    {
      GLfloat mcolorambient[] = { (GLfloat) 0.3 * primary_color.x(), (GLfloat) 0.3 * primary_color.y(), (GLfloat) 0.3 * primary_color.z(), (GLfloat) 1.0 };
      glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient );
      GLfloat mcolordiffuse[] = { primary_color.x(), primary_color.y(), primary_color.z(), (GLfloat) 1.0 };
      glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse );
    }
    if( i == num_mesh_verts/6 )
    {
      GLfloat mcolorambient[] = { (GLfloat) 0.3 * secondary_color.x(), (GLfloat) 0.3 * secondary_color.y(), (GLfloat) 0.3 * secondary_color.z(), (GLfloat) 1.0 };
      glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient );
      GLfloat mcolordiffuse[] = { secondary_color.x(), secondary_color.y(), secondary_color.z(), (GLfloat) 1.0 };
      glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse );
    }

    glBegin(GL_TRIANGLES);

    glNormal3f(v0.x(),v0.y(),v0.z());
    glVertex3f(v0.x(),v0.y(),v0.z());

    glNormal3f(v1.x(),v1.y(),v1.z());
    glVertex3f(v1.x(),v1.y(),v1.z());

    glNormal3f(v2.x(),v2.y(),v2.z());
    glVertex3f(v2.x(),v2.y(),v2.z());
    glEnd();
  }
}

void OpenGL3DSphereRenderer::drawVertexArray( const Eigen::Matrix<GLfloat,3,1>& primary_color, const Eigen::Matrix<GLfloat,3,1>& secondary_color )
{
  assert( sphere_radius == 1.0 );
  const int num_mesh_verts = computeNumVerts();

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_NORMAL_ARRAY);
  glVertexPointer( 3, GL_FLOAT, 0, this->sphere_verts );
  glNormalPointer( GL_FLOAT, 0, this->sphere_normals );

  // Quad 1
  GLfloat mcolorambient[] = { (GLfloat) 0.3 * primary_color.x(), (GLfloat) 0.3 * primary_color.y(), (GLfloat) 0.3 * primary_color.z(), (GLfloat) 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient );
  GLfloat mcolordiffuse[] = { primary_color.x(), primary_color.y(), primary_color.z(), (GLfloat) 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse );
  glDrawArrays( GL_TRIANGLES, 0, num_mesh_verts );

  // Quad 2
  glPushMatrix();
  glRotatef( 180.0, 0.0, 0.0, 1.0 );
  glDrawArrays( GL_TRIANGLES, 0, num_mesh_verts );
  glPopMatrix();

  // Quad 3
  GLfloat mcolorambient2[] = { (GLfloat) 0.3 * secondary_color.x(), (GLfloat) 0.3 * secondary_color.y(), (GLfloat) 0.3 * secondary_color.z(), (GLfloat) 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient2 );
  GLfloat mcolordiffuse2[] = { secondary_color.x(), secondary_color.y(), secondary_color.z(), 1.0f };
  glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse2 );
  glPushMatrix();
  glRotatef( 90.0, 0.0, 0.0, 1.0 );
  glDrawArrays( GL_TRIANGLES, 0, num_mesh_verts );
  glPopMatrix();

  // Quad 4
  glPushMatrix();
  glRotatef( 270.0, 0.0, 0.0, 1.0 );
  glDrawArrays( GL_TRIANGLES, 0, num_mesh_verts );
  glPopMatrix();
  
  glDisableClientState(GL_NORMAL_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
}

int OpenGL3DSphereRenderer::computeNumTriangles( int num_subdivs ) const
{
  int num_tri = 2;
  for( int i = 0; i < num_subdivs; ++i ) num_tri *= 4;
  return num_tri;
}

int OpenGL3DSphereRenderer::computeNumVerts()
{
  return 3 * computeNumTriangles( num_sphere_refinements );
}

void OpenGL3DSphereRenderer::allocateSphereMemory()
{
  assert( (sphere_verts==NULL) && (sphere_normals==NULL) );

  const int num_mesh_verts = computeNumVerts();

  // 3 coordinates per vertex
  sphere_verts = new GLfloat[ 3 * num_mesh_verts ];

  // 3 coordinates per vertex
  sphere_normals = new GLfloat[ 3 * num_mesh_verts ];
}

void OpenGL3DSphereRenderer::deallocateSphereMemory()
{
  if(this->sphere_verts != NULL)
  {
    delete[] this->sphere_verts;
    this->sphere_verts = NULL;
  }

  if(this->sphere_normals != NULL)
  {
    delete[] this->sphere_normals;
    this->sphere_normals = NULL;
  }
}

void OpenGL3DSphereRenderer::saveTriangleInMem( const Eigen::Matrix<GLfloat,3,1>& v1, const Eigen::Matrix<GLfloat,3,1>& v2, const Eigen::Matrix<GLfloat,3,1>& v3, int& current_vertex )
{
  this->sphere_verts[ 3 * current_vertex + 0 ] = this->sphere_radius * v1[0];
  this->sphere_verts[ 3 * current_vertex + 1 ] = this->sphere_radius * v1[1];
  this->sphere_verts[ 3 * current_vertex + 2 ] = this->sphere_radius * v1[2];
  this->sphere_normals[ 3 * current_vertex + 0 ] = v1[0];
  this->sphere_normals[ 3 * current_vertex + 1 ] = v1[1];
  this->sphere_normals[ 3 * current_vertex + 2 ] = v1[2];
  ++current_vertex;

  this->sphere_verts[ 3 * current_vertex + 0 ] = this->sphere_radius * v2[0];
  this->sphere_verts[ 3 * current_vertex + 1 ] = this->sphere_radius * v2[1];
  this->sphere_verts[ 3 * current_vertex + 2 ] = this->sphere_radius * v2[2];
  this->sphere_normals[ 3 * current_vertex + 0 ] = v2[0];
  this->sphere_normals[ 3 * current_vertex + 1 ] = v2[1];
  this->sphere_normals[ 3 * current_vertex + 2 ] = v2[2];
  ++current_vertex;

  this->sphere_verts[ 3 * current_vertex + 0 ] = this->sphere_radius * v3[0];
  this->sphere_verts[ 3 * current_vertex + 1 ] = this->sphere_radius * v3[1];
  this->sphere_verts[ 3 * current_vertex + 2 ] = this->sphere_radius * v3[2];
  this->sphere_normals[ 3 * current_vertex + 0 ] = v3[0];
  this->sphere_normals[ 3 * current_vertex + 1 ] = v3[1];
  this->sphere_normals[ 3 * current_vertex + 2 ] = v3[2];
  ++current_vertex;
}

void OpenGL3DSphereRenderer::subdivide( const Eigen::Matrix<GLfloat,3,1>& v1, const Eigen::Matrix<GLfloat,3,1>& v2, const Eigen::Matrix<GLfloat,3,1>& v3, int& current_vertex, int depth )
{
  assert( depth >= 0 );
  
  // If we hit the lowest level of recursion
  if( depth == 0 )
  {
    // Save the current triangle
    saveTriangleInMem( v1, v2, v3, current_vertex );
    return;
  }

  // New vertices lie on the midpoint of the three edges of the larger triangle
  Eigen::Matrix<GLfloat,3,1> v12 = ( v1 + v2 ) / 2.0f;
  Eigen::Matrix<GLfloat,3,1> v23 = ( v2 + v3 ) / 2.0f;
  Eigen::Matrix<GLfloat,3,1> v31 = ( v3 + v1 ) / 2.0f;

  // Ensure that the new vertices are on the surface of the sphere
  v12 = v12.normalized();
  v23 = v23.normalized();
  v31 = v31.normalized();

  // This triangle is divided into four children
  subdivide( v1, v12, v31, current_vertex, depth-1 );
  subdivide( v2, v23, v12, current_vertex, depth-1 );
  subdivide( v3, v31, v23, current_vertex, depth-1 );
  subdivide( v12, v23, v31, current_vertex, depth-1 );
}

void OpenGL3DSphereRenderer::generateSphere()
{
  deallocateSphereMemory();
  allocateSphereMemory();

  int current_vertex = 0;
  for ( int i = 0; i < 2; ++i )
  {
    const Eigen::Matrix<GLfloat,3,1> vertex_a( m_vdata[m_tindices[i][0]][0], m_vdata[m_tindices[i][0]][1], m_vdata[m_tindices[i][0]][2] );
    const Eigen::Matrix<GLfloat,3,1> vertex_b( m_vdata[m_tindices[i][1]][0], m_vdata[m_tindices[i][1]][1], m_vdata[m_tindices[i][1]][2] );
    const Eigen::Matrix<GLfloat,3,1> vertex_c( m_vdata[m_tindices[i][2]][0], m_vdata[m_tindices[i][2]][1], m_vdata[m_tindices[i][2]][2] );
    subdivide( vertex_a, vertex_b, vertex_c, current_vertex, this->num_sphere_refinements );
  }
}

