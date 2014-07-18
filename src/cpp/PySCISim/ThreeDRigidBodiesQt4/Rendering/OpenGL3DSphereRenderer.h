// OpenGL3DSphereRenderer.h
//
// Created by Breannan Smith on 09/06/2013

#ifndef __OPENGL_3D_SPHERE_RENDERER_H__
#define __OPENGL_3D_SPHERE_RENDERER_H__

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#include <iostream>
#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

class OpenGL3DSphereRenderer
{
public:

	OpenGL3DSphereRenderer( GLfloat radius = 1.0, int num_subdivisions = 0 );
	~OpenGL3DSphereRenderer();

	OpenGL3DSphereRenderer( const OpenGL3DSphereRenderer& sphere_renderer );
	OpenGL3DSphereRenderer& operator=( const OpenGL3DSphereRenderer& sphere_renderer );

  void drawFixedFunction( const Eigen::Matrix<GLfloat,3,1>& primary_color, const Eigen::Matrix<GLfloat,3,1>& secondary_color );

  void drawVertexArray( const Eigen::Matrix<GLfloat,3,1>& primary_color, const Eigen::Matrix<GLfloat,3,1>& secondary_color );


private:

  // For a given number of subdivisions, returns the number of traingles
  int computeNumTriangles( int num_subdivs ) const;

  // Computes the number of vertices for the current number of refinements.
  int computeNumVerts();

  // Allocates memory for the vertex arrays with vertices and normals.
  void allocateSphereMemory();

  // Deallocates memory for the vertex arrays with vertices and normals.
  void deallocateSphereMemory();

  // Saves the current triangle in the vertex array.
  void saveTriangleInMem( const Eigen::Matrix<GLfloat,3,1>& v1, const Eigen::Matrix<GLfloat,3,1>& v2, const Eigen::Matrix<GLfloat,3,1>& v3, int& current_vertex );

  // Recursive routine to refine the mesh.
  void subdivide( const Eigen::Matrix<GLfloat,3,1>& v1, const Eigen::Matrix<GLfloat,3,1>& v2, const Eigen::Matrix<GLfloat,3,1>& v3, int& current_vertex, int depth );

  // Builds vertex arrays with the sphere's vertices and normals.
  void generateSphere();

  // Values that define triangles of a icosahedron.
  //const static GLfloat X_CONST;
  //const static GLfloat Z_CONST;
  const static GLfloat m_vdata[4][3];
  const static GLint m_tindices[2][3];

  int num_sphere_refinements;
  GLfloat sphere_radius;
  GLfloat* sphere_verts;
  GLfloat* sphere_normals;

  std::vector<bool> m_side_a;

	//std::vector<SphereTriangle> m_triangles;
	//std::vector<Vertex> m_verts;
};


#endif


