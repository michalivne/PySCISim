// CameraController.h
//
// Breannan Smith
// Last updated: 03/09/2014

#ifndef __CAMERA_CONTROLLER_H__
#define __CAMERA_CONTROLLER_H__

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

class CameraController
{

public:

  CameraController();

  // Sets the ModelViewMatrix to have the camera settings defined by this CameraController.
  void positionCamera();

  // Sets the projection matrix. Caches data used when automatically positioning camera.
  void setPerspective( const GLdouble& fovy, const GLdouble& aspect, const GLdouble& near, const GLdouble& far );

  // TODO: Move the three params for x, y, z into a vector
  void setCamera( const Eigen::Matrix<GLdouble,3,1>& up, const GLdouble& theta, const GLdouble& phi, const GLdouble& rho, const Eigen::Matrix<GLdouble,3,1>& lookat );

  // Centers the camera at and sets distance from the input sphere so that is visible
  void centerCameraAtSphere( const Eigen::Matrix<GLdouble,3,1>& center, const GLdouble& radius );

  // Increments/decrements the camera's distance from the viewed point
  void addToDistFromCenter( const GLdouble& dst_amnt );

  // Returns the camera's distance to the viewed point
  const GLdouble& getDistFromCenter() const; 

  // Increments/decrements the camera's angle with respect to the zenith
  void addToZenithAngle( const GLdouble& znth_amnt );

  // Increments/decrements the camera's angle with respect to the azimuth
  void addToAzimuthAngle( const GLdouble& azmth_amnt );

  // Increments/decrements the cameras position horizontally in the current viewing plane
  void trackCameraHorizontal( const GLdouble& hrz_amnt );

  // Increments/decrements the cameras position vertically in the current viewing plane
  void trackCameraVertical( const GLdouble& vrt_amnt  );

  inline const GLdouble& theta() const
  {
    return m_theta_cam;
  }

  inline const GLdouble& phi() const
  {
    return m_phi_cam;
  }

  inline const GLdouble& rho() const
  {
    return m_rho_cam;
  }

  inline const Eigen::Matrix<GLdouble,3,1>& lookat() const
  {
    return m_cam_lookat;
  }

  inline Eigen::Matrix<GLdouble,3,1> up() const
  {
    return m_up_orientation * Eigen::Matrix<GLdouble,3,1>::UnitY();
  }

private:

  void updatePositionAndFrame();

  GLdouble m_fovy;
  GLdouble m_aspect;

  // Local frame of camera
  Eigen::Matrix<GLdouble,3,1> m_cam_x;
  Eigen::Matrix<GLdouble,3,1> m_cam_y;
  Eigen::Matrix<GLdouble,3,1> m_cam_z;

  // Spherical coordinates of camera
  // Note: theta wrt y
  //       phi wrt z
  GLdouble m_theta_cam;
  GLdouble m_phi_cam;
  GLdouble m_rho_cam;

  // Position camera sphere is centered at
  Eigen::Matrix<GLdouble,3,1>  m_cam_lookat;

  // Position of camera in cartesian coordinates
  // Note: This is not set directly, but computed from spherical coords
  Eigen::Matrix<GLdouble,3,1> m_cam_psn;

  // Rotation wrt <0,1,0> of up for the spherical coordinate system
  Eigen::Quaternion<GLdouble> m_up_orientation;

  const static GLdouble m_pi;

};

#endif
