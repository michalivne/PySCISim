// CameraController.cpp
//
// Breannan Smith
// Last updated: 03/09/2014

#include "CameraController.h"

#include <cassert>

const GLdouble CameraController::m_pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679821480865132823066470938446095505822317253594081284811174502841027019385211055596446229489549303819644288109756659334461284756482337867831652712019091456485669234603486104543266482133936072602491412737245870066063155881748815209209628292540917153643678925903600113305305488204665213841469519415116094330572703657595919530921861173819326117931051185480744623799627495673518857527248912279381830119491298336733624406566430860213949463952247371907021798609437027705392171762931767523846748184676694051320005681271452635608277857713427577896091736371787214684409012249534301465495853710507922796892589235420199561121290219608640344181598136297747713099605187072113499999983729780499510597317328160963185950244594553469083026425223082533446850352619311881710100031378387528865875332083814206171776691473035982534904287554687311595628638823537875937519577818577805321712268066130019278766111959092164201989;

CameraController::CameraController()
: m_fovy( 60.0 )
, m_aspect( 1.0 )
, m_theta_cam( m_pi / 3.0 )
, m_phi_cam( m_pi / 4.0 )
, m_rho_cam( 3.0 )
, m_cam_lookat( Eigen::Matrix<GLdouble,3,1>::Zero() )
, m_cam_psn( Eigen::Matrix<GLdouble,3,1>::Zero() )
, m_up_orientation( Eigen::Quaternion<GLdouble>::Identity() )
{
  updatePositionAndFrame();
}

void CameraController::positionCamera()
{
  {
    Eigen::Matrix<GLdouble,4,4> rotation = Eigen::Matrix<GLdouble,4,4>::Identity();
    rotation.block<1,3>(0,0) = m_cam_y;
    rotation.block<1,3>(1,0) = m_cam_x;
    rotation.block<1,3>(2,0) = -m_cam_z;

    assert( ( rotation.block<3,3>(0,0) * rotation.block<3,3>(0,0).transpose() - Eigen::Matrix<GLdouble,3,3>::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );

    glMultMatrixd( rotation.data() );
  }

  {
    const Eigen::Matrix<GLdouble,3,1> camera_psn = m_cam_lookat + m_cam_psn;
    glTranslated( -camera_psn.x(), -camera_psn.y(), -camera_psn.z() );
  }
}

void CameraController::setPerspective( const GLdouble& fovy, const GLdouble& aspect, const GLdouble& zNear, const GLdouble& zFar )
{
  const GLdouble top = tan( fovy / 360.0 * m_pi ) * zNear;
  const GLdouble right = top * aspect;
  glFrustum( -right, right, -top, top, zNear, zFar );
  m_fovy = fovy;
  m_aspect = aspect;
}

void CameraController::setCamera( const Eigen::Matrix<GLdouble,3,1>& up, const GLdouble& theta, const GLdouble& phi, const GLdouble& rho, const Eigen::Matrix<GLdouble,3,1>& lookat )
{
  m_up_orientation = Eigen::Quaternion<GLdouble>::FromTwoVectors( Eigen::Matrix<GLdouble,3,1>::UnitY(), up );
  assert( fabs( m_up_orientation.norm() - 1.0 ) < 1.0e-6 );
  m_theta_cam = theta;
  m_phi_cam = phi;
  assert( rho > 0.0 );
  m_rho_cam = rho;
  m_cam_lookat = lookat;

  updatePositionAndFrame();
}

void CameraController::centerCameraAtSphere( const Eigen::Matrix<GLdouble,3,1>& center, const GLdouble& radius )
{
  m_cam_lookat = center;

  const double degrees_to_radians = m_pi / 180.0;
  const double d1 = radius / sin( 0.5 * m_fovy * degrees_to_radians );
  const double d2 = radius / sin( 0.5 * m_aspect * m_fovy * degrees_to_radians );

  m_rho_cam = std::max( d1, d2 );

  updatePositionAndFrame();
}

void CameraController::addToDistFromCenter( const GLdouble& dst_amnt )
{
  m_rho_cam = std::max( m_rho_cam + dst_amnt, 0.1 );
  updatePositionAndFrame();
}

const GLdouble& CameraController::getDistFromCenter() const
{
  return m_rho_cam;
}

void CameraController::addToZenithAngle( const GLdouble& znth_amnt )
{
  m_theta_cam += znth_amnt;
  if( m_theta_cam < 0.0 ) m_theta_cam = 0.0;
  if( m_theta_cam > m_pi ) m_theta_cam = m_pi;
  updatePositionAndFrame();
}
	
void CameraController::addToAzimuthAngle( const GLdouble& azmth_amnt )
{
  m_phi_cam += azmth_amnt;
  updatePositionAndFrame();
}	

void CameraController::trackCameraHorizontal( const GLdouble& hrz_amnt  )
{
  m_cam_lookat += hrz_amnt * m_cam_y;
}
	
void CameraController::trackCameraVertical( const GLdouble& vrt_amnt  )
{
  m_cam_lookat += vrt_amnt * m_cam_x;
}

void CameraController::updatePositionAndFrame()
{
  assert( fabs( m_up_orientation.norm() - 1.0 ) < 1.0e-6 );

  m_cam_psn = m_up_orientation * Eigen::Matrix<GLdouble,3,1>( sin(m_theta_cam) * sin(m_phi_cam), cos(m_theta_cam), sin(m_theta_cam) * cos(m_phi_cam) );
  m_cam_z = -1.0 * m_cam_psn;
  m_cam_psn *= m_rho_cam;
  m_cam_y = m_up_orientation * Eigen::Matrix<GLdouble,3,1>( cos(m_phi_cam), 0.0, -sin(m_phi_cam) );
  m_cam_x = m_cam_y.cross( m_cam_z );

  assert( fabs( m_cam_x.norm() - 1.0 ) < 1.0e-6 );
  assert( fabs( m_cam_y.norm() - 1.0 ) < 1.0e-6 );
  assert( fabs( m_cam_z.norm() - 1.0 ) < 1.0e-6 );
}
