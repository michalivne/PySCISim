// GLWidget.cpp
//
// Breannan Smith
// Last updated: 06/17/2014

#include "GLWidget.h"

#include <QtGui>
#include <QtOpenGL>
#include <cmath>
#include <iostream>
#include <fstream>

#include "ThreeDRigidBodiesUtils/XMLSceneParser.h"

#include "SDIC/ConstrainedMaps/ImpactFrictionMap.h"
#include "SDIC/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "SDIC/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "SDIC/ConstrainedMaps/FrictionMaps/FrictionOperator.h"
#include "SDIC/UnconstrainedMaps/UnconstrainedMap.h"
#include "SDIC/MersenneTwister.h"
#include "SDIC/Timer/Timer.h"

#include "ThreeDRigidBodies/KinematicScripting.h"
#include "ThreeDRigidBodies/Geometry/RigidBodyBox.h"
#include "ThreeDRigidBodies/Geometry/RigidBodySphere.h"
#include "ThreeDRigidBodies/Geometry/RigidBodyTriangleMesh.h"
#include "ThreeDRigidBodies/Geometry/RigidBodyStaple.h"

#include "ThreeDRigidBodies/RenderingState.h"
#include "Rendering/OpenGL3DSphereRenderer.h"
#include "Rendering/BodyGeometryRenderer.h"
#include "Rendering/StapleRenderer.h"

GLWidget::GLWidget( QWidget* parent )
: QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
, m_camera_controller()
, m_render_at_fps( false )
, m_lock_camera( false )
, m_last_pos()
, m_left_mouse_button_pressed( false )
, m_right_mouse_button_pressed( false )
, m_body_colors()
, m_display_precision(0)
, m_display_HUD( true )
, m_display_xy_grid( false )
, m_display_yz_grid( false )
, m_display_xz_grid( false )
, m_movie_dir_name()
, m_movie_dir()
, m_output_frame(0)
, m_output_fps()
, m_steps_per_frame()
, m_unconstrained_map( NULL )
, m_impact_operator( NULL )
, m_friction_operator( NULL )
, m_impact_friction_map( NULL )
, m_iteration(0)
, m_dt(0.0)
, m_end_time( SCALAR_INFINITY )
, m_CoR( SCALAR_NAN )
, m_mu( SCALAR_NAN )
, m_scripting_callback( NULL )
, m_sim()
, m_H0()
, m_p0( Vector3s::Zero() )
, m_L0( Vector3s::Zero() )
, m_delta_H0( 0.0 )
, m_delta_p0( Vector3s::Zero() )
, m_delta_L0( Vector3s::Zero() )
, m_sphere_renderer( NULL )
, m_plane_renderers()
, m_cylinder_renderers()
, m_portal_renderers()
, m_body_renderers()
{}

GLWidget::~GLWidget()
{
  if( m_scripting_callback != NULL ) delete m_scripting_callback;
  if( m_unconstrained_map != NULL ) delete m_unconstrained_map;
  if( m_impact_operator != NULL ) delete m_impact_operator;
  if( m_friction_operator != NULL ) delete m_friction_operator;
  if( m_impact_friction_map != NULL ) delete m_impact_friction_map;
  if( m_sphere_renderer != NULL ) delete m_sphere_renderer;

  for( std::vector<BodyGeometryRenderer*>::size_type i = 0; i < m_body_renderers.size(); ++i )
  {
    if( m_body_renderers[i] != NULL ) delete m_body_renderers[i];
  }
  
  SAVE_TIMERS("timing.txt")
  DELETE_TIMERS()
}

QSize GLWidget::minimumSizeHint() const
{
  return QSize( 50, 50 );
}

QSize GLWidget::sizeHint() const
{
  return QSize( 512, 512 );
}

void GLWidget::openScene( const QString& xml_scene_file_name )
{
  // TODO: Memory leak occurs here if multiple scenes loaded!
  UnconstrainedMap* new_unconstrained_map = NULL;
  ImpactOperator* new_impact_operator = NULL;
  FrictionOperator* new_friction_operator = NULL;
  ImpactFrictionMap* new_impact_friction_map = NULL;

  std::string dt_string = "";
  std::string scripting_callback = "";
  RigidBodySimState sim_state;
  scalar dt;
  scalar end_time;
  scalar CoR;
  scalar mu;
  RenderingState new_render_state;

  const bool loaded_successfully = XMLSceneParser::parseXMLSceneFile( xml_scene_file_name.toStdString(), scripting_callback, sim_state, &new_unconstrained_map, dt_string, dt, end_time, &new_impact_operator, CoR, &new_friction_operator, mu, &new_impact_friction_map, new_render_state );

  if( !loaded_successfully )
  {
    std::cout << "Load failed." << std::endl;
    assert( new_unconstrained_map == NULL );
    assert( new_impact_operator == NULL );
    assert( new_friction_operator == NULL );
    assert( new_impact_friction_map ==  NULL );
    return;
  }

  //m_sim.resetCachedData();
  
  if( m_scripting_callback != NULL ) delete m_scripting_callback;
  KinematicScripting::initializeScriptingCallback( scripting_callback, &m_scripting_callback );
  assert( m_scripting_callback != NULL );

  m_dt = dt;
  m_CoR = CoR;
  m_mu = mu;

  // Compute the number of characters after the decimal point in the timestep string
  std::string::size_type right_start = dt_string.size();
  for( std::string::size_type i = 0; i < dt_string.size(); ++i )
  {
    if( dt_string[i] == '.' )
    {
      right_start = i + 1;
      break;
    }
  }
  m_display_precision = (int) ( dt_string.size() - right_start );

  if( m_unconstrained_map != NULL ) delete m_unconstrained_map;
  if( m_impact_operator != NULL ) delete m_impact_operator;
  if( m_friction_operator != NULL ) delete m_friction_operator;
  if( m_impact_friction_map != NULL ) delete m_impact_friction_map;

  m_unconstrained_map = new_unconstrained_map;
  m_impact_operator = new_impact_operator;
  m_friction_operator = new_friction_operator;
  m_impact_friction_map = new_impact_friction_map;

  m_sim.setState( sim_state );
  m_sim.clearConstraintCache();

  m_H0 = m_sim.computeTotalEnergy();
  m_p0 = m_sim.computeTotalMomentum();
  m_L0 = m_sim.computeTotalAngularMomentum();
  m_delta_H0 = 0.0;
  m_delta_p0 = Vector3s::Zero();
  m_delta_L0 = Vector3s::Zero();

  m_body_colors.resize( 3 * sim_state.nbodies() );
  {
    std::vector<QColor> colors;
    {
      // 0131 U
      const QColor yellow( 252, 245, 156 );
      colors.push_back( yellow );
      // 0331 U
      const QColor red( 255, 178, 190 );
      colors.push_back( red );
      // 0521 U
      const QColor magenta( 251, 170, 221 );
      colors.push_back( magenta );
      // 0631 U
      const QColor violet( 188, 148, 222 );
      colors.push_back( violet );
      // 0821 U
      const QColor blue( 103, 208, 238 );
      colors.push_back( blue );
      // 0921 U
      const QColor green( 114, 229, 210 );
      colors.push_back( green );
    }

    MersenneTwister mt(0);
    for( int i = 0; i < m_body_colors.size(); i += 3 )
    {
      // Select a random color
      const unsigned long color_num = mt.genrand_long( colors.size() - 1 );
      assert( color_num < colors.size() );
//      scalar r = 0.75; scalar g = 0.75; scalar b = 0.75;
//      // TODO: Can probably handle this better by just generating in another colorspace, but whatever...
//      // Generate colors until we get one with a luminance within [0.1,0.9]
//      //while( (0.2126 * r + 0.7152 * g + 0.0722 * b) > 0.9 || (0.2126 * r + 0.7152 * g + 0.0722 * b) < 0.1 )
//      //{
//        r += (scalar) mt.genrand_real1();
//        g += (scalar) mt.genrand_real1();
//        b += (scalar) mt.genrand_real1();
//      //}
//      r *= 0.5;
//      g *= 0.5;
//      b *= 0.5;
      m_body_colors.segment<3>(i) << colors[color_num].redF(), colors[color_num].greenF(), colors[color_num].blueF();
    }
  }

  // Create a renderer for the system
  for( std::vector<BodyGeometryRenderer*>::size_type i = 0; i < m_body_renderers.size(); ++i )
    if( m_body_renderers[i] != NULL ) delete m_body_renderers[i];
  m_body_renderers.clear();
  m_body_renderers.resize( m_sim.getState().geometry().size() );
  for( std::vector<RigidBodyGeometry*>::size_type i = 0; i < m_sim.getState().geometry().size(); ++i )
  {
    if( m_sim.getState().geometry()[i]->getType() == RigidBodyGeometry::STAPLE )
    {
      const RigidBodyStaple& geo = *dynamic_cast<const RigidBodyStaple*>( m_sim.getState().geometry()[i] );
      m_body_renderers[i] = new StapleRenderer( 4, geo.points(), geo.r() );
    }
    else
    {
      m_body_renderers[i] = NULL;
    }
  }

  m_iteration = 0;

  m_end_time = end_time;
  assert( m_end_time > 0.0 );

  assert( m_sim.getState().q().size() == 12 * m_sim.getState().nbodies() );
  
  // Backup the simulation state
  m_sim_state_backup = m_sim.getState();

  m_output_frame = 0;
  
  m_steps_per_frame = floor( 1.0 / ( m_dt * m_output_fps ) + 0.5 );

  initializeRenderingSettings( new_render_state );

  // If there are any intitial collisions, warn the user
  unsigned num_initial_collisions;
  scalar penetration_depth;
  m_sim.computeNumberOfCollisions( num_initial_collisions, penetration_depth );
  if( num_initial_collisions != 0 )
  {
    std::cerr << "Warning, detected initial contacts. " << std::endl;
    std::cerr << "  Number of collisions:   " << num_initial_collisions << std::endl;
    std::cerr << "  TotalPenetration depth: " << penetration_depth << std::endl;
  }
  
  updateGL();
}

void GLWidget::initializeRenderingSettings( const RenderingState& rendering_state )
{
  // Create plane renderers
  m_plane_renderers.clear();
  for( unsigned plane_renderer_index = 0; plane_renderer_index < rendering_state.numPlaneRenderers(); ++plane_renderer_index )
  {
    m_plane_renderers.push_back( StaticPlaneRenderer( rendering_state.planeRenderer(plane_renderer_index).index(), rendering_state.planeRenderer(plane_renderer_index).r() ) );
  }
  
  // Create cylinder renderers
  m_cylinder_renderers.clear();
  for( unsigned cylinder_renderer_index = 0; cylinder_renderer_index < rendering_state.numCylinderRenderers(); ++cylinder_renderer_index )
  {
    m_cylinder_renderers.push_back( StaticCylinderRenderer( rendering_state.cylinderRenderer(cylinder_renderer_index).index(), rendering_state.cylinderRenderer(cylinder_renderer_index).L() ) );
  }

  // Create the portal renderers
  m_portal_renderers.clear();
  for( unsigned portal_renderer_index = 0; portal_renderer_index < rendering_state.numPlanarPortalRenderers(); ++portal_renderer_index )
  {
    m_portal_renderers.push_back( PlanarPortalRenderer( rendering_state.portalRenderer( portal_renderer_index ).portalIndex(), rendering_state.portalRenderer( portal_renderer_index ).halfWidth0(), rendering_state.portalRenderer( portal_renderer_index ).halfWidth1() ) );
  }

  // Set the camera
  if( rendering_state.cameraSettingsInitialized() )
  {
    m_camera_controller.setCamera( rendering_state.cameraUp(), rendering_state.cameraTheta(), rendering_state.cameraPhi(), rendering_state.cameraRho(), rendering_state.cameraLookAt() );
  }
  else
  {
    centerCamera();
  }
}

void GLWidget::stepSystem()
{
  if( m_iteration * m_dt >= m_end_time )
  {
    std::cout << "Simulation complete. Exiting." << std::endl;
    std::exit( EXIT_SUCCESS );
  }

  const int next_iter = m_iteration + 1;

  assert( m_scripting_callback != NULL );
  m_scripting_callback->setRigidBodySimState( m_sim.state() );
  m_scripting_callback->startOfStepCallback( next_iter, m_dt );

  if( m_unconstrained_map == NULL && m_impact_operator == NULL && m_friction_operator == NULL )
  {
    return;
  }
  else if( m_unconstrained_map != NULL && m_impact_operator == NULL && m_friction_operator == NULL )
  {
    m_sim.flow( next_iter, m_dt, *m_unconstrained_map );
  }
  else if( m_unconstrained_map != NULL && m_impact_operator != NULL && m_friction_operator == NULL )
  {
    assert( m_scripting_callback != NULL );
    m_sim.flow( *m_scripting_callback, next_iter, m_dt, *m_unconstrained_map, *m_impact_operator, m_CoR );
  }
  else if( m_unconstrained_map != NULL && m_impact_operator != NULL && m_friction_operator != NULL && m_impact_friction_map != NULL )
  {
    assert( m_scripting_callback != NULL );
    m_sim.flow( *m_scripting_callback, next_iter, m_dt, *m_unconstrained_map, *m_impact_operator, m_CoR, *m_friction_operator, m_mu, *m_impact_friction_map );
  }
  else
  {
    std::cerr << "Impossible code path hit. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  ++m_iteration;

  m_delta_H0 = std::max( m_delta_H0, fabs( m_H0 - m_sim.computeTotalEnergy() ) );
  const Vector3s p = m_sim.computeTotalMomentum();
  m_delta_p0.x() = std::max( m_delta_p0.x(), fabs( m_p0.x() - p.x() ) );
  m_delta_p0.y() = std::max( m_delta_p0.y(), fabs( m_p0.y() - p.y() ) );
  m_delta_p0.z() = std::max( m_delta_p0.z(), fabs( m_p0.z() - p.z() ) );
  const Vector3s L = m_sim.computeTotalAngularMomentum();
  m_delta_L0.x() = std::max( m_delta_L0.x(), fabs( m_L0.x() - L.x() ) );
  m_delta_L0.y() = std::max( m_delta_L0.y(), fabs( m_L0.y() - L.y() ) );
  m_delta_L0.z() = std::max( m_delta_L0.z(), fabs( m_L0.z() - L.z() ) );

  if( !m_render_at_fps || m_iteration % m_steps_per_frame == 0 ) updateGL();

  if( m_movie_dir_name.size() != 0 )
  {
    if( m_iteration % m_steps_per_frame == 0 )
    {
      // Save a screenshot of the current state
      QString output_image_name = QString( tr("frame%1.png") ).arg( m_output_frame, 10, 10, QLatin1Char('0') );
      saveScreenshot( m_movie_dir.filePath( output_image_name ) );
      ++m_output_frame;
    }
  }
}

void GLWidget::resetSystem()
{
  m_sim.setState( m_sim_state_backup );
  //m_sim.resetCachedData();
  m_sim.clearConstraintCache();
  if( m_impact_friction_map != NULL ) m_impact_friction_map->resetCachedData();

  m_iteration = 0;

  m_H0 = m_sim.computeTotalEnergy();
  m_p0 = m_sim.computeTotalMomentum();
  m_L0 = m_sim.computeTotalAngularMomentum();
  m_delta_H0 = 0.0;
  m_delta_p0 = Vector3s::Zero();
  m_delta_L0 = Vector3s::Zero();

  updateGL();
}

void GLWidget::getSimData( double& time, double& T, double& U, Eigen::Vector3d& p, Eigen::Vector3d& L )
{
  time = m_iteration * m_dt;

  T = m_sim.computeKineticEnergy();
  U = m_sim.computePotentialEnergy();

  p = m_sim.computeTotalMomentum();
  L = m_sim.computeTotalAngularMomentum();
}

void GLWidget::initializeGL()
{
  glEnable( GL_DEPTH_TEST );
  qglClearColor( QColor(255,255,255,255) );
  glShadeModel( GL_SMOOTH );
  GLfloat global_ambient[] = { 0.5f, 0.5f, 0.5f, 1.0f };
  glLightModelfv( GL_LIGHT_MODEL_AMBIENT, global_ambient );
  GLfloat diffuse[] = {1.0f, 1.0f, 1.0f , 1.0f};
  glLightfv( GL_LIGHT0, GL_DIFFUSE, diffuse );
  glEnable( GL_LIGHTING );
  glEnable( GL_LIGHT0 );
  glEnable( GL_NORMALIZE ); // Prpobably slow... but fine for now

  m_sphere_renderer = new OpenGL3DSphereRenderer( 1.0, 4 );

  assert( checkGLErrors() );
}

void GLWidget::resizeGL( int width, int height )
{
  assert( width >= 0 ); assert( height >= 0 );
  glViewport( 0, 0, width, height );

  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();

  m_camera_controller.setPerspective( 54.43, (GLdouble) width / (GLdouble) height, 0.1, 1000.0 );

  assert( checkGLErrors() );
}

void GLWidget::paintGL()
{
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();

  m_camera_controller.positionCamera();

  if( axesDrawingIsEnabled() ) paintAxes();

  if( m_display_xy_grid ) paintXYRulers();
  
  if( m_display_yz_grid ) paintYZRulers();
  
  if( m_display_xz_grid ) paintXZRulers();
  
  paintSystem();

  if( m_display_HUD ) paintHUD();

  assert( autoBufferSwap() );

  assert( checkGLErrors() );
}

bool GLWidget::axesDrawingIsEnabled() const
{
  return m_left_mouse_button_pressed;
}

void GLWidget::paintAxes() const
{
  glPushAttrib(GL_COLOR);
  glPushAttrib(GL_LIGHTING);
  glPushAttrib(GL_LINE_WIDTH);

  glDisable(GL_LIGHTING);
  glLineWidth(2.0);

  // Draw the positive x, y, and z axis
  glColor3d(1.0,0.0,0.0);
  glBegin(GL_LINES);
  glVertex4d(0.0,0.0,0.0,1.0);
  glVertex4d(1.0,0.0,0.0,0.0);
  glEnd();
  glColor3d(0.0,1.0,0.0);
  glBegin(GL_LINES);
  glVertex4d(0.0,0.0,0.0,1.0);
  glVertex4d(0.0,1.0,0.0,0.0);
  glEnd();
  glColor3d(0.0,0.0,1.0);
  glBegin(GL_LINES);
  glVertex4d(0.0,0.0,0.0,1.0);
  glVertex4d(0.0,0.0,1.0,0.0);
  glEnd();

  // Draw the negative x, y, and z axis
  glLineStipple(1,0x00FF);
  glEnable(GL_LINE_STIPPLE);
  glColor3d(1.0,0.0,0.0);
  glBegin(GL_LINES);
  glVertex4d(-1.0,0.0,0.0,0.0);
  glVertex4d(0.0,0.0,0.0,1.0);
  glEnd();
  glColor3d(0.0,1.0,0.0);
  glBegin(GL_LINES);
  glVertex4d(0.0,-1.0,0.0,0.0);
  glVertex4d(0.0,0.0,0.0,1.0);
  glEnd();
  glColor3d(0.0,0.0,1.0);
  glBegin(GL_LINES);
  glVertex4d(0.0,0.0,-1.0,0.0);
  glVertex4d(0.0,0.0,0.0,1.0);
  glEnd();
  glDisable(GL_LINE_STIPPLE);

  glPopAttrib();
  glPopAttrib();
  glPopAttrib();
}

void GLWidget::paintXYRulers() const
{
  const int grid_width = 20;

  glPushAttrib(GL_COLOR);
  glPushAttrib(GL_LIGHTING);
  glPushAttrib(GL_LINE_WIDTH);
  
  glDisable(GL_LIGHTING);

  glLineWidth(0.5);
  glColor3d(0.5,0.5,0.5);
  glBegin( GL_LINES );
  for( int y = -grid_width; y <= grid_width; ++y )
  {
    if( y == 0 ) continue;
    glVertex3d( -grid_width, y, 0.0 );
    glVertex3d(  grid_width, y, 0.0 );
  }
  glEnd();

  glLineWidth(3.0);
  glColor3d(0.0,0.0,0.0);
  glBegin( GL_LINES );
  glVertex3d( -grid_width, 0.0, 0.0 );
  glVertex3d(  grid_width, 0.0, 0.0 );
  glEnd();

  glLineWidth(0.5);
  glColor3d(0.5,0.5,0.5);
  glBegin( GL_LINES );
  for( int x = -grid_width; x <= grid_width; ++x )
  {
    if( x == 0 ) continue;
    glVertex3d( x, -grid_width, 0.0 );
    glVertex3d( x,  grid_width, 0.0 );
  }
  glEnd();

  glLineWidth(3.0);
  glColor3d(0.0,0.0,0.0);
  glBegin( GL_LINES );
  glVertex3d( 0.0, -grid_width, 0.0 );
  glVertex3d( 0.0,  grid_width, 0.0 );
  glEnd();
  
  glPopAttrib();
  glPopAttrib();
  glPopAttrib();
}

void GLWidget::paintYZRulers() const
{
  const int grid_width = 20;
  
  glPushAttrib(GL_COLOR);
  glPushAttrib(GL_LIGHTING);
  glPushAttrib(GL_LINE_WIDTH);
  
  glDisable(GL_LIGHTING);
  
  glLineWidth(0.5);
  glColor3d(0.5,0.5,0.5);
  glBegin( GL_LINES );
  for( int y = -grid_width; y <= grid_width; ++y )
  {
    if( y == 0 ) continue;
    glVertex3d( 0.0, y, -grid_width );
    glVertex3d( 0.0, y, grid_width );
  }
  glEnd();
  
  glLineWidth(3.0);
  glColor3d(0.0,0.0,0.0);
  glBegin( GL_LINES );
  glVertex3d( 0.0, 0.0, -grid_width );
  glVertex3d( 0.0, 0.0,  grid_width );
  glEnd();
  
  glLineWidth(0.5);
  glColor3d(0.5,0.5,0.5);
  glBegin( GL_LINES );
  for( int z = -grid_width; z <= grid_width; ++z )
  {
    if( z == 0 ) continue;
    glVertex3d( 0.0, -grid_width, z );
    glVertex3d( 0.0,  grid_width, z );
  }
  glEnd();
  
  glLineWidth(3.0);
  glColor3d(0.0,0.0,0.0);
  glBegin( GL_LINES );
  glVertex3d( 0.0, -grid_width, 0.0 );
  glVertex3d( 0.0,  grid_width, 0.0 );
  glEnd();
  
  glPopAttrib();
  glPopAttrib();
  glPopAttrib();
}

void GLWidget::paintXZRulers() const
{
  const int grid_width = 20;
  
  glPushAttrib(GL_COLOR);
  glPushAttrib(GL_LIGHTING);
  glPushAttrib(GL_LINE_WIDTH);
  
  glDisable(GL_LIGHTING);
  
  glLineWidth(0.5);
  glColor3d(0.5,0.5,0.5);
  glBegin( GL_LINES );
  for( int x = -grid_width; x <= grid_width; ++x )
  {
    if( x == 0 ) continue;
    glVertex3d( x, 0.0, -grid_width );
    glVertex3d( x, 0.0,  grid_width );
  }
  glEnd();

  glLineWidth(3.0);
  glColor3d(0.0,0.0,0.0);
  glBegin( GL_LINES );
  glVertex3d( 0.0, 0.0, -grid_width );
  glVertex3d( 0.0, 0.0,  grid_width );
  glEnd();

  glLineWidth(0.5);
  glColor3d(0.5,0.5,0.5);
  glBegin( GL_LINES );
  for( int z = -grid_width; z <= grid_width; ++z )
  {
    if( z == 0 ) continue;
    glVertex3d( -grid_width, 0.0, z );
    glVertex3d(  grid_width, 0.0, z );
  }
  glEnd();
  
  glLineWidth(3.0);
  glColor3d(0.0,0.0,0.0);
  glBegin( GL_LINES );
  glVertex3d( -grid_width, 0.0, 0.0 );
  glVertex3d(  grid_width, 0.0, 0.0 );
  glEnd();
  
  glPopAttrib();
  glPopAttrib();
  glPopAttrib();
}

void GLWidget::getViewportDimensions( GLint& width, GLint& height ) const
{
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT,viewport);
  width = viewport[2];
  height = viewport[3];
}

void GLWidget::renderAtFPS( const bool render_at_fps )
{
  m_render_at_fps = render_at_fps;
}

void GLWidget::lockCamera( const bool lock_camera )
{
  m_lock_camera = lock_camera;
}

void GLWidget::toggleHUD()
{
  m_display_HUD = !m_display_HUD;

  updateGL();
}

void GLWidget::toggleXYGrid()
{
  m_display_xy_grid = !m_display_xy_grid;

  updateGL();
}

void GLWidget::toggleYZGrid()
{
  m_display_yz_grid = !m_display_yz_grid;
  
  updateGL();
}

void GLWidget::toggleXZGrid()
{
  m_display_xz_grid = !m_display_xz_grid;
  
  updateGL();
}

void GLWidget::centerCamera()
{
  if( m_lock_camera ) return;

  scalar radius;
  Eigen::Matrix<GLdouble,3,1> center;
  m_sim.computeBoundingSphere( radius, center );
  m_camera_controller.centerCameraAtSphere( center, radius );

  updateGL();
}

void GLWidget::saveScreenshot( const QString& file_name )
{
  std::cout << "Saving screenshot of time " << m_iteration * m_dt << " to " << file_name.toStdString() << std::endl;
  QImage frame_buffer = grabFrameBuffer();
  frame_buffer.save( file_name );
}

void GLWidget::setMovieDir( const QString& dir_name )
{
  m_movie_dir_name = dir_name;
  m_output_frame = 0;

  // Save a screenshot of the current state
  if( m_movie_dir_name.size() != 0 )
  {
    m_movie_dir.setPath( m_movie_dir_name );
    assert( m_movie_dir.exists() );

    QString output_image_name = QString( tr("frame%1.png") ).arg( m_output_frame, 10, 10, QLatin1Char('0') );
    saveScreenshot( m_movie_dir.filePath( output_image_name ) );
    ++m_output_frame;
  }
}

void GLWidget::setMovieFPS( const int fps )
{
  assert( fps > 0 );
  m_output_fps = fps;
  m_steps_per_frame = floor( 1.0 / ( m_dt * m_output_fps ) + 0.5 );
  //std::cout << "steps per frame: " << m_steps_per_frame << std::endl;
  //std::cout << "    " << 1.0 / ( m_rod_sim.timestep() * m_output_fps ) << std::endl;
}

void GLWidget::exportCameraSettings()
{
  std::cout << "<camera theta=\"" << m_camera_controller.theta() << "\" phi=\"" << m_camera_controller.phi() << "\" rho=\"";
  std::cout << m_camera_controller.rho() << "\" lookat=\"" << m_camera_controller.lookat().transpose() << "\" up=\"" << m_camera_controller.up().transpose() << "\"/>" << std::endl;
}

void GLWidget::paintSphere( const RigidBodySphere& sphere, const Vector3s& color ) const
{
  const Eigen::Matrix<GLfloat,3,1> primary_color = color.cast<GLfloat>();
  const Eigen::Matrix<GLfloat,3,1> secondary_color( 1.0, 1.0, 1.0 );
  glScaled( sphere.r(), sphere.r(), sphere.r() );
  assert( m_sphere_renderer != NULL );
  m_sphere_renderer->drawVertexArray( primary_color, secondary_color );
}

void GLWidget::paintBox( const RigidBodyBox& box, const Vector3s& color ) const
{
  // Vertices of the box
//  Vector3s v0( box_ptr.wx(), box_ptr.wy(), box_ptr.wz()); v0 *= 0.5;
//  Vector3s v1( box_ptr.wx(), box_ptr.wy(),-box_ptr.wz()); v1 *= 0.5;
//  Vector3s v2(-box_ptr.wx(), box_ptr.wy(),-box_ptr.wz()); v2 *= 0.5;
//  Vector3s v3(-box_ptr.wx(), box_ptr.wy(), box_ptr.wz()); v3 *= 0.5;
  
//  Vector3s v4( box_ptr.wx(),-box_ptr.wy(), box_ptr.wz()); v4 *= 0.5;
//  Vector3s v5( box_ptr.wx(),-box_ptr.wy(),-box_ptr.wz()); v5 *= 0.5;
//  Vector3s v6(-box_ptr.wx(),-box_ptr.wy(),-box_ptr.wz()); v6 *= 0.5;
//  Vector3s v7(-box_ptr.wx(),-box_ptr.wy(), box_ptr.wz()); v7 *= 0.5;
  
//  GLfloat mcolorambient[] = { 0.8*m_scene->getBodyColor(bdy_idx,0), 0.8*m_scene->getBodyColor(bdy_idx,1), 0.8*m_scene->getBodyColor(bdy_idx,2), 1.0f };
//  glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient );
//  GLfloat mcolordiffuse[] = { 0.9*m_scene->getBodyColor(bdy_idx,0), 0.9*m_scene->getBodyColor(bdy_idx,1), 0.9*m_scene->getBodyColor(bdy_idx,2), 1.0f };
//  glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse );

  GLfloat mcolorambient[] = { (GLfloat) (0.8 * color.x()), (GLfloat) (0.8 * color.y()), (GLfloat) (0.8 * color.z()), (GLfloat) 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient );
  GLfloat mcolordiffuse[] = { (GLfloat) (0.9 * color.x()), (GLfloat) (0.9 * color.y()), (GLfloat) (0.9 * color.z()), (GLfloat) 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse );
  
  glScaled( box.halfWidths().x(), box.halfWidths().y(), box.halfWidths().z() );
  
  glBegin(GL_QUADS);
    // Top of the box
    glNormal3d((GLdouble)0.0,(GLdouble)1.0,(GLdouble)0.0);
    glVertex3d((GLdouble)1.0,(GLdouble)1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)1.0,(GLdouble)1.0);

    // Bottom of the box
    glNormal3d((GLdouble)0.0,(GLdouble)-1.0,(GLdouble)0.0);
    glVertex3d((GLdouble)1.0,(GLdouble)-1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)-1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)-1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)-1.0,(GLdouble)1.0);

    // Positive x side of the box
    glNormal3d((GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
    glVertex3d((GLdouble)1.0,(GLdouble)-1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)-1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)1.0,(GLdouble)1.0);

    // Negative x side of the box
    glNormal3d((GLdouble)-1.0,(GLdouble)0.0,(GLdouble)0.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)-1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)-1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)1.0,(GLdouble)1.0);

    // Positive z side of the box
    glNormal3d((GLdouble)0.0,(GLdouble)0.0,(GLdouble)1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)-1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)-1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)1.0,(GLdouble)1.0);

    // Negative z side of the box
    glNormal3d((GLdouble)0.0,(GLdouble)0.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)-1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)-1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)1.0,(GLdouble)-1.0);
  glEnd();
}

void GLWidget::paintTriangleMesh( const RigidBodyTriangleMesh& mesh, const Vector3s& color ) const
{
  GLfloat mcolorambient[] = { (GLfloat) (0.8 * color.x()), (GLfloat) (0.8 * color.y()), (GLfloat) (0.8 * color.z()), (GLfloat) 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient );
  GLfloat mcolordiffuse[] = { (GLfloat) (0.9 * color.x()), (GLfloat) (0.9 * color.y()), (GLfloat) (0.9 * color.z()), (GLfloat) 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse );

  const Matrix3Xsc& vertices = mesh.vertices();
  const Matrix3Xuc& faces = mesh.faces();

  glBegin(GL_TRIANGLES);
  for( int i = 0; i < faces.cols(); ++i )
  {
    // Compute a normal for this face
    const Vector3s n = ( vertices.col( faces(1,i) ) - vertices.col( faces(0,i) ) ).cross( vertices.col( faces(2,i) ) - vertices.col( faces(0,i) ) ).normalized();
    // Send the normal to OpenGL
    glNormal3f( (GLfloat) n.x(), (GLfloat) n.y(), (GLfloat) n.z() );
    // Send the vertices to OpenGL
    glVertex3f( (GLfloat) vertices( 0, faces(0,i) ), (GLfloat) vertices( 1, faces(0,i) ), (GLfloat) vertices( 2, faces(0,i) ) );
    glVertex3f( (GLfloat) vertices( 0, faces(1,i) ), (GLfloat) vertices( 1, faces(1,i) ), (GLfloat) vertices( 2, faces(1,i) ) );
    glVertex3f( (GLfloat) vertices( 0, faces(2,i) ), (GLfloat) vertices( 1, faces(2,i) ), (GLfloat) vertices( 2, faces(2,i) ) );
  }
  glEnd();
}

void GLWidget::paintBody( const int geo_idx, const RigidBodyGeometry* geometry, const Vector3s& color ) const
{
  assert( geometry != NULL );
  // Determine which geometry type this is and execute the specific rendering routine
  
  if( geometry->getType() == RigidBodyGeometry::BOX )
  {
    const RigidBodyBox& box_geom = *dynamic_cast<const RigidBodyBox*>( geometry );
    paintBox( box_geom, color );
  }
  else if( geometry->getType() == RigidBodyGeometry::SPHERE )
  {
    const RigidBodySphere& sphere_geom = *dynamic_cast<const RigidBodySphere*>( geometry );
    paintSphere( sphere_geom, color );
  }
  else if( geometry->getType() == RigidBodyGeometry::STAPLE )
  {
    assert( geo_idx >= 0 ); assert( geo_idx < (int) m_body_renderers.size() );
    assert( m_body_renderers[geo_idx] != NULL );
    m_body_renderers[geo_idx]->renderBody( color.cast<GLfloat>() );
  }
  else if( geometry->getType() == RigidBodyGeometry::TRIANGLE_MESH )
  {
    const RigidBodyTriangleMesh& triangle_geom = *dynamic_cast<const RigidBodyTriangleMesh*>( geometry );
    paintTriangleMesh( triangle_geom, color );
  }
  else
  {
    std::cerr << "Invalid geometry type encountered in GLWidget::paintBody. This is bug. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
}

void GLWidget::paintSystem() const
{
  const RigidBodySimState& state = m_sim.getState();

  // Draw each body
  assert( state.q().size() == 12 * state.nbodies() );
  for( int i = 0; i < state.nbodies(); ++i )
  {
    glPushMatrix();
    // Translate to center of mass position
    glTranslated( state.q()( 3 * i + 0 ), state.q()( 3 * i + 1 ), state.q()( 3 * i + 2 ) );
    // Rotate about center of mass
    Eigen::Matrix<GLdouble,4,4,Eigen::ColMajor> gl_rot_mat;
    gl_rot_mat.setZero();
    gl_rot_mat.block<3,3>(0,0) = Eigen::Map<const Matrix33sr>( state.q().segment<9>( 3 * state.nbodies() + 9 * i ).data() );
    gl_rot_mat(3,3) = 1.0;
    glMultMatrixd( gl_rot_mat.data() );
    // Draw the geometry
    paintBody( state.getGeometryIndexOfBody( i ), state.getGeometryOfBody( i ), m_body_colors.segment<3>( 3 * i ) );
    glPopMatrix();
  }

  // Draw any static planes
  for( std::vector<StaticPlaneRenderer>::size_type i = 0; i < m_plane_renderers.size(); ++i )
  {
    assert( m_plane_renderers[i].idx() < m_sim.state().staticPlanes().size() );
    m_plane_renderers[i].draw( m_sim.state().staticPlanes()[ m_plane_renderers[i].idx() ] );
  }
  // Draw any static cylinders
  for( std::vector<StaticCylinderRenderer>::size_type i = 0; i < m_cylinder_renderers.size(); ++i )
  {
    assert( m_cylinder_renderers[i].idx() >= 0 ); assert( m_cylinder_renderers[i].idx() < (int) m_sim.state().staticCylinders().size() );
    m_cylinder_renderers[i].draw( m_sim.state().staticCylinders()[ m_cylinder_renderers[i].idx() ] );
  }
  // Draw any planar portals
  glPushAttrib( GL_COLOR );
  glPushAttrib( GL_LINE_WIDTH );
  glPushAttrib( GL_LIGHTING );
  glDisable( GL_LIGHTING );
  glLineWidth( 2.0 );
  {
    MersenneTwister random_number_gen( 123456 );
    for( std::vector<PlanarPortalRenderer>::size_type i = 0; i < m_portal_renderers.size(); ++i )
    {
      double r = random_number_gen.genrand_real1() * 255.0;
      double g = random_number_gen.genrand_real1() * 255.0;
      double b = random_number_gen.genrand_real1() * 255.0;
      qglColor( QColor( (int) r, (int) g, (int) b ) );
      assert( m_portal_renderers[i].idx() < m_sim.state().numPlanarPortals() );
      m_portal_renderers[i].draw( m_sim.state().planarPortal( m_portal_renderers[i].idx() ) );
    }
  }
  glPopAttrib();
  glPopAttrib();
  glPopAttrib();
}

void GLWidget::paintHUD()
{
  // String to display in upper left corner
  const QString time_string  = QString( tr(" t: ") ) + QString::number( m_iteration * m_dt, 'f', m_display_precision ) + ( m_end_time != SCALAR_INFINITY ? QString( tr(" / ") ) + QString::number( m_end_time ) : QString( tr("") ) );
  const QString delta_H      = QString( tr("dH: ") ) + QString::number( m_delta_H0 );
  const QString delta_p      = QString( tr("dp: ") ) + QString::number( m_delta_p0.x() ) + " " + QString::number( m_delta_p0.y() ) + " " + QString::number( m_delta_p0.z() );
  const QString delta_L      = QString( tr("dL: ") ) + QString::number( m_delta_L0.x() ) + " " + QString::number( m_delta_L0.y() ) + " " + QString::number( m_delta_L0.z() );

  qglColor( QColor( 0, 0, 0 ) );
  QFont font("Courier");
  //font.setStyleHint( QFont::TypeWriter );
  font.setPointSize( 12 );
  renderText( 2, font.pointSize(), time_string, font );
  renderText( 2, 2 * font.pointSize(), delta_H, font );
  renderText( 2, 3 * font.pointSize(), delta_p, font );
  renderText( 2, 4 * font.pointSize(), delta_L, font );

  assert( checkGLErrors() );
}

void GLWidget::mousePressEvent( QMouseEvent* event )
{
  if( m_lock_camera ) return;
  
  bool repaint_needed = false;

  if( event->buttons() & Qt::LeftButton )
  {
    m_left_mouse_button_pressed = true;
    repaint_needed = true;
  }
  if( event->buttons() & Qt::RightButton )
  {
    m_right_mouse_button_pressed = true;
  }

  if( repaint_needed ) updateGL();

  m_last_pos = event->pos();
}

void GLWidget::mouseReleaseEvent( QMouseEvent* event )
{
  if( m_lock_camera ) return;

  bool repaint_needed = false;

  if( !( event->buttons() & Qt::LeftButton ) && m_left_mouse_button_pressed )
  {
    m_left_mouse_button_pressed = false;
    repaint_needed = true;
  }
  if( !( event->buttons() & Qt::RightButton ) && m_right_mouse_button_pressed )
  {
    m_right_mouse_button_pressed = false;
  }

  if( repaint_needed ) updateGL();
}

void GLWidget::mouseMoveEvent( QMouseEvent* event )
{
  if( m_lock_camera ) return;
  
  const int dx = event->x() - m_last_pos.x();
  const int dy = event->y() - m_last_pos.y();
  m_last_pos = event->pos();

  if( event->buttons() & Qt::LeftButton )
  {
    const GLdouble horizontal_amount = 0.01*(GLdouble)dx;
    const GLdouble vertical_amount =   0.01*(GLdouble)dy;
    m_camera_controller.addToZenithAngle( -vertical_amount );
    m_camera_controller.addToAzimuthAngle( -horizontal_amount );
    updateGL();
  }
  else if( event->buttons() & Qt::MidButton )
  {
    const GLdouble horizontal_amount = 0.002*(GLdouble)dx;
    const GLdouble vertical_amount =   0.002*(GLdouble)dy;
    m_camera_controller.trackCameraHorizontal( -m_camera_controller.getDistFromCenter()*horizontal_amount );
    m_camera_controller.trackCameraVertical( m_camera_controller.getDistFromCenter()*vertical_amount  );
    updateGL();
  }
  else if( event->buttons() & Qt::RightButton )
  {
    const GLdouble horizontal_amount = 0.01*(GLdouble)dx;
    const GLdouble vertical_amount =   0.01*(GLdouble)dy;
    m_camera_controller.addToDistFromCenter( m_camera_controller.getDistFromCenter()*(horizontal_amount-vertical_amount) );
    updateGL();
  }

  assert( checkGLErrors() );
}

void GLWidget::wheelEvent( QWheelEvent* event )
{
  if( m_lock_camera ) return;
  
  m_camera_controller.addToDistFromCenter( -0.002*m_camera_controller.getDistFromCenter()*event->delta() );
  updateGL();

  assert( checkGLErrors() );
}

std::string GLWidget::glErrorToString( const GLenum error_code )
{
  switch( error_code )
  {
    case GL_NO_ERROR:
    return "GL_NO_ERROR";
    break;
    case GL_INVALID_ENUM:
    return "GL_INVALID_ENUM";
    break;
    case GL_INVALID_VALUE:
    return "GL_INVALID_VALUE";
    break;
    case GL_INVALID_OPERATION:
    return "GL_INVALID_OPERATION";
    break;
    case GL_INVALID_FRAMEBUFFER_OPERATION:
    return "GL_INVALID_FRAMEBUFFER_OPERATION";
    break;
    case GL_OUT_OF_MEMORY:
    return "GL_OUT_OF_MEMORY";
    break;
    case GL_STACK_UNDERFLOW:
    return "GL_STACK_UNDERFLOW";
    break;
    case GL_STACK_OVERFLOW:
    return "GL_STACK_OVERFLOW";
    break;
    default:
    return "Unknown error. Please contact the maintainer of this code.";
    break;
  }
}

bool GLWidget::checkGLErrors()
{
  const GLenum error_code = glGetError();
  if( error_code != GL_NO_ERROR )
  {
    std::cerr << "OpenGL error: " << glErrorToString( error_code ) << std::endl;
    return false;
  }
  return true;
}

ThreeDRigidBodySim* GLWidget::get_sim() {
	return &m_sim;
}

RigidBodySimState* GLWidget::get_sim_state_backup() {
	return &m_sim_state_backup;
}
