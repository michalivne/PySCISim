// ContentWidget.cpp
//
// Breannan Smith
// Last updated: 12/10/2014

#include "ContentWidget.h"

#include "GLWidget.h"
#include "EnergyWindow.h"
#include <iostream>
ContentWidget::ContentWidget( QWidget* parent )
: QWidget( parent )
, m_idle_timer( NULL )
, m_gl_widget( new GLWidget( this ) )
, m_energy_plot( NULL )
, m_xml_file_name()
{
  QVBoxLayout* mainLayout = new QVBoxLayout;
  mainLayout->addWidget(m_gl_widget);

  // Layout for controls
  QGridLayout* controls_layout= new QGridLayout;

  // Solver buttons
  m_simulate_checkbox = new QCheckBox( tr("Simulate") );
  m_simulate_checkbox->setChecked( false );
  controls_layout->addWidget( m_simulate_checkbox, 0, 0 );
  connect( m_simulate_checkbox, SIGNAL( toggled(bool) ), this, SLOT( simulateToggled(bool) ) );

  QPushButton* step_button = new QPushButton( tr("Step"), this );
  controls_layout->addWidget( step_button, 0, 1 );
  connect( step_button, SIGNAL( clicked() ), this, SLOT( takeStep() ) );

  QPushButton* reset_button = new QPushButton( tr("Reset"), this );
  controls_layout->addWidget( reset_button, 0, 2 );
  connect( reset_button, SIGNAL( clicked() ), this, SLOT( resetSystem() ) );

  // Button for rendering at the specified FPS
  m_render_at_fps_checkbox = new QCheckBox( tr("Render FPS") );
  controls_layout->addWidget( m_render_at_fps_checkbox, 0, 3 );
  connect( m_render_at_fps_checkbox, SIGNAL( toggled(bool) ), this, SLOT( renderAtFPSToggled(bool) ) );
  
  // Buttons for locking the camera controls
  m_lock_camera_button = new QCheckBox( tr("Lock Camera") );
  controls_layout->addWidget( m_lock_camera_button, 1, 0 );
  connect( m_lock_camera_button, SIGNAL( toggled(bool) ), this, SLOT( lockCameraToggled(bool) ) );

  // Buttons for displaying the energy plot
  m_energy_plot_button = new QCheckBox( tr("Plot Energy") );
  controls_layout->addWidget( m_energy_plot_button, 1, 1 );
  connect( m_energy_plot_button, SIGNAL( toggled(bool) ), this, SLOT( toggleEnergyWindow(bool) ) );

  // Movie export controls
  m_export_movie_checkbox = new QCheckBox( tr("Export Movie") );
  m_export_movie_checkbox->setChecked( false );
  controls_layout->addWidget( m_export_movie_checkbox, 1, 2 );
  connect( m_export_movie_checkbox, SIGNAL( toggled(bool) ), this, SLOT( exportMovieToggled(bool) ) );

  // Label for movie output FPS
  QLabel* fps_label = new QLabel( this );
  fps_label->setText( QString("FPS:") );
  controls_layout->addWidget( fps_label, 1, 3 );
  
  // Input for movie output FPS
  m_fps_spin_box = new QSpinBox( this );
  m_fps_spin_box->setRange( 1, 1000 );
  m_fps_spin_box->setValue( 60 );
  controls_layout->addWidget( m_fps_spin_box, 1, 4 );
  connect( m_fps_spin_box, SIGNAL( valueChanged(int) ), this, SLOT( movieFPSChanged(int) ) );
  m_gl_widget->setMovieFPS( m_fps_spin_box->value() );
  
  mainLayout->addLayout( controls_layout );

  setLayout( mainLayout );

  // Create a timer that triggers simulation steps when Qt4 is idle
  m_idle_timer = new QTimer( this );
  connect( m_idle_timer, SIGNAL( timeout() ), this, SLOT( takeStep() ) );

  m_energy_plot = new EnergyWindow;

  connect( m_energy_plot, SIGNAL( widgetVisibilityChanged(bool) ), this, SLOT( toggleEnergyWindow(bool) ) );
}

void ContentWidget::toggleSimulationCheckbox()
{
  assert( m_simulate_checkbox != NULL );
  m_simulate_checkbox->toggle();
}

void ContentWidget::disableMovieExport()
{
  assert( m_export_movie_checkbox != NULL );
  m_export_movie_checkbox->setCheckState( Qt::Unchecked );
}

void ContentWidget::takeStep()
{
  // Step the system
  assert( m_gl_widget != NULL );
  m_gl_widget->stepSystem();
  
  // Update the energy plot
  pushDataPointToGraph();
}

void ContentWidget::resetSystem()
{
  assert( m_gl_widget != NULL );
  m_gl_widget->resetSystem();

  // Reset the plot and push the initial state
  m_energy_plot->reset();
  pushDataPointToGraph();
}

void ContentWidget::openScene()
{
  // Obtain a file name from the user
  const QString xml_scene_file_name = getOpenFileNameFromUser( tr("Please Select a Scene File") );

  // Determine whether the requested file exists
  const bool file_exists = QFile::exists( xml_scene_file_name );

  // If the user provided a valid file
  if( file_exists )
  {
    // Attempt to load the file
    assert( m_gl_widget != nullptr );
    unsigned fps;
    bool render_at_fps;
    bool lock_camera;
    const bool successfully_loaded = m_gl_widget->openScene( xml_scene_file_name, fps, render_at_fps, lock_camera );

    // If the load was successful
    if( successfully_loaded )
    {
      // Update the energy plot
      assert( m_energy_plot != nullptr );
      m_energy_plot->reset();
      pushDataPointToGraph();

      // Make sure the simulation isn't running when we start
      assert( m_simulate_checkbox != nullptr );
      if( m_simulate_checkbox->isChecked() )
      {
        toggleSimulationCheckbox();
      }

      // Update UI elements
      assert( m_fps_spin_box != nullptr );
      m_fps_spin_box->setValue( fps );
      assert( m_render_at_fps_checkbox != nullptr );
      m_render_at_fps_checkbox->setCheckState( render_at_fps ? Qt::Checked : Qt::Unchecked );
      assert( m_lock_camera_button != nullptr );
      m_lock_camera_button->setCheckState( lock_camera ? Qt::Checked : Qt::Unchecked );

      m_xml_file_name = xml_scene_file_name;

      disableMovieExport();
    }
  }
  else
  {
    std::cerr << "Error, requested file " << xml_scene_file_name.toStdString() << " does not exist." << std::endl;
  }

  this->setFocus();
}

void ContentWidget::reloadScene()
{
  // Determine whether the requested file still exists
  const bool file_exists = QFile::exists( m_xml_file_name );

  // If the user provided a valid file
  if( file_exists )
  {
    // Attempt to load the file
    assert( m_gl_widget != nullptr );
    unsigned fps;
    bool render_at_fps;
    bool lock_camera;
    const bool successfully_loaded = m_gl_widget->openScene( m_xml_file_name, fps, render_at_fps, lock_camera );

    // If the load was successful
    if( successfully_loaded )
    {
      // Update the energy plot
      assert( m_energy_plot != nullptr );
      m_energy_plot->reset();
      pushDataPointToGraph();

      // Make sure the simulation isn't running when we start
      assert( m_simulate_checkbox != nullptr );
      if( m_simulate_checkbox->isChecked() )
      {
        toggleSimulationCheckbox();
      }

      // Update UI elements
      assert( m_fps_spin_box != nullptr );
      m_fps_spin_box->setValue( fps );
      assert( m_render_at_fps_checkbox != nullptr );
      m_render_at_fps_checkbox->setCheckState( render_at_fps ? Qt::Checked : Qt::Unchecked );
      assert( m_lock_camera_button != nullptr );
      m_lock_camera_button->setCheckState( lock_camera ? Qt::Checked : Qt::Unchecked );

      disableMovieExport();
    }
  }
  else
  {
    std::cerr << "Error, file " << m_xml_file_name.toStdString() << " no longer exists." << std::endl;
  }
  
  this->setFocus();
}

void ContentWidget::simulateToggled( const bool state )
{
  assert( m_idle_timer != NULL );

  // Start the idle timer
  if( state )
  {
    m_idle_timer->start(0);
  }
  // Stop the idle timer
  else
  {
    m_idle_timer->stop();
  }
}

void ContentWidget::renderAtFPSToggled( const bool render_at_fps )
{
  assert( m_gl_widget != NULL );
  m_gl_widget->renderAtFPS( render_at_fps );
}

void ContentWidget::lockCameraToggled( const bool lock_camera )
{
  assert( m_gl_widget != NULL );
  m_gl_widget->lockCamera( lock_camera );
}

void ContentWidget::exportMovieToggled( const bool checked )
{
  assert( m_gl_widget != NULL );

  if( checked )
  {
    // Attempt to get a directory name
    const QString dir_name = getDirectoryNameFromUser( tr("Please Specify an Image Directory") );
    if( dir_name.size() != 0 )
    {
      m_gl_widget->setMovieDir( dir_name );
    }
    else
    {
      assert( m_export_movie_checkbox != NULL );
      m_export_movie_checkbox->toggle();
    }
    this->setFocus();
  }
  else
  {
    m_gl_widget->setMovieDir( tr("") );
  }
}

void ContentWidget::toggleEnergyWindow( const bool state )
{
  if( state )
  {
    m_energy_plot->regeneratePlot();
    m_energy_plot->resize( size() );
    m_energy_plot->setWindowTitle( tr("Conserved Quantities") );
    m_energy_plot->show();
    m_energy_plot->raise();
  }
  else
  {
    m_energy_plot->hide();
    m_energy_plot_button->setChecked( false );
  }
}

void ContentWidget::toggleHUD()
{
  assert( m_gl_widget != NULL );
  m_gl_widget->toggleHUD();
}

void ContentWidget::toggleXYGrid()
{
  assert( m_gl_widget != NULL );
  m_gl_widget->toggleXYGrid();
}

void ContentWidget::toggleYZGrid()
{
  assert( m_gl_widget != NULL );
  m_gl_widget->toggleYZGrid();
}

void ContentWidget::toggleXZGrid()
{
  assert( m_gl_widget != NULL );
  m_gl_widget->toggleXZGrid();
}

void ContentWidget::centerCamera()
{
  assert( m_gl_widget != NULL );
  m_gl_widget->centerCamera();
}

void ContentWidget::exportImage()
{
  assert( m_gl_widget != NULL );
  const QString file_name = getSaveFileNameFromUser( tr("Please Specify an Image Name") );
  if( file_name.size() != 0 ) m_gl_widget->saveScreenshot( file_name );
  this->setFocus();
}

void ContentWidget::exportMovie()
{
  assert( m_export_movie_checkbox != NULL );
  m_export_movie_checkbox->setChecked( false );
  m_export_movie_checkbox->setChecked( true );
}

void ContentWidget::movieFPSChanged( int fps )
{
  m_gl_widget->setMovieFPS( fps );
}

void ContentWidget::exportCameraSettings()
{
  m_gl_widget->exportCameraSettings();
}

void ContentWidget::enablePerspectiveCamera()
{
  m_gl_widget->enablePerspectiveCamera();
}

void ContentWidget::enableOrthographicXYCamera()
{
  m_gl_widget->enableOrthographicXYCamera();
}

void ContentWidget::enableOrthographicZYCamera()
{
  m_gl_widget->enableOrthographicZYCamera();
}

void ContentWidget::enableOrthographicZXCamera()
{
  m_gl_widget->enableOrthographicZXCamera();
}

void ContentWidget::closeEvent( QCloseEvent* event )
{
  m_energy_plot->close();
}

void ContentWidget::pushDataPointToGraph()
{
  // Update the energy plot
  double time, T, U;
  Eigen::Vector3d p;
  Eigen::Vector3d L;
  m_gl_widget->getSimData( time, T, U, p, L );
  assert( m_energy_plot != NULL );
  m_energy_plot->pushData( time, T, U, p, L );

  // Generate the plot if it is displayed
  if( m_energy_plot_button->isChecked() ) m_energy_plot->regeneratePlot();
}

QString ContentWidget::getOpenFileNameFromUser( const QString& prompt )
{
  QString file_name = QFileDialog::getOpenFileName( this, prompt );
  activateWindow();
  return file_name;
}

QString ContentWidget::getSaveFileNameFromUser( const QString& prompt )
{
  QString file_name = QFileDialog::getSaveFileName( this, prompt );
  activateWindow();
  return file_name;
}

QString ContentWidget::getDirectoryNameFromUser( const QString& prompt )
{
  QString file_name = QFileDialog::getExistingDirectory(this, prompt );
  activateWindow();
  return file_name;
}

GLWidget* ContentWidget::get_gl_widget() {
  return m_gl_widget;
}
