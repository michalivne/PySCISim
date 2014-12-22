// ContentWidget.h
//
// Breannan Smith
// Last updated: 12/10/2014

#ifndef __CONTENT_WIDGET_H__
#define __CONTENT_WIDGET_H__

#include <QWidget>
#include <QtGui>

class GLWidget;
class EnergyWindow;

class ContentWidget : public QWidget
{

  Q_OBJECT

public:

  ContentWidget( QWidget* parent = NULL );

  void toggleSimulationCheckbox();
  void disableMovieExport();

public slots:

  void takeStep();
  void resetSystem();

  void openScene();
  void reloadScene();

  void simulateToggled( const bool state );
  
  void renderAtFPSToggled( const bool render_at_fps );

  void lockCameraToggled( const bool lock_camera );

  void exportMovieToggled( const bool checked );

  void toggleEnergyWindow( const bool state );

  void toggleHUD();
  void toggleXYGrid();
  void toggleYZGrid();
  void toggleXZGrid();
  void centerCamera();

  void exportImage();
  void exportMovie();
  void movieFPSChanged( int fps );

  void exportCameraSettings();

  void enablePerspectiveCamera();
  void enableOrthographicXYCamera();
  void enableOrthographicZYCamera();
  void enableOrthographicZXCamera();

  GLWidget* get_gl_widget();

protected:

  void closeEvent( QCloseEvent* event );

private:

  void pushDataPointToGraph();

  QString getOpenFileNameFromUser( const QString& prompt );
  QString getSaveFileNameFromUser( const QString& prompt );
  QString getDirectoryNameFromUser( const QString& prompt );

  QTimer* m_idle_timer;
  GLWidget* m_gl_widget;
  QCheckBox* m_simulate_checkbox;

  QCheckBox* m_render_at_fps_checkbox;
  
  QCheckBox* m_lock_camera_button;

  QCheckBox* m_energy_plot_button;
  EnergyWindow* m_energy_plot;

  QCheckBox* m_export_movie_checkbox;

  QString m_xml_file_name;

  QSpinBox* m_fps_spin_box;

};

#endif
