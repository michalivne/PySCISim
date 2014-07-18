#ifndef __ENERGY_WINDOW_H__
#define __ENERGY_WINDOW_H__

#include <QWidget>
#include <QtGui>

#include "QCustomPlot/qcustomplot.h"

#include "SDIC/Math/MathDefines.h"

class EnergyWindow : public QWidget
{
  Q_OBJECT

public:

  EnergyWindow( QWidget* parent = NULL );

  void reset();

  void pushData( const double& time, const double& T, const double& U, const Eigen::Vector3d& p, const Eigen::Vector3d& L );

  void regeneratePlot();

signals:

  void widgetVisibilityChanged( const bool visible );

protected:

  void hideEvent(QHideEvent*);

private:

  // Maximum number of data points to track
  int m_max_points;

  // Plot objects
  QCustomPlot* m_energy_plot;
  QCustomPlot* m_momentum_plot;
  QCustomPlot* m_angular_momentum_plot;

  // Track the min and max values for each plot
  Eigen::Vector2d m_time_range;
  Eigen::Vector2d m_energy_range;
  Eigen::Vector2d m_momentum_range;
  Eigen::Vector2d m_angular_momentum_range;

  bool m_needs_replot;
};

#endif
