#include "EnergyWindow.h"

#include <iostream>

EnergyWindow::EnergyWindow( QWidget* parent )
: QWidget( parent )
, m_max_points( 100000 ) // ~ 1/10 of a megabyte per plot. Should be plenty.
, m_energy_plot( NULL )
, m_momentum_plot( NULL )
, m_angular_momentum_plot( NULL )
, m_time_range( SCALAR_INFINITY, -SCALAR_INFINITY )
, m_energy_range( SCALAR_INFINITY, -SCALAR_INFINITY )
, m_momentum_range( SCALAR_INFINITY, -SCALAR_INFINITY )
, m_angular_momentum_range( SCALAR_INFINITY, -SCALAR_INFINITY )
, m_needs_replot(true)
{  
  QVBoxLayout* mainLayout = new QVBoxLayout;
  
  // Styles for the plots
  QPen red_pen;
  red_pen.setColor( QColor( 255, 0, 0 ) );
  red_pen.setWidthF( 2.0 );
  QPen green_pen;
  green_pen.setColor( QColor( 0, 255, 0 ) );
  green_pen.setWidthF( 2.0 );
  QPen blue_pen;
  blue_pen.setColor( QColor( 0, 0, 255 ) );
  blue_pen.setWidthF( 2.0 );

  // Energy plot
  m_energy_plot = new QCustomPlot;
  m_energy_plot->xAxis->setLabel( tr("Time") );
  m_energy_plot->yAxis->setLabel( tr("Energy") );
  m_energy_plot->addGraph();
  m_energy_plot->addGraph();
  m_energy_plot->addGraph();
  m_energy_plot->graph(0)->setName( tr("Kinetic Energy") );
  m_energy_plot->graph(1)->setName( tr("Potential Energy") );
  m_energy_plot->graph(2)->setName( tr("Total Energy") );
  m_energy_plot->graph(0)->setPen(red_pen);
  m_energy_plot->graph(1)->setPen(green_pen);
  m_energy_plot->graph(2)->setPen(blue_pen);
  m_energy_plot->legend->setVisible(true);

  //m_energy_plot->graph(0)->setAntialiased(false);
  //m_energy_plot->graph(1)->setAntialiased(false);
  //m_energy_plot->graph(2)->setAntialiased(false);

  // Momentum plot
  m_momentum_plot = new QCustomPlot;
  m_momentum_plot->xAxis->setLabel("Time");
  m_momentum_plot->yAxis->setLabel("Momentum");
  m_momentum_plot->addGraph();
  m_momentum_plot->addGraph();
  m_momentum_plot->addGraph();
  m_momentum_plot->graph(0)->setName( tr("P_x") );
  m_momentum_plot->graph(1)->setName( tr("P_y") );
  m_momentum_plot->graph(2)->setName( tr("P_z") );
  m_momentum_plot->graph(0)->setPen(red_pen);
  m_momentum_plot->graph(1)->setPen(green_pen);
  m_momentum_plot->graph(2)->setPen(blue_pen);
  m_momentum_plot->legend->setVisible(true);

  //m_momentum_plot->graph(0)->setAntialiased(false);
  //m_momentum_plot->graph(1)->setAntialiased(false);

  // Angular momentum plot
  m_angular_momentum_plot = new QCustomPlot;
  m_angular_momentum_plot->xAxis->setLabel("Time");
  m_angular_momentum_plot->yAxis->setLabel("Angular Momentum");
  m_angular_momentum_plot->addGraph();
  m_angular_momentum_plot->addGraph();
  m_angular_momentum_plot->addGraph();
  m_angular_momentum_plot->graph(0)->setName( tr("L_x") );
  m_angular_momentum_plot->graph(1)->setName( tr("L_y") );
  m_angular_momentum_plot->graph(2)->setName( tr("L_z") );
  m_angular_momentum_plot->graph(0)->setPen(red_pen);
  m_angular_momentum_plot->graph(1)->setPen(green_pen);
  m_angular_momentum_plot->graph(2)->setPen(blue_pen);
  m_angular_momentum_plot->legend->setVisible(true);

  //m_angular_momentum_plot->graph(0)->setAntialiased(false);

  mainLayout->addWidget(m_energy_plot);
  mainLayout->addWidget(m_momentum_plot);
  mainLayout->addWidget(m_angular_momentum_plot);
  
  setLayout(mainLayout);
}

void EnergyWindow::reset()
{
  m_energy_plot->graph(0)->clearData();
  m_energy_plot->graph(1)->clearData();
  m_energy_plot->graph(2)->clearData();
  m_momentum_plot->graph(0)->clearData();
  m_momentum_plot->graph(1)->clearData();
  m_momentum_plot->graph(2)->clearData();
  m_angular_momentum_plot->graph(0)->clearData();
  m_angular_momentum_plot->graph(1)->clearData();
  m_angular_momentum_plot->graph(2)->clearData();

  m_time_range << SCALAR_INFINITY, -SCALAR_INFINITY;
  m_energy_range << SCALAR_INFINITY, -SCALAR_INFINITY;
  m_momentum_range << SCALAR_INFINITY, -SCALAR_INFINITY;
  m_angular_momentum_range << SCALAR_INFINITY, -SCALAR_INFINITY;

  m_needs_replot = true;
}

void EnergyWindow::pushData( const double& time, const double& T, const double& U, const Eigen::Vector3d& p, const Eigen::Vector3d& L )
{
  // Time is employed by all plots
  //m_time_range(0) = std::min( m_time_range(0), time );
  //m_time_range(1) = std::max( m_time_range(1), time );

  // Add this data point to the energy plot
  m_energy_range(0) = std::min( m_energy_range(0), T );
  m_energy_range(1) = std::max( m_energy_range(1), T );
  m_energy_range(0) = std::min( m_energy_range(0), U );
  m_energy_range(1) = std::max( m_energy_range(1), U );
  m_energy_range(0) = std::min( m_energy_range(0), T+U );
  m_energy_range(1) = std::max( m_energy_range(1), T+U );
  // If the range is 0, simply inflate by some arbitrary number. Let's use 1/2.
  if( m_energy_range(0) == m_energy_range(1) )
  {
    m_energy_range(0) -= 0.5;
    m_energy_range(1) += 0.5;
  }

  // Add this data to the momentum plot
  m_momentum_range(0) = std::min( m_momentum_range(0), p.minCoeff() );
  m_momentum_range(1) = std::max( m_momentum_range(1), p.maxCoeff() );
  // If the range is 0, simply inflate by some arbitrary number. Let's use 1/2.
  if( m_momentum_range(0) == m_momentum_range(1) )
  {
    m_momentum_range(0) -= 0.5;
    m_momentum_range(1) += 0.5;
  }

  // Add this data to the angular momentum plot
  m_angular_momentum_range(0) = std::min( m_angular_momentum_range(0), L.minCoeff() );
  m_angular_momentum_range(1) = std::max( m_angular_momentum_range(1), L.maxCoeff() );
  // If the range is 0, simply inflate by some arbitrary number. Let's use 1/2.
  if( m_angular_momentum_range(0) == m_angular_momentum_range(1) )
  {
    m_angular_momentum_range(0) -= 0.5;
    m_angular_momentum_range(1) += 0.5;
  }

  if( m_energy_plot->graph(0)->data()->size() + 1 > m_max_points )
  {
    const double& remove_time = m_energy_plot->graph(0)->data()->keys()[1];
    
    m_energy_plot->graph(0)->removeDataBefore(remove_time);
    m_energy_plot->graph(1)->removeDataBefore(remove_time);
    m_energy_plot->graph(2)->removeDataBefore(remove_time);

    m_momentum_plot->graph(0)->removeDataBefore(remove_time);
    m_momentum_plot->graph(1)->removeDataBefore(remove_time);
    m_momentum_plot->graph(2)->removeDataBefore(remove_time);

    m_angular_momentum_plot->graph(0)->removeDataBefore(remove_time);
    m_angular_momentum_plot->graph(1)->removeDataBefore(remove_time);
    m_angular_momentum_plot->graph(2)->removeDataBefore(remove_time);
  }

  if( m_energy_plot->graph(0)->data()->keys().empty() )
  {
    m_time_range << time, time;
  }
  else
  {
    m_time_range << m_energy_plot->graph(0)->data()->keys().front(), time;
  }

  m_energy_plot->graph(0)->addData( time, T );
  m_energy_plot->graph(1)->addData( time, U );
  m_energy_plot->graph(2)->addData( time, T + U );
  m_energy_plot->xAxis->setRange( m_time_range(0), m_time_range(1) );
  m_energy_plot->yAxis->setRange( m_energy_range(0), m_energy_range(1) );

  m_momentum_plot->graph(0)->addData( time, p.x() );
  m_momentum_plot->graph(1)->addData( time, p.y() );
  m_momentum_plot->graph(2)->addData( time, p.z() );
  m_momentum_plot->xAxis->setRange( m_time_range(0), m_time_range(1) );
  m_momentum_plot->yAxis->setRange( m_momentum_range(0), m_momentum_range(1) );

  m_angular_momentum_plot->graph(0)->addData( time, L.x() );
  m_angular_momentum_plot->graph(1)->addData( time, L.y() );
  m_angular_momentum_plot->graph(2)->addData( time, L.z() );
  m_angular_momentum_plot->xAxis->setRange( m_time_range(0), m_time_range(1) );
  m_angular_momentum_plot->yAxis->setRange( m_angular_momentum_range(0), m_angular_momentum_range(1) );

  m_needs_replot = true;
}

void EnergyWindow::regeneratePlot()
{
  m_energy_plot->replot();
  m_momentum_plot->replot();
  m_angular_momentum_plot->replot();

  m_needs_replot = false;
}

void EnergyWindow::hideEvent( QHideEvent* )
{
  emit widgetVisibilityChanged( false );
}
