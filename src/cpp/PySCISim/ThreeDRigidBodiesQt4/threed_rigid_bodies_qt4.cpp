#include <QApplication>
#include <QDesktopWidget>

#include "Window.h"

void centerWindow( Window& window )
{
  QDesktopWidget* desktop = QApplication::desktop();

  const int screenWidth = desktop->screenGeometry().width();
  const int screenHeight = desktop->screenGeometry().height();

  const QSize windowSize = window.size();
  const int width = windowSize.width();
  const int height = windowSize.height();

  const int x = (screenWidth - width) / 2;
  const int y = (screenHeight - height) / 2;

  window.move( x, y );
}

int main( int argc, char **argv )
{
  //QApplication::setGraphicsSystem("opengl");
  QApplication app(argc, argv);
  Window window;
  window.resize(window.sizeHint());
  window.setWindowTitle( "Three Dimensional Rigid Body Simulation" );
  centerWindow( window );
  window.show();
  window.raise();
  return app.exec();
}

