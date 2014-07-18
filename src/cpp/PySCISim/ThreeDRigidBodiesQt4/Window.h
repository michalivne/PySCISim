// Window.h
//
// Breannan Smith
// Last updated: 02/25/2014

#ifndef __WINDOW_H__
#define __WINDOW_H__

#include <QMainWindow>

class ContentWidget;

class Window : public QMainWindow
{

public:

  Window( QWidget* parent = NULL );

  void keyPressEvent( QKeyEvent* event );

  ContentWidget* get_content_widget();

protected:

  void closeEvent( QCloseEvent* event );

private:

  ContentWidget* m_content_widget;

};

#endif
