#include "sweep/cdt.h"
#include <QtWidgets>

int qtGui(){
  int c = 0;
  QApplication app(c, NULL);
  auto window = new QFileDialog;
  window->show();
  app.exec();
  return 0;
}

