#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    vtkOpenGLRenderWindow::SetGlobalMaximumNumberOfMultiSamples(8);
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());

    MainWindow w;
    w.show();

    return a.exec();
}
