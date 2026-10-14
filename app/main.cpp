#include <QApplication>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>

#include "language.h"
#include "mainwindow.h"

int main(int argc, char* argv[]) {
    // VTK needs a specific OpenGL profile. It has to be set as the default
    // before the application object and the first OpenGL widget are created.
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    QApplication app(argc, argv);
    QCoreApplication::setOrganizationName("HatemSelim");  // where QSettings stores the language
    QCoreApplication::setApplicationName("PclApp");
    pclapp::app::applyLanguage(pclapp::app::savedLanguageCode());

    MainWindow window;
    window.show();
    return app.exec();
}
