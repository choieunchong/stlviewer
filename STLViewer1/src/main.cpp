#include "stlviewer.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    STLViewer w;
    w.show();
    return a.exec();
}
