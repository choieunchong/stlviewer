#ifndef STLVIEWER_H
#define STLVIEWER_H

#include <QMainWindow>
#include <vtkSTLReader.h>
#include <vtkMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <QColorDialog.h>
#include "CustomVTKWidget.h"
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

QT_BEGIN_NAMESPACE
namespace Ui { class STLViewer; }
QT_END_NAMESPACE

class STLViewer : public QMainWindow
{
    Q_OBJECT

public:
    STLViewer(QWidget *parent = nullptr);
    ~STLViewer();

private:
    Ui::STLViewer *ui;
    CustomVTKWidget* customVTKWidget;

    vtkSmartPointer<vtkActor> mActor;
    QColorDialog* mColorDialog;  

    vtkSmartPointer<vtkPolyData> mPolyData;

signals:

private slots:
    void ClickedOpen(bool);                         // Menu -> Open 
    void SetColor(QColor);                          // Actor Color Change
    void SetOpacity(int);                           // Acotr Opacity Change

};
#endif // STLVIEWER_H
