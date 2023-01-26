#pragma once
#include <QVTKOpenGLNativeWidget.h>
#include <QVTKInteractorAdapter.h>
#include <vtkSTLReader.h>
#include <vtkActor.h>
#include "CustomInteractorStyle.h"
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include "observer.h"

class CustomVTKWidget : public QVTKOpenGLNativeWidget , public Observer
{
public:
    CustomVTKWidget();
	CustomVTKWidget(QWidget* parent);
	~CustomVTKWidget();

    // Renderer Add Actor
    void AddActor(vtkSmartPointer<vtkActor>);

    void GetPolyData(vtkSmartPointer<vtkPolyData>);
    void GetSphere(vtkSmartPointer<vtkSphereSource>);
    void AddSphere(vtkSmartPointer<vtkActor>);

protected:
    // VTK Renderer
    vtkSmartPointer<vtkRenderer> mRenderer; 

    // VTK Render Window
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> mRenderWindow; 
    vtkSmartPointer<QVTKInteractor> mInteractor;
    QVTKInteractorAdapter* mvtkInteractorAdapter; 

    virtual void func(vtkSmartPointer<vtkActor>);

private: 
    
    CustomInteractorStyle* customInteractorStyle;
};