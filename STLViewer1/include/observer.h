#pragma once
#include <vtkActor.h>
class Observer
{
public : 
	virtual void func(vtkSmartPointer<vtkActor>) = 0;
};