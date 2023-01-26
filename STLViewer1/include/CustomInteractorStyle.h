#pragma once

#include <QWidget> 
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <QVTKInteractor.h>
#include <vtkInteractorStyle.h> 
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkParallelCoordinatesInteractorStyle.h>
#include <vtkPolyData.h>
#include <vtkCellPicker.h>
#include <vtkIdTypeArray.h>
#include <vtkSelectionNode.h>
#include <vtkSelection.h>
#include <vtkExtractSelection.h>
#include <vtkNamedColors.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkTriangle.h>
#include "observer.h"
#include "TriMesh.h"
#include <vtkDijkstraGraphGeodesicPath.h> //Dijkstra
#include <vtkMutableDirectedGraph.h>
#include <vtkVector.h>
#include <vtkGraphToPolyData.h>
#include <vector>
#include <queue>
#include <QHash>


using namespace std;

class CustomInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
	CustomInteractorStyle();
	~CustomInteractorStyle();

	void GetPolyData(vtkSmartPointer<vtkPolyData>);
	void GetSphere(vtkSmartPointer<vtkSphereSource>);
	void GetActor(vtkSmartPointer<vtkActor>);
	void addObserver(Observer*);

protected:
	virtual void OnRightButtonDown() override;
	virtual void OnRightButtonUp() override;
	virtual void OnLeftButtonDown() override;
	virtual void OnLeftButtonUp() override;

	virtual void OnMouseWheelForward() override;
	virtual void OnMouseWheelBackward() override;

	void dijkstra(int start);

	TriMesh convertToMesh(vtkSmartPointer<vtkPolyData>);
	vtkSmartPointer<vtkPolyData> convertToPolyData(TriMesh);
	vtkSmartPointer<vtkCellArray> mCellArray;
private:
	vtkSmartPointer<vtkPolyData> mPolyData;
	vtkSmartPointer<vtkSphereSource> mSphere;
	vtkSmartPointer<vtkActor> mActor;
	//vtkSmartPointer < vtkDijkstraGraphGeodesicPath> mDijkstra;
	Observer* mObserver;
	vtkSmartPointer<vtkPoints> mVertex;
	vector<pair<int, int> > graph[100001];
	int d[100001];
	int mCount;
	QHash<int, vtkSmartPointer<vtkPoints>> Pickhash;;
	QHash<int, OpenMesh::Vec3d> dvertexhash;
	QHash< int, OpenMesh::Vec3d> lvertexhash;
	OpenMesh::Vec3d mLvertex;
};