#pragma once
#include "CustomInteractorStyle.h"
#include <QDebug>
#include "TriMesh.h"
#include <vtkMath.h>
#include<iostream>
using namespace std;


CustomInteractorStyle::CustomInteractorStyle()
{
    mCount = 0;

}

CustomInteractorStyle::~CustomInteractorStyle()
{

}

void CustomInteractorStyle::OnRightButtonDown()
{
    __super::OnLeftButtonDown();
}

void CustomInteractorStyle::OnRightButtonUp()
{
    __super::OnLeftButtonUp();
}

void CustomInteractorStyle::dijkstra(int start)
{
    priority_queue<pair<double, int>>pq; // 거리, 노드 인덱스
    pq.push({ 0,start }); //시작 노드로 가기위한 최단 경로는 0으로 설정하여, 큐에 삽입.
    d[start] = 0;

    while (!pq.empty())
    {
        int dist = -pq.top().first; //현재 노드까지의 비용
        int now = pq.top().second; // 현재 노드
        pq.pop();

        if (d[now] < dist) // 이미 최단경로를 체크한 노드인 경우 패스
            continue;

        for (int i = 0; i < graph[now].size(); i++)
        {
            int cost = dist + graph[now][i].second; // 거쳐서 가는 노드의 비용을 계산
                                  // 현재노드까지 비용 + 다음 노드 비용
            if (cost < d[graph[now][i].first]) // 비용이 더 작다면 최단경로 테이블 값을 갱신.
            {
                d[graph[now][i].first] = cost;
                pq.push(make_pair(-cost, graph[now][i].first));
            }
        }
    }
}


void CustomInteractorStyle::OnLeftButtonDown()
{
    int* pos = GetInteractor()->GetEventPosition();

    vtkSmartPointer<vtkCellPicker>cellPicker = vtkSmartPointer<vtkCellPicker>::New(); //vertex를 잡기위한 picker
    cellPicker->SetTolerance(0.00001); // 영역 의 값을 잡는다

    // Pick from this location
    cellPicker->Pick(pos[0], pos[1], 0, GetCurrentRenderer());

    double* worldPosition = cellPicker->GetPickPosition();

    qDebug() << "Cell id is : " << cellPicker->GetCellId();
    int hid = 0;
    Pickhash[hid] = mVertex;
    mCount++;
    qDebug() << "count" << mCount;
    if (cellPicker->GetCellId() != -1)
    {
        qDebug() << "Pick Position is : (" << worldPosition[0] << ", " << worldPosition[1] << ", " << worldPosition[2] << ")";
        vtkNew<vtkNamedColors> colors;

        // Create a sphere
        vtkNew<vtkSphereSource> sphereSource;
        sphereSource->SetCenter(worldPosition[0], worldPosition[1], worldPosition[2]);
        sphereSource->SetRadius(0.5);
        // Make the surface smooth.
        sphereSource->SetPhiResolution(100);
        sphereSource->SetThetaResolution(100);


        OpenMesh::Vec3d pickingPosition(worldPosition[0], worldPosition[1], worldPosition[2]);

        vtkNew<vtkPolyDataMapper> mapper;
        mapper->SetInputConnection(sphereSource->GetOutputPort()); //sphererSource 의 주소값을 mapper에 담는다


        vtkNew<vtkActor> mActor;
        mActor->SetMapper(mapper);
        mActor->GetProperty()->SetColor(colors->GetColor3d("Red").GetData());
        mActor->GetProperty()->SetLineWidth(4);

        mObserver->func(mActor);

        vtkSmartPointer<vtkPolyData> polyData = vtkPolyDataMapper::SafeDownCast(cellPicker->GetActor()->GetMapper())->GetInput();
        TriMesh triMesh = convertToMesh(polyData);

        qDebug() << "dist hi";

        qDebug() << "fv_begin" << triMesh.fv_begin(TriMesh::FaceHandle(cellPicker->GetCellId()));
        qDebug() << "fv_end" << triMesh.fv_end(TriMesh::FaceHandle(cellPicker->GetCellId()));


        double min = 100;
        OpenMesh::Vec3d minpoint;
        vtkNew<vtkMath>math;

        for (TriMesh::FaceVertexIter fv_iter = triMesh.fv_begin(TriMesh::FaceHandle(cellPicker->GetCellId()));
            fv_iter != triMesh.fv_end(TriMesh::FaceHandle(cellPicker->GetCellId())); fv_iter++) // fv_begin 은 첫점부터 fv_end값 까지 돈다 
        {
            qDebug() << "fv_iter : " << fv_iter->idx();
            OpenMesh::Vec3d point = triMesh.point(fv_iter);         // point 반환 , 월드좌표에서 가장 가까운 vertex 거리 출력
            OpenMesh::Vec3d diff = point - pickingPosition;         // 거리가 적은 백터값 담기
            double distance = diff.length();                        // 거리가 가장 적은 vertex를 구해야한다.

            qDebug() << "distance" << distance;

            min = (min > distance) ? distance : min;
            if (min == distance) {
                minpoint = point;
            }

        }

        mVertex = vtkSmartPointer<vtkPoints>::New();
        mVertex->InsertNextPoint(minpoint[0], minpoint[1], minpoint[2]);
        qDebug() << "minpoint" << mVertex;


        int minId = 0;
        int eid = 0;
        vtkIdType id = minId;
        vtkIdType veid = eid;
        int did = 0;

        for (TriMesh::VertexIter v_it = triMesh.vertices_sbegin(); v_it != triMesh.vertices_end(); ++v_it)
        {
            if (minpoint[0] == triMesh.point(*v_it)[0] && minpoint[1] == triMesh.point(*v_it)[1] && minpoint[2] == triMesh.point(*v_it)[2])
            {
                minId = v_it.handle().idx();
                for (TriMesh::VertexVertexIter vv_it = triMesh.vv_iter(OpenMesh::VertexHandle(minId)); vv_it.is_valid(); ++vv_it)
                {
                    qDebug() << " v_it" << triMesh.point(*vv_it)[0] << triMesh.point(*vv_it)[1] << triMesh.point(*vv_it)[2];

                    qDebug() << "vId =" << minId;

                    eid = vv_it.handle().idx();

                    OpenMesh::Vec3d evertex = triMesh.point(vv_it);
                    dvertexhash[did] = evertex;
                    did++;
                    cout << "evertex:" << evertex;

                    qDebug() << "did:" << did;
                    if (mCount >= 2) {
                        bool flag = true;
                        while (flag) {
                            for (TriMesh::VertexVertexIter vv_it = triMesh.vv_iter(OpenMesh::VertexHandle(eid)); vv_it.is_valid(); ++vv_it)
                            {
                                if (evertex == minpoint)
                                {
                                    flag = false;
                                    break;
                                }
                                qDebug() << " v_ittt" << triMesh.point(*vv_it)[0] << triMesh.point(*vv_it)[1] << triMesh.point(*vv_it)[2];
                                qDebug() << "vId =" << eid;
                                mLvertex = triMesh.point(vv_it);
                                lvertexhash[eid] = mLvertex;

                                vtkNew<vtkSphereSource> dsphereSource;
                                dsphereSource->SetCenter(triMesh.point(*vv_it)[0], triMesh.point(*vv_it)[1], triMesh.point(*vv_it)[2]);
                                dsphereSource->SetRadius(0.5);
                                dsphereSource->SetPhiResolution(100);
                                dsphereSource->SetThetaResolution(100);

                                vtkNew<vtkPolyDataMapper>dmapper;
                                dmapper->SetInputConnection(dsphereSource->GetOutputPort());

                                vtkNew<vtkActor>dActor;
                                dActor->SetMapper(dmapper);
                                dActor->GetProperty()->SetColor(colors->GetColor3d("green").GetData());
                                mObserver->func(dActor);

                                vtkSmartPointer< vtkDijkstraGraphGeodesicPath> dij = vtkSmartPointer<vtkDijkstraGraphGeodesicPath>::New();

                                unique(mLvertex.begin(), mLvertex.end());
                                for (int i = 0; i < mLvertex.size(); i++) {
                                    cout << mLvertex[i] << " -----------------";
                                }

                            }
                        }
                    }
                }
            }
        }

        //현재 찍은 점의 vertex 의 개수를 알때 
        // 다음점의 꼭지점을 알고 가사이의 간선의 개수를 INF라 할때 어떻게 할 것 인가?

        vtkNew<vtkSphereSource> sphereSource1;
        sphereSource1->SetCenter(minpoint[0], minpoint[1], minpoint[2]);
        sphereSource1->SetRadius(0.8);
        // Make the surface smooth.
        sphereSource->SetPhiResolution(100);
        sphereSource->SetThetaResolution(100);
        vtkNew<vtkPolyDataMapper> mapper1;
        mapper1->SetInputConnection(sphereSource1->GetOutputPort());


        vtkNew<vtkActor>eActor;
        eActor->SetMapper(mapper1);
        eActor->GetProperty()->SetColor(colors->GetColor3d("blue").GetData());
        // mActor->GetProperty()->SetLineWidth(4);
        mObserver->func(eActor);
        vtkSmartPointer<vtkPolyData> polyData1 = vtkPolyDataMapper::SafeDownCast(cellPicker->GetActor()->GetMapper())->GetInput();
        TriMesh triMesh1 = convertToMesh(polyData1);


        vtkNew<vtkSphereSource> dsphereSource;
        dsphereSource->SetCenter(minpoint[0], minpoint[1], minpoint[2]);
        dsphereSource->SetRadius(0.5);
        dsphereSource->SetPhiResolution(100);
        dsphereSource->SetThetaResolution(100);

        vtkNew<vtkPolyDataMapper>dmapper;
        dmapper->SetInputConnection(dsphereSource->GetOutputPort());

        vtkNew<vtkActor>dActor;
        dActor->SetMapper(dmapper);
        dActor->GetProperty()->SetColor(colors->GetColor3d("green").GetData());
        mObserver->func(dActor);

        //triMesh.delete_face(OpenMesh::FaceHandle(cellPicker->GetCellId())); // pickwer로 선택한 하나의 삼각형 meshf를 지운다
        //qDebug() << "delete Cell ID : " << cellPicker->GetCellId(); 
        //triMesh.garbage_collection();


        vtkSmartPointer<vtkPolyData> meshToPoly = convertToPolyData(triMesh);
        vtkPolyDataMapper::SafeDownCast(cellPicker->GetActor()->GetMapper())->SetInputData(meshToPoly);
        vtkPolyDataMapper::SafeDownCast(cellPicker->GetActor()->GetMapper())->Modified();

        vtkSmartPointer<vtkPolyData> meshToPoly1 = convertToPolyData(triMesh1);
        vtkPolyDataMapper::SafeDownCast(cellPicker->GetActor()->GetMapper())->SetInputData(meshToPoly1);
        vtkPolyDataMapper::SafeDownCast(cellPicker->GetActor()->GetMapper())->Modified();

    }
}

void CustomInteractorStyle::OnLeftButtonUp()
{

}

void CustomInteractorStyle::OnMouseWheelForward()
{
    __super::OnMouseWheelForward();
}

void CustomInteractorStyle::OnMouseWheelBackward()
{
    __super::OnMouseWheelBackward();
}

void CustomInteractorStyle::GetPolyData(vtkSmartPointer<vtkPolyData> polyData)
{
    mPolyData = polyData;
}

void CustomInteractorStyle::GetSphere(vtkSmartPointer<vtkSphereSource> sphereSource)
{
    mSphere = sphereSource;
}

void CustomInteractorStyle::GetActor(vtkSmartPointer<vtkActor> sphereSource)
{
    mActor = sphereSource;
}

void CustomInteractorStyle::addObserver(Observer* observer)
{
    mObserver = observer;
}

TriMesh CustomInteractorStyle::convertToMesh(vtkSmartPointer<vtkPolyData> polyData)
{
    int nPoints = polyData->GetNumberOfPoints();
    int nCells = polyData->GetNumberOfCells();

    TriMesh triMesh;

    for (int vertexId = 0; vertexId < nPoints; ++vertexId)
    {
        double* point = polyData->GetPoint(vertexId);
        triMesh.add_vertex(OpenMesh::Vec3d(point[0], point[1], point[2]));
    }

    for (int cellId = 0; cellId < nCells; ++cellId)
    {
        int vertexId0 = polyData->GetCell(cellId)->GetPointIds()->GetId(0);
        int vertexId1 = polyData->GetCell(cellId)->GetPointIds()->GetId(1);
        int vertexId2 = polyData->GetCell(cellId)->GetPointIds()->GetId(2);

        OpenMesh::VertexHandle vertexHandle0(vertexId0);
        OpenMesh::VertexHandle vertexHandle1(vertexId1);
        OpenMesh::VertexHandle vertexHandle2(vertexId2);
        triMesh.add_face({ vertexHandle0, vertexHandle1, vertexHandle2 });
    }

    return triMesh;
}

vtkSmartPointer<vtkPolyData> CustomInteractorStyle::convertToPolyData(TriMesh triMesh)
{
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

    vtkSmartPointer<vtkPoints>   points = vtkSmartPointer<vtkPoints>::New();
    int vid = 0;
    for (TriMesh::VertexIter vertexItr = triMesh.vertices_begin(); vertexItr != triMesh.vertices_end(); ++vertexItr, ++vid)
    {
        OpenMesh::Vec3d from = triMesh.point(*vertexItr);
        double* coords = from.data();
        while (vid < vertexItr->idx())
        {
            vid++; points->InsertNextPoint(0, 0, 0);
        }
        points->InsertNextPoint(coords[0], coords[1], coords[2]);
    }
    polyData->SetPoints(points);

    mCellArray = vtkSmartPointer<vtkCellArray>::New();
    for (TriMesh::FaceIter faceItr = triMesh.faces_begin(); faceItr != triMesh.faces_end(); ++faceItr)
    {
        TriMesh::FaceVertexIter   faceVertexItr;
        faceVertexItr = triMesh.cfv_iter(*faceItr);
        int   v0 = (faceVertexItr++).handle().idx();
        int   v1 = (faceVertexItr++).handle().idx();
        int   v2 = faceVertexItr.handle().idx();

        vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
        triangle->GetPointIds()->SetId(0, v0);
        triangle->GetPointIds()->SetId(1, v1);
        triangle->GetPointIds()->SetId(2, v2);
        mCellArray->InsertNextCell(triangle);
    }
    //polyData->SetLines(cellArray);// dijkstra 꼭지점간 연결을 하려면 최소한의 호출을 해야한다

    polyData->SetPolys(mCellArray);
    polyData->Modified();

    return polyData;
}