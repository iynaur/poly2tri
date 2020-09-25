#include <vtkOBJReader.h>
#include <vtkSmartPointer.h>
#include <vtkCutter.h>
#include <vtkPolyDataMapper.h>
#include <vtkPlane.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkStripper.h>
#include <vtkContourFilter.h>
#include <vtkPoints.h>
#include <iostream>
using namespace std;


double Distance(double *a,double *b)
{
  double o=(a[0]-b[0])*(a[0]-b[0]);
  double p=(a[1]-b[1])*(a[1]-b[1]);
  double q=(a[2]-b[2])*(a[2]-b[2]);
  return o+p+q;
}
int main()
{

  //读取obj文件
  vtkSmartPointer<vtkOBJReader> reader =vtkSmartPointer<vtkOBJReader>::New();
  reader->SetFileName("./1.obj");
  reader->Update();

  vtkSmartPointer<vtkPolyData> inputPolyData=vtkSmartPointer<vtkPolyData>::New();
  inputPolyData = reader->GetOutput();

  vtkSmartPointer<vtkPolyDataMapper> inputMapper =vtkSmartPointer<vtkPolyDataMapper>::New();
  inputMapper->SetInputData(inputPolyData);

  //得到输入的obj模型的最小坐标
  double minBound[3];
  minBound[0] = inputPolyData->GetBounds()[0];
  minBound[1] = inputPolyData->GetBounds()[2];
  minBound[2] = inputPolyData->GetBounds()[4];

  //得到输入的obj模型的最小大坐标
  double maxBound[3];
  maxBound[0] = inputPolyData->GetBounds()[1];
  maxBound[1] = inputPolyData->GetBounds()[3];
  maxBound[2] = inputPolyData->GetBounds()[5];

  //得到输入的obj模型的中心坐标
  double center[3];
  center[0] = inputPolyData->GetCenter()[0];
  center[1] = inputPolyData->GetCenter()[1];
  center[2] = inputPolyData->GetCenter()[2];

  double distanceMin = inputPolyData->GetBounds()[4];
  double distanceMax = inputPolyData->GetBounds()[5];

  //创建切割平面
  vtkSmartPointer<vtkPlane> plane =vtkSmartPointer<vtkPlane>::New();
  srand(clock());
  plane->SetOrigin(0, 0, distanceMin + (rand()%100 / 100.0)*(distanceMax - distanceMin));//设置切割平面起点
  plane->SetNormal(0,0,1);//设置切割方向为X方向

  //创建模型切割器
  vtkSmartPointer<vtkCutter> cutter =vtkSmartPointer<vtkCutter>::New();
  cutter->SetCutFunction(plane);//设置切割平面
  cutter->SetInputData(inputPolyData);//设置模型
//  cutter->GenerateValues(3, distanceMin + 0*(rand()%100 / 100.0)*(distanceMax - distanceMin), distanceMax);//在模型的最大最小范围内等间距创建30个切面，得到轮廓线


  //将切线结果输出为vtk文件格式
  vtkSmartPointer<vtkPolyData> ResultPoly=cutter->GetOutput();//输出为polydata
  vtkSmartPointer<vtkPolyDataWriter> vtkWriter = vtkSmartPointer<vtkPolyDataWriter>::New();
  vtkWriter->SetInputConnection(cutter->GetOutputPort());

  vtkWriter->SetFileName("cut.vtk");
  vtkWriter->Write();


  vtkSmartPointer<vtkPolyDataMapper> cutterMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  cutterMapper->SetInputConnection( cutter->GetOutputPort());
  cutterMapper->ScalarVisibilityOff();

  vtkSmartPointer<vtkActor> planeActor =
    vtkSmartPointer<vtkActor>::New();
  planeActor->GetProperty()->SetColor(1,0,0);
  planeActor->GetProperty()->SetLineWidth(5);
  planeActor->SetMapper(cutterMapper);

  vtkSmartPointer<vtkActor> inputActor =
    vtkSmartPointer<vtkActor>::New();
  inputActor->GetProperty()->SetColor(0,1,0);
  inputActor->SetMapper(inputMapper);


  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(planeActor);
  renderer->AddActor(inputActor);

  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(600, 600);

  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);
  renderer->SetBackground(0,0,0);
  renderWindow->Render();

  interactor->Start();
  system("pcl_viewer cut.vtk");
  return 0;

}
