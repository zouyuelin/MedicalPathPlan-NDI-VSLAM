﻿#include <iostream>
#include "pathSetting.h"
#include <vtkAxesActor.h>
#include <vtkPolyDataNormals.h>

using namespace std;

// Define the user's vtkcommand


pathSetting::pathSetting(string stl_Path_, string texture_)
{
    stl_path = stl_Path_;
    texture = texture_;
    style = vtkSmartPointer<MouseInteractorStyleCenterline>::New();
    mycommand = vtkMyCommand::New();
}

void pathSetting::run()
{
    vtkSmartPointer<vtkPolyData> inputSurface = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> cappedSurface = vtkSmartPointer<vtkPolyData>::New();

    ExtractCenterline(stl_path, inputSurface, cappedSurface);

    style->RequireFinish();

    cout<<"The pathSetting is done !"<<endl;
}

vtkSmartPointer<vtkImageData> pathSetting::GetDICOMPolyData(string fileName)
{
    using ImageType = itk::Image<signed short, 3>; //多张dcm图片时

    itk::ImageSeriesReader< ImageType >::Pointer reader = itk::ImageSeriesReader< ImageType >::New();
    itk::GDCMImageIO::Pointer docomIO = itk::GDCMImageIO::New();
    reader->SetImageIO(docomIO);

    itk::GDCMSeriesFileNames::Pointer nameGenerator = itk::GDCMSeriesFileNames::New();
    nameGenerator->SetDirectory(fileName);

    std::vector< std::string > DICOMNames = nameGenerator->GetInputFileNames();
    reader->SetFileNames(DICOMNames);
    reader->Update();

    typedef itk::ImageToVTKImageFilter< ImageType> itkTovtkFilterType;
    itkTovtkFilterType::Pointer itkTovtkImageFilter = itkTovtkFilterType::New();
    itkTovtkImageFilter->SetInput(reader->GetOutput());
    itkTovtkImageFilter->Update();

    vtkSmartPointer<vtkImageData> imagedata = vtkSmartPointer<vtkImageData>::New();
    imagedata->DeepCopy(itkTovtkImageFilter->GetOutput());
    return imagedata;
}


void pathSetting::ReadSurface(string surfacePath, vtkPolyData* surface)
{
    // read surface
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(surfacePath.c_str());
    reader->Update();
    surface->DeepCopy(reader->GetOutput());
}

void pathSetting::ExtractCenterline(string surfacePath, vtkPolyData* inputSurface, vtkPolyData* cappedSurface)
{
    // read surface file
        ReadSurface(surfacePath, inputSurface);

        // capping
        vtkSmartPointer<vtkvmtkCapPolyData> capper = vtkSmartPointer<vtkvmtkCapPolyData>::New();
        capper->SetInputData(inputSurface);
        capper->Update();

        //清理 cleaning
        vtkSmartPointer<vtkCleanPolyData> cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
        cleaner->SetInputData(capper->GetOutput());
        cleaner->Update();

        //三角化
        vtkSmartPointer<vtkTriangleFilter> triangulator = vtkSmartPointer<vtkTriangleFilter>::New();
        triangulator->SetInputData(cleaner->GetOutput());
        triangulator->PassLinesOff();
        triangulator->PassVertsOff();
        triangulator->Update();

        int numberOfPolys = triangulator->GetOutput()->GetNumberOfPolys();
//        std::cout << "There are " << triangulator->GetOutput()->GetNumberOfPolys() << " polygons." << std::endl;

        //if poly number is not more than 3e4，there is no need to decline the datasets.
        if(numberOfPolys < 30000)
            cappedSurface->DeepCopy(triangulator->GetOutput());
        else
        {
            //三角数据减少 (1-3e4/numberOfPolys)*100% ，加快计算
            vtkSmartPointer<vtkDecimatePro> decimate = vtkSmartPointer<vtkDecimatePro>::New();
            decimate->SetInputConnection(triangulator->GetOutputPort());
            decimate->SetTargetReduction(1 - 3e4/(float)numberOfPolys );    //0.95
            decimate->PreserveTopologyOn();
            decimate->Update();

            cappedSurface->DeepCopy(decimate->GetOutput());
        }

        vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New(); //三维坐标系
            axes->SetPosition(0, 0, 0);
            axes->SetTotalLength(100, 100, 100);
            axes->SetShaftType(vtkAxesActor::LINE_SHAFT);
            axes->SetAxisLabels(0);

        cout<<"The number of the holes is:" <<capper->GetCapCenterIds()->GetNumberOfIds()<<endl;

        vtkSmartPointer<vtkPolyDataMapper> surfaceMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        surfaceMapper->SetInputData(triangulator->GetOutput());
        vtkSmartPointer<vtkActor> surfaceActor = vtkSmartPointer<vtkActor>::New();
        surfaceActor->SetMapper(surfaceMapper);

        //set the texture, but it looks like not working.
        vtkSmartPointer<vtkJPEGReader>reader=vtkSmartPointer<vtkJPEGReader>::New();
            reader->SetFileName(texture.c_str());

        vtkSmartPointer<vtkTexture> m_pTexture=vtkSmartPointer<vtkTexture>::New();
            m_pTexture->SetInputConnection(reader->GetOutputPort());
            m_pTexture->InterpolateOn();
            m_pTexture->Update();

        surfaceActor->SetTexture(m_pTexture);
        surfaceActor->GetProperty()->SetDiffuseColor(1, 0.49, 0.25);            //设置皮肤颜色的属性//(1, .49, .25)
        surfaceActor->GetProperty()->SetSpecular(0.3);    //0.3                     //设置反射率
        surfaceActor->GetProperty()->SetSpecularPower(40);                    //设置反射光强度
        surfaceActor->GetProperty()->ShadingOn();


        //------------------------------
        //计算法向量
        vtkSmartPointer<vtkPolyDataNormals> normFilter =
                  vtkSmartPointer<vtkPolyDataNormals>::New();
        normFilter->SetInputData(triangulator->GetOutput());
        normFilter->SetComputePointNormals(1);//开启点法向量计算
        normFilter->SetComputeCellNormals(0); //关闭单元法向量计算
        normFilter->SetAutoOrientNormals(1);
        normFilter->SetSplitting(0);
        normFilter->Update();
        //------------------------------



        // put the actor into render window
        vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New();
        ren->AddActor(surfaceActor);
        ren->AddActor(axes);

        vtkSmartPointer<vtkPolyData> centerline = vtkSmartPointer<vtkPolyData>::New();
//        vtkSmartPointer<vtkPolyDataMapper> centerlineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//        centerlineMapper->SetInputData(centerline);
//        vtkSmartPointer<vtkActor> centerlineActor = vtkSmartPointer<vtkActor>::New();
//        centerlineActor->SetMapper(centerlineMapper);
//        ren->AddActor(centerlineActor);

        vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
        renWin->AddRenderer(ren);
        renWin->SetSize(1920,1080);

        vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        iren->SetInteractorStyle(style);
        iren->SetRenderWindow(renWin);
        style->SetSurface(cappedSurface);
        style->SetCenterline(centerline);

        iren->Initialize();
        ren->ResetCamera();

        //init the vtkcommand
        mycommand->setTheStyle(style);
        mycommand->setThesurfaceForCollDetection(triangulator->GetOutput());
        mycommand->setThenormals(normFilter);

        //add an observer
        iren->AddObserver(vtkCommand::TimerEvent, mycommand);
        iren->CreateRepeatingTimer(5);
        //start the events loop
        iren->Start();

}
