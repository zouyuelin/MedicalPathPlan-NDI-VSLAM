#ifndef PATHSETTING_H
#define PATHSETTING_H
#include <iostream>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkUnstructuredGrid.h>
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <vtkUnstructuredGridVolumeMapper.h>
#include <vtkProperty.h>
#include <vtkIdList.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPointData.h>
#include <vtkLookupTable.h>
#include <vtkScalarBarActor.h>
#include <vtkCleanPolyData.h>
#include <vtkTriangleFilter.h>
#include <vtkXMLImageDataWriter.h>
#include <vtkImageData.h>
#include <vtkImageShiftScale.h>
#include <vtkPNGWriter.h>

#include <vtkvmtkCenterlineAttributesFilter.h>
#include <vtkvmtkCenterlineBranchExtractor.h>
#include <vtkvmtkCenterlineBranchGeometry.h>
#include <vtkvmtkPolyDataCenterlineGroupsClipper.h>
#include <vtkvmtkCenterlineBifurcationReferenceSystems.h>
#include <vtkvmtkPolyDataCenterlineAngularMetricFilter.h>
#include <vtkvmtkPolyDataCenterlineAbscissaMetricFilter.h>
#include <vtkvmtkPolyDataReferenceSystemBoundaryMetricFilter.h>
#include <vtkvmtkPolyDataMultipleCylinderHarmonicMappingFilter.h>
#include <vtkvmtkPolyDataStretchMappingFilter.h>
#include <vtkvmtkCapPolyData.h>
#include <vtkvmtkPolyDataPatchingFilter.h>
#include <vtkvmtkCenterlineGeometry.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <itkImageSeriesReader.h>
#include <itkImage.h>
#include <vtkDecimatePro.h>

#include "interactorStyleCenterline.h"
#include "Centerline.h"
#include "cameraRender.h"

//itk
#include <itkImageSeriesReader.h>
#include <itkGDCMImageIO.h>
#include <itkGDCMSeriesFileNames.h>
#include <itkImageToVTKImageFilter.h>
#include <vtkJPEGReader.h>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingFreeType)
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

class pathSetting
{
public:
    pathSetting(string stl_Path_, string texture_);

    void run();

    void ReadSurface(string surfacePath, vtkPolyData* surface);
    vtkSmartPointer<vtkImageData> GetDICOMPolyData(string fileName);
    void ExtractCenterline(string surfacePath, vtkPolyData* inputSurface, vtkPolyData* cappedSurface);

    vtkSmartPointer<MouseInteractorStyleCenterline> style;
    vtkMyCommand * mycommand = nullptr;

private:
    string stl_path;
    string texture;
};

#endif // PATHSETTING_H
