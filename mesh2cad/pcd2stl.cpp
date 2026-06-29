#include "pcd2stl.h"

#include <pcl/features/normal_3d.h>
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkSTLWriter.h>
#include <vtkPolyData.h>
#include <pcl/surface/poisson.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtkPolyDataReader.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkTriangleFilter.h>
#include <pcl/io/pcd_io.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyDataNormals.h>

pcl::PointCloud<pcl::PointNormal>::Ptr pcd2stl::computeNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius = 0.02)
{
    CzxTimer _(__func__);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    // 将点云和法线合并
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    return cloud_with_normals;
}

pcd2stl::pcd2stl(CP cloud)
{
	cloud_normal_ = computeNormal(cloud);
}

pcd2stl::pcd2stl(CP cloud, float radius)
{
    cloud_normal_ = computeNormal(cloud, radius);
}

pcd2stl::pcd2stl(CNP cloud_normal)
{
    cloud_normal_ = cloud_normal;
}

void pcd2stl::reconstructionPoisson()
{
    CzxTimer _(__func__);
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setInputCloud(cloud_normal_);
    poisson.setDepth(9); // 控制重建细节的参数
    poisson.reconstruct(mesh_);
}

void pcd2stl::saveSTL(string path)
{
    reconstructionPoisson();
    CzxTimer _(__func__);
    vtkSmartPointer<vtkPolyData> vtkPolyData_ = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::mesh2vtk(mesh_, vtkPolyData_);

    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(vtkPolyData_);
    triangleFilter->Update();

    vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();
    normals->SetInputData(triangleFilter->GetOutput());
    normals->Update();

    vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
    stlWriter->SetFileName(path.c_str());
    stlWriter->SetInputData(normals->GetOutput());
    stlWriter->SetFileTypeToBinary();
    stlWriter->Write();
}
