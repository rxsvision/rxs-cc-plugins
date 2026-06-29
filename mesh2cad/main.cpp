#include"pcd2stl.h"
#include <TopoDS_Shape.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <StlAPI_Reader.hxx>
#include <STEPControl_Writer.hxx>

int main(int argc, char* argv[])
{
    // Step 1: 读取 PCD 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("small.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the PCD file.\n");
        return -1;
    }

    pcd2stl ps(cloud, 2);
    ps.saveSTL("output_mesh.stl");

    TopoDS_Shape shape;

    // 读取 STL 文件
    StlAPI_Reader reader;
    reader.Read(shape, "output_mesh.stl");

    // 导出为 STEP 文件
    STEPControl_Writer writer;
    writer.Transfer(shape, STEPControl_AsIs);
    writer.Write("output.step");
}
