#include <vtkSmartPointer.h>
#include <vtkSTLReader.h>
#include <vtkPolyData.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkClipPolyData.h>
#include <vtkPlane.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <random>
#include"../czxDependence/czxTool.h"


// Function to compute the area of a triangle
double computeTriangleArea(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3) {
    Eigen::Vector3f v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    Eigen::Vector3f v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
    return 0.5 * v1.cross(v2).norm();
}

// Function to sample a point within a triangle using barycentric coordinates
pcl::PointXYZ samplePointInTriangle(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    // Generate two random numbers for barycentric coordinates
    float r1 = dis(gen);
    float r2 = dis(gen);

    // Ensure they are inside the triangle
    if (r1 + r2 > 1.0) {
        r1 = 1.0f - r1;
        r2 = 1.0f - r2;
    }

    // Compute the barycentric interpolation
    pcl::PointXYZ sampled_point;
    sampled_point.x = p1.x + r1 * (p2.x - p1.x) + r2 * (p3.x - p1.x);
    sampled_point.y = p1.y + r1 * (p2.y - p1.y) + r2 * (p3.y - p1.y);
    sampled_point.z = p1.z + r1 * (p2.z - p1.z) + r2 * (p3.z - p1.z);

    return sampled_point;
}

// Main function to cut and sample STL
pcl::PointCloud<pcl::PointXYZ>::Ptr cutAndSampleMeshZ(const std::string& input_file, float z_min, float z_max, float resolution) {
    // Load STL file using VTK
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(input_file.c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyData> poly_data = reader->GetOutput();

    // Clip the mesh along the Z axis using a plane
    vtkSmartPointer<vtkPlane> plane_min = vtkSmartPointer<vtkPlane>::New();
    plane_min->SetOrigin(0.0, 0.0, z_min);
    plane_min->SetNormal(0.0, 0.0, 1.0);

    vtkSmartPointer<vtkClipPolyData> clipper_min = vtkSmartPointer<vtkClipPolyData>::New();
    clipper_min->SetInputData(poly_data);
    clipper_min->SetClipFunction(plane_min);
    clipper_min->Update();

    vtkSmartPointer<vtkPolyData> clipped_data_min = clipper_min->GetOutput();

    vtkSmartPointer<vtkPlane> plane_max = vtkSmartPointer<vtkPlane>::New();
    plane_max->SetOrigin(0.0, 0.0, z_max);
    plane_max->SetNormal(0.0, 0.0, -1.0);

    vtkSmartPointer<vtkClipPolyData> clipper_max = vtkSmartPointer<vtkClipPolyData>::New();
    clipper_max->SetInputData(clipped_data_min);
    clipper_max->SetClipFunction(plane_max);
    clipper_max->Update();

    vtkSmartPointer<vtkPolyData> clipped_data = clipper_max->GetOutput();

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    vtkSmartPointer<vtkCellArray> triangles = clipped_data->GetPolys();
    vtkSmartPointer<vtkPoints> points = clipped_data->GetPoints();

    triangles->InitTraversal();
    vtkIdType npts;
    vtkIdType* pts;

    // Iterate through each triangle
    while (triangles->GetNextCell(npts, pts)) {
        if (npts != 3) continue; // Skip if not a triangle

        pcl::PointXYZ p1, p2, p3;
        double pt[3];
        points->GetPoint(pts[0], pt);
        p1 = pcl::PointXYZ(pt[0], pt[1], pt[2]);
        points->GetPoint(pts[1], pt);
        p2 = pcl::PointXYZ(pt[0], pt[1], pt[2]);
        points->GetPoint(pts[2], pt);
        p3 = pcl::PointXYZ(pt[0], pt[1], pt[2]);

        // Compute the area of the triangle
        double area = computeTriangleArea(p1, p2, p3);
        int points_per_triangle = area / (resolution * resolution);

        #pragma omp parallel for
        for (int i = 0; i < points_per_triangle; ++i) {
            pcl::PointXYZ sampled_point = samplePointInTriangle(p1, p2, p3);
            #pragma omp critical
            output_cloud->push_back(sampled_point);
        }
    }

    return output_cloud;
}

// Main function to cut and sample STL
pcl::PointCloud<pcl::PointXYZ>::Ptr cutAndSampleMeshX(const std::string& input_file, float x_min, float x_max, float resolution) {
    // Load STL file using VTK
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(input_file.c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyData> poly_data = reader->GetOutput();

    // Clip the mesh along the Z axis using a plane
    vtkSmartPointer<vtkPlane> plane_min = vtkSmartPointer<vtkPlane>::New();
    plane_min->SetOrigin(x_min, 0.0, 0);
    plane_min->SetNormal(1.0, 0.0, 0);

    vtkSmartPointer<vtkClipPolyData> clipper_min = vtkSmartPointer<vtkClipPolyData>::New();
    clipper_min->SetInputData(poly_data);
    clipper_min->SetClipFunction(plane_min);
    clipper_min->Update();

    vtkSmartPointer<vtkPolyData> clipped_data_min = clipper_min->GetOutput();

    vtkSmartPointer<vtkPlane> plane_max = vtkSmartPointer<vtkPlane>::New();
    plane_max->SetOrigin(x_max, 0.0, 0);
    plane_max->SetNormal(-1.0, 0.0, 0);

    vtkSmartPointer<vtkClipPolyData> clipper_max = vtkSmartPointer<vtkClipPolyData>::New();
    clipper_max->SetInputData(clipped_data_min);
    clipper_max->SetClipFunction(plane_max);
    clipper_max->Update();

    vtkSmartPointer<vtkPolyData> clipped_data = clipper_max->GetOutput();

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    vtkSmartPointer<vtkCellArray> triangles = clipped_data->GetPolys();
    vtkSmartPointer<vtkPoints> points = clipped_data->GetPoints();

    triangles->InitTraversal();
    vtkIdType npts;
    vtkIdType* pts;

    // Iterate through each triangle
    while (triangles->GetNextCell(npts, pts)) {
        if (npts != 3) continue; // Skip if not a triangle

        pcl::PointXYZ p1, p2, p3;
        double pt[3];
        points->GetPoint(pts[0], pt);
        p1 = pcl::PointXYZ(pt[0], pt[1], pt[2]);
        points->GetPoint(pts[1], pt);
        p2 = pcl::PointXYZ(pt[0], pt[1], pt[2]);
        points->GetPoint(pts[2], pt);
        p3 = pcl::PointXYZ(pt[0], pt[1], pt[2]);

        // Compute the area of the triangle
        double area = computeTriangleArea(p1, p2, p3);
        int points_per_triangle = area / (resolution * resolution);

        #pragma omp parallel for
        for (int i = 0; i < points_per_triangle; ++i) {
            pcl::PointXYZ sampled_point = samplePointInTriangle(p1, p2, p3);
            #pragma omp critical
            output_cloud->push_back(sampled_point);
        }
    }

    return output_cloud;
}

#define cp(i) cloud->points[i]

CP filterShelterAndDense(CP cloud)
{
    CP ret(new CloudT);
    PointT last;

    //确定起始点位置, 如果连续十个点没有上跳变,认为是起始点
    int i;
    for (i = 0; i < cloud->size(); i++)
    {
        int off = 1;
        int range = 10;
        while (cp(i + off - 1).y - cp(i + off).y > -0.1 && off < range)
        {
            off++;
        }
        if (off == range)
        {
            last = cp(i);
            ret->push_back(cp(i));
            break;
        }
    }

    for (i++; i < cloud->size(); i++)
    {
        if (cp(i).x - last.x < 0.005)
            continue;
        if (cp(i).y - last.y < -0.001 && cp(i).x - last.x < 0.04)
            continue;
        if (cp(i).y - last.y > 0.001 && cp(i).x - last.x < 0.01)
        {
            ret->points.pop_back();
            last = *(ret->end() - 1);
        }
        last = cp(i);
        ret->push_back(cp(i));
    }
    #ifdef CZX_DEBUG
    Tool::showComparison(cloud, ret);
    #endif
    return ret;
}

void autoSplit(string input_stl, vector<float> zofx, vector<float> xofy)
{
    string root = czx_file::createFolderWithCurrentDate()+"/";
    float resolution = 0.005f;

    int count = 1;
    for (auto& val : zofx)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = cutAndSampleMeshZ(input_stl, val - 0.02, val + 0.02, resolution);
        cloud_1->getMatrixXfMap(3, 4, 0).row(2).setZero();
        //cloud_1->getMatrixXfMap(3, 4, 0).row(1) *= -1;
        sort(cloud_1->begin(), cloud_1->end(), [](PointT a, PointT b) {return a.x < b.x; });
        cloud_1 = filterShelterAndDense(cloud_1);
        pcl::io::savePCDFileBinary(root+to_string(count)+"x.pcd", *cloud_1);
        count++;
    }
    for (auto& val : xofy)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = cutAndSampleMeshX(input_stl, val - 0.02, val + 0.02, resolution);
        cloud_1->getMatrixXfMap(3, 4, 0).row(0).swap(cloud_1->getMatrixXfMap(3, 4, 0).row(2));
        cloud_1->getMatrixXfMap(3, 4, 0).row(2).setZero();
        //cloud_1->getMatrixXfMap(3, 4, 0).row(1) *= -1;
        sort(cloud_1->begin(), cloud_1->end(), [](PointT a, PointT b) {return a.x < b.x; });
        cloud_1 = filterShelterAndDense(cloud_1);
        pcl::io::savePCDFileBinary(root+to_string(count)+"y.pcd", *cloud_1);
        count++;
    }
}

std::vector<std::string> splitString(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}


int main()
{
    auto conf = czx_file::readProfile("meshSlice.czx");

    std::string input_stl = conf["path"];

    vector<float> zofx;
    vector<float> xofy;

    std::string zofx_str = conf["z"];
    auto zofx_str_v = splitString(zofx_str, ',');
    for (auto& z_str : zofx_str_v)
    {
        zofx.push_back(std::stof(z_str));
    }
    std::string xofy_str = conf["x"];
    auto xofy_str_v = splitString(xofy_str, ',');
    for (auto& x_str : xofy_str_v)
    {
        xofy.push_back(std::stof(x_str));
    }

    autoSplit(input_stl, zofx, xofy);
    return 0;


    // Set the desired point sampling resolution
    //float resolution = 0.005f;

    //S5
    //float x1 = 154.030609,
    //    x2= 95.279526,
    //    x3= 30.740707,
    //    y4= -112.052437,
    //    y5= -77.699867,
    //    y6= -49.866207,
    //    y7= -14.974101,
    //    y8= 4.317663,
    //    y9= 35.614170,
    //    y10= 70.115456,
    //    y11=122.099
    //    ;


    //float x1 = 160.006119,
    //    x2 = 93.988876,
    //    x3 = 33.502892,
    //    y4 = -127.466515,
    //    y5 = -76.958534,
    //    y6 = 7.713197,
    //    y7 = 72.133072,
    //    y8 = 121.987038;
    //{
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = cutAndSampleMeshZ(input_stl, x1 - 0.02, x1 + 0.02, resolution);
    //    //cloud_1->getMatrixXfMap(3, 4, 0).row(0).swap(cloud_1->getMatrixXfMap(3, 4, 0).row(2));
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(2).setZero();
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(1) *= -1;
    //    sort(cloud_1->begin(), cloud_1->end(), [](PointT a, PointT b) {return a.x < b.x; });
    //    cloud_1 = filterShelterAndDense(cloud_1);
    //    pcl::io::savePCDFileBinary("1x.pcd", *cloud_1);
    //}

    //{
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = cutAndSampleMeshZ(input_stl, x2 - 0.02, x2 + 0.02, resolution);
    //    //cloud_1->getMatrixXfMap(3, 4, 0).row(0).swap(cloud_1->getMatrixXfMap(3, 4, 0).row(2));
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(2).setZero();
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(1) *= -1;
    //    sort(cloud_1->begin(), cloud_1->end(), [](PointT a, PointT b) {return a.x < b.x; });
    //    cloud_1 = filterShelterAndDense(cloud_1);
    //    pcl::io::savePCDFileBinary("2x.pcd", *cloud_1);
    //}

    //{
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = cutAndSampleMeshZ(input_stl, x3 - 0.02, x3 + 0.02, resolution);
    //    //cloud_1->getMatrixXfMap(3, 4, 0).row(0).swap(cloud_1->getMatrixXfMap(3, 4, 0).row(2));
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(2).setZero();
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(1) *= -1;
    //    sort(cloud_1->begin(), cloud_1->end(), [](PointT a, PointT b) {return a.x < b.x; });
    //    cloud_1 = filterShelterAndDense(cloud_1);
    //    pcl::io::savePCDFileBinary("3x.pcd", *cloud_1);
    //}

    //{
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = cutAndSampleMeshX(input_stl, y4 - 0.02, y4 + 0.02, resolution);
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(0).swap(cloud_1->getMatrixXfMap(3, 4, 0).row(2));
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(2).setZero();
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(1) *= -1;
    //    sort(cloud_1->begin(), cloud_1->end(), [](PointT a, PointT b) {return a.x < b.x; });
    //    cloud_1 = filterShelterAndDense(cloud_1);
    //    pcl::io::savePCDFileBinary("4y.pcd", *cloud_1);
    //}
    //{
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = cutAndSampleMeshX(input_stl, y5 - 0.02, y5 + 0.02, resolution);
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(0).swap(cloud_1->getMatrixXfMap(3, 4, 0).row(2));
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(2).setZero();
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(1) *= -1;
    //    sort(cloud_1->begin(), cloud_1->end(), [](PointT a, PointT b) {return a.x < b.x; });
    //    cloud_1 = filterShelterAndDense(cloud_1);
    //    pcl::io::savePCDFileBinary("5y.pcd", *cloud_1);
    //}
    //{
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = cutAndSampleMeshX(input_stl, y6 - 0.02, y6 + 0.02, resolution);
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(0).swap(cloud_1->getMatrixXfMap(3, 4, 0).row(2));
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(2).setZero();
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(1) *= -1;
    //    sort(cloud_1->begin(), cloud_1->end(), [](PointT a, PointT b) {return a.x < b.x; });
    //    cloud_1 = filterShelterAndDense(cloud_1);
    //    pcl::io::savePCDFileBinary("6y.pcd", *cloud_1);
    //}
    //{
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = cutAndSampleMeshX(input_stl, y7 - 0.02, y7 + 0.02, resolution);
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(0).swap(cloud_1->getMatrixXfMap(3, 4, 0).row(2));
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(2).setZero();
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(1) *= -1;
    //    sort(cloud_1->begin(), cloud_1->end(), [](PointT a, PointT b) {return a.x < b.x; });
    //    cloud_1 = filterShelterAndDense(cloud_1);
    //    pcl::io::savePCDFileBinary("7y.pcd", *cloud_1);
    //}
    //{
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = cutAndSampleMeshX(input_stl, y8 - 0.02, y8 + 0.02, resolution);
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(0).swap(cloud_1->getMatrixXfMap(3, 4, 0).row(2));
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(2).setZero();
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(1) *= -1;
    //    sort(cloud_1->begin(), cloud_1->end(), [](PointT a, PointT b) {return a.x < b.x; });
    //    cloud_1 = filterShelterAndDense(cloud_1);
    //    pcl::io::savePCDFileBinary("8y.pcd", *cloud_1);
    //}
    //{
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = cutAndSampleMeshX(input_stl, y9 - 0.02, y9 + 0.02, resolution);
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(0).swap(cloud_1->getMatrixXfMap(3, 4, 0).row(2));
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(2).setZero();
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(1) *= -1;
    //    sort(cloud_1->begin(), cloud_1->end(), [](PointT a, PointT b) {return a.x < b.x; });
    //    cloud_1 = filterShelterAndDense(cloud_1);
    //    pcl::io::savePCDFileBinary("9y.pcd", *cloud_1);
    //}
    //{
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = cutAndSampleMeshX(input_stl, y10 - 0.02, y10 + 0.02, resolution);
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(0).swap(cloud_1->getMatrixXfMap(3, 4, 0).row(2));
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(2).setZero();
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(1) *= -1;
    //    sort(cloud_1->begin(), cloud_1->end(), [](PointT a, PointT b) {return a.x < b.x; });
    //    cloud_1 = filterShelterAndDense(cloud_1);
    //    pcl::io::savePCDFileBinary("10y.pcd", *cloud_1);
    //}
    //{
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = cutAndSampleMeshX(input_stl, y11 - 0.02, y11 + 0.02, resolution);
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(0).swap(cloud_1->getMatrixXfMap(3, 4, 0).row(2));
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(2).setZero();
    //    cloud_1->getMatrixXfMap(3, 4, 0).row(1) *= -1;
    //    sort(cloud_1->begin(), cloud_1->end(), [](PointT a, PointT b) {return a.x < b.x; });
    //    cloud_1 = filterShelterAndDense(cloud_1);
    //    pcl::io::savePCDFileBinary("11y.pcd", *cloud_1);
    //}
    return 0;
}

