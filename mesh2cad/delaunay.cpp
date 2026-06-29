#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <fstream>
#include<pcl/io/ply_io.h>

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <pcl/surface/mls.h>
#include "delaunay.h"


CNP smooth(CP cloud, double dense, double radius=10)
{
    CzxTimer dsgd(__func__);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    CNP mls_points(new CloudNT);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);

    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius * dense);

    mls.process(*mls_points);
    pcl::copyPointCloud(*mls_points, *cloud);
    return mls_points;
}

pcl::PolygonMesh delaunay2D(CP cloud)
{
    using K = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Vb = CGAL::Triangulation_vertex_base_with_info_2<std::size_t, K>;
    using Tds = CGAL::Triangulation_data_structure_2<Vb>;
    using Delaunay = CGAL::Delaunay_triangulation_2<K, Tds>;
    using cgalPoint = Delaunay::Point;

    CzxTimer asfsf(__func__);
    std::vector< std::pair<cgalPoint, std::size_t > > pts;
    pts.reserve(cloud->size());
    for (int i = 0; i < cloud->size(); i++)
    {
        PointT p = cloud->points[i];
        pts.emplace_back(cgalPoint(p.x, p.y), i);
    }
    Delaunay triangulation;
    triangulation.insert(pts.begin(), pts.end());

    pcl::PolygonMesh triangles;
    for (auto f = triangulation.faces_begin(); f != triangulation.faces_end(); ++f) {
        pcl::Vertices face_vertices;
        face_vertices.vertices.push_back(f->vertex(0)->info()); 
        face_vertices.vertices.push_back(f->vertex(1)->info()); 
        face_vertices.vertices.push_back(f->vertex(2)->info());

        triangles.polygons.push_back(face_vertices);
    }

    pcl::toPCLPointCloud2(*cloud, triangles.cloud);
    return triangles;
}

pcl::PolygonMesh delaunay3D(CP cloud)
{
    using K = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Vb = CGAL::Triangulation_vertex_base_with_info_3<std::size_t, K>;
    using Tds = CGAL::Triangulation_data_structure_3<Vb>;
    using Delaunay = CGAL::Delaunay_triangulation_3<K, Tds>;
    using cgalPoint = Delaunay::Point;

    CzxTimer asfsf(__func__);
    std::vector< std::pair<cgalPoint, std::size_t > > pts;
    pts.reserve(cloud->size());
    for (int i = 0; i < cloud->size(); i++)
    {
        PointT p = cloud->points[i];
        pts.emplace_back(cgalPoint(p.x, p.y, p.z), i);
    }
    Delaunay triangulation;
    triangulation.insert(pts.begin(), pts.end());

    auto num = triangulation.number_of_facets();
    cout << num << endl;
    pcl::PolygonMesh triangles;
    for (auto f = triangulation.finite_facets_begin(); f != triangulation.finite_facets_end(); ++f) {
        pcl::Vertices face_vertices;
        auto triangle = triangulation.triangle(*f);
        if ((f->second & 1) == 0)
        {
            face_vertices.vertices.push_back(f->first->vertex((f->second + 2) & 3)->info());
            face_vertices.vertices.push_back(f->first->vertex((f->second + 1) & 3)->info());
            face_vertices.vertices.push_back(f->first->vertex((f->second + 3) & 3)->info());
        }
        else 
        {
            face_vertices.vertices.push_back(f->first->vertex((f->second + 1) & 3)->info());
            face_vertices.vertices.push_back(f->first->vertex((f->second + 2) & 3)->info());
            face_vertices.vertices.push_back(f->first->vertex((f->second + 3) & 3)->info());
        }
        triangles.polygons.push_back(face_vertices);
        //if (f->first->is_valid())
        //{
        //    face_vertices.vertices.push_back(f->first->vertex(0)->info());
        //    face_vertices.vertices.push_back(f->first->vertex(1)->info());
        //    face_vertices.vertices.push_back(f->first->vertex(2)->info());
        //    triangles.polygons.push_back(face_vertices);
        //}
    }
    //for (const auto& facet : CGAL::make_range(triangulation.facets_begin(), triangulation.facets_end()))
    //{
    //    auto i = facet.first->vertex(0)->info();
    //    i = facet.first->vertex(1);
    //}

    pcl::toPCLPointCloud2(*cloud, triangles.cloud);
    return triangles;
}

/*
int main() {
    CP cloud(new CloudT);
    pcl::io::loadPCDFile("small.pcd", *cloud);


    //cloud->points.resize(10);
    //for (size_t i = 0; i < cloud->points.size(); ++i) {
    //    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    //    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    //    cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    //}


    smooth(cloud, 0.1);
    auto mesh = delaunay2D(cloud);
    //auto mesh = delaunay3D(cloud);

    pcl::io::savePLYFile("test.ply", mesh);
    return 0;

}
*/