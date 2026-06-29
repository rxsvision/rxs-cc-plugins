#pragma once
#include"../czxDependence/czxTool.h"
class pcd2stl
{
public:
	pcl::PointCloud<pcl::PointNormal>::Ptr computeNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius);
	pcd2stl(CP cloud);
	pcd2stl(CP cloud, float radius);
	pcd2stl(CNP cloud_normal);

	void saveSTL(string path);

private:
	void reconstructionPoisson();
	CNP cloud_normal_;
	pcl::PolygonMesh mesh_;
};

