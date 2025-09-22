#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>  // 曲面重建所用头文件
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

void runAS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	// -------------------------------曲面重构-----------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> cavehull;
	cavehull.setInputCloud(cloud);
	cavehull.setAlpha(0.01);
	vector<pcl::Vertices> polygons;
	cavehull.reconstruct(*surface_hull, polygons);// 重建面要素到点云

	pcl::PolygonMesh mesh;
	cavehull.reconstruct(mesh);// 重建面要素到mesh 
	pcl::io::saveOBJFile("object_mesh_1.obj", mesh);
	cout << "Concave hull has: " << surface_hull->points.size() << " data points." << endl;
	// 保存结果
	pcl::PCDWriter writer;
	writer.write("hull.pcd", *surface_hull, false);
	// -------------------------------可视化---------------------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("hull"));
	viewer->setWindowName("alshape曲面重构");
	viewer->addPolygonMesh<pcl::PointXYZ>(surface_hull, polygons, "polyline");
	viewer->spin();

}
