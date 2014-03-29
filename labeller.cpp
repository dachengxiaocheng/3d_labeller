// =====================================================================================
//
//       Filename:  labeller.cpp
//
//    Description:
//
//        Version:  1.0
//        Created:  03/15/2014 08:06:56 PM
//       Revision:  none
//       Compiler:  g++
//
//         Author:  destine Lin (), siyuen.lin@gmail.com
//        Company:
//
// =====================================================================================
#include "include/point_types.h"
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>
#include <pcl/common/io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/visualization/impl/point_cloud_color_handlers.hpp>

typedef pcl::PointXYZRGBCamSL  PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//----------------------------------------------------------------------
//  Global variables list
//----------------------------------------------------------------------
pcl::visualization::PCLVisualizer viewer("3D Viewer");
PointCloudT::Ptr cloud_ptr(new PointCloudT);
PointCloudT::Ptr new_cloud_ptr (new PointCloudT);
PointCloudT cloud_marked;
std::string pressed_num;
pcl::IndicesPtr selected(new std::vector<int>);
std::string infile;
std::string outfile;
uint8_t r=255, g=0, b=0;
uint32_t seg_color = ((uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b);

// ===  FUNCTION  ======================================================================
//         Name:  markLabel
//  Description:
// =====================================================================================
void markLabel(PointCloudT &in, std::vector<int> &indices,  int label_num)
{
  for(size_t i = 0; i < indices.size(); i++)
    in.points[indices[i]].label = label_num;
}

// ===  FUNCTION  ======================================================================
//         Name:  checkOutPointCloud
//  Description:
// =====================================================================================
void checkOutPointCloud(PointCloudT &in, pcl::IndicesPtr &indices)
{
  PointCloudT tmp;
  pcl::ExtractIndices<PointT> ei;
  ei.setInputCloud(in.makeShared());
  ei.setIndices(indices);
  ei.filter(tmp);
  cloud_marked += tmp;
  ei.setNegative(true);
  ei.filter(tmp);
  *cloud_ptr = tmp;
  viewer.removePointCloud("sample cloud");
  *new_cloud_ptr = in;
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(new_cloud_ptr);
  viewer.addPointCloud<PointT> (new_cloud_ptr, rgb, "sample cloud");
}

// ===  FUNCTION  ======================================================================
//         Name:  keyboardEventOccurred
//  Description:
// =====================================================================================
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* in)
{
  boost::shared_ptr<PointCloudT> in_ptr = *static_cast<boost::shared_ptr<PointCloudT> *> (in);
  if(event.isCtrlPressed() && event.keyUp() && !pressed_num.empty()) {
    int label_num = boost::lexical_cast<int>(pressed_num);
    std::cout << "segment will be marked as label " << label_num << "." << std::endl;
    markLabel(*in_ptr, *selected, label_num);
    checkOutPointCloud(*in_ptr, selected);
    pressed_num.clear();
  }else {
    char word = event.getKeyCode();
    if(event.keyUp() && word >= '0' && word <= '9')
      pressed_num += word;
    else if (event.keyUp() && word == 'd')
      pressed_num.clear();
    else if (event.keyUp() && word == 's') {
      pcl::io::savePCDFile(outfile, cloud_marked);
      std::cout << "point cloud has saved into " << outfile << "." << std::endl;
    }
  }
}

// ===  FUNCTION  ======================================================================
//         Name:  selectionAreaOccurred
//  Description:
// =====================================================================================
void selectionAreaOccurred(const pcl::visualization::AreaPickingEvent &event, void* in)
{
  selected->clear();
  if(!event.getPointsIndices (*selected)) return;
  boost::shared_ptr<PointCloudT> in_ptr = *static_cast<boost::shared_ptr<PointCloudT> *> (in);
  viewer.removePointCloud("sample cloud");
  *new_cloud_ptr = *in_ptr;

  std::cout << selected->size() << " of points being selected" << std::endl;
  for(size_t i=0; i < selected->size(); i++)
    new_cloud_ptr->points[(*selected)[i]].rgb = *reinterpret_cast<float*>(&seg_color);
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(new_cloud_ptr);
  viewer.addPointCloud<PointT> (new_cloud_ptr, rgb, "sample cloud");
}

// ===  FUNCTION  ======================================================================
//         Name:  colorVisual
//  Description:
// =====================================================================================
void colorVisualInWindow(PointCloudT::ConstPtr cloud)
{
  viewer.setBackgroundColor(1, 1, 1);
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer.addPointCloud<PointT> (cloud, rgb, "sample cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer.initCameraParameters();
  viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&cloud);
  viewer.registerAreaPickingCallback(selectionAreaOccurred, (void*)&cloud);
}

  int
main ( int argc, char *argv[] )
{
  // Used data definition

  // Load in point cloud
  if(argc < 3) std::cout << "Usage: test.pcd output.pcd" << std::endl;
  infile = argv[1];
  outfile = argv[2];
  if(pcl::io::loadPCDFile<PointT>(infile, *cloud_ptr) != 0) {
    std::cout << "can not load into " << infile.c_str() << "." << std::endl;
    exit(-1);
  }
  std::cout << cloud_ptr->size() << " point cloud has been loaded into mem." << std::endl;

  colorVisualInWindow(cloud_ptr);

  // Main loop
  viewer.spin();
  return 0;
}				// ----------  end of function main  ----------
