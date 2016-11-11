/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id:$
 *
 */

// STL
#include <iostream>

// PCL
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "godel_scan_tools/surface_segmentation.h"
#include "godel_scan_tools/background_subtraction.h"

int
main (int argc, char** av)
{
  if (argc < 3)
  {
    pcl::console::print_info ("Syntax is: %s <source-pcd-background_file> <source-pcd-with_part_file> [-dump] [-select_segment]\n\n", av[0]);
    pcl::console::print_info ("If -dump is provided write the extracted clusters to cluster.dat\n\n");
    return (1);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr bg_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr bg_cloud_nonans(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_nans (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

  pcl::PCDWriter writer;
  if (pcl::io::loadPCDFile(av[1], *bg_cloud_ptr)==-1)
  {
    return -1;
  }
  if (pcl::io::loadPCDFile(av[2], *cloud_ptr)==-1)
  {
    return -1;
  }

  pcl::console::print_highlight ("Loaded background %s of size %lu and part %s of size %lu\n", 
				 av[1], bg_cloud_ptr->points.size (),
				 av[2], cloud_ptr->points.size ());


  
  // Remove the nans from both the part scan and the background scan
  cloud_ptr->is_dense = false;
  bg_cloud_ptr->is_dense = false;
  cloud_no_nans->is_dense = false;
  bg_cloud_nonans->is_dense = false;
  std::vector<int> indices, bg_indices;

  pcl::console::print_highlight ("Remove NANs from clouds\n");
  pcl::removeNaNFromPointCloud (*cloud_ptr, *cloud_no_nans, indices);
  pcl::removeNaNFromPointCloud (*bg_cloud_ptr, *bg_cloud_nonans, bg_indices);

  // remove the background from the part scan
  pcl::console::print_highlight ("subtract background\n");
  background_subtraction BS(bg_cloud_nonans, 5.0);
  for(int i=0; i<cloud_no_nans->points.size(); i+=(rand()%100)*cloud_no_nans->points.size()*.000005){
    cloud_no_nans->points[i].z +=8.0;
  }
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr part_cloud_ptr = BS.remove_background(cloud_no_nans);
  pcl::PointCloud<pcl::PointXYZ>::Ptr part_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  BOOST_FOREACH(pcl::PointXYZ pt, cloud_no_nans->points){
    int q = rand()%10;
    if (q ==0){
      if(pt.x !=0.0 && pt.y!=0.0 && pt.z !=0.0)      part_cloud_ptr->push_back(pt);
    }
  }

  // Segment the part into surface regions using a "region growing" scheme
  pcl::console::print_highlight ("segmenting\n");
  surfaceSegmentation SS(part_cloud_ptr);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  std::vector <pcl::PointIndices> clusters = SS.computeSegments(colored_cloud_ptr);
  pcl::console::print_highlight ("Segmented into %d clusters, colored cloud has %d points\n", clusters.size(), colored_cloud_ptr->points.size());

  // Write the resulting setmented cloud into a pcd file
  writer.write<pcl::PointXYZRGB> ("segmented_part.pcd", *colored_cloud_ptr, false);

  // at this point, we would want to select a segment for which we want the edges
  int selected_segment=-1;
  pcl::console::parse_argument (argc, av, "-select_segment", selected_segment);
  if(selected_segment<0){
    int max_size = 0;
    for(int i=0;i<clusters.size();i++){
      if(clusters[i].indices.size() > max_size){
	max_size = clusters[i].indices.size();
	selected_segment = i;
      }
    }
  }
  pcl::console::print_highlight ("select_segment %d has %d points\n", selected_segment, clusters[selected_segment].indices.size());
 
  // extract the selected segment from the part cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_surface_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_surface_ptr2(new pcl::PointCloud<pcl::PointXYZRGB>());
  for(int i=0; i<clusters[selected_segment].indices.size(); i++){
    pcl::PointXYZRGB pt(255,0,0);
    pt.x = part_cloud_ptr->points[i].x;
    pt.y = part_cloud_ptr->points[i].y;
    pt.z = part_cloud_ptr->points[i].z;
    segmented_surface_ptr2->points.push_back(pt); // save a red point cloud with same indices as colorless one
    segmented_surface_ptr->points.push_back(part_cloud_ptr->points[i]);
  }
  
  // now create a mesh from just that segment
  pcl::console::print_highlight ("computing mesh\n");
  SS.setInputCloud(segmented_surface_ptr);
  Mesh  tmesh =   SS.computeMesh();

  pcl::console::print_highlight ("computing boundary\n");
  std::vector<Mesh::HalfEdgeIndices> boundary_half_edges;
  SS.getBoundBoundaryHalfEdges(tmesh, boundary_half_edges);

  for(int j=0; j<boundary_half_edges.size(); j++){
    for(int i=0; i<boundary_half_edges[j].size(); i++){
      pcl::geometry::VertexIndex ei = tmesh.getOriginatingVertexIndex(boundary_half_edges[j][i]);
      int k = ei.get();
      segmented_surface_ptr2->points[k].r = 255;
      segmented_surface_ptr2->points[k].g = 255;
      segmented_surface_ptr2->points[k].b = 0;
    }
  }

  //  pcl::console::print_highlight ("visualization boundary has %d edges\n", boundary_half_edges.size());

  // show surface with boundary line drawn
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(segmented_surface_ptr2, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(segmented_surface_ptr2);
  viewer->addPointCloud<pcl::PointXYZRGB> (segmented_surface_ptr2, rgb, "surface cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "surface cloud");
  //  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  // note, all boundaries are provided, some may be interior. The exterior one is the one we want, but which is that? 
  // perhaps it is the longest, but perhaps not. If the points are uniformly distributed over the surface, then most of the time
  // the longest boundary_half_edge vector will be the exterior. 
  int q=0;
  for(int j=0; j<boundary_half_edges.size(); j++){
    //    pcl::console::print_highlight ("boundary %d has %d edges\n", j,boundary_half_edges[j].size());
    for(int i=0; i<boundary_half_edges[j].size(); i++){
      pcl::geometry::VertexIndex pnt1_index = tmesh.getOriginatingVertexIndex(boundary_half_edges[j][i]);
      pcl::geometry::VertexIndex pnt2_index = tmesh.getTerminatingVertexIndex(boundary_half_edges[j][i]);
      char line_number[255];
      sprintf(line_number,"%03d",q++);
      std::string ls = std::string("line_") + std::string(line_number);
      int d1 = pnt1_index.get();
      int d2 = pnt2_index.get();

      viewer->addLine<pcl::PointXYZ> (segmented_surface_ptr->points[d1],
				      segmented_surface_ptr->points[d2],
				      ls.c_str());
      viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.99, 0, ls.c_str());
    }
  }
  if (pcl::console::find_switch (argc, av, "-dump"))
  {
    pcl::console::print_highlight ("Writing clusters to clusters.dat\n");
    std::ofstream clusters_file;
    clusters_file.open ("clusters.dat");
    for (std::size_t i = 0; i < clusters.size (); ++i)
    {
      clusters_file << i << "#" << clusters[i].indices.size () << ": ";
      std::vector<int>::const_iterator pit = clusters[i].indices.begin ();
      clusters_file << *pit;
      for (; pit != clusters[i].indices.end (); ++pit)
        clusters_file << " " << *pit;
      clusters_file << std::endl;
    }
    clusters_file.close ();
  }

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return (0);
}
