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
  bg_cloud_ptr->isdense = false;
  cloud_no_nans->is_dense = false;
  bg_cloud_nonans->isdense = false;
  std::vector<int> indices, bg_indices;
  pcl::removeNaNFromPointCloud (*cloud_ptr, *cloud_no_nans, indices);
  pcl::removeNaNFromPointCloud (*bg_cloud_ptr, *bg_cloud_nonans, bg_indices);
  pcl::console::print_highlight ("Removed nans from background %lu to %lu\n", bg_cloud_ptr->points.size (), bg_cloud_nonans->points.size ());
  pcl::console::print_highlight ("Removed nans from part %lu to %lu\n", cloud_ptr->points.size (), cloud_no_nans->points.size ());

  // remove the background from the part scan
  background_subtraction BS(bg_cloud_nonans, 5.0);
  pcl::PointCloud<pcl::PointXYZ> part_cloud = remove_background(cloud_no_nans);

  // Segment the part into surface regions using a "region growing" scheme
  surfaceSegmentation SS(part_cloud);
  std::vector <pcl::PointIndices> clusters = SS.compute_Segments(colored_cloud_ptr);

  // Write the resulting setmented cloud into a pcd file
  writer.write<pcl::PointXYZRGB> ("segmented_part.pcd", *cloud_segmented, false);

  // at this point, we would want to select a segment for which we want the edges
  int selected_segment;
  pcl::console::parse_argument (argc, av, "-select_segment", selected_segment);
  pcl::console::print_highlight ("selecting segment %d for edge processing\n", selected_segment);
 
  // extract the selected segment from the part cloud
  pcl::PointCloud<pcl::PointXYZ> segmented_surface;
  for(int i=0; i<clusters[selected_segment].size(); i++){
    segmented_surface.push_back(cloud_no_nans.points[i]);
  }
  
  // now create a mesh from just that segment
  SS.setInputcloud(segmented_surface);
  pcl::geometry::TriangleMesh  tmesh =   SS.computeMesh();

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

  return (0);
}
