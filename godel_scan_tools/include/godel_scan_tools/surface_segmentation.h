#ifndef SURFACE_SEGMENTATION_H
#define SURFACE_SEGMENTATION_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>
#include <boost/foreach.hpp>


/** @class world_background_subtraction
      @brief Maintains record of baseline sensor data to provide method to remove them leaving only new objects in the scene
*/
class surfaceSegmentation{
 public:
  /** @brief default constructor */
  surfaceSegmentation()
    {
      // initialize pointers to cloud members
      input_cloud_= pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }
  /** @brief distructor */
  ~surfaceSegmentation()
    {
      input_cloud_->clear();
    };

  /** @brief constructor that sets the background cloud, also initializes the KdTree for searching
      @param bg_cloud the set of points defining the background
  */
  surfaceSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr icloud)
    {
      input_cloud_ =  pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      setInputCloud(icloud);
      removeNans();
      computeNormals();
    };
  /** @brief sets the background cloud, replaces whatever points exists if any
      @param background_cloud the cloud representing the background
  */
  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr icloud){
    input_cloud_->clear();
    BOOST_FOREACH(pcl::PointXYZ pt, *icloud){
      input_cloud_->push_back(pt);
    }
    kd_tree_.setInputCloud(input_cloud_);
    removeNans();
    computeNormals();
  }
  
  /** @brief adds new points to the background, and reinitializes the kd_tree for searching
      @param bg_cloud additional background points
  */
  void addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr icloud){
    // push input_cloud onto icloud and then add, this strange sequence keeps ordering of clouds and does not duplicate setInputCloud code
    BOOST_FOREACH(pcl::PointXYZ pt, input_cloud_->points){
      icloud->push_back(pt);
    }
    setInputCloud(icloud);
  }

  std::vector <pcl::PointIndices> computeSegments(pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud)
  {
    // Region growing
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
    rg.setSmoothModeFlag (false); // Depends on the cloud being processed
    rg.setSmoothnessThreshold (15.0 / 180.0 * M_PI);
    rg.setMaxClusterSize(1000000);
    rg.setSearchMethod (tree);
    rg.setMinClusterSize(10);
    rg.setNumberOfNeighbours (30);
    float smooth_thresh = rg.getSmoothnessThreshold();
    float resid_thresh = rg.getResidualThreshold();
    // rg.setCurvatureTestFlag();
    rg.setResidualTestFlag(true);
    rg.setResidualThreshold(resid_thresh);
    rg.setCurvatureThreshold(1.0);
    
    rg.setInputCloud (input_cloud_);
    rg.setInputNormals (normals_);
    
    rg.extract (clusters_);
    colored_cloud = rg.getColoredCloud();
    return(clusters_);
  }
  

  /** @brief computes mesh on the cloud results are in triangles_, parts_, and states_ */
  pcl::geometry::TriangleMesh computeMesh()
  {
    // concatenate normals to cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*input_cloud_, *normals_, *cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud (cloud_with_normals_);

    // Set typical values for the parameters
    gp3.setSearchRadius (0.025);
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);
    gp3.setSearchMethod (tree);    
    gp3.setInputCloud (cloud_with_normals_);

    // Get results
    gp3.reconstruct (triangles_);
    parts_ = gp3.getPartIDs();
    states_ = gp3.getPointStates();
    
    pcl::geometry::toHalfEdgeMesh(triangles_, HEM_);
    return(HEM_);
  }
  /*
  pcl::PointXYZ computeSegmentEdges(std::v segment_index)
  {
    // for each point on the surface segment, check to see if its on the boundary
    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    std::vector<int> used;
    pcl::PointIndices segment_point_indices = clusters[segment_index];
    for(int i=0; i<segment_point_indicies.size(); i++){
      if(states_(segment_point_indices[i]) == GP2::BOUNDARY){
	temp_cloud->push_back(input_cloud_->Points[i]);
	used.push_back(0);
      }
    }
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (temp_cloud);
    std::vec
    std::vector<int> indices;
    std::vector<float> d;
    for(int i=0;i<temp_cloud.Points.size();i++){
      pcl::PointXYZ pt = temp_cloud.Points[i];
      tree->nearestKSearch(pt, 5, indices, d);
      for(int j=0; j<5; j++){
	if(!used[indices[j]]){
	  used(indices[j] = 1;
	  
	}


  }
  */
 private:
  /** @brief remove any NAN points, otherwise many algorityms fail */
  void removeNans()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    nonans_cloud->is_dense = false;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud (*input_cloud_, *nonans_cloud, indices);
    setInputCloud(nonans_cloud);
  }
  /** @brief compute the normals and store in normals_, this is requried for both segmentation and meshing*/
  void computeNormals()
  {
    // Estimate the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (input_cloud_);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree); 

    //  ne.setRadiusSearch (3.0);
    ne.setKSearch (50);
    ne.compute (*normals_);
  }

 private:
   pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_; // used for computing normals and segments
   pcl::PointCloud<pcl::Normal>::Ptr normals_;
   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_;
 public:
      // mesh results 
      pcl::PolygonMesh triangles_;
      std::vector<int> parts_; 
      std::vector<int> states_;

      // segmentation results
      std::vector <pcl::PointIndices> clusters_;
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
      HalfEdgeMesh HEM_;
};
#endif
