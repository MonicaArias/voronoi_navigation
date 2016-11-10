/**
 * \file voronoi_layer.cpp
 * \A plugin to modify a costmap from the costmap_2d package
 * \author Monica Arias && Nataliya Nechyporenko
 * \version 0.1
 * \date 23 June 2016
 * 
 *
 */


#include<voronoi_layer/voronoi_layer.h>
#include <pluginlib/class_list_macros.h>
#include "dynamicvoronoi.h"


PLUGINLIB_EXPORT_CLASS(voronoi_layer_namespace::VoronoiLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;
const unsigned char VORONOI_SPACE = 1;
const unsigned char OBSTACLE_LAYER_FREE_SPACE = 100;


namespace voronoi_layer_namespace
{

VoronoiLayer::VoronoiLayer() {}

void VoronoiLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();
  
  
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>
  ::CallbackType cb = boost::bind(&VoronoiLayer::reconfigureCB, this, _1, _2);
  
  dsrv_->setCallback(cb);
}


void VoronoiLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
   size_x_ = master->getSizeInCellsX();
   size_y_ = master->getSizeInCellsY();
}


void VoronoiLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}


void VoronoiLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

}

void VoronoiLayer::updateCosts(costmap_2d::Costmap2D& master_grid, 
                int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  
  //Only modify the costs if the update is for the whole costmap
  if((max_i-min_i) >= size_x_ && (max_j-min_j) >= size_y_){	  
	  
	  //Creating an empty bool array of matching size to the costmap
	  bool **map=NULL;	  
	  map = new bool*[max_i-min_i];	
	  for (int x=0; x<(max_i-min_i); x++) {
		map[x] = new bool[max_j-min_j];
	  }
	  
	  //Assigning true if an obstacle is present
	  for (int j = min_j; j < max_j; j++)
	  {
	    for (int i = min_i; i < max_i; i++)
	    {
	      if(i == min_i || i == max_i-1 || j == min_j || j == max_j-1 ||
	        master_grid.getCost(i,j) == LETHAL_OBSTACLE){
	          map[i-min_i][j-min_j]= true;	        
		  }else{
			  map[i-min_i][j-min_j]= false;
		  }	    
	    }
	  }
	  
	  //Calculating the GVD
	  DynamicVoronoi voronoi;
	  voronoi.initializeMap(size_x_, size_y_, map);
	  voronoi.update(); // update distance map and Voronoi diagram
	  voronoi.prune();  // prune the Voronoi
	  
	  //Modifying the cost of the global costmap
	  for (int j = min_j; j < max_j; j++)
	  {
	    for (int i = min_i; i < max_i; i++)
	    {
	      //don't modify cost if there is an obstacle (including inflation)
	      if(master_grid.getCost(i,j) > OBSTACLE_LAYER_FREE_SPACE) 
			continue;
	      
	      if (voronoi.isVoronoi(i-min_i,j-min_j))
	        master_grid.setCost(i, j, VORONOI_SPACE); 
	    }
	  }
  }  
}

} // end namespace
