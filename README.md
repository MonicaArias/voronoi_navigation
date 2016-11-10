# voronoi_navigation
Costmap plugin for the Navigation Stack to add a voronoi layer, to make the robot prioritize going in between obstacles.

It is assumed you are using the Navigation Stack in Indigo version.


First download and compile the package with catkin_make.
You might have to execute the following command to make the config files executable:
     sudo chmod +x cfg/ObstaclePlugin.cfg


To use in your application, edit the plugins in your "global_costmap_params.yaml" file inside the param folder, 
such that they look like this: 

    plugins:  
     - {name: static_layer,            type: "costmap_2d::StaticLayer"} 
     - {name: obstacle_layer,          type: "voronoi_layer_namespace::ObstacleLayer"}          
     - {name: inflation_layer,         type: "costmap_2d::InflationLayer"}      
     - {name: voronoi_layer,          type: "voronoi_layer_namespace::VoronoiLayer"}   
     

Add the following line to your "costmap_global_common_params.yaml":

    voronoi_layer:
        enabled:              true


The launch file "simulation.launch" launches the gazebo simulation of the turtlebot with the navigation stack.

Computation of the voronoi diagram was made with the dynamic voronoi library available at: http://www.cs.utexas.edu/users/qr/software/evg-thin.html
