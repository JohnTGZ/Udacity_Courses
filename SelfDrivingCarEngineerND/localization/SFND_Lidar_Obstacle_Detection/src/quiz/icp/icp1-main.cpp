// Udacity C3 Localization
// Dec 21 2020
// Aaron Brown

using namespace std;

#include <string>
#include <sstream>
#include "helper.h"

#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc

/**
 * @brief 
 * 
 * @param target BLUE scan what you are trying to align the scan to
 * @param source second RED scan to align
 * @param startingPose 
 * @param iterations 
 * @return Eigen::Matrix4d 
 */
Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations){

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

	//TODO: complete the ICP function and return the corrected transform

    PointCloudT::Ptr src_transformed(new PointCloudT);
    Eigen::Matrix4d src_to_startpose_tf = transform2D(startingPose.theta, startingPose.position.x, startingPose.position.y);

    //1. Transform the source to the startingPose
	// transformPointCloud(INPUT, OUTPUT, TRANSFORM)
    pcl::transformPointCloud(*source, *src_transformed, src_to_startpose_tf);
  	
    //2. Create the PCL icp object
    pcl::IterativeClosestPoint<PointT, PointT> icp;

    //3. Set the icp object's values
    icp.setMaximumIterations(iterations);
    icp.setInputSource(src_transformed);
    icp.setInputTarget(target);
    // icp.setMaxCorrespondenceDistance()

    //4. Call align on the icp object.
	// This gives us the end result of the point cloud
    PointCloudT::Ptr cloud_icp(new PointCloudT);
    icp.align(*cloud_icp);

	// renderPointCloud(viewer, cloud_icp, "cloud_icp", Color(0,1,1), 10);

    //5. If icp converged get the icp objects output transform and adjust it by the startingPose, return the adjusted transform
    if (icp.hasConverged()){
        std::cout << "ICP has converged, with fitness score: " <<  icp.getFitnessScore() << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		
		//transformation should be done from starting pose, so it is transformed to starting pose.
		transformation_matrix = transformation_matrix * src_to_startpose_tf;
	
    //6. If icp did not converge log the message and return original identity matrix
    } 
	else {
        std::cout << "ICP did not converge, with fitness score: " <<  icp.getFitnessScore() << std::endl;
    }

	return transformation_matrix;

}

int main(){

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);

	// create a room
	double lowerX = -5;
	double upperX = 5;
	double lowerY = -5;
	double upperY = 5;
	vector<LineSegment> room;
	LineSegment top(0, 1, upperY, lowerX, upperX);
	room.push_back(top);
	LineSegment bottom(0, 1, lowerY, lowerX, upperX);
	room.push_back(bottom);
	LineSegment right(1, 0, upperX, lowerY, upperY);
	room.push_back(right);
	LineSegment left(1, 0, lowerX, lowerY, upperY);
	room.push_back(left);

	// create lidar
	// x=0, y=0, yaw=0, max_range= 100, resolution=128
	Lidar lidar(0, 0, 0, 100, 128);

	PointCloudT::Ptr poses (new PointCloudT); 	// ground truth
	PointCloudT::Ptr locator (new PointCloudT); // estimated locations

	// starting location
	poses->points.push_back(PointT(lidar.x, lidar.y, 0));
	locator->points.push_back(PointT(lidar.x, lidar.y, 0));

	// get map of room
	PointCloudT::Ptr map = lidar.scan(room);
	cout << "map captured " << map->points.size() << " points" << endl;

	// move around the room

	// Part 1. Localize from single step
	vector<Vect2> movement = {Vect2(0.5,pi/12)};

	// Part 2. TODO: localize after several steps
	if(true){ // Change to true
		movement.push_back(Vect2(0.8, pi/10));
		movement.push_back(Vect2(1.0, pi/6));
	}
	// Part 3. TODO: localize after randomly moving around the whole room
	if(true){ // Change to true
		srand(time(0));
		for(int i = 0; i < 10; i++){
			double mag = 0.5 * ((double) rand() / (RAND_MAX)) + 0.5;
			double angle = pi/8 * ((double) rand() / (RAND_MAX)) + pi/8;
			movement.push_back(Vect2(mag, angle));
		}
	}

	renderPointCloud(viewer, map, "map", Color(0,0,1)); // render map
	Pose location(Point(0,0), 0); //start off at center of map (0,0, yaw=0)
	PointCloudT::Ptr scan;
	int count = 0;
	for( Vect2 move : movement ){

		// execute move
		lidar.Move(move.mag, move.theta);
		poses->points.push_back(PointT(lidar.x, lidar.y, 0));

		// scan the room
		scan = lidar.scan(room);
		cout << "scan captured " << scan->points.size() << " points" << endl;
		renderPointCloud(viewer, scan, "scan_"+to_string(count), Color(1,0,0)); // render scan
		
		// perform localization
		Eigen::Matrix4d transform = ICP(map, scan, location, 40); //TODO: make the iteration count greater than zero
		Pose estimate = getPose(transform);
		// TODO: save estimate location and use it as starting pose for ICP next time
		location = estimate;

		locator->points.push_back(PointT(estimate.position.x, estimate.position.y, 0));
		
		// view transformed scan
        PointCloudT::Ptr scan_transformed(new PointCloudT);
		// TODO: perform the transformation on the scan using transform from ICP
        pcl::transformPointCloud(*scan, *scan_transformed, transform);
		// TODO: render the correct scan
		renderPointCloud(viewer, scan_transformed, "scan_icp", Color(0,1,0));

		count++;
	}

	// display ground truth poses vs estimated pose
	renderPointCloud(viewer, poses, "poses", Color(0,1,0), 8);
	renderPath(viewer, poses, "posePath", Color(0,1,0) );
	renderPointCloud(viewer, locator, "locator", Color(0,0,1), 6);
	renderPath(viewer, locator, "locPath", Color(0,0,1) );

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce ();
	}
		
	return 0;
}
