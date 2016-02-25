#pragma once

#include "CRForest.h"
#include <fstream>
#include <Eigen/Geometry>
#include "opencv2/core/core.hpp"

struct Vote {

	cv::Vec<float,POSE_SIZE> vote;
	const float* trace;
	const float* conf;
	bool operator<(const Vote& a) const { return trace<a.trace; }

};

class CRForestEstimator {

public:
	// Path to trees
	std::string g_treepath;
	// Number of trees
	int g_ntrees;
	// Patch width
	int g_p_width;
	// Patch height
	int g_p_height;
	//maximum distance form the sensor - used to segment the person
	int g_max_z;
	//head threshold - to classify a cluster of votes as a head
	int g_th;
	//threshold for the probability of a patch to belong to a head
	float g_prob_th;
	//threshold on the variance of the leaves
	float g_maxv;
	//stride (how densely to sample test patches - increase for higher speed)
	int g_stride;
	//radius used for clustering votes into possible heads
	float g_larger_radius_ratio;
	//radius used for mean shift
	float g_smaller_radius_ratio;
	//camera intrinsics
	float depth_intrinsic[9];

	CRForestEstimator(){ crForest = 0; };

	~CRForestEstimator(){ if(crForest) delete crForest; };

	void loadConfig(const char* filename);

	bool loadForest(const char* treespath, int ntrees = 0);

	float rad(float angle);
	
	Eigen::Affine3f GetTransformation(float x, float y, float z, float roll, float pitch, float yaw);

	cv::Rect getBoundingBox(const cv::Mat& im3D);

	void estimate( const cv::Mat & im3D, //input: 3d image (x,y,z coordinates for each pixel)
                   std::vector< cv::Vec<float,POSE_SIZE> >& means, //output: heads' centers and orientations (x,y,z,pitch,yaw,roll)
                   std::vector< std::vector< Vote > >& clusters, //all clusters
                   std::vector< Vote >& votes, //all votes
                   int stride = 5, //stride
                   float max_variance = 1000, //max leaf variance
                   float prob_th = 1.0, //threshold on the leaf's probability of belonging to a head
                   float larger_radius_ratio = 1.0, //for clustering heads
                   float smaller_radius_ratio = 6.0, //for mean shift
                   bool verbose = false, //print out more info
                   int threshold = 400 //head threshold
	);


private:

	CRForest* crForest;

};


