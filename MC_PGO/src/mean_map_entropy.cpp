//============================================================================
// Name        : MeanMapEntropy.cpp
// Author      : David Droeschel & Jan Razlaw
// Version     :
// Copyright   :
// Description : Calculates Mean Map Entropy and Mean Plane Variance of a point cloud
//============================================================================

#include <omp.h>

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>


struct PointTypeWithEntropy
{
	PCL_ADD_POINT4D;                  	// preferred way of adding a XYZ + padding
	float entropy;
	float planeVariance;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   	// make sure our new allocators are aligned
} EIGEN_ALIGN16;                    		// enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointTypeWithEntropy,
		(float, x, x)
		(float, y, y)
		(float, z, z)
		(float, entropy, entropy)
		(float, planeVariance, planeVariance)
)

typedef pcl::PointXYZ PointT;


double computeEntropy( pcl::PointCloud< PointT >::Ptr cloud ){

	Eigen::Vector4f centroid;
	Eigen::Matrix3f covarianceMatrixNormalized = Eigen::Matrix3f::Identity();;

	// estimate the XYZ centroid and the normalized covariance matrix
	pcl::compute3DCentroid (*cloud, centroid);
	pcl::computeCovarianceMatrixNormalized (*cloud, centroid, covarianceMatrixNormalized);

	// compute the determinant and return the entropy
	double determinant = static_cast<double>((( 2 * M_PI * M_E) * covarianceMatrixNormalized).determinant());

	return 0.5f*log(determinant);
}


double computePlaneVariance( pcl::PointCloud< PointT >::Ptr cloud ){

	double meanDistTopQuarter = 0;

	std::vector<double> sortedDistances;

	// fit plane using RANSAC
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	pcl::SACSegmentation< PointT > seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.005);
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);

	if( inliers->indices.size() < 3 ){
		PCL_ERROR ("Could not estimate a planar model for the given subset of points.");
		meanDistTopQuarter = std::numeric_limits<double>::infinity();
	}else{
		// compute the distances of the points to the plane
		for( size_t i = 0; i < cloud->points.size(); ++i ){
			double distancePointToPlane = (cloud->points[i].x * coefficients->values[0]) + (cloud->points[i].y * coefficients->values[1]) +( cloud->points[i].z * coefficients->values[2] ) + coefficients->values[3];
			sortedDistances.push_back(fabs(distancePointToPlane));
		}
		// sort distances
		std::sort(sortedDistances.begin(), sortedDistances.end());

		// compute mean of quartile that contains the largest distances
		int quarterOfArray = sortedDistances.size() / 4;
		for( size_t i = quarterOfArray * 3; i < sortedDistances.size(); i++ ){
			meanDistTopQuarter += sortedDistances[i];
		}
		meanDistTopQuarter /= static_cast<double> (quarterOfArray);
	}

	return meanDistTopQuarter;
}


int main( int argc, char** argv ) {

	pcl::PointCloud< PointT >::Ptr inputCloud (new pcl::PointCloud< PointT >);
	pcl::PointCloud< PointTypeWithEntropy >::Ptr outputCloud (new pcl::PointCloud< PointTypeWithEntropy >);

	double entropySum = 0.f;
	double planeVarianceSum = 0.f;
	int lonelyPoints = 0;

	// get pointcloud
	std::vector<int> fileIndices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	if (pcl::io::loadPCDFile< PointT> (argv[fileIndices[0]], *inputCloud) == -1)
	{
		PCL_ERROR ("Couldn't read file.\n");
		return (-1);
	}

	// get parameters if given
	int stepSize = 1;
	double radius = 0.3;
	int minNeighbors = 15;

	pcl::console::parse_argument (argc, argv, "-stepsize", stepSize);
    	pcl::console::parse_argument (argc, argv, "-radius", radius);
	bool punishSolitaryPoints = pcl::console::find_switch (argc, argv, "-punishSolitaryPoints");
    	pcl::console::parse_argument (argc, argv, "-minNeighbors", minNeighbors);

    	std::cout << "Stepsize = " << stepSize << std::endl;
	std::cout << "Radius for neighborhood search = " << radius << std::endl;
	if( !punishSolitaryPoints ){
		std::cout << "Paper version" << std::endl;
	}else{
		std::cout << "Punishing solitary points. \nMinimal number of neighbors that have to be found = " << minNeighbors << std::endl;
	}
	std::cout << "--- " << std::endl;

	pcl::StopWatch entropyTimer;
	entropyTimer.reset();

	pcl::KdTreeFLANN< PointT> kdtree;
	kdtree.setInputCloud (inputCloud);

	#pragma omp parallel reduction (+:entropySum, planeVarianceSum, lonelyPoints)
	{
		#pragma omp for schedule(dynamic)
		for (size_t i = 0; i < inputCloud->points.size(); i += stepSize ) {

			// print status
			if( i % (inputCloud->points.size()/20) == 0 ){
				int percent = i * 100 / inputCloud->points.size();
				std::cout << percent << " %" << std::endl;
			}

			// search for neighbors in radius
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			int numberOfNeighbors = kdtree.radiusSearch (inputCloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			// compute values if enough neighbors found
			double localEntropy = 0;
			double localPlaneVariance = 0;
			if( numberOfNeighbors > minNeighbors || !punishSolitaryPoints ){

				// save neighbors in localCloud
				pcl::PointCloud< PointT>::Ptr localCloud (new pcl::PointCloud< PointT>);

				for( size_t iz = 0; iz < pointIdxRadiusSearch.size(); ++iz ){
					localCloud->points.push_back(inputCloud->points[ pointIdxRadiusSearch[iz] ] );
				}

				// compute entropy and plane variance
				localEntropy = computeEntropy(localCloud);
				//localPlaneVariance = computePlaneVariance(localCloud);
			}else{
				localEntropy = std::numeric_limits<double>::infinity();
				//localPlaneVariance = std::numeric_limits<double>::infinity();
				lonelyPoints++;
			}

			// save values in new point
			PointTypeWithEntropy p;
			p.x = inputCloud->points[i].x;
			p.y = inputCloud->points[i].y;
			p.z = inputCloud->points[i].z;

            /*
			if (pcl_isfinite(localPlaneVariance)){
				planeVarianceSum += localPlaneVariance;
				p.planeVariance = static_cast<float>(localPlaneVariance);
			}else{
				// handle cases where no value could be computed
				if( !punishSolitaryPoints ){
					p.planeVariance = 0;
				}else{
					planeVarianceSum += radius;
					p.planeVariance = static_cast<float>(radius);
				}
			}
             */
			if (pcl_isfinite(localEntropy)){
				entropySum += localEntropy;
				p.entropy = static_cast<float>(localEntropy);
			}else{
				// handle cases where no value could be computed
				p.entropy = 0;
			}

			// add new point to output cloud
			#pragma omp critical
			{
				outputCloud->push_back( p );
			}
		}
	}

	// compute mean
	double meanMapEntropy = entropySum / (static_cast<double>(inputCloud->points.size() / stepSize));
	//double meanPlaneVariance = planeVarianceSum / (static_cast<double>(inputCloud->points.size() / stepSize));

	std::cout << "--- " << std::endl;
	std::cout << "Mean Map Entropy is " << meanMapEntropy << std::endl;
	//std::cout << "Mean Plane Variance is " << meanPlaneVariance << std::endl;

	std::cout << "Used " << entropyTimer.getTime() << " milliseconds to compute values for " << inputCloud->points.size() << " points." << std::endl;

	int pointsActuallyUsed = (inputCloud->points.size() / stepSize) - lonelyPoints;
	
	if( punishSolitaryPoints && (pointsActuallyUsed < lonelyPoints) ){
		std::cout << "Used more solitary than not-solitary points to compute the values. You should consider changing the parameters." << std::endl;
	}

	// save output cloud in the directory of the input cloud
	std::string saveDestination = argv[fileIndices[0]];
	saveDestination.replace(saveDestination.find_last_of("."),1,"_entropy.");
	if ( outputCloud->size() > 0 )
		pcl::io::savePCDFileASCII (saveDestination, *outputCloud );
	else
		PCL_ERROR ("Empty cloud. Saving error.\n");

	return 0;
}
