/// Pose Graph Optimization based on submaps

/// Author: Weitong Wu
/// Contact: wwtgeomatics@gmail.com
/// Date: 2023/6/1
/// Update: 2023/10/12
 
#include <iostream>
#include <omp.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include "io.h"
#include "types.h"
#include "imu_process.h"
#include "common_pt_operations.h"
#include "registration.h"
#include "fast_gicp/fast_gicp.hpp"
#include "fast_gicp/fast_vgicp.hpp"
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/io/LasWriter.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/StageWrapper.hpp>
#include <pdal/io/BufferReader.hpp>

using namespace std;
using namespace pdal;

#define NoConsecutiveSubmapSearch 10.01 //submap index search
#define EnableNonPlanarFilter true
#define EnableOverlapConstraint true
#define EnableLoopConstraint true

int main(int argc, char **argv) {
    cout << "Let's start pgo!" << endl;
    ros::init(argc, argv, "pgo_node");
    ros::NodeHandle nh;

    double whole_start_time = omp_get_wtime();

    string rosbag_path = string(argv[1]); // absolute path
    int rosbag_name_index = rosbag_path.find_last_of("/");
    cout << "the bag processed is: " << rosbag_path.substr(rosbag_name_index + 1) << endl;

    string odometry_pose_path = string(argv[2]);

    string extrinsic_path = string(argv[3]);

    int lidar_type = stoi(argv[4]);
    cout << "the lidar type is: ";
    switch (lidar_type) {
        case LIVOX:
            cout << "livox!" << endl;
            break;

        case VELO16:
            cout << "vlp-16!" << endl;
            break;

        case OUST64:
            cout << "ouster!" << endl;
            break;

        case HESAI:
            cout << "hesai!" << endl;
            break;

        default:
            cout << "Error LiDAR type!" << endl;
            return 0;
    }

    string lidar_topic = string(argv[5]);
    cout << "the lidar topic is: " << lidar_topic << endl;

    float bag_start, bag_dur;
    bag_start = stof(argv[6]);
    bag_dur = stof(argv[7]);
    cout << "bag start: " << bag_start << endl;
    cout << "bag_dur: " << bag_dur << endl;

    float submap_length; 
    float overlap_threshold;
    submap_length = stof(argv[8]);
    overlap_threshold = stof(argv[9]);

    int IfSimulation;
    IfSimulation = stoi(argv[10]);

    int submap_regis = 0; // 0、Ourmethod 1、NDT 2、FAST-GICP、3、VGICP
    //NDT parameter
    float ndt_voxel = 0.5;
    float ndt_outlier_ratio = 0.6;

    //fast gicp parameter
    int correspond_num = 0;

    //vgicp parameter
    float vgicp_voxel = 1.0;

    //for parameter sensitivity analysis
    //for (int j = 0; j < 4; ++j) {
    //submap_length += 1;
    //overlap_threshold += 0.1;
    //ndt_outlier_ratio += 0.1;
    //correspond_num += 5;
    //vgicp_voxel += 0.5;
    //submap_regis -= 1;

    /// Create result dir
    string root_dir = rosbag_path.substr(0, rosbag_name_index);
    //cout << "root dir: " << root_dir << endl;
    stringstream result_dir_path;
    double current_time = ros::Time().now().toSec();
    result_dir_path << root_dir << "/LOG_pgo_" << setprecision(13) << current_time;
    EnsureDir(result_dir_path.str());

    //1.load data
    double start_time = omp_get_wtime();
    vector<PoseData> odometry_poses;
    loadPoseFile(odometry_pose_path, odometry_poses);

    gtsam::Pose3 li_extrinsic;
    loadExtrinsic(extrinsic_path, li_extrinsic);

    //2.load rosbag information
    vector<ImuData> imu_data;
    double imu_dt;
    int point_filter_num = 3;
    loadImuDataFromRosbag(rosbag_path, imu_data, imu_dt);
    vector<LidarData> lidar_data;
    vector<string> topics;
    topics.push_back(lidar_topic);
    loadLidarDataFromRosbag(rosbag_path, lidar_type, point_filter_num, topics, bag_start, bag_dur, lidar_data);
    double load_data_time = omp_get_wtime() - start_time;

    /// Reorganize lidar data
    ITPointCloudPtr all_lidar_data(new ITPointCloud());
    for (int i = 1; i < lidar_data.size(); i++) {
        auto one_lidar_data = lidar_data[i];
        for (int j = 0; j < one_lidar_data.pointcloud.size(); ++j) {
            PointXYZIT pt;
            pt = one_lidar_data.pointcloud[j];
            pt.relative_time_ms =
                    one_lidar_data.start_time_s + one_lidar_data.pointcloud[j].relative_time_ms / 1000.0;
            //cout << setprecision(16) << pt.relative_time_ms << " ";
            all_lidar_data->push_back(pt);
        }
    }
    cout << "all lidar point size:" << all_lidar_data->size() << endl;
    lidar_data.clear();

    //3. Construct Submap
    start_time = omp_get_wtime();
    vector<Submap> vec_submap;
    int submap_size = bag_dur / submap_length;
    cout << "submap size: " << submap_size << endl;
    int pose_index = 0;
    int imu_data_start_index = 0;
    int imu_data_end_index = 0;
    int point_index = 0;

    //find pose_index corresponding to the bag start time
    for (int j = pose_index; j < odometry_poses.size() ; ++j) {
        if (fabs (odometry_poses[j].timestamp - all_lidar_data->front().relative_time_ms) < 0.05)
        {
            //cout << setprecision(13) << "odometry_poses[j].timestamp: " << odometry_poses[j].timestamp << endl;
            //cout << setprecision(13) << "all_lidar_data->front().relative_time_ms: " << all_lidar_data->front().relative_time_ms << endl;
            pose_index = j;
            break;
        }
    }
    cout << "pose start index: " << pose_index << endl;

    for (int i = 0; i < submap_size; ++i) {
        Submap one_submap;
        //submap start_time and end_time
        //put odom pose in
        one_submap.submap_start_time = odometry_poses[pose_index].timestamp;
        one_submap.submap_pose = odometry_poses[pose_index].pose;
        one_submap.odom_poses.push_back(odometry_poses[pose_index]);
        for (int j = pose_index + 1; j < odometry_poses.size(); ++j) {
            one_submap.odom_poses.push_back(odometry_poses[j]);
            if (odometry_poses[j].timestamp > one_submap.submap_start_time + submap_length) {
                one_submap.submap_end_time = odometry_poses[j].timestamp;
                pose_index = j;
                break;
            }
            if (j == odometry_poses.size() - 1) {
                one_submap.submap_end_time = odometry_poses[j].timestamp;
            }
        }

        //put imu measurement in
        vector<ImuData> one_segment_imu;
        find_imu_data_in_one_scan(one_submap.submap_start_time, one_submap.submap_end_time,
                                  imu_data, imu_data_start_index, imu_data_end_index, one_segment_imu);
        one_submap.imu_data = one_segment_imu;

        //put raw pointcloud in
        find_pt_inRange<PointXYZIT>(*all_lidar_data, one_submap.submap_start_time, one_submap.submap_end_time,
                                    point_index, one_submap.raw_pointcloud);

        /*
        cout << setprecision(13) << "submap start_time: " << one_submap.submap_start_time << " submap end_time: "
             << one_submap.submap_end_time;
        cout << setprecision(13) << " imu start_time: " << one_submap.imu_data.front().timestamp_s << " end_time: "
             << one_submap.imu_data.back().timestamp_s << endl;
        cout << setprecision(13) << " point start_time: " << one_submap.raw_pointcloud.front().relative_time_ms << " point end_time: "
             << one_submap.raw_pointcloud.back().relative_time_ms << endl;
        cout << setprecision(13) << " odom poses start_time: " << one_submap.odom_poses.front().timestamp << " end_time: "
             << one_submap.odom_poses.back().timestamp << endl;
        cout << "one submap raw pointcloud size: " << one_submap.raw_pointcloud.size() << endl;
        */

        vec_submap.push_back(one_submap);
    }
    all_lidar_data->clear();

    //4.Output initial map
    for (int i = 0; i < vec_submap.size(); ++i) {
        imu_data_start_index = 0;
        imu_data_end_index = 0;
        point_index = 0;
        Submap &one_submap = vec_submap[i];
        for (int j = 0; j < one_submap.odom_poses.size() - 1; ++j) {

            //get the raw scan between two discrete pose
            ITPointCloud raw_scan;
            if (IfSimulation == 0)
                find_pt_inRange<PointXYZIT>(one_submap.raw_pointcloud, one_submap.odom_poses[j].timestamp,
                                        one_submap.odom_poses[j + 1].timestamp, point_index, raw_scan);
            if (IfSimulation == 1)
                //The timestamp of a frame of simulation data is consistent
                find_pt_inDiscreteT<PointXYZIT>(one_submap.raw_pointcloud, one_submap.odom_poses[j].timestamp,
                                                point_index, raw_scan);

            //cout << "raw scan size: " << raw_scan.size();

            if (raw_scan.size() == 0)
                continue;

            //get the imu data between two discrete pose
            vector<ImuData> one_scan_imu;
            find_imu_data_in_one_scan(one_submap.odom_poses[j].timestamp, one_submap.odom_poses[j + 1].timestamp,
                                      one_submap.imu_data, imu_data_start_index, imu_data_end_index,
                                      one_scan_imu);

            //check
            /*
            cout << setprecision(13) << "pose start time: " << one_submap.odom_poses[j].timestamp << " "
                 << "pose end time: " << one_submap.odom_poses[j+1].timestamp;
            cout << setprecision(13) << " point start time: " << raw_scan.front().relative_time_ms << " "
                 << "point end time: " << raw_scan.back().relative_time_ms;
            cout << setprecision(13) << " imu start_time: " << one_scan_imu.front().timestamp_s << " end_time: "
                 << one_scan_imu.back().timestamp_s << endl;
            */

            PointCloudPtr undistorted_scan(new PointCloud());
            ITPointCloudPtr undistorted_ITscan(new ITPointCloud());
            if (IfSimulation == 1)
            {
                PointCloudPtr tmp(new PointCloud());
                for (int k = 0; k < raw_scan.size(); ++k) {
                    PointXYZ pt;
                    pt.x = raw_scan[k].x;
                    pt.y = raw_scan[k].y;
                    pt.z = raw_scan[k].z;
                    tmp->points.push_back(pt);
                }
                transformPointcloud(tmp, li_extrinsic, undistorted_scan);

                //transform to submap_start time and add to submap pointcloud
                PointCloudPtr scan_transformed(new PointCloud());
                transformPointcloud(undistorted_scan, one_submap.submap_pose.inverse() * one_submap.odom_poses[j].pose,
                                    scan_transformed);
                //filtering
                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
                sor.setInputCloud(scan_transformed);
                sor.setMeanK(6);
                sor.setStddevMulThresh(1.0);
                sor.filter(*scan_transformed);

                one_submap.pointcloud_dt += *scan_transformed;
            }else{
                vector<PoseData> imu_rotation;
                rotation_integration(one_scan_imu, imu_dt, imu_rotation);

                undistortRawScan1(raw_scan, li_extrinsic, one_submap.odom_poses[j], one_submap.odom_poses[j + 1],
                                  imu_rotation, undistorted_scan);
                undistortRawScan1(raw_scan, li_extrinsic, one_submap.odom_poses[j], one_submap.odom_poses[j + 1],
                                  imu_rotation, undistorted_ITscan);
                
                //cout << " scan size: " << undistorted_scan->size() << endl;
                //transform to submap_start time and add to submap pointcloud
                //Todo Merge static submaps
                PointCloudPtr scan_transformed(new PointCloud());
                ITPointCloudPtr ITscan_transformed(new ITPointCloud());
                transformPointcloud(undistorted_scan, one_submap.submap_pose.inverse() * one_submap.odom_poses[j].pose,
                                    scan_transformed);
                transformPointcloud(undistorted_ITscan, one_submap.submap_pose.inverse() * one_submap.odom_poses[j].pose,
                                    ITscan_transformed);
                //voxelGridFilter<PointXYZ>(scan_transformed, 0.1);
                one_submap.pointcloud_dt += *scan_transformed;
                one_submap.ITpointcloud_dt += *ITscan_transformed;
                
            }
        }
        //output the odom submap
        /*
        PointCloudPtr submap_transformed(new PointCloud());
        PointCloudPtr submap_raw(new PointCloud());
        *submap_raw = one_submap.pointcloud_dt;
        //cout << submap_raw->size() << endl;
        if (submap_raw->size() == 0)
            continue;
        transformPointcloud(submap_raw, one_submap.submap_pose, submap_transformed);
        stringstream submap_path;
        submap_path << result_dir_path.str() << "/submap_" << i << ".pcd";
        pcl::io::savePCDFileBinaryCompressed(submap_path.str(), *submap_transformed);
        */
    }

    // save las format
    /*
    PointTable table;
    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);
    table.layout()->registerDim(Dimension::Id::GpsTime);
    table.layout()->registerDim(Dimension::Id::Intensity);

    PointViewPtr view(new PointView(table));

    int las_point_index = 0;
    for (size_t i = 0; i < vec_submap.size(); i++)
    {
        ITPointCloudPtr submap_transformed(new ITPointCloud());
        ITPointCloudPtr submap_raw(new ITPointCloud());
        *submap_raw = vec_submap[i].ITpointcloud_dt;
        if (submap_raw->size() == 0)
            continue;
        transformPointcloud(submap_raw, vec_submap[i].submap_pose, submap_transformed);
        for (int j = 0; j < submap_transformed->points.size(); ++j)
        {
            view->setField(Dimension::Id::X, las_point_index + j, submap_transformed->points[j].x);
            view->setField(Dimension::Id::Y, las_point_index + j, submap_transformed->points[j].y);
            view->setField(Dimension::Id::Z, las_point_index + j, submap_transformed->points[j].z);
            view->setField(Dimension::Id::GpsTime, las_point_index + j, submap_transformed->points[j].relative_time_ms);
            view->setField(Dimension::Id::Intensity, las_point_index + j, submap_transformed->points[j].reflectivity);
        }
        las_point_index += vec_submap[i].ITpointcloud_dt.size();
    }

    BufferReader reader;
    reader.addView(view);

    LasWriter writer;
    writer.setInput(reader);
    stringstream las_path;
    las_path << result_dir_path.str() << "/odom_map.las";
    writer.setOptions(Option("filename", las_path.str()));

    try
    {
        writer.prepare(table);
        writer.execute(table);
        std::cout << "LAS file created successfully" << std::endl;
    }
    catch (const pdal::pdal_error &e)
    {
        std::cerr << "PDAL Error: " << e.what() << std::endl;
        return -1;
    }*/

    double end_time = omp_get_wtime();
    double submap_reconstruction_time = end_time - start_time;

    //5. Find non-consecutive submap constraint
    vector<pair<int, int>> nonConsecutivePair_index;
    vector<pair<int, int>> refine_nonConsecutivePair_index;
    vector<gtsam::Pose3> nonConsecutive_relativePose;
    start_time = omp_get_wtime();
    int submap_size_overlap;
    if (EnableOverlapConstraint)
        submap_size_overlap = vec_submap.size();
    else
        submap_size_overlap = 0;
    for (int j = 0; j < submap_size_overlap; ++j) {
        Submap &source_submap = vec_submap[j];
        for (int k = j + 2; k < j + NoConsecutiveSubmapSearch; ++k) {
            if (k > vec_submap.size() - 1)
                break;
            Submap &target_submap = vec_submap[k];
            PointCloudPtr submap_raw(new PointCloud());
            PointCloudPtr submap1_raw(new PointCloud());
            PointCloudPtr submap_transformed(new PointCloud());
            PointCloudPtr submap1_transformed(new PointCloud());
            *submap_raw = source_submap.pointcloud_dt;
            *submap1_raw = target_submap.pointcloud_dt;
            transformPointcloud(submap_raw, source_submap.submap_pose, submap_transformed);
            transformPointcloud(submap1_raw, target_submap.submap_pose, submap1_transformed);
            double overlap_ratio = overlap_calculate(submap_transformed, submap1_transformed);
            if (overlap_ratio > overlap_threshold) {
                cout << "overlap ratio between " << j << "&" << k << ": " << overlap_ratio << endl;
                nonConsecutivePair_index.push_back(pair<int, int>(j, k));
            }
        }
    }
    end_time = omp_get_wtime();
    double submap_retrival_time = end_time - start_time;
    start_time = omp_get_wtime();
    for (int j = 0; j < nonConsecutivePair_index.size(); ++j) {
        int source_index = nonConsecutivePair_index[j].first;
        int target_index = nonConsecutivePair_index[j].second;
        PointCloudPtr source_submap(new PointCloud());
        PointCloudPtr target_submap(new PointCloud());
        *source_submap = vec_submap[source_index].pointcloud_dt;
        *target_submap = vec_submap[target_index].pointcloud_dt;
        //do the registration
        //Downsample

        voxelGridFilter<PointXYZ>(source_submap, 0.1);
        voxelGridFilter<PointXYZ>(target_submap, 0.2);

        pcl::KdTreeFLANN<PointXYZ>::Ptr kd_tree(new pcl::KdTreeFLANN<PointXYZ>());
        kd_tree->setInputCloud(target_submap);
        gtsam::Pose3 relative_pose = vec_submap[target_index].submap_pose.inverse() *
                                     vec_submap[source_index].submap_pose;

        if (submap_regis == 0) {
            if (EnableNonPlanarFilter)
            {
                //non-planar point filtering
                ///PCA calculation
                std::vector<int> point_search_ind;
                std::vector<float> point_search_sqdis;
                pcl::KdTreeFLANN<PointXYZ>::Ptr kd_tree_source(new pcl::KdTreeFLANN<PointXYZ>());
                kd_tree_source->setInputCloud(source_submap);
                int neighbor_number = 20;
                PointCloudPtr source_submap_filtered(new PointCloud());
                for (int j = 0; j < source_submap->size(); ++j) {
                    kd_tree_source->nearestKSearch(source_submap->points[j], neighbor_number, point_search_ind,
                                                   point_search_sqdis);

                    Eigen::Vector3d point_avg(0, 0, 0);
                    for (int k = 0; k < neighbor_number; ++k) {
                        PointXYZ pt = source_submap->points[point_search_ind[k]];
                        Eigen::Vector3d pt1(pt.x, pt.y, pt.z);
                        point_avg += pt1;
                    }
                    point_avg /= double(neighbor_number);

                    Eigen::Matrix3d pt_covar = Eigen::Matrix3d::Zero();
                    for (int k = 0; k < neighbor_number; ++k) {
                        PointXYZ pt = source_submap->points[point_search_ind[k]];
                        Eigen::Vector3d pt1(pt.x, pt.y, pt.z);
                        pt_covar += (pt1 - point_avg) * (pt1 - point_avg).transpose();
                    }
                    pt_covar /= neighbor_number;

                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pt_covar);

                    /// note Eigen library sort eigenvalues in increasing order
                    double eigen_ratio = saes.eigenvalues()[0] / saes.eigenvalues()[2];
                    if (eigen_ratio < 0.05)
                        source_submap_filtered->push_back(source_submap->points[j]);
                }
                cout << "source map point size: " << source_submap->points.size();
                cout << "  source map filtered point size: " << source_submap_filtered->points.size() << endl;
                scanToMapRegi(source_submap_filtered, target_submap, kd_tree, relative_pose,
                              15, 1, 0.05, 0.1);
            }
            else
                scanToMapRegi(source_submap, target_submap, kd_tree, relative_pose,
                              15, 1, 0.05, 0.1);
        }

        if (submap_regis == 2) {
            fast_gicp::FastGICP <pcl::PointXYZ, pcl::PointXYZ> fgicp_mt;
            PointCloudPtr aligned(new PointCloud());
            Eigen::Matrix4f initial_value = Eigen::Matrix4f::Identity();
            initial_value.block<3, 3>(0, 0) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose).rotation().matrix().cast<float>();
            initial_value.block<3, 1>(0, 3) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose).translation().cast<float>();
            fgicp_mt.setInputTarget(target_submap);
            fgicp_mt.setInputSource(source_submap);
            fgicp_mt.setCorrespondenceRandomness(correspond_num);
            fgicp_mt.align(*aligned, initial_value);
            Eigen::Matrix4d refine_value = fgicp_mt.getFinalTransformation().cast<double>();
            relative_pose = gtsam::Pose3(gtsam::Rot3(refine_value.block<3, 3>(0, 0)),
                                         gtsam::Point3(refine_value.block<3, 1>(0, 3)));
        }

        if (submap_regis == 3)
        {
            fast_gicp::FastVGICP <pcl::PointXYZ, pcl::PointXYZ> vgicp_mt;
            PointCloudPtr aligned(new PointCloud());
            Eigen::Matrix4f initial_value = Eigen::Matrix4f::Identity();
            initial_value.block<3, 3>(0, 0) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose).rotation().matrix().cast<float>();
            initial_value.block<3, 1>(0, 3) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose).translation().cast<float>();
            vgicp_mt.setInputTarget(target_submap);
            vgicp_mt.setInputSource(source_submap);
            vgicp_mt.setResolution(vgicp_voxel);
            vgicp_mt.setNumThreads(4);
            vgicp_mt.align(*aligned, initial_value);
            Eigen::Matrix4d refine_value = vgicp_mt.getFinalTransformation().cast<double>();
            relative_pose = gtsam::Pose3(gtsam::Rot3(refine_value.block<3, 3>(0, 0)),
                                         gtsam::Point3(refine_value.block<3, 1>(0, 3)));
        }

        if (submap_regis == 1) {

            pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> pcl_ndt;
            pcl_ndt.setResolution(ndt_voxel);
            pcl_ndt.setOulierRatio(ndt_outlier_ratio);
            PointCloudPtr aligned(new PointCloud());
            Eigen::Matrix4f initial_value = Eigen::Matrix4f::Identity();
            initial_value.block<3, 3>(0, 0) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose).rotation().matrix().cast<float>();
            initial_value.block<3, 1>(0, 3) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose).translation().cast<float>();
            pcl_ndt.setInputTarget(target_submap);
            pcl_ndt.setInputSource(source_submap);
            pcl_ndt.align(*aligned, initial_value);
            Eigen::Matrix4d refine_value = pcl_ndt.getFinalTransformation().cast<double>();
            relative_pose = gtsam::Pose3(gtsam::Rot3(refine_value.block<3, 3>(0, 0)),
                                         gtsam::Point3(refine_value.block<3, 1>(0, 3)));
        }

        /*
        stringstream submap_path;
        submap_path << result_dir_path.str() << "/source_submap_" << source_index << "&" << target_index << ".pcd";
        pcl::io::savePCDFileBinaryCompressed(submap_path.str(), src_cloud_transformed);
        stringstream submap_path1;
        submap_path1 << result_dir_path.str() << "/target_submap_" << source_index << "&" << target_index << ".pcd";
        pcl::io::savePCDFileBinaryCompressed(submap_path1.str(), *target_submap);
        */
        cout << "refine nonConsecutive pair: " << source_index << " " << target_index << endl;
        refine_nonConsecutivePair_index.push_back(pair<int, int>(source_index, target_index));
        nonConsecutive_relativePose.push_back(relative_pose.inverse());
    }
    end_time = omp_get_wtime();
    double relative_constraint_time = end_time - start_time;

    //6.Find submap loop constraint
    vector<pair<int, int>> init_loopPair_index;
    start_time = omp_get_wtime();
    int submap_size_loop;
    if (EnableLoopConstraint)
        submap_size_loop = vec_submap.size();
    else
        submap_size_loop = 0;
    for (int i = 0; i < submap_size_loop; ++i) {
        Submap &source_submap = vec_submap[i];
        bool first_loop = true;
        gtsam::Point3 last_tran;
        for (int j = i + 1; j < vec_submap.size() - 1; ++j) {
            Submap &target_submap = vec_submap[j];
            if (gtsam::distance3(source_submap.submap_pose.translation(),
                                 target_submap.submap_pose.translation()) < 3) {
                if (abs(i - j) > NoConsecutiveSubmapSearch) {
                    if (first_loop) {
                        init_loopPair_index.push_back(pair<int, int>(i, j));
                        cout << "loop pair: " << i << " " << j << endl;
                        last_tran = target_submap.submap_pose.translation();
                        first_loop = false;
                    } else {
                        //Filter redundant loop constraint when still
                        if (gtsam::distance3(last_tran, target_submap.submap_pose.translation()) > 1) {
                            init_loopPair_index.push_back(pair<int, int>(i, j));
                            cout << "loop pair: " << i << " " << j << endl;
                            last_tran = target_submap.submap_pose.translation();
                        }
                    }
                }
            }
        }
    }
    end_time = omp_get_wtime();
    submap_retrival_time += end_time - start_time;

    //Registration get relative constraint
    start_time = omp_get_wtime();
    vector<pair<int, int>> refine_loopPair_index;
    vector<gtsam::Pose3> loop_relativePose;
    for (int j = 0; j < init_loopPair_index.size(); ++j) {
        double overlap_ratio = -1;
        int source_index = init_loopPair_index[j].first;
        int target_index = init_loopPair_index[j].second;
        PointCloudPtr submap_raw(new PointCloud());

        // do the registration
        /// initial value by submap group to submap group registration
        PointCloudPtr source_submap_group(new PointCloud());
        int submap_neighbor_num = 3;
        for (int k = source_index - submap_neighbor_num; k < source_index + submap_neighbor_num; ++k)
        {
            if (k >= 0 && k < vec_submap.size())
            {
                *submap_raw = vec_submap[k].pointcloud_dt;
                PointCloudPtr submap_inSource(new PointCloud());
                transformPointcloud(submap_raw,
                                    vec_submap[source_index].submap_pose.inverse() * vec_submap[k].submap_pose,
                                    submap_inSource);
                *source_submap_group += *submap_inSource;
            }
        }
        PointCloudPtr target_submap_group(new PointCloud());
        for (int k = target_index - submap_neighbor_num; k < target_index + submap_neighbor_num; ++k)
        {
            if (k >= 0 && k < vec_submap.size())
            {
                PointCloudPtr submap_inTarget(new PointCloud());
                *submap_raw = vec_submap[k].pointcloud_dt;
                transformPointcloud(submap_raw,
                                    vec_submap[target_index].submap_pose.inverse() * vec_submap[k].submap_pose,
                                    submap_inTarget);
                *target_submap_group += *submap_inTarget;
            }
        }
        //overlap calculate
        cout << "submap group overlap: ";
        PointCloudPtr tmp1(new PointCloud());
        transformPointcloud(source_submap_group, vec_submap[target_index].submap_pose.inverse() * vec_submap[source_index].submap_pose, tmp1);
        overlap_ratio = overlap_calculate(tmp1, target_submap_group);
        cout << overlap_ratio << endl;
        if (overlap_ratio < 0.1)
            continue;

        /*
        stringstream submap_path;
        submap_path << result_dir_path.str() << "/submap_" << source_index << ".pcd";
        pcl::io::savePCDFileBinaryCompressed(submap_path.str(), *source_submap_group);
        stringstream submap_path1;
        submap_path1 << result_dir_path.str() << "/submap_" << target_index << ".pcd";
        pcl::io::savePCDFileBinaryCompressed(submap_path1.str(), *target_submap_group);
         */

        // registration
        // Downsample
        voxelGridFilter<PointXYZ>(source_submap_group, 0.1);
        voxelGridFilter<PointXYZ>(target_submap_group, 0.2);

        pcl::KdTreeFLANN<PointXYZ>::Ptr kd_tree(new pcl::KdTreeFLANN<PointXYZ>());
        kd_tree->setInputCloud(target_submap_group);
        gtsam::Pose3 relative_pose = vec_submap[target_index].submap_pose.inverse() *
                                     vec_submap[source_index].submap_pose;

        if (submap_regis == 0)
        {
            if (EnableNonPlanarFilter)
            {
                /// PCA calculation
                std::vector<int> point_search_ind;
                std::vector<float> point_search_sqdis;
                pcl::KdTreeFLANN<PointXYZ>::Ptr kd_tree_source(new pcl::KdTreeFLANN<PointXYZ>());
                kd_tree_source->setInputCloud(source_submap_group);
                int neighbor_number = 20;
                PointCloudPtr source_submap_filtered(new PointCloud());
                for (int j = 0; j < source_submap_group->size(); ++j)
                {
                    kd_tree_source->nearestKSearch(source_submap_group->points[j], neighbor_number, point_search_ind,
                                                   point_search_sqdis);

                    Eigen::Vector3d point_avg(0, 0, 0);
                    for (int k = 0; k < neighbor_number; ++k)
                    {
                        PointXYZ pt = source_submap_group->points[point_search_ind[k]];
                        Eigen::Vector3d pt1(pt.x, pt.y, pt.z);
                        point_avg += pt1;
                    }
                    point_avg /= double(neighbor_number);

                    Eigen::Matrix3d pt_covar = Eigen::Matrix3d::Zero();
                    for (int k = 0; k < neighbor_number; ++k)
                    {
                        PointXYZ pt = source_submap_group->points[point_search_ind[k]];
                        Eigen::Vector3d pt1(pt.x, pt.y, pt.z);
                        pt_covar += (pt1 - point_avg) * (pt1 - point_avg).transpose();
                    }
                    pt_covar /= neighbor_number;

                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pt_covar);

                    /// note Eigen library sort eigenvalues in increasing order
                    double eigen_ratio = saes.eigenvalues()[0] / saes.eigenvalues()[2];
                    if (eigen_ratio < 0.05)
                        source_submap_filtered->push_back(source_submap_group->points[j]);
                }
                cout << "source map point size: " << source_submap_group->points.size();
                cout << "  source map filtered point size: " << source_submap_filtered->points.size() << endl;

                scanToMapRegi(source_submap_filtered, target_submap_group, kd_tree, relative_pose,
                              15, 3, 0.1, 0.1);
            }
            else
                scanToMapRegi(source_submap_group, target_submap_group, kd_tree, relative_pose,
                              15, 3, 0.1, 0.1);
        }

        if (submap_regis == 2)
        {
            fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> fgicp_mt;
            PointCloudPtr aligned(new PointCloud());
            Eigen::Matrix4f initial_value = Eigen::Matrix4f::Identity();
            initial_value.block<3, 3>(0, 0) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose)
                                                  .rotation()
                                                  .matrix()
                                                  .cast<float>();
            initial_value.block<3, 1>(0, 3) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose)
                                                  .translation()
                                                  .cast<float>();
            fgicp_mt.setInputTarget(target_submap_group);
            fgicp_mt.setInputSource(source_submap_group);
            fgicp_mt.setCorrespondenceRandomness(correspond_num);
            fgicp_mt.align(*aligned, initial_value);
            Eigen::Matrix4d refine_value = fgicp_mt.getFinalTransformation().cast<double>();
            relative_pose = gtsam::Pose3(gtsam::Rot3(refine_value.block<3, 3>(0, 0)),
                                         gtsam::Point3(refine_value.block<3, 1>(0, 3)));
        }

        if (submap_regis == 3)
        {
            fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> vgicp_mt;
            PointCloudPtr aligned(new PointCloud());
            Eigen::Matrix4f initial_value = Eigen::Matrix4f::Identity();
            initial_value.block<3, 3>(0, 0) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose)
                                                  .rotation()
                                                  .matrix()
                                                  .cast<float>();
            initial_value.block<3, 1>(0, 3) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose)
                                                  .translation()
                                                  .cast<float>();
            vgicp_mt.setInputTarget(target_submap_group);
            vgicp_mt.setInputSource(source_submap_group);
            vgicp_mt.setResolution(vgicp_voxel);
            vgicp_mt.setNumThreads(4);
            vgicp_mt.align(*aligned, initial_value);
            Eigen::Matrix4d refine_value = vgicp_mt.getFinalTransformation().cast<double>();
            relative_pose = gtsam::Pose3(gtsam::Rot3(refine_value.block<3, 3>(0, 0)),
                                         gtsam::Point3(refine_value.block<3, 1>(0, 3)));
        }

        if (submap_regis == 1)
        {
            pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> pcl_ndt;
            pcl_ndt.setResolution(ndt_voxel);
            pcl_ndt.setOulierRatio(ndt_outlier_ratio);
            PointCloudPtr aligned(new PointCloud());
            Eigen::Matrix4f initial_value = Eigen::Matrix4f::Identity();
            initial_value.block<3, 3>(0, 0) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose)
                                                  .rotation()
                                                  .matrix()
                                                  .cast<float>();
            initial_value.block<3, 1>(0, 3) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose)
                                                  .translation()
                                                  .cast<float>();
            pcl_ndt.setInputTarget(target_submap_group);
            pcl_ndt.setInputSource(source_submap_group);
            pcl_ndt.align(*aligned, initial_value);
            Eigen::Matrix4d refine_value = pcl_ndt.getFinalTransformation().cast<double>();
            relative_pose = gtsam::Pose3(gtsam::Rot3(refine_value.block<3, 3>(0, 0)),
                                         gtsam::Point3(refine_value.block<3, 1>(0, 3)));
        }

        stringstream submap_path;
        PointCloudPtr tmp(new PointCloud());
        transformPointcloud(source_submap_group, relative_pose, tmp);
        submap_path << result_dir_path.str() << "/source_submap_" << source_index << "&" << target_index << ".pcd";
        pcl::io::savePCDFileBinaryCompressed(submap_path.str(), *tmp);
        stringstream submap_path1;
        submap_path1 << result_dir_path.str() << "/target_submap_" << source_index << "&" << target_index << ".pcd";
        pcl::io::savePCDFileBinaryCompressed(submap_path1.str(), *target_submap_group);

        /*
        stringstream submap_path;
        submap_path << result_dir_path.str() << "/submap_" << source_index << ".pcd";
        pcl::io::savePCDFileBinaryCompressed(submap_path.str(), src_cloud_transformed);
        stringstream submap_path1;
        submap_path1 << result_dir_path.str() << "/submap_" << target_index << ".pcd";
        pcl::io::savePCDFileBinaryCompressed(submap_path1.str(), *target_submap_group);
         */

        refine_loopPair_index.push_back(pair<int, int>(source_index, target_index));
        cout << "refine loop pair: " << source_index << " " << target_index << endl;
        loop_relativePose.push_back(relative_pose.inverse());
    }
    end_time = omp_get_wtime();
    relative_constraint_time += end_time - start_time;

    //refine adjacent submap constraint using odometry constraints
    vector<gtsam::Pose3> adjacent_relativePose;
    int adjacent_submap_count = 0;
    for (int j = 0; j < vec_submap.size() - 1; ++j) {
    //for (int j = 0; j < 0; ++j) {
        start_time = omp_get_wtime();
        int source_index = j;
        int target_index = j + 1;
        PointCloudPtr source_submap(new PointCloud());
        PointCloudPtr target_submap(new PointCloud());
        *source_submap = vec_submap[source_index].pointcloud_dt;
        *target_submap = vec_submap[target_index].pointcloud_dt;

        PointCloudPtr source_submap_transformed(new PointCloud());
        PointCloudPtr target_submap_transformed(new PointCloud());
        transformPointcloud(source_submap, vec_submap[source_index].submap_pose, source_submap_transformed);
        transformPointcloud(target_submap, vec_submap[target_index].submap_pose, target_submap_transformed);
        double overlap_ratio = overlap_calculate(source_submap_transformed, target_submap_transformed);
        cout << "overlap ratio between " << source_index << "&" << target_index << ": " << overlap_ratio << endl;
        end_time = omp_get_wtime();
        submap_retrival_time += start_time - end_time;

        if (overlap_ratio < overlap_threshold) {
            adjacent_relativePose.push_back(vec_submap[source_index].submap_pose.inverse() *
                                            vec_submap[target_index].submap_pose);
            continue;
        }

        //do the registration
        //point-to-plane ICP
        //Downsample
        adjacent_submap_count++;
        start_time = omp_get_wtime();

        voxelGridFilter<PointXYZ>(source_submap, 0.1);
        voxelGridFilter<PointXYZ>(target_submap, 0.2);

        pcl::KdTreeFLANN<PointXYZ>::Ptr kd_tree(new pcl::KdTreeFLANN<PointXYZ>());
        kd_tree->setInputCloud(target_submap);
        gtsam::Pose3 relative_pose = vec_submap[target_index].submap_pose.inverse() *
                                     vec_submap[source_index].submap_pose;

        if (submap_regis == 0)
        {
            if (EnableNonPlanarFilter)
            {
                ///PCA calculation
                std::vector<int> point_search_ind;
                std::vector<float> point_search_sqdis;
                pcl::KdTreeFLANN<PointXYZ>::Ptr kd_tree_source(new pcl::KdTreeFLANN<PointXYZ>());
                kd_tree_source->setInputCloud(source_submap);
                int neighbor_number = 20;
                PointCloudPtr source_submap_filtered(new PointCloud());
                for (int j = 0; j < source_submap->size(); ++j) {
                    kd_tree_source->nearestKSearch(source_submap->points[j], neighbor_number, point_search_ind,
                                                   point_search_sqdis);

                    Eigen::Vector3d point_avg(0, 0, 0);
                    for (int k = 0; k < neighbor_number; ++k) {
                        PointXYZ pt = source_submap->points[point_search_ind[k]];
                        Eigen::Vector3d pt1(pt.x, pt.y, pt.z);
                        point_avg += pt1;
                    }
                    point_avg /= double(neighbor_number);

                    Eigen::Matrix3d pt_covar = Eigen::Matrix3d::Zero();
                    for (int k = 0; k < neighbor_number; ++k) {
                        PointXYZ pt = source_submap->points[point_search_ind[k]];
                        Eigen::Vector3d pt1(pt.x, pt.y, pt.z);
                        pt_covar += (pt1 - point_avg) * (pt1 - point_avg).transpose();
                    }
                    pt_covar /= neighbor_number;

                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pt_covar);

                    /// note Eigen library sort eigenvalues in increasing order
                    double eigen_ratio = saes.eigenvalues()[0] / saes.eigenvalues()[2];
                    if (eigen_ratio < 0.05)
                        source_submap_filtered->push_back(source_submap->points[j]);
                }
                cout << "source map point size: " << source_submap->points.size();
                cout << "  source map filtered point size: " << source_submap_filtered->points.size() << endl;

                scanToMapRegi(source_submap_filtered, target_submap, kd_tree, relative_pose,
                              15, 1, 0.05, 0.1);

                //        stringstream submap_path;
                //        submap_path << result_dir_path.str() << "/submap_" << source_index << ".pcd";
                //        pcl::io::savePCDFileBinaryCompressed(submap_path.str(), *source_submap);
                //        stringstream submap_filtered_path;
                //        submap_filtered_path << result_dir_path.str() << "/submap_filtered_" << source_index << ".pcd";
                //        pcl::io::savePCDFileBinaryCompressed(submap_filtered_path.str(), *source_submap_filtered);
            }
            else
                scanToMapRegi(source_submap, target_submap, kd_tree, relative_pose,
                         15, 1, 0.05, 0.1);
        }

        if (submap_regis == 2) {
            fast_gicp::FastGICP <pcl::PointXYZ, pcl::PointXYZ> fgicp_mt;
            PointCloudPtr aligned(new PointCloud());
            Eigen::Matrix4f initial_value = Eigen::Matrix4f::Identity();
            initial_value.block<3, 3>(0, 0) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose).rotation().matrix().cast<float>();
            initial_value.block<3, 1>(0, 3) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose).translation().cast<float>();
            fgicp_mt.setInputTarget(target_submap);
            fgicp_mt.setInputSource(source_submap);
            fgicp_mt.setCorrespondenceRandomness(correspond_num);
            fgicp_mt.align(*aligned, initial_value);
            Eigen::Matrix4d refine_value = fgicp_mt.getFinalTransformation().cast<double>();
            relative_pose = gtsam::Pose3(gtsam::Rot3(refine_value.block<3, 3>(0, 0)),
                                                      gtsam::Point3(refine_value.block<3, 1>(0, 3)));
        }

        if (submap_regis == 3)
        {
            fast_gicp::FastVGICP <pcl::PointXYZ, pcl::PointXYZ> vgicp_mt;
            PointCloudPtr aligned(new PointCloud());
            Eigen::Matrix4f initial_value = Eigen::Matrix4f::Identity();
            initial_value.block<3, 3>(0, 0) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose).rotation().matrix().cast<float>();
            initial_value.block<3, 1>(0, 3) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose).translation().cast<float>();
            vgicp_mt.setInputTarget(target_submap);
            vgicp_mt.setInputSource(source_submap);
            vgicp_mt.setResolution(vgicp_voxel);
            vgicp_mt.setNumThreads(4);
            vgicp_mt.align(*aligned, initial_value);
            Eigen::Matrix4d refine_value = vgicp_mt.getFinalTransformation().cast<double>();
            relative_pose = gtsam::Pose3(gtsam::Rot3(refine_value.block<3, 3>(0, 0)),
                                         gtsam::Point3(refine_value.block<3, 1>(0, 3)));
        }

        if (submap_regis == 1) {

            pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> pcl_ndt;
            pcl_ndt.setResolution(ndt_voxel);
            pcl_ndt.setOulierRatio(ndt_outlier_ratio);
            PointCloudPtr aligned(new PointCloud());
            Eigen::Matrix4f initial_value = Eigen::Matrix4f::Identity();
            initial_value.block<3, 3>(0, 0) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose).rotation().matrix().cast<float>();
            initial_value.block<3, 1>(0, 3) = (vec_submap[target_index].submap_pose.inverse() *
                                               vec_submap[source_index].submap_pose).translation().cast<float>();
            pcl_ndt.setInputTarget(target_submap);
            pcl_ndt.setInputSource(source_submap);
            pcl_ndt.align(*aligned, initial_value);
            Eigen::Matrix4d refine_value = pcl_ndt.getFinalTransformation().cast<double>();
            relative_pose = gtsam::Pose3(gtsam::Rot3(refine_value.block<3, 3>(0, 0)),
                                                      gtsam::Point3(refine_value.block<3, 1>(0, 3)));
        }

        adjacent_relativePose.push_back(relative_pose.inverse());
        cout << "adjacent pose:" << j << "&" << j + 1 << "refine" << endl;
        end_time = omp_get_wtime();
        relative_constraint_time += end_time - start_time;
    }

    //7.Pose graph optimization
    //pose graph setup
    start_time = omp_get_wtime();
    gtsam::NonlinearFactorGraph pose_graph;
    gtsam::Values initial_values;
    auto prior_noise_model = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
    pose_graph.addPrior(0, vec_submap[0].submap_pose, prior_noise_model);

    for (int j = 0; j < vec_submap.size(); ++j) {
        initial_values.insert(j, vec_submap[j].submap_pose);
    }

    //add adjacent submap constraints
    for (int j = 0; j < vec_submap.size() - 1; ++j) {
        auto odometry_noise_model = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());

        pose_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(j, j + 1,
                                                                      adjacent_relativePose[j],
                                                                      odometry_noise_model);
    }
    //add loop submap constraints
    cout << "refine_loopPair size: " << refine_loopPair_index.size() << endl;
    for (int j = 0; j < refine_loopPair_index.size(); ++j) {
 
        auto robustLoopNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(
                5),
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7).finished()));

        //auto loop_noise_model = gtsam::noiseModel::Diagonal::Variances(
        //        (gtsam::Vector(6) << 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7).finished());


        pose_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(refine_loopPair_index[j].first,
                                                                      refine_loopPair_index[j].second,
                                                                      loop_relativePose[j],
                                                                      robustLoopNoise);
    }
    // add non-consecutive constraints
    cout << "refine_nonConsecutivePair size: " << refine_nonConsecutivePair_index.size() << endl;
    for (int j = 0; j < refine_nonConsecutivePair_index.size(); ++j) {
        
        auto noise_model = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 2e-8, 2e-8, 2e-8, 2e-8, 2e-8, 2e-8).finished());


        pose_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(refine_nonConsecutivePair_index[j].first,
                                                                      refine_nonConsecutivePair_index[j].second,
                                                                      nonConsecutive_relativePose[j],
                                                                      noise_model);
    }

    cout << "adjacent_submap_count: " << adjacent_submap_count << endl;
    //optimization
    cout << "optimizing ..." << endl;
    gtsam::GaussNewtonParams params;
    params.setVerbosity("TERMINATION");//  show info about stopping conditions
    gtsam::GaussNewtonOptimizer optimizer(pose_graph, initial_values, params);
    gtsam::Values result = optimizer.optimize();
    cout << "result size: " << result.size() << std::endl;
    cout << "Optimization complete" << std::endl;
    end_time = omp_get_wtime();
    double optimization_time = end_time - start_time;

    //output time
    cout << "load data time: " << load_data_time << "s" << endl;
    cout << "submap_reconstruction_time: " << submap_reconstruction_time << "s" << endl;
    cout << "submap_retrival_time: " << submap_retrival_time << "s" << endl;
    cout << "relative_constraint_time: " << relative_constraint_time << "s" << endl;
    cout << "optimization_time: " << optimization_time << "s" << endl;
    cout << "whole time: " << omp_get_wtime() - whole_start_time << "s" << endl;

    //update submap poses, output map
    PointCloudPtr map(new PointCloud());
    for (int j = 0; j < vec_submap.size(); ++j) {
        vec_submap[j].submap_pose = result.at<gtsam::Pose3>(j);
        //output the submap
        PointCloudPtr submap_transformed(new PointCloud());
        PointCloudPtr submap_raw(new PointCloud());
        *submap_raw = vec_submap[j].pointcloud_dt;
        transformPointcloud(submap_raw, vec_submap[j].submap_pose, submap_transformed);
        stringstream submap_path;
        submap_path << result_dir_path.str() << "/submap_" << j << ".pcd";
        //pcl::io::savePCDFileBinaryCompressed(submap_path.str(), *submap_transformed);
        *map += *submap_transformed;
    }
    stringstream map_path;
    map_path << result_dir_path.str() << "/slam_map.pcd";
    pcl::io::savePCDFileBinaryCompressed(map_path.str(), *map);

    //update frame poses, output poses
    vector<PoseData> refine_poses;
    for (int j = 0; j < vec_submap.size(); ++j) {
        gtsam::Pose3 update = vec_submap[j].submap_pose * vec_submap[j].odom_poses.front().pose.inverse();
        for (int k = 0; k < vec_submap[j].odom_poses.size(); ++k) {
            PoseData refine_pose;
            refine_pose.timestamp = vec_submap[j].odom_poses[k].timestamp;
            refine_pose.pose = update * vec_submap[j].odom_poses[k].pose;
            refine_poses.push_back(refine_pose);
        }
    }
    stringstream refine_traj_path;
    refine_traj_path << result_dir_path.str() << "/traj.txt";
    writePoseFile(refine_traj_path.str(), refine_poses);
    //}
    return 0;
}
