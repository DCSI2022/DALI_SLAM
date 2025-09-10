/// Pose Graph Optimization baseline

/// Author: Weitong Wu
/// Contact: wwtgeomatics@gmail.com
/// Date: 2023/10/8

/// Reference: LeGO-LOAM code

#include <iostream>
#include <omp.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include "io.h"
#include "types.h"
#include "imu_process.h"
#include "common_pt_operations.h"
#include "registration.h"
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <pcl/registration/ndt.h>

using namespace std;

struct KeyFrame {
    double frame_start_time;
    gtsam::Pose3 frame_pose;
    ITPointCloud raw_pointcloud; //raw pt
    PointCloud pointcloud_dt; //undistorted pt using odom_poses and imu integration
    int index_in_odom;
};

int main(int argc, char **argv) {
    cout << "Let's start pgo baseline!" << endl;
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

    double key_frame_interval = 0.05; //default 0.3
    int key_frame_num = 10;

    //for parameter sensitivity test
    //for (int j = 0; j < 4; ++j) {

    /// Create result dir
    string root_dir = rosbag_path.substr(0, rosbag_name_index);
    //cout << "root dir: " << root_dir << endl;
    stringstream result_dir_path;
    double current_time = ros::Time().now().toSec();
    result_dir_path << root_dir << "/LOG_pgo_" << setprecision(13) << current_time;
    EnsureDir(result_dir_path.str());

    //1.load data
    vector<PoseData> odometry_poses;
    loadPoseFile(odometry_pose_path, odometry_poses);

    gtsam::Pose3 li_extrinsic;
    loadExtrinsic(extrinsic_path, li_extrinsic);

    //2.load rosbag information
    vector<ImuData> imu_data;
    double imu_dt;
    int point_filter_num = 3;
    loadImuDataFromRosbag(rosbag_path, imu_data, imu_dt);
    imu_dt = 0.00166707; //600hz
    vector<LidarData> lidar_data;
    vector<string> topics;
    topics.push_back(lidar_topic);
    loadLidarDataFromRosbag(rosbag_path, lidar_type, point_filter_num, topics, bag_start, bag_dur, lidar_data);
    //cout << setprecision(13) << lidar_data[0].start_time_s << endl;
    //cout << setprecision(13) << lidar_data.back().start_time_s << endl;

    for (int j = 0; j < lidar_data.size(); ++j) {
        for (int k = 0; k < lidar_data[j].pointcloud.size(); ++k) {
            lidar_data[j].pointcloud[k].relative_time_ms =
                    lidar_data[j].start_time_s + lidar_data[j].pointcloud[k].relative_time_ms / 1000.0;
        }
    }

    //3.key frame selection
    vector<KeyFrame> key_frames;
    double last_key_frame_time;
    int lidar_frame_index = 0;
    for (int j = 0; j < odometry_poses.size(); ++j) {
    //for (int j = 0; j < 100; ++j) {
        if (j == 0) {
            KeyFrame key_frame;
            key_frame.frame_start_time = odometry_poses[j].timestamp;
            key_frame.frame_pose = odometry_poses[j].pose;
            key_frame.index_in_odom = j;
            for (int k = lidar_frame_index; k < lidar_data.size(); ++k) {
                if ((key_frame.frame_start_time - lidar_data[k].start_time_s) < 0.05) {
                    key_frame.raw_pointcloud = lidar_data[k].pointcloud;
                    lidar_frame_index = k;
                    break;
                }
            }
            last_key_frame_time = key_frame.frame_start_time;
            key_frames.push_back(key_frame);
        } else {
            if ((odometry_poses[j].timestamp - last_key_frame_time) > key_frame_interval) {
                KeyFrame key_frame;
                key_frame.frame_start_time = odometry_poses[j].timestamp;
                //cout << "key frame time: " << setprecision(13) << key_frame.frame_start_time << endl;
                key_frame.frame_pose = odometry_poses[j].pose;
                key_frame.index_in_odom = j;
                for (int k = lidar_frame_index; k < lidar_data.size(); ++k) {
                    if ((key_frame.frame_start_time - lidar_data[k].start_time_s) < 0.05) {
                        key_frame.raw_pointcloud = lidar_data[k].pointcloud;
                        lidar_frame_index = k;
                        //cout << "key frame time: " << setprecision(13) << lidar_data[k].start_time_s << endl;
                        break;
                    }
                }
                last_key_frame_time = key_frame.frame_start_time;
                key_frames.push_back(key_frame);
            }
        }
    }
    cout << "key frame size: " << key_frames.size() << endl;

    //4.motion distortion remove
    int imu_data_start_index = 0;
    int imu_data_end_index = 0;
    PointCloudPtr odom_map(new PointCloud());
    vector<PoseData> odom_poses;
    int count = 0;
    for (int j = 0; j < key_frames.size(); ++j) {
    //for (int j = 0; j < 100; ++j) {
        vector<ImuData> one_scan_imu;
        find_imu_data_in_one_scan(key_frames[j].frame_start_time, key_frames[j].frame_start_time + 0.1,
                                  imu_data, imu_data_start_index, imu_data_end_index, one_scan_imu);

        //cout <<"imu size: " << one_scan_imu.size() << endl;
        //cout << "imu start time: " << setprecision(13) << one_scan_imu[0].timestamp_s << endl;

        vector<PoseData> imu_rotation;
        rotation_integration(one_scan_imu, imu_dt, imu_rotation);

        PointCloudPtr undistorted_scan(new PointCloud());
        undistortRawScan1(key_frames[j].raw_pointcloud, li_extrinsic, odometry_poses[key_frames[j].index_in_odom],
                          odometry_poses[key_frames[j].index_in_odom + 1],
                          imu_rotation, undistorted_scan);
        key_frames[j].pointcloud_dt = *undistorted_scan;

        //cout << "point size: " << undistorted_scan->size() << endl;
        if (undistorted_scan->size() == 0)
        {
            count++;
            continue;
        }
        stringstream frame_path;
        frame_path << result_dir_path.str() << "/" << j - count << ".pcd";
        pcl::io::savePCDFileBinaryCompressed(frame_path.str(), *undistorted_scan);
        PoseData odom_pose;
        odom_pose.timestamp = key_frames[j].frame_start_time;
        odom_pose.pose = key_frames[j].frame_pose;
        odom_poses.push_back(odom_pose);

        //output map test
        PointCloudPtr scan_transformed(new PointCloud());
        transformPointcloud(undistorted_scan, key_frames[j].frame_pose, scan_transformed);
        *odom_map += *scan_transformed;
    }
    stringstream odom_map_path;
    odom_map_path << result_dir_path.str() << "/odom_map.pcd";
    //pcl::io::savePCDFileBinaryCompressed(odom_map_path.str(), *odom_map);

    //write odom pose
    stringstream odom_traj_path;
    odom_traj_path << result_dir_path.str() << "/pose.txt";
    writePoseFile(odom_traj_path.str(), odom_poses);

    return 0;

    //5.loop detection
    vector<pair<int, int>> loopPair_index;
    for (int i = 0; i < key_frames.size(); ++i) {
        KeyFrame &source_keyframe = key_frames[i];
        for (int j = i + 1; j < key_frames.size() - 1; ++j) {
            KeyFrame &target_keyframe = key_frames[j];
            double time = fabs(target_keyframe.frame_start_time - source_keyframe.frame_start_time);
            double distance = gtsam::distance3(source_keyframe.frame_pose.translation(),
                                               target_keyframe.frame_pose.translation());
            double angle = fabs((source_keyframe.frame_pose.rotation().inverse() *
                            target_keyframe.frame_pose.rotation()).axisAngle().second * 57.3);

            if (time > 30 && distance < 3 && angle < 15) {
                loopPair_index.push_back(pair<int, int>(i, j));
                cout << "loop pair: " << i << " " << j << endl;
                break;
            }
        }
    }
    cout << "loopPair size: " << loopPair_index.size() << endl;
    //6.loop constraint
    vector<gtsam::Pose3> loop_relativePose;
    vector<pair<int, int>> refine_loopPair_index;
    for (int j = 0; j < loopPair_index.size(); ++j) {
        int source_index = loopPair_index[j].first;
        int target_index = loopPair_index[j].second;
        PointCloudPtr keyframe_raw(new PointCloud());
        PointCloudPtr keyframe1_raw(new PointCloud());
        PointCloudPtr keyframe_transformed(new PointCloud());
        PointCloudPtr keyframep1_transformed(new PointCloud());
        *keyframe_raw = key_frames[source_index].pointcloud_dt;
        *keyframe1_raw = key_frames[target_index].pointcloud_dt;
        transformPointcloud(keyframe_raw, key_frames[source_index].frame_pose, keyframe_transformed);
        transformPointcloud(keyframe1_raw, key_frames[target_index].frame_pose, keyframep1_transformed);
        double overlap_ratio = overlap_calculate(keyframe_transformed, keyframep1_transformed,1,1);
        cout << "overlap ratio between " << source_index << "&" << target_index << ": " << overlap_ratio << endl;

        if (overlap_ratio > 0.1)
        {
            PointCloudPtr source_keyframe(new PointCloud());
            *source_keyframe = key_frames[source_index].pointcloud_dt;
            PointCloudPtr target_keyframe_submap(new PointCloud());
            for (int k = target_index - key_frame_num; k < target_index + key_frame_num; ++k) {
                if (k >= 0 && k < key_frames.size()) {
                    PointCloudPtr keyframe_inTarget(new PointCloud());
                    *keyframe_raw = key_frames[k].pointcloud_dt;
                    transformPointcloud(keyframe_raw,
                                        key_frames[target_index].frame_pose.inverse() * key_frames[k].frame_pose,
                                        keyframe_inTarget);
                    *target_keyframe_submap += *keyframe_inTarget;
                }
            }

            voxelGridFilter<PointXYZ>(source_keyframe, 0.1);
            voxelGridFilter<PointXYZ>(target_keyframe_submap, 0.2);

            pcl::KdTreeFLANN<PointXYZ>::Ptr kd_tree(new pcl::KdTreeFLANN<PointXYZ>());
            kd_tree->setInputCloud(target_keyframe_submap);
            gtsam::Pose3 relative_pose = key_frames[target_index].frame_pose.inverse() *
                                         key_frames[source_index].frame_pose;

            scanToMapRegi(source_keyframe, target_keyframe_submap, kd_tree, relative_pose,
                          15, 3, 0.1, 0.1);

            /*
            stringstream submap_path;
            PointCloudPtr tmp(new PointCloud());
            transformPointcloud(source_keyframe, relative_pose, tmp);
            submap_path << result_dir_path.str() << "/source_submap_" << source_index << "&" << target_index << ".pcd";
            pcl::io::savePCDFileBinaryCompressed(submap_path.str(), *tmp);
            stringstream submap_path1;
            submap_path1 << result_dir_path.str() << "/target_submap_" << source_index << "&" << target_index << ".pcd";
            pcl::io::savePCDFileBinaryCompressed(submap_path1.str(), *target_keyframe_submap);
            */

            refine_loopPair_index.push_back(pair<int, int>(source_index, target_index));
            cout << "refine loop pair: " << source_index << " " << target_index << endl;
            loop_relativePose.push_back(relative_pose.inverse());
        }
    }

    //7.pgo
    gtsam::NonlinearFactorGraph pose_graph;
    gtsam::Values initial_values;
    auto prior_noise_model = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
    pose_graph.addPrior(0, key_frames[0].frame_pose, prior_noise_model);

    for (int j = 0; j < key_frames.size(); ++j) {
        initial_values.insert(j, key_frames[j].frame_pose);
    }

    //add adjacent submap constraints
    for (int j = 0; j < key_frames.size() - 1; ++j) {
        auto odometry_noise_model = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());

        pose_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(j, j + 1,
                                                                      key_frames[j].frame_pose.inverse() *
                                                                              key_frames[j + 1].frame_pose,
                                                                      odometry_noise_model);
    }
    //add loop submap constraints
    cout << "refine_loopPair size: " << refine_loopPair_index.size() << endl;
    for (int j = 0; j < refine_loopPair_index.size(); ++j) {
    //for (int j = 0; j < 0; ++j) {
        /*
        double loopNoise = 1e-7; 
        gtsam::Vector robustNoiseVector6(6); 
        robustNoiseVector6
                << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
        auto robustLoopNoise = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Cauchy::Create(
                        5), 
                gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
        */
        auto loop_noise_model = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7).finished());


        pose_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(refine_loopPair_index[j].first,
                                                                      refine_loopPair_index[j].second,
                                                                      loop_relativePose[j],
                                                                      loop_noise_model);
    }

    //optimization
    cout << "optimizing ..." << endl;
    gtsam::GaussNewtonParams params;
    params.setVerbosity("TERMINATION");//  show info about stopping conditions
    gtsam::GaussNewtonOptimizer optimizer(pose_graph, initial_values, params);
    gtsam::Values result = optimizer.optimize();
    cout << "result size: " << result.size() << std::endl;
    cout << "Optimization complete" << std::endl;

    //update frame poses, output poses
    vector<PoseData> refine_poses;
    for (int j = 0; j < key_frames.size(); ++j) {
        PoseData refine_pose;
        refine_pose.timestamp = key_frames[j].frame_start_time;
        refine_pose.pose = result.at<gtsam::Pose3>(j);
        refine_poses.push_back(refine_pose);
    }

    stringstream refine_traj_path;
    refine_traj_path << result_dir_path.str() << "/traj.txt";
    writePoseFile(refine_traj_path.str(), refine_poses);

    double whole_end_time = omp_get_wtime();
    cout << "whole time: " << whole_end_time - whole_start_time << "s" << endl;
    //}

    return 0;
}
