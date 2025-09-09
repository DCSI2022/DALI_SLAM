/// Common point operations and other functions
/// Author: Weitong Wu, wwtgeomatics@gmail.com
/// Version 1.1
/// Create Time: 2022/5/23
/// Update Time: 2023/6/25

#include "common_pt_operations.h"
#include <iomanip>

void undistortPointCloud(LidarData raw_pc, vector<PoseData> &pose_vec, gtsam::Pose3 extrinsic, int &pos_index,
                         PointCloudPtr undistort_pc, gtsam::Pose3 &scan_end_pose) {
    //IPointCloudPtr tmp_pc(new IPointCloud());
    PointCloudPtr tmp_pc(new PointCloud());
    for (int i = 0; i < raw_pc.pointcloud.size(); ++i) {
        double point_time = raw_pc.start_time_s +
                            raw_pc.pointcloud.points[i].relative_time_ms / 1000.0;
        double point_time1 = calUTCWeekSec(point_time);
        for (int j = pos_index; j < pose_vec.size() - 1; ++j) {
            if (point_time1 > pose_vec[j].timestamp &&
                point_time1 < pose_vec[j + 1].timestamp) {

                double ratio =
                        (point_time1 - pose_vec[j].timestamp) / (pose_vec[j + 1].timestamp - pose_vec[j].timestamp);

                // interpolate
                gtsam::Pose3 point_pose = pose_vec[j].pose * gtsam::Pose3().expmap(
                        gtsam::Pose3().logmap(pose_vec[j].pose.inverse() * pose_vec[j + 1].pose) * ratio);
                if (i == raw_pc.pointcloud.size() - 1)
                    scan_end_pose = point_pose;
                //PointXYZI pt, pt_transformed;
                PointXYZ pt, pt_transformed;
                pt.x = raw_pc.pointcloud.points[i].x;
                pt.y = raw_pc.pointcloud.points[i].y;
                pt.z = raw_pc.pointcloud.points[i].z;
                //pt.intensity = raw_pc.pointcloud.points[i].reflectivity;
                transformPoint(pt, point_pose * extrinsic, pt_transformed);
                tmp_pc->points.push_back(pt_transformed);
                pos_index = j;
                break;
            }
        }
    }
    // transform pc to scan_end coordinate
    transformPointcloud(tmp_pc, extrinsic.inverse() * scan_end_pose.inverse(), undistort_pc);
}

void transformRawScan(ITPointCloudPtr raw_scan, gtsam::Pose3 scan_start_pose, gtsam::Pose3 scan_end_pose,
                      double scan_start_time, double scan_end_time,
                      PointCloudPtr scan_transformed) {
    for (int i = 0; i < raw_scan->size(); ++i) {
        PointXYZIT pt_raw = raw_scan->points[i];
        PointXYZ point_raw1, point_transformed;
        point_raw1.x = pt_raw.x;
        point_raw1.y = pt_raw.y;
        point_raw1.z = pt_raw.z;

        double ratio = (pt_raw.relative_time_ms - scan_start_time) / (scan_end_time - scan_start_time);

        // interpolate
        gtsam::Pose3 point_pose = scan_start_pose * gtsam::Pose3().expmap(
                gtsam::Pose3().logmap(
                        scan_start_pose.inverse() * scan_end_pose) * ratio);

        transformPoint(point_raw1, point_pose, point_transformed);
        scan_transformed->points.push_back(point_transformed);
    }
}

///@brief create directory if does not exist
///
/// \param[in] dir in std::string format
bool EnsureDir(const std::string &dir) {
    if (dir == "") {
        return false;
    }
    bool bSuccess = true;
    boost::filesystem::path fullpath(dir);
    boost::filesystem::path parent_path = fullpath.parent_path();
    if (!boost::filesystem::exists(parent_path)) {
        bSuccess |= EnsureDir(parent_path.string());
        bSuccess |= boost::filesystem::create_directory(fullpath);
    } else if (!boost::filesystem::exists(fullpath)) {
        bSuccess |= boost::filesystem::create_directory(fullpath);
    }
    return bSuccess;
}

// undistort rawscan using imu rotation and scan_start_pose and end pose
void undistortRawScan1(ITPointCloud raw_scan, gtsam::Pose3 li_extrinsic,
                       PoseData scan_start_pose, PoseData scan_end_pose,
                       vector<PoseData> &imu_rotation, PointCloudPtr undistorted_scan) {
    int imu_rotation_index = 0;
    for (int i = 0; i < raw_scan.size(); ++i) {
        PointXYZIT pt_raw = raw_scan.points[i];
        for (int j = imu_rotation_index; j < imu_rotation.size() - 1; ++j) {
            if (pt_raw.relative_time_ms > imu_rotation[j].timestamp
                && pt_raw.relative_time_ms < imu_rotation[j + 1].timestamp) {
                PointXYZ pt_raw1, pt_transformed;
                pt_raw1.x = pt_raw.x;
                pt_raw1.y = pt_raw.y;
                pt_raw1.z = pt_raw.z;
                double rotation_ratio = (pt_raw.relative_time_ms - imu_rotation[j].timestamp) /
                                        (imu_rotation[j + 1].timestamp - imu_rotation[j].timestamp);
                double translation_ratio = (pt_raw.relative_time_ms - scan_start_pose.timestamp) /
                                           (scan_end_pose.timestamp - scan_start_pose.timestamp);

                gtsam::Rot3 point_rotation;
                rotationInterpo(imu_rotation[j].pose.rotation(),
                                imu_rotation[j + 1].pose.rotation(),
                                rotation_ratio, point_rotation);

                gtsam::Point3 point_translation;
                auto scan_end_translation = (scan_start_pose.pose.inverse() * scan_end_pose.pose).translation();
                translationInterpo(gtsam::Point3(), scan_end_translation, translation_ratio, point_translation);

                gtsam::Pose3 point_pose = gtsam::Pose3(point_rotation, point_translation);
                transformPoint(pt_raw1, point_pose * li_extrinsic, pt_transformed);
                undistorted_scan->push_back(pt_transformed);
                imu_rotation_index = j;
                break;
            }
        }
    }
}

void rotationInterpo(gtsam::Rot3 rotation1, gtsam::Rot3 rotation2, double ratio, gtsam::Rot3 &interpo_rotation) {
    interpo_rotation = rotation1.expmap(gtsam::Rot3().logmap(rotation1.inverse() * rotation2) * ratio);
}

void translationInterpo(gtsam::Point3 tran1, gtsam::Point3 tran2, double ratio, gtsam::Point3 &interpo_tran) {
    interpo_tran = tran1 + (tran2 - tran1) * ratio;
}

double overlap_calculate(PointCloudPtr pc_1, PointCloudPtr pc_2, float voxel_size, int mini_pts_voxel) {
    // 创建一个 hash 表，用于快速查找每个点所在的 voxel 索引
    std::unordered_map<size_t, std::vector<size_t>> pc1_hash;
    std::unordered_map<size_t, std::vector<size_t>> pc2_hash;
    for (size_t i = 0; i < pc_1->points.size(); ++i) {
        PointXYZ pt = pc_1->points[i];
        size_t idx = static_cast<size_t>(std::floor(pt.x / voxel_size)) +
                     (static_cast<size_t>(std::floor(pt.y / voxel_size)) << 10) +
                     (static_cast<size_t>(std::floor(pt.z / voxel_size)) << 20);
        pc1_hash[idx].push_back(i);
    }
    for (size_t i = 0; i < pc_2->points.size(); ++i) {
        PointXYZ pt = pc_2->points[i];
        size_t idx = static_cast<size_t>(std::floor(pt.x / voxel_size)) +
                     (static_cast<size_t>(std::floor(pt.y / voxel_size)) << 10) +
                     (static_cast<size_t>(std::floor(pt.z / voxel_size)) << 20);
        pc2_hash[idx].push_back(i);
    }
    // compute overlap ratio
    int num_overlap_voxel = 0, num_voxel_pc1 = 0, num_voxel_pc2 = 0;
    for (auto iter = pc1_hash.begin(); iter != pc1_hash.end(); ++iter) {
        if (iter->second.size() >= mini_pts_voxel) {
            ++num_voxel_pc1;
        }
    }
    for (auto iter = pc2_hash.begin(); iter != pc2_hash.end(); ++iter) {
        if (iter->second.size() >= mini_pts_voxel) {
            ++num_voxel_pc2;
        }
    }
    for (auto iter = pc1_hash.begin(); iter != pc1_hash.end(); ++iter) {
        for (auto iter1 = pc2_hash.begin(); iter1 != pc2_hash.end(); ++iter1) {
            if (iter1->first == iter->first) {
                if (iter->second.size() >= mini_pts_voxel && iter1->second.size() >= mini_pts_voxel) {
                    ++num_overlap_voxel;
                    break;
                }
            }
        }
    }
    //cout << "num_voxel_pc1 num_voxel_pc2 num_overlap_voxel: " << num_voxel_pc1 << " "
    //     << num_voxel_pc2 << " " << num_overlap_voxel << endl;

    return double(num_overlap_voxel) / num_voxel_pc1;
}

double overlap_calcul_kdtree(PointCloud &pc, KD_TREE<pcl::PointXYZ> &ikdtree, double voxle_size) {
    int pt_size = pc.size();
    vector<PointXYZ, Eigen::aligned_allocator<PointXYZ>> nearest_points;
    vector<float> point_search_sqdis;
    int count = 0;
    for (int i = 0; i < pc.size(); ++i) {
        ikdtree.Nearest_Search(pc.points[i], 5, nearest_points, point_search_sqdis);
        if (point_search_sqdis[4] < voxle_size)
            count++;
    }
    //cout << "scan size & count size : " << pt_size << " " << count << endl;
    return double(count) / pt_size;
}

