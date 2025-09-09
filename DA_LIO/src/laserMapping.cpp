// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com
// Modifier: WeitongWu           wwtgeomatics@gmail.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include "registration.h"
#include "common_pt_operations.h"
#include <basalt/spline/ceres_local_param.hpp>
#include <basalt/spline/se3_spline.h>
#include <ceres/ceres.h>
#include <sophus/interpolate.hpp>
#include <basalt/spline/ceres_spline_helper.h>
#include "orientation_cost_functor.h"
#include "position_cost_functor.h"

using namespace basalt;

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot22[MAXN], s_plot23[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool   start_fit_spline = false;
ros::Publisher pubLaserCloudRaw;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
deque<PoseData> lf_pose_vec; //low-frequency lidar poses
static constexpr double s_to_ns = 1e9;
const int order = 3;
double knot_spacing = 0.05; //50ms
double knot_pose_spacing = 0.09; //90ms
int64_t knot_spacing_ns = static_cast<int64_t>(knot_spacing * s_to_ns);
double degeneracy = 0; //threshold for degeneracy detection

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    //for test
    sensor_msgs::PointCloud2 laserCloudRawmsg;
    pcl::toROSMsg(*ptr, laserCloudRawmsg);
    laserCloudRawmsg.header.stamp = ros::Time().fromSec(last_timestamp_lidar);
    laserCloudRawmsg.header.frame_id = "lidar";
    pubLaserCloudRaw.publish(laserCloudRawmsg);

    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(lidar_end_time) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }

    //Degeneracy detecion and Jacobian remapping
    cout << "effec_feat_num: " << effct_feat_num << endl;
    if (effct_feat_num != 0)
    {
        Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(effct_feat_num, 6);
        for (int j = 0; j < effct_feat_num; ++j) {
            Jacobian.block<1,6>(j,0) = ekfom_data.h_x.block<1,6>(j,0);
        }
        Matrix<double,6,6> coefficient_matrix = Jacobian.transpose() * Jacobian;
        coefficient_matrix = Jacobian.transpose() * Jacobian;
        SelfAdjointEigenSolver<Matrix<double, 6, 6>> eigen_solver(coefficient_matrix);
        if (eigen_solver.info() != Eigen::Success)
        {
            abort();
        }
        if (eigen_solver.eigenvalues()[0] > degeneracy * degeneracy)
            return;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::VectorXd singlevalues = svd.singularValues();
        cout << "singlevalues: " << singlevalues.transpose() << endl;
        Eigen::MatrixXd U = svd.matrixU();
        Eigen::MatrixXd V = svd.matrixV();

        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(effct_feat_num, 6);
        for (int j = 0; j < 6; ++j) {
            if (singlevalues[j] > degeneracy) {
                S(j, j) = singlevalues[j];
            }
            else
                S(j, j) = 0.0001;
        }
        Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(effct_feat_num, 6);
        ekfom_data.h_x <<  U * S * V.transpose(), tmp;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

void calculate_matching_stability(state_ikfom &s, vector<double> &eigenvalue_vec, Matrix<double, 6, 6> &coefficient_matrix) {
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    /** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++) {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];


        /** Find the closest surfaces in the map **/
        ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
        point_selected_surf[i] =
                points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                           : true;

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f)) {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9) {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++) {
        if (point_selected_surf[i]) {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1) {
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    MatrixXd Jacobian = MatrixXd::Zero(effct_feat_num, 6);

    for (int i = 0; i < effct_feat_num; i++) {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);

        Jacobian.block<1, 6>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A);
    }
    //Matrix<double,6,6> coefficient_matrix = Jacobian.transpose() * Jacobian;
    coefficient_matrix = Jacobian.transpose() * Jacobian;
    SelfAdjointEigenSolver<Matrix<double, 6, 6>> eigen_solver(coefficient_matrix);
    if (eigen_solver.info() != Eigen::Success)
    {
        abort();
    }
    for (int j = 0; j < 6; ++j) {
        eigenvalue_vec.push_back(eigen_solver.eigenvalues()[j]);
    }
}

template <int N>
bool inquireSplinePose(Se3Spline<N> &spline, double dt, double t, Eigen::Vector3d &position, Sophus::SO3d &rotation)
{
    // inquire transformation from spline
    int64_t st_ns = t * 1e9 - spline.minTimeNs();

    if (st_ns < 0)
    {
        return false;
    }

    int64_t dt_ns = dt * 1e9;
    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    if (size_t(s + N) > spline.numKnots())
    {
        return false;
    }

    {
        std::vector<const double *> vec;
        for (int i = 0; i < N; i++)
        {
            vec.emplace_back(spline.getKnotSO3(s + i).data());
        }

        CeresSplineHelper<N>::template evaluate_lie<double, Sophus::SO3>(
                &vec[0], u, 1 / dt, &rotation);
    }

    {
        std::vector<const double *> vec;
        for (int i = 0; i < N; i++)
        {
            vec.emplace_back(spline.getKnotPos(s + i).data());
        }

        CeresSplineHelper<N>::template evaluate<double, 3, 0>(
                &vec[0], u, 1 / dt, &position);
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<string>("map_file_path",map_file_path,"");
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    nh.param<double>("mapping/fov_degree",fov_deg,180);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    nh.param<double>("mapping/degeneracy",degeneracy,4.48);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;
    
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    FILE *f_traj_tum;
    stringstream traj_tum_path;
    double current_time = ros::Time().now().toSec();
    traj_tum_path << root_dir << "/Log/traj" << setprecision(13) << current_time << ".txt";
    f_traj_tum = fopen(traj_tum_path.str().c_str(),"w");
    fprintf(f_traj_tum, "#sec,x,y,z,qx,qy,qz,qw");
    fprintf(f_traj_tum, "\r\n");
    fflush(f_traj_tum);

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    //Debug
    //ofstream f_spline_pose(root_dir + "/Log/spline_pose.txt");
    //ofstream f_linear_pose(root_dir + "/Log/linear_pose.txt");
    //ofstream f_imu_pose(root_dir + "/Log/imu_poses.txt");
    //ofstream f_imu_pose_correct(root_dir + "/Log/imu_poses_correct.txt");
    ofstream f_degenerate(root_dir + "/Log/mini_eigenvalue.txt");

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    pubLaserCloudRaw = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_raw",100000);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/Odometry", 100000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 100000);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);//capture the "ctrl+c" signal
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        if(sync_packages(Measures)) //get one lidar scan and corresponding imu measurements
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            vector<Pose6D> imu_poses_vec;
            PointCloudXYZI::Ptr feats_undistort_copy(new PointCloudXYZI()); //raw point cloud copy for re-undistort
            p_imu->Process(Measures, kf, feats_undistort, feats_undistort_copy, imu_poses_vec); //imu integration and correct distortion
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();//map is maintained as a cube centered at the current location

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                lf_pose_vec.clear();
                continue;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

            vector<double> eigenvalue_vec;
            double mini_eigenvalue;
            Matrix<double, 6, 6> coefficient_matrix;
            calculate_matching_stability(state_point, eigenvalue_vec, coefficient_matrix);
            if (eigenvalue_vec.size() != 0)
            {
                mini_eigenvalue = eigenvalue_vec[0];
                for (int j = 0; j < 6; ++j) {
                    if (j == 0)
                        cout << "eigen value of lidar jacobian: ";
                    cout << eigenvalue_vec[j] << " ";
                    if (j == 5)
                        cout << endl;
                }

                f_degenerate.setf(ios::fixed);
                f_degenerate.setf(ios::showpoint);
                f_degenerate.precision(4);
                f_degenerate << lidar_end_time << " " << eigenvalue_vec[0] << endl;
            }

            //scan-to-Map registration using filter predicted pose as initial guess
            gtsam::Pose3 scanToMap_pose = gtsam::Pose3(gtsam::Rot3(state_point.rot.toRotationMatrix()),
                                                       gtsam::Point3(state_point.pos));
            gtsam::Pose3 extrinsic = gtsam::Pose3(gtsam::Rot3(state_point.offset_R_L_I.toRotationMatrix()),
                                                  gtsam::Point3(state_point.offset_T_L_I));
            pcl::PointCloud<PointType>::Ptr scan(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr scan_inIMU(new pcl::PointCloud<PointType>());
            for (int j = 0; j < feats_down_body->size(); ++j) {
                PointType pt;
                pt.x = feats_down_body->points[j].x;
                pt.y = feats_down_body->points[j].y;
                pt.z = feats_down_body->points[j].z;
                scan->push_back(pt);
            }
            transformPointcloud(scan, extrinsic, scan_inIMU);
            //voxelGridFilter<PointType>(scan_inIMU, 0.4);
            scanToMapRegi(scan_inIMU, ikdtree, scanToMap_pose,
                          15, 1.0, 0.05, 0.1);
            //cout << "scan-to-Map pose: ";
            //scanToMap_pose.print();
            PoseData lf_pose;
            lf_pose.timestamp = Measures.lidar_end_time;
            lf_pose.pose = scanToMap_pose;
            lf_pose_vec.push_back(lf_pose);
            //cout << "lf_pose_vec size: " << lf_pose_vec.size() << endl;
            cout << "scan-to-Map registration done" << endl;

            if (lf_pose_vec.size() == 4) //3 order
                start_fit_spline = true;

            if (start_fit_spline)
            {
                //init spline
                cout << "start to init lf-spline" << endl;
                int64_t start_t_ns = (lf_pose_vec.front().timestamp + knot_pose_spacing) * s_to_ns;
                double knot_t = lf_pose_vec.front().timestamp + knot_pose_spacing;
                Se3Spline<order> lf_spline(knot_spacing_ns, start_t_ns);

                vector<double> knots_ts;
                while (knot_t < lf_pose_vec.back().timestamp + 2 * knot_spacing)
                {
                    //cout << setprecision(13) << "knot time: " << knot_t << " ";
                    knots_ts.push_back(knot_t);
                    knot_t += knot_spacing;
                }
                //cout << "knots_ts size: " << knots_ts.size() << endl;

                //fitting the spline and correct imu integration poses
                //1.First, initialize the spline control points according to the low-frequency pose and fit the low-frequency spline
                // interpolate knots with dt pose
                Eigen::aligned_vector<Sophus::SO3d> init_knots_r;
                Eigen::aligned_vector<Eigen::Vector3d> init_knots_p;

                int pose_index = 0;
                for (size_t i = 0; i < knots_ts.size(); i++)
                {
                    for (size_t j = pose_index; j < lf_pose_vec.size() - 1; j++)
                    {
                        if (knots_ts[i] >= lf_pose_vec[j].timestamp && knots_ts[i] < lf_pose_vec[j + 1].timestamp)
                        {
                            Eigen::Vector3d p1 = lf_pose_vec[j].pose.translation();
                            Sophus::SO3d r1 = Sophus::SO3d(lf_pose_vec[j].pose.rotation().matrix());

                            Eigen::Vector3d p2 = lf_pose_vec[j + 1].pose.translation();
                            Sophus::SO3d r2 = Sophus::SO3d(lf_pose_vec[j + 1].pose.rotation().matrix());

                            double dt = (knots_ts[i] - lf_pose_vec[j].timestamp) / (lf_pose_vec[j + 1].timestamp - lf_pose_vec[j].timestamp);

                            Eigen::Vector3d p = p1 + dt * (p2 - p1);
                            Sophus::SO3d r;
                            r = Sophus::interpolate(r1, r2, dt);

                            init_knots_p.push_back(p);
                            init_knots_r.push_back(r);

                            pose_index = j;
                            break;
                        }
                    }
                    //Since the time of the last two control points exceeds the time of the last low-frequency pose, only the last pose can be assigned to them.
                    if (i == knots_ts.size()-1 || i == knots_ts.size()-2)
                    {
                        Eigen::Vector3d p = lf_pose_vec.back().pose.translation();
                        Sophus::SO3d r = Sophus::SO3d(lf_pose_vec.back().pose.rotation().matrix());
                        init_knots_p.push_back(p);
                        init_knots_r.push_back(r);
                    }
                }
                //cout << "knot_r size: " << init_knots_r.size() << endl;

                /*
                Eigen::Quaterniond tmp_q(init_knots_r[3].matrix());

                f_linear_pose.setf(ios::fixed);
                f_linear_pose.setf(ios::showpoint);
                f_linear_pose.precision(4);
                f_linear_pose << knots_ts[3] << " " << init_knots_p[3][0] << " " << init_knots_p[3][1] << " " << init_knots_p[3][2]
                              << " " << tmp_q.x() << " " << tmp_q.y() << " " << tmp_q.z() << " " << tmp_q.w() << endl;
                */

                // add knots to spline
                for (int i = 0; i < init_knots_p.size(); ++i)
                {
                    lf_spline.knotsPushBack(Sophus::SE3d(init_knots_r[i], init_knots_p[i]));
                }

                ///Construct problem(fitting bspline with dt_pose),add RotationLocalParametrization
                ceres::Problem problem;

                ///Add Measurement(cost function)(the pose calculated by bspline vs the pose linear interpolated by dt_lo_pose)
                for (int i = 0; i < lf_pose_vec.size(); i++) {
                    int64_t t_ns = static_cast<int64_t>(lf_pose_vec[i].timestamp * s_to_ns);

                    int64_t st_ns = t_ns - lf_spline.minTimeNs();

                    if (st_ns < 0) {
                        //cout << "dt pose time small than spline start time" << endl;
                        continue;
                    }

                    int64_t s = st_ns / knot_spacing_ns;
                    //cout << "s is: " << s << "and s+ order is: " << s + order << "and numKnots is: "
                    //     << lf_spline.numKnots() << endl;
                    double u = double(st_ns % knot_spacing_ns) / double(knot_spacing_ns);

                    if (size_t(s + order) > lf_spline.numKnots()) { continue; }

                    using FunctorT = PositionCostFunctor<order>;
                    double noise = 1.0;
                    FunctorT *functor = new FunctorT(lf_pose_vec[i].pose.translation(), u, 1 / knot_spacing, 1.0 / noise);

                    ceres::DynamicAutoDiffCostFunction<FunctorT> *cost_function =
                            new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

                    // parameter blocks for translation
                    for (int i = 0; i < order; i++) {
                        cost_function->AddParameterBlock(3);
                    }
                    cost_function->SetNumResiduals(3);

                    std::vector<double *> vec;
                    for (int i = 0; i < order; i++) {
                        vec.emplace_back(lf_spline.getKnotPos(s + i).data());
                        problem.AddParameterBlock(lf_spline.getKnotPos(s + i).data(), 3);
                        // fix first control point
                        if (int(s + i) == 0 )
                            problem.SetParameterBlockConstant(vec.back());
                    }
                    problem.AddResidualBlock(cost_function, NULL, vec);
                }

                for (int i = 0; i < lf_pose_vec.size(); i++) {
                    int64_t t_ns = static_cast<int64_t>(lf_pose_vec[i].timestamp * s_to_ns);

                    int64_t st_ns = t_ns - lf_spline.minTimeNs();

                    if (st_ns < 0) {
                        //cout << "dt pose time small than spline start time" << endl;
                        continue;
                    }

                    int64_t s = st_ns / knot_spacing_ns;
                    double u = double(st_ns % knot_spacing_ns) / double(knot_spacing_ns);

                    if (size_t(s + order) > lf_spline.numKnots()) { continue; }

                    using FunctorT = OrientationCostFunctor<order>;
                    double noise = 1.0;
                    FunctorT *functor = new FunctorT(Sophus::SO3d(lf_pose_vec[i].pose.rotation().matrix()), u,
                                                     1 / knot_spacing, 1.0 / noise);

                    ceres::DynamicAutoDiffCostFunction<FunctorT> *cost_function =
                            new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

                    // parameter blocks for orientation
                    for (int i = 0; i < order; i++) {
                        cost_function->AddParameterBlock(4);
                    }
                    cost_function->SetNumResiduals(3);

                    std::vector<double *> vec;
                    for (int i = 0; i < order; i++) {
                        vec.emplace_back(lf_spline.getKnotSO3(s + i).data());
                        ceres::LocalParameterization *local_parameterization =
                                new LieLocalParameterization<Sophus::SO3d>();
                        problem.AddParameterBlock(lf_spline.getKnotSO3(s + i).data(), 4, local_parameterization);
                        // fix first control point
                        if (int(s + i) == 0 )
                            problem.SetParameterBlockConstant(vec.back());
                    }
                    problem.AddResidualBlock(cost_function, NULL, vec);
                }

                ///Solve problem
                //cout << "\n\nOptimizing ...\n\n";
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
                options.max_num_iterations = 50;
                options.num_threads = 8;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                //cout << summary.FullReport() << endl;

                Sophus::SO3d inquire_rotaion;
                Eigen::Vector3d inquire_translation;
                double inquire_time = lf_pose_vec[2].timestamp;
                /*
                while (inquire_time < lf_pose_vec[3].timestamp)
                {
                    inquireSplinePose(lf_spline, knot_spacing, inquire_time, inquire_translation, inquire_rotaion);
                    Eigen::Quaterniond tmp_q(inquire_rotaion.matrix());
                    inquire_time += 0.01;

                    f_spline_pose.setf(ios::fixed);
                    f_spline_pose.setf(ios::showpoint);
                    f_spline_pose.precision(4);
                    f_spline_pose << inquire_time << " " << inquire_translation[0] << " " << inquire_translation[1] << " " << inquire_translation[2]
                                  << " " << tmp_q.x() << " " << tmp_q.y() << " " << tmp_q.z() << " " << tmp_q.w() << endl;
                }*/

                cout << "lf-spline fitting done" << endl;

                if (imu_poses_vec.size() != 0)
                {
                    cout << "start to fit lf_imu_spline" << endl;
                    //Let each IMU time add the frame start time
                    for (int j = 0; j < imu_poses_vec.size(); ++j) {
                        imu_poses_vec[j].offset_time += Measures.lidar_beg_time;
                        //cout << setprecision(13) << imu_poses_vec[j].offset_time << " ";
                    }
                    vector<PoseData> lf_imu_pose_vec;
                    V3D pos_imu;
                    M3D R_imu;
                    for (int j = 0; j < lf_pose_vec.size(); ++j) {
                        lf_imu_pose_vec.push_back(lf_pose_vec[j]);
                        if (j == lf_pose_vec.size()-1){
                            PoseData lf_imu_pose;
                            lf_imu_pose.timestamp = lf_pose_vec[j].timestamp;
                            pos_imu << VEC_FROM_ARRAY(imu_poses_vec.back().pos);
                            R_imu << MAT_FROM_ARRAY(imu_poses_vec.back().rot);
                            lf_imu_pose.pose = gtsam::Pose3(gtsam::Rot3(R_imu),gtsam::Point3(pos_imu));
                            lf_imu_pose_vec.push_back(lf_imu_pose);
                        }
                    }
                    Se3Spline<order> lf_imu_spline(knot_spacing_ns, start_t_ns);
                    
                    init_knots_p.clear();
                    init_knots_r.clear();

                    pose_index = 0;
                    for (size_t i = 0; i < knots_ts.size(); i++)
                    {
                        for (size_t j = pose_index; j < lf_imu_pose_vec.size() - 1; j++)
                        {
                            if (knots_ts[i] >= lf_imu_pose_vec[j].timestamp && knots_ts[i] < lf_imu_pose_vec[j + 1].timestamp)
                            {
                                Eigen::Vector3d p1 = lf_imu_pose_vec[j].pose.translation();
                                Sophus::SO3d r1 = Sophus::SO3d(lf_imu_pose_vec[j].pose.rotation().matrix());

                                Eigen::Vector3d p2 = lf_imu_pose_vec[j + 1].pose.translation();
                                Sophus::SO3d r2 = Sophus::SO3d(lf_imu_pose_vec[j + 1].pose.rotation().matrix());

                                double dt = (knots_ts[i] - lf_imu_pose_vec[j].timestamp) / (lf_imu_pose_vec[j + 1].timestamp - lf_imu_pose_vec[j].timestamp);

                                Eigen::Vector3d p = p1 + dt * (p2 - p1);
                                Sophus::SO3d r;
                                r = Sophus::interpolate(r1, r2, dt);

                                init_knots_p.push_back(p);
                                init_knots_r.push_back(r);

                                pose_index = j;
                                break;
                            }
                        }  
                        if (i == knots_ts.size()-1 || i == knots_ts.size()-2)
                        {
                            Eigen::Vector3d p = lf_imu_pose_vec.back().pose.translation();
                            Sophus::SO3d r = Sophus::SO3d(lf_imu_pose_vec.back().pose.rotation().matrix());
                            init_knots_p.push_back(p);
                            init_knots_r.push_back(r);
                        }
                    }
                    //cout << "knot_r size: " << init_knots_r.size() << endl;
                    // add knots to spline
                    for (int i = 0; i < init_knots_p.size(); ++i)
                    {
                        lf_imu_spline.knotsPushBack(Sophus::SE3d(init_knots_r[i], init_knots_p[i]));
                    }

                    ///Construct problem(fitting bspline with dt_pose),add RotationLocalParametrization
                    ceres::Problem problem2;

                    ///Add Measurement(cost function)(the pose calculated by bspline vs the pose linear interpolated by dt_lo_pose)
                    for (int i = 0; i < lf_imu_pose_vec.size(); i++) {
                        int64_t t_ns = static_cast<int64_t>(lf_imu_pose_vec[i].timestamp * s_to_ns);

                        int64_t st_ns = t_ns - lf_imu_spline.minTimeNs();

                        if (st_ns < 0) {
                            //cout << "dt pose time small than spline start time" << endl;
                            continue;
                        }

                        int64_t s = st_ns / knot_spacing_ns;
                        //cout << "s is: " << s << "and s+ order is: " << s + order << "and numKnots is: "
                        //     << lf_spline.numKnots() << endl;
                        double u = double(st_ns % knot_spacing_ns) / double(knot_spacing_ns);

                        if (size_t(s + order) > lf_imu_spline.numKnots()) { continue; }

                        using FunctorT = PositionCostFunctor<order>;
                        double noise = 1.0;
                        FunctorT *functor = new FunctorT(lf_imu_pose_vec[i].pose.translation(), u, 1 / knot_spacing, 1.0 / noise);

                        ceres::DynamicAutoDiffCostFunction<FunctorT> *cost_function =
                                new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

                        // parameter blocks for translation
                        for (int i = 0; i < order; i++) {
                            cost_function->AddParameterBlock(3);
                        }
                        cost_function->SetNumResiduals(3);

                        std::vector<double *> vec;
                        for (int i = 0; i < order; i++) {
                            vec.emplace_back(lf_imu_spline.getKnotPos(s + i).data());
                            problem2.AddParameterBlock(lf_imu_spline.getKnotPos(s + i).data(), 3);
                            if (int(s + i) == 0 )
                                problem2.SetParameterBlockConstant(vec.back());
                        }
                        problem2.AddResidualBlock(cost_function, NULL, vec);
                    }

                    for (int i = 0; i < lf_imu_pose_vec.size(); i++) {
                        int64_t t_ns = static_cast<int64_t>(lf_imu_pose_vec[i].timestamp * s_to_ns);

                        int64_t st_ns = t_ns - lf_imu_spline.minTimeNs();

                        if (st_ns < 0) {
                            //cout << "dt pose time small than spline start time" << endl;
                            continue;
                        }

                        int64_t s = st_ns / knot_spacing_ns;
                        double u = double(st_ns % knot_spacing_ns) / double(knot_spacing_ns);

                        if (size_t(s + order) > lf_imu_spline.numKnots()) { continue; }

                        using FunctorT = OrientationCostFunctor<order>;
                        double noise = 1.0;
                        FunctorT *functor = new FunctorT(Sophus::SO3d(lf_imu_pose_vec[i].pose.rotation().matrix()), u,
                                                         1 / knot_spacing, 1.0 / noise);

                        ceres::DynamicAutoDiffCostFunction<FunctorT> *cost_function =
                                new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

                        // parameter blocks for orientation
                        for (int i = 0; i < order; i++) {
                            cost_function->AddParameterBlock(4);
                        }
                        cost_function->SetNumResiduals(3);

                        std::vector<double *> vec;
                        for (int i = 0; i < order; i++) {
                            vec.emplace_back(lf_imu_spline.getKnotSO3(s + i).data());
                            ceres::LocalParameterization *local_parameterization =
                                    new LieLocalParameterization<Sophus::SO3d>();
                            problem2.AddParameterBlock(lf_imu_spline.getKnotSO3(s + i).data(), 4, local_parameterization);
                            if (int(s + i) == 0 )
                                problem2.SetParameterBlockConstant(vec.back());
                        }
                        problem2.AddResidualBlock(cost_function, NULL, vec);
                    }

                    ///Solve problem
                    //cout << "\n\nOptimizing ...\n\n";
                    ceres::Solver::Options options2;
                    options2.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
                    options2.max_num_iterations = 50;
                    options2.num_threads = 8;
                    options2.minimizer_progress_to_stdout = false;
                    ceres::Solver::Summary summary2;
                    ceres::Solve(options, &problem2, &summary2);
                    //cout << summary2.FullReport() << endl;

                    //Update high-frequency IMU pose
                    vector<gtsam::Pose3> imu_gtsam_pose_vec;
                    //f_imu_pose.setf(ios::fixed);
                    //f_imu_pose.setf(ios::showpoint);
                    //f_imu_pose.precision(4);
                    for (int j = 0; j < imu_poses_vec.size(); ++j) {
                        pos_imu << VEC_FROM_ARRAY(imu_poses_vec[j].pos);
                        R_imu << MAT_FROM_ARRAY(imu_poses_vec[j].rot);
                        gtsam::Pose3 imu_gtsam_pose = gtsam::Pose3(gtsam::Rot3(R_imu), gtsam::Point3(pos_imu));
                        imu_gtsam_pose_vec.push_back(imu_gtsam_pose);
                        Eigen::Quaterniond tmp_q(R_imu);
                        //f_imu_pose << imu_poses_vec[j].offset_time << " " << pos_imu[0] << " " << pos_imu[1] << " " << pos_imu[2]
                        //           << " " << tmp_q.x() << " " << tmp_q.y() << " " << tmp_q.z() << " " << tmp_q.w() << endl;
                    }

                    //update
                    vector<Pose6D> imu_poses_correct_vec;
                    //f_imu_pose_correct.setf(ios::fixed);
                    //f_imu_pose_correct.setf(ios::showpoint);
                    //f_imu_pose_correct.precision(4);
                    V3D angvel_avr, acc_imu, vel_imu;
                    Eigen::Vector3d imu_state_pos;
                    SO3 imu_state_rot;
                    for (int j = 0; j < imu_poses_vec.size(); ++j) {
                        inquire_time = imu_poses_vec[j].offset_time;
                        inquireSplinePose(lf_spline, knot_spacing, inquire_time, inquire_translation, inquire_rotaion);
                        gtsam::Pose3 lf_spline_pose = gtsam::Pose3(gtsam::Rot3(inquire_rotaion.matrix()),
                                                                   gtsam::Point3(inquire_translation));
                        inquireSplinePose(lf_imu_spline, knot_spacing, inquire_time, inquire_translation, inquire_rotaion);
                        gtsam::Pose3 lf_spline_imu_pose = gtsam::Pose3(gtsam::Rot3(inquire_rotaion.matrix()),
                                                                   gtsam::Point3(inquire_translation));
                        gtsam::Pose3 imu_gtsam_pose_correct = lf_spline_pose * lf_spline_imu_pose.inverse() * imu_gtsam_pose_vec[j];

                        angvel_avr << VEC_FROM_ARRAY(imu_poses_vec[j].gyr);
                        acc_imu << VEC_FROM_ARRAY(imu_poses_vec[j].acc);
                        vel_imu << VEC_FROM_ARRAY(imu_poses_vec[j].vel);

                        imu_poses_correct_vec.push_back(
                                set_pose6d(imu_poses_vec[j].offset_time, acc_imu, angvel_avr,
                                           vel_imu, imu_gtsam_pose_correct.translation(),
                                           imu_gtsam_pose_correct.rotation().matrix()));

                        Eigen::Quaterniond tmp_q(imu_gtsam_pose_correct.rotation().matrix());
                        //f_imu_pose_correct << imu_poses_vec[j].offset_time << " " << imu_gtsam_pose_correct.translation()[0] << " "
                        //                   << imu_gtsam_pose_correct.translation()[1] << " "
                        //                   << imu_gtsam_pose_correct.translation()[2]
                        //                   << " " << tmp_q.x() << " " << tmp_q.y() << " " << tmp_q.z() << " " << tmp_q.w() << endl;

                        if (j == imu_poses_vec.size()-1)
                        {
                            imu_state_pos = imu_gtsam_pose_correct.translation();
                            imu_state_rot = SO3(tmp_q);
                            //imu_state_pos = imu_gtsam_pose_vec[j].translation();
                            //imu_state_rot = SO3(imu_gtsam_pose_vec[j].rotation().matrix());
                        }
                    }
                    cout << "correct imu integration poses done" << endl;

                    //re-undistort feats_undistort and get feats_down_body
                    /*** undistort each lidar point (backward propagation) ***/
                    //让每一个IMU的时间减去帧起始时间
                    for (int j = 0; j < imu_poses_vec.size(); ++j) {
                        imu_poses_vec[j].offset_time -= Measures.lidar_beg_time;
                        imu_poses_correct_vec[j].offset_time -= Measures.lidar_beg_time;
                    }
                    if (feats_undistort_copy->points.begin() == feats_undistort_copy->points.end()) return 0;
                    auto it_pcl = feats_undistort_copy->points.end() - 1;
                    for (auto it_kp = imu_poses_correct_vec.end() - 1; it_kp != imu_poses_correct_vec.begin(); it_kp--)
                    //for (auto it_kp = imu_poses_vec.end() - 1; it_kp != imu_poses_vec.begin(); it_kp--)
                    {
                        auto head = it_kp - 1;
                        auto tail = it_kp;
                        R_imu<<MAT_FROM_ARRAY(head->rot);
                        // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
                        vel_imu<<VEC_FROM_ARRAY(head->vel);
                        pos_imu<<VEC_FROM_ARRAY(head->pos);
                        acc_imu<<VEC_FROM_ARRAY(tail->acc);
                        angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

                        for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
                        {
                            double dt = it_pcl->curvature / double(1000) - head->offset_time;

                            /* Transform to the 'end' frame, using only the rotation
                             * Note: Compensation direction is INVERSE of Frame's moving direction
                             * So if we want to compensate a point at timestamp-i to the frame-e
                             * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
                            M3D R_i(R_imu * Exp(angvel_avr, dt));

                            V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
                            V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state_pos); // minus imu_state.pos in there. the same as minus imu_state.pos in [imu_state.rot.conjugate() * ( XX - imu_state.pos)]
                            V3D P_compensate = state_point.offset_R_L_I.conjugate() * (imu_state_rot.conjugate() * (R_i * (state_point.offset_R_L_I * P_i + state_point.offset_T_L_I) + T_ei) - state_point.offset_T_L_I);

                            // save Undistorted points and their rotation
                            it_pcl->x = P_compensate(0);
                            it_pcl->y = P_compensate(1);
                            it_pcl->z = P_compensate(2);

                            if (it_pcl == feats_undistort_copy->points.begin()) break;
                        }
                    }
                    cout << "re-undistort feats_undistort done" << endl;
                    //downsample
                    downSizeFilterSurf.setInputCloud(feats_undistort_copy);
                    downSizeFilterSurf.filter(*feats_down_body);
                    feats_down_size = feats_down_body->points.size();
                    normvec->resize(feats_down_size);
                    feats_down_world->resize(feats_down_size);
                    pointSearchInd_surf.resize(feats_down_size);
                    Nearest_Points.resize(feats_down_size);
                }
            }
            t1 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3(state_point.rot.toRotationMatrix()),
                                             gtsam::Point3(state_point.pos));
            lf_pose.pose = pose;
            lf_pose.timestamp = Measures.lidar_end_time;
            lf_pose_vec.pop_back();
            lf_pose_vec.push_back(lf_pose);
            if (lf_pose_vec.size() == 4)
                lf_pose_vec.pop_front();
            start_fit_spline = false;

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);

            //write traj
            fprintf(f_traj_tum, "%lf ", Measures.lidar_end_time);
            fprintf(f_traj_tum, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));
            fprintf(f_traj_tum, "%lf %lf %lf %lf", geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w);
            fprintf(f_traj_tum, "\r\n");
            fflush(f_traj_tum);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                //aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                //aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                //aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                //aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                //aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                //aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_end_time;
                s_plot[time_log_counter] = t5 - t0; //total time
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot22[time_log_counter] = feats_down_body->points.size();
                s_plot23[time_log_counter] = t1 - t0; // undistort + voxel downsample
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = match_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = solve_time;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ time ] total: %0.6f undistort + voxel downsample: %0.6f match: %0.6f increment: %0.6f delete: %0.6f \n",t5-t0,t1-t0,match_time,kdtree_incremental_time,kdtree_delete_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        FILE *fp2;
        stringstream runtimelog_path;
        double current_time = ros::Time().now().toSec();
        runtimelog_path << root_dir << "/Log/fast_lio_time_log" << setprecision(13) << current_time << ".csv";
        fp2 = fopen(runtimelog_path.str().c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, scan point size(voxel downsample), undistort+downsample, incremental time, match time, solve_time, delete size, delete time, tree size start, tree size end, add point size\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%d,%0.8f,%0.8f,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d\n",T1[i],s_plot[i],int(s_plot2[i]),int(s_plot22[i]),s_plot23[i], s_plot3[i],s_plot4[i], s_plot9[i], int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]));
        }
        fclose(fp2);
    }

    return 0;
}
