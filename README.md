# DALI-SLAM: Degeneracy-Aware LiDAR-inertial SLAM with novel distortion correction and accurate multi-constraint pose graph optimization

The code will be open-sourced and refined after acceptance.

> **DALI-SLAM: Degeneracy-Aware LiDAR-inertial SLAM with novel distortion correction and accurate multi-constraint pose graph optimization**<br/>
> [Weitong Wu](https://www.researchgate.net/profile/Weitong-Wu?ev=hdr_xprf), [Chi Chen](https://3s.whu.edu.cn/info/1025/1364.htm), [Bisheng Yang](https://3s.whu.edu.cn/info/1025/1415.htm), [Xianghong Zou](https://zouxianghong.github.io/), [Fuxun Liang](https://liangfxwhu.github.io/), [Yuhang Xu](https://www.researchgate.net/profile/Yuhang-Xu-12), [Xiufeng He](https://dxy.hhu.edu.cn/2017/0412/c6458a93886/page.htm)<br/>
ISPRS Journal of Photogrammetry and Remote Sensing, 2025, 221: 92-108<br/>
> [**Paper**](https://www.sciencedirect.com/science/article/pii/S0924271625000413)

## 🔭 Introduction

<strong>Abstract:</strong> LiDAR-Inertial simultaneous localization and mapping (LI-SLAM) plays a crucial role in various applications such as robot localization and low-cost 3D mapping. However, factors including inaccurate motion distortion estimation and pose graph constraints, and frequent LiDAR feature degeneracy present significant challenges for existing LI-SLAM methods. <strong> To address these issues, we propose DALI-SLAM, an accurate and robust LI-SLAM that consists of degeneracy-aware LiDAR-inertial odometry (DA-LIO) with a dual spline-based motion distortion correction (DS-MDC) module, and multi-constraint pose graph optimization (MC-PGO). </strong> Considering the cumulative errors of micro-electromechanical systems (MEMS) inertial measurement unit (IMU) integration, two continuous-time trajectories in the sliding window are fitted to update the discrete IMU poses for accurate motion distortion correction. In the LiDAR-inertial fusion stage, LiDAR feature degeneracy is detected by analyzing the Jacobian matrix and a remapping strategy is introduced into the updating of error state Kalman Filter (ESKF) to mitigate the influence of degeneracy. Furthermore, in the back-end optimization stage, three types of submap constraints are accurately built with dedicated strategy through a robust variant of the iterative closest point (ICP) method. The proposed method is comprehensively validated using data collected from a helmet-based laser scanning system (HLS) in representative indoor and outdoor environments. Experiment results demonstrate that the proposed method outperforms the SOTA methods on the test data. Specifically, the proposed DS-MDC module reduces trajectory root mean square errors (RMSEs) by 7.9%, 5.8%, and 3.1%, while the degeneracy-aware update strategy achieves additional reductions of 43.3%, 17.7%, and 4.9%, respectively, across three typical sequences compared to existing methods, thereby effectively improving trajectory accuracy. Furthermore, the results of DA-LIO demonstrate an outdoor maximum drift accuracy of one thousandth of a meter, achieving superior performance compared to the SOTA method FAST-LIO2. After performing MC-PGO, the RMSEs of the trajectories are reduced by 25.2%, 9.2%, and 52.4%, respectively, across three typical sequences, demonstrating better performance compared to the SOTA method HBA.
</p>

## 🔗 Related Works
<strong>Dataset:</strong>

[<u>WHU-Helmet Dataset</u>](https://github.com/kafeiyin00/WHU-HelmetDataset): A helmet-based multi-sensor SLAM dataset for the evaluation of real-time 3D mapping in large-scale GNSS-denied environments

<strong>Calibration</strong>

[<u>AFLI-Calib</u>](https://github.com/DCSI2022/AFLI_Calib): Robust LiDAR-IMU extrinsic self-calibration based on adaptive frame length LiDAR odometry 

## 🔗 Competition
DALI-SLAM has been served as a system (partially modified) to participate
 in [ICCV 2023 SLAM Challenge](https://superodometry.com/iccv23_challenge_LiI), achieving 3rd place on the LiDAR inertial track, 1st place in RPE metric, and 2nd place in ATE metric.

## 💡 Citation
If you find this repo helpful, please give us a star .
Please consider citing DALI-SLAM if this program benefits your project
```
@article{wu2025dali,
  title={DALI-SLAM: Degeneracy-aware LiDAR-inertial SLAM with novel distortion correction and accurate multi-constraint pose graph optimization},
  author={Wu, Weitong and Chen, Chi and Yang, Bisheng and Zou, Xianghong and Liang, Fuxun and Xu, Yuhang and He, Xiufeng},
  journal={ISPRS Journal of Photogrammetry and Remote Sensing},
  volume={221},
  pages={92--108},
  year={2025},
  publisher={Elsevier}
}
```

## 🔗 Acknowledgments
We sincerely thank the excellent projects:
- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)
- [basalt-headers](https://github.com/VladyslavUsenko/basalt-headers)
- [GTSAM](https://github.com/borglab/gtsam) 
- [Ceres](https://github.com/ceres-solver/ceres-solver)
- [Sopuhs](https://github.com/strasdat/Sophus)
- [PCL](https://github.com/PointCloudLibrary/pcl)
