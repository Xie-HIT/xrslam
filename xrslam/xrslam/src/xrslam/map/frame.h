#ifndef XRSLAM_FRAME_H
#define XRSLAM_FRAME_H

#include <xrslam/common.h>
#include <xrslam/estimation/preintegrator.h>
#include <xrslam/estimation/state.h>
#include <xrslam/utility/identifiable.h>
#include <xrslam/utility/tag.h>

namespace xrslam {

class Config;
class Track;
class Map;
class ReprojectionErrorFactor;

enum FrameTag {
    FT_KEYFRAME = 0,
    FT_NO_TRANSLATION,
    FT_FIX_POSE,
    FT_FIX_MOTION
};

class Frame : public Tagged<FrameTag>, public Identifiable<Frame> {
    friend class Track;
    friend class Map;
    struct construct_by_frame_t;
    Map *map;

  public:
    Frame();
    Frame(const Frame &frame, const construct_by_frame_t &construct);
    virtual ~Frame();

    std::unique_ptr<Frame> clone() const; // 深拷贝

    size_t keypoint_num() const { return bearings.size(); }

    void append_keypoint(const vector<3> &keypoint);

    const vector<3> &get_keypoint(size_t keypoint_index) const {
        runtime_assert(abs(bearings[keypoint_index].norm() - 1.0) <= 0.001,
                       "bearing vector is not normalized");
        return bearings[keypoint_index];
    }

    Track *get_track(size_t keypoint_index) const {
        return tracks[keypoint_index];
    }

    // 若此 Track 未建立，则建立 Track，然后将此 Track 注册到 Map 和 Frame 中
    Track *get_track(size_t keypoint_index, Map *allocation_map);

    void detect_keypoints(Config *config); // 检测角点
    void track_keypoints(Frame *next_frame, Config *config); // 光流跟踪角点

    PoseState get_pose(const ExtrinsicParams &sensor) const; // 获取 body 系的位姿
    void set_pose(const ExtrinsicParams &sensor, const PoseState &pose); // 设置 body 系的位姿

    bool has_map() const { return map != nullptr; }

    std::unique_lock<std::mutex> lock() const; // 获得 map 的锁

    matrix<3> K; // 内参
    matrix<2> sqrt_inv_cov;
    std::shared_ptr<Image> image; // 图像

    PoseState pose; // 相机位姿
    MotionState motion; // IMU 状态
    ExtrinsicParams camera; // 外参
    ExtrinsicParams imu;

    // 预积分
    PreIntegrator preintegration;
    PreIntegrator keyframe_preintegration;

    std::vector<std::unique_ptr<Frame>> subframes; // 关键帧会附带一些子帧，用于增强跟踪效果
    std::vector<std::unique_ptr<ReprojectionErrorFactor>>
        reprojection_error_factors;

  private:
    std::vector<vector<3>> bearings; // 特征点的方向向量（单位模长）
    std::vector<Track *> tracks; // 当前帧的所有 Track
};

} // namespace xrslam

#endif // XRSLAM_FRAME_H
