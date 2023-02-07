#ifndef XRSLAM_FRONTEND_WORKER_H
#define XRSLAM_FRONTEND_WORKER_H
#include <xrslam/common.h>
#include <xrslam/estimation/state.h>
#include <xrslam/utility/worker.h>

namespace xrslam {

class Config;
class Frame;
class Initializer;
class SlidingWindowTracker;
class Localizer;

class FrontendWorker : public Worker {
  public:
    FrontendWorker(XRSLAM::Detail *detail, std::shared_ptr<Config> config);
    ~FrontendWorker();

    bool empty() const override;
    void work(std::unique_lock<std::mutex> &l) override;

    void issue_frame(Frame *frame); // 从 FeatureTracker 发送（即拷贝）帧到 FrontendWorker 中

    // 以下方法都与 ARDemo 有关
    size_t create_virtual_object();
    OutputObject get_virtual_object_pose_by_id(size_t id);

    std::tuple<double, size_t, PoseState, MotionState> get_latest_state() const;
    SysState get_system_state() const;

    bool global_localization_state() const;
    void set_global_localization_state(bool state);
    void query_frame();

    std::unique_ptr<Localizer> localizer;

  private:
    std::deque<size_t> pending_frame_ids; // 图像帧序列

    XRSLAM::Detail *detail;
    std::shared_ptr<Config> config;
    std::unique_ptr<Initializer> initializer;
    std::unique_ptr<SlidingWindowTracker> sliding_window_tracker;  // 含有初始化时候的 Map

    std::tuple<double, size_t, PoseState, MotionState> latest_state;
    mutable std::mutex latest_state_mutex;

    bool global_localization_flag = false;
};

} // namespace xrslam

#endif // XRSLAM_FRONTEND_WORKER_H
