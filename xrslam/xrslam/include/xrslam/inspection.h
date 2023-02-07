#ifndef XRSLAM_INSPECTOR_H
#define XRSLAM_INSPECTOR_H

#include <any>
#include <mutex>
#include <xrslam/xrslam.h>

namespace xrslam {

using point2i = vector<2, false, int>;
using color3b = vector<3, false, unsigned char>;

struct KeyframeState {
    double t;
    quaternion q;
    vector<3> p;
    vector<3> v;
    vector<3> bg;
    vector<3> ba;
};

struct Landmark {
    vector<3> p;
    bool triangulated;
};

class InspectPainter {
  public:
    virtual ~InspectPainter() = default;
    virtual void set_image(const Image *image) = 0;
    virtual void point(const point2i &p, const color3b &c, int size = 1,
                       int style = 0) {}
    virtual void line(const point2i &p1, const point2i &p2, const color3b &c,
                      int thickness = 1) {}
};

class InspectionSupport {
    struct VersionTag;

  public:
    enum InspectItem {
        RESERVED = 0, // 保留位置：记录版本号
        input_data_fps,
        input_real_fps,
        input_output_lag,
        feature_tracker_angle_misalignment,
        feature_tracker_painter,
        sliding_window_track_painter,
        sliding_window_reprojection_painter,
        sliding_window_landmarks,  // 优化后的滑窗内 3D 路标点
        sliding_window_current_bg, // 优化后的滑窗内最后一帧的 bg
        sliding_window_current_ba, // 优化后的滑窗内最后一帧的 ba
        feature_tracker_time,
        bundle_adjustor_solve_time,
        bundle_adjustor_marginalization_time,
        ITEM_COUNT  // 这个是以上枚举值的总个数
    };

    InspectionSupport(const VersionTag &tag);
    ~InspectionSupport();

    // 获得 support() 静态函数中的静态变量 s_support.storage 的相应选项
    // 每个选项都包含一个内容信息、一个锁
    static std::pair<std::any &, std::unique_lock<std::mutex>>
    get(InspectItem item);

  private:
    static InspectionSupport &support(); // 会产生一个和具体版本号关联的 InspectionSupport 静态对象 s_support
    std::vector<std::pair<std::any, std::mutex>> storage;
};

} // namespace xrslam

#if defined(XRSLAM_ENABLE_DEBUG_INSPECTION)
#define inspect_debug(item, var)                                               \
    if constexpr (auto [var, var##_lock] = ::xrslam::InspectionSupport::get(   \
                      ::xrslam::InspectionSupport::item);                      \
                  true)
#else
#define inspect_debug(item, var) if constexpr (std::any var; false)
#endif

#define inspect(item, var)                                                     \
    if constexpr (auto [var, var##_lock] = ::xrslam::InspectionSupport::get(   \
                      ::xrslam::InspectionSupport::item);                      \
                  true)

#endif // XRSLAM_INSPECTOR_H
