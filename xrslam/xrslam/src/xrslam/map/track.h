#ifndef XRSLAM_TRACK_H
#define XRSLAM_TRACK_H

#include <xrslam/common.h>
#include <xrslam/estimation/state.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/utility/identifiable.h>
#include <xrslam/utility/tag.h>

namespace xrslam {

enum TrackTag { TT_VALID = 0, TT_TRIANGULATED, TT_FIX_INVD };

class Track : public Tagged<TrackTag>, public Identifiable<Track> {
    friend class Map;
    size_t map_index; // 此 Track 在 Map 中的索引
    Map *map;
    Track();

  public:
    Track(const Map::construct_by_map_t &) : Track() {}

    virtual ~Track();

    size_t keypoint_num() const { return keypoint_refs.size(); }

    std::pair<Frame *, size_t> first_keypoint() const {
        return *keypoint_refs.begin();
    }

    std::pair<Frame *, size_t> last_keypoint() const {
        return *keypoint_refs.rbegin();
    }

    Frame *first_frame() const { return keypoint_refs.begin()->first; }

    Frame *last_frame() const { return keypoint_refs.rbegin()->first; }

    const std::map<Frame *, size_t, compare<Frame *>> &keypoint_map() const {
        return keypoint_refs;
    }

    // 判别 frame 是否跟踪上了此 Track 对应的路标点
    bool has_keypoint(Frame *frame) const {
        return keypoint_refs.count(frame) > 0;
    }

    size_t get_keypoint_index(Frame *frame) const {
        if (has_keypoint(frame)) {
            return keypoint_refs.at(frame);
        } else {
            return nil();
        }
    }

    const vector<3> &get_keypoint(Frame *frame) const;
    void add_keypoint(Frame *frame, size_t keypoint_index); // 添加 Track
    void remove_keypoint(Frame *frame, bool suicide_if_empty = true); // 移除 Track

    std::optional<vector<3>> triangulate() const; // 基于所有的 Track 信息进行三角化
    double triangulation_angle(const vector<3> &p) const; // 返回最大的三角化视差角

    vector<3> get_landmark_point() const; // 基于主帧中的逆深度和主帧位姿，反投影出路标点在世界坐标系下的位置
    void set_landmark_point(const vector<3> &p); // 设置主帧中的你逆深度

    std::unique_lock<std::mutex> lock() const { return map->lock(); }

    LandmarkState landmark; // 路标点状态：包含逆深度

  private:
    // （帧，帧中的角点索引），map 的长度为 Track 的长度
    // map 内部按照 Frame 的 id 进行升序排序
    std::map<Frame *, size_t, compare<Frame *>> keypoint_refs;
};

} // namespace xrslam

#endif // XRSLAM_TRACK_H
