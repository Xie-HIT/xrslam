#include <xrslam/utility/worker.h>

namespace xrslam {

void Worker::worker_loop() {
    while (worker_running) {
        auto l = lock();

        // 队列里没有帧的时候不执行
#if defined(XRSLAM_ENABLE_THREADING)
        if (worker_running && empty()) {
            worker_cv.wait(l, [this] { return !worker_running || !empty(); });
        }
#else
        if (empty())
            break;
#endif
        if (!worker_running)
            break;
        work(l);
    }
}

} // namespace xrslam
