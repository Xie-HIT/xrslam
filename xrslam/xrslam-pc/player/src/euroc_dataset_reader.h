#ifndef XRSLAM_PC_EUROC_DATASET_READER_H
#define XRSLAM_PC_EUROC_DATASET_READER_H

#include <dataset_reader.h>
#include <deque>
#include <string>

class EurocDatasetReader : public DatasetReader {
  public:
    EurocDatasetReader(const std::string &filename);
    NextDataType next() override;
    std::shared_ptr<xrslam::Image> read_image() override;
    std::pair<double, xrslam::vector<3>> read_gyroscope() override;
    std::pair<double, xrslam::vector<3>> read_accelerometer() override;

  private:
    std::deque<std::pair<double, NextDataType>> all_data; // 所有数据（图像、IMU）：为了按时间戳对所有数据排序
    std::deque<std::pair<double, xrslam::vector<3>>> gyroscope_data; // （时间戳，陀螺仪数据），按时间戳排序
    std::deque<std::pair<double, xrslam::vector<3>>> accelerometer_data; // （时间戳，加速度计数据），按时间戳排序
    std::deque<std::pair<double, std::string>> image_data; // （时间戳，图像路径），按时间戳排序
};

struct CameraCsv {
    struct CameraData {
        double t;
        std::string filename;
    };

    std::vector<CameraData> items;

    void load(const std::string &filename) {
        items.clear();
        FILE *csv = fopen(filename.c_str(), "r");
        if (csv) {
            char header_line[2048];
            fscanf(csv, "%2047[^\r]\r\n", header_line);
            char filename_buffer[2048];
            CameraData item;
            while (!feof(csv)) {
                memset(filename_buffer, 0, 2048);
                if (fscanf(csv, "%lf,%2047[^\r]\r\n", &item.t,
                           filename_buffer) != 2) {
                    break;
                }
                item.t *= 1e-9;
                item.filename = std::string(filename_buffer);
                items.emplace_back(std::move(item));
            }
            fclose(csv);
        }
    }

    void save(const std::string &filename) const {
        if (FILE *csv = fopen(filename.c_str(), "w")) {
            fputs("#t[ns],filename[string]\n", csv);
            for (auto item : items) {
                fprintf(csv, "%lld,%s\n", int64_t(item.t * 1e9),
                        item.filename.c_str());
            }
            fclose(csv);
        }
    }
};

struct ImuCsv {
    struct ImuData {
        double t;
        struct {
            double x;
            double y;
            double z;
        } w;
        struct {
            double x;
            double y;
            double z;
        } a;
    };

    std::vector<ImuData> items;

    void load(const std::string &filename) {
        items.clear();
        FILE *csv = fopen(filename.c_str(), "r");
        if (csv) {
            char header_line[2048];
            fscanf(csv, "%2047[^\r]\r\n", header_line);
            ImuData item;
            while (!feof(csv) &&
                   fscanf(csv, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\r\n", &item.t,
                          &item.w.x, &item.w.y, &item.w.z, &item.a.x, &item.a.y,
                          &item.a.z) == 7) {
                item.t *= 1e-9;
                items.emplace_back(std::move(item));
            }
            fclose(csv);
        }
    }

    void save(const std::string &filename) const {
        if (FILE *csv = fopen(filename.c_str(), "w")) {
            fputs("#t[ns],w.x[rad/s:double],w.y[rad/s:double],w.z[rad/"
                  "s:double],a.x[m/s^2:double],a.y[m/s^2:double],a.z[m/"
                  "s^2:double]\n",
                  csv);
            for (auto item : items) {
                fprintf(csv, "%lld,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e\n",
                        int64_t(item.t * 1e9), item.w.x, item.w.y, item.w.z,
                        item.a.x, item.a.y, item.a.z);
            }
            fclose(csv);
        }
    }
};

#endif
