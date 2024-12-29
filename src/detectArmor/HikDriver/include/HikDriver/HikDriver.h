/**
 * @projectName IntelligentHanding
 * @file HikDriver.h
 * @brief 海康相机驱动
 *
 * @author yx
 * @date 2023-07-15 20:12
 */

#ifndef HIKDRIVER_HIKDRIVER_H
#define HIKDRIVER_HIKDRIVER_H

#include <cstring>
#include <unistd.h>//包含unix系统调用函数

#include <string>
#include <map>
#include <thread>
#include <shared_mutex>

#include <MvCameraControl.h>
#include <opencv2/opencv.hpp>

//枚举类 定义多种触发源类型 表示触发相机拍照的方式
enum class TriggerSource {
    Line0,
    Line1,
    Line2,
    Line3,
    Counter,
    Software = 7,
    FrequencyConverter
};
//储存相机的基本信息
struct DriverInfo {
    std::string name;//名称
    std::string type;//类型
    std::string ip;//ip地址
};

class HikDriver;

//从海康相机获取的一帧图像数据
class HikFrame {
    friend class HikDriver;
private:
    cv::Mat m_rgb_frame = {};//存储图像数据
    int64_t m_timestamp = 0;  // 时间戳 毫秒
public:
    //默认构造函数
    HikFrame() =default;

    //构造函数 接受：Mat int64_t两个参数
    HikFrame(const cv::Mat& frame, const int64_t& timestamp)
            : m_rgb_frame(frame.clone()),
              m_timestamp(timestamp) {}

    //拷贝构造
    HikFrame(const HikFrame& other)
            : m_timestamp(other.m_timestamp),
              m_rgb_frame(other.m_rgb_frame.clone()) {}

    //移动构造
    HikFrame(HikFrame&& other) noexcept
            : m_rgb_frame(std::move(other.m_rgb_frame)),
              m_timestamp(other.m_timestamp) {}

    //析构函数
    ~HikFrame() =default;

    //拷贝赋值操作符
    HikFrame& operator=(const HikFrame& other) {
        if (this != &other) {
            m_timestamp = other.m_timestamp;
            m_rgb_frame = other.m_rgb_frame.clone();
        }
        return *this;
    }

    //移动赋值操作符
    HikFrame& operator=(HikFrame&& other) noexcept {
        if (this != &other) {
            m_timestamp = other.m_timestamp;
            m_rgb_frame = std::move(other.m_rgb_frame);
        }
        return *this;
    }

    // [[nodiscard]]c++17属性 表示返回值不可忽略
    //inline（内联函数）表示小的频繁调用的函数
    //该函数返回一个指向m_rgb_frame的指针
    [[nodiscard]] inline const cv::Mat* getRgbFrame() const { return &m_rgb_frame; }
    //const表示该函数不会修改类的任何成员变量 保证他是一个只读操作 函数返回一个时间戳 表示图像的捕获时间
    [[nodiscard]] inline int64_t getTimestamp() const { return m_timestamp; }
    //判断图像数据是否有效
    bool empty() const { return m_rgb_frame.empty(); }
};

class HikDriver {
public:
    template<typename T>
    //模版结构体 意味着ParamInfo可以存储任意类型的数据
    struct ParamInfo {
        T current;//当前参数
        T min;//参数的最小值
        T max;//参数的最大值
        //构造函数 接受一个MVCC_FLOATVALUE类型的引用p作为参数 初始化成员变量
        //explicit该函数不能隐式调用 必须显示的指定
        explicit ParamInfo(MVCC_FLOATVALUE & p) {
            current = p.fCurValue;
            min = p.fMin;
            max = p.fMax;
        }
        //const 表示该函数不会改变成员变量的值和状态 因此可以安全的在常亮对象上调用
        [[nodiscard]] std::string toString() const {
            std::string info;
            info = "current: " + std::to_string(current) +
                    "\nmin: " + std::to_string(min) +
                    "\nmax: " + std::to_string(max);
            return info;
        }
    };

    /**
     * @brief 默认: 索引为0, 非触发式取流
     */
    HikDriver();
    /**
     * @param index 相机索引, 非触发模式
     */
    explicit HikDriver(int index);
    /**
     * @brief 触发式取流
     * @param index 相机索引
     * @param trigger 触发源
     */
    //构造函数 初始化HikDriver对象
    HikDriver(int index, const TriggerSource& trigger);
    //析构函数 销毁HikDriver对象
    ~HikDriver();

    /**
     * @brief 获取相机的曝光、增益、曝光模式、增益模式参数
     * @return 参数组成的字符串
     */
    [[nodiscard]] std::string getParamInfo() const;

    /**
     * @brief 显示曝光、增益、曝光模式、增益模式参数
     */
    void showParamInfo() const;

    /**
     * @brief 枚举所有设备
     * @param mode Gige | USB
     * @return
     */
    bool findDriver(int mode= MV_GIGE_DEVICE | MV_USB_DEVICE);

    /**
     * @brief 重新加载帧信息
     */
    void reloadFrameInfo();

    /**
     * @brief 连接相机. 打开相机，并初始化帧信息
     * @param index 索引
     * @param trigger 触发源 如果使用该参数则为触发式
     * @return success | failed
     */
    bool connectDriver(const int& index);
    //TriggerSource 枚举类型定义相机触发操作的不同来源
    bool connectDriver(const int& index, const TriggerSource& trigger);

    /**
     * @brief 取流线程(只在非触发模式下可用)
     */
    void startReadThread();
    inline void stopReadThread() { m_read_thread_running = false; }
    inline bool readThreadIsRunning() const { return m_read_thread_running; }

    /**
     * @brief 从取流线程中获取一帧数据(必须开启取流线程)
     * @see HikDriver::startReadThread()
     */
    HikFrame getFrame() const;
    void getFrame(HikFrame& frame);
    /**
     * @brief 获取一帧数据(非取流线程， 该函数在对象所在线程运行)
     * @param frame
     * @param method 0: ; 1: 更高效的取流方式
     */
    void getFrame(HikFrame& frame, const char& method);

    /**
     * @brief 触发式取流
     * @param frame[out] 获取的图像
     */
    void triggerImageData(HikFrame& frame);

    void setTriggerMode(bool mode);
    std::string getTriggerMode() const;

    std::string getTriggerSources() const;
    void setTriggerSource(const TriggerSource& trigger);

    float getTriggerDelay() const;
    void setTriggerDelay(const float& delay_us);

    /**
     * @brief 设置自动曝光时间的范围
     * @param lower
     * @param upper
     */
    void setExposureTimeRange(int lower, int upper);

    /**
     * @brief 设置自动增益时间的范围
     * @param lower
     * @param upper
     */
    void setGainRange(int lower, int upper);

    /**
     * @brief 设置自动曝光、自动增益
     */
    void setAuto(int brightness=255);

    /**
     * @brief 自动曝光 type
     */
     void setAutoExposureTime(int mode);
    [[nodiscard]] std::string getAutoExposureTime() const;

    /**
     * 自动增益
     * @param mode 0 | 1 => OFF | ON
     */
     void setAutoGain(int mode);
    [[nodiscard]] std::string getAutoGain() const;

    /**
     * @brief 设置曝光时间(自动将自动曝光关闭)
     * @param timems
     */
    void setExposureTime(float timems);
    [[nodiscard]] ParamInfo<float> getExposureTime() const;

    /**
     * @brief 设置增益值(自动将自动增益关闭)
     * @param val
     */
    void setGain(float val);
    [[nodiscard]] HikDriver::ParamInfo<float> getGain() const;

    [[nodiscard]] unsigned int getDadaSize() const { return m_data_size; }

    [[nodiscard]] bool isConnected() const { return m_opened_status && m_initialized_frame_info; }

    /**
     * @brief 保存/加载配置
     * @param user_file 用户文件
     * @param device_file 设备文件
     */
    void saveAccess(const std::string& user_file, const std::string& device_file);

    void loadAccess(const std::string& user_file, const std::string& device_file);

    DriverInfo getDriverInfo(const int& index) const;

private:
    int m_index = 0;
    bool m_is_trigger = false;  // 是否是触发式
    TriggerSource m_trigger_source = TriggerSource::Software; // 触发源
    void* m_handle = nullptr; // 设备句柄
    static const std::map<unsigned int, std::string> m_ErrorMess;
    MV_CC_DEVICE_INFO_LIST m_devices;  // 所有设备
    bool m_opened_status = false;  // 打开状态
    bool m_initialized_frame_info = false;  // 是否已经初始化完成信息
    MVCC_INTVALUE m_frame_size_info = {};  // 帧大小信息
    unsigned int m_data_size = 0;  // 帧大小
    std::thread m_read_thread;
    bool m_read_thread_running = false;
    HikFrame m_hik_frame;
    mutable std::shared_mutex m_frame_mutex;

    bool connectDriver();

    /**
     * 打开设备
     * @param index 索引
     * @return
     */
    bool openDriver(int index=0);

    /**
     * 初始化帧信息
     * @return
     */
    bool initFrameInfo();

    void readImageFunction();

    /**
    * @brief 读取一帧图像(主动取流)
    * @param data_buffer
    * @param frame_info
    * @param timeout_ms
    * @return
    */
    bool readImageData(unsigned char* data_buffer,
                       MV_FRAME_OUT_INFO_EX& frame_info,
                       const unsigned int& timeout_ms=1000);
    /**
     * @brief 读取一帧图像(主动取流)(更高效)
     * @param frame
     * @param timeout_ms
     * @return
     */
    bool readImageData(MV_FRAME_OUT& frame, const unsigned int& timeout_ms=1000);

    static bool checkErrorCode(const unsigned int& code, const std::string& mess="");
};

std::string to_string(const DriverInfo& info);
#endif //HIKDRIVER_HIKDRIVER_H
