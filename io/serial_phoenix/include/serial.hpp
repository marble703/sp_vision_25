#pragma once

#include <filesystem>

#include <serial_driver/serial_driver.hpp>

namespace serial_phoenix {

using SPconfig = drivers::serial_driver::SerialPortConfig;

struct SerialCode {
    // 串口操作返回码
    enum class Value : int {
        NONE    = 0, // 未知状态
        OK      = 1, // 操作成功
        ILLEGAL = 2, // 非法操作

        OPEN_FAIL          = 10, // 打开出现未知失败
        OPEN_OPENED        = 11, // 设备已打开
        OPEN_DEV_NOT_EXIST = 12, // 设备不存在

        CLOSE_FAIL       = 20, // 关闭失败
        CLOSE_NOT_OPENED = 21, // 关闭时设备未打开
        CLOSE_READING    = 22, // 关闭时正在读取
        CLOSE_WRITING    = 23, // 关闭时正在写入

        READ_FAIL          = 30, // 读取失败
        READ_NOT_OPENED    = 31, // 读取时设备未打开
        READ_BYTE_MISMATCH = 32, // 读取字节数不匹配
        READ_BUSY          = 33, // 读取时设备忙

        WRITE_FAIL          = 40, // 写入失败
        WRITE_NOT_OPENED    = 41, // 写入时设备未打开
        WRITE_BYTE_MISMATCH = 43, // 写入字节数不匹配
        WRITE_BUSY          = 42, // 写入时设备忙
    };
    Value value { Value::NONE };

    // 构造 & 隐式转换自 Value
    constexpr SerialCode() = default;
    constexpr SerialCode(Value v) noexcept: value(v) {}

    // 转换为 bool，OK 为 true，其他为 false
    constexpr explicit operator bool() const noexcept {
        return value == Value::OK;
    }

    // 取反运算符
    constexpr bool operator!() const noexcept {
        return value != Value::OK;
    }

    // 获取错误码
    constexpr Value code() const noexcept {
        return value;
    }
};

/**
 * @brief 串口通信类
 * @note 用于与管理串口设备并进行通信
 * 
 */
class Serial {
public:
    Serial()  = default;
    ~Serial() = default;

    // 禁用拷贝构造与赋值
    Serial(const Serial&)            = delete;
    Serial& operator=(const Serial&) = delete;

    // 启用移动构造与赋值(未测试)
    Serial(Serial&& other) noexcept:
        owned_ctx_(std::move(other.owned_ctx_)),
        serial_port_(std::move(other.serial_port_)),
        read_buffer_(std::move(other.read_buffer_)),
        write_buffer_(std::move(other.write_buffer_)),
        bytes_(other.bytes_) {}

    Serial& operator=(Serial&& other) noexcept {
        if (this != &other) {
            owned_ctx_    = std::move(other.owned_ctx_);
            serial_port_  = std::move(other.serial_port_);
            read_buffer_  = std::move(other.read_buffer_);
            write_buffer_ = std::move(other.write_buffer_);
            bytes_        = other.bytes_;
        }
        return *this;
    }

    /**
     * @brief 打开串口
     * 
     * @param port 串口地址
     * @param config 串口配置
     * @param bytes_ 每包字节数
     * @return true 打开成功
     * @return false 打开失败
     */
    SerialCode open(
        std::string port                 = "/dev/ttyACM0",
        std::shared_ptr<SPconfig> config = nullptr,
        size_t bytes_                    = 32
    );

    /**
     * @brief 检查串口是否已打开
     * 
     * @return true 已打开
     * @return false 未打开
     */
    bool is_open() const;

    /**
     * @param bytes 每包字节数
     * 
     * @return true 设置成功
     * @return false 设置失败（串口未打开）
     */
    bool SetByte(size_t bytes);

    /**
     * @brief 关闭串口
     * 
     * @return SerialCode 串口操作返回码
     */
    SerialCode close();

    /**
     * @brief 读取数据
     * @note 该函数是线程安全的，会加锁
     * 
     * @tparam T 输出数据类型, 由于使用 memcpy 进行数据拷贝, T 需要是标准布局类型
     * @param out 输出数据引用
     * @return SerialCode 串口操作返回码
     */
    template<typename T>
    SerialCode read(T& out);

    /**
     * @brief 读取数据
     * @note 该函数不是线程安全的，不加锁
     * @note 用户需要自行确保线程安全, 并避免与 read() 函数混用
     * 
     * @tparam T 输出数据类型, 由于使用 memcpy 进行数据拷贝, T 需要是标准布局类型
     * @param out 输出数据引用
     * @return SerialCode 串口操作返回码
     */
    template<typename T>
    SerialCode read_unsafe(T& out);

    /**
     * @brief 写入数据
     * @note 该函数是线程安全的，会加锁
     * 
     * @tparam T 输入数据类型, 由于使用 memcpy 进行数据拷贝, T 需要是标准布局类型
     * @param in 输入数据引用
     * @return SerialCode 串口操作返回码
     */
    template<typename T>
    SerialCode write(T&& in);

    /**
     * @brief 写入数据
     * @note 该函数不是线程安全的，不加锁
     * @note 用户需要自行确保线程安全, 并避免与 write() 函数混用
     * 
     * @tparam T 输入数据类型, 由于使用 memcpy 进行数据拷贝, T 需要是标准布局类型
     * @param in 输入数据引用
     * @return SerialCode 串口操作返回码
     */
    template<typename T>
    SerialCode write_unsafe(T&& in);

private:
    // 串口对象
    std::unique_ptr<IoContext> owned_ctx_;                            // 控制异步io线程数
    std::shared_ptr<drivers::serial_driver::SerialPort> serial_port_; // 串口

    // 缓冲区
    std::vector<uint8_t> read_buffer_;  // 读缓冲区
    std::vector<uint8_t> write_buffer_; // 写缓冲区
    std::mutex read_mutex_;             // 读锁
    std::mutex write_mutex_;            // 写锁

    // 校验
    size_t bytes_ { 0 }; // 包字节数
};

#include "serial_impl.hpp"
} // namespace serial_phoenix