#include "../include/serial.hpp"
#include <cstring>

namespace serial_phoenix {
SerialCode Serial::open(std::string port, std::shared_ptr<SPconfig> config, size_t bytes_) {
    if (this->is_open()) {
        return SerialCode::Value::OPEN_OPENED;
    }
    if (std::filesystem::exists(port) == false) {
        return SerialCode::Value::OPEN_DEV_NOT_EXIST;
    }

    this->bytes_ = bytes_;
    this->read_buffer_.resize(this->bytes_);
    this->write_buffer_.resize(this->bytes_);

    if (config == nullptr) {
        config = std::make_shared<SPconfig>(
            uint32_t(115200),
            drivers::serial_driver::FlowControl::NONE,
            drivers::serial_driver::Parity::NONE,
            drivers::serial_driver::StopBits::ONE
        );
    }
    this->owned_ctx_ = std::make_unique<IoContext>(2);
    this->serial_port_ =
        std::make_shared<drivers::serial_driver::SerialPort>(*owned_ctx_, port, *config);
    this->serial_port_->open();

    if (this->serial_port_->is_open()) {
        return SerialCode::Value::OK;
    } else {
        return SerialCode::Value::OPEN_FAIL;
    }
}

bool Serial::is_open() const {
    if (this->serial_port_ && this->serial_port_->is_open()) {
        return true;
    } else {
        return false;
    }
}

bool Serial::SetByte(size_t bytes) {
    if (this->is_open()) {
        this->bytes_ = bytes;
        return true;
    } else {
        return false;
    }
}

SerialCode Serial::close() {
    if (this->serial_port_ && this->serial_port_->is_open()) {
        this->serial_port_->close();
        this->owned_ctx_.reset();
        this->bytes_ = 0;
        return !this->serial_port_->is_open() ? SerialCode::Value::OK
                                              : SerialCode::Value::CLOSE_FAIL;
    } else {
        return SerialCode::Value::CLOSE_NOT_OPENED;
    }
}
} // namespace serial_phoenix