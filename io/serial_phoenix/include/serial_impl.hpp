template<typename T>
inline SerialCode Serial::read(T& out) {
    static_assert(std::is_trivially_copyable_v<T>, "out type must be TriviallyCopyable for memcpy");
    std::lock_guard<std::mutex> read_guard(this->read_mutex_);

    if (!this->is_open()) {
        return SerialCode::Value::READ_NOT_OPENED;
    }
    try {
        this->read_buffer_.resize(sizeof(T));
        this->bytes_ = this->serial_port_->receive(this->read_buffer_);
    } catch (const std::exception &e) {
        return SerialCode::Value::READ_FAIL;
    }

    if (this->bytes_ != sizeof(T)) {
        return SerialCode::Value::READ_BYTE_MISMATCH;
    }

    std::memcpy(&out, this->read_buffer_.data(), sizeof(T));
    return SerialCode::Value::OK;
}

template<typename T>
inline SerialCode Serial::read_unsafe(T& out) {
    static_assert(std::is_trivially_copyable_v<T>, "out type must be TriviallyCopyable for memcpy");

    if (!this->is_open()) {
        return SerialCode::Value::READ_NOT_OPENED;
    }
    try {
        this->read_buffer_.resize(sizeof(T));
        this->bytes_ = this->serial_port_->receive(this->read_buffer_);
    } catch (const std::exception &e) {
        return SerialCode::Value::READ_FAIL;
    }

    if (this->bytes_ != sizeof(T)) {
        return SerialCode::Value::READ_BYTE_MISMATCH;
    }

    std::memcpy(&out, this->read_buffer_.data(), sizeof(T));
    return SerialCode::Value::OK;
}

template<typename T>
inline SerialCode Serial::write(T&& in) {
    static_assert(std::is_trivially_copyable_v<T>, "T must be TriviallyCopyable for wire send");
    std::lock_guard<std::mutex> write_guard(this->write_mutex_);

    if (!this->is_open()) {
        return SerialCode::Value::WRITE_NOT_OPENED;
    }
    try {
        this->write_buffer_.resize(sizeof(T));
        std::memcpy(this->write_buffer_.data(), &in, sizeof(T));
        this->serial_port_->send(this->write_buffer_);
    } catch (const std::exception &e) {
        return SerialCode::Value::WRITE_FAIL;
    }
    return SerialCode::Value::OK;
}

template<typename T>
inline SerialCode Serial::write_unsafe(T&& in) {
    static_assert(std::is_trivially_copyable_v<T>, "T must be TriviallyCopyable for wire send");

    if (!this->is_open()) {
        return SerialCode::Value::WRITE_NOT_OPENED;
    }
    try {
        this->write_buffer_.resize(sizeof(T));
        std::memcpy(this->write_buffer_.data(), &in, sizeof(T));
        this->serial_port_->send(this->write_buffer_);
    } catch (const std::exception &e) {
        return SerialCode::Value::WRITE_FAIL;
    }
    return SerialCode::Value::OK;
}