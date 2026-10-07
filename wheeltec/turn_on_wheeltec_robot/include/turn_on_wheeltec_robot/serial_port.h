#ifndef __TURN_ON_WHEELTEC_ROBOT_SERIAL_PORT_H_
#define __TURN_ON_WHEELTEC_ROBOT_SERIAL_PORT_H_

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include <chrono>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace wheeltec {

// Exception thrown by Serial on I/O errors. Mirrors the limited use of
// serial::IOException in the original wjwwood/serial dependency.
class IOException : public std::runtime_error {
public:
    explicit IOException(const std::string &msg) : std::runtime_error(msg) {}
};

// Minimal timeout configuration compatible with serial::Timeout.
struct Timeout {
    uint32_t inter_byte_timeout_ms = 0;
    uint32_t read_timeout_constant_ms = 0;
    uint32_t read_timeout_multiplier_ms = 0;
    uint32_t write_timeout_constant_ms = 0;
    uint32_t write_timeout_multiplier_ms = 0;

    static Timeout simpleTimeout(uint32_t timeout_ms) {
        Timeout t;
        t.read_timeout_constant_ms = timeout_ms;
        t.write_timeout_constant_ms = timeout_ms;
        return t;
    }
};

// Minimal POSIX-based serial port implementation. Provides a drop-in subset of
// the wjwwood/serial API used by this package (setPort, setBaudrate,
// setTimeout, open, isOpen, close, read, write).
class Serial {
public:
    Serial() = default;
    ~Serial() { close(); }

    Serial(const Serial &) = delete;
    Serial &operator=(const Serial &) = delete;

    void setPort(const std::string &port) { port_ = port; }
    void setBaudrate(uint32_t baudrate) { baudrate_ = baudrate; }
    void setTimeout(const Timeout &timeout) { timeout_ = timeout; }

    void open() {
        if (isOpen()) {
            return;
        }
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            throw IOException("Failed to open serial port: " + port_);
        }
        // Switch back to blocking reads; we drive timeouts with select().
        int flags = fcntl(fd_, F_GETFL, 0);
        if (flags >= 0) {
            (void)fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
        }
        configureTermios();
    }

    bool isOpen() const { return fd_ >= 0; }

    void close() {
        if (fd_ >= 0) {
            (void)::close(fd_);
            fd_ = -1;
        }
    }

    // Writes up to `size` bytes. Returns the number of bytes written.
    size_t write(const uint8_t *data, size_t size) {
        if (!isOpen()) {
            throw IOException("Serial port not open");
        }
        size_t total = 0;
        auto start = std::chrono::steady_clock::now();
        uint32_t timeout_ms = timeout_.write_timeout_constant_ms;
        while (total < size) {
            ssize_t n = ::write(fd_, data + total, size - total);
            if (n < 0) {
                if (errno == EINTR) {
                    continue;
                }
                throw IOException("write() failed on serial port");
            }
            if (n == 0) {
                break;
            }
            total += static_cast<size_t>(n);
            if (timeout_ms == 0) {
                break;
            }
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start);
            if (elapsed.count() >= timeout_ms) {
                break;
            }
        }
        (void)tcdrain(fd_);
        return total;
    }

    // Reads up to `size` bytes within the configured read timeout. Blocks
    // until either the buffer is full, the timeout expires, or an error
    // occurs. Returns the number of bytes actually read.
    size_t read(uint8_t *buffer, size_t size) {
        if (!isOpen()) {
            throw IOException("Serial port not open");
        }
        size_t total = 0;
        auto start = std::chrono::steady_clock::now();
        uint32_t timeout_ms = timeout_.read_timeout_constant_ms;
        while (total < size) {
            if (timeout_ms != 0) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start);
                long remaining = static_cast<long>(timeout_ms) -
                                 static_cast<long>(elapsed.count());
                if (remaining <= 0) {
                    break;
                }
                struct timeval tv;
                tv.tv_sec = remaining / 1000;
                tv.tv_usec = (remaining % 1000) * 1000;
                fd_set fds;
                FD_ZERO(&fds);
                FD_SET(fd_, &fds);
                int ret = ::select(fd_ + 1, &fds, nullptr, nullptr, &tv);
                if (ret < 0) {
                    if (errno == EINTR) {
                        continue;
                    }
                    throw IOException("select() failed on serial port");
                }
                if (ret == 0) {
                    break;
                }
            }
            ssize_t n = ::read(fd_, buffer + total, size - total);
            if (n < 0) {
                if (errno == EINTR) {
                    continue;
                }
                throw IOException("read() failed on serial port");
            }
            if (n == 0) {
                break;
            }
            total += static_cast<size_t>(n);
        }
        return total;
    }

private:
    void configureTermios() {
        struct termios options;
        if (tcgetattr(fd_, &options) != 0) {
            throw IOException("tcgetattr() failed");
        }

        speed_t baud = baudToTermios(baudrate_);
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        // 8N1 mode
        options.c_cflag &= static_cast<unsigned int>(~(PARENB | CSTOPB | CSIZE));
        options.c_cflag |= CS8;
        options.c_cflag |= (CLOCAL | CREAD);

        // Disable hardware flow control
        options.c_cflag &= static_cast<unsigned int>(~CRTSCTS);

        // Raw input mode
        options.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO | ECHOE | ISIG));

        // Disable software flow control and special input handling
        options.c_iflag &=
            static_cast<unsigned int>(~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR));

        // Raw output mode
        options.c_oflag &= static_cast<unsigned int>(~OPOST);

        // Non-blocking read with 0.1s character timeout as a fallback if
        // select() is bypassed.
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 1;

        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            throw IOException("tcsetattr() failed");
        }
        (void)tcflush(fd_, TCIOFLUSH);
    }

    static speed_t baudToTermios(uint32_t baudrate) {
        switch (baudrate) {
            case 0: return B0;
            case 50: return B50;
            case 75: return B75;
            case 110: return B110;
            case 134: return B134;
            case 150: return B150;
            case 200: return B200;
            case 300: return B300;
            case 600: return B600;
            case 1200: return B1200;
            case 1800: return B1800;
            case 2400: return B2400;
            case 4800: return B4800;
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            case 500000: return B500000;
            case 576000: return B576000;
            case 921600: return B921600;
            case 1000000: return B1000000;
            default: return B115200;
        }
    }

    std::string port_;
    uint32_t baudrate_ = 115200;
    Timeout timeout_;
    int fd_ = -1;
};

}  // namespace wheeltec

#endif  // __TURN_ON_WHEELTEC_ROBOT_SERIAL_PORT_H_
