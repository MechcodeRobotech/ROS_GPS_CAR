/**
 * Fake HardwareSerial for the native GPS parser tests: a byte queue you can
 * push canned NMEA sentences into.
 */
#ifndef FAKE_SERIAL_H
#define FAKE_SERIAL_H

#include <cstring>
#include <string>

#define SERIAL_8N1 0x800001c

class FakeSerial {
public:
    void begin(unsigned long baud, int = SERIAL_8N1, int = -1, int = -1) { baud_ = baud; }
    void end() {}
    void setRxBufferSize(int) {}

    int available() { return (int)(buffer_.size() - read_pos_); }
    int read() {
        if (read_pos_ >= buffer_.size()) return -1;
        return (unsigned char)buffer_[read_pos_++];
    }

    /** Test helper: queue raw characters as if the GPS had sent them. */
    void push(const char *text) { buffer_ += text; }
    void clear() { buffer_.clear(); read_pos_ = 0; }
    unsigned long baud() const { return baud_; }

private:
    std::string buffer_;
    size_t read_pos_ = 0;
    unsigned long baud_ = 0;
};

#endif // FAKE_SERIAL_H
