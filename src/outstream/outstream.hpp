#pragma once

#include <iostream>
#include <sstream>
#include <type_traits>
#include <span>
#include <array>

namespace piccante::outstream {


class CustomSink {
        public:
    virtual void write(const char* v, std::size_t s) = 0;
    virtual void flush() = 0;

    virtual ~CustomSink() = default;
};

template <typename CharT, typename Traits = std::char_traits<CharT>>
class StreamBuf : public std::basic_streambuf<CharT, Traits> {
        public:
    explicit StreamBuf(CustomSink& sink) noexcept : sink_(sink) {
        // No buffer allocation - we'll handle each character directly
    }

    CustomSink& sink() { return sink_; }

        protected:
    // Handle individual character output with no buffering
    typename Traits::int_type overflow(typename Traits::int_type ch) override {
        if (!Traits::eq_int_type(ch, Traits::eof())) {
            CharT c = Traits::to_char_type(ch);                            // NOLINT
            sink_.write(reinterpret_cast<const char*>(&c), sizeof(CharT)); // NOLINT
            return ch;
        }
        return Traits::eof();
    }

    // Optimize for writing chunks directly
    std::streamsize xsputn(const CharT* s, std::streamsize count) override {
        sink_.write(reinterpret_cast<const char*>(s), count * sizeof(CharT)); // NOLINT
        return count;
    }

    // Support for flushing
    int sync() override {
        sink_.flush();
        return 0;
    }

        private:
    CustomSink& sink_;
};

// Stream class using the separate streambuf
template <typename CharT, typename Traits = std::char_traits<CharT>>
class Stream : public std::basic_ostream<CharT, Traits> {
        private:
    StreamBuf<CharT, Traits> streambuf_;

        public:
    explicit Stream(CustomSink& sink)
        : std::basic_ostream<CharT, Traits>(nullptr), streambuf_(sink) {
        this->init(&streambuf_);
    }

    // Access to the underlying sink if needed
    CustomSink& sink() { return streambuf_.sink(); }
};

} // namespace outstream
