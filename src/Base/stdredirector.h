#ifndef STDREDIRECTOR_H
#define STDREDIRECTOR_H

#include <iostream>
#include <streambuf>
#include <string>
#include <memory>
#include <mutex>
#include <glog/logging.h>

class GlogStreambuf : public std::streambuf {
public:
    enum LogLevel {
        INFO_LEVEL,
        WARNING_LEVEL,
        ERROR_LEVEL
    };

    explicit GlogStreambuf(LogLevel level) : log_level_(level) {}

    virtual ~GlogStreambuf() {
        sync();
    }

protected:
    virtual int overflow(int c) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (c != EOF) {
            buffer_ += static_cast<char>(c);
            if (c == '\n') {
                flush_buffer_unlocked();
            }
        }
        return c;
    }

    virtual int sync() override {
        std::lock_guard<std::mutex> lock(mutex_);
        flush_buffer_unlocked();
        return 0;
    }

    virtual std::streamsize xsputn(const char* s, std::streamsize n) override {
        std::lock_guard<std::mutex> lock(mutex_);

        for (std::streamsize i = 0; i < n; ++i) {
            buffer_ += s[i];
            if (s[i] == '\n') {
                flush_buffer_unlocked();
            }
        }
        return n;
    }

private:
    void flush_buffer_unlocked() {
        if (!buffer_.empty()) {
            std::string log_content = buffer_;
            if (log_content.back() == '\n') {
                log_content.pop_back();
            }

            if (!log_content.empty()) {
                switch (log_level_) {
                case INFO_LEVEL:
                    LOG(INFO) << log_content;
                    break;
                case WARNING_LEVEL:
                    LOG(WARNING) << log_content;
                    break;
                case ERROR_LEVEL:
                    LOG(ERROR) << log_content;
                    break;
                }
            }
            buffer_.clear();
        }
    }

    std::string buffer_;
    LogLevel log_level_;
    std::mutex mutex_;
};

class StdRedirector {
public:
    StdRedirector()
        : cout_buf_(std::make_unique<GlogStreambuf>(GlogStreambuf::INFO_LEVEL)),
        cerr_buf_(std::make_unique<GlogStreambuf>(GlogStreambuf::ERROR_LEVEL)),
        clog_buf_(std::make_unique<GlogStreambuf>(GlogStreambuf::WARNING_LEVEL)),
        original_cout_buf_(nullptr),
        original_cerr_buf_(nullptr),
        original_clog_buf_(nullptr),
        redirected_(false) {
    }

    ~StdRedirector() {
        restore();
    }

    StdRedirector(const StdRedirector&) = delete;
    StdRedirector& operator=(const StdRedirector&) = delete;

    void redirect() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!redirected_) {
            original_cout_buf_ = std::cout.rdbuf();
            original_cerr_buf_ = std::cerr.rdbuf();
            original_clog_buf_ = std::clog.rdbuf();

            std::cout.rdbuf(cout_buf_.get());
            std::cerr.rdbuf(cerr_buf_.get());
            std::clog.rdbuf(clog_buf_.get());

            redirected_ = true;
        }
    }

    void restore() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (redirected_) {
            std::cout.rdbuf(original_cout_buf_);
            std::cerr.rdbuf(original_cerr_buf_);
            std::clog.rdbuf(original_clog_buf_);

            redirected_ = false;
        }
    }

    bool is_redirected() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return redirected_;
    }

    void flush() {
        if (redirected_) {
            std::cout.flush();
            std::cerr.flush();
            std::clog.flush();
        }
    }

private:
    std::unique_ptr<GlogStreambuf> cout_buf_;
    std::unique_ptr<GlogStreambuf> cerr_buf_;
    std::unique_ptr<GlogStreambuf> clog_buf_;

    std::streambuf* original_cout_buf_;
    std::streambuf* original_cerr_buf_;
    std::streambuf* original_clog_buf_;

    bool redirected_;
    mutable std::mutex mutex_;
};

class ScopedStdRedirect {
public:
    explicit ScopedStdRedirect(StdRedirector& redirector)
        : redirector_(redirector), was_redirected_(redirector.is_redirected()) {
        if (!was_redirected_) {
            redirector_.redirect();
        }
    }

    ~ScopedStdRedirect() {
        if (!was_redirected_) {
            redirector_.restore();
        }
    }

    ScopedStdRedirect(const ScopedStdRedirect&) = delete;
    ScopedStdRedirect& operator=(const ScopedStdRedirect&) = delete;

private:
    StdRedirector& redirector_;
    bool was_redirected_;
};

extern StdRedirector g_std_redirector;

inline void enable_std_to_glog() {
    g_std_redirector.redirect();
}

inline void disable_std_to_glog() {
    g_std_redirector.restore();
}

inline bool is_std_redirected() {
    return g_std_redirector.is_redirected();
}

inline void flush_std_to_glog() {
    g_std_redirector.flush();
}

inline ScopedStdRedirect make_scoped_std_redirect() {
    return ScopedStdRedirect(g_std_redirector);
}

/*
Usage Examples:

1. Basic Usage:
   enable_std_to_glog();
   std::cout << "This goes to LOG(INFO)" << std::endl;
   std::cerr << "This goes to LOG(ERROR)" << std::endl;
   std::clog << "This goes to LOG(WARNING)" << std::endl;
   disable_std_to_glog();

2. RAII Style (Recommended):
   {
       auto redirect = make_scoped_std_redirect();
       std::cout << "This output is redirected to glog" << std::endl;
   } // Automatically restored here

3. Manual Control:
   StdRedirector redirector;
   redirector.redirect();
   std::cout << "Redirected output" << std::endl;
   redirector.restore();

4. Check Status:
   if (is_std_redirected()) {
       std::cout << "Currently redirected" << std::endl;
   }

5. Force Flush:
   std::cout << "Output without newline";
   flush_std_to_glog(); // Force immediate output to glog
*/

#endif