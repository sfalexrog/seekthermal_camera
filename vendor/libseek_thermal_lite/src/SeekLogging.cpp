#include "SeekLogging.h"
#include <vector>
#include <cstdio>
#include <cstdarg>

namespace
{

void logFnStdout(LibSeek::Severity severity, const char* message, void* user_data)
{
    switch(severity)
    {
        case LibSeek::Severity::Debug:
        case LibSeek::Severity::Info:
        case LibSeek::Severity::Warning:
            std::printf("%s", message);
            break;

        case LibSeek::Severity::Error:
        case LibSeek::Severity::Fatal:
            std::fprintf(stderr, "%s", message);
            break;
        
        default:
            std::fprintf(stderr, "Unexpected severity, expect failure. Message: %s", message);
    }
}

// FIXME: What about multiple instances?
LibSeek::LogFn* g_logFn = logFnStdout;
void* g_userData = nullptr;
LibSeek::Severity g_MinSeverity = LibSeek::Severity::Info;

} // anonymous namespace

namespace LibSeek
{

void setLogFn(const LogFn* log_callback, void* user_data)
{
    g_logFn = log_callback;
    g_userData = user_data;
}

void log(Severity severity, const char* fmt, ...)
{
    if (severity < g_MinSeverity) return;
    va_list args1;
    va_start(args1, fmt);
    va_list args2;
    va_copy(args2 ,args1);
    std::vector<char> buf(1 + std::vsnprintf(nullptr, 0, fmt, args1));
    va_end(args1);
    std::vsnprintf(buf.data(), buf.size(), fmt, args2);
    va_end(args2);
    g_logFn(severity, buf.data(), g_userData);
}

}
