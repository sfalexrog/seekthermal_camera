#pragma once

#include "SeekLogging.h"

#define debug(fmt, ...)     LibSeek::log(LibSeek::Severity::Debug, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__);
#define info(fmt, ...)      LibSeek::log(LibSeek::Severity::Info, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__);
#define warn(fmt, ...)      LibSeek::log(LibSeek::Severity::Warning, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__);
#define error(fmt, ...)     LibSeek::log(LibSeek::Severity::Error, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__);
#define fatal(fmt, ...)     LibSeek::log(LibSeek::Severity::Fatal, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__);
