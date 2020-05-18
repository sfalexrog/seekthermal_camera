/*
 *  Seek debug macros
 *  Author: Maarten Vandersteegen
 */

#ifndef SEEK_DEBUG_H
#define SEEK_DEBUG_H

#include <stdio.h>

namespace LibSeek
{

enum Severity
{
    Debug,
    Info,
    Warning,
    Error,
    Fatal
};

using LogFn = void(Severity severity, const char* message, void* user_data);

void setMinLogSeverity(Severity minSeverity);
void setLogFn(const LogFn* log_callback, void* user_data);

void log(Severity severity, const char* fmt, ...);

}

#endif /* SEEK_DEBUG_H */
