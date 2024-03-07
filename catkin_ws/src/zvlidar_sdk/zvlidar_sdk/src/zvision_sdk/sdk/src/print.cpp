// MIT License
//
// Copyright(c) 2019 ZVISION. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include "print.h"

#include <string>
#include <time.h>
#include <chrono>
#if defined _WIN32
# include <windows.h>

#ifndef _MSC_VER
# define COMMON_LVB_UNDERSCORE    0
# define COMMON_LVB_REVERSE_VIDEO 0
#endif

WORD
convertAttributesColor(int attribute, int fg, int bg = 0)
{
    static WORD wAttributes[7] = { 0,                        // TT_RESET
        FOREGROUND_INTENSITY ,    // TT_BRIGHT
        0,                        // TT_DIM
        COMMON_LVB_UNDERSCORE,    // TT_UNDERLINE
        0,                        // TT_BLINK
        COMMON_LVB_REVERSE_VIDEO, // TT_REVERSE
        0                         // TT_HIDDEN
    };
    static WORD wFgColors[8] = { 0,                                                  // TT_BLACK
        FOREGROUND_RED,                                     // TT_RED
        FOREGROUND_GREEN ,                                  // TT_GREEN
        FOREGROUND_GREEN | FOREGROUND_RED ,                 // TT_YELLOW
        FOREGROUND_BLUE ,                                   // TT_BLUE
        FOREGROUND_RED | FOREGROUND_BLUE ,                  // TT_MAGENTA
        FOREGROUND_GREEN | FOREGROUND_BLUE,                 // TT_CYAN
        FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_RED // TT_WHITE
    };
    static WORD wBgColors[8] = { 0,                                                  // TT_BLACK
        BACKGROUND_RED,                                     // TT_RED
        BACKGROUND_GREEN ,                                  // TT_GREEN
        BACKGROUND_GREEN | BACKGROUND_BLUE ,                // TT_YELLOW
        BACKGROUND_BLUE ,                                   // TT_BLUE
        BACKGROUND_RED | BACKGROUND_BLUE ,                  // TT_MAGENTA
        BACKGROUND_GREEN | BACKGROUND_BLUE,                 // TT_CYAN
        BACKGROUND_GREEN | BACKGROUND_BLUE | BACKGROUND_RED // TT_WHITE
    };

    return wAttributes[attribute] | wFgColors[fg] | wBgColors[bg];
}

#endif

void zvision::log::print(LogLevel level, const char *format, ...)
{
    FILE *stream = (level == Warn || level == Error) ? stderr : stdout;
    switch (level)
    {
    case Debug:
        change_text_color(stream, Reset, Green);
        break;
    case Warn:
        change_text_color(stream, Bright, Yellow);
        break;
    case Error:
        change_text_color(stream, Bright, Red);
        break;
    case Info:
    default:
        break;
    }

	// add timestamp  ->
	std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> ptime = \
		std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
	time_t timestamp_ms = ptime.time_since_epoch().count();

	std::string tstmp = std::string("timestamp(ms):") + std::to_string(timestamp_ms)+"\n";
	printf("%s", tstmp.data());
	// <-


    va_list ap;

    va_start(ap, format);
    vfprintf(stream, format, ap);
    va_end(ap);

    reset_text_color(stream);
}

void zvision::log::change_text_color(FILE *stream, int attribute, int fg)
{
#ifdef _WIN32
    HANDLE h = GetStdHandle((stream == stdout) ? STD_OUTPUT_HANDLE : STD_ERROR_HANDLE);
    SetConsoleTextAttribute(h, convertAttributesColor(attribute, fg));
#else
    char command[13];
    // Command is the control command to the terminal
    sprintf(command, "%c[%d;%dm", 0x1B, attribute, fg + 30);
    fprintf(stream, "%s", command);
#endif
}

void zvision::log::reset_text_color(FILE *stream)
{
#ifdef WIN32
    HANDLE h = GetStdHandle((stream == stdout) ? STD_OUTPUT_HANDLE : STD_ERROR_HANDLE);
    SetConsoleTextAttribute(h, convertAttributesColor(0, White, Black));
#else
    char command[13];
    // Command is the control command to the terminal
    sprintf(command, "%c[0;m", 0x1B);
    fprintf(stream, "%s", command);
#endif
}

