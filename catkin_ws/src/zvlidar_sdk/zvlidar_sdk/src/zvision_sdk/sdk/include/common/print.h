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


#ifndef PRINT_H_
#define PRINT_H_

#include <stdio.h>
#include <stdarg.h>

#define LOG_ERROR(...)   zvision::log::print (zvision::log::Error, __VA_ARGS__)
#define LOG_WARN(...)    zvision::log::print (zvision::log::Warn, __VA_ARGS__)
#define LOG_INFO(...)    zvision::log::print (zvision::log::Info, __VA_ARGS__)
#define LOG_DEBUG(...)   zvision::log::print (zvision::log::Debug, __VA_ARGS__)

namespace zvision
{
    namespace log
    {
        /** \brief Set of return code. */
        enum LogLevel
        {
            Debug,
            Info,
            Warn,
            Error,
        };

        enum ColorAttribute
        {
            Reset = 0,
            Bright = 1,
            Dim = 2,
            Underline = 3,
            Blink = 4,
            Reverse = 7,
            Hiden = 8
        };

        enum Color
        {
            Black,
            Red,
            Green,
            Yellow,
            Blue,
            Magenta,
            Cyan,
            White
        };

        /** \brief Print a message
        * \param level the log level
        * \param format the message
        */
        void print(LogLevel level, const char *format, ...);

        /** \brief Change the text color (on either stdout or stderr) with an attr:fg
        * \param stream the output stream (stdout, stderr, etc)
        * \param attribute the text attribute
        * \param fg the foreground color
        */
        void change_text_color(FILE *stream, int attribute, int fg);

        /** \brief Reset the text color (on either stdout or stderr) to its original state
        * \param stream the output stream (stdout, stderr, etc)
        */
        void reset_text_color(FILE *stream);

    }

}


#endif // end PRINT_H_
