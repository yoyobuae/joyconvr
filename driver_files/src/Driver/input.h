/*
 * MIT License
 *
 * Copyright (c) 2021 Gerald Young (Yoyobuae)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the “Software”),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef INPUT_H
#define INPUT_H

#pragma once

#include <linux/input-event-codes.h>

class Mouse {
public:
    Mouse();
    ~Mouse();

    bool open(const char *device);
    void close();

    bool read(int *x, int *y);

private:
    int fd;
};

extern "C" bool getKey(int key);

extern "C" void readJoyButton();
extern "C" bool getJoyButton(int button);
extern "C" void readJoyAxis();
extern "C" int getJoyAxis(int axis);
extern "C" bool getLeftIMU(double q[4]);
extern "C" void resetLeftIMU();
extern "C" bool getRightIMU(double q[4]);
extern "C" void resetRightIMU();

#endif // INPUT_H
