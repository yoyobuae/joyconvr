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

#include <errno.h>
#include <error.h>
#include <fcntl.h>
#include <glob.h>
#include <linux/input.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>

#include "input.h"


Mouse::Mouse() : fd(-1) { }

Mouse::~Mouse()
{
    close();
}

bool Mouse::open(const char *device)
{
    if (device == nullptr)
        return false;

    if (fd >= 0) {
        close();
    }

    fd = ::open(device, O_RDONLY);

    if (fd < 0)
        return false;

    int flags = fcntl(fd, F_GETFL, 0);
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        close();
        return false;
    }

    return true;
}

void Mouse::close()
{
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}

bool Mouse::read(int *x, int *y)
{
    if (fd < 0)
        return false;

    bool ret = false;
    struct input_event ev;

    do {
        auto bytes_read = ::read(fd, &ev, sizeof(ev));

        if (bytes_read < 0) {
            break;
        }

        if (bytes_read == 0) {
            break;
        }

        if (bytes_read < static_cast<decltype(bytes_read)>(sizeof(ev))) {
            break;
        }

        if (ev.type == EV_REL && ev.code == REL_X && x != nullptr) {
            *x += ev.value;
            ret = true;
        }

        if (ev.type == EV_REL && ev.code == REL_Y && y != nullptr) {
            *y += ev.value;
            ret = true;
        }

    } while (true);

    return ret;
}

class Keyboard {
public:
    Keyboard();
    ~Keyboard();

    bool open(const char *device);
    void close();
    bool isOpen();

    bool read();

    bool a[KEY_CNT];

private:
    int fd;
    bool state[KEY_CNT];
};

Keyboard::Keyboard()
    : fd(-1)
{
    memset(state, 0, sizeof(state));
    memset(a, 0, sizeof(a));
}

Keyboard::~Keyboard()
{
    close();
}

bool Keyboard::open(const char *device)
{
    if (device == nullptr)
        return false;

    if (fd >= 0)
        close();

    fd = ::open(device, O_RDONLY);

    if (fd < 0)
        return false;

    int flags = fcntl(fd, F_GETFL, 0);
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        return false;
    }

    return true;
}

void Keyboard::close()
{
    if (fd >= 0)
        ::close(fd);
}

bool Keyboard::isOpen()
{
    if (fd >= 0) {
        return true;
    } else {
        return false;
    }
}

bool Keyboard::read()
{
    bool ret = false;

    for (int c = 0; c < KEY_MAX; c++) {
        if (a != nullptr && a[c] && !state[c]) {
            a[c] = false;
            ret = true;
        }
    }

    if (fd < 0)
        return false;

    struct input_event ev;

    do {
        auto bytes_read = ::read(fd, &ev, sizeof(ev));

        if (bytes_read < 0) {
            break;
        }

        if (bytes_read == 0) {
            break;
        }

        if (bytes_read < static_cast<decltype(bytes_read)>(sizeof(ev))) {
            break;
        }

        if (ev.type == EV_KEY && ev.code > 0 && ev.code < KEY_MAX) {
            switch (ev.value) {
                case 0:
                    state[ev.code] = false;
                    ret = true;
                    break;
                case 1:
                    state[ev.code] = true;
                    if (a != nullptr) a[ev.code] = true;
                    ret = true;
                    break;
                default:
                    // Do nothing
                    break;
            }
        }
    } while (true);

    return ret;
}

class Joystick {
public:
    Joystick(int deadzone);
    ~Joystick();

    bool open(const char *device);
    void close();
    bool isOpen();

    bool read();

    bool a[KEY_CNT];
    int b[ABS_CNT];

private:
    int fd;
    bool state[KEY_CNT];
    int axes[ABS_CNT];
    int deadzone;
};

Joystick::Joystick(int deadzone)
    : fd(-1)
    , deadzone(deadzone)
{
    memset(state, 0, sizeof(state));
    memset(axes, 0, sizeof(axes));
    memset(a, 0, sizeof(a));
    memset(b, 0, sizeof(b));
}

Joystick::~Joystick()
{
    close();
}

bool Joystick::open(const char *device)
{
    if (device == nullptr)
        return false;

    if (fd >= 0)
        close();

    fd = ::open(device, O_RDONLY);

    if (fd < 0)
        return false;

    int flags = fcntl(fd, F_GETFL, 0);
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        return false;
    }

    return true;
}

void Joystick::close()
{
    if (fd >= 0)
        ::close(fd);
}

bool Joystick::isOpen()
{
    if (fd >= 0) {
        return true;
    } else {
        return false;
    }
}

bool Joystick::read()
{
    bool ret = false;

    for (int c = 0; c < KEY_MAX; c++) {
        if (a != nullptr && a[c] && !state[c]) {
            a[c] = false;
            ret = true;
        }
    }

    if (fd < 0)
        return false;

    struct input_event ev;

    do {
        auto bytes_read = ::read(fd, &ev, sizeof(ev));

        if (bytes_read < 0) {
            break;
        }

        if (bytes_read == 0) {
            break;
        }

        if (bytes_read < static_cast<decltype(bytes_read)>(sizeof(ev))) {
            break;
        }

        if (ev.type == EV_KEY && ev.code > 0 && ev.code < KEY_MAX) {
            switch (ev.value) {
                case 0:
                    state[ev.code] = false;
                    ret = true;
                    break;
                case 1:
                    state[ev.code] = true;
                    if (a != nullptr) a[ev.code] = true;
                    ret = true;
                    break;
                default:
                    // Do nothing
                    break;
            }
        }

        if (ev.type == EV_ABS && ev.code >= 0 && ev.code < ABS_MAX) {
            if (ev.value != state[ev.code]) {
                if (abs(ev.value) > abs(deadzone)) {
                    if (b != nullptr) b[ev.code] = ev.value;
                    state[ev.code] = ev.value;
                    ret = true;
                }
                else
                {
                    if (b != nullptr) b[ev.code] = 0;
                    if (state[ev.code]) ret = true;
                    state[ev.code] = 0;
                }
            }
        }
    } while (true);

    return ret;
}

Keyboard keyboard;
static std::chrono::time_point<std::chrono::steady_clock> lastGetKeyTime;

bool do_glob(const char *pattern, glob_t *pglob)
{
    int ret = glob(pattern, GLOB_NOSORT, NULL, pglob);

    if (ret != 0) {
        return false;
    }

    return true;
}

const char *get_device_name(const char *path, char *buffer, size_t size)
{
    int fd = open(path, O_RDONLY);

    if (fd < 0)
        return "";

    //Print Device Name
    int res = ioctl(fd, EVIOCGNAME (size), buffer);
    close(fd);

    if (res < 0)
        return "";

    return buffer;
}

char *find_device_with_name(const char *pattern)
{
    glob_t globbuf;
    if (do_glob("/dev/input/event*", &globbuf) == false)
        return NULL;

    for (size_t c = 0; c < globbuf.gl_pathc; c++) {
        char name[256];
        if (strcmp(get_device_name(globbuf.gl_pathv[c], name, sizeof(name)), pattern) == 0) {
            char *ret = strdup(globbuf.gl_pathv[c]);
            globfree(&globbuf);
            return ret;
        }
    }
    globfree(&globbuf);
    return NULL;

}

bool getKey(int key)
{
    if (!keyboard.isOpen()) {
        char *device = find_device_with_name("Genius SlimStar 335");
        if (device) {
            keyboard.open(device);
            free(device);
        }
    }

    if (keyboard.isOpen()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<int, std::ratio<1, 60>>>(now - lastGetKeyTime);
        if (elapsed.count() > 0) {
            keyboard.read();
        }

        if (key >= 0 && key < KEY_MAX)
            return keyboard.a[key];
    }

    return false;
}

Joystick joystick(0);
static std::chrono::time_point<std::chrono::steady_clock> lastGetJoystickTime;

bool getJoyButton(int button)
{
    if (!joystick.isOpen()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<int, std::ratio<5, 1>>>(now - lastGetJoystickTime);
        if (elapsed.count() > 0) {
            char *device = find_device_with_name("Nintendo Switch Combined Joy-Cons");
            if (device) {
                joystick.open(device);
                free(device);
            }
            lastGetJoystickTime = now;
        }
    }

    if (joystick.isOpen()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<int, std::ratio<1, 60>>>(now - lastGetJoystickTime);
        if (elapsed.count() > 0) {
            joystick.read();
            lastGetJoystickTime = now;
        }

        if (button >= 0 && button < KEY_MAX)
            return joystick.a[button];
    }

    return false;
}

int getJoyAxis(int axis)
{
    if (!joystick.isOpen()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<int, std::ratio<5, 1>>>(now - lastGetJoystickTime);
        if (elapsed.count() > 0) {
            char *device = find_device_with_name("Nintendo Switch Combined Joy-Cons");
            if (device) {
                joystick.open(device);
                free(device);
            }
            lastGetJoystickTime = now;
        }
    }

    if (joystick.isOpen()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<int, std::ratio<1, 60>>>(now - lastGetJoystickTime);
        if (elapsed.count() > 0) {
            joystick.read();
            lastGetJoystickTime = now;
        }

        if (axis >= 0 && axis < ABS_MAX)
            return joystick.b[axis];
    }

    return 0;
}
