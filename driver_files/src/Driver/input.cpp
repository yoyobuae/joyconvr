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
#include <linux/input-event-codes.h>

#include <chrono>

#include <Eigen/Geometry>

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

/******************************* Keyboard class *******************************/

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

/***************************** End Keyboard class *****************************/


/******************************* Joystick class *******************************/

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

/***************************** End Joystick class *****************************/

/**************************** Madgwick IMU filter  ****************************/

// Math library required for ‘sqrt’
#include <math.h>

// System constants
// #define deltat 0.001f // sampling period in seconds (shown as 1 ms)
#define gyroMeasError 3.14159265358979f * (20.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta

// Params
// double w_x, w_y, w_z; // gyroscope measurements in rad/s
// double a_x, a_y, a_z; // accelerometer measurements
// double SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions

void filterUpdate(double w_x, double w_y, double w_z, double a_x, double a_y, double a_z, double deltat, double &SEq_1, double &SEq_2, double &SEq_3, double &SEq_4)
{
    // Local system variables
    double norm; // vector norm
    double SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
    double f_1, f_2, f_3; // objective function elements
    double J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    double SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error

    // Axulirary variables to avoid reapeated calcualtions
    double halfSEq_1 = 0.5f * SEq_1;
    double halfSEq_2 = 0.5f * SEq_2;
    double halfSEq_3 = 0.5f * SEq_3;
    double halfSEq_4 = 0.5f * SEq_4;
    double twoSEq_1 = 2.0f * SEq_1;
    double twoSEq_2 = 2.0f * SEq_2;
    double twoSEq_3 = 2.0f * SEq_3;

    // Normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;

    // Compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SEq_4; // J_12 negated in matrix multiplication
    J_13or22 = twoSEq_1;
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication

    // Compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

    // Normalise the gradient
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 /= norm;
    SEqHatDot_2 /= norm;
    SEqHatDot_3 /= norm;
    SEqHatDot_4 /= norm;

    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

    // Compute then integrate the estimated quaternion derrivative
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

    // Normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;
}

/************************** Madgwick IMU filter End ***************************/


/******** Snippet from drivers/hid/hid-nintendo.c on Linux Kernel tree ********/

/*
 * The controller's accelerometer has a sensor resolution of 16bits and is
 * configured with a range of +-8000 milliGs. Therefore, the resolution can be
 * calculated thus: (2^16-1)/(8000 * 2) = 4.096 digits per milliG
 * Resolution per G (rather than per millliG): 4.096 * 1000 = 4096 digits per G
 * Alternatively: 1/4096 = .0002441 Gs per digit
 */
static const __s32 JC_IMU_MAX_ACCEL_MAG           = 32767;
static const __u16 JC_IMU_ACCEL_RES_PER_G         = 4096;
static const __u16 JC_IMU_ACCEL_FUZZ              = 10;
static const __u16 JC_IMU_ACCEL_FLAT              = 0;

/*
 * The controller's gyroscope has a sensor resolution of 16bits and is
 * configured with a range of +-2000 degrees/second.
 * Digits per dps: (2^16 -1)/(2000*2) = 16.38375
 * dps per digit: 16.38375E-1 = .0610
 *
 * STMicro recommends in the datasheet to add 15% to the dps/digit. This allows
 * the full sensitivity range to be saturated without clipping. This yields more
 * accurate results, so it's the technique this driver uses.
 * dps per digit (corrected): .0610 * 1.15 = .0702
 * digits per dps (corrected): .0702E-1 = 14.247
 *
 * Now, 14.247 truncating to 14 loses a lot of precision, so we rescale the
 * min/max range by 1000.
 */
static const __s32 JC_IMU_PREC_RANGE_SCALE        = 1000;
/* Note: change mag and res_per_dps if prec_range_scale is ever altered */
static const __s32 JC_IMU_MAX_GYRO_MAG            = 32767000; /* (2^16-1)*1000 */
static const __u16 JC_IMU_GYRO_RES_PER_DPS        = 14247; /* (14.247*1000) */
static const __u16 JC_IMU_GYRO_FUZZ               = 10;
static const __u16 JC_IMU_GYRO_FLAT               = 0;

/****** End snippet from drivers/hid/hid-nintendo.c on Linux Kernel tree ******/


/********************************* IMU class **********************************/

struct gyro_sample {
    double x;
    double y;
    double z;
};

class IMU {
public:
    IMU();
    ~IMU();

    bool open(const char *device);
    void close();
    bool isOpen();

    bool read(double q[4]);
    void reset();

private:
    bool update();
    void drift_compensation(double &w_x, double &w_y, double &w_z);

    int fd;
    int axes[ABS_CNT];
    int count = 0;
    __s32 timestamp = 0, prev_timestamp = 0;
    double deltat = 0.0f;
    double SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f;
    static const int gyro_history_size = 50;
    struct gyro_sample gyro_history[gyro_history_size];
    int gyro_history_head = 0;
    double gyro_activity_level = 0;
    double g_bias_x = 0.0, g_bias_y = 0.0, g_bias_z = 0.0;
    bool valid = false;
};

IMU::IMU()
    : fd(-1)
{
    memset(axes, 0, sizeof(axes));
    memset(gyro_history, 0, sizeof(gyro_history));
}

IMU::~IMU()
{
    close();
}

bool IMU::open(const char *device)
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

void IMU::close()
{
    if (fd >= 0)
        ::close(fd);
}

bool IMU::isOpen()
{
    if (fd >= 0) {
        return true;
    } else {
        return false;
    }
}

bool IMU::read(double q[4])
{
    bool ret = false;

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

        if (ev.type == EV_ABS && ev.code >= 0 && ev.code < ABS_MAX) {
            axes[ev.code] = ev.value;
        }

        if (ev.type == EV_MSC && ev.code == MSC_TIMESTAMP) {
            timestamp = ev.value;
        }

        if (ev.type == EV_SYN && ev.code == SYN_REPORT) {
            if (count > 15) {
                ret = update();
                valid = true;
            }
            prev_timestamp = timestamp;
            count++;
        }
    } while (true);

    if (ret) {
        q[0] = SEq_1;
        q[1] = SEq_2;
        q[2] = SEq_3;
        q[3] = SEq_4;
    }
    return ret;
}

void IMU::reset()
{
    SEq_1 = 1.0f;
    SEq_2 = 0.0f;
    SEq_3 = 0.0f;
    SEq_4 = 0.0f;
}

bool IMU::update()
{
    /* Convert accelerometer values to G units */
    double a_x = (double)axes[ABS_X]/JC_IMU_ACCEL_RES_PER_G;
    double a_y = (double)axes[ABS_Y]/JC_IMU_ACCEL_RES_PER_G;
    double a_z = (double)axes[ABS_Z]/JC_IMU_ACCEL_RES_PER_G;

    /* Convert accelerometer values to radian/s */
    double w_x = (double)axes[ABS_RX]*(M_PI/180)/JC_IMU_GYRO_RES_PER_DPS;
    double w_y = (double)axes[ABS_RY]*(M_PI/180)/JC_IMU_GYRO_RES_PER_DPS;
    double w_z = (double)axes[ABS_RZ]*(M_PI/180)/JC_IMU_GYRO_RES_PER_DPS;

    /* Calculate timestamp delta since last update and convert to seconds */
    double deltat = (double)(timestamp - prev_timestamp)/1000000.0f;

    drift_compensation(w_x, w_y, w_z);

    filterUpdate(w_x, w_y, w_z, a_x, a_y, a_z, deltat, SEq_1, SEq_2, SEq_3, SEq_4);

    return true;
}

void IMU::drift_compensation(double &w_x, double &w_y, double &w_z)
{
    /* Gyro activity level
     * Activity level is just sumation of the absolute value of all gyro
     * measurements in number of the most gyro measurements.
     *
     * New gyro measurements added to a circular buffer and are also added
     * to a running sum and old measurements removed from the buffer are
     * substracted from the running sum.
     *
     * This has the same effect as if every measurement in the circular
     * buffer was summed up but it's far less operations per iteration
     */
    gyro_activity_level -= fabs(gyro_history[gyro_history_head].x);
    gyro_activity_level -= fabs(gyro_history[gyro_history_head].y);
    gyro_activity_level -= fabs(gyro_history[gyro_history_head].z);
    gyro_activity_level += fabs(w_x);
    gyro_activity_level += fabs(w_y);
    gyro_activity_level += fabs(w_z);
    struct gyro_sample sample = { w_x, w_y, w_z };
    gyro_history[gyro_history_head++] = sample;
    gyro_history_head %= gyro_history_size;

    /* Update gyro biases if activity level is low enough
     * An activity level under 5.0 roughtly corresponds to the controller
     * being held relatively still on the user's hand.
     *
     * If the controller is held still then gyro bias is estimated by
     * calculating a rolling average for each of the gyro axes.
     */
    if (gyro_activity_level < 2.0) {
        double rate = 0.001;
        int gyro_history_midpoint = (gyro_history_head + gyro_history_size/2) % gyro_history_size;
        g_bias_x = (1 - rate)*g_bias_x + rate*gyro_history[gyro_history_midpoint].x;
        g_bias_y = (1 - rate)*g_bias_y + rate*gyro_history[gyro_history_midpoint].y;
        g_bias_z = (1 - rate)*g_bias_z + rate*gyro_history[gyro_history_midpoint].z;
    }

    /* Compensate for gyro biases */
    w_x -= g_bias_x;
    w_y -= g_bias_y;
    w_z -= g_bias_z;
}

/******************************* End IMU class ********************************/
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
    {
        errno = 0;
        return "";
    }

    //Print Device Name
    int res = ioctl(fd, EVIOCGNAME (size), buffer);
    close(fd);

    if (res < 0)
    {
        errno = 0;
        return "";
    }

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

IMU left_imu;
static std::chrono::time_point<std::chrono::steady_clock> lastGetLeftIMUTime;

bool getLeftIMU(double q[4])
{
    bool ret = false;
    if (!left_imu.isOpen()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<int, std::ratio<5, 1>>>(now - lastGetLeftIMUTime);
        if (elapsed.count() > 0) {
            char *device = find_device_with_name("Nintendo Switch Left Joy-Con IMU");
            if (device) {
                left_imu.open(device);
                free(device);
            }
            lastGetLeftIMUTime = now;
        }
    }

    if (left_imu.isOpen()) {
        ret = left_imu.read(q);
    }

    return ret;
}

void resetLeftIMU()
{
    if (left_imu.isOpen()) {
        left_imu.reset();
    }
}

IMU right_imu;
static std::chrono::time_point<std::chrono::steady_clock> lastGetRightIMUTime;

bool getRightIMU(double q[4])
{
    bool ret = false;
    if (!right_imu.isOpen()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<int, std::ratio<5, 1>>>(now - lastGetRightIMUTime);
        if (elapsed.count() > 0) {
            char *device = find_device_with_name("Nintendo Switch Right Joy-Con IMU");
            if (device) {
                right_imu.open(device);
                free(device);
            }
            lastGetRightIMUTime = now;
        }
    }

    if (right_imu.isOpen()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<int, std::ratio<1, 60>>>(now - lastGetRightIMUTime);
        if (elapsed.count() > 0) {
            ret = right_imu.read(q);
            lastGetRightIMUTime = now;
        }
    }

    return ret;
}

void resetRightIMU()
{
    if (right_imu.isOpen()) {
        right_imu.reset();
    }
}
