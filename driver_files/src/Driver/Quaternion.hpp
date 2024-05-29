#include <cmath>

class Quaternion {
public:
    double w, x, y, z;

    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    // Computes the dot product of this quaternion with another.
    double dot(const Quaternion& other) const {
        return w * other.w + x * other.x + y * other.y + z * other.z;
    }

    // Compute the quaternion product of this quaternion with another
    Quaternion prod(const Quaternion& other) const {
        return Quaternion(w * other.w - x * other.x - y * other.y - z * other.z,
                          w * other.x + x * other.w + y * other.z - z * other.y,
                          w * other.y - x * other.z + y * other.w + z * other.x,
                          w * other.z + x * other.y - y * other.x + z * other.w);
    }

    // Conjugates this quaternion
    Quaternion conj() const {
        return Quaternion(w, -x, -y, -z);
    }

    // Normalizes this quaternion.
    Quaternion normalized() const {
        double norm = std::sqrt(w * w + x * x + y * y + z * z);
        return Quaternion(w / norm, x / norm, y / norm, z / norm);
    }

    // Scales this quaternion by a scalar value.
    Quaternion operator*(double scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    // Adds this quaternion to another.
    Quaternion operator+(const Quaternion& other) const {
        return Quaternion(w + other.w, x + other.x, y + other.y, z + other.z);
    }

    // Unary negation.
    Quaternion operator-() const {
        return Quaternion(-w, -x, -y, -z);
    }
};

Quaternion slerp(const Quaternion& q0, const Quaternion& q1, double t) {
    double dot = q0.dot(q1);

    Quaternion end = q1;
    if (dot < 0.0) {
        end = -q1; // Take the shortest path
        dot = -dot;
    }

    if (dot > 0.9995) {
        // If the quaternions are very close, linearly interpolate and normalize the result
        Quaternion result = q0 + (end +(-q0)) * t;
        return result.normalized();
    }

    double theta_0 = std::acos(dot); // theta_0 = angle between input vectors
    double theta = theta_0 * t; // theta = angle between q0 and result

    double sin_theta = std::sin(theta); // Compute this value once
    double sin_theta_0 = std::sin(theta_0); // Compute this value once

    double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0; // == sin(theta_0 - theta) / sin(theta_0)
    double s1 = sin_theta / sin_theta_0;

    return (q0 * s0) + (end * s1);
}

Quaternion drift_compensation(const Quaternion& IMU, const Quaternion& camera, Quaternion *state) {
    Quaternion difference = camera.prod(IMU.conj());
    *state = slerp(*state, difference, 0.005);
    return state->prod(IMU);
}

