#include <iostream>
#include <sstream>
#include <algorithm>

#include "ControllerDevice.hpp"

#include "input.h"


const double pi = std::acos(-1);

inline vr::HmdQuaternion_t HmdQuaternion_Init( double w, double x, double y, double z )
{
    vr::HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

inline vr::HmdQuaternion_t HmdQuaternion_Init_Angle( double angle, double x, double y, double z )
{
	double rad = 2 * pi * angle / 360;
	double rad_2 = rad / 2;
	return HmdQuaternion_Init(std::cos(rad_2),
				  -std::sin(rad_2) * x,
				  -std::sin(rad_2) * y,
				  -std::sin(rad_2) * z);
}

inline vr::HmdQuaternion_t HmdQuaternion_Product(vr::HmdQuaternion_t quat_a, vr::HmdQuaternion_t quat_b)
{
    vr::HmdQuaternion_t quat_res;
	quat_res.w = quat_a.w*quat_b.w - quat_a.x*quat_b.x - quat_a.y*quat_b.y - quat_a.z*quat_b.z;
	quat_res.x = quat_a.w*quat_b.x + quat_a.x*quat_b.w + quat_a.y*quat_b.z - quat_a.z*quat_b.y;
	quat_res.y = quat_a.w*quat_b.y - quat_a.x*quat_b.z + quat_a.y*quat_b.w + quat_a.z*quat_b.x;
	quat_res.z = quat_a.w*quat_b.z + quat_a.x*quat_b.y - quat_a.y*quat_b.x + quat_a.z*quat_b.w;
	return quat_res;
}

static void normalizeQuat(double pose[])
{
    //normalize
    double mag = sqrt(pose[3] * pose[3] +
        pose[4] * pose[4] +
        pose[5] * pose[5] +
        pose[6] * pose[6]);

    pose[3] /= mag;
    pose[4] /= mag;
    pose[5] /= mag;
    pose[6] /= mag;
}

JoyconVrDriver::ControllerDevice::ControllerDevice(std::string serial, ControllerDevice::Handedness handedness):
    serial_(serial),
    handedness_(handedness)
{
}

std::string JoyconVrDriver::ControllerDevice::GetSerial()
{
    return this->serial_;
}

void JoyconVrDriver::ControllerDevice::Update()
{
    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return;

    // Check if we need to keep vibrating
    if (this->did_vibrate_) {
        this->vibrate_anim_state_ += (GetDriver()->GetLastFrameTime().count()/1000.f);
        if (this->vibrate_anim_state_ > 1.0f) {
            this->did_vibrate_ = false;
            this->vibrate_anim_state_ = 0.0f;
        }
    }

    // Setup pose for this frame
    auto pose = this->last_pose_;

    double q[4];
    bool q_valid = false;

    if (this->handedness_ == Handedness::LEFT) {
        pose.vecPosition[0] = -0.2;
        pose.vecPosition[1] = 1.3;
        pose.vecPosition[2] = -0.2;

        q_valid = getLeftIMU(q);

        GetDriver()->GetInput()->UpdateBooleanComponent(application_button_click_component_, getJoyButton(BTN_Z), 0); //Application Menu
        GetDriver()->GetInput()->UpdateBooleanComponent(grip_button_click_component_, getJoyButton(BTN_TL2), 0); //Grip
        GetDriver()->GetInput()->UpdateBooleanComponent(system_button_click_component_, getJoyButton(BTN_SELECT), 0); //System
        GetDriver()->GetInput()->UpdateBooleanComponent(trackpad_button_click_component_, getJoyButton(BTN_THUMBL), 0); //Trackpad

        float x = static_cast<float>(getJoyAxis(ABS_X))/32768.0f;
        float y = static_cast<float>(getJoyAxis(ABS_Y))/-32768.0f;
        float norm = hypot(x, y);
        float arct = atan2(y, x);

        // Deadzone and touch calcs
        bool trackpad_touch = norm > 0.125f;
        float trackpad_norm = norm > 0.25f ? (norm - 0.25f) / 0.75f : 0.0f;
        float trackpad_x = trackpad_norm * cos(arct);
        float trackpad_y = trackpad_norm * sin(arct);

        GetDriver()->GetInput()->UpdateBooleanComponent(trackpad_touch_component_, trackpad_touch, 0); //Trackpad
        GetDriver()->GetInput()->UpdateScalarComponent(trackpad_x_component_, trackpad_x, 0); //Trackpad x
        GetDriver()->GetInput()->UpdateScalarComponent(trackpad_y_component_, trackpad_y, 0); //Trackpad y


        if (getJoyButton(BTN_TL)) { //Trigger
            GetDriver()->GetInput()->UpdateScalarComponent(trigger_value_component_, 1.0, 0);
        } else {
            GetDriver()->GetInput()->UpdateScalarComponent(trigger_value_component_, 0.0, 0);
        }

        if (getJoyButton(BTN_DPAD_RIGHT)) { // IMU reset
            resetLeftIMU();
        }
    }
    else if (this->handedness_ == Handedness::RIGHT) {
        pose.vecPosition[0] = 0.2;
        pose.vecPosition[1] = 1.3;
        pose.vecPosition[2] = -0.2;

        q_valid = getRightIMU(q);

        GetDriver()->GetInput()->UpdateBooleanComponent(application_button_click_component_, getJoyButton(BTN_MODE), 0); //Application Menu
        GetDriver()->GetInput()->UpdateBooleanComponent(grip_button_click_component_, getJoyButton(BTN_TR2), 0); //Grip
        GetDriver()->GetInput()->UpdateBooleanComponent(system_button_click_component_, getJoyButton(BTN_START), 0); //System
        GetDriver()->GetInput()->UpdateBooleanComponent(trackpad_button_click_component_, getJoyButton(BTN_THUMBR), 0); //Trackpad

        float x = static_cast<float>(getJoyAxis(ABS_RX))/32768.0f;
        float y = static_cast<float>(getJoyAxis(ABS_RY))/-32768.0f;
        float norm = hypot(x, y);
        float arct = atan2(y, x);

        // Deadzone and touch calcs
        bool trackpad_touch = norm > 0.125f;
        float trackpad_norm = norm > 0.25f ? (norm - 0.25f) / 0.75f : 0.0f;
        float trackpad_x = trackpad_norm * cos(arct);
        float trackpad_y = trackpad_norm * sin(arct);

        GetDriver()->GetInput()->UpdateBooleanComponent(trackpad_touch_component_, trackpad_touch, 0); //Trackpad
        GetDriver()->GetInput()->UpdateScalarComponent(trackpad_x_component_, trackpad_x, 0); //Trackpad x
        GetDriver()->GetInput()->UpdateScalarComponent(trackpad_y_component_, trackpad_y, 0); //Trackpad y


        if (getJoyButton(BTN_TR)) { //Trigger
            GetDriver()->GetInput()->UpdateScalarComponent(trigger_value_component_, 1.0, 0);
        } else {
            GetDriver()->GetInput()->UpdateScalarComponent(trigger_value_component_, 0.0, 0);
        }

        if (getJoyButton(BTN_WEST)) { // IMU reset
            resetRightIMU();
        }
    }

    if (q_valid) {
        pose.qRotation.w = q[0];
        pose.qRotation.x = -q[2];
        pose.qRotation.y = q[3];
        pose.qRotation.z = -q[1];

        pose.poseTimeOffset = 0;

        pose.poseIsValid = true;

        pose.result = vr::TrackingResult_Running_OK;

        pose.deviceIsConnected = true;

        pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
        pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

        // Post pose
        GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));

        this->last_pose_ = pose;
    }
}

DeviceType JoyconVrDriver::ControllerDevice::GetDeviceType()
{
    return DeviceType::CONTROLLER;
}

JoyconVrDriver::ControllerDevice::Handedness JoyconVrDriver::ControllerDevice::GetHandedness()
{
    return this->handedness_;
}

vr::TrackedDeviceIndex_t JoyconVrDriver::ControllerDevice::GetDeviceIndex()
{
    return this->device_index_;
}

vr::EVRInitError JoyconVrDriver::ControllerDevice::Activate(uint32_t unObjectId)
{
    this->device_index_ = unObjectId;

    std::stringstream ss;
    ss << this->device_index_;
    GetDriver()->Log("Activating controller serial " + this->serial_ + " with index " + ss.str());

    // Get the properties handle
    auto props = GetDriver()->GetProperties()->TrackedDeviceToPropertyContainer(this->device_index_);

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, "vive_controller");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{htc}/input/vive_controller_profile.json");

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "ViveMV");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ManufacturerName_String, "HTC");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_TrackingSystemName_String, "VR Controller");
    GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_SerialNumber_String, this->serial_.c_str());

    uint64_t supportedButtons = 0xFFFFFFFFFFFFFFFFULL;
    GetDriver()->GetProperties()->SetUint64Property(props, vr::Prop_SupportedButtons_Uint64, supportedButtons);

    // Give SteamVR a hint at what hand this controller is for
    if (this->handedness_ == Handedness::LEFT) {
        GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_LeftHand);
    }
    else if (this->handedness_ == Handedness::RIGHT) {
        GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_RightHand);
    }
    else {
        GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_OptOut);
    }

    // this file tells the UI what to show the user for binding this controller as well as what default bindings should
    // be for legacy or other apps
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{htc}/input/vive_controller_profile.json");

    //  Buttons handles
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/application_menu/click", &application_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/grip/click", &grip_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/system/click", &system_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trackpad/click", &trackpad_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trackpad/touch", &trackpad_touch_component_);

    // Analog handles
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trackpad/x", &trackpad_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trackpad/y", &trackpad_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trigger/value", &trigger_value_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

    GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_TrackPad);

    // create our haptic component
    GetDriver()->GetInput()->CreateHapticComponent(props, "/output/haptic", &haptic_component_);

    return vr::EVRInitError::VRInitError_None;
}

void JoyconVrDriver::ControllerDevice::Deactivate()
{
    this->device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void JoyconVrDriver::ControllerDevice::EnterStandby()
{
}

void* JoyconVrDriver::ControllerDevice::GetComponent(const char* pchComponentNameAndVersion)
{
    return nullptr;
}

void JoyconVrDriver::ControllerDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t JoyconVrDriver::ControllerDevice::GetPose()
{
    return last_pose_;
}

void JoyconVrDriver::ControllerDevice::Log(std::string message)
{
    std::string message_endl = message + "\n";
    vr::VRDriverLog()->Log(message_endl.c_str());
}
