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

#include "HandPoses.hpp"

enum HandSkeletonBone
{
        eBone_Root = 0,
        eBone_Wrist,
        eBone_Thumb0,
        eBone_Thumb1,
        eBone_Thumb2,
        eBone_Thumb3,
        eBone_IndexFinger0,
        eBone_IndexFinger1,
        eBone_IndexFinger2,
        eBone_IndexFinger3,
        eBone_IndexFinger4,
        eBone_MiddleFinger0,
        eBone_MiddleFinger1,
        eBone_MiddleFinger2,
        eBone_MiddleFinger3,
        eBone_MiddleFinger4,
        eBone_RingFinger0,
        eBone_RingFinger1,
        eBone_RingFinger2,
        eBone_RingFinger3,
        eBone_RingFinger4,
        eBone_PinkyFinger0,
        eBone_PinkyFinger1,
        eBone_PinkyFinger2,
        eBone_PinkyFinger3,
        eBone_PinkyFinger4,
        eBone_Aux_Thumb,
        eBone_Aux_IndexFinger,
        eBone_Aux_MiddleFinger,
        eBone_Aux_RingFinger,
        eBone_Aux_PinkyFinger,
        eBone_Count
};

vr::BoneIndex_t commonBones[] = {
        eBone_Root,
        eBone_Wrist,
};

vr::BoneIndex_t thumbBones[] = {
        eBone_Thumb0,
        eBone_Thumb1,
        eBone_Thumb2,
        eBone_Thumb3,
        eBone_Aux_Thumb,
};

vr::BoneIndex_t indexBones[] = {
        eBone_IndexFinger0,
        eBone_IndexFinger1,
        eBone_IndexFinger2,
        eBone_IndexFinger3,
        eBone_IndexFinger4,
        eBone_Aux_IndexFinger,
};

vr::BoneIndex_t middleBones[] = {
        eBone_MiddleFinger0,
        eBone_MiddleFinger1,
        eBone_MiddleFinger2,
        eBone_MiddleFinger3,
        eBone_MiddleFinger4,
        eBone_Aux_MiddleFinger,
};

vr::BoneIndex_t ringBones[] = {
        eBone_RingFinger0,
        eBone_RingFinger1,
        eBone_RingFinger2,
        eBone_RingFinger3,
        eBone_RingFinger4,
        eBone_Aux_RingFinger,
};

vr::BoneIndex_t pinkyBones[] = {
        eBone_PinkyFinger0,
        eBone_PinkyFinger1,
        eBone_PinkyFinger2,
        eBone_PinkyFinger3,
        eBone_PinkyFinger4,
        eBone_Aux_PinkyFinger,
};

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

        if (c > 60)
            Log("Reading from left IMU");
        q_valid = getLeftIMU(q);

        float x = static_cast<float>(getJoyAxis(ABS_X))/32768.0f;
        float y = static_cast<float>(getJoyAxis(ABS_Y))/-32768.0f;
        float norm = hypot(x, y);
        float arct = atan2(y, x);

        // Deadzone and touch calcs
        bool trackpad_touch = norm > 0.1875f;
        float trackpad_norm = norm > 0.25f ? (norm - 0.25f) / 0.75f : 0.0f;
        float trackpad_x = trackpad_norm * cos(arct);
        float trackpad_y = trackpad_norm * sin(arct);

        /*
         * Minus button         /input/application_menu
         * SR button            /input/grip
         * Capture button       /input/system
         * Joystick press       /input/trackpad/click
         *
         * Dpad Left            /input/left
         * Dpad Down            /input/down
         * Dpad Up              /input/up
         * Dpad Right           /input/right
         *
         * Joystick X axis      /input/trackpad/x
         * Joystick Y axis      /input/trackpad/y
         *
         * ZL button            /input/trigger/value
         *
         * SL button            Reset IMU orientation
         * L button             Special functions
         */
        if (getJoyButton(BTN_TL)) {
            if (getJoyButton(BTN_THUMBL)) {
                auto_click_left_trackpad_request = true;
            }
        } else {
            if (auto_click_left_trackpad_request) {
                auto_click_left_trackpad_request = false;
                auto_click_left_trackpad = !auto_click_left_trackpad;
            }
            GetDriver()->GetInput()->UpdateBooleanComponent(application_button_click_component_, getJoyButton(BTN_SELECT), 0);  // Application Menu
            GetDriver()->GetInput()->UpdateBooleanComponent(grip_button_click_component_, getJoyButton(BTN_TRIGGER_HAPPY2), 0); // Grip
            GetDriver()->GetInput()->UpdateBooleanComponent(system_button_click_component_, getJoyButton(BTN_Z), 0);            // System
            GetDriver()->GetInput()->UpdateBooleanComponent(left_trackpad_button_click_component_,
                                                            getJoyButton(BTN_THUMBL) || (trackpad_touch && auto_click_left_trackpad), 0);     // Trackpad

            GetDriver()->GetInput()->UpdateBooleanComponent(left_button_click_component_,  getJoyButton(BTN_DPAD_LEFT), 0);
            GetDriver()->GetInput()->UpdateBooleanComponent(down_button_click_component_,  getJoyButton(BTN_DPAD_DOWN), 0);
            GetDriver()->GetInput()->UpdateBooleanComponent(up_button_click_component_,    getJoyButton(BTN_DPAD_UP), 0);
            GetDriver()->GetInput()->UpdateBooleanComponent(right_button_click_component_, getJoyButton(BTN_DPAD_RIGHT), 0);
        }

        GetDriver()->GetInput()->UpdateBooleanComponent(left_trackpad_touch_component_, trackpad_touch, 0); // Trackpad touch
        GetDriver()->GetInput()->UpdateScalarComponent(left_trackpad_x_component_, trackpad_x, 0);          // Trackpad x
        GetDriver()->GetInput()->UpdateScalarComponent(left_trackpad_y_component_, trackpad_y, 0);          // Trackpad y


        if (getJoyButton(BTN_TL2)) { // Trigger
            GetDriver()->GetInput()->UpdateScalarComponent(trigger_value_component_, 1.0, 0);
        } else {
            GetDriver()->GetInput()->UpdateScalarComponent(trigger_value_component_, 0.0, 0);
        }

        if (getJoyButton(BTN_TRIGGER_HAPPY1)) { // IMU reset
            resetLeftIMU();
        }

#if 1
        vr::VRBoneTransform_t hand_pose[NUM_BONES];

        for (int i = 0; i < sizeof(commonBones)/sizeof(commonBones[0]); i++)
        {
            hand_pose[commonBones[i]] = left_open_hand_pose[commonBones[i]];
        }
        for (int i = 0; i < sizeof(thumbBones)/sizeof(thumbBones[0]); i++)
        {
            hand_pose[thumbBones[i]] = trackpad_touch ? left_closed_hand_pose[thumbBones[i]] : left_open_hand_pose[thumbBones[i]];
        }
        for (int i = 0; i < sizeof(indexBones)/sizeof(indexBones[0]); i++)
        {
            hand_pose[indexBones[i]] = getJoyButton(BTN_TL) ? left_closed_hand_pose[indexBones[i]] : left_open_hand_pose[indexBones[i]];
        }
        for (int i = 0; i < sizeof(middleBones)/sizeof(middleBones[0]); i++)
        {
            hand_pose[middleBones[i]] = getJoyButton(BTN_TL2) ? left_closed_hand_pose[middleBones[i]] : left_open_hand_pose[middleBones[i]];
        }
        for (int i = 0; i < sizeof(ringBones)/sizeof(ringBones[0]); i++)
        {
            hand_pose[ringBones[i]] = getJoyButton(BTN_TRIGGER_HAPPY1) ? left_closed_hand_pose[ringBones[i]] : left_open_hand_pose[ringBones[i]];
        }
        for (int i = 0; i < sizeof(pinkyBones)/sizeof(pinkyBones[0]); i++)
        {
            hand_pose[pinkyBones[i]] = getJoyButton(BTN_TRIGGER_HAPPY1) ? left_closed_hand_pose[pinkyBones[i]] : left_open_hand_pose[pinkyBones[i]];
        }

        GetDriver()->GetInput()->UpdateSkeletonComponent(skeleton_component_, vr::VRSkeletalMotionRange_WithController, hand_pose, NUM_BONES);
        GetDriver()->GetInput()->UpdateSkeletonComponent(skeleton_component_, vr::VRSkeletalMotionRange_WithoutController, hand_pose, NUM_BONES);
#endif
    }
    else if (this->handedness_ == Handedness::RIGHT) {
        pose.vecPosition[0] = 0.2;
        pose.vecPosition[1] = 1.3;
        pose.vecPosition[2] = -0.2;

        if (c > 60)
            Log("Reading from right IMU");
        q_valid = getRightIMU(q);

        float x = static_cast<float>(getJoyAxis(ABS_RX))/32768.0f;
        float y = static_cast<float>(getJoyAxis(ABS_RY))/-32768.0f;
        float norm = hypot(x, y);
        float arct = atan2(y, x);

        // Deadzone and touch calcs
        bool trackpad_touch = norm > 0.1875f;
        float trackpad_norm = norm > 0.25f ? (norm - 0.25f) / 0.75f : 0.0f;
        float trackpad_x = trackpad_norm * cos(arct);
        float trackpad_y = trackpad_norm * sin(arct);

        /*
         * Plus button          /input/application_menu
         * SL button            /input/grip
         * Home button          /input/system
         * Joystick press       /input/trackpad/click
         *
         * A button             /input/a
         * B button             /input/b
         * X button             /input/x
         * Y button             /input/y
         *
         * Joystick X axis      /input/trackpad/x
         * Joystick Y axis      /input/trackpad/y
         *
         * ZR button            /input/trigger/value
         *
         * SR button            Reset IMU orientation
         * R button             Unbound
         */
        if (getJoyButton(BTN_TR)) {
            if (getJoyButton(BTN_THUMBR)) {
                auto_click_right_trackpad_request = true;
            }
        } else {
            if (auto_click_right_trackpad_request) {
                auto_click_right_trackpad_request = false;
                auto_click_right_trackpad = !auto_click_right_trackpad;
            }
            GetDriver()->GetInput()->UpdateBooleanComponent(application_button_click_component_, getJoyButton(BTN_START), 0);   // Application Menu
            GetDriver()->GetInput()->UpdateBooleanComponent(grip_button_click_component_, getJoyButton(BTN_TRIGGER_HAPPY3), 0); // Grip
            GetDriver()->GetInput()->UpdateBooleanComponent(system_button_click_component_, getJoyButton(BTN_MODE), 0);         // System
            GetDriver()->GetInput()->UpdateBooleanComponent(right_trackpad_button_click_component_,
                                                            getJoyButton(BTN_THUMBR) || (trackpad_touch && auto_click_right_trackpad), 0);     // Trackpad

            GetDriver()->GetInput()->UpdateBooleanComponent(a_button_click_component_, getJoyButton(BTN_B), 0); // Annoyingly hid-nintendo driver
            GetDriver()->GetInput()->UpdateBooleanComponent(b_button_click_component_, getJoyButton(BTN_A), 0); // has A and B buttons swapped
            GetDriver()->GetInput()->UpdateBooleanComponent(x_button_click_component_, getJoyButton(BTN_X), 0);
            GetDriver()->GetInput()->UpdateBooleanComponent(y_button_click_component_, getJoyButton(BTN_Y), 0);
        }

        GetDriver()->GetInput()->UpdateBooleanComponent(right_trackpad_touch_component_, trackpad_touch, 0); // Trackpad touch
        GetDriver()->GetInput()->UpdateScalarComponent(right_trackpad_x_component_, trackpad_x, 0);          // Trackpad x
        GetDriver()->GetInput()->UpdateScalarComponent(right_trackpad_y_component_, trackpad_y, 0);          // Trackpad y


        if (getJoyButton(BTN_TR2)) { // Trigger
            GetDriver()->GetInput()->UpdateScalarComponent(trigger_value_component_, 1.0, 0);
        } else {
            GetDriver()->GetInput()->UpdateScalarComponent(trigger_value_component_, 0.0, 0);
        }

        if (getJoyButton(BTN_TRIGGER_HAPPY4)) { // IMU reset
            resetRightIMU();
        }

#if 1
        vr::VRBoneTransform_t hand_pose[NUM_BONES];

        for (int i = 0; i < sizeof(commonBones)/sizeof(commonBones[0]); i++)
        {
            hand_pose[commonBones[i]] = right_open_hand_pose[commonBones[i]];
        }
        for (int i = 0; i < sizeof(thumbBones)/sizeof(thumbBones[0]); i++)
        {
            hand_pose[thumbBones[i]] = trackpad_touch ? right_closed_hand_pose[thumbBones[i]] : right_open_hand_pose[thumbBones[i]];
        }
        for (int i = 0; i < sizeof(indexBones)/sizeof(indexBones[0]); i++)
        {
            hand_pose[indexBones[i]] = getJoyButton(BTN_TR) ? right_closed_hand_pose[indexBones[i]] : right_open_hand_pose[indexBones[i]];
        }
        for (int i = 0; i < sizeof(middleBones)/sizeof(middleBones[0]); i++)
        {
            hand_pose[middleBones[i]] = getJoyButton(BTN_TR2) ? right_closed_hand_pose[middleBones[i]] : right_open_hand_pose[middleBones[i]];
        }
        for (int i = 0; i < sizeof(ringBones)/sizeof(ringBones[0]); i++)
        {
            hand_pose[ringBones[i]] = getJoyButton(BTN_TRIGGER_HAPPY4) ? right_closed_hand_pose[ringBones[i]] : right_open_hand_pose[ringBones[i]];
        }
        for (int i = 0; i < sizeof(pinkyBones)/sizeof(pinkyBones[0]); i++)
        {
            hand_pose[pinkyBones[i]] = getJoyButton(BTN_TRIGGER_HAPPY4) ? right_closed_hand_pose[pinkyBones[i]] : right_open_hand_pose[pinkyBones[i]];
        }
        GetDriver()->GetInput()->UpdateSkeletonComponent(skeleton_component_, vr::VRSkeletalMotionRange_WithController, hand_pose, NUM_BONES);
        GetDriver()->GetInput()->UpdateSkeletonComponent(skeleton_component_, vr::VRSkeletalMotionRange_WithoutController, hand_pose, NUM_BONES);
#endif
    }

    if (c++ > 60) {
        std::stringstream ss;
        ss << "serial: " << this->serial_;
        ss <<" index: " << this->device_index_;
        ss <<" q_valid: " << q_valid;
        ss <<" q[0]: " << q[0];
        ss <<" q[1]: " << q[1];
        ss <<" q[2]: " << q[2];
        ss <<" q[3]: " << q[3];
        Log(ss.str());
        c = 0;
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

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, "joyconvr_controller");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{joyconvr}/input/joyconvr_controller_profile.json");

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "JoyconVR");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ManufacturerName_String, "JoyconVR");
#if 0
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");
#else
    if (this->handedness_ == Handedness::LEFT) {
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "{joyconvr}joyconvr_controller_left_1_0");
    }
    else {
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "{joyconvr}joyconvr_controller_right_1_0");
    }
#endif

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
    // GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{htc}/input/vive_controller_profile.json");

    //  Buttons handles
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/application_menu/click", &application_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/grip/click", &grip_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/system/click", &system_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/a/click", &a_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/b/click", &b_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/x/click", &x_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/left/click", &left_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/down/click", &down_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/up/click", &up_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/right/click", &right_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/lefttrackpad/click", &left_trackpad_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/lefttrackpad/touch", &left_trackpad_touch_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/righttrackpad/click", &right_trackpad_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/righttrackpad/touch", &right_trackpad_touch_component_);

    // Analog handles
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/lefttrackpad/x", &left_trackpad_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/lefttrackpad/y", &left_trackpad_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/righttrackpad/x", &right_trackpad_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/righttrackpad/y", &right_trackpad_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trigger/value", &trigger_value_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

    GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_TrackPad);

    // create our haptic component
    GetDriver()->GetInput()->CreateHapticComponent(props, "/output/haptic", &haptic_component_);

#if 1
    // create skeleton component
    if (this->handedness_ == Handedness::LEFT) {
        GetDriver()->GetInput()->CreateSkeletonComponent(props, "/input/skeleton/left", "/skeleton/hand/left", "/user/hand/left/pose/tip", vr::VRSkeletalTracking_Estimated, nullptr, 0, &skeleton_component_);
    }
    else if (this->handedness_ == Handedness::RIGHT) {
        GetDriver()->GetInput()->CreateSkeletonComponent(props, "/input/skeleton/right", "/skeleton/hand/right", "/user/hand/right/pose/tip", vr::VRSkeletalTracking_Estimated, nullptr, 0, &skeleton_component_);
    }
#endif

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
