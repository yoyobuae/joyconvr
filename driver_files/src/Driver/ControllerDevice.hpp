#pragma once

#include "cmath_fix.h"

#include <Driver/IVRDevice.hpp>
#include <Native/DriverFactory.hpp>

namespace JoyconVrDriver {
    class ControllerDevice : public IVRDevice {
        public:

            enum class Handedness {
                LEFT,
                RIGHT,
                ANY
            };

            ControllerDevice(std::string serial, Handedness handedness = Handedness::ANY);
            ~ControllerDevice() = default;

            // Inherited via IVRDevice
            virtual std::string GetSerial() override;
            virtual void Update() override;
            virtual vr::TrackedDeviceIndex_t GetDeviceIndex() override;
            virtual DeviceType GetDeviceType() override;
            virtual Handedness GetHandedness();

            virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
            virtual void Deactivate() override;
            virtual void EnterStandby() override;
            virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
            virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
            virtual vr::DriverPose_t GetPose() override;

            virtual void Log(std::string message);
    private:
        vr::TrackedDeviceIndex_t device_index_ = vr::k_unTrackedDeviceIndexInvalid;
        std::string serial_;
        Handedness handedness_;

        vr::DriverPose_t last_pose_ = IVRDevice::MakeDefaultPose();

        bool did_vibrate_ = false;
        float vibrate_anim_state_ = 0.f;
        int c = 0;

        bool auto_click_left_trackpad_request = false;
        bool auto_click_left_trackpad = false;
        bool auto_click_right_trackpad_request = false;
        bool auto_click_right_trackpad = false;

        vr::VRInputComponentHandle_t application_button_click_component_ = 0;
        vr::VRInputComponentHandle_t grip_button_click_component_ = 0;
        vr::VRInputComponentHandle_t system_button_click_component_ = 0;
        vr::VRInputComponentHandle_t a_button_click_component_ = 0;
        vr::VRInputComponentHandle_t b_button_click_component_ = 0;
        vr::VRInputComponentHandle_t x_button_click_component_ = 0;
        vr::VRInputComponentHandle_t y_button_click_component_ = 0;
        vr::VRInputComponentHandle_t left_button_click_component_ = 0;
        vr::VRInputComponentHandle_t down_button_click_component_ = 0;
        vr::VRInputComponentHandle_t up_button_click_component_ = 0;
        vr::VRInputComponentHandle_t right_button_click_component_ = 0;
        vr::VRInputComponentHandle_t left_trackpad_button_click_component_ = 0;
        vr::VRInputComponentHandle_t left_trackpad_touch_component_ = 0;
        vr::VRInputComponentHandle_t left_trackpad_x_component_ = 0;
        vr::VRInputComponentHandle_t left_trackpad_y_component_ = 0;
        vr::VRInputComponentHandle_t right_trackpad_button_click_component_ = 0;
        vr::VRInputComponentHandle_t right_trackpad_touch_component_ = 0;
        vr::VRInputComponentHandle_t right_trackpad_x_component_ = 0;
        vr::VRInputComponentHandle_t right_trackpad_y_component_ = 0;
        vr::VRInputComponentHandle_t trigger_value_component_ = 0;
        vr::VRInputComponentHandle_t haptic_component_ = 0;
        vr::VRInputComponentHandle_t skeleton_component_ = 0;
    };
};
