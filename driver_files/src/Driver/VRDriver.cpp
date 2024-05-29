#include "apriltagrpc.capnp.hpp"
#include <kj/debug.h>
#include <capnp/ez-rpc.h>
#include <capnp/message.h>
#include <cmath>
#include <thread>

#include "VRDriver.hpp"
#include <Driver/ControllerDevice.hpp>

#define ENABLE_HMD

extern double leftPose[7];
extern double rightPose[7];

vr::EVRInitError JoyconVrDriver::VRDriver::Init(vr::IVRDriverContext* pDriverContext)
{
    // Perform driver context initialisation
    if (vr::EVRInitError init_error = vr::InitServerDriverContext(pDriverContext); init_error != vr::EVRInitError::VRInitError_None) {
        return init_error;
    }

    Log("Activating JoyconVR Driver...");


#ifdef ENABLE_HMD
    {
        Log("joyconvr: Activating HMD");
        m_pNullHmdLatest = new HMDDevice();
        vr::VRServerDriverHost()->TrackedDeviceAdded(m_pNullHmdLatest->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, m_pNullHmdLatest);
    }
#endif

    {
        std::shared_ptr<IVRDevice> addtracker;
        addtracker = std::make_shared<ControllerDevice>("JOYVR-000001", ControllerDevice::Handedness::LEFT);
        this->AddDevice(addtracker);
    }

    {
        std::shared_ptr<IVRDevice> addtracker;
        addtracker = std::make_shared<ControllerDevice>("JOYVR-000002", ControllerDevice::Handedness::RIGHT);
        this->AddDevice(addtracker);
    }

    std::thread rpcThread(&JoyconVrDriver::VRDriver::RpcThread, this);
    rpcThread.detach();

    Log("JoyconVR Driver Loaded Successfully");

	return vr::VRInitError_None;
}

void JoyconVrDriver::VRDriver::Cleanup()
{
#ifdef ENABLE_HMD
    delete m_pNullHmdLatest;
    m_pNullHmdLatest = NULL;
#endif
}



void JoyconVrDriver::VRDriver::RpcThread()
{
    auto deadline = std::chrono::steady_clock::now();
    deadline += std::chrono::seconds((uint64_t) (5.0));
    std::this_thread::sleep_until(deadline);
    deadline = std::chrono::steady_clock::now();

    capnp::EzRpcClient client("192.168.9.130:54321");
    AprilTagRpc::Client apriltagrpc = client.getMain<AprilTagRpc>();
    auto& waitScope = client.getWaitScope();

    Log("RpcThread: thread staring...");
    while (true) {

        auto leftRequest = apriltagrpc.getTrackerPoseRequest();
        auto rightRequest = apriltagrpc.getTrackerPoseRequest();
        leftRequest.setIndex(8);
        rightRequest.setIndex(9);
        leftRequest.setTimeOffset(0.0);
        rightRequest.setTimeOffset(0.0);
        auto leftPromise = leftRequest.send();
        auto rightPromise = rightRequest.send();
        deadline += std::chrono::nanoseconds((uint64_t) (1e9 / 60.f));
        std::this_thread::sleep_until(deadline);
        auto leftResponse = leftPromise.wait(waitScope);
        auto rightResponse = rightPromise.wait(waitScope);

        leftPose[0] = leftResponse.getPose().getX();
        leftPose[1] = leftResponse.getPose().getY();
        leftPose[2] = leftResponse.getPose().getZ();
        leftPose[3] = leftResponse.getPose().getQw();
        leftPose[4] = leftResponse.getPose().getQx();
        leftPose[5] = leftResponse.getPose().getQy();
        leftPose[6] = leftResponse.getPose().getQz();

        rightPose[0] = rightResponse.getPose().getX();
        rightPose[1] = rightResponse.getPose().getY();
        rightPose[2] = rightResponse.getPose().getZ();
        rightPose[3] = rightResponse.getPose().getQw();
        rightPose[4] = rightResponse.getPose().getQx();
        rightPose[5] = rightResponse.getPose().getQy();
        rightPose[6] = rightResponse.getPose().getQz();
    }
}

void JoyconVrDriver::VRDriver::RunFrame()
{
#ifdef ENABLE_HMD
    if (m_pNullHmdLatest) {
        m_pNullHmdLatest->RunFrame();
    }
#endif

    // Collect events
    vr::VREvent_t event;
    std::vector<vr::VREvent_t> events;
    while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event)))
    {
        events.push_back(event);
    }
    this->openvr_events_ = events;

    for (auto& device : this->devices_)
        device->Update();
}

bool JoyconVrDriver::VRDriver::ShouldBlockStandbyMode()
{
    return false;
}

void JoyconVrDriver::VRDriver::EnterStandby()
{
}

void JoyconVrDriver::VRDriver::LeaveStandby()
{
}

std::vector<std::shared_ptr<JoyconVrDriver::IVRDevice>> JoyconVrDriver::VRDriver::GetDevices()
{
    return this->devices_;
}

std::vector<vr::VREvent_t> JoyconVrDriver::VRDriver::GetOpenVREvents()
{
    return this->openvr_events_;
}

std::chrono::milliseconds JoyconVrDriver::VRDriver::GetLastFrameTime()
{
    return this->frame_timing_;
}

bool JoyconVrDriver::VRDriver::AddDevice(std::shared_ptr<IVRDevice> device)
{
    vr::ETrackedDeviceClass openvr_device_class;
    // Remember to update this switch when new device types are added
    switch (device->GetDeviceType()) {
        case DeviceType::CONTROLLER:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_Controller;
            break;
        case DeviceType::HMD:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_HMD;
            break;
        case DeviceType::TRACKER:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker;
            break;
        case DeviceType::TRACKING_REFERENCE:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference;
            break;
        default:
            return false;
    }
    bool result = vr::VRServerDriverHost()->TrackedDeviceAdded(device->GetSerial().c_str(), openvr_device_class, device.get());
    if(result)
        this->devices_.push_back(device);
    return result;
}

JoyconVrDriver::SettingsValue JoyconVrDriver::VRDriver::GetSettingsValue(std::string key)
{
    vr::EVRSettingsError err = vr::EVRSettingsError::VRSettingsError_None;
    int int_value = vr::VRSettings()->GetInt32(settings_key_.c_str(), key.c_str(), &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return int_value;
    }
    err = vr::EVRSettingsError::VRSettingsError_None;
    float float_value = vr::VRSettings()->GetFloat(settings_key_.c_str(), key.c_str(), &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return float_value;
    }
    err = vr::EVRSettingsError::VRSettingsError_None;
    bool bool_value = vr::VRSettings()->GetBool(settings_key_.c_str(), key.c_str(), &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return bool_value;
    }
    std::string str_value;
    str_value.reserve(1024);
    vr::VRSettings()->GetString(settings_key_.c_str(), key.c_str(), str_value.data(), 1024, &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return str_value;
    }
    err = vr::EVRSettingsError::VRSettingsError_None;

    return SettingsValue();
}

void JoyconVrDriver::VRDriver::Log(std::string message)
{
    std::string message_endl = message + "\n";
    vr::VRDriverLog()->Log(message_endl.c_str());
}

vr::IVRDriverInput* JoyconVrDriver::VRDriver::GetInput()
{
    return vr::VRDriverInput();
}

vr::CVRPropertyHelpers* JoyconVrDriver::VRDriver::GetProperties()
{
    return vr::VRProperties();
}

vr::IVRServerDriverHost* JoyconVrDriver::VRDriver::GetDriverHost()
{
    return vr::VRServerDriverHost();
}

