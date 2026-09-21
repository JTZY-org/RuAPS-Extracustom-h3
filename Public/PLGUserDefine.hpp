#pragma once
#include <functional>
#include <vector>
#include <deque>
#include "V4L2ToolKit.hpp"

#include <vector>
#include <mutex>
#include <stdexcept>
#include <optional>

enum BoxFailedSafeFlag
{
    _flag_FailedSafe_RCLose = 1 << 0,
    _flag_FailedSafe_FakeRCLose = 1 << 1,
    _flag_FailedSafe_AngleLimit = 1 << 2,
    _flag_FailedSafe_MPUNoRespond = 1 << 3,
    _flag_FailedSafe_ESCNoRespond = 1 << 4,
    _flag_FailedSafe_SpeedReferenceZ = 1 << 8,
    _flag_FailedSafe_SpeedReferenceZT = 1 << 9,
    _flag_FailedSafe_SpeedReferenceX = 1 << 10,
    _flag_FailedSafe_SpeedReferenceXT = 1 << 11,
    _flag_FailedSafe_SpeedReferenceY = 1 << 12,
    _flag_FailedSafe_SpeedReferenceYT = 1 << 13,

    _flag_PreARMFailed_GyroNotStable = 1 << 0,
    _flag_PreARMFailed_AngleNotSync = 1 << 1,
    _flag_PreARMFailed_NavigationNotSync = 1 << 2,
    _flag_PreARMFailed_AccelNotStable = 1 << 3,
    _flag_PreARMFailed_DTNotSync = 1 << 4,
    _flag_PreARMFailed_TEMPTOOHIGH = 1 << 5,
};

struct ControllerData
{
    // bool isAPMDown;

    int *_Accel_ClippedTimes;
    float *_Accel_Accelration[3]; // Body frame static accel, in cm/ss
    float *_Accel_VIBE[3];
    int *_Accel_RawG[3];  // 2048lsb/g
    float *_Accel_GForce; // 2048lsb/g

    float *_ATT_Quaterion[4];
    float *_ATT_EulerAngle[3]; // deg,yaw 0-360deg, If mag avaliable, yaw will be mag angle
    float *_Gyro_AngleRate[3]; // deg/s
    int *_Mag_RawL[3];
    float *_ATT_EulerAngleYawV; // yaw in +-180degs

    float *_Baro_Temp;             // deg in C
    float *_Baro_PressureHPA;      // in HPA
    float *_Baro_AGLAltitudeCM;    // if AGLHold enble, will be override by Snoar when avaliable
    double *_RangeFinder_AGLAltCM; // snoar Altitude input in cm

    bool *_SYS_DISARMFlag;       // becareful this is disarm
    int *_SYS_TimeInfo[10];      // TODO: current not provide
    uint16_t *_SYS_PreARMFlag;   // check BoxFailedSafeFlag
    uint16_t *_SYS_FailSafeFlag; // check BoxFailedSafeFlag
    int *_SYS_APMStatus;         // 1 is init compelete, 2 is boot complete,-1 is start init ,-2 is deinit
    // No Flightmode Provided

    double *_NAV_Speed[3];        // Body Frame speed estimated, in cm/s
    double *_NAV_Global_Speed[2]; // not provide yet
    float *_NAV_Relative_Head;    // not provide yet
    int *_NAV_Global_Pos[3];      // lat and lon, gps alt not provide yet, 1e-7 int
    int *_NAV_Global_HOME[2];     // TODO: not provide yet
    double *_NAV_Relative_Pos[3]; // Body Frame pos estimated, in cm
    int *_NAV_Global_SATCount;
    float *_NAV_Global_Head; // TODO: not provide yet
    int *_NAV_GPS_HDOP;      // TODO: not provide yet

    int *_SEN_FLOW_Quality; // 0 - 255 large better

    int *_RC_Channel_Raw[16]; // RC channel, should be 1000 - 2000
    int *_EF_Channel_Raw[16]; // ESC output final, 1000 - 2000

    float *_GYRO_CYCLE_TIME;        // att thread time, in us
    float *_Battery_Voltage;        // in V, 4S is 16.4V
    float *_Battery_Voltage_Single; // single bat vol, mind be 2.5~4.2V

    double *_CPU_Core_Temp; // deg in C, 85deg will trigle prearm failed protect

    // *all navigation is Body frame, Dont caculate as earth frame
    void (*APMControllerARM)(void);                                                   // In UserAuto, it will take off to 50cm or setting takeoff altitude
    void (*APMControllerDISARM)(void);                                                // lock down ESC immediately
    void (*APMControllerGPSPosition)(int lat, int lng, float yawdeg, int alt);        // format in 1e-7 int, check _NAV_Global_Pos, yawdeg using _ATT_EulerAngleYawV, +-180deg
    void (*APMControllerPosition)(int x, int y, int z, float yawdeg, bool resetHome); // In CM, if homereset is true, _NAV_Relative_Pos will be 0. yawdeg using _ATT_EulerAngleYawV, +-180deg
    void (*APMControllerSpeed)(int x, int y, int z, float yawdegs);                   // In CM/S, yawdegs in deg/s
    void (*APMControllerServo)(int pin, int PWMInUs);                                 // you can override Motor output for fun, PWM is 1000 - 2000, If Oneshot enable, only oneshot here, use PWM for servo
};

struct UserAppData
{
    ControllerData APMData;
    V4L2Tools::V4l2Data cameraFrame;
    void (*pushBroadcastData)(std::vector<uint8_t>);
    std::deque<std::vector<uint8_t>> (*getBroadcastRecv)();
};