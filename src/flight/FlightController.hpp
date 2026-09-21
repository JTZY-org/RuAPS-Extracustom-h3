#pragma once

#include "Public/PLGUserDefine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <vector>

namespace FlightProtocol
{
    constexpr uint8_t kCmdArm = 0xCB;
    constexpr uint8_t kCmdArmAlt = 0xCC;
    constexpr uint8_t kCmdLand = 0xB0;
    constexpr uint8_t kCmdLandAlt = 0xB1;
    constexpr uint8_t kCmdPosRelHoriz = 0xD1;
    constexpr uint8_t kCmdPosVert = 0xD2;
    constexpr uint8_t kCmdGimbal = 0xFB;

    // 0xFB CONTROLDATA: 00 FB + tilt_u16_le + pan_u16_le
    // Change pin numbers to match hardware wiring (APMControllerServo).
    constexpr int kGimbalTiltPin = 8; // ch1 = up/down
    constexpr int kGimbalPanPin = 9;  // ch2 = left/right
    constexpr int kServoPwmMin = 1000;
    constexpr int kServoPwmMax = 2000;

    constexpr double kLandAltCm = 8.0;
    constexpr int kLandStableMs = 800;
    constexpr double kHorizArriveCm = 10.0;
    constexpr double kVertArriveCm = 5.0;
}

class FlightController
{
public:
    enum FlightState
    {
        STATE_IDLE,
        STATE_PENDING_ARM,
        STATE_PENDING_DISARM,
        STATE_LANDING
    };

    FlightState m_state = STATE_IDLE;
    std::chrono::steady_clock::time_point m_lowAltStartTime{};

    bool m_horizActive = false;
    int m_targetX = 0;
    int m_targetY = 0;
    int m_horizHoldAlt = 0;

    bool m_vertActive = false;
    int m_targetAlt = 0;
    int m_vertHoldX = 0;
    int m_vertHoldY = 0;

public:
    void updateState(UserAppData &data)
    {
        if (m_state == STATE_PENDING_ARM)
        {
            std::cout << "[FlightController] Executing Action: ARM\n";
            if (data.APMData.APMControllerARM)
            {
                data.APMData.APMControllerARM();
            }
            m_state = STATE_IDLE;
            return;
        }

        if (m_state == STATE_PENDING_DISARM)
        {
            std::cout << "[FlightController] Executing Action: DISARM\n";
            clearPositionModes();
            if (data.APMData.APMControllerSpeed)
            {
                data.APMData.APMControllerSpeed(0, 0, 0, 0.0f);
            }
            if (data.APMData.APMControllerDISARM)
            {
                data.APMData.APMControllerDISARM();
            }
            m_state = STATE_IDLE;
            return;
        }

        if (m_state == STATE_LANDING)
        {
            updateLanding(data);
            return;
        }

        updatePositionHold(data);
    }

    void processCmd(UserAppData &data)
    {
        if (data.getBroadcastRecv == nullptr)
        {
            return;
        }

        auto queue = data.getBroadcastRecv();
        while (!queue.empty())
        {
            auto packet = std::move(queue.front());
            queue.pop_front();

            logPacket(packet);

            if (tryApplyGimbal(data, packet))
            {
                continue;
            }

            if (packet.empty())
            {
                continue;
            }

            const uint8_t cmd = packet[0];
            const bool isArmCmd =
                (cmd == FlightProtocol::kCmdArm || cmd == FlightProtocol::kCmdArmAlt) &&
                packet.size() >= 2 &&
                (packet[1] == 0x00 || packet[1] == 0x01);
            const bool isLandStart =
                (cmd == FlightProtocol::kCmdLand || cmd == FlightProtocol::kCmdLandAlt) &&
                packet.size() >= 2 && packet[1] == 0x01;

            if (!isArmCmd && !gateAllowsCommand(data, isLandStart))
            {
                std::cout << "[FlightController] COMMAND REJECTED - State conflict or disarmed.\n";
                continue;
            }

            if (isArmCmd)
            {
                handleArmDisarm(packet[1]);
            }
            else if ((cmd == FlightProtocol::kCmdLand || cmd == FlightProtocol::kCmdLandAlt) &&
                     packet.size() >= 2)
            {
                handleLand(packet[1]);
            }
            else if (cmd == FlightProtocol::kCmdPosRelHoriz)
            {
                handlePosHoriz(data, packet);
            }
            else if (cmd == FlightProtocol::kCmdPosVert)
            {
                handlePosVert(data, packet);
            }
            else
            {
                std::cout << "[FlightController] Unhandled cmd=0x" << std::hex << std::uppercase
                          << static_cast<int>(cmd) << std::dec << "\n";
            }
        }
    }

private:
    static void logPacket(const std::vector<uint8_t> &packet)
    {
        std::cout << "[FlightController] Recv Broadcast (HEX):";
        for (uint8_t byte : packet)
        {
            std::cout << ' ' << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                      << static_cast<int>(byte);
        }
        std::cout << std::dec << std::endl;
    }

    static bool isDisarmed(const UserAppData &data)
    {
        // _SYS_DISARMFlag: true means locked / disarmed
        if (data.APMData._SYS_DISARMFlag == nullptr)
        {
            return true;
        }
        return *data.APMData._SYS_DISARMFlag;
    }

    bool gateAllowsCommand(const UserAppData &data, bool isLandStart) const
    {
        if (isDisarmed(data))
        {
            return false;
        }
        if (m_state == STATE_IDLE)
        {
            return true;
        }
        // Allow duplicate LAND while already landing
        return isLandStart && m_state == STATE_LANDING;
    }

    void clearPositionModes()
    {
        m_horizActive = false;
        m_vertActive = false;
    }

    static float readYaw(const UserAppData &data)
    {
        if (data.APMData._ATT_EulerAngleYawV != nullptr)
        {
            return *data.APMData._ATT_EulerAngleYawV;
        }
        return 0.0f;
    }

    static void readNav(const UserAppData &data, double &x, double &y, double &z)
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        if (data.APMData._NAV_Relative_Pos[0] != nullptr)
        {
            x = *data.APMData._NAV_Relative_Pos[0];
        }
        if (data.APMData._NAV_Relative_Pos[1] != nullptr)
        {
            y = *data.APMData._NAV_Relative_Pos[1];
        }
        if (data.APMData._NAV_Relative_Pos[2] != nullptr)
        {
            z = *data.APMData._NAV_Relative_Pos[2];
        }
    }

    static void setPosition(UserAppData &data, int x, int y, int z, bool resetHome)
    {
        if (data.APMData.APMControllerPosition)
        {
            data.APMData.APMControllerPosition(x, y, z, readYaw(data), resetHome);
        }
    }

    void handleArmDisarm(uint8_t action)
    {
        if (action == 0x01)
        {
            m_state = STATE_PENDING_ARM;
            return;
        }
        if (action == 0x00)
        {
            // Cancel landing / pos modes immediately; physical disarm in updateState
            m_lowAltStartTime = {};
            clearPositionModes();
            m_state = STATE_PENDING_DISARM;
        }
    }

    void handleLand(uint8_t action)
    {
        if (action == 0x01)
        {
            if (m_state != STATE_LANDING)
            {
                clearPositionModes();
                m_state = STATE_LANDING;
                m_lowAltStartTime = {};
            }
            return;
        }
        if (action == 0x00 && m_state == STATE_LANDING)
        {
            std::cout << "[FlightController] Landing canceled\n";
            m_state = STATE_IDLE;
            m_lowAltStartTime = {};
        }
    }

    static bool tryApplyGimbal(UserAppData &data, const std::vector<uint8_t> &packet)
    {
        // CONTROLDATA: reverse(u8)=0, mark(u8)=0xFB, tilt(u16 LE), pan(u16 LE)
        if (packet.size() < 6)
        {
            return false;
        }
        if (packet[0] != 0x00 || packet[1] != FlightProtocol::kCmdGimbal)
        {
            return false;
        }

        const uint16_t tiltRaw =
            static_cast<uint16_t>(packet[2]) | (static_cast<uint16_t>(packet[3]) << 8);
        const uint16_t panRaw =
            static_cast<uint16_t>(packet[4]) | (static_cast<uint16_t>(packet[5]) << 8);
        const int tiltPwm = std::clamp(static_cast<int>(tiltRaw), FlightProtocol::kServoPwmMin,
                                       FlightProtocol::kServoPwmMax);
        const int panPwm = std::clamp(static_cast<int>(panRaw), FlightProtocol::kServoPwmMin,
                                      FlightProtocol::kServoPwmMax);

        std::cout << "[FlightController] Gimbal tilt pin=" << FlightProtocol::kGimbalTiltPin
                  << " pwm=" << tiltPwm << " pan pin=" << FlightProtocol::kGimbalPanPin
                  << " pwm=" << panPwm << "\n";
        if (data.APMData.APMControllerServo)
        {
            data.APMData.APMControllerServo(FlightProtocol::kGimbalTiltPin, tiltPwm);
            data.APMData.APMControllerServo(FlightProtocol::kGimbalPanPin, panPwm);
        }
        return true;
    }

    void handlePosHoriz(UserAppData &data, const std::vector<uint8_t> &packet)
    {
        // <Bhh>: cmd, x(int16 LE), y(int16 LE)
        if (packet.size() < 5)
        {
            std::cout << "[FlightController] D1 packet too short\n";
            return;
        }

        const int16_t targetX =
            static_cast<int16_t>(static_cast<uint16_t>(packet[1]) |
                                 (static_cast<uint16_t>(packet[2]) << 8));
        const int16_t targetY =
            static_cast<int16_t>(static_cast<uint16_t>(packet[3]) |
                                 (static_cast<uint16_t>(packet[4]) << 8));

        double curX = 0.0;
        double curY = 0.0;
        double curZ = 0.0;
        readNav(data, curX, curY, curZ);

        m_targetX = targetX;
        m_targetY = targetY;
        m_horizHoldAlt = static_cast<int>(std::lround(curZ));
        m_horizActive = true;
        m_vertActive = false;

        std::cout << "[FlightController] D1 target=(" << m_targetX << "," << m_targetY
                  << ") hold_alt=" << m_horizHoldAlt << "\n";
        setPosition(data, m_targetX, m_targetY, m_horizHoldAlt, false);
    }

    void handlePosVert(UserAppData &data, const std::vector<uint8_t> &packet)
    {
        // <Bh>: cmd, alt(int16 LE)
        if (packet.size() < 3)
        {
            std::cout << "[FlightController] D2 packet too short\n";
            return;
        }

        const int16_t targetAlt =
            static_cast<int16_t>(static_cast<uint16_t>(packet[1]) |
                                 (static_cast<uint16_t>(packet[2]) << 8));

        double curX = 0.0;
        double curY = 0.0;
        double curZ = 0.0;
        readNav(data, curX, curY, curZ);

        m_horizActive = false;
        m_targetAlt = targetAlt;
        m_vertHoldX = static_cast<int>(std::lround(curX));
        m_vertHoldY = static_cast<int>(std::lround(curY));
        m_vertActive = true;

        std::cout << "[FlightController] D2 target_alt=" << m_targetAlt << " hold_xy=("
                  << m_vertHoldX << "," << m_vertHoldY << ")\n";
        setPosition(data, m_vertHoldX, m_vertHoldY, m_targetAlt, false);
    }

    void updateLanding(UserAppData &data)
    {
        if (m_lowAltStartTime.time_since_epoch().count() == 0)
        {
            std::cout << "[FlightController] Executing Action: START LANDING (Move to Z=0)\n";
            setPosition(data, 0, 0, 0, true);
            m_lowAltStartTime = std::chrono::steady_clock::now();
        }

        double currentAlt = 999.0;
        if (data.APMData._NAV_Relative_Pos[2] != nullptr)
        {
            currentAlt = *data.APMData._NAV_Relative_Pos[2];
        }

        if (std::abs(currentAlt) <= FlightProtocol::kLandAltCm)
        {
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - m_lowAltStartTime)
                                     .count();
            std::cout << "[FlightController] Landing check. Elapsed: " << elapsed << " ms / "
                      << FlightProtocol::kLandStableMs << " ms (Alt: " << currentAlt << " cm)\n";

            if (elapsed >= FlightProtocol::kLandStableMs)
            {
                std::cout << "[FlightController] Ground touchdown confirmed. Disarming...\n";
                if (data.APMData.APMControllerSpeed)
                {
                    data.APMData.APMControllerSpeed(0, 0, 0, 0.0f);
                }
                if (data.APMData.APMControllerDISARM)
                {
                    data.APMData.APMControllerDISARM();
                }
                clearPositionModes();
                m_state = STATE_IDLE;
                m_lowAltStartTime = {};
            }
        }
        else
        {
            m_lowAltStartTime = std::chrono::steady_clock::now();
        }
    }

    void updatePositionHold(UserAppData &data)
    {
        if (m_state != STATE_IDLE)
        {
            return;
        }

        double curX = 0.0;
        double curY = 0.0;
        double curZ = 0.0;
        readNav(data, curX, curY, curZ);

        if (m_horizActive)
        {
            const double dist = std::hypot(curX - m_targetX, curY - m_targetY);
            if (dist <= FlightProtocol::kHorizArriveCm)
            {
                std::cout << "[FlightController] Horiz arrived dist=" << dist
                          << "cm. Resetting home.\n";
                setPosition(data, 0, 0, m_horizHoldAlt, true);
                m_horizActive = false;
                setPosition(data, 0, 0, m_horizHoldAlt, false);
            }
            else
            {
                setPosition(data, m_targetX, m_targetY, m_horizHoldAlt, false);
            }
            return;
        }

        if (m_vertActive)
        {
            setPosition(data, m_vertHoldX, m_vertHoldY, m_targetAlt, false);
            if (std::abs(curZ - m_targetAlt) <= FlightProtocol::kVertArriveCm)
            {
                std::cout << "[FlightController] Vert arrived cur=" << curZ
                          << "cm target=" << m_targetAlt << "cm\n";
                m_vertActive = false;
            }
        }
    }
};
