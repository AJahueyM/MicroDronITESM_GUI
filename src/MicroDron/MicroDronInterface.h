//
// Created by abiel on 8/19/20.
//

#ifndef MICRODRONITESM_GUI_MICRODRONINTERFACE_H
#define MICRODRONITESM_GUI_MICRODRONINTERFACE_H

#include "SimplePID.h"

enum class ConnectionState{
    SettingUp,
    Connecting,
    Connected
};

class MicroDronInterface {
public:
    /**
     * Retrieves last valid Pitch PID read from drone
     * @return
     */
    virtual SimplePID getPitchPid() const = 0;

    /**
    * Retrieves last valid Roll PID read from drone
    * @return
    */
    virtual SimplePID getRollPid() const = 0;

    /**
    * Retrieves last valid Yaw PID read from drone
    * @return
    */
    virtual SimplePID getYawPid() const = 0;

    /**
    * Retrieves last valid Height PID read from drone
    * @return
    */
    virtual SimplePID getHeightPid() const = 0;

    /**
    * Attempts to send a new pitch PID to drone
    * @return
    */
    virtual void setPitchPid(SimplePID pitchPid) = 0;

    /**
    * Attempts to send a new roll PID to drone
    * @return
    */
    virtual void setRollPid(SimplePID rollPid) = 0;

    /**
    * Attempts to send a new yaw PID to drone
    * @return
    */
    virtual void setYawPid(SimplePID yawPid) = 0;

    /**
    * Attempts to send a new height PID to drone
    * @return
    */
    virtual void setHeightPid(SimplePID heightPid) = 0;

    /**
    * Updates heartbeat of program, the drone keeps track of how often it is received, if it does not
     * receive this message often enough the dron will stop for safety.
    * @return
    */
    virtual void sendHeartBeat() = 0;

    /**
     * Retrieve last valid pitch read from drone
     * @return
     */
    virtual float getPitch() const = 0;

    /**
     * Retrieve last valid roll read from drone
     * @return
     */
    virtual float getRoll() const = 0;

    /**
     * Retrieve last valid yaw read from drone
     * @return
     */
    virtual float getYaw() const = 0;

    /**
     * Retrieve last valid height read from drone
     * @return
     */
    virtual float getHeight() const = 0;

    /**
     * Retrieve last valid operation mode read from drone
     * @return
     */
    virtual int getMode() const = 0;

    /**
     * Retrieve last K value read from drone, this represents the biggest motor output the drone will attempt to use
     * Range from 0 to 5000
     * @return
     */
    virtual float getK() const = 0;

    /**
     * Retrieve last motor1 output read from drone
     * @return
     */
    virtual float getMotorOutput1() const = 0;

    /**
     * Retrieve last motor2 output read from drone
     * @return
     */
    virtual float getMotorOutput2() const = 0;

    /**
     * Retrieve last motor3 output read from drone
     * @return
     */
    virtual float getMotorOutput3() const = 0;

    /**
     * Retrieve last motor4 output read from drone
     * @return
     */
    virtual float getMotorOutput4() const = 0;

    /**
     * Attempts to send all manual motor outputs at once
     * Range from 0 to 5000
     * @return
     */
    virtual void setAllMotorOutput(float output) = 0;

    /**
     * Attempts to send individual motor outputs to all motors
     * Range from 0 to 5000
     * @param output1
     * @param output2
     * @param output3
     * @param output4
     */
    virtual void setAllMotorOutput(float output1, float output2, float output3, float output4) = 0;

    /**
     * Attempts to update the PID Setpoints on the drone
     * @param pitch
     * @param roll
     * @param yaw
     * @param height
     */
    virtual void setSetpoints(float pitch, float roll, float yaw, float height) = 0;

    /**
     * Attempts to update K value, this represents the biggest motor output the drone will attempt to use
     * @param pitch
     * @param roll
     * @param yaw
     * @param height
     */
    virtual void setK(float newK) = 0;

    /**
     * Immediately send command to stop all motors
     */
    virtual void emergencyStop() = 0;

    /**
     * Report whether a valid message has been read from the drone recently
     * @return
     */
    virtual bool isConnected() const = 0;

    /**
     * Retrieve the time between heartbeats that the drone has received
     * @return
     */
    virtual float getHeartbeatTime() const = 0;
};

#endif //MICRODRONITESM_GUI_MICRODRONINTERFACE_H
