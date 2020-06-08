//
// Created by alberto on 5/04/19.
//

#ifndef MICRODRONITESM_GUI_MICRODRONINTERFACE_H
#define MICRODRONITESM_GUI_MICRODRONINTERFACE_H
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <mutex>


struct SimplePID{
    float p = 0;
    float i = 0;
    float d = 0;
    bool clamped = false;
    float maxOutput = 0;
    float minOutput = 0;
    bool continuous = false;
    float maxInput = 0;
    float minInput = 0;
};

/**
 * Class that manages the communications with the MicroDron, it reads the telemetry that the drone sends and
 * allows for its easy access, it also enables the user to control the drone.
 *
 * By: Alberto Jahuey Moncada   A01039835@itesm.mx
 */

enum class ConectionState{
    SettingUp,
    Connecting,
    Connected
};


class MicroDronInterface {
public:
    /**
     * Setups connection to drone, requires an IP address and port, will create a thread that updates
     * communications with the drone
     * @param ipAddress
     * @param port
     */
    MicroDronInterface(std::string  ipAddress, int port);

    /**
     * Stops update thread
     */
    ~MicroDronInterface();


    /**
     * Retrieves last valid Pitch PID read from drone
     * @return
     */
    const SimplePID &getPitchPid() const;

    /**
    * Retrieves last valid Roll PID read from drone
    * @return
    */
    const SimplePID &getRollPid() const;

    /**
    * Retrieves last valid Yaw PID read from drone
    * @return
    */
    const SimplePID &getYawPid() const;

    /**
    * Retrieves last valid Height PID read from drone
    * @return
    */
    const SimplePID &getHeightPid() const;

    /**
    * Attempts to send a new pitch PID to drone
    * @return
    */
    void setPitchPid(SimplePID pitchPid);

    /**
    * Attempts to send a new roll PID to drone
    * @return
    */
    void setRollPid(SimplePID rollPid);

    /**
    * Attempts to send a new yaw PID to drone
    * @return
    */
    void setYawPid(SimplePID yawPid);

    /**
    * Attempts to send a new height PID to drone
    * @return
    */
    void setHeightPid(SimplePID heightPid);

    /**
    * Updates heartbeat of program, the drone keeps track of how often it is received, if it does not
     * receive this message often enough the dron will stop for safety.
    * @return
    */
    void sendHeartBeat();

    /**
     * Retrieve last valid pitch read from drone
     * @return
     */
    float getPitch() const;

    /**
     * Retrieve last valid roll read from drone
     * @return
     */
    float getRoll() const;

    /**
     * Retrieve last valid yaw read from drone
     * @return
     */
    float getYaw() const;

    /**
     * Retrieve last valid height read from drone
     * @return
     */
    float getHeight() const;

    /**
     * Retrieve last valid operation mode read from drone
     * @return
     */
    int getMode() const;

    /**
     * Retrieve last K value read from drone, this represents the biggest motor output the drone will attempt to use
     * Range from 0 to 5000
     * @return
     */
    float getK() const;

    /**
     * Retrieve last motor1 output read from drone
     * @return
     */
    float getMotorOutput1() const;

    /**
     * Retrieve last motor2 output read from drone
     * @return
     */
    float getMotorOutput2() const;

    /**
     * Retrieve last motor3 output read from drone
     * @return
     */
    float getMotorOutput3() const;

    /**
     * Retrieve last motor4 output read from drone
     * @return
     */
    float getMotorOutput4() const;

    /**
     * Attempts to send all manual motor outputs at once
     * Range from 0 to 5000
     * @return
     */
    void setAllMotorOutput(float output);

    /**
     * Attempts to send individual motor outputs to all motors
     * Range from 0 to 5000
     * @param output1
     * @param output2
     * @param output3
     * @param output4
     */
    void setAllMotorOutput(float output1, float output2, float output3, float output4);

    /**
     * Attempts to update the PID Setpoints on the drone
     * @param pitch
     * @param roll
     * @param yaw
     * @param height
     */
    void setSetpoints(float pitch, float roll, float yaw, float height);

    /**
     * Attempts to update K value, this represents the biggest motor output the drone will attempt to use
     * @param pitch
     * @param roll
     * @param yaw
     * @param height
     */
    void setK(float newK);

    /**
     * Immediately send command to stop all motors
     */
    void emergencyStop();

    /**
     * Report whether a valid message has been read from the drone recently
     * @return
     */
    bool isConnected() const;

    /**
     * Retrieve the time between heartbeats that the drone has received
     * @return
     */
    float getHeartbeatTime() const;

private:

    static const short int BUFFER_SIZE = 200;
    /**
     * Update comms thread, reads the incoming buffer. It waits for the start of the message and its end, then
     * parses it and updates the current known status of the drone.
     */
    void updateComms();

    void update(const char buffer[BUFFER_SIZE]);

    void updatePID(char pidCode, SimplePID pid);

    std::thread updateThread;
    bool isRunning = true;

    std::string ipAddress;
    int port;
    int sock{};
    struct sockaddr_in address{};
    struct sockaddr_in serv_addr{};
    bool connected = false;

    std::string manualMotorControlTemplate = ",M %f %f %f %f\n";
    std::string setpointControlTemplate = ",S %f %f %f %f\n";
    std::string emergencyStopTemplate = ",E\n";
    std::string pidConfigUpdateTemplate = ",P %c %f %f %f %f %f %f %f %f %f\n";
    std::string kUpdateTemplate = ",K %f\n";
    std::string kHeartbeatTemplate = ",L\n";

    float pitch = 0;
    float roll = 0;
    float yaw = 0;
    float height = 0;

    int mode = 0;
    float motorOutput1 = 0, motorOutput2 = 0, motorOutput3 = 0, motorOutput4 = 0;
    float droneTime = 0.0f;

    float k = 0;
    SimplePID pitchPid, rollPid, yawPid, heightPid;

    std::chrono::high_resolution_clock::time_point lastTimeUpdate;
    ConectionState connectionState;
    std::string lastMessage;

    bool heartbeatReset = false;
    std::string commandToSend;
    std::mutex commandMutex;
    bool needToSendCommand = false;
};


#endif //MICRODRONITESM_GUI_MICRODRONINTERFACE_H
