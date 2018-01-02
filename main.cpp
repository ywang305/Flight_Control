/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>


#include "flight_control_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

// User defined application
class Task {
public:
  Task(Vehicle* v): vehicle(v){}

  void PositionCtrSample();
  void PositionCtrInteractive();
  void AttitudeCtrInteractive();
  
private:
  Vehicle* vehicle;

  /* this func writtern based version 3.2's attitudeControl*/
  void attitudeControl(float rollDesired, float pitchDesired, float yawDesired, int timeoutInMs = 2000, float thresholdInDeg = 1.0);
};

/* !Task methods definition
*/
void Task::PositionCtrSample() {
  monitoredTakeoff(vehicle);
  moveByPositionOffset(vehicle, 0, 6, 6, 30);  // xoffset, yoffset, zoffset, yaw(deg)
  moveByPositionOffset(vehicle, 6, 0, -3, -30);
  moveByPositionOffset(vehicle, -6, -6, 0, 0);
  monitoredLanding(vehicle);
}

void Task::PositionCtrInteractive() {
  std::string info = "|[x y z yaw] to offset, [:q] to quit    |\n";

  monitoredTakeoff(vehicle);

  std::cout << info;
  std::string command;
  while(std::getline(std::cin, command)) {
    if(command==":q") break;
    std::stringstream ss(command);
    float x=0, y=0, z=0, yaw=0;
    ss >> x >> y >> z >> yaw;
    moveByPositionOffset(vehicle, x, y, z, yaw);
    std::cout << info;
  }
  monitoredLanding(vehicle);
}

void AttitudeCtrInteractive() {
  std::string info = "|[roll pitch yaw] to offset, [:q] to quit    |\n";

  monitoredTakeoff(vehicle);

  std::cout << info;
  std::string command;
  while(std::getline(std::cin, command)) {
    if(command==":q") break;
    std::stringstream ss(command);
    float r=0, p=0, y=0;
    ss >> r >> p >> y;
    attitudeControl(r, p, y);
    std::cout << info;
  }
  monitoredLanding(vehicle);
}



void Task::attitudeControl(float rollDesired, float pitchDesired, float yawDesired, int timeoutInMs, float thresholdInDeg) {
    // Wait for data to come in
    sleep(1);
    float curZ = vehicle->broadcast->getGlobalPosition().altitude;
    Telemetry::Quaternion broadcastQ = vehicle->broadcast->getQuaternion();
    Vector3f curEuler  = toEulerAngle((static_cast<void*>(&broadcastQ)));

    double thresholdInRad = DEG2RAD*thresholdInDeg;
    double rollDesiredRad = DEG2RAD*rollDesired;
    double pitchDesiredRad = DEG2RAD*pitchDesired;
    double yawDesiredRad = DEG2RAD*yawDesired;

    int elapsedTime = 0;
    //! 50Hz Attitude control loop
    while(std::abs(curEuler.x/DEG2RAD - rollDesiredRad) > thresholdInRad || std::abs(curEuler.y/DEG2RAD - pitchDesiredRad) > thresholdInRad || std::abs(curEuler.z/DEG2RAD - yawDesiredRad) > thresholdInRad) {
      if (elapsedTime >= timeoutInMs)
        break;
      vehicle->control->attitudeAndVertPosCtrl(rollDesired, pitchDesired, yawDesired, curZ);
      usleep(20000);
      elapsedTime += 20;
      broadcastQ = vehicle->broadcast->getQuaternion();
      curEuler  = toEulerAngle((static_cast<void*>(&broadcastQ)));
    }
    if (elapsedTime >= timeoutInMs)   std::cout << "Timed out \n";
}







/*! main
 *
 */
int
main(int argc, char** argv)
{
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();  // dji_linux_helper.cpp, comment out: exceptThrow in void LinuxSetup::initVehicle()
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);

  // Display interactive prompt
  std::string availableCommands = "| Available commands:          |\n";
              availableCommands +="| [task1] Position Ctrl Sample |\n";
              availableCommands +="| [task2] Position Ctrl Cmd    |\n";
              availableCommands +="| [task3] Attitude Ctrl Cmd    |\n";
  std::cout << availableCommands;

  std::vector<std::string> commands{"task1", "task2"};

  std::string command;
  while(!std::getline(std::cin, command)
        || commands.end()==std::find(commands.begin(), commands.end(), command)) {
    std::cout << "Error command !\n" + availableCommands;
  }
  
  Task task(vehicle);

  if(command == "task1") {
    task.PositionCtrSample();
  }
  else if(command == "task2") {
    task.PositionCtrInteractive();
  }
  else if(command == "task3") {
    task.AttitudeCtrInteractive();
  }

  return 0;
}
