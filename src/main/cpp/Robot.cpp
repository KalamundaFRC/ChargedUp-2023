// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <iostream>
using namespace std;

void Robot::RobotInit() {
  frontLeftM.SetInverted(true);
  backLeftM.SetInverted(true);
  frontRightM.SetInverted(false);
  backRightM.SetInverted(false);
  //funnyHub.Enabled();
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  autoTimer.Reset();
  gyro.Reset();
}

// float getAccelNoGravity(float angle, float rawAccel){//gravity compensation

//   return rawAccel - (9800 * (std::sin(angle)));
//std::cout << std::sin(leAngle) << ",    "<< leAccel << ",    " << leAngle << ",    " << getAccelNoGravity(leAngle, leAccel) << std::endl;

// }
void Robot::AutonomousPeriodic() {
  /*Balance Auto*/
  float leAngle = gyro.GetGyroAngleY().value();

   std::cout << leAngle << std::endl;

  if(gyro.GetGyroAngleY() > 25_deg)
  {
    leftPower = 0.25;
    rightPower = 0.25;
  }
  else if(gyro.GetGyroAngleY() < -25_deg)
  {
    leftPower = -0.25;
    rightPower = -0.25;
  }
  else 
  {
    leftPower = 0;
    rightPower = 0;
  }

  victor.Set(ControlMode::PercentOutput, leftPower);
  victor.Set(ControlMode::PercentOutput, rightPower);

  /*Score&Taxi Auto*/
  autoTimer.Start();
   if(autoTimer.Get() <= 2_s){
   Solenoid.Set(frc::DoubleSolenoid::kReverse);
   }
   else if(autoTimer.Get() <= 5_s){
   leftPower = 0.30;
   rightPower = 0.28;
  }
  else
  {
    leftPower = 0;
    rightPower = 0;
    autoTimer.Stop();
  }

   frontLeftM.Set(ControlMode::PercentOutput, leftPower);
   frontRightM.Set(ControlMode::PercentOutput, rightPower);
   backLeftM.Set(ControlMode::PercentOutput, leftPower);
   backRightM.Set(ControlMode::PercentOutput, rightPower);
  
}

void Robot::TeleopInit() {
  autoTimer.Reset();
  frontLeftM.SetNeutralMode(Brake);
  frontRightM.SetNeutralMode(Brake);
  backLeftM.SetNeutralMode(Brake);
  backRightM.SetNeutralMode(Brake);
  Solenoid.Set(frc::DoubleSolenoid::kForward);
}

void Robot::TeleopPeriodic() {

  /*Joystick Code*/
  double turn = 0;
  double forward = 0;

  if(std::abs(driver.GetY()) > deadzone){
    forward = driver.GetY();
  };

  forward = forward*forward*forward;

  if(std::abs(driver.GetZ()) > deadzone){
    turn = driver.GetZ();
  }
  //smoother turning?
  turn = turn*turn*(turn < 0 ? -1 : 1);


  leftPower = forward - turn;
  rightPower = forward + turn;

  if(std::fabs(driver.GetThrottle() > 1)){
    rightPower *= limit;
    leftPower *= limit;
  }

  if(driver.GetTriggerPressed()){
    Solenoid.Toggle();
  }

  /*Xbox Controller*/
  //leftPower = (driver.GetLeftY())*(driver.GetLeftY())*(driver.GetLeftY());
  //rightPower = (driver.GetRightY())*(driver.GetRightY())*(driver.GetRightY());
  // if(codriver.GetRightBumperPressed()){
  //   Solenoid.Toggle();
  // }


  frontLeftM.Set(ControlMode::PercentOutput, leftPower);
  frontRightM.Set(ControlMode::PercentOutput, rightPower);
  backLeftM.Set(ControlMode::PercentOutput, leftPower);
  backRightM.Set(ControlMode::PercentOutput, rightPower);

} 

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
