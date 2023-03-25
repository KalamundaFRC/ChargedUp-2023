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
}

float getAccelNoGravity(float angle, float rawAccel){//gravity compensation

  return rawAccel - (9800 * (std::sin(angle)));

}
void Robot::AutonomousPeriodic() {

  float leAngle = gyro.GetGyroAngleY().value();
  float leAccel = gyro.GetAccelX().value();

    std::cout << std::sin(leAngle) << ",    "<< leAccel << ",    " << leAngle << ",    " << getAccelNoGravity(leAngle, leAccel) << std::endl;
   
  //gravity compensation




  //  // maybe change 0_deg to 2.5 -> 4 degrees
  // if (gyro.GetAccelZ() == units::meters_per_second_squared_t(0))
  // {
  //   leftPower = -0.3;
  //   rightPower = -0.3;
  // }
  if(gyro.GetGyroAngleX() > 0_deg)
  {
    leftPower = -0.25;
    rightPower = -0.25;
  }
  else if(gyro.GetGyroAngleX() < 0_deg)
  {
    leftPower = 0.25;
    rightPower = 0.25;
  }
  else
  {
    leftPower = 0;
    rightPower = 0;
  }

  //backup*
  autoTimer.Start();
   if(autoTimer.Get() <= 2_s){
   Solenoid.Set(frc::DoubleSolenoid::kReverse);
   //leftPower = 0.23;
   //rightPower = 0.23;
   }
   else if(autoTimer.Get() <= 5_s){ //4.5
   //leftPower = 0.55;
   //rightPower = 0.50;
   leftPower = 0.23;
   rightPower = 0.20;
  }
  else
  {
    //Solenoid.Set(frc::DoubleSolenoid::kForward);
    leftPower = 0;
    rightPower = 0;
    autoTimer.Stop();
  }

  
  //if(autoTimer.m_running)
  
  victor.Set(ControlMode::PercentOutput, leftPower);
  victor.Set(ControlMode::PercentOutput, rightPower);


  // frontLeftM.Set(ControlMode::PercentOutput, leftPower);
  // frontRightM.Set(ControlMode::PercentOutput, rightPower);
  // backLeftM.Set(ControlMode::PercentOutput, leftPower);
  // backRightM.Set(ControlMode::PercentOutput, rightPower);
  frontLeftM.SetNeutralMode(Brake);
  frontRightM.SetNeutralMode(Brake);
  backLeftM.SetNeutralMode(Brake);
  backRightM.SetNeutralMode(Brake);
  
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
  //bool negative = turn < 0;
  turn = turn*turn*(turn < 0 ? -1 : 1);


  leftPower = forward - turn;
  rightPower = forward + turn;

  if(std::fabs(driver.GetThrottle() > 1)){
    rightPower *= limit;
    leftPower *= limit;
  }

  /*Xbox Controller*/
  //leftPower = (driver.GetLeftY())*(driver.GetLeftY())*(driver.GetLeftY());
  //rightPower = (driver.GetRightY())*(driver.GetRightY())*(driver.GetRightY());


  frontLeftM.Set(ControlMode::PercentOutput, leftPower);
  frontRightM.Set(ControlMode::PercentOutput, rightPower);
  backLeftM.Set(ControlMode::PercentOutput, leftPower);
  backRightM.Set(ControlMode::PercentOutput, rightPower);

  // if (rightSolenoid.Get(frc::DoubleSolenoid::Value()) == 0) {
  //   if(codriver.GetRightBumperPressed()){
  //     rightSolenoid.Set(frc::DoubleSolenoid::Value(1));
  //   } 
  // } else if (rightSolenoid.Get(frc::DoubleSolenoid::Value()) == 1)
  // {
  //   if(codriver.GetRightBumperPressed()) {
  //     rightSolenoid.Set(frc::DoubleSolenoid::Value(0));
  //   }
  // }
  

  // if(codriver.GetLeftBumperPressed()){
  //   Solenoid.Set(frc::DoubleSolenoid::Value(1));
  // }
  // else{
  //   rightSolenoid.Set(frc::DoubleSolenoid::Value(0));
  // }
  // if(codriver.GetRightBumperPressed()){
  //   // Solenoid.Set(1, frc::DoubleSolenoid::Value(0));
  //   Solenoid.Set(frc::DoubleSolenoid::kForward);
  // }
  // else{
  //   Solenoid.Set(frc::DoubleSolenoid::kReverse);
  // }

  // if(codriver.GetRightBumperPressed()){
  //   Solenoid.Toggle();
  // }
  // if(driver.GetTriggerPressed()){
  //   Solenoid.Toggle();
  // }


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
