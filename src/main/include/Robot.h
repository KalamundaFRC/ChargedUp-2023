// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once
#include <iostream>
#include <string>
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/motorcontrol/PWMMotorController.h>
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/motorcontrol/MotorController.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <math.h>
#include <frc/AnalogGyro.h>
#include <frc/ADIS16448_IMU.h>

class Robot : public frc::TimedRobot {
 public:
   frc::XboxController codriver{1};
   frc::Joystick driver{0};
   //frc::Compressor funnyHub{1, frc::PneumaticsModuleType::REVPH};
   frc::DoubleSolenoid Solenoid{1, frc::PneumaticsModuleType::REVPH, 0, 1};
   frc::ADIS16448_IMU gyro;
   frc::Timer autoTimer;
   
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

   //*Xbox Controller* 
   double rightPower = 0;
   double leftPower = 0;
   double deadzone = 0.15;
   double limit = 0.5;
   TalonSRX frontLeftM{1};
   TalonSRX frontRightM{3};
   TalonSRX backLeftM{2};
   TalonSRX backRightM{4};
   //Test Board
   VictorSPX victor{18};

  private:
 
};



