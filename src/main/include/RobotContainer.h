// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/RealOdometry.h"
#include "TorqueControl.h"
#include "DutyCycleControl.h"
#include "Telemetry.h"

class RobotContainer
{
private:
    units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps;                 // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
                                               .WithDeadband(MaxSpeed * 0.15)
                                               .WithRotationalDeadband(MaxAngularRate * 0.15)                    // Add a 15% deadband
                                               .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors

    swerve::requests::TorqueControl torqueControl;
    swerve::requests::DutyCycleControl dutyCycleControl;

    frc2::CommandXboxController joystick{0};

public:
    subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};

    RealOdometry realOdometry{
        [this]()
        {
            return drivetrain.GetRotation3d().ToRotation2d();
        }};

    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

private:
    double testDutyCycle = 1.0;

    void ConfigureBindings();
};
