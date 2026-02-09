// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

RobotContainer::RobotContainer()
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto &&
                                {
                                    return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed)      // Drive forward with negative Y (forward)
                                        .WithVelocityY(-joystick.GetLeftX() * MaxSpeed)              // Drive left with negative X (left)
                                        .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                                }));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([]
                                { return swerve::requests::Idle{}; })
            .IgnoringDisable(true));

    joystick.A().WhileTrue(
        frc2::cmd::RunOnce([this](){ drivetrain.TareEverything(); ctre::phoenix6::SignalLogger::Start(); })
        .AndThen(
            drivetrain.ApplyRequest([this]
            {
                return velocityControl.WithVelocity(0_tps);
            })
        ).Repeatedly().WithTimeout(1_s)
        .AndThen(
        drivetrain.ApplyRequest([this]() -> auto &&
                                { return velocityControl.WithVelocity(100_tps); }).Repeatedly()
        ).FinallyDo([this] {ctre::phoenix6::SignalLogger::Stop();}));

    // reset the field-centric heading on y button press
    joystick.Y().OnTrue(drivetrain.RunOnce([this]
                                           { drivetrain.SeedFieldCentric(); }));

    // joystick.LeftBumper().Debounce(40_ms).OnTrue(frc2::cmd::RunOnce(ctre::phoenix6::SignalLogger::Start));
    // joystick.RightBumper().Debounce(40_ms).OnTrue(frc2::cmd::RunOnce(ctre::phoenix6::SignalLogger::Stop));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return frc2::cmd::Print("No autonomous command configured");
}