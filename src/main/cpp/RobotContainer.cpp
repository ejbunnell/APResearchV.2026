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
        frc2::cmd::RunOnce([this](){ drivetrain.TareEverything(); }).AndThen(
        drivetrain.ApplyRequest([this]() -> auto &&
                                { return dutyCycleControl.WithDutyCycle(testDutyCycle); }).Repeatedly()
            .AlongWith(Telemetry
            {
                [this] 
                { 
                    return std::vector<frc::Pose2d>{realOdometry.GetPose(), drivetrain.GetState().Pose}; 
                },
                [this] 
                { 
                    return std::vector<double>
                        {
                            realOdometry.CalculatePercentError(drivetrain.GetState().Pose),
                            drivetrain.GetAverageTorque().value(),
                            drivetrain.GetAverageTorqueCurrent().value(),
                            drivetrain.GetModuleKT(0).value(),
                            realOdometry.CalculateSlipRatio(drivetrain.GetAverageOmega()),
                            realOdometry.GetVelocity().value(),
                            drivetrain.GetAverageOmega().value()
                        }; 
                },
                [this] 
                {  
                    std::vector<std::vector<double>> data;
                    for (int i = 0; i < 4; i++)
                    {
                        data.push_back
                        (
                            std::vector<double>
                                {
                                    drivetrain.GetModuleTorque(i).value(),
                                    drivetrain.GetModuleTorqueCurrent(i).value(),
                                    drivetrain.GetModuleKT(i).value(),
                                    drivetrain.GetModuleOmega(i).value()
                                }
                        );
                    }
                    return data;
                }

            }.ToPtr().Repeatedly())
        ));

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