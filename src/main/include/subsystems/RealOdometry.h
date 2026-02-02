#pragma once
#include <frc2/command/SubsystemBase.h>
#include "generated/TunerConstants.h"
#include "subsystems/Limelight.h"
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

class RealOdometry : frc2::SubsystemBase
{
public:
    RealOdometry(std::function<frc::Rotation2d()> robotAngleFunction);

    void Periodic() override;

    frc::Pose2d GetPose()
    {
        return odometry.GetEstimatedPosition();
    }

    double CalculatePercentError(frc::Pose2d measuredPose)
    {
        return ((measuredPose.X() - GetPose().X()) / GetPose().X()).value() * 100;
    }

    double CalculateSlipRatio(units::radians_per_second_t omega)
    {
        double radiusTimesOmega = (TunerConstants::kWheelRadius.convert<units::meters>() * omega).value();
        return (radiusTimesOmega - velocity.value()) / radiusTimesOmega;
    }

    units::meters_per_second_t GetVelocity()
    {
        return velocity;
    }

private:
    ctre::phoenix6::hardware::CANrange canRange{1};
    Limelight limelightLow{"limelight-low", LimelightConstants::kLowOffset};
    Limelight limelightHigh{"limelight-high", LimelightConstants::kHighOffset};

    std::function<frc::Rotation2d()> robotAngleFunction;

    frc::SwerveDriveKinematics<4> kinematics{
        frc::Translation2d{TunerConstants::FrontLeft.LocationX, TunerConstants::FrontLeft.LocationY},
        frc::Translation2d{TunerConstants::FrontRight.LocationX, TunerConstants::FrontRight.LocationY},
        frc::Translation2d{TunerConstants::BackLeft.LocationX, TunerConstants::BackLeft.LocationY},
        frc::Translation2d{TunerConstants::BackRight.LocationX, TunerConstants::BackRight.LocationY}};

    // Odometry object that allows for vision input with a standard deviation
    frc::SwerveDrivePoseEstimator<4> odometry{
        kinematics,
        frc::Rotation2d(),
        {frc::SwerveModulePosition(), frc::SwerveModulePosition(), frc::SwerveModulePosition(), frc::SwerveModulePosition()},
        frc::Pose2d()};

    units::meter_t lastPosition;
    units::second_t lastTime;
    units::meters_per_second_t velocity;
};