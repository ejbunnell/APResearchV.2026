#include "subsystems/RealOdometry.h"

RealOdometry::RealOdometry(std::function<frc::Rotation2d()> robotAngleFunction)
{
    this->robotAngleFunction = robotAngleFunction;

    canRange.GetConfigurator().Apply(configs::CANrangeConfiguration{});
    configs::CANrangeConfiguration config;
    config.ToFParams.UpdateMode = signals::UpdateModeValue::LongRangeUserFreq;

    canRange.GetConfigurator().Apply(config);
}

void RealOdometry::Periodic()
{
    frc::Rotation2d robotAngle = robotAngleFunction();
    odometry.UpdateWithTime(
        frc::Timer::GetFPGATimestamp(),
        robotAngle,
        {frc::SwerveModulePosition(), frc::SwerveModulePosition(), frc::SwerveModulePosition(), frc::SwerveModulePosition()});

    PoseEstimate lowPose = limelightLow.GetPose(robotAngle.Degrees(), 0_rad_per_s);
    odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3>{0.9, 0.9, 9999.0});
    odometry.AddVisionMeasurement(lowPose.pose, lowPose.timestampSeconds);
    PoseEstimate highPose = limelightHigh.GetPose(robotAngle.Degrees(), 0_rad_per_s);
    odometry.AddVisionMeasurement(highPose.pose, highPose.timestampSeconds);
    odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3>{0.01, 9999.0, 9999.0});
    if (canRange.GetMeasurementHealth().GetValue() == ctre::phoenix6::signals::MeasurementHealthValue::Good)
    {
        odometry.AddVisionMeasurement(frc::Pose2d{frc::Translation2d{canRange.GetDistance().GetValue(), odometry.GetEstimatedPosition().Y()}, robotAngle}, frc::Timer::GetFPGATimestamp());
    }

    units::meter_t currentPosition = odometry.GetEstimatedPosition().X();
    units::meter_t deltaX = currentPosition - lastPosition;
    units::second_t currentTime = utils::GetSystemTime();
    velocity = deltaX / (currentTime - lastTime);
    lastPosition = currentPosition;
    lastTime = currentTime;
}