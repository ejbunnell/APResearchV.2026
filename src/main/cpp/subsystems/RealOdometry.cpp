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
        {frc::SwerveModulePosition{}, frc::SwerveModulePosition{}, frc::SwerveModulePosition{}, frc::SwerveModulePosition{}});

    PoseEstimate highPose = limelightHigh.GetPose(robotAngle.Degrees(), 0_rad_per_s);
    odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3>{0.5, 9999.0, 9999.0});
    if (highPose.tagCount > 1) odometry.AddVisionMeasurement(highPose.pose, highPose.timestampSeconds);
    odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3>{0.01, 9999.0, 9999.0});
    if (canRange.GetDistance().GetValue() < 1_m && canRange.GetMeasurementHealth().GetValue() == ctre::phoenix6::signals::MeasurementHealthValue::Good)
    {
        // odometry.AddVisionMeasurement(frc::Pose2d{frc::Translation2d{26.625_ft - canRange.GetDistance().GetValue() - 11.25_in, 0_m}, robotAngle}, frc::Timer::GetFPGATimestamp());
    }

    units::meter_t currentPosition = odometry.GetEstimatedPosition().X();
    units::meter_t deltaX = currentPosition - lastPosition;
    units::second_t currentTime = utils::GetSystemTime();
    velocity = deltaX / (currentTime - lastTime);
    lastPosition = currentPosition;
    lastTime = currentTime;
}