#pragma once

#include <functional>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/SubsystemBase.h>

#include "Includes.h"

class ModuleTelemetry
{
public:
    ModuleTelemetry(std::string name);
    void PublishTorque(double torque);
    void PublishTorqueCurrent(double torqueCurrent);
    void PublishKT(double kT);
    void PublishWheelOmega(double wheelOmega);
private:
    std::string name;
    std::shared_ptr<nt::NetworkTable> GetTable()
    {
        return nt::NetworkTableInstance::GetDefault().GetTable("AP Research Data/" + name);
    }
    nt::DoublePublisher torque; // = GetTable()->GetDoubleTopic("torque (Nm)").Publish();
    nt::DoublePublisher torqueCurrent; // = GetTable()->GetDoubleTopic("torqueCurrent (A)").Publish();
    nt::DoublePublisher kT; // = GetTable()->GetDoubleTopic("kT (Nm per A)").Publish();
    nt::DoublePublisher wheelOmega; // = GetTable()->GetDoubleTopic("wheelOmega (rad per s)").Publish();
};

class Telemetry : public frc2::SubsystemBase
{
public:
    Telemetry(std::function<std::vector<frc::Pose2d>()> odometrySupplier,
        std::function<std::vector<double>()> robotDataSupplier,
        std::function<std::vector<std::vector<double>>()> moduleDataSupplier,
        std::function<std::vector<std::vector<frc::SwerveModuleState>>()> statesSupplier) 
    : odometrySupplier(odometrySupplier), robotDataSupplier(robotDataSupplier), moduleDataSupplier(moduleDataSupplier), statesSupplier(statesSupplier) {}

    void Periodic() override;

private:
    std::shared_ptr<nt::NetworkTable> GetTable()
    {
        return nt::NetworkTableInstance::GetDefault().GetTable("AP Research Data");
    }

    nt::DoublePublisher realOdometryX = GetTable()->GetDoubleTopic("realOdometryX (m)").Publish();
    nt::DoublePublisher encoderOdometryX = GetTable()->GetDoubleTopic("encoderOdometryX (m)").Publish();
    nt::DoublePublisher percentError = GetTable()->GetDoubleTopic("percentError (%)").Publish();
    nt::DoublePublisher averageTorque = GetTable()->GetDoubleTopic("averageTorque (Nm)").Publish();
    nt::DoublePublisher averageTorqueCurrent = GetTable()->GetDoubleTopic("averageTorqueCurrent (A)").Publish();
    nt::DoublePublisher kT = GetTable()->GetDoubleTopic("kT (Nm per A)").Publish();
    nt::DoublePublisher slipRatio = GetTable()->GetDoubleTopic("slipRatio").Publish();
    nt::DoublePublisher robotVelocity = GetTable()->GetDoubleTopic("robotVelocity (m per s)").Publish();
    nt::DoublePublisher averageWheelOmega = GetTable()->GetDoubleTopic("averageWheelOmega (rad per s)").Publish();

    nt::StructPublisher<frc::Pose2d> realOdometry = GetTable()->GetStructTopic<frc::Pose2d>("realPose").Publish();
    nt::StructPublisher<frc::Pose2d> encoderOdometry = GetTable()->GetStructTopic<frc::Pose2d>("encoderPose").Publish();

    nt::StructArrayPublisher<frc::SwerveModuleState> swerveState = nt::NetworkTableInstance::GetDefault().GetTable("Swerve")->GetStructArrayTopic<frc::SwerveModuleState>("state").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> swerveGoal = nt::NetworkTableInstance::GetDefault().GetTable("Swerve")->GetStructArrayTopic<frc::SwerveModuleState>("goal").Publish();
    
    ModuleTelemetry frontLeft{"frontLeft"};
    ModuleTelemetry frontRight{"frontRight"};
    ModuleTelemetry backLeft{"backLeft"};
    ModuleTelemetry backRight{"backRight"};

    std::vector<ModuleTelemetry*> modulesTelemetry = 
    {
        &frontLeft, &frontRight, &backLeft, &backRight
    };
    
    std::function<std::vector<frc::Pose2d>()> odometrySupplier;
    std::function<std::vector<double>()> robotDataSupplier;
    std::function<std::vector<std::vector<double>>()> moduleDataSupplier;
    std::function<std::vector<std::vector<frc::SwerveModuleState>>()> statesSupplier;
    
    
};