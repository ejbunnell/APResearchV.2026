#pragma once

#include <functional>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "Includes.h"

#include <ctre/phoenix6/SignalLogger.hpp>

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
        return nt::NetworkTableInstance::GetDefault().GetTable("AP Research Data")->GetSubTable(name);
    }
    nt::DoublePublisher torque = GetTable()->GetDoubleTopic("torque (Nm)").Publish();
    nt::DoublePublisher torqueCurrent = GetTable()->GetDoubleTopic("torqueCurrent (A)").Publish();
    nt::DoublePublisher kT = GetTable()->GetDoubleTopic("kT (Nm/A)").Publish();
    nt::DoublePublisher wheelOmega = GetTable()->GetDoubleTopic("wheelOmega (rad/s)").Publish();
};

class Telemetry : public frc2::CommandHelper<frc2::Command, Telemetry>
{
public:
    Telemetry(std::function<std::vector<double>()> odometrySupplier,
        std::function<std::vector<double>()> robotDataSupplier,
        std::function<std::vector<std::vector<double>>()> moduleDataSupplier) 
    : odometrySupplier(odometrySupplier), robotDataSupplier(robotDataSupplier), moduleDataSupplier(moduleDataSupplier) {}

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<nt::NetworkTable> GetTable()
    {
        return nt::NetworkTableInstance::GetDefault().GetTable("AP Research Data");
    }

    nt::DoublePublisher realOdometry = GetTable()->GetDoubleTopic("realOdometry (m)").Publish();
    nt::DoublePublisher wheelOdometry = GetTable()->GetDoubleTopic("wheelOdometry (m)").Publish();
    nt::DoublePublisher percentError = GetTable()->GetDoubleTopic("percentError (%)").Publish();
    nt::DoublePublisher averageTorque = GetTable()->GetDoubleTopic("averageTorque (Nm)").Publish();
    nt::DoublePublisher averageTorqueCurrent = GetTable()->GetDoubleTopic("averageTorqueCurrent (A)").Publish();
    nt::DoublePublisher kT = GetTable()->GetDoubleTopic("kT (Nm/A)").Publish();
    nt::DoublePublisher slipRatio = GetTable()->GetDoubleTopic("slipRatio").Publish();
    nt::DoublePublisher robotVelocity = GetTable()->GetDoubleTopic("robotVelocity (m/s)").Publish();
    nt::DoublePublisher averageWheelOmega = GetTable()->GetDoubleTopic("averageWheelOmega (rad/s)").Publish();
    
    ModuleTelemetry frontLeft{"frontLeft"};
    ModuleTelemetry frontRight{"frontRight"};
    ModuleTelemetry backLeft{"backLeft"};
    ModuleTelemetry backRight{"backRight"};

    std::vector<ModuleTelemetry*> modulesTelemetry = 
    {
        &frontLeft, &frontRight, &backLeft, &backRight
    };
    
    std::function<std::vector<double>()> odometrySupplier;
    std::function<std::vector<double>()> robotDataSupplier;
    std::function<std::vector<std::vector<double>>()> moduleDataSupplier;
    
};