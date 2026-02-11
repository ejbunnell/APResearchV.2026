#include "Telemetry.h"

void Telemetry::Periodic()
{
    realOdometry.Set(odometrySupplier().at(0));
    encoderOdometry.Set(odometrySupplier().at(1));
    realOdometryX.Set(odometrySupplier().at(0).X().value());
    encoderOdometryX.Set(odometrySupplier().at(1).X().value());

    percentError.Set(robotDataSupplier().at(0));
    averageTorque.Set(robotDataSupplier().at(1));
    averageTorqueCurrent.Set(robotDataSupplier().at(2));
    kT.Set(robotDataSupplier().at(3));
    slipRatio.Set(robotDataSupplier().at(4));
    robotVelocity.Set(robotDataSupplier().at(5));
    averageWheelOmega.Set(robotDataSupplier().at(6));

    for (int i = 0; i < 4; i++)
    {
        ModuleTelemetry *module = modulesTelemetry.at(i);
        std::vector<double> data = moduleDataSupplier().at(i);
        module->PublishTorque(data.at(0));
        module->PublishTorqueCurrent(data.at(1));
        module->PublishKT(data.at(2));
        module->PublishWheelOmega(data.at(3));
    }

    swerveState.Set(statesSupplier().at(0));
    swerveGoal.Set(statesSupplier().at(1));
}

ModuleTelemetry::ModuleTelemetry(std::string name)
{
    this->name = name;
    torque = GetTable()->GetDoubleTopic("torque (Nm)").Publish();
    torqueCurrent = GetTable()->GetDoubleTopic("torqueCurrent (A)").Publish();
    kT = GetTable()->GetDoubleTopic("kT (Nm per A)").Publish();
    wheelOmega = GetTable()->GetDoubleTopic("wheelOmega (rad per s)").Publish();
}

void ModuleTelemetry::PublishTorque(double torque)
{
    this->torque.Set(torque);
}

void ModuleTelemetry::PublishTorqueCurrent(double torqueCurrent)
{
    this->torqueCurrent.Set(torqueCurrent);
}

void ModuleTelemetry::PublishKT(double kT)
{
    this->kT.Set(kT);
}

void ModuleTelemetry::PublishWheelOmega(double wheelOmega)
{
    this->wheelOmega.Set(wheelOmega);
}