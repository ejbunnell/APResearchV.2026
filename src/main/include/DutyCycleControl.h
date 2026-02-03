#pragma once

#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <ctre/phoenix6/swerve/impl/SwerveDrivetrainImpl.hpp>
#include <ctre/phoenix6/TalonFX.hpp>


class DutyCycleControl : public ctre::phoenix6::swerve::requests::SwerveRequest
{

public:
    double DutyCycle = 0;
    // SwerveRequest::ControlParameters const &parameters, std::span<std::unique_ptr<impl::SwerveModuleImpl> const> modulesToApply
    ctre::phoenix::StatusCode Apply(ctre::phoenix6::swerve::requests::SwerveRequest::ControlParameters const &parameters, std::span<std::unique_ptr<ctre::phoenix6::swerve::impl::SwerveModuleImpl> const> modulesToApply) override
    {
        for (size_t i = 0; i < modulesToApply.size(); ++i)
        {

            modulesToApply[i]->Apply(controls::DutyCycleOut{DutyCycle}, controls::PositionVoltage{0_deg});
        }
        return ctre::phoenix::StatusCode::OK;
    }

    DutyCycleControl &WithDutyCycle(double newDutyCycle) &
    {
        this->DutyCycle = std::move(newDutyCycle);
        return *this;
    }
};

