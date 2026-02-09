#pragma once

#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <ctre/phoenix6/swerve/impl/SwerveDrivetrainImpl.hpp>
#include <ctre/phoenix6/TalonFX.hpp>


class VelocityControl : public ctre::phoenix6::swerve::requests::SwerveRequest
{

public:
    units::turns_per_second_t Velocity{0};
    ctre::phoenix::StatusCode Apply(ctre::phoenix6::swerve::requests::SwerveRequest::ControlParameters const &parameters, std::span<std::unique_ptr<ctre::phoenix6::swerve::impl::SwerveModuleImpl> const> modulesToApply) override
    {
        for (size_t i = 0; i < modulesToApply.size(); ++i)
        {
            modulesToApply[i]->Apply(ctre::phoenix6::controls::VelocityVoltage{Velocity}, ctre::phoenix6::controls::PositionVoltage{180_deg});
        }
        return ctre::phoenix::StatusCode::OK;
    }

    VelocityControl &WithVelocity(units::turns_per_second_t newVelocity) &
    {
        this->Velocity = std::move(newVelocity);
        return *this;
    }
};

