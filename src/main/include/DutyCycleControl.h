#pragma once

#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <ctre/phoenix6/swerve/impl/SwerveDrivetrainImpl.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix6;
using namespace swerve;
using namespace requests;
namespace ctre::phoenix6::swerve::requests
{

    class DutyCycleControl : public SwerveRequest
    {

    public:
        double DutyCycle = 0;

        ctre::phoenix::StatusCode Apply(SwerveRequest::ControlParameters const &parameters, std::vector<std::unique_ptr<impl::SwerveModuleImpl>> const &modulesToApply) override
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

}