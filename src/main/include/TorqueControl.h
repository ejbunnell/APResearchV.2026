#pragma once

#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <ctre/phoenix6/swerve/impl/SwerveDrivetrainImpl.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix6;
using namespace swerve;
using namespace requests;
namespace ctre::phoenix6::swerve::requests
{

    class TorqueControl : public SwerveRequest
    {

    public:
        units::ampere_t TorqueCurrent = 0_A;

        ctre::phoenix::StatusCode Apply(SwerveRequest::ControlParameters const &parameters, std::vector<std::unique_ptr<impl::SwerveModuleImpl>> const &modulesToApply) override
        {
            for (size_t i = 0; i < modulesToApply.size(); ++i)
            {

                modulesToApply[i]->Apply(controls::TorqueCurrentFOC{TorqueCurrent}, controls::PositionVoltage{0_deg});
            }
            return ctre::phoenix::StatusCode::OK;
        }

        TorqueControl &WithTorqueCurrent(units::ampere_t newTorqueCurrent) &
        {
            this->TorqueCurrent = std::move(newTorqueCurrent);
            return *this;
        }
    };

}