// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot()
{
	ctre::phoenix6::SignalLogger::EnableAutoLogging(false);
}

void Robot::RobotPeriodic()
{
	frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit()
{
	
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit()
{
}

void Robot::TeleopPeriodic() 
{
	if (tractionControl)
	{
		double slipRatio = m_container.realOdometry.CalculateSlipRatio(m_container.drivetrain.GetAverageOmega());
		if (slipRatio > -1.2 && slipRatio < 1.2)
		{
			frc::Translation2d offset{0.7 * slipRatio * (m_container.drivetrain.GetAverageOmega() / (1_rad * 2 * std::numbers::pi)) * 1.875_in * 0.02_s, 0_m};
			m_container.drivetrain.ResetTranslation(m_container.drivetrain.GetState().Pose.Translation() - offset);
		}
	}
}

void Robot::TeleopExit() {}

void Robot::TestInit()
{
	frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
