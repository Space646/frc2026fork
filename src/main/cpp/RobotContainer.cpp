// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

RobotContainer::RobotContainer()
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // drivetrain.SetDefaultCommand(
    //     // Drivetrain will execute this command periodically
    //     drivetrain.ApplyRequest([this]() -> auto&& {
    //         return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeedConst)  // Drive forward with negative Y (forward)
    //             .WithVelocityY(-joystick.GetLeftX() * MaxSpeedConst)  // Drive left with negative X (left)
    //             .WithRotationalRate(-joystick.GetRightX() * MaxAngularRateConst);  // Drive counterclockwise with negative X (left)
    //     })
    // );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    // PS4 button mappings:
    // Cross (X) button for brake
    joystick.Cross().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    
    // Circle (O) button for point mode
    joystick.Circle().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));

    // Run SysId routines when holding Share/Options and Triangle/Square.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Share() && joystick.Triangle()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Share() && joystick.Square()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Options() && joystick.Triangle()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Options() && joystick.Square()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // Reset the field-centric heading on L1 button press
    joystick.L1().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return frc2::cmd::Print("No autonomous command configured");
}