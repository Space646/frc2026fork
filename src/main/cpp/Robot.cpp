// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <generated/TunerConstants.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <ctre/phoenix6/swerve/impl/SwerveModuleImpl.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include "RobotContainer.h"
#include <frc2/command/Commands.h>
swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(5.41_mps * 0.1).WithRotationalDeadband(0.75_tps * 0.1) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);
Robot::Robot() {
    /* Configure CANdle */
    configs::CANdleConfiguration cfg{};
    /* set the LED strip type and brightness */
    cfg.LED.StripType = signals::StripTypeValue::GRB;
    cfg.LED.BrightnessScalar = 0.5;
    /* disable status LED when being controlled */
    cfg.CANdleFeatures.StatusLedWhenActive = signals::StatusLedWhenActiveValue::Disabled;
    m_candle.GetConfigurator().Apply(cfg);
    // m_orchestra.AddInstrument(muzykant1);
    m_orchestra.AddInstrument(muzykant2);
    m_orchestra.AddInstrument(muzykant3);
    m_orchestra.AddInstrument(muzykant4);
    auto status = m_orchestra.LoadMusic("hava.chrp");



    

    /* clear all previous animations */
    for (int i = 0; i < 8; ++i) {
        m_candle.SetControl(controls::EmptyAnimation{i});
    }

    /* set the onboard LEDs to a solid color */
    // m_candle.SetControl(controls::SolidColor{0, 3}.WithColor(kGreen));
    // m_candle.SetControl(controls::SolidColor{4, 7}.WithColor(kWhite));

    /* add animations to chooser */
    m_animChooser.SetDefaultOption("Rainbow", AnimationType::Rainbow);
    m_animChooser.AddOption("Color Flow", AnimationType::ColorFlow);
    m_animChooser.AddOption("Fire", AnimationType::Fire);
    m_animChooser.AddOption("Larson", AnimationType::Larson);
    m_animChooser.AddOption("RGB Fade", AnimationType::RgbFade);
    m_animChooser.AddOption("Single Fade", AnimationType::SingleFade);
    m_animChooser.AddOption("Strobe", AnimationType::Strobe);
    m_animChooser.AddOption("Twinkle", AnimationType::Twinkle);
    m_animChooser.AddOption("Twinkle Off", AnimationType::TwinkleOff);
    m_animChooser.AddOption("None", AnimationType::None);

    frc::SmartDashboard::PutData("LED Animation", &m_animChooser);
}

void Robot::RobotPeriodic() {
    current = pdh.GetTotalCurrent();
    voltage = pdh.GetVoltage();
    totalPower = current*voltage;
    if (totalPower>maxPower)
    {
      maxPower=totalPower;
    }
    frc::SmartDashboard::PutNumber("Voltage",voltage);
    frc::SmartDashboard::PutNumber("Total Power", totalPower);
    frc::SmartDashboard::PutNumber("Max Power", maxPower);


    frc2::CommandScheduler::GetInstance().Run();

    /* if the animation selection changes, change animation */
    auto const animSelection = m_animChooser.GetSelected();
    if (m_animState != animSelection) {
        m_animState = animSelection;
        
        /* clear current animation first */
        m_candle.SetControl(controls::EmptyAnimation{0});
        
        switch (m_animState) {
            case AnimationType::None:
                /* keep animation cleared */
                break;
            case AnimationType::ColorFlow:
                m_candle.SetControl(
                    controls::ColorFlowAnimation{kStripStartIdx, kStripEndIdx}.WithSlot(0)
                        .WithColor(kViolet)
                );
                break;
            case AnimationType::Fire:
                m_candle.SetControl(
                    controls::FireAnimation{kStripStartIdx, kStripEndIdx}.WithSlot(0)
                        .WithCooling(0.4)
                        .WithSparking(0.5)
                );
                break;
            case AnimationType::Larson:
                m_candle.SetControl(
                    controls::LarsonAnimation{kStripStartIdx, kStripEndIdx}.WithSlot(0)
                        .WithColor(kRed)
                );
                break;
            case AnimationType::Rainbow:
                m_candle.SetControl(
                    controls::RainbowAnimation{kStripStartIdx, kStripEndIdx}.WithSlot(0)
                );
                break;
            case AnimationType::RgbFade:
                m_candle.SetControl(
                    controls::RgbFadeAnimation{kStripStartIdx, kStripEndIdx}.WithSlot(0)
                );
                break;
            case AnimationType::SingleFade:
                m_candle.SetControl(
                    controls::SingleFadeAnimation{kStripStartIdx, kStripEndIdx}.WithSlot(0)
                        .WithColor(kRed)
                );
                break;
            case AnimationType::Strobe:
                m_candle.SetControl(
                    controls::StrobeAnimation{kStripStartIdx, kStripEndIdx}.WithSlot(0)
                        .WithColor(kRed)
                );
                break;
            case AnimationType::Twinkle:
                m_candle.SetControl(
                    controls::TwinkleAnimation{kStripStartIdx, kStripEndIdx}.WithSlot(0)
                        .WithColor(kViolet)
                );
                break;
            case AnimationType::TwinkleOff:
                m_candle.SetControl(
                    controls::TwinkleOffAnimation{kStripStartIdx, kStripEndIdx}.WithSlot(0)
                        .WithColor(kViolet)
                );
                break;
        }
    }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::DisabledExit() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {
    double forward;
    double strafe;
    double turn;
    bool targetVisible = false;
    double targetYaw = 0.0;
    auto results = camera.GetAllUnreadResults();
    if (results.size() > 0) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        auto result = results[results.size() - 1];
        if (result.HasTargets()) {
        // At least one AprilTag was seen by the camera
        for (auto& target : result.GetTargets()) {
            if (target.GetFiducialId() == 7) {
            // Found Tag 7, record its information
            targetYaw = target.GetYaw();
            targetVisible = true;
            }
        }
    }
    if (targetVisible) {
    // Driver wants auto-alignment to tag 7
    // And, tag 7 is in sight, so we can turn toward it.
    // Override the driver's turn command with an automatic one that turns
    // toward the tag.
    turn =
        -1.0 * targetYaw * VISION_TURN_kP * constants::Swerve::kMaxAngularSpeed.value();
    }
    auto velX = forward * 1_mps;
    auto velY = strafe * 1_mps;
    auto rot = turn * 1_rad_per_s;
    drivetrain.SetControl(
        drive.WithVelocityX(3_mps)  // Drive forward with negative Y (forward)
        .WithVelocityY(0_mps)  // Drive left with negative X (left)
        .WithRotationalRate(4_rad_per_s
        ));  // Drive counterclockwise with negative X (left)
}
}


void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
        m_autonomousCommand->Cancel();
    }
    m_orchestra.Play();
    }

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}
void Robot::TestExit() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif