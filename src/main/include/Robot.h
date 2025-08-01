// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <optional>
#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include "RobotContainer.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/phoenix6/CANdle.hpp>
#include <ctre/phoenix6/Orchestra.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PowerDistribution.h>

#include <photon/PhotonCamera.h>

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <generated/TunerConstants.h>
#include "VisionSim.h"
#include "subsystems/CommandSwerveDrivetrain.h"

using namespace ctre::phoenix6;
using namespace ctre::phoenix6::hardware;

ctre::phoenix6::hardware::Pigeon2 pigeon2{13};
class Robot : public frc::TimedRobot {
private:
    using RGBWColor = ctre::phoenix6::signals::RGBWColor;
    /* color can be constructed from RGBW, a WPILib Color/Color8Bit, HSV, or hex */
    static constexpr RGBWColor kGreen{0, 217, 0, 0};
    static constexpr RGBWColor kWhite = RGBWColor{frc::Color::kWhite} * 0.5; /* half brightness */
    static constexpr RGBWColor kViolet = RGBWColor::FromHSV(270_deg, 0.9, 0.8);
    static constexpr RGBWColor kRed = RGBWColor::FromHex("#D9000000").value();
    double voltage = 0;
    double current = 0;
    double totalPower = 0;
    double maxPower = 0;
    ctre::phoenix6::Orchestra m_orchestra;
    TalonFX muzykant1{2,"rio"};
    TalonFX muzykant2{5,"rio"};
    TalonFX muzykant3{8,"rio"};
    TalonFX muzykant4{11,"rio"};
    
    


    /**
     * Start and end index for LED animations.
     * 0-7 are onboard, 8-399 are an external strip.
     * Using the entire external strip for single animation.
     */
    static constexpr int kStripStartIdx = 8;
    static constexpr int kStripEndIdx = 85;

    ctre::phoenix6::hardware::CANdle m_candle{14, "rio"};
    frc::PowerDistribution pdh{15, frc::PowerDistribution::ModuleType::kRev};

    enum class AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    };

    AnimationType m_animState{AnimationType::None};
    frc::SendableChooser<AnimationType> m_animChooser;

    std::optional<frc2::CommandPtr> m_autonomousCommand;
    RobotContainer m_container;
    photon::PhotonCamera camera{constants::Vision::kCameraName};
    subsystems::CommandSwerveDrivetrain drivetrain = TunerConstants::CreateDrivetrain(); //I need help with this; I have absolutely no idea how to connect this
    VisionSim vision{&camera};
    frc::XboxController controller{0};
    static constexpr double VISION_TURN_kP = 0.005;

public:
    Robot();
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void DisabledExit() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void AutonomousExit() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TeleopExit() override;
    void TestInit() override;
    void TestPeriodic() override;
    void TestExit() override;
    void SimulationPeriodic() override;
    void SimulationInit() override;
};