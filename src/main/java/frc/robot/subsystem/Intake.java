// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
    private final TalonFX m_IntakeRun;
  public Intake() {
  this.m_IntakeRun = new TalonFX(BotConstants.Intake.intakeID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    // Set ramp times (seconds to go from 0 → full output)
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5; // 0.5 seconds
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;

       m_IntakeRun.getConfigurator().apply(config);
  }

     public Command runIntake() {
    return Commands.runEnd(
        () -> m_IntakeRun.set(.35),
        () -> m_IntakeRun.set(0),
        this
    );
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
