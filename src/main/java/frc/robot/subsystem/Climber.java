// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
    private final TalonFX m_Climber1 = new TalonFX(BotConstants.Climber.Climber_1_ID);

  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
