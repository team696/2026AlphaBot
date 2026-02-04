// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;



public class Hopper extends SubsystemBase {

  private final TalonFX m_Hopper = new TalonFX(BotConstants.Hopper.HopperID);
    private final TalonFX m_Magazine = new TalonFX(BotConstants.Hopper.MagazineID);

  /** Creates a new Hopper. */
  public Hopper() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
