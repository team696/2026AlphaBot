// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;

public class ShooterAndTurret extends SubsystemBase {
  /** Creates a new ShooterAndTurret. */
      private final TalonFX m_Shooter1;


  public ShooterAndTurret(){
     this.m_Shooter1 = new TalonFX(BotConstants.Shooter.shooterflywheel_1_ID);
         TalonFXConfiguration config = new TalonFXConfiguration();

     config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5; // 0.5 seconds
       config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;

       m_Shooter1.getConfigurator().apply(config);
  }


  
     public Command runShooter() {
    // first lambda: what to do while running
    // second lambda: what to do when the command ends
    return Commands.runEnd(
        () -> m_Shooter1.set(-0.40),
        () -> m_Shooter1.set(0),
        this // the subsystem
    );
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
