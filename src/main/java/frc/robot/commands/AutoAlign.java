// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.CommandSwerveDrivetrain;
import frc.robot.util.Field;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {

  PIDController pidController;
  /** Creates a new AutoAlign. */
  public AutoAlign() {
      pidController = new PIDController(0.0056, 0.00, 0);
      pidController.enableContinuousInput(-180, 180);

      addRequirements(CommandSwerveDrivetrain.get());


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}//Flibba 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rAxis = 0;
   

  }

    public Rotation2d target_theta(){
      return new Rotation2d(Math.atan2(
        Field.hub_position.getY() - CommandSwerveDrivetrain.get().getPose().getY(),
        Field.hub_position.getX() - CommandSwerveDrivetrain.get().getPose().getX()
        ));
    }


 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
