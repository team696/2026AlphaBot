// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystem.Swerve;
import frc.robot.util.Auto;
import frc.robot.util.constants.DriveConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final SwerveRequest.FieldCentricFacingAngle FCFARequest = 
    new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(DriveConstants.MaxSpeed* .01)
        .withRotationalDeadband(DriveConstants.MaxAngularRate * .01) // Add a  deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withHeadingPID(5., 0, 0).withMaxAbsRotationalRate(DegreesPerSecond.of(360))
        .withRotationalDeadband(DegreesPerSecond.of(1)); 

  

  public Robot() {
    m_robotContainer = new RobotContainer();



<<<<<<< HEAD
  
=======
     Auto.initialize(
            new Auto.NamedCommand("shoot", Swerve.get().fakeCommand()),
            new Auto.NamedCommand("Pass_1", Swerve.get().fakeCommand()),
            new Auto.NamedCommand("Pass_1", Swerve.get().fakeCommand()),
            new Auto.NamedCommand("Intake", Swerve.get().fakeCommand()),
            new Auto.NamedCommand("Reset Intake", Swerve.get().fakeCommand()),
            new Auto.NamedCommand("Climb L1", Swerve.get().fakeCommand()),
            new Auto.NamedCommand("Auto Align", Swerve.get().applyRequest(()->
                    FCFARequest.withVelocityX((-joystick.getLeftY() / 2) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) // Drive forward with negative Y (forward)
                    .withVelocityY((-joystick.getLeftX() / 2) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) // Drive left with negative X (left)
                    .withTargetDirection(Swerve.get().hub_target_Theta())))
            
        );
>>>>>>> 5de2db1b0a63392528bab2163ded7b95911343cb

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = Auto.getSelectedAuto();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
