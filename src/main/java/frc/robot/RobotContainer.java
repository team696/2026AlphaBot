// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.BotConstants.Shooter;
import frc.robot.HumanControls.DriverPannel;
import frc.robot.HumanControls.SingleXboxController;
import frc.robot.commands.GyroReset;
import frc.robot.subsystem.Swerve;
import frc.robot.util.constants.DriveConstants;
import frc.robot.subsystem.ShooterAndTurret;
import frc.robot.subsystem.Intake;


public class RobotContainer {
    public final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public final  double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
	
    	 /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband( DriveConstants.MaxSpeed* 0.15).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.15) // Add a  deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
 
    private final Telemetry logger = new Telemetry(MaxSpeed);

     private final CommandXboxController joystick = new CommandXboxController(0);
    public final Intake intake = new Intake();
    public final ShooterAndTurret shooterAndTurret = new ShooterAndTurret();

    public final SwerveRequest.FieldCentricFacingAngle FCFARequest = 
    new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(DriveConstants.MaxSpeed* 0.15)
        .withRotationalDeadband(DriveConstants.MaxAngularRate * 0.15) // Add a  deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withHeadingPID(5, 0, .1); 

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
      // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        Swerve.get().setDefaultCommand(
            // Drivetrain will execute this command periodically
            Swerve.get().applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() / 2 * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() / 2 * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        joystick.leftTrigger().whileTrue(shooterAndTurret.runShooter());

        joystick.rightTrigger().whileTrue(intake.runIntake());
       
        joystick.a().onTrue((new GyroReset(Swerve.get())));
        
        joystick.b().whileTrue(
            // Drivetrain will execute this command periodically
            Swerve.get().applyRequest(() ->
                FCFARequest.withVelocityX(joystick.getLeftY() / 2 * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() / 2 * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(Swerve.get().target_Theta())
            )
        );

        Swerve.get().registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
