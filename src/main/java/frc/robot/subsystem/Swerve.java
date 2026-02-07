package frc.robot.subsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
<<<<<<< HEAD
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
=======
>>>>>>> 5de2db1b0a63392528bab2163ded7b95911343cb

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
<<<<<<< HEAD
import edu.wpi.first.math.util.Units;
=======
>>>>>>> 5de2db1b0a63392528bab2163ded7b95911343cb
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.LimeLightCam;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.constants;
import frc.robot.TunerConstants;
import frc.robot.HumanControls.DriverPannel;
import frc.robot.util.BaseCam.AprilTagResult;
import frc.robot.util.BaseCam.measurementTrust;
import frc.robot.util.constants.DriveConstants;
import frc.robot.util.constants.FieldConstants;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;

public final class Swerve extends TunerSwerveDrivetrain implements Subsystem, Sendable {
	private static Swerve m_Swerve;	
	LimeLightCam exampleCamera = new LimeLightCam("limelight-front");
	public static Field2d fieldSim = new Field2d();

	public static synchronized Swerve get() {
		if (m_Swerve == null)
			m_Swerve = TunerConstants.createDrivetrain();
		return m_Swerve;
	}

	
	public Swerve(
			SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
		super(drivetrainConstants, 0, modules);

		if (Utils.isSimulation()) {
			simulationInit();
		}
	}

	@Override
	public void periodic() {
		exampleCamera.addVisionEstimate(this::addVisionMeasurement, this::acceptEstimate);
		fieldSim.setRobotPose(this.getState().Pose);
		SmartDashboard.putData("field", fieldSim);
		SmartDashboard.putNumber("target theta degrees", Swerve.get().hub_target_Theta().getDegrees());
	}

	public Pose2d getPose(){
		return Swerve.get().getState().Pose;
	}

	// Untested Code, Not Meant to Actually be used, just an example.
	boolean acceptEstimate(AprilTagResult latestResult, measurementTrust stdDeviations) {
		if (latestResult.distToTag > 4)
			return false; // Disregard measurements if we are more than 4 meters away
		if (this.getState().Speeds.omegaRadiansPerSecond > 2 * Math.PI)
			return false; // Disregard measurement if we are spinning faster than 360 deg second

		// Trust The measurement less the further we are
		double stdDeviationCoefficient = latestResult.distToTag *
				latestResult.distToTag / 12;

		stdDeviations.translation = .5 * stdDeviationCoefficient;
		stdDeviations.rotation = .5 * stdDeviationCoefficient;
		return true;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Value", () -> 1, null);
	}

	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}

	@Override
	public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
		super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
	}

	@Override
	public void addVisionMeasurement(
			Pose2d visionRobotPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs) {
		super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
				visionMeasurementStdDevs);
	}

	double m_lastSimTime;
	Notifier simUpdate;

	private void simulationInit() {
		m_lastSimTime = Utils.getCurrentTimeSeconds();
		simUpdate = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - m_lastSimTime;
			m_lastSimTime = currentTime;

			/* use the measured time delta, get battery voltage from WPILib */
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		simUpdate.setName("Swerve Simulation Update");
		simUpdate.startPeriodic(0.005);
	}

	public boolean finishRotation(Rotation2d target){
	return this.getPose().getRotation() == target;
	}

	  public Rotation2d hub_target_Theta(){
      return new Rotation2d(Math.atan2(
        FieldConstants.hub_position.getY() - Swerve.get().getPose().getY(),
        FieldConstants.hub_position.getX() - Swerve.get().getPose().getX()
        ));}

public Rotation2d blue_corner1_Theta(){
      return new Rotation2d(Math.atan2(
        0 - Swerve.get().getPose().getY(),
       0 - Swerve.get().getPose().getX()
        ));}

		public Rotation2d blue_corner3_Theta(){
      return new Rotation2d(Math.atan2(
         316.64  - Swerve.get().getPose().getY(),
      0- Swerve.get().getPose().getX()
        ));}

		
public Command fakeCommand() {
    System.out.println("Something happened");
	return Commands.none();
}

<<<<<<< HEAD
	// Since we are using a holonomic drivetrain, the rotation component of this pose
	// represents the goal holonomic rotation
	Pose2d targetPose = new Pose2d(Units.inchesToMeters(91.055), 
							Units.inchesToMeters(147.47), Rotation2d.fromDegrees(0));

	// Create the constraints to use while pathfinding
	PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

public Command alignToClimb(){
	// Since AutoBuilder is configured, we can use it to build pathfinding commands
	return AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0); // Goal end velocity in meters/sec
}

=======
>>>>>>> 5de2db1b0a63392528bab2163ded7b95911343cb
  
}