package frc.robot.subsystem;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.TunerConstants;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.BaseCam.AprilTagResult;
import frc.robot.util.Field;
import frc.robot.util.LimeLightCam;

public final class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem, Sendable {
	private static CommandSwerveDrivetrain m_Swerve;
	private static LimeLightCam frontTable = new LimeLightCam("limelight-front");

	public static synchronized CommandSwerveDrivetrain get() {
		if (m_Swerve == null)
			m_Swerve = TunerConstants.createDrivetrain();
		return m_Swerve;
	}

	public CommandSwerveDrivetrain(
			SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
		super(drivetrainConstants, 0, modules);
		if (Utils.isSimulation()) {
			simulationInit();
		}
	}

	@Override
	public void periodic() {
		  // This runs 50 times per second
        frontTable.addVisionEstimate(this::addVisionMeasurement, this::acceptEstimate);    
	}

	
    boolean acceptEstimate(AprilTagResult latestResult) {
      if(latestResult.distToTag > 3){
        return false;
      }
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

	public Pose2d getPose(){
		return this.getState().Pose;
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
		simUpdate.setName("CommandSwerveDrivetrain Simulation Update");
		simUpdate.startPeriodic(0.005);
	}


	public Command rotateToAngle(Rotation2d targetAngle) {
    return applyRequest(() -> 
        new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(calculateRotationSpeed(
                Rotation2d.fromRadians(m_Swerve.getPose().getRotation().getRadians()), 
                targetAngle))
    );
}


 public Rotation2d target_theta(){
      return new Rotation2d(Math.atan2(
        Field.hub_position.getY() - CommandSwerveDrivetrain.get().getPose().getY(),
        Field.hub_position.getX() - CommandSwerveDrivetrain.get().getPose().getX()
        ));
    }



private double calculateRotationSpeed(Rotation2d current, Rotation2d target) {
    double error = target.minus(current).getRadians();
    double kP = 2.0; // Tune this value
    return error * kP;
}



}