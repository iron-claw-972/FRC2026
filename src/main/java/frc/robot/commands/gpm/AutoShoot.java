package frc.robot.commands.gpm;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.util.ShooterPhysics;
import frc.robot.util.ShooterPhysics.TurretState;

public class AutoShoot extends Command {
	private Drivetrain drive;
	private Hood hood;
	private Shooter shooter;
	private TurretState target_state;

	private ChassisSpeeds fieldRelVel;

	// apex of parabola in meters
	private double speed = 8;
	private double height = 1.85;

	public AutoShoot(Drivetrain drive, Hood hood, Shooter shooter) {
		this.drive = drive;
		this.hood = hood;
		this.shooter = shooter;
	}

	private Translation2d getRobotPosition() {
		return drive.getPose().getTranslation();
	}

	private void updateFieldRelVel() {
		ChassisSpeeds robotRelVel = drive.getChassisSpeeds();
		fieldRelVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelVel, drive.getYaw());
	}

	@Override
	public void initialize() {
		updateFieldRelVel();
	}

	@Override
	public void execute() {
		updateFieldRelVel();

		Logger.recordOutput("FieldRelativeVelocity", new Translation2d(fieldRelVel.vxMetersPerSecond,
				fieldRelVel.vyMetersPerSecond));
		Logger.recordOutput("HubLocation", FieldConstants.HUB_BLUE);

		// var state = ShooterPhysics.getShotParamsBySpeed(
		// new Translation2d(fieldRelVel.vxMetersPerSecond,
		// fieldRelVel.vyMetersPerSecond),
		// getRobotPosition(),
		// FieldConstants.HUB_BLUE,
		// speed);

		Optional<ShooterPhysics.TurretState> state = Optional.of(ShooterPhysics.getShotParams(
				new Translation2d(fieldRelVel.vxMetersPerSecond,
						fieldRelVel.vyMetersPerSecond).times(2.0),
				FieldConstants.HUB_BLUE
						.minus(new Translation3d(getRobotPosition()).plus(ShooterConstants.SHOOTER_TRANSFORM)),
				height));

		Logger.recordOutput("TurretState", state.get());

		if (state.isPresent()) {
			target_state = state.get();

			hood.setSetpoint(Units.radiansToDegrees(target_state.pitch()));
			shooter.setShooter(-target_state.exitVel());
			drive.setIsAlign(true);
			drive.setAlignAngle(target_state.yaw().getRadians());
			shooter.setFeeder(ShooterConstants.FEEDER_RUN_POWER);

			if (hood.atSetpoint() && drive.atAlignAngle() && shooter.atTargetSpeed()) {
				// shooter.setFeeder(1);
				// shooter.setFeeder(ShooterConstants.FEEDER_RUN_POWER);
			} else {
				// shooter.setFeeder(0);
			}

			SmartDashboard.putNumber("Target Hood Angle", target_state.pitch());
			SmartDashboard.putNumber("Target Exit Velocity", target_state.exitVel());
		}
	}

	@Override
	public void end(boolean canceled) {
		drive.setIsAlign(false);
		shooter.setFeeder(0);
		shooter.setShooter(0);
	}
}
