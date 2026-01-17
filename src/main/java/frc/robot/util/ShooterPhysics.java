package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.Constants;

public class ShooterPhysics {
	// pitch in radians, going up from the horizontal
	// exit velocity speed in m/s
	public record TurretState(Rotation2d yaw, double pitch, double exitVel) {
	};

	public static TurretState getShotParams(Translation2d robotVelocity, Translation2d robot, Translation3d target,
			double peakHeight) {
		Translation3d robotToTarget = target.minus(new Translation3d(robot));
		Translation3d exitVel = getRequiredExitVelocity(robotVelocity, robotToTarget, peakHeight);

		Translation2d as2d = exitVel.toTranslation2d();
		Rotation2d yaw = as2d.getAngle();
		double magnitude2d = as2d.getDistance(Translation2d.kZero);

		double pitch = new Translation2d(magnitude2d, exitVel.getZ()).getAngle().getRadians();
		pitch %= Math.PI * 2;
		double speed = exitVel.getDistance(Translation3d.kZero);

		return new TurretState(yaw, pitch, speed);
	}

	// assumes shot from (0, 0, 0)
	// only public for unit testing, don't actually use this directly
	public static Translation3d getRequiredExitVelocity(Translation2d robotVelocity, Translation3d target,
			double peakZ) {
		// z = v_z_exit_vel * t - .5 * g * t²
		// want vertex of this equation to equal peakZ
		// t_vertex = -v_z_exit_vel / -g
		// t_vertex = v_z_exit_vel / g
		// z_vertex = v_z_exit_vel * (v_z_exit_vel / g) - .5 * g * (v_z_exit_vel / g)²
		// peakZ = v_z_exit_vel² / g - .5 * v_z_exit_vel² / g
		// peakZ = .5 * v_z_exit_vel² / g
		// v_z_exit_vel² = 2 * peakZ * g
		// v_z_exit_vel = √(2 * peakZ * g)
		double zExitVel = Math.sqrt(2 * peakZ * Constants.GRAVITY_ACCELERATION);

		// now we need time to hit target
		// z_target = v_z_exit_vel * t - .5 * g * t²
		// 0 = -.5 * g * t² + v_z_exit_vel * t - z_target
		// quadratic formula
		// t = (-v_z_exit_vel ± √(v_z_exit_vel² - 4 * (-.5 * g) * (-z_target))) / (2 *
		// -.5 * g)
		// t = (-v_z_exit_vel ± √(v_z_exit_vel² - 2 * g * z_target)) / -g
		// onlz use - because we only want the part where it's coming down, and that
		// gives the longer time
		double t = (-zExitVel - Math.sqrt(Math.pow(zExitVel, 2) - 2 * Constants.GRAVITY_ACCELERATION * target.getZ()))
				/ -Constants.GRAVITY_ACCELERATION;

		if (t < 0)
			throw new RuntimeException("Time should never be negative (got t=" + t + ").");

		// calculate x and z exit_vel
		// x = (v_x_robot + v_x_exit_vel) * t
		// x_target = (v_x_robot + v_x_exit_vel) * t_target
		// v_x_robot + v_x_exit_vel = x_target / t_target
		// v_x_exit_vel = x_target / t_target - v_x_robot
		double xExitVel = target.getX() / t - robotVelocity.getX();
		// same for y
		double yExitVel = target.getY() / t - robotVelocity.getY();
		return new Translation3d(xExitVel, yExitVel, zExitVel);
	}

	// call with default tolerance
	public static Optional<Translation3d> getExitVelocityForSpeed(Translation2d initialVelocity, Translation3d target,
			double speed) {
		return getExitVelocityForSpeed(initialVelocity, target, speed, 0.1);
	}

	public static Optional<Translation3d> getExitVelocityForSpeed(Translation2d initialVelocity, Translation3d target,
			double speed, double tolerance) {

		// TODO: detect when the given velocity is insufficient and exit before maxIters

		// guess a peak height
		double guess = 10;
		int maxIters = 20;
		while (maxIters >= 0) {
			maxIters--;
			Translation3d guessVelocity = getRequiredExitVelocity(initialVelocity, target, guess);
			double guessSpeed = guessVelocity.getNorm();
			double difference = speed - guessSpeed;

			// we've already hit zero height and are trying to go lower
			if (guess <= 0 && difference < 0)
				return Optional.empty();

			if (Math.abs(difference) <= tolerance)
				return Optional.of(guessVelocity);

			guess += difference * 1.7; // experimentally determined value
		}

		return Optional.empty();
	}
}
