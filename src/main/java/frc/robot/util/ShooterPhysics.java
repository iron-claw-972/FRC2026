package frc.robot.util;

import com.google.errorprone.annotations.CheckReturnValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.Constants;

class ShooterPhysics {
	// pitch in radians, going up from the horizontal
	// speed in m/s
	public record TurretState(Rotation2d yaw, double pitch, double speed) {
	};

	@CheckReturnValue
	public static TurretState getShotParams(Translation2d initialVelocity, Translation2d robot, Translation3d target,
			double height) {
		Translation3d robotToTarget = target.minus(new Translation3d(robot));
		Translation3d impulse = getRequiredImpulse(initialVelocity, robotToTarget, height);

		Translation2d as2d = impulse.toTranslation2d();
		Rotation2d yaw = as2d.getAngle();
		double magnitude2d = as2d.getDistance(Translation2d.kZero);

		double pitch = new Translation2d(magnitude2d, impulse.getZ()).getAngle().getRadians();
		pitch %= Math.PI * 2;
		double speed = impulse.getDistance(Translation3d.kZero);

		return new TurretState(yaw, pitch, speed);
	}

	// assumes shot from (0, 0, 0)
	// only public for unit testing, don't actually use this directly
	@CheckReturnValue
	public static Translation3d getRequiredImpulse(Translation2d initialVelocity, Translation3d target, double peakZ) {
		// z = v_z_impulse * t - .5 * g * t²
		// want vertex of this equation to equal peakZ
		// t_vertex = -v_z_impulse / -g
		// t_vertex = v_z_impulse / g
		// z_vertex = v_z_impulse * (v_z_impulse / g) - .5 * g * (v_z_impulse / g)²
		// peakZ = v_z_impulse² / g - .5 * v_z_impulse² / g
		// peakZ = .5 * v_z_impulse² / g
		// v_z_impulse² = 2 * peakZ * g
		// v_z_impulse = √(2 * peakZ * g)
		double zImpulse = Math.sqrt(2 * peakZ * Constants.GRAVITY_ACCELERATION);

		// now we need time to hit target
		// z_target = v_z_impulse * t - .5 * g * t²
		// 0 = -.5 * g * t² + v_z_impulse * t - z_target
		// quadratic formula
		// t = (-v_z_impulse ± √(4 * (-.5 * g) * (-z_target))) / (2 * -.5 * g)
		// t = (-v_z_impulse ± √(2 * g * z_target)) / -g
		// onlz use - because we only want the part where it's coming down, and that
		// gives the longer time
		double t = (-zImpulse - Math.sqrt(2 * Constants.GRAVITY_ACCELERATION * target.getZ()))
				/ -Constants.GRAVITY_ACCELERATION;

		if (t < 0)
			throw new RuntimeException("Time should never be negative (got t=" + t + ").");

		// calculate x and z impulse
		// x = (v_x_robot + v_x_impulse) * t
		// x_target = (v_x_robot + v_x_impulse) * t_target
		// v_x_robot + v_x_impulse = x_target / t_target
		// v_x_impulse = x_target / t_target - v_x_robot
		double xImpulse = target.getX() / t - initialVelocity.getX();
		// same for y
		double yImpulse = target.getY() / t - initialVelocity.getY();
		return new Translation3d(xImpulse, yImpulse, zImpulse);
	}
}
