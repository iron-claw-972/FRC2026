package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.Constants;

public class ShooterPhysics {
	// pitch in radians, going up from the horizontal
	// exit velocity speed in m/s
	public record TurretState(Rotation2d yaw, double pitch, double exitVel, double height) {
		public boolean satisfies(Constraints constraints) {
			if (height < constraints.height())
				return false;
			if (exitVel > constraints.maxVel())
				return false;
			if (pitch > constraints.maxPitch() || pitch < constraints.minPitch())
				return false;
			return true;
		}
	};

	public record Constraints(double height, double maxVel, double minPitch, double maxPitch) {
	};

	/**
	 * Calculates shot parameters for SOTM using Physics™.
	 *
	 * @param robotVelocity The x and y velocity of the robot, field relative.
	 * @param robotToTarget The robot to target transform. Angles are field
	 *                      relative, positions are with the robot as the origin.
	 * @param height        The peak height the trajectory should reach.
	 * @return A TurretState that represents the shot the robot should take.
	 */
	public static TurretState getShotParams(Translation2d robotVelocity, Translation3d robotToTarget,
			double height) {
		Translation3d exitVel = getRequiredExitVelocity(robotVelocity, robotToTarget, height);
		return cvtShot(exitVel, height);
	}

	public static Optional<TurretState> getConstrainedParams(Translation2d robotVelocity, Translation3d robotToTarget,
			Constraints constraints) {
		// establish a lower bound
		double minHeight = Math.max(Math.max(robotToTarget.getZ(), constraints.height()), 0.01);
		Optional<TurretState> withMinPitch = withAngle(robotVelocity, robotToTarget, constraints.minPitch());
		if (withMinPitch.isPresent())
			minHeight = Math.max(minHeight, withMinPitch.get().height());

		TurretState withMinHeight = cvtShot(getRequiredExitVelocity(robotVelocity, robotToTarget, minHeight),
				minHeight);
		if (withMinHeight.satisfies(constraints))
			return Optional.of(withMinHeight);

		// the only reason this is empty is if the highest posible trajectory is too low
		// to intersect the goal
		Optional<TurretState> withMaxPitchOpt = withAngle(robotVelocity, robotToTarget, constraints.maxPitch());
		if (withMaxPitchOpt.isEmpty())
			return Optional.empty();
		TurretState withMaxPitch = withMaxPitchOpt.get();

		// the range from withMinHeight to withMaxPitch will satisfy pitch and height
		// constraints
		// now we need to satisfy the speed constraint

		TurretState withMinSpeed = withMinimumSpeed(robotVelocity, robotToTarget);
		// ordered such that the first element is valid and the second is not
		Pair<TurretState, TurretState> newRange;

		if (withMinSpeed.height() < withMinHeight.height()) {
			// the minimum speed is below the lower bound, but doesn't satisfy constraints
			return Optional.empty();

		} else if (withMinSpeed.height() > withMaxPitch.height()) {
			// the minimum speed is above the upper bound
			if (withMaxPitch.satisfies(constraints))
				// keep optimizing to find the lowest height
				newRange = new Pair<TurretState, TurretState>(withMaxPitch, withMinHeight);
			else
				return Optional.empty();

		} else {
			// the minimum speed is within the ok range
			assert withMinSpeed.satisfies(constraints);
			newRange = new Pair<TurretState, TurretState>(withMinSpeed, withMinHeight);
		}

		// now we binary search the new range
		TurretState lastValid = newRange.getFirst();
		// use a 5cm tolerance
		while (Math.abs(newRange.getFirst().height() - newRange.getSecond().height()) > .05) {
			double avgHeight = (newRange.getFirst().height() + newRange.getSecond().height()) / 2;
			TurretState guess = cvtShot(getRequiredExitVelocity(robotVelocity, robotToTarget, avgHeight), avgHeight);
			if (guess.satisfies(constraints)) {
				if (guess.height() < lastValid.height())
					lastValid = guess;
				newRange = new Pair<TurretState, TurretState>(guess, newRange.getSecond());
			} else {
				newRange = new Pair<TurretState, TurretState>(newRange.getFirst(), guess);
			}
		}

		return Optional.of(lastValid);
	}

	public static TurretState cvtShot(Translation3d velocity, double height) {
		Translation2d onGround = velocity.toTranslation2d();
		Rotation2d yaw = onGround.getAngle();
		double magnitude2d = onGround.getNorm();

		double pitch = new Translation2d(magnitude2d, velocity.getZ()).getAngle().getRadians();
		double speed = velocity.getDistance(Translation3d.kZero);

		return new TurretState(yaw, pitch, speed, height);
	}

	/**
	 * Actually does the SOTM math. Assumes shots are from (0, 0, 0). This is only
	 * public so it can be unit tested, and shouldn't be called directly.
	 *
	 * @param robotVelocity The velocity of the robot, field relative.
	 * @param target        The translation from the robot to the target, field
	 *                      relative. Aka, the position of the target if the robot
	 *                      was at the origin.
	 * @param peakZ         The height of the highest point of the generated
	 *                      trajectory.
	 * @return Velocity that should be imparted on the ball, field relative.
	 */
	public static Translation3d getRequiredExitVelocity(Translation2d robotVelocity, Translation3d target,
			double peakZ) {
		if (target.getZ() > peakZ)
			throw new IllegalArgumentException(
					"The target (" + target + ") cannot be higher than the max trajectory height (" + peakZ + ").");

		// z = v_z_exit_vel * t - .5 * g * t²
		// want vertex of this equation to equal peakZ
		// t_vertex = -v_z_exit_vel / -g
		// t_vertex = v_z_exit_vel / g
		// z_vertex = v_z_exit_vel * (v_z_exit_vel / g) - .5 * g * (v_z_exit_vel / g)²
		// peakZ = v_z_exit_vel² / g - .5 * v_z_exit_vel² / g
		// peakZ = .5 * v_z_exit_vel² / g
		// v_z_exit_vel² = 2 * peakZ * g
		// v_z_exit_vel = √(2 * peakZ * g)
		// keep a second variable to avoid floating point precision issues where the
		// sqrt for t ends up being NaN sometimes when peakZ = target.getZ()
		double zExitVelSquared = 2 * peakZ * Constants.GRAVITY_ACCELERATION;
		double zExitVel = Math.sqrt(zExitVelSquared);

		// now we need time to hit target
		// z_target = v_z_exit_vel * t - .5 * g * t²
		// 0 = -.5 * g * t² + v_z_exit_vel * t - z_target
		// quadratic formula
		// t = (-v_z_exit_vel ± √(v_z_exit_vel² - 4 * (-.5 * g) * (-z_target))) / (2 *
		// -.5 * g)
		// t = (-v_z_exit_vel ± √(v_z_exit_vel² - 2 * g * z_target)) / -g
		// onlz use - because we only want the part where it's coming down, and that
		// gives the longer time
		double t = (-zExitVel - Math.sqrt(zExitVelSquared - 2 * Constants.GRAVITY_ACCELERATION * target.getZ()))
				/ -Constants.GRAVITY_ACCELERATION;

		// this is not equivalent to t <= 0 because of NaNs
		if (!(t > 0))
			throw new RuntimeException("Time should never be negative (got t=" + t + " with target: " + target
					+ " and peakZ: " + peakZ + ").");

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
	public static TurretState withMinimumSpeed(Translation2d initialVelocity, Translation3d target) {
		return withMinimumSpeed(initialVelocity, target, 0.001);
	}

	public static TurretState withMinimumSpeed(Translation2d initialVelocity, Translation3d target,
			double tolerance) {
		// trying to calculate a shot for height=0 returns NaN
		double effectiveMinHeight = Math.max(target.getZ(), 0.01);

		// guess a peak height
		double guess = target.getZ() + 2;
		int maxIters = 50;
		while (maxIters >= 0) {
			maxIters--;

			// this will throw an exception, so avoid it
			// we still might have just overshot, so keep checking
			if (guess < effectiveMinHeight)
				guess = effectiveMinHeight;

			Translation3d guessVelocity = getRequiredExitVelocity(initialVelocity, target, guess);
			Translation3d guessVelocityMore = getRequiredExitVelocity(initialVelocity, target, guess + 0.1);
			double derivative = (guessVelocityMore.getNorm() - guessVelocity.getNorm()) / 0.1;
			// System.out.println(guess + "\t\t" + guessVelocity.getNorm() + "\t\t" +
			// derivative);

			// we've already hit minimum height and are trying to go lower
			if (guess <= effectiveMinHeight && derivative > 0)
				return cvtShot(guessVelocity, guess);

			if (Math.abs(derivative) <= tolerance)
				return cvtShot(guessVelocity, guess);

			// desmos guesstimation indicates this works quite well
			guess -= derivative * (Math.pow(guess, 2) + 1) / 10;
		}

		throw new RuntimeException("Failed to compute a trajectory for a minimum speed.");
	}

	public static Optional<TurretState> withAngle(Translation2d initialVelocity, Translation3d target,
			double pitch) {
		return withAngle(initialVelocity, target, pitch, .01);
	}

	public static Optional<TurretState> withAngle(Translation2d initialVelocity, Translation3d target,
			double pitch, double tolerance) {
		if (pitch <= 0 || pitch >= Math.PI / 2)
			throw new IllegalArgumentException("Pitch must be in the range 0 < pitch < pi/2 (got: " + pitch + ").");

		// System.out.println(
		// "Solving for pitch=" + pitch + " with target=" + target + " and
		// initialVelocity=" + initialVelocity);

		// trying to calculate a shot for height=0 returns NaN
		double effectiveMinHeight = Math.max(target.getZ(), 0.01);

		TurretState first = cvtShot(getRequiredExitVelocity(initialVelocity, target, effectiveMinHeight),
				effectiveMinHeight);
		// if a shot requires going up a kilometer, it's probably not doable
		TurretState second = cvtShot(getRequiredExitVelocity(initialVelocity, target, 1000.), 1000.);

		if (first.pitch() > pitch + tolerance)
			return Optional.empty();
		else if (second.pitch() < pitch)
			return Optional.of(second); // it's close enough

		int maxIters = 50;
		var range = new Pair<TurretState, TurretState>(first, second);
		while (maxIters >= 0) {
			maxIters--;
			assert range.getSecond().height() > range.getFirst().height();

			double guessHeight = (range.getFirst().height() + range.getSecond().height()) / 2;
			TurretState guess = cvtShot(getRequiredExitVelocity(initialVelocity, target, guessHeight), guessHeight);

			// System.out.println(range + "\t\tguess=" + guess);

			if (range.getSecond().pitch() - range.getFirst().pitch() <= tolerance) {
				// we've found a valid angle
				if (Math.abs(guess.pitch() - pitch) <= tolerance)
					return Optional.of(guess);
				// we've narrowed the range but haven't found a valid angle
				// should be covered by the checks before the loop
				assert false;
			}

			if (guess.pitch() > pitch)
				range = new Pair<TurretState, TurretState>(range.getFirst(), guess);
			else
				range = new Pair<TurretState, TurretState>(guess, range.getSecond());
		}

		throw new RuntimeException("Solving for angle did not converge (velocity: " + initialVelocity + ", target: "
				+ target + ", pitch: " + pitch + ", tolerance: " + tolerance + ").");
	}
}
