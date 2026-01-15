
package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

class ShooterPhysicsTest {
	private static final double epsilon = .001;

	@BeforeEach
	public void prepare() {
	}

	@AfterEach
	public void cleanup() {
	}

	@Test
	public void basicImpulseTest() {
		Translation3d transform = ShooterPhysics.getRequiredExitVelocity(Translation2d.kZero, Translation3d.kZero, 1);
		assertEquals(0, transform.getX(), epsilon);
		assertEquals(0, transform.getY(), epsilon);
		assertEquals(4.427, transform.getZ(), epsilon);

		Translation3d transform2 = ShooterPhysics.getRequiredExitVelocity(Translation2d.kZero, new Translation3d(10, 0, 0),
				1);
		assertTrue(transform2.getX() > 0, transform2.toString());
		assertEquals(0, transform2.getY(), epsilon);
		assertEquals(4.427, transform2.getZ(), epsilon);

		Translation3d transform3 = ShooterPhysics.getRequiredExitVelocity(Translation2d.kZero,
				new Translation3d(0, -6, 0), 1);
		assertEquals(0, transform3.getX(), epsilon);
		assertTrue(transform3.getY() < 0, transform3.toString());
		assertEquals(4.427, transform3.getZ(), epsilon);

		Translation3d transform4 = ShooterPhysics.getRequiredExitVelocity(Translation2d.kZero,
				new Translation3d(6, 3, 0), 2);
		assertEquals(6. / 3, transform4.getX() / transform4.getY(), epsilon);

		Translation3d transform5 = ShooterPhysics.getRequiredExitVelocity(Translation2d.kZero,
				new Translation3d(12.14, 3.21, 0), 2);
		assertEquals(12.14 / 3.21, transform5.getX() / transform5.getY(), epsilon);

		Translation3d transform6 = ShooterPhysics.getRequiredExitVelocity(Translation2d.kZero,
				new Translation3d(-16.32, 3.3, 0), 2);
		assertEquals(-16.32 / 3.3, transform6.getX() / transform6.getY(), epsilon);
	}

	@Test
	public void initialVelocityTest() {
		Translation2d initialVelocityDiff = new Translation2d(5.1, -6.5);
		Translation3d transform = ShooterPhysics.getRequiredExitVelocity(Translation2d.kZero, new Translation3d(1, 2, 3), 4);
		Translation3d transform2 = ShooterPhysics.getRequiredExitVelocity(initialVelocityDiff, new Translation3d(1, 2, 3),
				4);
		assertEquals(transform.getX(), transform2.getX() + initialVelocityDiff.getX(), epsilon);
		assertEquals(transform.getY(), transform2.getY() + initialVelocityDiff.getY(), epsilon);
		assertEquals(transform.getZ(), transform2.getZ(), epsilon);
	}

	@Test
	public void yawTest() {
		ShooterPhysics.TurretState state1 = ShooterPhysics.getShotParams(Translation2d.kZero, Translation2d.kZero,
				new Translation3d(1, 0, 0), 1);
		// different for this one because it's close to 0, so the angle wraps and
		// assertEquals can't handle that
		assertTrue(Math.abs((state1.yaw()).getRadians()) <= epsilon, state1.toString());

		ShooterPhysics.TurretState state2 = ShooterPhysics.getShotParams(Translation2d.kZero, Translation2d.kZero,
				new Translation3d(0, 1, 0), 1);
		assertEquals(new Rotation2d(Math.PI / 2).getRadians(), state2.yaw().getRadians(), epsilon);

		ShooterPhysics.TurretState state3 = ShooterPhysics.getShotParams(Translation2d.kZero, Translation2d.kZero,
				new Translation3d(-1, 0, 0), 1);
		assertEquals(new Rotation2d(Math.PI).getRadians(), state3.yaw().getRadians(), epsilon);

		ShooterPhysics.TurretState state4 = ShooterPhysics.getShotParams(Translation2d.kZero, Translation2d.kZero,
				new Translation3d(0, -1, 0), 1);
		assertEquals(new Rotation2d(-Math.PI / 2).getRadians(), state4.yaw().getRadians(), epsilon);
	}

	@Test
	public void pitchTest() {
		// check random values are within range
		ShooterPhysics.TurretState state1 = ShooterPhysics.getShotParams(Translation2d.kZero, Translation2d.kZero,
				new Translation3d(1, 0, 0), 1);
		assertTrue(state1.pitch() >= 0 && state1.pitch() <= Math.PI / 2, state1.toString());

		ShooterPhysics.TurretState state2 = ShooterPhysics.getShotParams(new Translation2d(12.2, -1.3),
				new Translation2d(.2, 1.2),
				new Translation3d(1.1, 12, 11.1), 22.1);
		assertTrue(state2.pitch() >= 0 && state2.pitch() <= Math.PI / 2, state2.toString());

		ShooterPhysics.TurretState state3 = ShooterPhysics.getShotParams(new Translation2d(1.9, 9.1),
				new Translation2d(-.3, -8.4),
				new Translation3d(11.2, -13.1, 4.1), 5.6);
		assertTrue(state3.pitch() >= 0 && state3.pitch() <= Math.PI / 2, state3.toString());

		// try a steep shot
		ShooterPhysics.TurretState state4 = ShooterPhysics.getShotParams(Translation2d.kZero, Translation2d.kZero,
				new Translation3d(1, 0, 99), 100);
		assertTrue(state4.pitch() >= Math.PI * 7 / 16 && state4.pitch() <= Math.PI / 2, state4.toString());

		ShooterPhysics.TurretState state5 = ShooterPhysics.getShotParams(Translation2d.kZero, Translation2d.kZero,
				new Translation3d(1, 0, 0), 100);
		assertTrue(state5.pitch() >= Math.PI * 7 / 16 && state5.pitch() <= Math.PI / 2, state5.toString());

		// try a shallow shot
		ShooterPhysics.TurretState state6 = ShooterPhysics.getShotParams(Translation2d.kZero, Translation2d.kZero,
				new Translation3d(100, 50, 1), 2);
		assertTrue(state6.pitch() >= 0 && state6.pitch() <= Math.PI / 16, state6.toString());
	}
}
