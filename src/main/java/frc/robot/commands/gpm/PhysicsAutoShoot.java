package frc.robot.commands.gpm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.ShooterPhysics;

public class PhysicsAutoShoot extends Command {
	private Turret turret;
	private Hood hood;
	private Shooter shooter;
	private Drivetrain drivetrain;
	private ShooterPhysics.Constraints constraints;

	public PhysicsAutoShoot(Turret turret, Hood hood, Shooter shooter, Drivetrain drivetrain) {
		this.turret = turret;
		this.hood = hood;
		this.shooter = shooter;
		this.drivetrain = drivetrain;

		this.constraints = new ShooterPhysics.Constraints(2.2, ShooterConstants.SHOOTER_VELOCITY,
				HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);

		addRequirements(turret, hood, shooter);
	}

	@Override
	public void execute() {
		ChassisSpeeds chassisSpeeds = drivetrain.getChassisSpeeds();
		Translation2d robotVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond,
				chassisSpeeds.vyMetersPerSecond);
		Translation3d robotToTarget = FieldConstants.getHubTranslation()
				.minus(new Translation3d(drivetrain.getPose().getTranslation()));

		var stateOpt = ShooterPhysics.getConstrainedParams(
				robotVelocity,
				robotToTarget,
				this.constraints);

		if (stateOpt.isEmpty())
			return;
		ShooterPhysics.TurretState state = stateOpt.get();

		turret.setFieldRelativeTarget(state.yaw(), 0.0);
		hood.setFieldRelativeTarget(new Rotation2d(state.pitch()), 0.0);
		shooter.setShooter(state.exitVel());
	}

	@Override
	public void end(boolean interrupted) {
		// stop the turret where it is
		turret.setFieldRelativeTarget(new Rotation2d(turret.getPositionRad()), 0.0);
		hood.setFieldRelativeTarget(new Rotation2d(0), 0.0);
		shooter.setShooter(0);
	}
}
