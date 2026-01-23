package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.drive_comm.AimAtTarget;
import frc.robot.commands.gpm.AlphaIntakeBall;
import frc.robot.commands.gpm.AutoShoot;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.IntakeAlpha;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {
  private final GameController driver = new GameController(Constants.DRIVER_JOY);
  private final BooleanSupplier slowModeSupplier = driver.get(Button.RIGHT_JOY);
  private Hood hood;
  private Shooter shooter;
  private IntakeAlpha intake;

  public GameControllerDriverConfig(Drivetrain drive, Hood hood, Shooter shooter, IntakeAlpha intake) {
    super(drive);
    this.hood = hood;
    this.shooter = shooter;
    this.intake = intake;
  }

  @Override
  public void configureControls() {
    // Reset yaw to be away from driver
    driver.get(Button.START).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
        new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI))));

    // Cancel commands
    driver.get(driver.RIGHT_TRIGGER_BUTTON).onTrue(new InstantCommand(() -> {
      getDrivetrain().setIsAlign(false);
      getDrivetrain().setDesiredPose(() -> null);
      CommandScheduler.getInstance().cancelAll();
    }));

    // Align wheels
    // driver.get(PS5Button.MUTE).onTrue(new FunctionalCommand(
    // () -> getDrivetrain().setStateDeadband(false),
    // getDrivetrain()::alignWheels,
    // interrupted -> getDrivetrain().setStateDeadband(true),
    // () -> false, getDrivetrain()).withTimeout(2));

    if (intake != null && shooter != null) {
      // shoots it
      driver.get(driver.RIGHT_TRIGGER_BUTTON).onTrue(
          new SequentialCommandGroup(
              new InstantCommand(() -> shooter.setShooter(-ShooterConstants.SHOOTER_VELOCITY)),
              new InstantCommand(() -> shooter.setFeeder(ShooterConstants.FEEDER_RUN_POWER))))
          .onFalse(
              new InstantCommand(() -> {
                shooter.deactivateShooterAndFeeder();
              }));

      // Intake
      driver.get(driver.RIGHT_TRIGGER_BUTTON).whileTrue(new AlphaIntakeBall(intake));

      if (hood != null) {
        driver.get(Button.B).whileTrue(new AutoShoot(getDrivetrain(), hood, shooter));
      }

      driver.get(Button.A)
          .whileTrue(new AimAtTarget(getDrivetrain(),
              new Pose2d(FieldConstants.HUB_BLUE.toTranslation2d(), Rotation2d.kZero)));
    }
  }

  @Override
  public double getRawForwardTranslation() {
    return driver.get(Axis.LEFT_Y);
  }

  @Override
  public double getRawSideTranslation() {
    return driver.get(Axis.LEFT_X);
  }

  @Override
  public double getRawRotation() {
    return driver.get(Axis.RIGHT_X);
  }

  @Override
  public double getRawHeadingAngle() {
    return Math.atan2(driver.get(Axis.RIGHT_X), -driver.get(Axis.RIGHT_Y)) - Math.PI / 2;
  }

  @Override
  public double getRawHeadingMagnitude() {
    return Math.hypot(driver.get(Axis.RIGHT_X), driver.get(Axis.RIGHT_Y));
  }

  @Override
  public boolean getIsSlowMode() {
    return slowModeSupplier.getAsBoolean();
  }

  @Override
  public boolean getIsAlign() {
    return false;
    // return kDriver.LEFT_TRIGGER_BUTTON.getAsBoolean();
  }

  public GameController getGameController() {
    return driver;
  }

}
