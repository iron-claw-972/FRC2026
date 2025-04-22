package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {
  private final GameController driver = new GameController(Constants.DRIVER_JOY);
  private final BooleanSupplier slowModeSupplier = driver.get(Button.RIGHT_JOY);

  public GameControllerDriverConfig(Drivetrain drive) {
    super(drive);
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
