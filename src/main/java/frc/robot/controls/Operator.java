// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DoNothing;
import frc.robot.commands.gpm.ManualShoot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Controls for the operator, which are almost a duplicate of most of the
 * driver's controls
 */
public class Operator {

  private final PS5Controller driver = new PS5Controller(Constants.OPERATOR_JOY);

  private final Drivetrain drive;
  private final Turret turret;
  private final Hood hood;
  private final Shooter shooter;
  private final Spindexer spindexer;

  public Operator(Drivetrain drive, Turret turret, Hood hood, Shooter shooter, Spindexer spindexer) {
    this.drive = drive;
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    this.spindexer = spindexer;
  }

  public void configureControls() {
    // Cancel commands, could be removed if the operator doesn't need this button
    // driver.get(driver.RIGHT_TRIGGER_BUTTON).onTrue(new InstantCommand(() -> {
    // drive.setIsAlign(false);
    // drive.setDesiredPose(() -> null);
    // CommandScheduler.getInstance().cancelAll();
    // }));

    driver.get(PS5Button.CROSS).toggleOnTrue(new ManualShoot(turret, drive, hood, shooter, spindexer));
  }

  public double getLeftXAxis() {
    return driver.get(PS5Axis.LEFT_X);
  }

  public double getLeftYAxis() {
    return driver.get(PS5Axis.LEFT_Y);
  }

  public double getRightXAxis() {
    return driver.get(PS5Axis.RIGHT_X);
  }

  public double getRightYAxis() {
    return driver.get(PS5Axis.RIGHT_Y);
  }

  public Rotation2d getLeftRotation() {
    return new Rotation2d(getLeftXAxis(), getLeftYAxis());
  }

  public double getRightTrigger() {
    return driver.get(PS5Axis.RIGHT_TRIGGER);
  }

  public double getLeftTrigger() {
    return driver.get(PS5Axis.LEFT_TRIGGER);
  }

}
