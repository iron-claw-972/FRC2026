// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import lib.controllers.GameController;

/** 
 * Controls for the operator, which are almost a duplicate of most of the driver's controls
 */
public class Operator {

    private final GameController driver = new GameController(Constants.OPERATOR_JOY);

    private final Drivetrain drive;

    public Operator(Drivetrain drive) {
        this.drive = drive;
    }

    public void configureControls() {
        // Cancel commands, could be removed if the operator doesn't need this button
        driver.get(driver.RIGHT_TRIGGER_BUTTON).onTrue(new InstantCommand(() -> {
            drive.setIsAlign(false);
            drive.setDesiredPose(() -> null);
            CommandScheduler.getInstance().cancelAll();
        }));
    }


    public Trigger getRightTrigger(){
        return new Trigger(driver.RIGHT_TRIGGER_BUTTON);
    }
    public Trigger getLeftTrigger(){
        return new Trigger(driver.LEFT_TRIGGER_BUTTON);
    }
    public GameController getGameController(){
        return driver;
    }
}
