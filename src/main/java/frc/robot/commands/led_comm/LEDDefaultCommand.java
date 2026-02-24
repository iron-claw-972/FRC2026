package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.drivetrain.Drivetrain;
// import frc.robot.subsystems.outtake.Outtake; // TODO: Outtake subsystem not implemented on current robot
import frc.robot.util.Vision.Vision;
import lib.controllers.PS5Controller.PS5Button;

public class LEDDefaultCommand extends Command {
    private Vision vision;
    private LED led;
    private PS5Controller controller;
    // private Outtake outtake;
    private Drivetrain drivetrain;
    private boolean allianceIsRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    

    public LEDDefaultCommand(LED led) {
        this.led = led;
        // this.outtake = outtake;
        addRequirements(led);
    }

    @Override
    public void execute() {
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // if (vision.oneCameraDisconnected()) {
        //     // flash if camera disconnected
        //     led.setStrobeLights(255, 100, 0);
        // } else 
        if (fiveSecondsBeforeChange() && allianceIsRed) {
            // blink alliance color and rumble if red alliance 5 seconds before hub shifts
            led.setStrobeLights(255, 0, 0);
            controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
        } else if (fiveSecondsBeforeChange()) {
            // blink alliance color and rumble if blue alliance 5 seconds before hub shifts
            led.setStrobeLights(0, 0, 255);
            controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
        } else 
        if (playingDefense()) {
            new DefenseLightsCommand(led, 0, 67);
        } else 
        if (DriverStation.isAutonomous() && allianceIsRed){
            // Dimmer light for auto in red alliance
            led.setLEDs(50, 0, 0);
        } else if (DriverStation.isAutonomous()){
            // Dimmer light for auto in blue alliance
            led.setLEDs(0, 0, 50);
        } else if ((allianceIsRed && gameData.equals("B") && matchTime <= 105 && matchTime >= 80) || (allianceIsRed && gameData.equals("B") && matchTime <= 55 && matchTime >= 30)) {
            // turn light off for inactive hub if red alliance and blue inactive first
            led.setLEDs(0, 0, 0);
        } else if ((allianceIsRed && gameData.equals("R") && matchTime <= 130 && matchTime >= 105) || (allianceIsRed && gameData.equals("R") && matchTime <= 80 && matchTime >= 55)) {
            // turn light off for inactive hub if red alliance and red inactive first
            led.setLEDs(0, 0, 0);
        } else if ((gameData.equals("B") && matchTime <= 130 && matchTime >= 105) || (gameData.equals("B") && matchTime <= 80 && matchTime >= 55)) {
            // turn off lights for inactive hub if blue alliance and blue inactive first
            led.setLEDs(0, 0, 0);
        } else if ((gameData.equals("R") && matchTime <= 105 && matchTime >= 80) || (gameData.equals("R") && matchTime <= 55 && matchTime >= 30)) {
            // turn light off for inactive hub if blue alliance and red inactive first
            led.setLEDs(0, 0, 0);
        } else if (allianceIsRed) {
            // Red alliance
            // TODO need to fix 2 color wave
            //led.setTwoColorWave(255, 0, 0, 255, 255, 255);
            led.setLEDs(255, 0, 0);
        } else {
            // Blue alliance
            // led.setTwoColorWave(0, 0, 255, 255, 255, 255);
            // TODO remake 2 color wave
            led.setLEDs(0, 0, 255);
        }
    }

    private boolean fiveSecondsBeforeChange() {
        double time = DriverStation.getMatchTime();
        if((time <= 135 && time >= 130) || (time <= 110 && time >= 105) || (time <= 85 && time >= 80) || (time <= 60 && time >= 55) || (time <= 35 && time >= 30)){
            return true;
        }
        return false;
    }

    private boolean playingDefense() {
        // TODO: add automatic defense lights
        return false;
    }
}
