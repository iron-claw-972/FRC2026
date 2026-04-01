package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.LED.LED;
import frc.robot.util.HubActive;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.PS5Button;

public class LEDDefaultCommand extends Command {
    private LED led;
    private boolean allianceIsRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    private final PS5Controller controller = new PS5Controller(Constants.DRIVER_JOY);
    private int counter = 0;

    public LEDDefaultCommand(LED led) {
        this.led = led;
        addRequirements(led);
    }

    @Override
    public void execute() {
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // if (vision.oneCameraDisconnected() ||
        // DriverStation.isJoystickConnected(Constants.DRIVER_JOY)) {
        // // flash if camera disconnected
        // led.setStrobeLights(255, 100, 0);
        // controller.endRumble();
        // } else
        if (fiveSecondsBeforeChange() && allianceIsRed) {
            // blink alliance color and rumble if red alliance 5 seconds before hub shifts
            led.setStrobeLights(255, 0, 0);
            //counter = 0;
        } else if (fiveSecondsBeforeChange()) {
            // blink alliance color and rumble if blue alliance 5 seconds before hub shifts
            led.setStrobeLights(0, 0, 255);
            //counter = 0;
        // } else if(controller.get(PS5Button.LB).getAsBoolean()){
        //     counter++;

        //     if (counter == 1) {
        //         led.alternate(255, 0, 0, 0, 0, 255, 5, 0, 67);
        //     } else if (counter == 20) {
        //         led.alternate(0, 0, 255, 255, 0, 0, 5, 0, 67);
        //     }
        //     if (counter >= 40) {
        //         counter = 0;
        // }
        }else if (DriverStation.isAutonomous() && allianceIsRed) {
            // Dimmer light for auto in red alliance
            led.setLEDs(50, 0, 0);
            //counter = 0;
        } else if (DriverStation.isAutonomous()) {
            // Dimmer light for auto in blue alliance
            led.setLEDs(0, 0, 50);
            //counter = 0;
        } else if (allianceIsRed && HubActive.isHubActive()){
            led.setTwoColorWave(255, 0, 0, 255, 255, 255);
            //counter = 0;
        } else if (!allianceIsRed && HubActive.isHubActive()) {
            led.setTwoColorWave(0, 0, 255, 255, 255, 255);
            //counter = 0;
        } else if (!HubActive.isHubActive()){
            led.setLEDs(0, 0, 0);
            //counter = 0;
        }

        // previous LED Hub code

        // } else if (gameData != null && ((allianceIsRed && gameData.equals("B") && matchTime <= 105 && matchTime >= 80)
        //         || (allianceIsRed && gameData.equals("B") && matchTime <= 55 && matchTime >= 30))) {
        //     // turn light off for inactive hub if red alliance and blue inactive first
        //     led.setLEDs(0, 0, 0);
        //     counter = 0;
        // } else if (gameData != null && ((allianceIsRed && gameData.equals("R") && matchTime <= 130 && matchTime >= 105)
        //         || (allianceIsRed && gameData.equals("R") && matchTime <= 80 && matchTime >= 55))) {
        //     // turn light off for inactive hub if red alliance and red inactive first
        //     led.setLEDs(0, 0, 0);
        //     counter = 0;
        // } else if (gameData != null && ((!allianceIsRed && gameData.equals("B") && matchTime <= 130 && matchTime >= 105)
        //         || (!allianceIsRed && gameData.equals("B") && matchTime <= 80 && matchTime >= 55))) {
        //     // turn off lights for inactive hub if blue alliance and blue inactive first
        //     led.setLEDs(0, 0, 0);
        //     counter = 0;
        // } else if (gameData != null && ((!allianceIsRed && gameData.equals("R") && matchTime <= 105 && matchTime >= 80)
        //         || (!allianceIsRed && gameData.equals("R") && matchTime <= 55 && matchTime >= 30))) {
        //     // turn light off for inactive hub if blue alliance and red inactive first
        //     led.setLEDs(0, 0, 0);
        //     counter = 0;
        // } else if (allianceIsRed) {
        //     // Red alliance
        //     led.setTwoColorWave(255, 0, 0, 255, 255, 255);
        //     counter = 0;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean fiveSecondsBeforeChange() {
        double time = DriverStation.getMatchTime();
        if ((time <= 135 && time >= 130) || (time <= 110 && time >= 105) || (time <= 85 && time >= 80)
                || (time <= 60 && time >= 55) || (time <= 35 && time >= 30)) {
            return true;
        }
        return false;
    }
}
