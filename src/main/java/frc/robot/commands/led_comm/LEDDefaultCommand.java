package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.drivetrain.Drivetrain;
// import frc.robot.subsystems.outtake.Outtake; // TODO: Outtake subsystem not implemented on current robot
import frc.robot.util.Vision.Vision;

public class LEDDefaultCommand extends Command {
    private Vision vision;
    private LED led;
    // private Outtake outtake;
    private Drivetrain drivetrain;
    // TODO: change this to actual climb coordinate
    private double climbYCoordinate = 1.08;
    private boolean allianceIsRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    private String gameData = DriverStation.getGameSpecificMessage();

    public LEDDefaultCommand(LED led, Drivetrain drivetrain, Vision vision) {
        this.led = led;
        // this.outtake = outtake;
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(led);
    }

    @Override
    public void execute() {
        double matchTime = DriverStation.getMatchTime();
        if (vision.oneCameraDisconnected()) {
            // flash if camera disconnected
            led.setStrobeLights(255, 100, 0);
        } else if (playingDefense()) {
            // When playing defense
            led.defenseLights();
        } else if (DriverStation.isAutonomous() && allianceIsRed){
            // Dimmer light for auto in red alliance
            led.setLEDs(100, 0, 0);
        } else if (DriverStation.isAutonomous()){
            // Dimmer light for auto in blue alliance
            led.setLEDs(0, 0, 100);
        } else if ((allianceIsRed && gameData.equals("B") && matchTime <= 105 && matchTime >= 80) || (allianceIsRed && gameData.equals("B") && matchTime <= 55 && matchTime >= 30)) {
            // turn light off for inactive hub if red alliance and blue inactive first
            led.setLEDs(0, 0, 0);
        } else if ((allianceIsRed && gameData.equals("B") && matchTime <= 130 && matchTime >= 105) || (allianceIsRed && gameData.equals("B") && matchTime <= 80 && matchTime >= 55)) {
            // turn light off for inactive hub if red alliance and red inactive first
            led.setLEDs(0, 0, 0);
        } else if ((gameData.equals("R") && matchTime <= 130 && matchTime >= 105) || (gameData.equals("R") && matchTime <= 80 && matchTime >= 55)) {
            // turn off lights for inactive hub if blue alliance and blue inactive first
            led.setLEDs(0, 0, 0);
        } else if ((gameData.equals("R") && matchTime <= 105 && matchTime >= 80) || (gameData.equals("R") && matchTime <= 55 && matchTime >= 30)) {
            // turn light off for inactive hub if blue alliance and red inactive first
            led.setLEDs(0, 0, 0);
        } else if (allianceIsRed) {
            // Red alliance
            led.setTwoColorWave(255, 0, 0, 255, 255, 255);
        } else {
            // Blue alliance
            led.setTwoColorWave(0, 0, 255, 255, 255, 255);
        }
    }

    // private boolean climbAligned() {
    //     double yCoordinate = drivetrain.getPose().getY();
    //     return Math.abs(yCoordinate - climbYCoordinate) < 0.03;
    // }

    private boolean playingDefense() {
        double xCoordinate = drivetrain.getPose().getX();
        double xCoordinateHalfway = 50;
        if (allianceIsRed) {
            return xCoordinate > xCoordinateHalfway;
        } else if (!allianceIsRed) {
            return xCoordinate < xCoordinateHalfway;
        }
        return false;
    }
}
