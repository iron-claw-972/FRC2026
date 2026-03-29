package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;

public class TurnOffLEDsCommand extends Command {
    private LED led;

    public TurnOffLEDsCommand(LED led) {
        this.led = led;

        addRequirements(led);
    }

    @Override
    public void initialize() {
        led.setLEDs(0, 0, 0);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
