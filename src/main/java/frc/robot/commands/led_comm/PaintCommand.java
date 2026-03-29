package frc.robot.commands.led_comm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;

public class PaintCommand extends Command {
    private int start;
    private int end;
    private LED led;

    public PaintCommand(LED led, int start, int end) {
        this.led = led;
        this.end = end;
        this.start = start;

        addRequirements(led);
    }

    @Override
    public void initialize() {
        led.setSection(0, 0, 255, start, end);
    }

    @Override
    public void execute() {
        led.setTwoColorWave(30, 0, 80, 255, 255, 255);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
