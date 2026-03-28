package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

public class Music extends Command {
    private Orchestra orchestra;

    public Music(TalonFX[] motors) {
        orchestra = new Orchestra(Filesystem.getDeployDirectory() + "/chirp/file.chrp");
        for (TalonFX motor : motors)
            orchestra.addInstrument(motor);
    }

    @Override
    public void initialize() {
        orchestra.play();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        orchestra.stop();
    }
}
