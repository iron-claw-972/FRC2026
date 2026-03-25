package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Elastic;
import frc.robot.util.HubActive;
import frc.robot.util.ShooterPhysics;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

public class LogCommand extends Command {

    private boolean hubActive = false;

    public LogCommand() {
    }

    @Override
    public void execute() {
        boolean current = HubActive.isHubActive();
        Logger.recordOutput("HubActive", current);

        if (current && !hubActive) {
            Elastic.sendNotification(new Notification(NotificationLevel.INFO, "HUB ACTIVE", ""));
        } else if (!current && hubActive) {
            Elastic.sendNotification(new Notification(NotificationLevel.INFO, "HUB DEACTIVATED", ""));
        }

        int x = 1/0;

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
