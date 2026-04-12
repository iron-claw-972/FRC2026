package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.util.Elastic;
import frc.robot.util.HubActive;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

public class LogCommand extends Command {

    private boolean hubActive = false;

    public LogCommand() {
    }

    @Override
    public void execute() {
        boolean current = HubActive.isHubActive();
        if (!Constants.DISABLE_LOGGING) {
            Logger.recordOutput("HubActive", current);
        }
        
        if (current && !hubActive) {
            Elastic.sendNotification(new Notification(NotificationLevel.INFO, "HUB ACTIVE", ""));
        } else if (!current && hubActive) {
            Elastic.sendNotification(new Notification(NotificationLevel.INFO, "HUB DEACTIVATED", ""));
        }

        hubActive = current;

        // keep this
        SmartDashboard.putString("WON AUTO?", (HubActive.wonAuto()) ? "WON" : "LOST");
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
