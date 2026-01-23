package frc.robot.commands.drive_comm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AimAtTarget extends Command {
    private Drivetrain drivetrain;
    private Pose2d pose;
    private Pose2d target;

    public AimAtTarget(Drivetrain drivetrain, Pose2d target) {
        this.drivetrain = drivetrain;
        this.target = target;

        this.pose = drivetrain.getPose();
    }

    @Override
    public void initialize() {
        drivetrain.setIsAlign(true);
    }

    @Override
    public void execute() {
        pose = drivetrain.getPose();

        drivetrain.setAlignAngle(angleToTarget());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setIsAlign(false);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atAlignAngle();
    }

    private double angleToTarget() {
        double run = target.getX() - pose.getX();
        double rise = target.getY() - pose.getY();
        return Math.atan2(run, rise);
    }
    
    public Pose2d getTarget() {
            return this.target;
    }

    // private double robotRelativeAngle(double headingRads) {
    //     double angleToTarget = angleToTarget();
    //     double diff = angleToTarget - headingRads;
    //     return normalizeRadians(diff);
    // }

    // private double normalizeRadians(double angle) {
    //     while (angle > Math.PI) {
    //         angle -= 2.0 * Math.PI;
    //     }
    //     while (angle <= -Math.PI) {
    //         angle += 2.0 * Math.PI;
    //     }
    //     return angle;
    // }

}
