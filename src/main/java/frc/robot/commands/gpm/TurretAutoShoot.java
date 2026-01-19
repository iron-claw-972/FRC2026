package frc.robot.commands.gpm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.Vision.TurretVision;

public class TurretAutoShoot extends Command {
    private Turret turret;
    private Drivetrain drivetrain;
    private TurretVision turretVision;

    double turretSetpoint;
    double adjustedSetpoint;
    double yawToTagCamera;
    double yawToTag;

    private boolean turretCamEnabled = true;

    public TurretAutoShoot(Turret turret, Drivetrain drivetrain, TurretVision turretVision){
        this.turret = turret;
        this.drivetrain = drivetrain;
        this.turretVision = turretVision;
    }

    public void align() {
        Translation2d drivepose = drivetrain.getPose().getTranslation();
        Translation2d target = FieldConstants.getHubTranslation().toTranslation2d();
        double D_y = target.getY() - drivepose.getY();
        double D_x = target.getX() - drivepose.getX();
        double angleRad = Math.atan2(D_y, D_x);
        System.out.println("Aligning the turn to degree angle: " + Units.radiansToDegrees(angleRad));
        turretSetpoint = Units.radiansToDegrees(angleRad);
    }

    public void adjustWithTurretCam(){
        if(turretVision.canSeeTag(26) || turretVision.canSeeTag(10)){
            if(Robot.getAlliance() == Alliance.Blue){
                yawToTagCamera = -1 * turretVision.getYawToTagRad(26).get();
            }
            else{
                yawToTagCamera = -1 * turretVision.getYawToTagRad(10).get();
            }
            double error = yawToTagCamera - yawToTag;
            adjustedSetpoint += error;
        }
    }

    public void updateYawToTag(){
        double D_y = FieldConstants.getHubTranslation().getY() - drivetrain.getPose().getY();
        double D_x = FieldConstants.getHubTranslation().getX() - drivetrain.getPose().getX();
        double angleToTag = Units.radiansToDegrees(Math.atan(D_y / D_x));
        yawToTag = angleToTag - turretSetpoint;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    
}
