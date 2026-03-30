package frc.robot.commands.gpm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.BrownOut.BrownOutConstants;
import frc.robot.util.BrownOut.BrownOutLevel;

public class BrownOutControl extends Command {
    private Shooter shooter;
    private Spindexer spindexer;
    private Turret turret;
    private Intake intake;
    private Hood hood;
    private Drivetrain drivetrain;

    public BrownOutLevel[] levels = {
        BrownOutConstants.BROWNOUT_LVL_ONE, 
        BrownOutConstants.BROWNOUT_LVL_TWO, 
        BrownOutConstants.BROWNOUT_LVL_THREE, 
        BrownOutConstants.BROWNOUT_LVL_FOUR,
        BrownOutConstants.BROWNOUT_LVL_FIVE,
    };

    public BrownOutControl(Shooter shooter, Spindexer spindexer, Turret turret, Intake intake, Hood hood, Drivetrain drivetrain) {
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.turret = turret;
        this.intake = intake;
        this.hood = hood;
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        BrownOutLevel level = monitor();
        applyLevel(level);
    }

    public BrownOutLevel monitor() {
        // pretty sure this is where you get it. Need to check if this is same as logs. 
        // voltage 6.3 is brownout where issues occur, but 4.75 is dead robot
        int level = 1;
        double batteryVoltage = RobotController.getBatteryVoltage();
        if (batteryVoltage > BrownOutConstants.LEVEL_ONE_LIMIT) { // normal
            level = 1;
            return levels[0];
        } else if (batteryVoltage > BrownOutConstants.LEVEL_TWO_LIMIT) { // if 7.5 to 6.75
            level = 2;
            return levels[1]; // lower drivetrain
        } else if (batteryVoltage > BrownOutConstants.LEVEL_THREE_LIMIT) { // if 6.75 to 6.0 (browning out)
            level = 3;
            return levels[2];
        } else if (batteryVoltage > BrownOutConstants.LEVEL_FOUR_LIMIT) { // if 6.0 to 5.0 (mayday)
            level = 4;
            return levels[3];
        } else { // were are on life support at this point 5.25 to 4.75
            level = 5;
            return levels[4];
        }
        // Logger.recordOutput("BrownoutProtection/Level", level);
    }

    public void applyLevel(BrownOutLevel level) {
        double shooterCurrent = level.shooterCurrent;
        double turretCurrent = level.turretCurrent;
        double hoodCurrent = level.hoodCurrent;
        double spindexerCurrent = level.spindexerCurrent;
        double intakeCurrent = level.intakeCurrent;
        double steerCurrent = level.steerCurrent;
        double driveCurrent = level.driveCurrent;

        // apply them / set them
        shooter.setNewCurrentLimit(shooterCurrent);
        turret.setCurrentLimits(turretCurrent);
        hood.setCurrentLimits(hoodCurrent);
        spindexer.setNewCurrentLimit(spindexerCurrent);
        intake.setCurrentLimits(intakeCurrent);
        drivetrain.applyNewModuleCurrents(steerCurrent, driveCurrent);

    }

    @Override
    public void end(boolean interrupted) {
        // Nothing
        applyLevel(levels[0]); // disable
    }
}
