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
        double batteryVoltage = RobotController.getBatteryVoltage();
        if (batteryVoltage > 8.25) {
            return levels[0];
        } else if (batteryVoltage > 7.75) {
            return levels[1];
        } else if (batteryVoltage > 7.25) {
            return levels[2];
        } else if (batteryVoltage > 6.75) {
            return levels[3];
        } else {
            return levels[4];
        }
    }

    public void applyLevel(BrownOutLevel level) {
        double shooterCurrent = level.shooterCurrent;
        double turretCurrent = level.turretCurrent;
        double hoodCurrent = level.hoodCurrent;
        double spindexerCurrent = level.spindexerCurrent;
        double intakeCurrent = level.intakeCurrent;
        double steerCurrent = level.steerCurrent;
        double driveCurrent = level.driveCurrent;

        shooter.setNewCurrentLimit(shooterCurrent);
        turret.setCurrentLimits(turretCurrent);
        hood.setCurrentLimits(hoodCurrent);
        spindexer.setNewCurrentLimit(spindexerCurrent);
        intake.setCurrentLimits(intakeCurrent);

        // TODO: set drivetrain currents. I'll do it once we fix drivetrain.
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing
        applyLevel(levels[0]); // disable
    }
}
