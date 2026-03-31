package frc.robot.subsystems.Brownout;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.turret.Turret;

public class BrownOutControl extends SubsystemBase {
    private Shooter shooter;
    private Spindexer spindexer;
    private Turret turret;
    private Intake intake;
    private Hood hood;
    private Drivetrain drivetrain;
    
    PowerDistribution pDis = new PowerDistribution(); // check if we get accurate readings

    public BrownOutLevel[] levels = {
        BrownOutConstants.BROWNOUT_LVL_ONE, 
        BrownOutConstants.BROWNOUT_LVL_TWO, 
        BrownOutConstants.BROWNOUT_LVL_THREE, 
        BrownOutConstants.BROWNOUT_LVL_FOUR,
        BrownOutConstants.BROWNOUT_LVL_FIVE,
    };

    public BrownOutLevel level = levels[0];
    public Timer recoverLevelTimer = new Timer();

    public BrownOutControl(Shooter shooter, Spindexer spindexer, Turret turret, Intake intake, Hood hood, Drivetrain drivetrain) {
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.turret = turret;
        this.intake = intake;
        this.hood = hood;
        this.drivetrain = drivetrain;

        recoverLevelTimer.reset();
        recoverLevelTimer.start();
    }

    @Override
    public void periodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        double currentTotal = getAllCurrentsFromPowerDistribution();
        // double currentMonitoring = getAllCurrentsFromSubsystems(); <-- Can be used if power dis is incorrect

        BrownOutLevel levelToApply = levels[monitor(batteryVoltage, currentTotal) - 1];
        if (level.levelNumber != levelToApply.levelNumber) {
            // no need to overwrite the same thing
            if (level.levelNumber < levelToApply.levelNumber) {
                // if we are limiting more
                level = levelToApply;
                recoverLevelTimer.reset(); // we just decreased allowance so reset so we don't go back up instantly
                recoverLevelTimer.start();
            } else {
                // increasing level
                if (recoverLevelTimer.hasElapsed(BrownOutConstants.RECOVER_TIME)) {
                    // if we've stabilized
                    level = levelToApply;
                    recoverLevelTimer.reset();
                    recoverLevelTimer.start();
                }
            }
            applyLevel(level);
        }

        Logger.recordOutput("BrownoutProtection/Level", level.levelNumber);
        Logger.recordOutput("BrownoutProtection/BatteryVoltage", batteryVoltage);
        Logger.recordOutput("BrownoutProtection/TotalCurrentDistribution", currentTotal);

        // gets the amount of the total draw that the 
        Logger.recordOutput("BrownoutProtection/Distribution/Shooter", shooterPercentageDraw());
        Logger.recordOutput("BrownoutProtection/Distribution/Intake", intakePercentageDraw());
        Logger.recordOutput("BrownoutProtection/Distribution/Hood", hoodPercentageDraw());
        Logger.recordOutput("BrownoutProtection/Distribution/Turret", turretPercentageDraw());
        Logger.recordOutput("BrownoutProtection/Distribution/Spindexer", spindexerPercentageDraw());
        Logger.recordOutput("BrownoutProtection/Distribution/Drivetrain", drivetrainPercentageDraw());

        // elastic :)
        SmartDashboard.putNumber("Brownout Level", level.levelNumber);
        SmartDashboard.putNumber("Brownout Timer", recoverLevelTimer.get());
    }

    public int monitor(double batteryVoltage, double currentTotal) {
        // pretty sure this is where you get it. Need to check if this is same as logs. 
        // voltage 6.3 is brownout where issues occur, but 4.75 is dead robot
        int level = 1;
        
        // used to make sure we don't change levels instantly when returning to higher allowance

        if (batteryVoltage > BrownOutConstants.LEVEL_ONE_LIMIT) { // normal
            level = 1;
        } else if (batteryVoltage > BrownOutConstants.LEVEL_TWO_LIMIT) { // if 7.5 to 6.75
            level = 2;
        } else if (batteryVoltage > BrownOutConstants.LEVEL_THREE_LIMIT) { // if 6.75 to 6.0 (browning out)
            level = 3;
        } else if (batteryVoltage > BrownOutConstants.LEVEL_FOUR_LIMIT) { // if 6.0 to 5.0 (mayday)
            level = 4;
        } else { // were are on life support at this point 5.25 to 4.75
            level = 5;
        }
        return level;
    }

    // currently ignores other stuff
    private double getAllCurrentsFromPowerDistribution() {
        double[] currents = pDis.getAllCurrents();
        double totalCurrent = 0;
        for (int i=0; i<currents.length; i++) {
            totalCurrent += currents[0];
        }
        return totalCurrent;
    }

    // currently uses stator currents
    // IMPORTANT: technically we should be uses supply current but since all of our subsystems use stator...
    private double getAllCurrentsFromSubsystems() {
        double currentWithoutDrivetrain = shooter.getLeftStatorCurrent() +
            shooter.getRightStatorCurrent() +
            intake.getLeftStatorCurrent() +
            intake.getRightStatorCurrent() +
            intake.getRollerStatorCurrent() +
            hood.getStatorCurrent() +
            turret.getStatorCurrent() +
            spindexer.getStatorCurrent();
        double drivetrainModuleCurrent = 0;
        for (double current : drivetrain.getStatorCurrents()) {
            drivetrainModuleCurrent += current;
        }

        return currentWithoutDrivetrain + drivetrainModuleCurrent;
    }

    private double shooterPercentageDraw() {
        double shooterDraw = shooter.getRightStatorCurrent() + shooter.getLeftStatorCurrent();
        return shooterDraw / getAllCurrentsFromSubsystems() * 100;
    }

    private double intakePercentageDraw() {
        double intakeDraw = intake.getRightStatorCurrent() + intake.getLeftStatorCurrent() + intake.getRollerStatorCurrent();
        return intakeDraw / getAllCurrentsFromSubsystems() * 100;
    }

    private double spindexerPercentageDraw() {
        double spindexerDraw = spindexer.getStatorCurrent();
        return spindexerDraw / getAllCurrentsFromSubsystems() * 100;
    }

    private double hoodPercentageDraw() {
        double hoodDraw = hood.getStatorCurrent();
        return hoodDraw / getAllCurrentsFromSubsystems() * 100;
    }

    private double turretPercentageDraw() {
        double turretDraw = turret.getStatorCurrent();
        return turretDraw / getAllCurrentsFromSubsystems() * 100;
    }

    private double drivetrainPercentageDraw() {
        double drivetrainModuleDraw = 0;
        for (double current : drivetrain.getStatorCurrents()) {
            drivetrainModuleDraw += current;
        }
        return drivetrainModuleDraw / getAllCurrentsFromSubsystems() * 100;
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
}
