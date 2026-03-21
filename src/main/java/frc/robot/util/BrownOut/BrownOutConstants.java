package frc.robot.util.BrownOut;

import frc.robot.constants.IntakeConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.turret.TurretConstants;

public class BrownOutConstants {
    /* level one should be less protected that level two and so on. So the current limits will be higher at lvl one.
        Order is:
        Shooter, (applied to both motors)
        Hood,
        Spindexer,
        Turret,
        Intake,
        Steer (main power draw) (is applied to continous and peak)
        Drive (main power draw) (is applied to continous and peak)
    */

    // currently for show. I would imagine u would decrease movement: drivetrain, then bps impacters: intake/indexing speed, and then a bit on aiming: turret/hood. 
    // I don't see a world where you would decrease shooter current, but we need to do some testing to see how much current we are at when shooting
    
    // normal
    public static final BrownOutLevel BROWNOUT_LVL_ONE = new BrownOutLevel(
        ShooterConstants.SHOOTER_CURRENT_LIMIT, 
        HoodConstants.NORMAL_CURRENT_LIMIT, 
        SpindexerConstants.currentLimit, 
        TurretConstants.NORMAL_CURRENT_LIMIT, 
        IntakeConstants.NORMAL_CURRENT_LIMIT, 
        DriveConstants.STEER_PEAK_CURRENT_LIMIT, 
        DriveConstants.DRIVE_PEAK_CURRENT_LIMIT
    );

    // should deplete drivetrain a bit and lower everything else slightly. Preserve Shooter.
    public static final BrownOutLevel BROWNOUT_LVL_TWO = new BrownOutLevel(
        ShooterConstants.SHOOTER_CURRENT_LIMIT, 
        HoodConstants.NORMAL_CURRENT_LIMIT, 
        SpindexerConstants.currentLimit, 
        TurretConstants.NORMAL_CURRENT_LIMIT, 
        IntakeConstants.NORMAL_CURRENT_LIMIT, 
        DriveConstants.STEER_PEAK_CURRENT_LIMIT, 
        DriveConstants.DRIVE_PEAK_CURRENT_LIMIT
    );

    // lower bps systems: intake & spindexer. Preserve Shooter. Slight lower on evertthing else
    public static final BrownOutLevel BROWNOUT_LVL_THREE = new BrownOutLevel(
        ShooterConstants.SHOOTER_CURRENT_LIMIT, 
        HoodConstants.NORMAL_CURRENT_LIMIT, 
        SpindexerConstants.currentLimit, 
        TurretConstants.NORMAL_CURRENT_LIMIT, 
        IntakeConstants.NORMAL_CURRENT_LIMIT, 
        DriveConstants.STEER_PEAK_CURRENT_LIMIT, 
        DriveConstants.DRIVE_PEAK_CURRENT_LIMIT
    );

    // lower aiming systems: turret & hood. Preserve Shooter. Slight lower on everything else
    public static final BrownOutLevel BROWNOUT_LVL_FOUR = new BrownOutLevel(
        ShooterConstants.SHOOTER_CURRENT_LIMIT, 
        HoodConstants.NORMAL_CURRENT_LIMIT, 
        SpindexerConstants.currentLimit, 
        TurretConstants.NORMAL_CURRENT_LIMIT, 
        IntakeConstants.NORMAL_CURRENT_LIMIT, 
        DriveConstants.STEER_PEAK_CURRENT_LIMIT, 
        DriveConstants.DRIVE_PEAK_CURRENT_LIMIT
    );

    // now we have to deplete shooter... THIS IS REALLY BAD IS IT COMES TO THIS.
    public static final BrownOutLevel BROWNOUT_LVL_FIVE = new BrownOutLevel(
        ShooterConstants.SHOOTER_CURRENT_LIMIT, 
        HoodConstants.NORMAL_CURRENT_LIMIT, 
        SpindexerConstants.currentLimit, 
        TurretConstants.NORMAL_CURRENT_LIMIT, 
        IntakeConstants.NORMAL_CURRENT_LIMIT, 
        DriveConstants.STEER_PEAK_CURRENT_LIMIT, 
        DriveConstants.DRIVE_PEAK_CURRENT_LIMIT
    );

}