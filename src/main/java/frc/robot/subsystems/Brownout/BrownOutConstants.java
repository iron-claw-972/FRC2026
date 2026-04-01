package frc.robot.subsystems.Brownout;

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
    
    // level numbers limits
    // -- voltage
    public static final double LEVEL_ONE_LIMIT = 7.75;
    public static final double LEVEL_TWO_LIMIT = 7.25;
    public static final double LEVEL_THREE_LIMIT = 6.75;
    public static final double LEVEL_FOUR_LIMIT = 6.5;
    // -- current
    // if we have been at a certain threshold average for too long (40 seconds) then lower currents
    public static final double CURRENT_THRESHOLD_AMP = 120; // A <-- double check
    public static final double CURRENT_TWENTY_LIMIT = 20.0; // sec
    public static final double CURRENT_TWENTY_FIVE_LIMIT = 25.0; // sec
    public static final double CURRENT_THIRTY_LIMIT = 30.0; // sec
    public static final double CURRENT_THIRTY_FIVE_LIMIT = 35.0; // sec
    
    public static final double RECOVER_TIME = 10.0; // sec
    public static final double DEBOUNCE_CURRENT_TIME = 2.0; //sec

    // normal
    public static final BrownOutLevel BROWNOUT_LVL_ONE = new BrownOutLevel(
        ShooterConstants.SHOOTER_CURRENT_LIMIT, 
        HoodConstants.NORMAL_CURRENT_LIMIT, 
        SpindexerConstants.currentLimit, 
        TurretConstants.NORMAL_CURRENT_LIMIT, 
        IntakeConstants.NORMAL_CURRENT_LIMIT, 
        DriveConstants.STEER_PEAK_CURRENT_LIMIT, 
        DriveConstants.DRIVE_PEAK_CURRENT_LIMIT,
        1
    );

    // should deplete drivetrain a bit and lower everything else slightly. Preserve Shooter.
    public static final BrownOutLevel BROWNOUT_LVL_TWO = new BrownOutLevel(
        ShooterConstants.SHOOTER_CURRENT_LIMIT * 1.0, // keep as same 
        HoodConstants.NORMAL_CURRENT_LIMIT * 1.0, // preserve aiming speed
        SpindexerConstants.currentLimit * 1.0, // preserve indexing speed
        TurretConstants.NORMAL_CURRENT_LIMIT * 1.0, // preserve aiming speed
        IntakeConstants.NORMAL_CURRENT_LIMIT * 1.0, // preserve indexing speed
        DriveConstants.STEER_PEAK_CURRENT_LIMIT * 0.8, // lower drive rotation
        DriveConstants.DRIVE_PEAK_CURRENT_LIMIT * 0.8,  // lower drive movement
        2
    );

    // lower bps systems: intake & spindexer. Preserve Shooter. Slight lower on evertthing else
    public static final BrownOutLevel BROWNOUT_LVL_THREE = new BrownOutLevel(
        ShooterConstants.SHOOTER_CURRENT_LIMIT * 1.0, // keep as same 
        HoodConstants.NORMAL_CURRENT_LIMIT * 1.0, // preserve aiming speed
        SpindexerConstants.currentLimit * 0.8, // preserve indexing speed
        TurretConstants.NORMAL_CURRENT_LIMIT * 1.0, // preserve aiming speed
        IntakeConstants.NORMAL_CURRENT_LIMIT * 0.8, // preserve indexing speed
        DriveConstants.STEER_PEAK_CURRENT_LIMIT * 0.7, // lower drive rotation
        DriveConstants.DRIVE_PEAK_CURRENT_LIMIT * 0.7,  // lower drive movement
        3
    );

    // lower aiming systems: turret & hood. Preserve Shooter. Slight lower on everything else
    public static final BrownOutLevel BROWNOUT_LVL_FOUR = new BrownOutLevel(
        ShooterConstants.SHOOTER_CURRENT_LIMIT * 1.0, // keep as same 
        HoodConstants.NORMAL_CURRENT_LIMIT * 0.8, // preserve aiming speed
        SpindexerConstants.currentLimit * 0.6, // preserve indexing speed
        TurretConstants.NORMAL_CURRENT_LIMIT * 0.8, // preserve aiming speed
        IntakeConstants.NORMAL_CURRENT_LIMIT * 0.6, // preserve indexing speed
        DriveConstants.STEER_PEAK_CURRENT_LIMIT * 0.5, // lower drive rotation
        DriveConstants.DRIVE_PEAK_CURRENT_LIMIT * 0.5,  // lower drive movement
        4
    );

    // now we have to deplete shooter... THIS IS REALLY BAD IS IT COMES TO THIS.
    public static final BrownOutLevel BROWNOUT_LVL_FIVE = new BrownOutLevel(
        ShooterConstants.SHOOTER_CURRENT_LIMIT * 0.8, // keep as same 
        HoodConstants.NORMAL_CURRENT_LIMIT * 0.7, // preserve aiming speed
        SpindexerConstants.currentLimit * 0.5, // preserve indexing speed
        TurretConstants.NORMAL_CURRENT_LIMIT * 0.7, // preserve aiming speed
        IntakeConstants.NORMAL_CURRENT_LIMIT * 0.5, // preserve indexing speed
        DriveConstants.STEER_PEAK_CURRENT_LIMIT * 0.45, // lower drive rotation
        DriveConstants.DRIVE_PEAK_CURRENT_LIMIT * 0.45,  // lower drive movement
        5
    );

}