package frc.robot.subsystems.Brownout;

public class BrownOutLevel {
    // in percentage values (0.0-1.0)
    public double shooterCurrent; // A  -> priority one
    public double hoodCurrent; // A -> priority two
    public double turretCurrent; // A -> priority two
    public double spindexerCurrent; // A -> priority three
    public double intakeCurrent; // A -> priority three
    public double steerCurrent; // A -> priority four? (idk)
    public double driveCurrent; // A -> priority four? (idk)
    public int levelNumber;

    public BrownOutLevel(
        double shooterCurrent, 
        double hoodCurrent,
        double spindexerCurrent,
        double turretCurrent,
        double intakeCurrent,
        double steerCurrent,
        double driveCurrent,
        int levelNumber
    ) {
        this.shooterCurrent = shooterCurrent;
        this.hoodCurrent = hoodCurrent;
        this.spindexerCurrent = spindexerCurrent;
        this.turretCurrent = turretCurrent;
        this.intakeCurrent = intakeCurrent;
        this.steerCurrent = steerCurrent;
        this.driveCurrent = driveCurrent;
        this.levelNumber = levelNumber;
    }

}

