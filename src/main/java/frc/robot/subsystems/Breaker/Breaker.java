package frc.robot.subsystems.Breaker;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Breaker extends SubsystemBase {
    public PowerDistribution pDis = new PowerDistribution();
    public Breaker() {

    }
    

    @Override
    public void periodic() {
        
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

    // // currently uses stator currents
    // // IMPORTANT: technically we should be uses supply current but since all of our subsystems use stator...
    // private double getAllCurrentsFromSubsystems() {
    //     double currentWithoutDrivetrain = shooter.getLeftStatorCurrent() +
    //         shooter.getRightStatorCurrent() +
    //         intake.getLeftStatorCurrent() +
    //         intake.getRightStatorCurrent() +
    //         intake.getRollerStatorCurrent() +
    //         hood.getStatorCurrent() +
    //         turret.getStatorCurrent() +
    //         spindexer.getStatorCurrent();
    //     double drivetrainModuleCurrent = 0;
    //     for (double current : drivetrain.getStatorCurrents()) {
    //         drivetrainModuleCurrent += current;
    //     }

    //     return currentWithoutDrivetrain + drivetrainModuleCurrent;
    // }
}
