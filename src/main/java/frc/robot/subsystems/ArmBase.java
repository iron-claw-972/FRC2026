package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class ArmBase extends SubsystemBase {

    private Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    private MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    private MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("arm", 25, ArmConstants.START_ANGLE));

    public ArmBase(){
        // Puts the mechanism on the smartdashboard
        SmartDashboard.putData("arm", mechanism2d);
    }
    
    public void setSetpoint(double setpoint){

    }

    public double getAngle(){
        return 0;
    }

    public boolean atSetpoint(){
        return false;
    }

    public void periodic(){

    }

    public void displayPosition(double angle){
        ligament2d.setAngle(angle);
    }

}
