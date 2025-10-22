package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IndexerConstants;

public class Indexer extends SubsystemBase{
    private TalonFX motor;
    private LaserCanInterface sensor;
    private double velocity = 0.0;
    private double sensorDistance = 0.0;

    public Indexer(){
        motor = new TalonFX(IdConstants.INDEXER_MOTOR); // initializing motor with the IdConstants
        if (Robot.isSimulation()){
            // Simulation code here
        }
        else{
            sensor = new LaserCan(IdConstants.INDEXER_SENSOR); // initializing sensor with IdConstants
            try {
                sensor.setRangingMode(RangingMode.SHORT); // the distance of the sensor range (z-axis?)
                sensor.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS); // the time interval the sensor checks (20 miliseconds)
                sensor.setRegionOfInterest(new RegionOfInterest(-4, -4, 8,8)); // width times height from coordiate (-4,-4) (x,y axis)
            } catch(ConfigurationFailedException e){
                // Exception 
            }
        } 
    }
    @Override
    public void periodic(){
        if (Robot.isReal()){
            velocity = motor.getVelocity().getValueAsDouble() / IndexerConstants.GEAR_RATIO;
        }
        else {
            // simulation stuff
        }
        var measurement = sensor.getMeasurement();
        sensorDistance = (measurement == null || measurement.status > 0) ? 314159 : measurement.distance_mm;
        // log stuff here
        
    }

    public void run(){
        motor.set(IndexerConstants.speed);
    }

    public void slow(){
        motor.set(0.6);
    }

    public void reverse(){
        motor.set(-IndexerConstants.speed);
    }

    public void stop(){
        motor.stopMotor();
    }

    public double getVelocity(){
        return velocity;
    }

    public double getSensorDistance(){
        return sensorDistance;
    }
    
}