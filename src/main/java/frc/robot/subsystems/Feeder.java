package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase{
    TalonFX FeederMotor=new TalonFX(12);
    public Feeder (){
        
    }
public void turnFeederOn(){
    FeederMotor.set(1);

}
public void turnFeederOff(){
    FeederMotor.set(0);
}
public void reverseFeeder(){
    FeederMotor.set(-1);
}
public void reverseFeederOff(){
    FeederMotor.set(0);
}
}
