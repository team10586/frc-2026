package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase{
    TalonFX FeederMotor=new TalonFX(12);

    public Feeder (){
          var talonFXConfigurator = FeederMotor.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();
 
          limitConfigs.StatorCurrentLimit = 40;
        limitConfigs.StatorCurrentLimitEnable = true;
    }
public void turnFeederOn(){
    FeederMotor.set(0.4);

}
public void turnFeederOff(){
    FeederMotor.set(0);
}
public void reverseFeeder(){
    FeederMotor.set(-.4);
}
public void reverseFeederOff(){
    FeederMotor.set(0);
}
}
