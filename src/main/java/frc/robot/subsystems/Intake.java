package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    TalonFX IntakeMotor=new TalonFX(15);
    PIDController IntakeMotorController = new PIDController(.035, 0.03, 0); // .25 second spin time
    double targetSpeed = 0; // range from -1 (full reverse ) to 1 (full power)
    public Intake (){
        IntakeMotor.setNeutralMode(NeutralModeValue.Coast);
    }


    @Override
    public void periodic(){

        double powerFromPID = IntakeMotorController.calculate(IntakeMotor.getVelocity().getValueAsDouble() , targetSpeed);
        if(targetSpeed == 0){
        IntakeMotor.set(0);

        }
        else{
        IntakeMotor.set(powerFromPID);


        }
    }

public void turnIntakeOn(){
    // IntakeMotor.set(.75);
targetSpeed=75;
}
public void turnIntakeOff(){
    IntakeMotor.set(0);
    targetSpeed=0;
}

public void SpeedyShoot(){
    // IntakeMotor.set(1);
    targetSpeed=100;
}
public void SpeedyShootOff(){
    IntakeMotor.set(0);
    targetSpeed=0;
}
public void IntakeOnly(){
    targetSpeed=40;
    
}
public void IntakeOnlyOff(){
    IntakeMotor.set(0);
    targetSpeed=0;
}
}
