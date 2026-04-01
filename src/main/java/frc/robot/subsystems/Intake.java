package frc.robot.subsystems;

import java.lang.annotation.Target;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
    TalonFX IntakeMotor = new TalonFX(15);
    PIDController IntakeMotorController = new PIDController(.040, 0.03, 0); // .25 second spin time
    double targetSpeed = 0; 
    RobotContainer m_rc;
    InterpolatingDoubleTreeMap table;

    public Intake(RobotContainer rc) {
        m_rc = rc;
        IntakeMotor.setNeutralMode(NeutralModeValue.Coast);
        table = new InterpolatingDoubleTreeMap();
        table.put(2.0, 60.0);
        table.put(3.5, 70.0);
        table.put(3.0, 60.0);

        // IntakeMotor.currentlimit
        var talonFXConfigurator = IntakeMotor.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();

        // enable stator current limit
        limitConfigs.StatorCurrentLimit = 80;
        limitConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(limitConfigs);

    }

    @Override
    public void periodic() {

        double powerFromPID = IntakeMotorController.calculate(IntakeMotor.getVelocity().getValueAsDouble(),
                targetSpeed);
        if (targetSpeed == 0) {
            IntakeMotor.set(0);

        }
        else {
            IntakeMotor.set(powerFromPID);

        }
        calculateFlywheelSpeed();
    }

    double flywheelSpeed = 60;

    public void calculateFlywheelSpeed() { 

        var dist = m_rc.distanceToTag;
     flywheelSpeed = table.get(dist);

    }

    public void turnIntakeOn() {
        // IntakeMotor.set(.75);

        targetSpeed = flywheelSpeed;
    }

    public void turnIntakeOff() {
        IntakeMotor.set(0);
        targetSpeed = 0;
    }

    public void SpeedyShoot() {
        // IntakeMotor.set(1);
        targetSpeed = 100;
    }

    public void SpeedyShootOff() {
        IntakeMotor.set(0);
        targetSpeed = 0;
    }

    public void IntakeOnly() {
        targetSpeed = 40;

    }

    public void IntakeOnlyOff() {
        IntakeMotor.set(0);
        targetSpeed = 0;
    }
    public void IncreaseSpeed(){
        targetSpeed += 10;
     }
    public void DecreaseSpeed(){
        targetSpeed -= 10;
    }
}
