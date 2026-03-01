// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    boolean twoDrivers = false;
    double limelight_aim_proportional()
    {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target     
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .025;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= 3.14;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController drivController = new CommandXboxController(0);
    private final CommandXboxController opController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake WALL_E_Intake = new Intake();
    public final Feeder Feeder_WALL_E = new Feeder();
    
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest( () -> 
                {

                    if(drivController.x().getAsBoolean()){

                        return drive.withVelocityX(-drivController.getLeftY() * MaxSpeed).withVelocityY(-drivController.getLeftX() * MaxSpeed)
                        .withRotationalRate(limelight_aim_proportional()) ;

                    }else{
                        return drive.withVelocityX(-drivController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-drivController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-drivController.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)

                    }

                }
        ));
            //() ->

            //     drive.withVelocityX(-drivController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            //         .withVelocityY(-drivController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            //         .withRotationalRate(-drivController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            // )
 

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        drivController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-drivController.getLeftY(), -drivController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        drivController.back().and(drivController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        drivController.back().and(drivController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        drivController.start().and(drivController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        drivController.start().and(drivController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        drivController.povDown().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        // WALL_E_Intake
        if(twoDrivers){
            opController.leftTrigger().onTrue(new InstantCommand( ()-> WALL_E_Intake.turnIntakeOn()));
            opController.leftTrigger().onFalse(new InstantCommand( ()-> WALL_E_Intake.turnIntakeOff()));
            opController.rightBumper().onTrue(new InstantCommand(()-> Feeder_WALL_E.turnFeederOn()));
            opController.rightBumper().onFalse(new InstantCommand(()-> Feeder_WALL_E.turnFeederOff()));
            opController.leftBumper().onTrue(new InstantCommand(()-> Feeder_WALL_E.reverseFeeder()));
            opController.leftBumper().onFalse(new InstantCommand(()-> Feeder_WALL_E.reverseFeederOff()));
            opController.y().onTrue (new InstantCommand(()->WALL_E_Intake.SpeedyShoot()));
            opController.y().onFalse (new InstantCommand(()->WALL_E_Intake.SpeedyShootOff()));
            opController.rightTrigger().onTrue(new InstantCommand(()->WALL_E_Intake.IntakeOnly()));
            opController.rightTrigger().onFalse(new InstantCommand(()->WALL_E_Intake.IntakeOnlyOff()));



        } else {
             // drivController.rightTrigger().onTrue(new InstantCommand( ()-> WALL_E_Intake.turnIntakeOn()));
            //drivController.rightTrigger().onFalse(new InstantCommand( ()-> WALL_E_Intake.turnIntakeOff()));
           // drivController.leftBumper().onTrue(new InstantCommand(()-> Feeder_WALL_E.turnFeederOn()));
            //drivController.leftBumper().onFalse(new InstantCommand(()-> Feeder_WALL_E.turnFeederOff()));
            //drivController.leftTrigger().onTrue(new InstantCommand(()-> Feeder_WALL_E.reverseFeeder()));
            //drivController.leftTrigger().onFalse(new InstantCommand(()-> Feeder_WALL_E.reverseFeederOff()));
            //drivController.y().onTrue (new InstantCommand(()->WALL_E_Intake.SpeedyShoot()));
            //drivController.y().onFalse (new InstantCommand(()->WALL_E_Intake.SpeedyShootOff()));
            //drivController.rightTrigger().onTrue(new InstantCommand(()->WALL_E_Intake.IntakeOnly()));
            //drivController.rightTrigger().onFalse(new InstantCommand(()->WALL_E_Intake.IntakeOnlyOff()));
        drivController.leftTrigger().onTrue(new InstantCommand( ()-> WALL_E_Intake.turnIntakeOn()));
            drivController.leftTrigger().onFalse(new InstantCommand( ()-> WALL_E_Intake.turnIntakeOff()));
            drivController.rightBumper().onTrue(new InstantCommand(()-> Feeder_WALL_E.turnFeederOn()));
            drivController.rightBumper().onFalse(new InstantCommand(()-> Feeder_WALL_E.turnFeederOff()));
            drivController.leftBumper().onTrue(new InstantCommand(()-> Feeder_WALL_E.reverseFeeder()));
            drivController.leftBumper().onFalse(new InstantCommand(()-> Feeder_WALL_E.reverseFeederOff()));
            drivController.y().onTrue (new InstantCommand(()->WALL_E_Intake.SpeedyShoot()));
            drivController.y().onFalse (new InstantCommand(()->WALL_E_Intake.SpeedyShootOff()));
            drivController.rightTrigger().onTrue(new InstantCommand(()->WALL_E_Intake.IntakeOnly()));
            drivController.rightTrigger().onFalse(new InstantCommand(()->WALL_E_Intake.IntakeOnlyOff()));

        }
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
