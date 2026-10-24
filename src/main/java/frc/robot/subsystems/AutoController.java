package frc.robot.subsystems;

import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
//import com.ctre.phoenix6.swerve.controls.NeutralOut;
//import com.ctre.phoenix6.swerve.utility.wheelForceCalculator.feedforwards
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoController extends SubsystemBase {
    // Optional<DriverStation.Alliance> alliance;

    RobotConfig config;
    CommandSwerveDrivetrain m_drivetrain;
    RobotContainer m_rc;

    public AutoController(RobotContainer rc) {
        m_rc = rc;
        ChassisSpeeds chassisSpeeds = m_drivetrain.getState().Speeds;
        Pose2d pos = m_drivetrain.getState().Pose;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        final SwerveRequest.ApplyRobotSpeeds req = new SwerveRequest.ApplyRobotSpeeds();
        AutoBuilder.configure(
                () -> new Pose2d(m_rc.drivetrain.getState().Pose.getTranslation(), m_rc.drivetrain.getState().Pose.getRotation()), // Robot pose supplier
                m_rc.drivetrain::resetPose, // Method to reset
                // odometry (will be called if your auto has a starting pose)
                () -> m_rc.drivetrain.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> m_rc.drivetrain.setControl( req.withSpeeds(speeds) ), // Method that will drive the robot given
                                                                         // ROBOT RELATIVE ChassisSpeeds. Also
                                                                           // optionally outputs individual module
                                                                           // feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    else return false;
                },
                this // Reference to this subsystem to set requirements
        );
        new EventTrigger("Shoot").onTrue(new InstantCommand( () -> m_rc.WALL_E_Intake.turnIntakeOn()));
    }

    @Override
    public void periodic() {

    }

}