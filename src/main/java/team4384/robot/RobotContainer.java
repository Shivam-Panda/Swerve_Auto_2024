package team4384.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import team4384.robot.commands.*;
import team4384.robot.constants.SwerveMap;
import team4384.robot.subsystems.*;

import java.util.concurrent.ConcurrentMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public final Joystick driver = new Joystick(0);

    private final JoystickButton forward = new JoystickButton(driver, 7);

    /* Drive Controls */
 /*   private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;*/

    private final int translationAxis = Joystick.kDefaultYChannel;
    private final int strafeAxis = Joystick.kDefaultXChannel;
    private final int rotationAxis = Joystick.kDefaultZChannel;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton zeroOdo = new JoystickButton(driver, 3);
    private final JoystickButton turner = new JoystickButton(driver, 8);

    private final double kP = 0.08;
    private final double kD = 1.0;
    private final double kI = 0.1;


    /* Subsystems */
//    private  BbIntakeTurner IntakeTuner = new BbIntakeTurner();
//    private  BbIntake Intake = new BbIntake();
//    private  BbArmBase ArmBase = new BbArmBase();
    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
       s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> driver.getRawAxis(rotationAxis),
                    () -> driver.getRawAxis(3),
                    new JoystickButton(driver, 1),
                    new JoystickButton(driver, 4)
            )
        );


        AutoBuilder.configureHolonomic(
            s_Swerve::getPose, 
            s_Swerve::resetOdometry, 
            s_Swerve::getModuleStates, 
            s_Swerve::autoDrive, 
            new HolonomicPathFollowerConfig(
                    new PIDConstants(kP, kI, 0),
                    new PIDConstants(kP, kI, 0),
                    5.0,
                    0.38,
                    new ReplanningConfig()
            ),
            () -> false,
            s_Swerve);
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        zeroOdo.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())));

        forward.onTrue(new Command() {
            private final Timer timer = new Timer();
            @Override
            public void initialize() {
                timer.reset();
                timer.start();
            }
            @Override
            public void execute() {
                while (timer.get() <= 1) {
                    s_Swerve.drive(
                            new Translation2d(0.05, 0),
                            0,
                            true,
                            true
                    );
                    s_Swerve.swerveOdometry.update(s_Swerve.getYaw(), s_Swerve.getModulePositions());
                }
                while(timer.get() >= 1 && timer.get() <= 2) {
                    s_Swerve.drive(
                            new Translation2d(0, 0.05),
                            0,
                            true,
                            true
                    );
                    s_Swerve.swerveOdometry.update(s_Swerve.getYaw(), s_Swerve.getModulePositions());
                }
                while(timer.get() >= 2 && timer.get() <= 3) {
                    s_Swerve.drive(
                            new Translation2d(-0.05, 0),
                            0,
                            true,
                            true
                    );
                    s_Swerve.swerveOdometry.update(s_Swerve.getYaw(), s_Swerve.getModulePositions());
                }
                while(timer.get() >= 3 && timer.get() <= 4) {
                    s_Swerve.drive(
                            new Translation2d(0, -0.05),
                            0,
                            true,
                            true
                    );
                    s_Swerve.swerveOdometry.update(s_Swerve.getYaw(), s_Swerve.getModulePositions());
                }
            }
        });

        turner.onTrue(new InstantCommand() {
            private double initGyro;
            @Override
            public void initialize() {
               super.initialize();
               s_Swerve.gyro.reset();
               initGyro = s_Swerve.gyro.getRotation2d().getDegrees();
            }

            @Override
            public void execute() {
                while(s_Swerve.gyro.getRotation2d().getDegrees() - initGyro <= 180) {
                    s_Swerve.drive(
                            new Translation2d(0,0),
                            0.05,
                            true,
                            true
                    );
                }

            }
        });
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("New Path");
        return AutoBuilder.followPath(path);
    }
}
