package team4384.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import team4384.robot.commands.*;
import team4384.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
 /*   private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;*/

    private final int translationAxis = Joystick.kDefaultYChannel;
    private final int strafeAxis = Joystick.kDefaultXChannel;
    private final int rotationAxis = Joystick.kDefaultZChannel;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

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
                    new JoystickButton(driver, 3),
                    new JoystickButton(driver, 4)
            )
        );


        AutoBuilder.configureHolonomic(
            s_Swerve::getPose, 
            s_Swerve::resetOdometry, 
            s_Swerve::getModuleStates, 
            s_Swerve::autoDrive, 
            new HolonomicPathFollowerConfig(5.0, 0.4, new ReplanningConfig()),
            () -> true, 
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
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("paths/First Path");
        return AutoBuilder.followPath(path);
    }

    public void HandleArmJoystick() {
        /** Intake Tuner */

//        if (IntakeTurnUp.getAsBoolean()) {
//            IntakeTuner.Up();
//        }
//        else if (IntakeTurnDown.getAsBoolean() && IntakeTuner.isLimit()) {
//            IntakeTuner.Down();
//        }
//        else {
//            IntakeTuner.stopMotor();
//        }
//
//        /** Intake */
//        SmartDashboard.putBoolean("Suck Cube", IntakeSuckCube.getAsBoolean());
//        if (IntakeSuckCube.getAsBoolean()) {
//            Intake.suckCube();
//        }
//        else if (IntakespitCube.getAsBoolean()) {
//            if (-Manip.getRawAxis(1, .1) == -1) {
//                Intake.spitCube(1);
//            }
//            else {
//                Intake.spitCube();
//            }
//        }
//        else if (IntakeSuckCone.getAsBoolean()) {
//            Intake.suckCone();
//        }
//        else if (IntakespitCone.getAsBoolean()) {
//            Intake.spitCone();
//        }
//        else {
//            Intake.stop();
//        }
//
//        /** Arm Base */
//        if (Manip.getRawAxis(0, .1) == -1) {
//            if (ArmDown.getAsBoolean()) {
//                ArmBase.overrideDown();
//            }
//            else {
//                ArmBase.stop();
//            }
//        }
//        else {
//            if (ArmUp.getAsBoolean()) {
//                ArmBase.Up();
//            }
//            else if (ArmDown.getAsBoolean()) {
//                ArmBase.Down();
//            }
//            else {
//                ArmBase.stop();
//            }
//        }
    }

    public void UpdateSmartBoard() {
//        IntakeTuner.display();
//        ArmBase.display();
    }
}
