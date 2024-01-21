package team4384.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import team4384.lib.util.COTSFalconSwerveConstants;
import team4384.lib.util.SwerveModuleConstants;

public class SwerveMap {
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    //public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
    // public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
    public static final double trackWidth = Units.inchesToMeters(20);
    public static final double wheelBase = Units.inchesToMeters(23);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 0.2; //TODO: This must be tuned to specific robot
    public static final double maxTeleSpeed = 0.3; //TODO: This must be tuned to specific robot
    public static final double maxSlowTeleSpeed = 0.2; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 0.5; //TODO: This must be tuned to specific robot
    public static final double maxRotSpeed = .3;
    public static final double maxAcceleration = 0.1;

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class FL0 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 33;
        public static final int angleMotorID = 34;
        public static final int canCoderID = 42;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(5.712890);
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class FR1 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 31;
        public static final int angleMotorID = 32;
        public static final int canCoderID = 41;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(26.71875);
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class BL2 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 35;
        public static final int angleMotorID = 36;
        public static final int canCoderID = 43;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(117.4218875);
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class BR3 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 37;
        public static final int angleMotorID = 38;
        public static final int canCoderID = 44;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(169.54101625);
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
}
