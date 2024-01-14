package team4384.robot.constants;

public class RobotMap {
    public static final double stickDeadband = 0.1; // Joystick Deadband

    /* Motor Controller CAN IDs */
    public static final int INTAKE_TURNER = 51;
    public static final int INTAKE = 52;
    public static final int ARM_BASE_LEFT = 49;
    public static final int ARM_BASE_RIGHT = 50;

    /* Motor Controller Speeds */
    public static final double INTAKE_TURN_SPEED = .5;
    public static final double CUBE_SPEED = .4;
    public static final double CONE_SPEED = .3;
    /* Digital Input IDs */
    public static final int INTAKE_TURNER_LIMIT_SWITCH = 0;
    public static final int ARM_BASE_LIMIT_SWITCH = 1;
    public static final double ARM_BASE_SPEED = .4;
    public static final double OVERRIDE_ARM_BASE_SPEED = .26;
}
