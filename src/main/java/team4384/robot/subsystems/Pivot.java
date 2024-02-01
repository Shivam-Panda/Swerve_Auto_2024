package team4384.robot.subsystems;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team4384.robot.constants.RobotMap;


public class Pivot extends SubsystemBase {
    private final TalonFX leader = new TalonFX(RobotMap.CAN_Constants.pivotMotor_1);
    private final TalonFX follower = new TalonFX(RobotMap.CAN_Constants.pivotMotor_2);

    public Pivot() {
        follower.setControl(new StrictFollower(leader.getDeviceID()));
    }

    public void raise() {
        leader.set(RobotMap.Speeds.pivotSpeed);
    }

    public void lower() {
        leader.set(RobotMap.Speeds.pivotSpeed * -1);
    }

    // Set Positions
    public void amp_set() {}

    public void intake_stow() {}

    public void full_stow() {}

    public void stage_set() {}

    // Harsh Break

    public void halt() {
        leader.stopMotor();
        follower.stopMotor();
    }

    @Override
    public void periodic() {}
}
