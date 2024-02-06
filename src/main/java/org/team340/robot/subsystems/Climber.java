package org.team340.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.ClimberConstants;
import org.team340.robot.Constants.RobotMap;

// TODO Subject to change. Climber is not finalized mechanically

/**
 * This subsystem is to pull the robot up onto the chain.
 * <br></br><em><b>The disign is not finalized.</b></em>
 */
public class Climber extends GRRSubsystem {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final DigitalInput leftLimit;
    private final DigitalInput rightLimit;
    private final SparkPIDController leftPID;
    private final SparkPIDController rightPID;

    public Climber() {
        super("Climber");
        leftMotor = createSparkMax("Left Motor", RobotMap.CLIMBER_LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = createSparkMax("Right Motor", RobotMap.CLIMBER_RIGHT_MOTOR, MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        leftLimit = createDigitalInput("Left Limit", RobotMap.CLIMBER_LEFT_LIMIT);
        rightLimit = createDigitalInput("Right Limit", RobotMap.CLIMBER_RIGHT_LIMIT);
        leftPID = leftMotor.getPIDController();
        rightPID = rightMotor.getPIDController();

        ClimberConstants.CLIMBER_MOTORS_CONFIG.apply(leftMotor);
        ClimberConstants.CLIMBER_MOTORS_CONFIG.apply(rightMotor);
        ClimberConstants.CLIMBER_ENCODER_CONFIG.apply(leftMotor, leftEncoder);
        ClimberConstants.CLIMBER_ENCODER_CONFIG.apply(rightMotor, rightEncoder);
        ClimberConstants.CLIMBER_PID_CONFIG.apply(leftMotor, leftPID);
        ClimberConstants.CLIMBER_PID_CONFIG.apply(rightMotor, rightPID);
    }

    public boolean getLeftLimit() {
        return !leftLimit.get();
    }

    public boolean getRightLimit() {
        return !rightLimit.get();
    }
}
