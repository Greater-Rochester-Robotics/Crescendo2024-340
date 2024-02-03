package org.team340.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.ClimberConstants;
import org.team340.robot.Constants.RobotMap;

public class Climber extends GRRSubsystem {

    private CANSparkMax leftMotor = createSparkMax("Left Climber Motor", RobotMap.CLIMBER_LEFT_MOTOR, MotorType.kBrushless);
    private CANSparkMax rightMotor = createSparkMax("Right Climber Motor", RobotMap.CLIMBER_RIGHT_MOTOR, MotorType.kBrushless);

    private SparkPIDController leftPID = leftMotor.getPIDController();
    private SparkPIDController rightPID = rightMotor.getPIDController();

    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();

    private DigitalInput leftLimit = createDigitalInput("Left Limit", RobotMap.CLIMBER_LEFT_LIMIT);
    private DigitalInput rightLimit = createDigitalInput("Right Limit", RobotMap.CLIMBER_RIGHT_LIMIT);

    public Climber() {
        super("Climber");
        ClimberConstants.CLIMBER_MOTORS_CONFIG.apply(leftMotor);
        ClimberConstants.CLIMBER_MOTORS_CONFIG.apply(rightMotor);

        ClimberConstants.CLIMBER_PID_CONFIG.apply(leftMotor, leftPID);
        ClimberConstants.CLIMBER_PID_CONFIG.apply(rightMotor, rightPID);

        ClimberConstants.CLIMBER_ENCODER_CONFIG.apply(leftMotor, leftEncoder);
        ClimberConstants.CLIMBER_ENCODER_CONFIG.apply(rightMotor, rightEncoder);
    }

    public boolean getLeftLimit() {
        return !leftLimit.get();
    }

    public boolean getRightLimit() {
        return !rightLimit.get();
    }
}
