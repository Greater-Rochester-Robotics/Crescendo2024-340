package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.dashboard.Tunable;
import org.team340.lib.util.GRRSubsystem;
import org.team340.lib.util.rev.RelativeEncoderConfig;
import org.team340.lib.util.rev.SparkMaxConfig;
import org.team340.lib.util.rev.SparkMaxConfig.Frame;
import org.team340.lib.util.rev.SparkPIDControllerConfig;
import org.team340.robot.Constants.RobotMap;

@Logged
public class Amplifier extends GRRSubsystem {

    public static enum AmplifierPosition {
        /** Retracted position. */
        kRetract(0.2),
        /** Extended position. */
        kExtend(5.0);

        private final Tunable<Double> length;

        private AmplifierPosition(double length) {
            this.length = Tunable.doubleValue("Amplifier/Positions/" + this.name(), length);
        }

        private double length() {
            return length.get();
        }
    }

    private static final Tunable<Double> kMinPos = Tunable.doubleValue("Amplifier/kMinPos", 0.0);
    private static final Tunable<Double> kMaxPos = Tunable.doubleValue("Amplifier/kMaxPos", 8.75);
    private static final Tunable<Double> kTareSpeed = Tunable.doubleValue("Amplifier/kTareSpeed", -3.0);

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;

    private double target = 0.0;

    public Amplifier() {
        motor = new CANSparkMax(RobotMap.kAmplifier, MotorType.kBrushed);
        encoder = motor.getAlternateEncoder(Type.kQuadrature, 8192);
        pid = motor.getPIDController();

        pid.setFeedbackDevice(encoder);

        SparkMaxConfig.defaults()
            .setSmartCurrentLimit(20)
            .setIdleMode(IdleMode.kCoast)
            .setInverted(false)
            .setOpenLoopRampRate(0.25)
            .setClosedLoopRampRate(0.25)
            .setPeriodicFramePeriod(Frame.S4, 20)
            .apply(motor);

        new RelativeEncoderConfig()
            .setPositionConversionFactor(1.0)
            .setVelocityConversionFactor(1.0)
            .setInverted(false)
            .apply(motor, encoder);

        new SparkPIDControllerConfig().setPID(0.2, 0.0015, 1.2).setIZone(0.8).apply(motor, pid);

        Tunable.pidController("Amplifier/PID", pid);

        encoder.setPosition(0.0);
    }

    /**
     * Tares the zero position by retracting the amplifier until
     * the command ends, then setting its position to zero.
     */
    public Command tare() {
        return commandBuilder("Amplifier.tare()")
            .onExecute(() -> motor.setVoltage(kTareSpeed.get()))
            .onEnd(() -> {
                motor.stopMotor();
                encoder.setPosition(0.0);
            });
    }

    /**
     * Applies a position to the amplifier. Does not end.
     */
    public Command apply(AmplifierPosition position) {
        return commandBuilder("Amplifier.apply(" + position.name() + ")")
            .onExecute(() -> {
                target = MathUtil.clamp(position.length(), kMinPos.get(), kMaxPos.get());
                pid.setReference(target, ControlType.kPosition);
            })
            .onEnd(() -> motor.stopMotor());
    }
}
