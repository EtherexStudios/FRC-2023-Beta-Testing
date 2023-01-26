package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.math.controller.ArmFeedforward;

/** Elevator subsystem with feed-forward and PID for position */
public class ArmPID extends SubsystemBase {
    public static final double kMaxSpeed =2.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
  
    private static final int kEncoderResolution = 4096;
    
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    
    private static final double kS = 0.0;
    private static final double kG = 0.0;
    private static final double kV = 0.0;

    private static final double kMetersPerRotation = 2 * Math.PI * kWheelRadius;
    private static final double kRotationsPerMeter = 1 / kMetersPerRotation;
    private static final double kNativeUnitsPerRotation = kEncoderResolution;

    private final WPI_TalonFX m_motor = new WPI_TalonFX(6);
    private final WPI_TalonSRX encoder = new WPI_TalonSRX(0); // add encoder

    // private final PIDController m_motorPIDController = new PIDController(kP, kI, kD);
    public final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAngularSpeed));
    private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV);

    // private TrapezoidProfile m_trapezoidProfile;
    private TrapezoidProfile.State m_setpoint;
    private double feedforward;
    private double pid;
    private TrapezoidProfile.State goal;
        
    /**
    * Reset elevator to bottom
    */
    public ArmPID() {
    m_motor.set(ControlMode.PercentOutput, 0);
    setSetpoint(0);

    // Reset encoders
    m_motor.getSensorCollection().setIntegratedSensorPosition(0, 30);
    encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    /*
    * Convert from TalonFX elevator position to native units
    */
    /*
    public double heightToNative(double rotationUnits) {
        return rotationUnits * kRotationsPerMeter * kNativeUnitsPerRotation;
    }
    */
  
    // set setpoint
    public void setSetpoint(double setpoint) {
        m_setpoint = new TrapezoidProfile.State(setpoint, 0);
        
    }

    /*
    * Compute voltages using feedforward and pid
    */
    @Override
        public void periodic() {
        m_controller.setGoal(m_setpoint); 
        goal = m_controller.getSetpoint();
        feedforward = m_feedforward.calculate(goal.position, goal.velocity);
        pid = m_controller.calculate(encoder.getSelectedSensorPosition(), goal);
        m_motor.setVoltage(feedforward + pid);
        
    }
}
