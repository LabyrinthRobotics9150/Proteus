package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorSubsystem extends SubsystemBase {
    public static final SparkFlex elevatorMotor = new SparkFlex(Constants.OperatorConstants.kElevatorLeaderCanId, MotorType.kBrushless);
    public static final SparkClosedLoopController controller = elevatorMotor.getClosedLoopController();
    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    // encoderFactor [ in / rev ]
    // 22 [ tooth / rev ] sprocket * 0.25 [ in / tooth ] chain pitch * 2 cascade stages
    double encoderFactor = 22.0 * 0.25 * 2.0;

    // freeSpeed [ in / sec ]
    // (6784 [ RPM ] / 60 [ seconds / minute ]) * encoderFactor [ in / rev ]
    double freeSpeed = (6784.0 / 60.0) * encoderFactor;
    public static final SparkLimitSwitch forwardLimitSwitch = elevatorMotor.getForwardLimitSwitch();
    public static final SparkLimitSwitch reverseLimitSwitch = elevatorMotor.getReverseLimitSwitch();

    private TrapezoidProfile motionProfile;
    private final Timer profileTimer = new Timer();
    private TrapezoidProfile.State targetState = new TrapezoidProfile.State();
    private TrapezoidProfile.State currentState = new TrapezoidProfile.State();

    // (max velocity and acceleration)
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
        .01, // (units per second) (speed capped)
        .01  // (units per second squared)
    );

    public ElevatorSubsystem() {
        // We need to call `put` once for SmartDashboard to populate the entries, then we call `get` in periodic for the live-config
        SmartDashboard.putNumber("Gravity FF", 0.45);
        SmartDashboard.putNumber("Elevator kP", 0.04);
        SmartDashboard.putNumber("Elevator kI", 0.0);
        SmartDashboard.putNumber("Elevator kD", 0.0);
        SmartDashboard.putNumber("maxVel", 2.0); // altered max speeds for safety
        SmartDashboard.putNumber("maxAcc", 1.0);
        SmartDashboard.putBoolean("Overwrite Elevator Config", false);
        // Initialize the encoder position at 0
        elevatorEncoder.setPosition(0);
        profileTimer.start();

        // velocityConversionFactor [ in/s per RPM ] = encoderFactor [ in / rev ] / 60 [ seconds / minute ]
        EncoderConfig encoderConfig = new EncoderConfig().positionConversionFactor(encoderFactor).velocityConversionFactor(encoderFactor / 60.0);
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig().p(0.04).i(0.0).d(0.0);

        // TODO: add limit switch config here, then change kNoResetSafeParameters to kResetSafeParameters.
        elevatorMotor.configure(
                new SparkFlexConfig().apply(encoderConfig).idleMode(SparkBaseConfig.IdleMode.kBrake).apply(closedLoopConfig),
                SparkBase.ResetMode.kNoResetSafeParameters,
                // NOTE: Persist can be important in case of mid-match temporary power loss to the motor controller (such as in a bad brownout scenario),
                // because if the settings are not saved to the controller, they will not be restored until this constructor is re-ran.
                SparkBase.PersistMode.kPersistParameters);
        // TODO: also configure the follower motor on startup perhaps?
        // Would help in case of firmware update or other operations errantly modifying settings

        // Initialize the motion profile with limits
        motionProfile = new TrapezoidProfile(constraints);
    }

    @Override
    public void periodic() {
        double aFF = SmartDashboard.getNumber("Gravity FF", 0.45);
        double kP = SmartDashboard.getNumber("Elevator kP", 0.04);
        double kI = SmartDashboard.getNumber("Elevator kI", 0.0);
        double kD = SmartDashboard.getNumber("Elevator kD", 0.0);
        if (SmartDashboard.getBoolean("Overwrite Elevator Config", false)) {
            ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig().p(kP).i(kI).d(kD);
            elevatorMotor.configure(
                    new SparkFlexConfig().apply(closedLoopConfig),
                    SparkBase.ResetMode.kNoResetSafeParameters,
                    SparkBase.PersistMode.kNoPersistParameters);
            SmartDashboard.putBoolean("Overwrite Elevator Config", false);
        }
        // Update the motion profile
        if (motionProfile != null) {
            double elapsedTime = profileTimer.get();
            currentState = motionProfile.calculate(0.02, currentState, targetState);

            SmartDashboard.putNumber("Target Position", targetState.position);
            SmartDashboard.putNumber("Target Velocity", targetState.velocity);
            SmartDashboard.putNumber("Profile Position", currentState.position);
            SmartDashboard.putNumber("Profile Velocity", currentState.velocity);
            SmartDashboard.putNumber("Current Position", elevatorEncoder.getPosition());
            SmartDashboard.putNumber("Current Velocity", elevatorEncoder.getVelocity());
            SmartDashboard.putNumber("Position Error", currentState.position - elevatorEncoder.getPosition());
            SmartDashboard.putNumber("Velocity Error", currentState.velocity - elevatorEncoder.getVelocity());

            if (forwardLimitSwitch.isPressed() || reverseLimitSwitch.isPressed()) {
                if ((forwardLimitSwitch.isPressed() && targetState.position > getHeight()) ||
                    (reverseLimitSwitch.isPressed() && targetState.position < getHeight())) {
                    killElevator();
                    motionProfile = null;
            } else {
                controller.setReference(currentState.position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, aFF + 12.0 * (currentState.velocity / freeSpeed), SparkClosedLoopController.ArbFFUnits.kVoltage);
                }
            }
        }
    }

    public void setElevatorSpeed(double speed) {
        motionProfile = null;
        // Safety checks for limit switches
        if ((forwardLimitSwitch.isPressed() && speed > 0) || 
            (reverseLimitSwitch.isPressed() && speed < 0)) {
            stopElevator();
        } else {
            elevatorMotor.set(speed);
        }
    }

    public void stopElevator() {
        elevatorMotor.set(0);
        targetState = new TrapezoidProfile.State(getHeight(), 0); // Hold the current position
        motionProfile = null; // Stop the motion profile
    }

    public double getHeight() {
        return elevatorEncoder.getPosition();
    }

    public void setHeight(double targetHeight) {
        // Safety checks for limit switches
        if ((forwardLimitSwitch.isPressed() && targetHeight > getHeight()) || 
            (reverseLimitSwitch.isPressed() && targetHeight < getHeight())) {
            stopElevator();
        } else {
            // Create a new motion profile
            targetState = new TrapezoidProfile.State(targetHeight, 0);
            currentState = new TrapezoidProfile.State(getHeight(), 0);
            motionProfile = new TrapezoidProfile(constraints);
            profileTimer.reset(); // Reset the timer for the new motion profile
        }
    }

    public Command goToHeight(double targetHeight) {
        return new FunctionalCommand(
            () -> {
                targetState = new TrapezoidProfile.State(targetHeight, 0);
                currentState = new TrapezoidProfile.State(getHeight(), elevatorEncoder.getVelocity());
                motionProfile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                        Math.min(SmartDashboard.getNumber("maxVel", .5), 1.0), // Hard cap at 5.5
                        Math.min(SmartDashboard.getNumber("maxAcc", .5), 1.0)
                    )
                );
                profileTimer.reset();
            },
            () -> {},
            interrupted -> {},
            () -> Math.abs(getHeight() - targetHeight) < 0.5, // Looser tolerance for smooth stop
            this
        );
    }

    public void killElevator() {
        elevatorMotor.stopMotor();
        targetState = new TrapezoidProfile.State(getHeight(), 0); // Hold the current position
        motionProfile = null; // Stop the motion profile
    }

    public void onDisable() {
        motionProfile = null;
        elevatorMotor.stopMotor();
    }

    public double getVelocity() {
        return elevatorEncoder.getVelocity();
    }
}
