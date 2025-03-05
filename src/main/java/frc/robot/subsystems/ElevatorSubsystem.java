package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    // Leader motor.
    public static final SparkFlex elevatorMotor = new SparkFlex(Constants.OperatorConstants.kElevatorLeaderCanId, MotorType.kBrushless);
    public static final SparkClosedLoopController controller = elevatorMotor.getClosedLoopController();
    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    // Follower motor.
    public static final SparkFlex followerMotor = new SparkFlex(Constants.OperatorConstants.kElevatorFollowerCanId, MotorType.kBrushless);

    // Define gear ratio.
    private final double gearRatio = 50.0;
    // Encoder conversion: 22 teeth * 0.25 in/tooth * 2 stages = 11 inches per revolution, divided by gear ratio.
    private final double encoderFactor = (22.0 * 0.25 * 2.0) / gearRatio;
    // Free speed [in/sec]: (6784 RPM / 60) * encoderFactor.
    private final double freeSpeed = (6784.0 / 60.0) * encoderFactor;

    public static final SparkLimitSwitch forwardLimitSwitch = elevatorMotor.getForwardLimitSwitch();
    public static final SparkLimitSwitch reverseLimitSwitch = elevatorMotor.getReverseLimitSwitch();

    // Motion profile variables.
    private TrapezoidProfile motionProfile = null;
    private final Timer profileTimer = new Timer();
    private TrapezoidProfile.State targetState = new TrapezoidProfile.State();
    private TrapezoidProfile.State currentState = new TrapezoidProfile.State();

    // Motion profile constraints.
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
        6,  // max velocity (inches/sec)
        3    // max acceleration (inches/sec²)
    );
    private double previousPosition;

    public ElevatorSubsystem() {
        // Set SmartDashboard default tuning values.
        SmartDashboard.putNumber("Gravity FF", 1.2);   // kG: Voltage to hold position.
        SmartDashboard.putNumber("Elevator kP", 0.5);
        SmartDashboard.putNumber("Elevator kI", 0.0);
        SmartDashboard.putNumber("Elevator kD", 0.1);
        SmartDashboard.putNumber("maxVel", 2.0);
        SmartDashboard.putNumber("maxAcc", 1.0);
        SmartDashboard.putBoolean("Overwrite Elevator Config", false);
        
        // Initialize encoder and timer.
        elevatorEncoder.setPosition(0);
        profileTimer.reset();
        profileTimer.start();

        // Build leader motor configuration.
        EncoderConfig encoderConfig = new EncoderConfig()
                                        .positionConversionFactor(encoderFactor)
                                        .velocityConversionFactor(encoderFactor / 60.0);
        // Use lower PID gains in closed-loop config.
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig().p(0.5).i(0.0).d(0.1);
        SparkFlexConfig leaderConfig = new SparkFlexConfig();
        leaderConfig.apply(encoderConfig);
        leaderConfig.apply(closedLoopConfig);
        // Set idle mode via setter.
        leaderConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        
        elevatorMotor.configure(
            leaderConfig,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );

        // Build follower motor configuration.
        SparkFlexConfig followerConfig = new SparkFlexConfig();
        // Set inversion in the config.
        followerConfig.inverted(true);
        followerMotor.configure(
            followerConfig,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
    }

    @Override
    public void periodic() {
        // Update tuning constants from SmartDashboard.
        double kG = SmartDashboard.getNumber("Gravity FF", .6);
        double kP = SmartDashboard.getNumber("Elevator kP", 0.7);
        double kI = SmartDashboard.getNumber("Elevator kI", 0.0);
        double kD = SmartDashboard.getNumber("Elevator kD", 0.05);
        if (SmartDashboard.getBoolean("Overwrite Elevator Config", false)) {
            ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig().p(kP).i(kI).d(kD);
            SparkFlexConfig newLeaderConfig = new SparkFlexConfig();
            newLeaderConfig.apply(closedLoopConfig);
            elevatorMotor.configure(
                newLeaderConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters
            );
            SmartDashboard.putBoolean("Overwrite Elevator Config", false);
        }

        if (motionProfile != null) {
            // Calculate the time delta since the last update.
            double dt = profileTimer.get();
            profileTimer.reset();
            // Use actual elapsed time (dt) for trajectory calculation.
            currentState = motionProfile.calculate(dt, currentState, targetState);

            SmartDashboard.putNumber("Target Position", targetState.position);
            SmartDashboard.putNumber("Target Velocity", targetState.velocity);
            SmartDashboard.putNumber("Profile Position", currentState.position);
            SmartDashboard.putNumber("Profile Velocity", currentState.velocity);
            SmartDashboard.putNumber("Current Position", elevatorEncoder.getPosition());
            SmartDashboard.putNumber("Current Velocity", elevatorEncoder.getVelocity());
            SmartDashboard.putNumber("Position Error", currentState.position - elevatorEncoder.getPosition());
            SmartDashboard.putNumber("Velocity Error", currentState.velocity - elevatorEncoder.getVelocity());

            // Check limit switches before commanding movement.
            if ((forwardLimitSwitch.isPressed() && targetState.position > getHeight()) ||
                (reverseLimitSwitch.isPressed() && targetState.position < getHeight())) {
                    if(reverseLimitSwitch.isPressed()) {
                        elevatorEncoder.setPosition(0);
                    }
                    killElevator();
                motionProfile = null;
                System.out.println("Limit switch activated. Elevator movement halted.");
            } else {
                // Define a velocity feedforward constant.
                double kV = 12.0;
                double direction = Math.signum(currentState.velocity); // +1 for up, -1 for down, 0 for stationary
                double arbFF = (direction * kG) + (kV * (currentState.velocity / freeSpeed));
                controller.setReference(
                    currentState.position,
                    SparkBase.ControlType.kPosition,
                    ClosedLoopSlot.kSlot0,
                    arbFF,
                    SparkClosedLoopController.ArbFFUnits.kVoltage
                );
                // Mirror the leader's command to the follower.
                double leaderCommand = elevatorMotor.get();
                followerMotor.set(leaderCommand);
            }
        }
    }

    public void setElevatorSpeed(double speed) {
        motionProfile = null;
        if ((forwardLimitSwitch.isPressed() && speed > 0) ||
            (reverseLimitSwitch.isPressed() && speed < 0)) {
            stopElevator();
        } else {
            elevatorMotor.set(speed);
            followerMotor.set(speed);
        }
    }

    public void stopElevator() {
        elevatorMotor.set(0);
        //followerMotor.set(0);
        targetState = new TrapezoidProfile.State(getHeight(), 0); // Hold current position.
        motionProfile = null;
    }

    public double getHeight() {
        return elevatorEncoder.getPosition();
    }

    public void setHeight(double targetHeight) {
        if ((forwardLimitSwitch.isPressed() && targetHeight > getHeight()) ||
            (reverseLimitSwitch.isPressed() && targetHeight < getHeight())) {
            stopElevator();
        } else {
            System.out.println("Moving elevator to new height: " + targetHeight);
            targetState = new TrapezoidProfile.State(targetHeight, 0);
            currentState = new TrapezoidProfile.State(getHeight(), elevatorEncoder.getVelocity());
            // Determine direction and adjust constraints
            boolean movingUp = targetHeight > getHeight();
            double maxVel = movingUp ? 3.0 : 6.0; // Lower max velocity when moving up
            double maxAcc = movingUp ? 1.5 : 3.0;  // Lower max acceleration when moving up
            motionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVel, maxAcc));
            profileTimer.reset();
        }
    }

    public edu.wpi.first.wpilibj2.command.Command goToHeight(double targetHeight) {
        return new edu.wpi.first.wpilibj2.command.FunctionalCommand(
            () -> {
                targetState = new TrapezoidProfile.State(targetHeight, 0);
                currentState = new TrapezoidProfile.State(getHeight(), elevatorEncoder.getVelocity());
                motionProfile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                        Math.min(SmartDashboard.getNumber("maxVel", 2.0), 2.0),
                        Math.min(SmartDashboard.getNumber("maxAcc", 1.0), 1.0)
                    )
                );
                profileTimer.reset();
            },
            () -> {},
            interrupted -> {},
            () -> Math.abs(getHeight() - targetHeight) < 0.5,
            this
        );
    }

    public void returnToPreviousPosition() {
        setHeight(previousPosition);
    }

    public void killElevator() {
        elevatorMotor.stopMotor();
        followerMotor.stopMotor();
        targetState = new TrapezoidProfile.State(getHeight(), 0);
        motionProfile = null;
    }

    public void onDisable() {
        motionProfile = null;
        elevatorMotor.stopMotor();
        followerMotor.stopMotor();
    }

    public double getVelocity() {
        return elevatorEncoder.getVelocity();
    }
}
