package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    public static LaserCan laserCan;
    public static final SparkFlex IntakePivotMotor = new SparkFlex(Constants.OperatorConstants.kIntakePivotCanId, MotorType.kBrushless); 
    public static final SparkFlex IntakeWheelsMotor = new SparkFlex(Constants.OperatorConstants.kIntakeWheelsCanId, MotorType.kBrushless);
    private final PIDController pidController = new PIDController(0.5, 0, 0);

    AbsoluteEncoder intakePivotEncoder = IntakePivotMotor.getAbsoluteEncoder();
    public double HOME_POSITION = 0.83;
    public double BALL_POSITION = 0.38;

    // (max velocity and acceleration)
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(.01, 0.01); // Adjust values as needed

    // profile states
    private TrapezoidProfile.State targetState = new TrapezoidProfile.State(HOME_POSITION, 0);
    private TrapezoidProfile.State currentState = new TrapezoidProfile.State(HOME_POSITION, 0);

    private final Timer timer = new Timer();
    private TrapezoidProfile profile = new TrapezoidProfile(constraints);

    public IntakeSubsystem(int laserCANcanId) {
        timer.start();
        setHeight(HOME_POSITION); // Initialize to home position
        laserCan = new LaserCan(laserCANcanId);

        // Initialize LaserCAN settings
        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.err.println("LaserCAN configuration failed: " + e.getMessage());
        }

    }

    @Override
    public void periodic() {
        // Periodically get the latest measurement from the LaserCAN
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        // Update the current state based on the profile
        currentState = profile.calculate(timer.get(), currentState, targetState);

        // Use the PID controller to follow the profile
        double output = pidController.calculate(getHeight(), currentState.position);
        IntakePivotMotor.set(output);
    }

    // Set the target height for the pivot
    public void setHeight(double targetHeight) {
        targetState = new TrapezoidProfile.State(targetHeight, 0); // Target velocity is 0
        timer.reset(); // Reset the timer for the new profile
    }

    // Get the current height of the pivot
    public double getHeight() {
        return intakePivotEncoder.getPosition();
    }

    // Stop the pivot motor
    public void stopPivot() {
        IntakePivotMotor.stopMotor();
    }

    public double getTargetPosition() {
        return targetState.position;
    }

    // Wheels motor
    public void moveWheel(double speed) {
        IntakeWheelsMotor.set(speed);
    }

    public void stopWheel() {
        IntakeWheelsMotor.stopMotor();
    }

    public double getEncoderPosition() {
        return IntakeWheelsMotor.getEncoder().getPosition();
    }

}