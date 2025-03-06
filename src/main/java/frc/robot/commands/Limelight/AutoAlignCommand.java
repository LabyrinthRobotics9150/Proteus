package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;

public class AutoAlignCommand extends Command {
    public enum AlignmentDirection { LEFT, CENTER, RIGHT }

    private final CommandSwerveDrivetrain m_drivetrain;
    private final VisionSubsystem m_limelight;
    private final int m_tagID;
    private final AlignmentDirection m_direction;
    
    private final PIDController m_rotationController;
    private final PIDController m_translationController;
    
    private static final SwerveRequest.RobotCentric alignRequest = 
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, 
                           int tagID, AlignmentDirection direction) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        m_tagID = tagID;
        m_direction = direction;

        m_rotationController = createRotationController();
        m_translationController = createTranslationController();

        addRequirements(m_limelight);
    }

    private PIDController createRotationController() {
        PIDController controller = new PIDController(0.05, 0.0, 0.001);
        controller.setTolerance(0.01);
        return controller;
    }

    private PIDController createTranslationController() {
        PIDController controller = new PIDController(0.4, 0.0, 0.0006);
        controller.setTolerance(0.01);
        return controller;
    }

    @Override
    public void execute() {
        try {
            RawFiducial target = getTargetFiducial();
            double rotationOutput = calculateRotation(target);
            double translationOutput = calculateTranslation(target);
            
            driveRobot(rotationOutput, translationOutput);
            updateTelemetry(target, rotationOutput, translationOutput);
            
        } catch (VisionSubsystem.NoSuchTargetException e) {
            handleTargetLoss();
        }
    }

    private RawFiducial getTargetFiducial() {
        return (m_tagID == -1) ? 
            m_limelight.getClosestFiducial() : 
            m_limelight.getFiducialWithId(m_tagID);
    }

    private double calculateRotation(RawFiducial fiducial) {
        double setpoint = getRotationSetpoint();
        return m_rotationController.calculate(fiducial.txnc, setpoint);
    }

    private double getRotationSetpoint() {
        switch(m_direction) {
            case LEFT: return VisionConstants.LEFT_ALIGN_TX_SETPOINT;
            case RIGHT: return VisionConstants.RIGHT_ALIGN_TX_SETPOINT;
            default: return 0.0;
        }
    }

    private double calculateTranslation(RawFiducial fiducial) {
        return m_translationController.calculate(fiducial.distToRobot, VisionConstants.TARGET_DISTANCE_METERS);
    }

    private void driveRobot(double rotation, double translation) {
        double scaledRotation = rotation * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.75;
        double scaledTranslation = translation * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.6;
        
        m_drivetrain.setControl(
            alignRequest
                .withRotationalRate(-scaledRotation)
                .withVelocityX(-scaledTranslation)
        );
    }

    private void updateTelemetry(RawFiducial target, double rotation, double translation) {
        SmartDashboard.putNumber("Align/Rotation Output", rotation);
        SmartDashboard.putNumber("Align/Translation Output", translation);
        SmartDashboard.putNumber("Align/TXnc", target.txnc);
        SmartDashboard.putNumber("Align/Distance", target.distToRobot);
    }

    private void handleTargetLoss() {
        System.out.println("Target lost! Stopping...");
        m_drivetrain.setControl(idleRequest);
    }

    @Override
    public boolean isFinished() {
        return m_rotationController.atSetpoint() && m_translationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(idleRequest);
    }

    // Simplified constructors
    public AutoAlignCommand(CommandSwerveDrivetrain d, VisionSubsystem v) {
        this(d, v, -1, AlignmentDirection.CENTER);
    }
    public AutoAlignCommand(CommandSwerveDrivetrain d, VisionSubsystem v, int id) {
        this(d, v, id, AlignmentDirection.CENTER);
    }
    public AutoAlignCommand(CommandSwerveDrivetrain d, VisionSubsystem v, AlignmentDirection dir) {
        this(d, v, -1, dir);
    }
}