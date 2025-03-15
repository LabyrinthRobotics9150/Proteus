package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.Limelight.AutoAlignCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.BooleanSupplier;

public class AutonomousSequence extends SequentialCommandGroup {

    private Pose2d initialPose;
    
    /**
     * Constructs the autonomous sequence.
     *
     * @param drivetrain the swerve drivetrain subsystem
     * @param limelight  the vision subsystem
     * @param elevator   the elevator subsystem
     * @param intake     the intake subsystem
     */
    public AutonomousSequence(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, ElevatorSubsystem elevator, IntakeSubsystem intake) {
        addCommands(
            new DriveForwardCommand(drivetrain, 4.0).withTimeout(3),
        
            // (3) Hold alignment using right-alignment.
            new AutoAlignCommand(drivetrain, limelight, true).withTimeout(4).andThen(new WaitCommand(1)),

            // (4) Raise the elevator to level 4 (approx. 3.9 meters) and hold.
            new ElevatorRaise(elevator, 3.9).withTimeout(1).andThen(new WaitCommand(0.1)),

            // (5) Run the shoot command (spin intake wheels at 0.5 speed) for 2 seconds.
            new ShootCommand(intake, 0.5).withTimeout(1),

            // (6) Lower the elevator back to 0.
            new ElevatorRaise(elevator, 0).withTimeout(2).andThen(new WaitCommand(0.25))

            /* OR, autoalign barge */
            );
    }
    
    // Returns true if the vision is aligned (using a threshold on txnc).
    private static boolean isAligned(VisionSubsystem limelight) {
        try {
            RawFiducial fiducial = limelight.getClosestFiducial();
            return Math.abs(fiducial.txnc) < 1.0; // alignment threshold (degrees)
        } catch (VisionSubsystem.NoSuchTargetException e) {
            return false;
        }
    }
    
    // --- Custom RepeatUntilCommand implementation ---
    private static class RepeatUntilCommand extends Command {
        private final Command command;
        private final BooleanSupplier condition;
        
        public RepeatUntilCommand(Command command, BooleanSupplier condition) {
            this.command = command;
            this.condition = condition;
            addRequirements(command.getRequirements().toArray(new edu.wpi.first.wpilibj2.command.Subsystem[0]));
        }
        
        @Override
        public void initialize() {
            command.initialize();
        }
        
        @Override
        public void execute() {
            if (command.isFinished()) {
                command.end(false);
                command.initialize();
            } else {
                command.execute();
            }
        }
        
        @Override
        public boolean isFinished() {
            return condition.getAsBoolean();
        }
        
        @Override
        public void end(boolean interrupted) {
            command.end(interrupted);
        }
    }
    
    // --- Inner classes for search-pattern commands ---
    
    private static class DriveForwardCommand extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private final Pose2d targetPose;
        private final PIDController xController = new PIDController(0.3, 0, 0.02);
        private final PIDController yController = new PIDController(0.3, 0, 0.02);

        public DriveForwardCommand(CommandSwerveDrivetrain drivetrain, double meters) {
            this.drivetrain = drivetrain;
            Pose2d current = drivetrain.getState().Pose;
            targetPose = new Pose2d(
                current.getX() + meters * current.getRotation().getCos(),
                current.getY() + meters * current.getRotation().getSin(),
                current.getRotation()
            );
            addRequirements(drivetrain);
            xController.setTolerance(0.05);
            yController.setTolerance(0.05);
            drivetrain.setControl(new SwerveRequest.Idle());
        }

        @Override
        public void execute() {
            Pose2d current = drivetrain.getState().Pose;
            double xOutput = xController.calculate(current.getX(), targetPose.getX());
            double yOutput = yController.calculate(current.getY(), targetPose.getY());
            drivetrain.setControl(new SwerveRequest.RobotCentric()
                    .withVelocityX(xOutput)
                    .withVelocityY(yOutput)
                    .withRotationalRate(0));
        }

        @Override
        public boolean isFinished() {
            return xController.atSetpoint() && yController.atSetpoint();
        }

        @Override
        public void end(boolean interrupted) {
            drivetrain.setControl(new SwerveRequest.Idle());
        }
    }

    // Rotates the robot by a given number of degrees.
    private static class RotateCommand extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private final PIDController rotationController = new PIDController(0.05, 0, 0.005);
        private final double targetDegrees;

        public RotateCommand(CommandSwerveDrivetrain drivetrain, double degrees) {
            this.drivetrain = drivetrain;
            targetDegrees = drivetrain.getState().Pose.getRotation().getDegrees() + degrees;
            rotationController.enableContinuousInput(-180, 180);
            rotationController.setTolerance(1.5);
        }

        @Override
        public void execute() {
            double currentDegrees = drivetrain.getState().Pose.getRotation().getDegrees();
            double output = rotationController.calculate(currentDegrees, targetDegrees);
            drivetrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(output));
        }

        @Override
        public boolean isFinished() {
            return rotationController.atSetpoint();
        }

        @Override
        public void end(boolean interrupted) {
            drivetrain.setControl(new SwerveRequest.Idle());
        }
    }
    
    // Follows a trajectory-like path back to a target pose.
    private static class FollowPathCommand extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private final Pose2d targetPose;
        private final PIDController xController = new PIDController(0.4, 0, 0.03);
        private final PIDController yController = new PIDController(0.4, 0, 0.03);
        private final PIDController rotationController = new PIDController(0.1, 0, 0.01);

        public FollowPathCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
            this.drivetrain = drivetrain;
            this.targetPose = targetPose;
            rotationController.enableContinuousInput(-180, 180);
            addRequirements(drivetrain);
            xController.setTolerance(0.1);
            yController.setTolerance(0.1);
            rotationController.setTolerance(2.0);
        }

        @Override
        public void execute() {
            Pose2d current = drivetrain.getState().Pose;
            double xOutput = xController.calculate(current.getX(), targetPose.getX());
            double yOutput = yController.calculate(current.getY(), targetPose.getY());
            double rotationOutput = rotationController.calculate(
                current.getRotation().getDegrees(), 
                targetPose.getRotation().getDegrees()
            );
            drivetrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(xOutput)
                    .withVelocityY(yOutput)
                    .withRotationalRate(rotationOutput));
        }

        @Override
        public boolean isFinished() {
            return xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
        }

        @Override
        public void end(boolean interrupted) {
            drivetrain.setControl(new SwerveRequest.Idle());
        }
    }
}
