// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Elevator.HoldAndReturnCommand;
import frc.robot.commands.Elevator.MoveElevatorCommand;
import frc.robot.commands.Elevator.MoveElevatorManualCommand;
import frc.robot.commands.Intake.BallCommand;
import frc.robot.commands.Intake.IntakeScoreCommand;
import frc.robot.commands.Intake.MovePivotManualCommand;
import frc.robot.commands.Intake.WheelMoveCommand;
import frc.robot.commands.Limelight.AutoAlignCommand;
import frc.robot.commands.Limelight.FollowClosestAprilTagCommand;
import frc.robot.commands.Limelight.AutoAlignCommand.AlignmentDirection;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem(0);
  private final VisionSubsystem limelight = new VisionSubsystem();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final CommandXboxController m_primaryController =
  new CommandXboxController(OperatorConstants.kDriverControllerPort);

private final CommandXboxController m_secondaryController = 
new CommandXboxController(OperatorConstants.kSecondaryControllerPort);
  /* INITIALIZING ALL OBJECTS FOR COMMANDS */

  // primary
  IntakeScoreCommand intakeScoreCommand = new IntakeScoreCommand(m_elevator, m_intake);
  AutoAlignCommand alignRightCommand = new AutoAlignCommand( drivetrain, limelight);
  AutoAlignCommand alignLeftCommand = new AutoAlignCommand(drivetrain, limelight, AlignmentDirection.LEFT);
  
  // secondary
    HoldAndReturnCommand level4Command = new HoldAndReturnCommand(m_elevator, 4);
    HoldAndReturnCommand level3Command = new HoldAndReturnCommand(m_elevator, 2);
    HoldAndReturnCommand level2Command = new HoldAndReturnCommand(m_elevator, .6);
    HoldAndReturnCommand level1Command = new HoldAndReturnCommand(m_elevator, 0);
    WheelMoveCommand wheelMoveCommand = new WheelMoveCommand(m_intake, .2);
    WheelMoveCommand wheelMoveReverseCommand = new WheelMoveCommand(m_intake, -.2);
    BallCommand ballCommand = new BallCommand(m_intake);

  
      private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
      private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  
      /* Setting up bindings for necessary control of the swerve drive platform */
      private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
              .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
      //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
      //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  
      private final Telemetry logger = new Telemetry(MaxSpeed);
      // determines which commands are enabled;
      boolean TESTING_MODE = false;
      // slowmode toggle for command
      public static boolean m_slowMode = false;

      // Deadzone and cubic scaling helper function
      private double shapeInput(double raw, double mixer) {
          final double DEADZONE_RANGE = 0.05;
          final double SLOPE = 1.0 / (1.0 - DEADZONE_RANGE);
          // deadzone
          if (Math.abs(raw) <= DEADZONE_RANGE) {
              return 0.0;
          }
          // adjusting raw input
          double adjusted = raw > 0 ? raw - DEADZONE_RANGE : raw + DEADZONE_RANGE;
          adjusted *= SLOPE;

          return mixer * Math.pow(adjusted, 3) + (1 - mixer) * adjusted;
      }

      public RobotContainer() {
          configureBindings();
      }
  
      private void configureBindings() {
        // drivetrain exponential scaling
        drivetrain.setDefaultCommand(
          drivetrain.applyRequest(() -> {
              // Get raw controller inputs
              double leftY = -m_primaryController.getLeftY();
              double leftX = -m_primaryController.getLeftX();
              double rightX = -m_primaryController.getRightX();
      
              // Apply deadzone and cubic scaling
              double x = shapeInput(leftY, 0.75);
              double y = shapeInput(leftX, 0.75);
              double z = shapeInput(rightX, 0.0);
      
              // Apply base scaling
              x *= 0.8;
              y *= 0.8;
              z *= 1.2;  
      
              // Apply slow mode multipliers when active
              if (m_slowMode) {
                  x *= 0.15;  
                  y *= 0.15;
                  z *= 0.3;  
              }
      
              return drive
                  .withVelocityX(x * MaxSpeed)
                  .withVelocityY(y * MaxSpeed)
                  .withRotationalRate(z * MaxAngularRate);
          })
      );
    
        // DRIVE

        /*
        m_primaryController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_primaryController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_primaryController.getLeftY(), -m_primaryController.getLeftX()))
        ));
        
        
    
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_primaryController.back().and(m_primaryController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_primaryController.back().and(m_primaryController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_primaryController.start().and(m_primaryController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_primaryController.start().and(m_primaryController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */
        // reset the field-centric heading on y
        m_primaryController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    
        drivetrain.registerTelemetry(logger::telemeterize);

      // change binding later - slow mode
    m_primaryController.leftBumper()
    .whileTrue(Commands.runOnce(() -> m_slowMode = true))
    .onFalse(Commands.runOnce(() -> m_slowMode = false));

    if(TESTING_MODE) {

    // Left Bumper - align left
    m_secondaryController.leftBumper()
    .whileTrue(alignLeftCommand);

    // Right Bumper - align right
    m_secondaryController.rightBumper()
    .whileTrue(alignRightCommand);

    // Right Trigger - manual elevator raise
    //m_primaryController.rightTrigger()
    //.whileTrue(MoveElevatorManualCommand.moveElevator(m_elevator, .1));

    // left Trigger - manual elevator lower
    //m_primaryController.leftTrigger()
    //.whileTrue(MoveElevatorManualCommand.moveElevator(m_elevator, -.02));

    m_secondaryController.leftTrigger()
    .whileTrue(wheelMoveCommand);

    m_secondaryController.rightTrigger()
    .whileTrue(wheelMoveReverseCommand);

    //m_primaryController.b()
    //.whileTrue(alignRightCommand);

    // A - Level 4
    m_secondaryController.a()
    .whileTrue(level4Command);

    // B - Level
    m_secondaryController.b()
    .whileTrue(level3Command);

    // X - Level 2
    m_secondaryController.x()
    .whileTrue(level2Command);

    // Y - Level 1
    m_secondaryController.y()
    .whileTrue(level1Command);

    // B - Ball command
    m_primaryController.b()
    .whileTrue(ballCommand);

    m_primaryController.rightTrigger()
    .whileTrue(intakeScoreCommand);

    } else {
    /* PRIMARY */

    // Right Trigger - intake / outtake dependant on where the pivot arm is
    m_primaryController.rightTrigger()
    .onTrue(intakeScoreCommand);

    // Rb - Auto-align to right coral spoke
    m_primaryController.rightBumper()
    .whileTrue(alignRightCommand);

    // Lb - Auto-align to left coral spoke
    m_primaryController.leftBumper()
    .whileTrue(alignLeftCommand);

    /*
      new Trigger(limelight::hasTarget)
          .and(m_primaryController.rightBumper())
          .whileTrue(rb_Command);
  }  
     */

    // B - Ball command
    m_primaryController.b()
    .whileTrue(ballCommand);
    }


    /* SECONDARY */

    // elevator heights

    // A - Level 4
    m_secondaryController.a()
    .whileTrue(level4Command);

    // B - Level 3
    m_secondaryController.b()
    .whileTrue(level3Command);

    // X - Level 2
    m_secondaryController.x()
    .whileTrue(level2Command);

    // Y - Level 1
    m_secondaryController.y()
    .whileTrue(level1Command);

    // RT - intake in
    m_secondaryController.rightTrigger()
    .whileTrue(wheelMoveCommand);
}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
