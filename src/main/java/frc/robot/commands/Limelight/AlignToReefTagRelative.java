package frc.robot.commands.Limelight;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivebase;
  private double tagID = -1;

  private static final SwerveRequest.RobotCentric alignRequest = 
    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivebase) {
    xController = new PIDController(.7, 0.0, 0);  // Vertical movement
    yController = new PIDController(1.3, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(6, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(0);
    rotController.setTolerance(2); // tune

    xController.setSetpoint(.2);
    xController.setTolerance(.5);

    yController.setSetpoint(isRightScore ? 4 : -4);
    yController.setTolerance(.5);

    tagID = LimelightHelpers.getFiducialID("");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      //drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      drivebase.setControl(alignRequest
          .withVelocityX(xSpeed)
          .withVelocityY(ySpeed)
          .withRotationalRate(rotValue)
      );

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
     drivebase.setControl(idleRequest);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.setControl(idleRequest);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(.5) ||
        stopTimer.hasElapsed(.3);
  }
}