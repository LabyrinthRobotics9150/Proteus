// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondaryControllerPort = 1;

        // SPARK MAX CAN IDs
        // intake
        public static final int kIntakePivotCanId = 1;
        public static final int kIntakeWheelsCanId = 2;
        public static final int kIntakeFunnelWheelCanId = 6;

        // funnel pivot
        public static final int kFunnelPivotCanId = 3;
        
        // elevator
        public static final int kElevatorFollowerCanId = 5;
        public static final int kElevatorLeaderCanId = 4;
  }
  public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "limelight";
    public static final double TARGET_DISTANCE_METERS = 0.3;
    public static final double LEFT_ALIGN_TX_SETPOINT = -0.2;
    public static final double RIGHT_ALIGN_TX_SETPOINT = 0.2;
    public static final int[] ALLOWED_TAG_IDS = {0,1,5,8,9,10,11,12};

    // autoalign constants
    private static final int[] ALLOWED_AUTOALIGN_TAG_IDS = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};

    // Camera pose constants
    public static final double CAMERA_X_OFFSET = 0.3556;
    public static final double CAMERA_Y_OFFSET = 0.1016;
    public static final double CAMERA_Z_OFFSET = 0.3429;
    public static final double CAMERA_ROLL = 0;
    public static final double CAMERA_PITCH = -2;
    public static final double CAMERA_YAW = 0;

    public static final Double ROTATE_P = .1;
    public static final Double ROTATE_I = .00;
    public static final Double ROTATE_D = .001;
    public static final Double TOLERANCE = .01;

    public static final Double MOVE_P = .4;
    public static final Double MOVE_I = .00;
    public static final Double MOVE_D = .0006;

    public static final double STRAFE_P = 0.8;
    public static final double STRAFE_I = 0.0;
    public static final double STRAFE_D = 0.0006;


  }
}
