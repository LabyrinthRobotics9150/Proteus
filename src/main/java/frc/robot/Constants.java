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

        // funnel pivot
        public static final int kFunnelPivotCanId = 3;
        
        // elevator
        public static final int kElevatorFollowerCanId = 5;
        public static final int kElevatorLeaderCanId = 4;
  }
  public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "";
    public static final Double LEFT_ALIGN_TX_SETPOINT = .3;
    public static final Double RIGHT_ALIGN_TX_SETPOINT = .3;
  }
}
