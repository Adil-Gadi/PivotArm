// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Pivot 30:1, Wrist: 5.583:1, Claw: 12:1

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ClawConstants {
    public static final int clawMotorId = -1;
  }

  public static class PivotArmConstants {
    public static final int pivotMotorId = 12;
    public static final int wristMotorId = 13;
    public static final int wristEncoderId = 14;
    // public static final int pivotEncoderId = -1;

    public static final double wristIntakePosition = 0; // rotations
    public static final double pivotIntakePosition = 0.25; // rotations
    public static final double wristOuttakeLeftPosition = 0.25; // rotations
    public static final double wristOuttakeRightPosition = -wristOuttakeLeftPosition; // rotations
    public static final double pivotOuttakePosition = 0.15; // rotations

    public static final double wristOffset = 0.0876;

    public static final double wristSensorToMechanismRatio = 5.583; // change to your mechanism ratio
    public static final double wristRotorToSensorRatio = 1.0; // 1.0 for internal sensor (set appropriately if fusing a
                                                             // CANcoder)
    public static final double wristkP = 0.6;
    public static final double wristkI = 0.1;
    public static final double wristkD = 0.05;
    public static final double wristkS = 0.1;
    public static final double wristkV = 0.1;
    public static final double wristkA = 0;
    public static final double wristkG = 0;

    public static final double pivotSensorToMechanismRatio = 30.0; // change to your mechanism ratio
    public static final double pivotRotorToSensorRatio = 1.0; // 1.0 for internal sensor (set appropriately if fusing a
                                                             // CANcoder)
    public static final double pivotkP = 0.4;
    public static final double pivotkI = 0.1;
    public static final double pivotkD = 0.05;
    public static final double pivotkS = 0.0;
    public static final double pivotkV = 0.0;
    public static final double pivotkA = 0;
    public static final double pivotkG = 0;
  }
}
