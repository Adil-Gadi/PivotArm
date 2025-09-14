// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Pivot 30:1, Wrist: 12:1, Claw: 1:1

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
    public static final int pivotMotorId = -1;
    public static final int wristMotorId = -1;
    public static final int wristEncoderId = -1;
    public static final int pivotEncoderId = -1;

    public static final double wristIntakePosition = 75.0 / 360.0; // rotations
    public static final double pivotIntakePosition = 75.0 / 360.0; // rotations
    public static final double wristOuttakePosition = 0 / 360.0; // rotations
    public static final double pivotOuttakePosition = 0 / 360.0; // rotations

    public static final double wristSensorToMechanismRatio = 1.0 / 12.0; // change to your mechanism ratio
    public static final double wristRotorToSensorRatio = 1.0; // 1.0 for internal sensor (set appropriately if fusing a
                                                             // CANcoder)
    public static final double wristkP = 1;
    public static final double wristkI = 0;
    public static final double wristkD = 0.1;
    public static final double wristkS = 0.1;
    public static final double wristkV = 0.1;
    public static final double wristkA = 0;
    public static final double wristkG = 0;

    public static final double pivotSensorToMechanismRatio = 1.0 / 30.0; // change to your mechanism ratio
    public static final double pivotRotorToSensorRatio = 1.0; // 1.0 for internal sensor (set appropriately if fusing a
                                                             // CANcoder)
    public static final double pivotkP = 1;
    public static final double pivotkI = 0;
    public static final double pivotkD = 0.1;
    public static final double pivotkS = 0.1;
    public static final double pivotkV = 0.1;
    public static final double pivotkA = 0;
    public static final double pivotkG = 0;
  }
}
