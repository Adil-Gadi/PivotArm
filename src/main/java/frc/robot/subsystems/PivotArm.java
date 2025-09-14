// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// 1 rotation = 360 degrees

public class PivotArm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  TalonFX pivotMotor;
  TalonFX wristMotor;
  CANcoder wristCC = new CANcoder(Constants.PivotArmConstants.wristEncoderId);

  private final MotionMagicVoltage wristMmReq = new MotionMagicVoltage(0).withSlot(0);
  private final MotionMagicVoltage pivotMmReq = new MotionMagicVoltage(0).withSlot(0);

  public PivotArm() {
    pivotMotor = new TalonFX(Constants.PivotArmConstants.pivotMotorId);
    wristMotor = new TalonFX(Constants.PivotArmConstants.wristMotorId);

    // Wrist Motor Configuration
    TalonFXConfiguration wristCfg = new TalonFXConfiguration();

    CANcoderConfiguration wristCCCfg = new CANcoderConfiguration();
      wristCCCfg.MagnetSensor = new MagnetSensorConfigs()
      .withMagnetOffset( /* your measured offset in rotations */ 0.0);
    wristCC.getConfigurator().apply(wristCCCfg);

    FeedbackConfigs wristFb = new FeedbackConfigs();

    wristFb.SensorToMechanismRatio = Constants.PivotArmConstants.wristSensorToMechanismRatio;
    wristFb.RotorToSensorRatio = Constants.PivotArmConstants.wristRotorToSensorRatio;
    wristFb.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristFb.FeedbackRemoteSensorID = Constants.PivotArmConstants.wristEncoderId;
    wristCfg.Feedback = wristFb;

    wristCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Slot0Configs wristS0 = new Slot0Configs();
    // Basic starting points (FRC-ish), you WILL tune these:
    wristS0.kP = Constants.PivotArmConstants.wristkP; // proportional (mechanism units)
    wristS0.kI = Constants.PivotArmConstants.wristkI; // integral
    wristS0.kD = Constants.PivotArmConstants.wristkD; // derivative (per sec)
    // Feedforward terms (recommended in Phoenix 6): kS (static), kV (per rps), kA
    // (per rps^2), kG (gravity)
    wristS0.kS = Constants.PivotArmConstants.wristkS; // overcome friction
    wristS0.kV = Constants.PivotArmConstants.wristkV; // scales with velocity (tune!)
    wristS0.kA = Constants.PivotArmConstants.wristkA; // acceleration FF if needed
    wristS0.kG = Constants.PivotArmConstants.wristkG; // gravity FF for arms/elevators
    wristCfg.Slot0 = wristS0;

    wristCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    wristCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply the whole configuration
    wristMotor.getConfigurator().apply(wristCfg);

    configureMotionMagicForWrist(1, 0.1, 0.1);

    // Pivot Motor Configuration
    TalonFXConfiguration pivotCfg = new TalonFXConfiguration();

    FeedbackConfigs pivotFb = new FeedbackConfigs();

    pivotFb.SensorToMechanismRatio = Constants.PivotArmConstants.pivotSensorToMechanismRatio;
    pivotFb.RotorToSensorRatio = Constants.PivotArmConstants.pivotRotorToSensorRatio;
    wristFb.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristFb.FeedbackRemoteSensorID = Constants.PivotArmConstants.wristEncoderId;
    pivotCfg.Feedback = pivotFb;

    pivotCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Slot0Configs pivotS0 = new Slot0Configs();
    // Basic starting points (FRC-ish), you WILL tune these:
    pivotS0.kP = Constants.PivotArmConstants.pivotkP; // proportional (mechanism units)
    pivotS0.kI = Constants.PivotArmConstants.pivotkI; // integral
    pivotS0.kD = Constants.PivotArmConstants.pivotkD; // derivative (per sec)
    // Feedforward terms (recommended in Phoenix 6): kS (static), kV (per rps), kA
    // (per rps^2), kG (gravity)
    pivotS0.kS = Constants.PivotArmConstants.pivotkS; // overcome friction
    pivotS0.kV = Constants.PivotArmConstants.pivotkV; // scales with velocity (tune!)
    pivotS0.kA = Constants.PivotArmConstants.pivotkA; // acceleration FF if needed
    pivotS0.kG = Constants.PivotArmConstants.pivotkG; // gravity FF for arms/elevators
    pivotS0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotCfg.Slot0 = pivotS0;

    pivotCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    pivotCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotMotor.getConfigurator().apply(pivotCfg);

    configureMotionMagicForPivot(1, 0.1, 0.1);
  }

  private void configureMotionMagicForWrist(double cruiseRps, double accelRps2, double jerkRps3) {
    MotionMagicConfigs mm = new MotionMagicConfigs();
    mm.MotionMagicCruiseVelocity = cruiseRps; // rps
    mm.MotionMagicAcceleration = accelRps2; // rps^2
    mm.MotionMagicJerk = jerkRps3; // rps^3 (jerk)
    wristMotor.getConfigurator().apply(mm);
  }

  private void configureMotionMagicForPivot(double cruiseRps, double accelRps2, double jerkRps3) {
    MotionMagicConfigs mm = new MotionMagicConfigs();
    mm.MotionMagicCruiseVelocity = cruiseRps; // rps
    mm.MotionMagicAcceleration = accelRps2; // rps^2
    mm.MotionMagicJerk = jerkRps3; // rps^3 (jerk)
    pivotMotor.getConfigurator().apply(mm);
  }

  private void setWristPosition(double targetRotations) {
    wristMotor.setControl(wristMmReq.withPosition(targetRotations));
  }

  private void setPivotPosition(double targetRotations) {
    pivotMotor.setControl(pivotMmReq.withPosition(targetRotations));
  }

  public Command intake() {
    return Commands.run(
      () -> {
        setWristPosition(Constants.PivotArmConstants.wristIntakePosition);
        setPivotPosition(Constants.PivotArmConstants.pivotIntakePosition);
      }, this).until(() -> atIntakeSetpoint());
  }

  public Command outtake() {
    return Commands.run(
      () -> {
        setWristPosition(Constants.PivotArmConstants.wristOuttakePosition);
        setPivotPosition(Constants.PivotArmConstants.pivotOuttakePosition);
      }, this).until(() -> atOuttakeSetpoint());
  }

  public boolean atIntakeSetpoint() {
    double wristError = Math.abs(getWristPosition() - Constants.PivotArmConstants.wristIntakePosition);
    double pivotError = Math.abs(getPivotPosition() - Constants.PivotArmConstants.pivotIntakePosition);
    return wristError < 0.02 && pivotError < 0.02; // tolerance
  }

  public boolean atOuttakeSetpoint() {
    double wristError = Math.abs(getWristPosition() - Constants.PivotArmConstants.wristOuttakePosition);
    double pivotError = Math.abs(getPivotPosition() - Constants.PivotArmConstants.pivotOuttakePosition);
    return wristError < 0.02 && pivotError < 0.02; // tolerance
  }

  public double getWristPosition() {
    return wristCC.getAbsolutePosition().getValueAsDouble();
  }

  public double getPivotPosition() {
    return pivotMotor.getRotorPosition().getValueAsDouble();
  }

  public double positionToDeg(double position) {
    return position * 360.0;
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
