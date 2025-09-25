package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

// 1 rotation = 360 degrees

public class PivotArm extends SubsystemBase {

  TalonFX pivotMotor;
  TalonFX wristMotor;
  CANcoder wristCC = new CANcoder(Constants.PivotArmConstants.wristEncoderId);

  private final MotionMagicVoltage wristMmReq = new MotionMagicVoltage(0).withSlot(0);
  private final MotionMagicVoltage pivotMmReq = new MotionMagicVoltage(0).withSlot(0);

  private final PositionDutyCycle pivotPosReq = new PositionDutyCycle(0);
  private final PositionDutyCycle wristPosReq = new PositionDutyCycle(0);

  public PivotArm() {
    pivotMotor = new TalonFX(Constants.PivotArmConstants.pivotMotorId);
    wristMotor = new TalonFX(Constants.PivotArmConstants.wristMotorId);

    // Wrist Motor Configuration
    TalonFXConfiguration wristCfg = new TalonFXConfiguration();

    CANcoderConfiguration wristCCCfg = new CANcoderConfiguration();
    wristCCCfg.MagnetSensor = new MagnetSensorConfigs()
        .withMagnetOffset(Constants.PivotArmConstants.wristOffset);
    wristCCCfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    wristCC.getConfigurator().apply(wristCCCfg);

    FeedbackConfigs wristFb = new FeedbackConfigs();

    // Set Abs Encoder Position to Internal Encoder Position
    wristMotor.setPosition(wristCC.getAbsolutePosition().getValueAsDouble());

    wristFb.SensorToMechanismRatio = Constants.PivotArmConstants.wristSensorToMechanismRatio;
    // wristFb.RotorToSensorRatio = Constants.PivotArmConstants.wristRotorToSensorRatio;
    wristFb.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // wristFb.FeedbackRemoteSensorID = Constants.PivotArmConstants.wristEncoderId;
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
    wristCfg.MotorOutput.PeakForwardDutyCycle = 0.1;
    wristCfg.MotorOutput.PeakReverseDutyCycle = -0.1;
    wristCfg.Voltage.PeakForwardVoltage = 4;
    wristCfg.Voltage.PeakReverseVoltage = -4;
    wristCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply the whole configuration
    wristMotor.getConfigurator().apply(wristCfg);

    configureMotionMagicForWrist(0.1, 0.1, 0.1);

    // Pivot Motor Configuration
    TalonFXConfiguration pivotCfg = new TalonFXConfiguration();

    FeedbackConfigs pivotFb = new FeedbackConfigs();

    pivotFb.SensorToMechanismRatio = Constants.PivotArmConstants.pivotSensorToMechanismRatio;
    pivotFb.RotorToSensorRatio = Constants.PivotArmConstants.pivotRotorToSensorRatio;
    // wristFb.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // wristFb.FeedbackRemoteSensorID = Constants.PivotArmConstants.wristEncoderId;
    pivotCfg.Feedback = pivotFb;

    pivotCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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

    configureMotionMagicForPivot(0.1, 0.1, 0.1);
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

  public void setWristPosition(double targetRotations) {
    wristMotor.setControl(wristPosReq.withPosition(targetRotations));
  }

  public void setPivotPosition(double targetRotations) {
    pivotMotor.setControl(pivotPosReq.withPosition(targetRotations));
  }

  public Command intake() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        setPivotPosition(Constants.PivotArmConstants.pivotIntakePosition);
      }, this),
      new WaitUntilCommand(this::pivotAtIntake).withTimeout(0.5),
      new InstantCommand(() -> {
        setWristPosition(Constants.PivotArmConstants.wristIntakePosition);
      }, this)
    );
  }

  public Command outtakeLeft() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        setPivotPosition(Constants.PivotArmConstants.pivotOuttakePosition);
      }, this),
      new WaitUntilCommand(this::pivotAtOuttake).withTimeout(0.5),
      new InstantCommand(() -> {
        setWristPosition(Constants.PivotArmConstants.wristOuttakeLeftPosition);
      }, this)
    );
  }

  public Command outtakeRight() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        setPivotPosition(Constants.PivotArmConstants.pivotOuttakePosition);
      }, this),
      new WaitUntilCommand(this::pivotAtOuttake).withTimeout(0.5),
      new InstantCommand(() -> {
        setWristPosition(Constants.PivotArmConstants.wristOuttakeRightPosition);
      }, this)
    );
  }

  public boolean atIntakeSetpoint() {
    double wristError = Math.abs(getWristPosition() - Constants.PivotArmConstants.wristIntakePosition);
    double pivotError = Math.abs(getPivotPosition() - Constants.PivotArmConstants.pivotIntakePosition);
    return wristError < 0.02 && pivotError < 0.02; // tolerance
  }

  public boolean atOuttakeLeftSetpoint() {
    double wristError = Math.abs(getWristPosition() - Constants.PivotArmConstants.wristOuttakeLeftPosition);
    double pivotError = Math.abs(getPivotPosition() - Constants.PivotArmConstants.pivotOuttakePosition);
    return wristError < 0.02 && pivotError < 0.02; // tolerance
  }

  public boolean atOuttakeRightSetpoint() {
    double wristError = Math.abs(getWristPosition() - Constants.PivotArmConstants.wristOuttakeRightPosition);
    double pivotError = Math.abs(getPivotPosition() - Constants.PivotArmConstants.pivotOuttakePosition);
    return wristError < 0.02 && pivotError < 0.02; // tolerance
  }

  public boolean pivotAtIntake() {
    double pivotError = Math.abs(getPivotPosition() - Constants.PivotArmConstants.pivotIntakePosition);
    return pivotError < 0.02; // tolerance
  }

  public boolean pivotAtOuttake() {
    double pivotError = Math.abs(getPivotPosition() - Constants.PivotArmConstants.pivotOuttakePosition);
    return pivotError < 0.02; // tolerance
  }

  public double getWristPosition() {
    return wristMotor.getPosition().getValueAsDouble();
  }

  public double getPivotPosition() {
    return pivotMotor.getPosition().getValueAsDouble();
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

    // System.out.println("Pivot Position");
    // System.out.println(getPivotPosition());
    // System.out.println("Wrist Position");
    // System.out.println(getWristPosition());
    // System.out.println("At Intake");
    // System.out.println(atIntakeSetpoint());
    // System.out.println("At Outake");
    // System.out.println(atOuttakeSetpoint());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
