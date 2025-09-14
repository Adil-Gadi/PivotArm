// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */

    final TalonFX clawMotor;

    DutyCycleOut clawMotorRequest;

    public Claw() {
        clawMotor = new TalonFX(Constants.ClawConstants.clawMotorId);

        clawMotorRequest = new DutyCycleOut(0.0);

    }

    // Speed from 0 to 1 where 1 is full speed
    public void setSpeed(double speed) {
        clawMotor.setControl(clawMotorRequest.withOutput(speed));
    }

    public void stop() {
        clawMotor.setControl(clawMotorRequest.withOutput(0.0));
    }

    public void intake() {
        setSpeed(0.5);
        setDirection(true);
    }

    public void fastIntake() {
        setSpeed(0.75);
        setDirection(true);
    }

    public void outtake() {
        setSpeed(0.5);
        setDirection(false);
    }

    public void setDirection(boolean clockwise) {

        InvertedValue direction = clockwise ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        clawMotor.getConfigurator().apply(
                new TalonFXConfiguration() {
                    {
                        MotorOutput.Inverted = direction;
                    }
                });
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    // enum ClawSpeed {
    // intake
    // }

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
