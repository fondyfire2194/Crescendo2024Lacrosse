// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Pref;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax climberMotor;
  RelativeEncoder climberEncoder;
  SparkPIDController climberController;
  double simRPM = 0;
  public boolean showClimber = true;
  private int climberFaultSeen;
  private int climberStickyFaultSeen;
  private int loopctr;
  private double holdPosition;

  public ClimberSubsystem() {
    climberMotor = new CANSparkMax(CANIDConstants.climberID, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();
    climberController = climberMotor.getPIDController();
    climberController.setP(.2);
    configMotor(climberMotor, climberEncoder, true);

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kMinimal);
    motor.setSmartCurrentLimit(Constants.ClimberConstants.climberContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.ClimberConstants.climberIdleMode);
    encoder.setVelocityConversionFactor(Constants.ClimberConstants.climberConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.ClimberConstants.climberConversionPositionFactor);
    motor.enableVoltageCompensation(Constants.ClimberConstants.voltageComp);
    motor.setOpenLoopRampRate(3);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (loopctr > 50) {
      climberStickyFaultSeen = getStickyFaults();
      climberFaultSeen = getFaults();
      loopctr = 0;
    }

    SmartDashboard.putNumber("Climber RPM", climberEncoder.getVelocity());
    SmartDashboard.putNumber("Amps", climberMotor.getOutputCurrent());
    SmartDashboard.putNumber("Position", climberEncoder.getPosition());

  }

  public void stopMotor() {
    climberMotor.stopMotor();
    climberMotor.setVoltage(0);
    simRPM = 0;
  }

  public void runClimberMotor(double speed) {
    climberMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public void runLowerClimberMotor(double speed) {
    if (climberEncoder.getPosition() < -60) {
      climberMotor.setVoltage(speed * RobotController.getBatteryVoltage());
    } else {
      climberMotor.setVoltage(speed * 0.5 * RobotController.getBatteryVoltage());
    }
  }

  public Command stopClimberCommand() {
    return Commands.runOnce(() -> stopMotor(), this);
  }

  public Command raiseClimberArmsCommand(double speed) {
    return Commands.run(() -> runClimberMotor(-speed));
  }

  public Command lowerClimberArmsCommand(double speed) {
    return Commands.run(() -> runLowerClimberMotor(speed));
  }

  public Command lockClimberArm() {
    return Commands.run((() -> lockArms()));
  }

private void lockArms(){
  climberController.setReference(holdPosition, ControlType.kPosition);
}

  public double getRPM() {
    return climberEncoder.getVelocity();
  }

  public Command clearFaultsCommand() {
    climberFaultSeen = 0;
    climberStickyFaultSeen = 0;
    return Commands.runOnce(() -> climberMotor.clearFaults());
  }

  public int getFaults() {
    return climberMotor.getFaults();
  }

  public int getStickyFaults() {
    return climberMotor.getStickyFaults();
  }

}
