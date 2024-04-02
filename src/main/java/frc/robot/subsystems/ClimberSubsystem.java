// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ExternalFollower;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Pref;
import frc.robot.Constants.CANIDConstants;
import monologue.Logged;
import monologue.Annotations.Log;


public class ClimberSubsystem extends SubsystemBase implements Logged{
  /** Creates a new Climber. */
  CANSparkMax climberMotorLeft;
  CANSparkMax climberMotorRight;
  Servo climberLock;

  RelativeEncoder climberEncoderLeft;
  RelativeEncoder climberEncoderRight;
  public boolean showClimber = true;

  public ClimberSubsystem() {
    climberMotorLeft = new CANSparkMax(CANIDConstants.climberIDLeft, MotorType.kBrushless);
    climberMotorRight = new CANSparkMax(CANIDConstants.climbreIDRight, MotorType.kBrushless);
    climberEncoderLeft = climberMotorLeft.getEncoder();
    climberEncoderRight = climberMotorRight.getEncoder();

    climberLock = new Servo(0);
    unlockClimber();
    configMotor(climberMotorRight, climberEncoderRight, false);
    configMotor(climberMotorLeft, climberEncoderLeft, true);

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
    //motor.setOpenLoopRampRate(3);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putNumber("Climber RPM Left", getRPMLeft());
    // SmartDashboard.putNumber("Climber RPM Right", getRPMRight());
    //SmartDashboard.putNumber("ClimberLeft", loopctr)

    // SmartDashboard.putNumber("Amps", climberMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Position", climberEncoder.getPosition());

  }

  public void stopMotors() {
    climberMotorLeft.stopMotor();
    climberMotorLeft.setVoltage(0);
    climberMotorRight.stopMotor();
    climberMotorRight.setVoltage(0);
  }

  public Command stopClimberCommand() {
    return Commands.runOnce(() -> stopMotors(), this);
  }

  public void runClimberMotor(double speed) {
    if (getPositionLeft() > 130) {
      speed = speed*0.5;
    }
    climberMotorLeft.setVoltage(speed * RobotController.getBatteryVoltage());
    climberMotorRight.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public void lowerClimber(double speed) {
    // if (climberEncoder.getPosition() > 60) {
    //   runClimberMotor(speed);
    // } else {
    //   runClimberMotor(speed * .5);
    // }
    if (getPositionLeft() < 10) {
      runClimberMotor(speed * 0.2);
    } else {
        runClimberMotor(speed);
    }
  }

  public Command lowerClimberArmsCommand(double speed) {
    return Commands.run(() -> lowerClimber(-speed));
  }

  public Command raiseClimberArmsCommand(double speed) {
    return Commands.run(() -> runClimberMotor(speed));
  }

  @Log.NT(key = "ClimberLeftRPM")
  public double getRPMLeft() {
    return climberEncoderLeft.getVelocity();
  }

  @Log.NT(key = "ClimberRightRPM")
  public double getRPMRight() {
    return climberEncoderRight.getVelocity();
  }

  @Log.NT(key = "ClimberPositionLeft")
  public double getPositionLeft() {
    return climberEncoderLeft.getPosition();
  }

  @Log.NT(key = "ClimberPositionRight")
  public double getPositionRight() {
    return climberEncoderRight.getPosition();
  }

  public Command clearFaultsLeftCommand() {
    return Commands.runOnce(() -> climberMotorLeft.clearFaults());
  }

   public Command clearFaultsRightCommand() {
    return Commands.runOnce(() -> climberMotorRight.clearFaults());
  }

  public int getFaultsLeft() {
    return climberMotorLeft.getFaults();
  }

   public int getFaultsRight() {
    return climberMotorRight.getFaults();
  }

  public int getStickyFaultsLeft() {
    return climberMotorLeft.getStickyFaults();
  }

    public int getStickyFaultsRight() {
    return climberMotorRight.getStickyFaults();
  }

  @Log.NT(key = "ClimberLeftAmps")
  public double getLeftAmps() {
    return climberMotorLeft.getOutputCurrent();
  }

  @Log.NT(key = "ClimberRightAmps")
  public double getRightAmps() {
    return climberMotorRight.getOutputCurrent();
  }

  public void lockClimber() {
    climberLock.set(Pref.getPref("LockNumber"));
  }

  public Command lockClimberCommand() {
    return Commands.runOnce(()->lockClimber());
  }

  public void unlockClimber() {
    climberLock.set(Pref.getPref("UnlockNumber"));
  }

  public Command unlockClimberCommand() {
    return Commands.runOnce(()->unlockClimber());
  }

  public Command clearFaultsCommand() {
    return new SequentialCommandGroup(clearFaultsLeftCommand(), clearFaultsRightCommand());
  }
}
