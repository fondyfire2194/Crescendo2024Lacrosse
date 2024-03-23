// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;


public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax climberMotor;
  RelativeEncoder climberEncoder;
  public boolean showClimber = true;

  private int loopctr;


  public ClimberSubsystem() {
    climberMotor = new CANSparkMax(CANIDConstants.climberID, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();;
    configMotor(climberMotor, climberEncoder, false);
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

    SmartDashboard.putNumber("Climber RPM", getRPM());
    SmartDashboard.putNumber("Amps", climberMotor.getOutputCurrent());
    SmartDashboard.putNumber("Position", climberEncoder.getPosition());

  }

  public void stopMotor() {
    climberMotor.stopMotor();
    climberMotor.setVoltage(0);
  
  }

  public Command stopClimberCommand() {
    return Commands.runOnce(() -> stopMotor(), this);
  }

  public void runClimberMotor(double speed) {
    climberMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public void lowerClimber(double speed) {
    if (climberEncoder.getPosition() > 60) {
      runClimberMotor(speed);
    } else {
      runClimberMotor(speed * .5);
    }
  }

  public Command lowerClimberArmsCommand(double speed) {
    return Commands.run(() -> lowerClimber(-speed));
  }

  public Command raiseClimberArmsCommand(double speed) {
    return Commands.run(() -> runClimberMotor(speed));
  }

  public double getRPM() {
    return climberEncoder.getVelocity();
  }

  public Command clearFaultsCommand() {
    return Commands.runOnce(() -> climberMotor.clearFaults());
  }

  public int getFaults() {
    return climberMotor.getFaults();
  }

  public int getStickyFaults() {
    return climberMotor.getStickyFaults();
  }

}
