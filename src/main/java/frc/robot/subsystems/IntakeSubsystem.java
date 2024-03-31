// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;


import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Pref;

public class IntakeSubsystem extends SubsystemBase {

  public CANSparkMax intakeMotor;
  RelativeEncoder intakeEncoder;
  public SparkPIDController intakeController;

  private int loopctr;
  private boolean m_showScreens;
  private boolean runIntake;
  public boolean jogging;
  private SlewRateLimiter intakeLimiter = new SlewRateLimiter(1500);
 

  /** Creates a new Intake. */
  public IntakeSubsystem(boolean showScreens) {
    m_showScreens = showScreens;
    intakeMotor = new CANSparkMax(Constants.CANIDConstants.intakeID, MotorType.kBrushless);
    intakeController = intakeMotor.getPIDController();
    intakeEncoder = intakeMotor.getEncoder();
    configMotor(intakeMotor, intakeEncoder, false);

    if (m_showScreens) {

      Shuffleboard.getTab("IntakeSubsystem").add(this)
          .withSize(2, 1)
          .withPosition(0, 0);

      Shuffleboard.getTab("IntakeSubsystem").addNumber("IntakeRPM",
          () -> round2dp(getRPM(), 0))
          .withSize(1, 1)
          .withPosition(0, 1);

      Shuffleboard.getTab("IntakeSubsystem").addNumber("IntakeAmps",
          () -> round2dp(getAmps(), 1))
          .withSize(1, 1)
          .withPosition(1, 1);

      Shuffleboard.getTab("IntakeSubsystem").addNumber("IntakeVolts",
          () -> intakeMotor.getAppliedOutput())
          .withSize(1, 1)
          .withPosition(1, 1);

      Shuffleboard.getTab("IntakeSubsystem")
          .addBoolean("StickyFault", () -> getStickyFaults() != 0)
          .withPosition(0, 2).withSize(1, 1)
          .withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "black"));

      Shuffleboard.getTab("IntakeSubsystem")
          .add("ClearFault", clearFaultsCommand())
          .withPosition(1, 2).withSize(1, 1);
    }

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(Constants.IntakeConstants.intakeContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.IntakeConstants.intakeIdleMode);
    encoder.setVelocityConversionFactor(Constants.IntakeConstants.intakeConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.IntakeConstants.intakeConversionPositionFactor);
    motor.enableVoltageCompensation(Constants.IntakeConstants.voltageComp);
    // intakeMotor.setClosedLoopRampRate(1);
    // intakeMotor.setOpenLoopRampRate(1);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  public void stopMotor() {
    intakeMotor.stopMotor();
    intakeController.setReference(0, ControlType.kVelocity);
    resetRunIntake();
  }

  public Command stopIntakeCommand() {
    return Commands.runOnce(() -> stopMotor(), this);
  }

  public Command startIntakeCommand() {
    return Commands.runOnce(() -> setRunIntake(), this);
  }

  public void setRunIntake() {
    runIntake = true;
  }

  public void resetRunIntake() {
    runIntake = false;
  }

  public boolean getRunIntake() {
    return runIntake;
  }

  public double getRPM() {
    return intakeEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    loopctr++;

    if (runIntake) {
      //double rpm = intakeLimiter.calculate(Pref.getPref("IntakeSpeed"));
      runAtVelocity(Pref.getPref("IntakeSpeed"));
    }
    if (!runIntake && !jogging) {
      stopMotor();
      //intakeLimiter.reset(0);
    }
  }

  private void runAtVelocity(double rpm) {
    intakeController.setReference(rpm, ControlType.kVelocity);
  }

  public void reverseMotor() {
    runAtVelocity(IntakeConstants.reverseRPM);
  }

  public double getAmps() {
    return intakeMotor.getOutputCurrent();
  }

  public void setPID() {
    intakeController.setP(Pref.getPref("IntakeKp"));
    intakeController.setFF(IntakeConstants.intakeKFF);
      }

  public Command clearFaultsCommand() {
    return Commands.runOnce(() -> intakeMotor.clearFaults());
  }

  public int getFaults() {
    return intakeMotor.getFaults();
  }

  public int getStickyFaults() {
    return intakeMotor.getStickyFaults();
  }

  public String getFirmwareVersion() {
    return intakeMotor.getFirmwareString();
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

}
