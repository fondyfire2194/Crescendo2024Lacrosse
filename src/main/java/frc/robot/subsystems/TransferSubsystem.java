// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.TransferConstants;

import frc.robot.Pref;

public class TransferSubsystem extends SubsystemBase {

  public CANSparkMax transferMotor;
  public SparkPIDController transferController;
  RelativeEncoder transferEncoder;

  private final TimeOfFlight m_detectNoteSensor = new TimeOfFlight(CANIDConstants.transferDistanceSensorID);

  public boolean m_showScreens;

  private int transferFaultSeen;
  private int transferStickyFaultSeen;

  private Debouncer noteSensorDebouncer;

  private int loopctr;
  public double sensedPosition;
  public SparkLimitSwitch m_limitSwitch;

  /** Creates a new transfer. */
  public TransferSubsystem(boolean showScreens) {
    m_showScreens = showScreens;
    transferMotor = new CANSparkMax(CANIDConstants.transferID, MotorType.kBrushless);
    transferEncoder = transferMotor.getEncoder();
    transferController = transferMotor.getPIDController();

    configMotor(transferMotor, transferEncoder, true);
    noteSensorDebouncer = new Debouncer(.25);

     m_limitSwitch = transferMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
     m_limitSwitch.enableLimitSwitch(true);


    m_detectNoteSensor.setRangingMode(RangingMode.Short, 40);

    if (m_showScreens) {

      Shuffleboard.getTab("IntakeSubsystem").add(this)
          .withSize(2, 1)
          .withPosition(3, 0);

      Shuffleboard.getTab("IntakeSubsystem").addNumber("TransferRPM",
          () -> round2dp(getRPM(), 0))
          .withSize(1, 1)
          .withPosition(3, 1);

      Shuffleboard.getTab("IntakeSubsystem").addNumber("TransferAmps",
          () -> round2dp(getAmps(), 1))
          .withSize(1, 1)
          .withPosition(4, 1);

      Shuffleboard.getTab("IntakeSubsystem").addNumber("NoteSensorInches",
          () -> round2dp(getSensorDistanceInches(), 1))
          .withSize(1, 1)
          .withPosition(3, 2);

      Shuffleboard.getTab("IntakeSubsystem").addBoolean("NoteSensed", () -> noteAtIntake())
          .withSize(1, 1)
          .withPosition(4, 2)
          .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

      Shuffleboard.getTab("IntakeSubsystem")
          .addBoolean("StickyFaultTfr", () -> getStickyFaults() != 0)
          .withPosition(3, 3).withSize(1, 1)
          .withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "black"));

      Shuffleboard.getTab("IntakeSubsystem")
          .add("ClearFaultTfr", clearFaultsCommand())
          .withPosition(4, 3).withSize(1, 1);

    }

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder,
      boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(TransferConstants.transferContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(TransferConstants.transferIdleMode);
    // encoder.setVelocityConversionFactor(TransferConstants.transferConversionVelocityFactor);
    // encoder.setPositionConversionFactor(TransferConstants.transferConversionPositionFactor);
    motor.enableVoltageCompensation(TransferConstants.voltageComp);
    motor.setOpenLoopRampRate(1);
    motor.setClosedLoopRampRate(1);
    motor.burnFlash();
    encoder.setPosition(0.0);

  }

  public void stopMotor() {
    transferController.setReference(0, ControlType.kVelocity);
    transferMotor.stopMotor();

  }

  public Command stopTransferCommand() {
    return Commands.runOnce(() -> stopMotor(), this);
  }

  public Command transferToShooterCommand() {
    return Commands
        .run(() -> transferToShooter(), this)
        .until(() -> noteSensorDebouncer.calculate(!noteAtIntake()))
        .withTimeout(TransferConstants.clearShooterTime)
        .andThen(stopTransferCommand());
  }

  public void transferToShooter() {
    enableLimitSwitch(false);
    transferController.setReference(Pref.getPref("TransferToShootSpeed"), ControlType.kVelocity);
  }

  public void runToSensor() {
    enableLimitSwitch(true);
    transferController.setReference(Pref.getPref("TransferIntakingSpeed"), ControlType.kVelocity);
  }

  public double getSensorDistanceInches() {
    return Units.metersToInches(m_detectNoteSensor.getRange() / 1000);
  }

  public boolean noteAtIntake() {
    return getSensorDistanceInches() > 0
        && getSensorDistanceInches() < Pref.getPref("SensorDistance");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("SENSINCHES", getSensorDistanceInches());
    loopctr++;
    if (loopctr > 50) {
      transferStickyFaultSeen = getStickyFaults();
      transferFaultSeen = getFaults();
      loopctr = 0;
    }

     SmartDashboard.putBoolean("LSE", m_limitSwitch.isLimitSwitchEnabled());

    SmartDashboard.putBoolean("PLLIM", m_limitSwitch.isPressed());;


  }

  public void enableLimitSwitch(boolean enable) {
    m_limitSwitch.enableLimitSwitch(enable);
  }

  public boolean getLimitSwitchEnabled() {
    return m_limitSwitch.isLimitSwitchEnabled();
  }

  public double getAmps() {
    return transferMotor.getOutputCurrent();
  }

  public double getRPM() {
    return transferEncoder.getVelocity();
  }

  public void setVelPID() {
    REVLibError ok = transferController.setFF(TransferConstants.transferKFF, 0);
    REVLibError okp = transferController.setP(TransferConstants.transferKp, 0);
  }

  public void setPosPID() {
    transferController.setP(TransferConstants.transferPositionKp, 1);
  }

  public void positionAfterSensor() {
    transferController.setReference(sensedPosition, ControlType.kPosition, 1);
  }

  public Command positionAfterSensorCommand() {
    return Commands.runOnce(() -> positionAfterSensorCommand());
  }

  public Command setTransferPIDCommand() {
    return Commands.runOnce(() -> setVelPID())
        .alongWith(
            Commands.runOnce(() -> setPosPID()));

  }

   public boolean onPlusHardwareLimit() {
        return transferMotor.getFault(FaultID.kHardLimitRev);
    }

    public boolean onMinusHardwareLimit() {
        return transferMotor.getFault(FaultID.kHardLimitRev);
    }

  public int getFaults() {
    return transferMotor.getFaults();
  }

  public int getStickyFaults() {
    return transferMotor.getStickyFaults();
  }

  public void clearFaults() {
    transferMotor.clearFaults();
  }

  public Command clearFaultsCommand() {
    transferFaultSeen = 0;
    transferStickyFaultSeen = 0;
    return Commands.runOnce(() -> transferMotor.clearFaults());
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

  public double getPosition() {
    return transferEncoder.getPosition();
  }

  public double getVelocity() {
    return transferEncoder.getVelocity();
  }

  public boolean isStopped() {
    return Math.abs(getVelocity()) < Pref.getPref("TransferIntakingSpeed") / 20;
  }

}
