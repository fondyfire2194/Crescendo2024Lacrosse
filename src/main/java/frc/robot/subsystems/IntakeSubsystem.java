// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.NotSerializableException;
import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.SpikeFilter;
import frc.robot.Pref;

public class IntakeSubsystem extends SubsystemBase {

  public CANSparkMax intakeMotor;
  RelativeEncoder intakeEncoder;
  public SparkPIDController intakeController;
  public int intakeFaultSeen;
  public int intakeStickyFaultSeen;

  private int loopctr;
  private boolean m_showScreens;
  private boolean runIntake;
  public boolean jogging;
  private double intakeAmpsHighStartTime;
  private double intakeAmpsHighStopTime;
  private LinearFilter filter;
  private double filteredAmps;
  private boolean noteStuck;
  private boolean noteClear;

  /** Creates a new Intake. */
  public IntakeSubsystem(boolean showScreens) {
    m_showScreens = showScreens;
    intakeMotor = new CANSparkMax(Constants.CANIDConstants.intakeID, MotorType.kBrushless);
    intakeController = intakeMotor.getPIDController();
    intakeEncoder = intakeMotor.getEncoder();
    configMotor(intakeMotor, intakeEncoder, false);

    filter = LinearFilter.movingAverage(3);

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
    intakeMotor.setClosedLoopRampRate(1);
    intakeMotor.setOpenLoopRampRate(1);

    intakeAmpsHighStartTime = 0;
    intakeAmpsHighStopTime = 0;

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

    if (loopctr > 50) {
      intakeStickyFaultSeen = getStickyFaults();
      intakeFaultSeen = getFaults();
      loopctr = 0;
    }

    if (runIntake && !noteStuck)
      intakeController.setReference(Pref.getPref("IntakeSpeed"), ControlType.kVelocity);
    if (!runIntake && !jogging)
      stopMotor();

    if (runIntake) {
      filteredAmps = filter.calculate(intakeMotor.getOutputCurrent());
    } else
      filteredAmps = 0;

    noteStuck = false;//runIntake && checkNoteStuck() && !noteClear;

    if (runIntake && noteStuck)
      intakeController.setReference(-500, ControlType.kVelocity);

    // noteClear = noteStuck && checkNoteClear();

    // if (noteClear) {
    //   intakeAmpsHighStartTime = 0;
    //   intakeAmpsHighStopTime = 0;
    // }

  }

  public double getAmps() {
    return filteredAmps;
  }

  private boolean checkNoteStuck() {

    if (intakeAmpsHighStartTime == 0 && getAmps() > IntakeConstants.stuckNoteAmps) {
      intakeAmpsHighStartTime = Timer.getFPGATimestamp();
    }

    if (intakeAmpsHighStartTime != 0 && getAmps() < IntakeConstants.stuckNoteAmps)
      intakeAmpsHighStartTime = 0;

    return (intakeAmpsHighStartTime != 0
        && Timer.getFPGATimestamp() > intakeAmpsHighStartTime + IntakeConstants.anpsHighTimeLimit);
  }

  // private boolean checkNoteClear() {
  //   if (getAmps() < IntakeConstants.freeNoteAmps)
  //     intakeAmpsHighStopTime = Timer.getFPGATimestamp();

  //   return intakeAmpsHighStopTime != 0
  //       && Timer.getFPGATimestamp() > intakeAmpsHighStopTime + IntakeConstants.anpsHighTimeLimit;
  // }

 
  public void setPID() {
    intakeController.setP(IntakeConstants.intakeKp);
    intakeController.setFF(IntakeConstants.intakeKFF);
  }

  public Command clearFaultsCommand() {
    intakeFaultSeen = 0;
    intakeStickyFaultSeen = 0;
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
