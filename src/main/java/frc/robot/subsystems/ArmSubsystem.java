package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Pref;
import static edu.wpi.first.units.Units.Volts;

public class ArmSubsystem extends ProfiledPIDSubsystem {

    public final CANSparkMax armMotor = new CANSparkMax(CANIDConstants.armID, MotorType.kBrushless);

    public final CANcoder armCancoder;

    private final RelativeEncoder armEncoder;

    public ArmFeedforward armfeedforward;

    public boolean armMotorConnected;

    public double appliedOutput;

    private boolean useSoftwareLimit;

    public boolean inIZone;

    private boolean m_showScreens;

    public double armVolts;

    private double feedforward;
    private int ctr;

    private double acceleration;

    private double lastTime;

    private double lastSpeed;

    private double lastPosition;

    public double appliedVolts;

    public double armAngleRads;

    private PIDController pid = new PIDController(.1, 0.0, 0);

    private double pidout;

    public double angleTolerance = ArmConstants.angleTolerance;

    public boolean enableArm;

    private double activeKv;

    private double lastGoal;

    public ArmSubsystem(boolean showScreens) {
        super(
                new ProfiledPIDController(
                        5,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                ArmConstants.kTrapVelocityRadPerSecond,
                                ArmConstants.kTrapAccelerationRadPerSecSquared)),
                0);

        m_showScreens = showScreens;
        useSoftwareLimit = false;

        // armMotor = new CANSparkMax(CANIDConstants.armID, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
        armCancoder = new CANcoder(CANIDConstants.armCancoderID, "CV1");

        configMotor(armMotor, armEncoder, false);

        setSoftwareLimits();

        enableSoftLimits(useSoftwareLimit);

        armfeedforward = new ArmFeedforward(ArmConstants.armKs, ArmConstants.armKg, ArmConstants.armKv,
                ArmConstants.armKa);

        if (m_showScreens) {

            Shuffleboard.getTab("ShooterSubsystem").add(this)
                    .withPosition(3, 0).withSize(2, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .addBoolean("OnTravelLimit", () -> onLimit())
                    .withPosition(5, 0).withSize(1, 1)
                    .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "black"));

            Shuffleboard.getTab("ShooterSubsystem")
                    .add("SetAllPID", setPIDGainsCommand())
                    .withPosition(6, 0).withSize(1, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .addNumber("CurrentAngleDeg", () -> round2dp(Units.radiansToDegrees(getAngleRadians())))
                    .withPosition(4, 1).withSize(1, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .addNumber("CanCoderDeg", () -> round2dp(getCanCoderDeg()))
                    .withPosition(5, 1).withSize(1, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .addNumber("Setpoint",
                            () -> round2dp(Units.radiansToDegrees(getController().getSetpoint().position)))
                    .withPosition(7, 3).withSize(1, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .addNumber("VelRadPerSec", () -> round2dp(getRadsPerSec()))
                    .withPosition(3, 1).withSize(1, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .addNumber("VperMPS", () -> round2dp(getVoltsPerRadPerSec()))
                    .withPosition(6, 2).withSize(1, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .addNumber("Goal", () -> round2dp(Units.radiansToDegrees(getController().getGoal().position)))
                    .withPosition(6, 1).withSize(1, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .addBoolean("StickyFault", () -> getStickyFaults() != 0)
                    .withPosition(3, 2).withSize(1, 1)
                    .withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "black"));

            Shuffleboard.getTab("ShooterSubsystem")
                    .add("ClearFault", clearFaultsCommand())
                    .withPosition(4, 2).withSize(1, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .addNumber("Amps", () -> armMotor.getOutputCurrent())
                    .withPosition(5, 0).withSize(1, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .add("Arm to 15 Deg", setGoalCommand(Units.degreesToRadians(15)))
                    .withPosition(3, 3).withSize(1, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .add("Arm to 25 Deg", setGoalCommand(Units.degreesToRadians(25)))
                    .withPosition(4, 3).withSize(1, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .add("Arm to 45 Deg", setGoalCommand(Units.degreesToRadians(45)))
                    .withPosition(5, 3).withSize(1, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .add("Arm to 60 Deg", setGoalCommand(Units.degreesToRadians(60)))
                    .withPosition(6, 3).withSize(1, 1);

            Shuffleboard.getTab("ShooterSubsystem")
                    .addBoolean("PIDEnabled", () -> isEnabled() && enableArm)
                    .withPosition(5, 2).withSize(1, 1);

            armEncoder.setPosition(Units.degreesToRadians(15));

        }
        setGoal(Units.degreesToRadians(18)); //15
        // pid.setIZone(Units.degreesToRadians(.5));
        // pid.setIntegratorRange(-Units.degreesToRadians(.1),
        // Units.degreesToRadians(.1));
        pid.reset();
        setKp();
    }

    private void configMotor(CANSparkMax motor, RelativeEncoder encoder, boolean reverse) {
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(ArmConstants.armContinuousCurrentLimit);
        motor.setInverted(reverse);
        motor.setIdleMode(ArmConstants.armIdleMode);
        encoder.setVelocityConversionFactor(ArmConstants.armConversionVelocityFactor);
        encoder.setPositionConversionFactor(ArmConstants.armConversionPositionFactor);
        motor.enableVoltageCompensation(ArmConstants.voltageComp);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kPositionOnly);
        motor.burnFlash();
    }

    public void periodicRobot() {

        armAngleRads = getAngleRadians();
        if (!enableArm) {
            setGoal(armAngleRads);
        }

    }

    @Override
    protected void useOutput(double output, State goalState) {

        pidout = pid.calculate(armAngleRads, getController().getSetpoint().position);

        // SmartDashboard.putNumber("CTR", ctr++);
        // SmartDashboard.putNumber("PID", pidout);
        // SmartDashboard.putNumber("ff", feedforward);
        // SmartDashboard.putNumber("poserr", pid.getPositionError());
        // SmartDashboard.putNumber("velerr", getController().getVelocityError());

        double tempv = getController().getSetpoint().velocity;
        double tempp = getController().getSetpoint().position;

        // SmartDashboard.putNumber("TrapVel", tempv);
        // SmartDashboard.putNumber("TrapPos", tempp);
        // SmartDashboard.putNumber("MOTROAO", armMotor.getAppliedOutput());

        // SmartDashboard.putNumber("ARMANFR", armAngleRads);
        // SmartDashboard.putNumber("ARMANDG", Units.radiansToDegrees(armAngleRads));

        acceleration = (getController().getSetpoint().velocity - lastSpeed)
                / (Timer.getFPGATimestamp() - lastTime);

        // feedforward =
        // armfeedforward.calculate(getController().getSetpoint().position,
        // getController().getSetpoint().velocity,
        // acceleration);

        // SmartDashboard.putNumber("KGVAL", Pref.getPref("armFFKg")
        //         * Math.cos(getController().getSetpoint().position));
        // SmartDashboard.putNumber("KVVAL", Pref.getPref("armFFKv")
        //         * getController().getSetpoint().velocity);
        // SmartDashboard.putNumber("KSVAL", Pref.getPref("armFFKs") *
        //         Math.signum(getController().getSetpoint().velocity));
        // SmartDashboard.putNumber("KAVAL", Pref.getPref("armFFKa") * acceleration);

        feedforward = Pref.getPref("armFFKs") *
                Math.signum(getController().getSetpoint().velocity)
                + Pref.getPref("armFFKg") * Math.cos(getController().getSetpoint().position)
                + Pref.getPref("armFFKv") * getController().getSetpoint().velocity // this was commented out for some
                                                                                   // reason?
                + activeKv * getController().getSetpoint().velocity
                + Pref.getPref("armFFKa") * acceleration;
        // Add the feedforward to the PID output to get the motor output

        lastSpeed = getController().getSetpoint().velocity;
        lastPosition = getController().getSetpoint().position;

        lastTime = Timer.getFPGATimestamp();

        double out = pidout + feedforward;

        armMotor.setVoltage(out);
    }

    @Override
    protected double getMeasurement() {
        return armAngleRads;
    }

    public void trackDistance(double meters) {
        setTolerance(ArmConstants.angleTolerance);
        double angle = Constants.armAngleMap.get(meters);
        setGoal(angle);
    }

    public void resetController() {
        getController().reset(getAngleRadians());
    }

    public void setTolerance(double tolerance) {
        angleTolerance = tolerance;
    }

    public Command setGoalCommand(double angleRads) {
        return Commands.sequence(
                Commands.runOnce(() -> setTolerance(ArmConstants.angleTolerance)),
                Commands.runOnce(() -> resetController()),
                Commands.runOnce(() -> setGoal(angleRads), this),
                Commands.runOnce(() -> enable(), this));
    }

    public Command setGoalCommand(double angleRads, double tolerance) {
        return Commands.sequence(
                Commands.runOnce(() -> angleTolerance = Units.degreesToRadians(tolerance)),
                Commands.runOnce(() -> getController().reset(getAngleRadians()), this),
                Commands.run(() -> setUpDownKv(angleRads)),
                Commands.runOnce(() -> setGoal(angleRads), this),
                Commands.runOnce(() -> enable(), this));
    }

    public Command positionToIntakeUDACommand() {
        return Commands.sequence(
                setGoalCommand(Units.degreesToRadians(20)),
                new WaitCommand(1),
                setGoalCommand(ArmConstants.pickupAngle));
    }

    public void setUpDownKv(double rads) {
        activeKv = Pref.getPref("armFFKv");
        if (rads > lastGoal)
            activeKv = Pref.getPref("armUpFFKv");
        lastGoal = rads;
    }

    public void incrementArmAngle(double valdeg) {
        double temp = getCurrentGoal();
        temp += Units.degreesToRadians(valdeg);
        if (temp > ArmConstants.armMaxRadians)
            temp = ArmConstants.armMaxRadians;
        setGoal(temp);
    }

    public void decrementArmAngle(double valdeg) {
        double temp = getCurrentGoal();
        temp -= Units.degreesToRadians(valdeg);
        if (temp < ArmConstants.armMinRadians)
            temp = ArmConstants.armMinRadians;
        setGoal(temp);
    }

    public double getCurrentGoal() {
        return getController().getGoal().position;
    }

    public double getAngleRadians() {
        return getCanCoderRad();
        // if (RobotBase.isReal())
        // return armEncoder.getPosition();
        // else
        // return getCurrentGoal();
    }

    public double getAngleDegrees() {
        return getCanCoderDeg();
    }

    public boolean atSetpoint() {
        return Math.abs(getCurrentGoal() - getAngleRadians()) < angleTolerance;
        // return getController().atSetpoint() || RobotBase.isSimulation();
    }

    public double getVoltsPerRadPerSec() {
        appliedVolts = armMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        double temp = appliedVolts / getRadsPerSec();
        if (temp < 1 || temp > 3)
            temp = 0;
        return temp;
    }

    public double getRadsPerSec() {
        return armEncoder.getVelocity();
    }

    public double getCanCoderRadsPerSec() {
        return Math.PI * armCancoder.getVelocity().getValueAsDouble();
    }

    public double getDegreesPerSec() {
        return Units.radiansToDegrees(armEncoder.getVelocity());
    }

    public boolean onPlusSoftwareLimit() {
        return armMotor.getFault(FaultID.kSoftLimitFwd);
    }

    public boolean onMinusSoftwareLimit() {
        return armMotor.getFault(FaultID.kSoftLimitRev);
    }

    public boolean onPlusHardwareLimit() {
        return armMotor.getFault(FaultID.kHardLimitRev);
    }

    public boolean onMinusHardwareLimit() {
        return armMotor.getFault(FaultID.kHardLimitRev);
    }

    public boolean onLimit() {
        return onPlusHardwareLimit() || onMinusHardwareLimit() || onPlusSoftwareLimit() || onMinusSoftwareLimit();
    }

    public void stop() {
        armMotor.setVoltage(0);
    }

    public double getAmps() {
        return armMotor.getOutputCurrent();
    }

    public void setSoftwareLimits() {
        armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) ArmConstants.armMinRadians);
        armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) ArmConstants.armMaxRadians);
        armMotor.setIdleMode(IdleMode.kBrake);
    }

    public void enableSoftLimits(boolean on) {
        armMotor.enableSoftLimit(SoftLimitDirection.kForward, on);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, on);
    }

    public boolean isBraked() {
        return armMotor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getSoftwareLimitsEnabled() {
        return armMotor.isSoftLimitEnabled(SoftLimitDirection.kForward)
                || armMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
    }

    public Command clearFaultsCommand() {
        return Commands.runOnce(() -> armMotor.clearFaults());
    }

    public int getFaults() {
        return armMotor.getFaults();
    }

    public int getStickyFaults() {
        return armMotor.getStickyFaults();
    }

    public double round2dp(double number) {
        number = Math.round(number * 100);
        number /= 100;
        return number;
    }

    public double getCanCoderDeg() {
        return Units.radiansToDegrees(getCanCoderRad());
    }

    public double getCanCoderRad() {
        double temp = (armCancoder.getAbsolutePosition().getValueAsDouble()
                * Math.PI) + ArmConstants.cancoderOffsetRadians;
        if (temp > Math.PI)
            temp = temp - Math.PI;
            return temp;
    }

    public double getCanCoderRadPerSec() {
        return armCancoder.getVelocity().getValueAsDouble() * Math.PI;
    }

    public void setKp() {
        pid.setP(Pref.getPref("armKp"));
        // getController().setP(Pref.getPref("armKp"));
        SmartDashboard.putNumber("PPP", pid.getP());
    }

    public void setKd() {
        // getController().setP(Pref.getPref("armKd"));
        pid.setD(Pref.getPref("armKd"));
    }

    public void setKi() {
        // getController().setP(Pref.getPref("armKi"));
        pid.setI(Pref.getPref("armKi"));
    }

    public Command setPIDGainsCommand() {
        return Commands.runOnce(() -> setKPKIKD());
    }

    public void setKPKIKD() {
        setKp();
        setKi();
        setKd();
    }

    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (volts) -> {
                        armMotor.setVoltage(volts.in(Volts));
                    },
                    null,
                    this));

    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(Direction.kForward);
    }

    public Command quasistaticBackward() {
        return sysIdRoutine.quasistatic(Direction.kReverse);
    }

    public Command dynamicForward() {
        return sysIdRoutine.dynamic(Direction.kForward);
    }

    public Command dynamicBackward() {
        return sysIdRoutine.dynamic(Direction.kReverse);
    }

}
