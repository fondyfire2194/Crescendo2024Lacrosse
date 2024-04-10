package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Pref;
import frc.robot.utils.AllianceUtil;
import monologue.Annotations.Log;
import monologue.LogLevel;
import monologue.Logged;

public class SwerveSubsystem extends SubsystemBase implements Logged {
  // The gyro sensor

  private final AHRS gyro;

  private SwerveDrivePoseEstimator swervePoseEstimator;

  private SwerveModule[] mSwerveMods;

  private Field2d field;

  private Pose2d simOdometryPose = new Pose2d();

  private boolean lookForNote;

  private double keepAngle = 0.0;
  private double timeSinceRot = 0.0;
  private double lastRotTime = 0.0;
  private double timeSinceDrive = 0.0;
  private double lastDriveTime = 0.0;

  private static final Matrix<N3, N1> ODOMETRY_STDDEV = VecBuilder.fill(0.03, 0.03, Math.toRadians(1));
  private static final Matrix<N3, N1> VISION_STDDEV = VecBuilder.fill(0.5, 0.5, Math.toRadians(40));

  private final PIDController m_keepAnglePID =

      new PIDController(Constants.KeepAngle.kp, Constants.KeepAngle.ki, Constants.KeepAngle.kd);

  public PIDController m_alignPID = new PIDController(SwerveConstants.alignKp, 0, SwerveConstants.alighKd);
  public PIDController m_alignNotePID = new PIDController(SwerveConstants.alignNoteKp, 0, SwerveConstants.alignNoteKd);

  private final Timer m_keepAngleTimer = new Timer();

  double poseDifferencefl = 0;
  double poseDifferencefr = 0;
  double poseDifference = 0;
  SwerveModuleState[] xLockStates = new SwerveModuleState[4];

  public boolean m_showScreens;

  private boolean onTarget;

  private int loopctr;

  double xlim = Units.inchesToMeters(12);
  double ylim = Units.inchesToMeters(12);
  double deglim = Units.degreesToRadians(5);

  private Pose2d llpose = new Pose2d();
  private Pose2d llposefl = new Pose2d();
  private Pose2d llposefr = new Pose2d();
  private double latencyfl = 0;
  private double latencyfr = 0;

  @Log.NT(key = "yerror")
  double yerror = 0;
  @Log.NT(key = "caplatency")
  double capLatency = 0;
  @Log.NT(key = "pipelatency")
  double pipelineLatency = 0;
  @Log.NT(key = "latency")
  double latency = 0;
  @Log.NT(key = "cappose")
  Pose2d robotPose = new Pose2d();
  @Log.NT(key = "capposey")
  double poseY;
  @Log.NT(key = "capposex")
  double poseX;
  @Log.NT(key = "poseerrr2d")
  Rotation2d poser = new Rotation2d();
  @Log.NT(key = "corry")
  double correctedY;
  @Log.NT(key = "correctionpose")
  Pose2d correctionPose = new Pose2d();

  double area = 0;
  double areafl = 0;
  double areafr = 0;

  int numberTargets = 0;
  private double timestampSeconds;

  private double tagDistance;
  private double tagDistancefl;
  private double tagDistancefr;

  private boolean firstTime = true;

  @Log.NT(key = "noteseentime")
  private double noteSeenTime;

  public SwerveSubsystem(boolean showScreens) {
    m_showScreens = showScreens;
    SmartDashboard.putNumber("DCF", Constants.SwerveConstants.driveConversionPositionFactor);

    xLockStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    xLockStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    xLockStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    xLockStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

    if (RobotBase.isSimulation()) {

      // thetaPID.setP(0);

      // xPID.setP(1.0);

      // yPID.setP(0);

    }

    m_keepAngleTimer.reset();
    m_keepAngleTimer.start();
    m_keepAnglePID.enableContinuousInput(-Math.PI, Math.PI);

    gyro = new AHRS(SPI.Port.kMXP, (byte) 100);

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
        new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
        new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
        new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
    };

    swervePoseEstimator = new SwerveDrivePoseEstimator(
        Constants.SwerveConstants.swerveKinematics,
        getYaw(),
        getPositions(),
        new Pose2d(),
        ODOMETRY_STDDEV,
        VISION_STDDEV);

    simOdometryPose = swervePoseEstimator.getEstimatedPosition();

    field = new Field2d();

    resetModuleEncoders();

    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPoseEstimator,
        this::getSpeeds,
        this::driveRobotRelative,
        Constants.SwerveConstants.pathFollowerConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    zeroGyro();
    SmartDashboard.putBoolean("GYRO", gyro.isConnected());

    resetPoseEstimator(new Pose2d());

    if (m_showScreens) {

      Shuffleboard.getTab("Drivetrain").add(this)
          .withSize(2, 1).withPosition(0, 0);

      Shuffleboard.getTab("Drivetrain").add("SetDriveKp", setDriveKp())
          .withSize(1, 1).withPosition(5, 0);

      Shuffleboard.getTab("Drivetrain").addNumber("DriveKp", () -> getDriveKp())
          .withSize(1, 1).withPosition(5, 1);

      Shuffleboard.getTab("Drivetrain").add("SetDriveFF", setDriveFF())
          .withSize(1, 1).withPosition(6, 0);

      Shuffleboard.getTab("Drivetrain").addNumber("DriveFF%",
          () -> getDriveFF() * Constants.SwerveConstants.kmaxTheoreticalSpeed)
          .withSize(1, 1).withPosition(6, 1);

      Shuffleboard.getTab("Drivetrain").add("SetAngleKp", setAngleKp())
          .withSize(1, 1).withPosition(7, 0);

      Shuffleboard.getTab("Drivetrain").addNumber("AngleKp", () -> getAngleKp())
          .withSize(1, 1).withPosition(7, 1);

      Shuffleboard.getTab("Drivetrain").add("SetAlignKp", setAlignKpCommand())
          .withSize(1, 1).withPosition(8, 0);

      Shuffleboard.getTab("Drivetrain").addNumber("AlignKp", () -> getAlignKp())
          .withSize(1, 1).withPosition(8, 1);

      Shuffleboard.getTab("Drivetrain").add("ResetPose", this.setPoseToX0Y0())
          .withSize(1, 1).withPosition(2, 0);

      Shuffleboard.getTab("Drivetrain").addDoubleArray("Odometry",
          () -> new double[] { getPose().getX(), getPose().getY(),
              getPose().getRotation().getDegrees() })
          .withPosition(0, 2)
          .withSize(2, 1);

      Shuffleboard.getTab("Drivetrain").addDoubleArray("OdometryFL",
          () -> new double[] { llposefl.getX(), llposefl.getY(),
              llposefl.getRotation().getDegrees() })
          .withPosition(0, 3)
          .withSize(2, 1);
      Shuffleboard.getTab("Drivetrain").addDoubleArray("OdometryFR",
          () -> new double[] { llposefr.getX(), llposefr.getY(),
              llposefr.getRotation().getDegrees() })
          .withPosition(0, 4)
          .withSize(2, 1);

      Shuffleboard.getTab("Drivetrain").addNumber("VXMPS", () -> getSpeeds().vxMetersPerSecond)
          .withPosition(3, 2)
          .withSize(1, 1);

      Shuffleboard.getTab("Drivetrain").addNumber("VYMPS", () -> getSpeeds().vyMetersPerSecond)
          .withPosition(4, 2)
          .withSize(1, 1);

      Shuffleboard.getTab("Drivetrain").addNumber("VOMPS", () -> getSpeeds().omegaRadiansPerSecond)
          .withPosition(5, 2)
          .withSize(1, 1);

      Shuffleboard.getTab("Drivetrain").addNumber("PoseX Meters", () -> getX())
          .withPosition(6, 2)
          .withSize(1, 1);
      Shuffleboard.getTab("Drivetrain").addNumber("PoseY Meters", () -> getY())
          .withPosition(7, 2)
          .withSize(1, 1);
      Shuffleboard.getTab("Drivetrain").addNumber("PoseDeg", () -> getHeading().getDegrees())
          .withPosition(8, 2)
          .withSize(1, 1);

    }

    setModuleDriveFF();
    setModuleDriveKp();
    setModuleAngleKp();
    firstTime = true;
  }

  // public PathPlannerPath getPathToNearSource() {
  //   List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
  //     new Pose2d(getPose().getTranslation().getX(), getPose().getTranslation().getY(), Rotation2d.fromDegrees(180)),
  //     new Pose2d(6.79, 1.60, Rotation2d.fromDegrees(180))
  //   );
  //   PathPlannerPath path = new PathPlannerPath(bezierPoints, new PathConstraints(3.0, 3.0, 2 * Math.PI, 3 * Math.PI), new GoalEndState(0.0, Rotation2d.fromDegrees(180)));
  //   path.preventFlipping = false; 
  //   return path;
  // }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public ChassisSpeeds getFieldRelativeSpeeds(double translation, double strafe, double rotation) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(translation, strafe, rotation, getHeading());
  }

  public void drive(double translation, double strafe, double rotation, boolean fieldRelative, boolean isOpenLoop,
      boolean keepAngle) {
    if (keepAngle) {
      rotation = performKeepAngle(translation, strafe, rotation); // Calls the keep angle function to update the keep
                                                                  // angle or rotate
    }

    if (Math.abs(rotation) < 0.02) {
      rotation = 0.0;
    }

    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translation, strafe, rotation, getHeading())
                : new ChassisSpeeds(translation, strafe, rotation),
            .02));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.kmaxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.kmaxSpeed);
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void xLock() {
    setStates(xLockStates);
  }

  public Command xLockCommand() {
    return Commands.runOnce(() -> xLock());
  }

  public void resetModuleEncoders() {
    mSwerveMods[0].resetAngleToAbsolute();
    mSwerveMods[1].resetAngleToAbsolute();
    mSwerveMods[2].resetAngleToAbsolute();
    mSwerveMods[3].resetAngleToAbsolute();
  }

  public double getHeadingDegrees() {
    return Math.IEEEremainder((gyro.getAngle()), 360);
  }

  @Log.NT(key = "Poseestimate")
  public Pose2d getPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  public double getX() {
    return getPose().getX();
  }

  public double getY() {
    return getPose().getY();
  }

  public double getModuleStickyFaults() {
    return mSwerveMods[0].getStickyFaults()
        + mSwerveMods[1].getStickyFaults()
        + mSwerveMods[2].getStickyFaults()
        + mSwerveMods[3].getStickyFaults();
  }

  public void setModuleDriveKp() {
    mSwerveMods[0].setDriveKp();
    mSwerveMods[1].setDriveKp();
    mSwerveMods[2].setDriveKp();
    mSwerveMods[3].setDriveKp();
  }

  public void setModuleDriveFF() {
    mSwerveMods[0].setDriveFF();
    mSwerveMods[1].setDriveFF();
    mSwerveMods[2].setDriveFF();
    mSwerveMods[3].setDriveFF();
  }

  public void setModuleAngleKp() {
    mSwerveMods[0].setAngleKp();
    mSwerveMods[1].setAngleKp();
    mSwerveMods[2].setAngleKp();
    mSwerveMods[3].setAngleKp();
  }

  public double getDriveKp() {
    return mSwerveMods[0].getDriveKp();
  }

  public double getDriveFF() {
    return mSwerveMods[0].getDriveFF();
  }

  public Command setDriveKp() {
    return Commands.runOnce(() -> setModuleDriveKp());
  }

  public Command setDriveFF() {
    return Commands.runOnce(() -> setModuleDriveFF());
  }

  public Command setAngleKp() {
    return Commands.runOnce(() -> setModuleAngleKp());
  }

  public double getAngleKp() {
    return mSwerveMods[0].getAngleKp();
  }

  public boolean driveIsBraked() {
    return mSwerveMods[0].driveIsBraked();
  }

  public boolean getIsRotating() {
    return gyro.isRotating();
  }

  public void setIdleMode(boolean brake) {
    mSwerveMods[0].setIdleMode(brake);
    mSwerveMods[1].setIdleMode(brake);
    mSwerveMods[2].setIdleMode(brake);
    mSwerveMods[3].setIdleMode(brake);
  }

  public Rotation2d getHeading() {
    Rotation2d heading = new Rotation2d();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
      heading = getPose().getRotation().plus(new Rotation2d(Math.PI));
    else
      heading = getPose().getRotation();
    return heading;
  }

  public void resetPoseEstimator(Pose2d pose) {
    zeroGyro();
    swervePoseEstimator.resetPosition(getYaw(), getPositions(), pose);
    simOdometryPose = pose;
  }

  public Command setPose(Pose2d pose) {
    return Commands.runOnce(() -> resetPoseEstimator(pose));
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return swervePoseEstimator;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public boolean getOnTarget() {
    return onTarget;
  }

  public void setOnTarget(boolean on) {
    onTarget = on;
  }

  public void zeroGyro() {
    gyro.reset();
    updateKeepAngle();
  }

  public Rotation2d getYaw() {
    if (RobotBase.isReal())
      return gyro.getRotation2d();
    else
      return simOdometryPose.getRotation();
  }

  public float getPitch() {
    return gyro.getPitch();
  }

  public float getRoll() {
    return gyro.getRoll();
  }

  public Field2d getField() {
    return field;
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
  }

  public void setLookForNote() {
    lookForNote = true;
  }

  public void resetLookForNote() {
    lookForNote = false;
  }

  public boolean getLookForNote() {
    return lookForNote;
  }

  @Override
  public void periodic() {

    loopctr++;

    // getPose();

    swervePoseEstimator.update(getYaw(), getPositions());

    getPose();

    putStates();

    // doNoteVisionCorrection();

    if (firstTime && isStopped())
      firstTime = false;

    boolean leftHasTarget = CameraConstants.frontLeftCamera.isActive
        && LimelightHelpers.getTV(CameraConstants.frontLeftCamera.camname);

    if (leftHasTarget)
      doVisionValuesFL();

    boolean rightHasTarget = CameraConstants.frontRightCamera.isActive
        && LimelightHelpers.getTV(CameraConstants.frontRightCamera.camname);

    if (rightHasTarget)
      doVisionValuesFR();
  }
  

  @Log.NT(key = "SpeakerDistance")
  public double getDistanceFromSpeaker() {
    return Constants.getActiveSpeakerPose().getTranslation()
        .getDistance(getPose().getTranslation());
  }

  @Log.NT(key = "FL_Pose")
  private Pose2d getLLPoseFL() {
    return LimelightHelpers
        .getBotPoseEstimate_wpiBlue(CameraConstants.frontLeftCamera.camname).pose;
  }

  @Log.NT(key = "FL_NumTargets")
  private int getNumberTargetsFL() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraConstants.frontLeftCamera.camname).tagCount;
  }

  @Log.NT(key = "FL_Area")
  private double getAreaFL() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraConstants.frontLeftCamera.camname).avgTagArea;
  }

  @Log.NT(key = "FL_Dist")
  private double getDistanceFL() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraConstants.frontLeftCamera.camname).avgTagDist;
  }

  @Log.NT(key = "FL_Timestamp")
  private double getTimestampFL() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraConstants.frontLeftCamera.camname).timestampSeconds;
  }

  @Log.NT(key = "FL_PoseDifference")
  private double getPoseDifferenceFL() {
    return getPose().getTranslation()
        .getDistance(getLLPoseFL().getTranslation());
  }

  private void doVisionValuesFL() {
    numberTargets = getNumberTargetsFL();
    area = getAreaFL();
    llpose = getLLPoseFL();
    timestampSeconds = getTimestampFL();
    tagDistance = getDistanceFL();
    if (!AllianceUtil.isRedAlliance()) {
      Translation2d t2d = llpose.getTranslation();
      Rotation2d r = llpose.getRotation();
      Rotation2d r180 = r.rotateBy(new Rotation2d(0)); // Didn't need to rotate robot. This might be alliance specific?
      llpose = new Pose2d(t2d, r180);
    }
    // if (llpose.getX() == 0.0
    // || llpose.getX() > FieldConstants.FIELD_LENGTH
    // || llpose.getY() < 0.0
    // || llpose.getY() > FieldConstants.FIELD_WIDTH)
    // return;

    // distance from current pose to vision estimated pose
    poseDifference = getPoseDifferenceFL();

    if (cameraSelection.intValue() == 1 || cameraSelection.intValue() == 3
        || DriverStation.isTeleopEnabled() || DriverStation.isDisabled()) {
      double xyStds = .3;
      double radStds = .8;
      swervePoseEstimator.setVisionMeasurementStdDevs(
          VecBuilder.fill(xyStds, xyStds, radStds));
      if ((numberTargets > 1 && poseDifference < 2) || tagDistance < 8 || poseDifference < 0.5 || area > 0.3) {

        SmartDashboard.putNumber("VOL", timestampSeconds);
        swervePoseEstimator.addVisionMeasurement(
            llpose,
            timestampSeconds);

      }
    }

  }

  @Log.NT(key = "FR_Pose")
  private Pose2d getLLPoseFR() {
    return LimelightHelpers
        .getBotPoseEstimate_wpiBlue(CameraConstants.frontRightCamera.camname).pose;
  }

  @Log.NT(key = "FR_NumTargets")
  private int getNumberTargetsFR() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraConstants.frontRightCamera.camname).tagCount;
  }

  @Log.NT(key = "FR_Area")
  private double getAreaFR() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraConstants.frontRightCamera.camname).avgTagArea;
  }

  @Log.NT(key = "FR_Dist")
  private double getDistanceFR() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraConstants.frontRightCamera.camname).avgTagDist;
  }

  @Log.NT(key = "FR_Timestamp")
  private double getTimestampFR() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraConstants.frontLeftCamera.camname).timestampSeconds;
  }

  @Log.NT(key = "FR_PoseDifference")
  private double getPoseDifferenceFR() {
    return swervePoseEstimator.getEstimatedPosition().getTranslation()
        .getDistance(getLLPoseFR().getTranslation());
  }

  private void doVisionValuesFR() {
    numberTargets = getNumberTargetsFR();
    area = getAreaFR();
    llpose = getLLPoseFR();
    timestampSeconds = getTimestampFR();
    tagDistance = getDistanceFR();
    if (!AllianceUtil.isRedAlliance()) {
      Translation2d t2d = llpose.getTranslation();
      Rotation2d r = llpose.getRotation();
      Rotation2d r180 = r.rotateBy(new Rotation2d(0)); // Didn't need to rotate robot. This might be alliance specific?
      llpose = new Pose2d(t2d, r180);
    }
    if (llpose.getX() == 0.0
        || llpose.getX() > FieldConstants.FIELD_LENGTH
        || llpose.getY() < 0.0
        || llpose.getY() > FieldConstants.FIELD_WIDTH)
      return;

    // distance from current pose to vision estimated pose
    poseDifference = getPoseDifferenceFR();

    if (cameraSelection.intValue() == 2 || cameraSelection.intValue() == 3
        || DriverStation.isTeleopEnabled() || DriverStation.isDisabled()) {
      double xyStds = .3;
      double radStds = .8;
      swervePoseEstimator.setVisionMeasurementStdDevs(
          VecBuilder.fill(xyStds, xyStds, radStds));
      if ((numberTargets > 1 && poseDifference < 2) || tagDistance < 8 || poseDifference < 0.5 || area > 0.3) {
        SmartDashboard.putNumber("VOR", timestampSeconds);

        swervePoseEstimator.addVisionMeasurement(
            llpose,
            timestampSeconds);

      }
    }
  }

  public void doNoteVisionCorrection() {
    String rname = CameraConstants.rearCamera.camname;
    double corrGain = .001;

    if (LimelightHelpers.getTV(rname)) {

      yerror = LimelightHelpers.getTX(rname);
      capLatency = LimelightHelpers.getLatency_Capture(rname);
      pipelineLatency = LimelightHelpers.getLatency_Pipeline(rname);
      robotPose = getPose();
      poseY = robotPose.getY();
      poseX = robotPose.getX();
      poser = robotPose.getRotation();
      correctedY = poseY - yerror * corrGain;
      correctionPose = new Pose2d(poseX, correctedY, poser);
      latency = capLatency + pipelineLatency;

      if (firstTime) {
        noteSeenTime = Timer.getFPGATimestamp();
        swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.3, .3, .8));

        swervePoseEstimator.addVisionMeasurement(
            correctionPose,
            latency);

        firstTime = false;
      }
    }
  }

  /**
   * Keep angle function is performfz to combat drivetrain drift without the need
   * of constant "micro-adjustments" from the driver.
   * A PIDController is used to attempt to maintain the robot heading to the
   * keepAngle value. This value is updated when the robot
   * is rotated manually by the driver input
   * 
   * @return rotation command in radians/s
   * @param xSpeed is the input drive X speed command
   * @param ySpeed is the input drive Y speed command
   * @param rot    is the input drive rotation speed command
   */
  private double performKeepAngle(double xSpeed, double ySpeed, double rot) {
    double output = rot; // Output shouold be set to the input rot command unless the Keep Angle PID is
                         // called
    if (Math.abs(rot) >= 0.01) { // If the driver commands the robot to rotate set the
                                 // last rotate time to the current time
      lastRotTime = m_keepAngleTimer.get();
    }
    if (Math.abs(xSpeed) >= 0.01
        || Math.abs(ySpeed) >= 0.01) { // if driver commands robot to translate set the
                                       // last drive time to the current time
      lastDriveTime = m_keepAngleTimer.get();
    }
    timeSinceRot = m_keepAngleTimer.get() - lastRotTime; // update variable to the current time - the last rotate time

    timeSinceDrive = m_keepAngleTimer.get() - lastDriveTime; // update variable to the current time - the last drive
                                                             // time
    if (timeSinceRot < 0.25) { // Update keepAngle until 0.5s after rotate command stops to allow rotation
                               // move to finish

      keepAngle = getYaw().getRadians();

    } else if (Math.abs(rot) <= 0.01 && timeSinceDrive < 0.25) { // Run Keep angle pid
                                                                 // until 0.75s after drive
                                                                 // command stops to combat
                                                                 // decel drift
      output = m_keepAnglePID.calculate(getYaw().getRadians(), keepAngle); // Set output command to the result of the
                                                                           // Keep Angle PID
    }
    return output;
  }

  public void updateKeepAngle() {
    keepAngle = getYaw().getRadians();
  }

  private void resetAll() {

    // gyro.reset();
    resetModuleEncoders();
    // swervePoseEstimator.resetPosition(getYaw(), getPositions(), new Pose2d());
    swervePoseEstimator.resetPosition(new Rotation2d(Math.PI), getPositions(), new Pose2d());

    simOdometryPose = new Pose2d();
    updateKeepAngle();

  }

  public void alignToAngle(double angle) {
    double angleComp = getAngleComp();
    drive(0, 0, m_alignPID.calculate(angle, angleComp), false, false, false);
  }

  public double getAngleComp() {
    double temp = 0;
    Pose2d rel = getPose().relativeTo(Constants.getActiveSpeakerPose());
    double angle = rel.getRotation().getRadians();
    temp = Math.sin(angle) * .001;
    return temp;
  }

  public Command setPoseToX0Y0() {
    return Commands.runOnce(() -> resetAll());
  }

  public double getAlignKp() {
    return m_alignPID.getP();
  }

  public void setAlignKp() {
    m_alignPID.setP(Pref.getPref("AlignkP"));
  }

  public Command setAlignKpCommand() {
    return Commands.runOnce(() -> setAlignKp());
  }

  public PIDController getAlignPID() {
    return m_alignPID;
  }

  @Override
  public void simulationPeriodic() {

    SwerveModuleState[] measuredStates

        = new SwerveModuleState[] {
            mSwerveMods[0].getState(),
            mSwerveMods[1].getState(),
            mSwerveMods[2].getState(),
            mSwerveMods[3].getState()
        };

    ChassisSpeeds speeds = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(measuredStates);

    simOdometryPose = simOdometryPose.exp(
        new Twist2d(
            speeds.vxMetersPerSecond * .02,
            speeds.vyMetersPerSecond * .02,
            speeds.omegaRadiansPerSecond * .02));

  }

  private void putStates() {

    double[] realStates = {
        mSwerveMods[0].getState().angle.getDegrees(),
        mSwerveMods[0].getState().speedMetersPerSecond,
        mSwerveMods[1].getState().angle.getDegrees(),
        mSwerveMods[1].getState().speedMetersPerSecond,
        mSwerveMods[2].getState().angle.getDegrees(),
        mSwerveMods[2].getState().speedMetersPerSecond,
        mSwerveMods[3].getState().angle.getDegrees(),
        mSwerveMods[3].getState().speedMetersPerSecond
    };

    double[] theoreticalStates = {
        mSwerveMods[0].getDesiredState().angle.getDegrees(),
        mSwerveMods[0].getDesiredState().speedMetersPerSecond,
        mSwerveMods[1].getDesiredState().angle.getDegrees(),
        mSwerveMods[1].getDesiredState().speedMetersPerSecond,
        mSwerveMods[2].getDesiredState().angle.getDegrees(),
        mSwerveMods[2].getDesiredState().speedMetersPerSecond,
        mSwerveMods[3].getDesiredState().angle.getDegrees(),
        mSwerveMods[3].getDesiredState().speedMetersPerSecond
    };

    SmartDashboard.putNumberArray("Theoretical States", theoreticalStates);
    SmartDashboard.putNumberArray("Real States", realStates);

    SmartDashboard.putNumber("KeepAngle", keepAngle);
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

  public boolean isStopped() {
    return mSwerveMods[0].isStopped()
        && mSwerveMods[1].isStopped()
        && mSwerveMods[2].isStopped()
        && mSwerveMods[3].isStopped();
  }

  public Command clearFaultsCommand() {
    return Commands.parallel(
        mSwerveMods[0].clearFaultsCommand(),
        mSwerveMods[1].clearFaultsCommand(),
        mSwerveMods[2].clearFaultsCommand(),
        mSwerveMods[3].clearFaultsCommand());
  }

  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.per(Second).of(1.0), Volts.of(4.0), null, null),
      new SysIdRoutine.Mechanism(
          (volts) -> {
            mSwerveMods[0].setCharacterizationVolts(volts.in(Volts));
            mSwerveMods[1].setCharacterizationVolts(volts.in(Volts));

            mSwerveMods[2].setCharacterizationVolts(volts.in(Volts));
            mSwerveMods[3].setCharacterizationVolts(volts.in(Volts));
          },
          log -> {
            log.motor("Front Left")
                .linearVelocity(MetersPerSecond.of(mSwerveMods[0].getDriveVelocity()))
                .linearPosition(Meters.of(mSwerveMods[0].getPosition().distanceMeters))
                .voltage(Volts.of(mSwerveMods[0].getVoltage()));

            log.motor("Front Right")
                .linearVelocity(MetersPerSecond.of(mSwerveMods[1].getDriveVelocity()))
                .linearPosition(Meters.of(mSwerveMods[1].getPosition().distanceMeters))
                .voltage(Volts.of(mSwerveMods[1].getVoltage()));

            log.motor("Back Left")
                .linearVelocity(MetersPerSecond.of(mSwerveMods[2].getDriveVelocity()))
                .linearPosition(Meters.of(mSwerveMods[2].getPosition().distanceMeters))
                .voltage(Volts.of(mSwerveMods[2].getVoltage()));

            log.motor("Back Right")
                .linearVelocity(MetersPerSecond.of(mSwerveMods[3].getDriveVelocity()))
                .linearPosition(Meters.of(mSwerveMods[3].getPosition().distanceMeters))
                .voltage(Volts.of(mSwerveMods[3].getVoltage()));
          },
          this));

  public Integer cameraSelection = 0;

  public Command quasistaticForward() {
    return Commands.sequence(
        runOnce(
            () -> {
              mSwerveMods[0].setCharacterizationVolts(0.0);
              mSwerveMods[1].setCharacterizationVolts(0.0);
              mSwerveMods[2].setCharacterizationVolts(0.0);
              mSwerveMods[3].setCharacterizationVolts(0.0);
            }),
        Commands.waitSeconds(0.50),
        sysIdRoutine.quasistatic(Direction.kForward))
        .finallyDo(
            () -> {
              mSwerveMods[0].stopCharacterizing();
              mSwerveMods[1].stopCharacterizing();
              mSwerveMods[2].stopCharacterizing();
              mSwerveMods[3].stopCharacterizing();
            });
  }

  public Command quasistaticBackward() {
    return Commands.sequence(
        runOnce(
            () -> {
              mSwerveMods[0].setCharacterizationVolts(0.0);
              mSwerveMods[1].setCharacterizationVolts(0.0);
              mSwerveMods[2].setCharacterizationVolts(0.0);
              mSwerveMods[3].setCharacterizationVolts(0.0);
            }),
        Commands.waitSeconds(0.50),
        sysIdRoutine.quasistatic(Direction.kReverse))
        .finallyDo(
            () -> {
              mSwerveMods[0].stopCharacterizing();
              mSwerveMods[1].stopCharacterizing();
              mSwerveMods[2].stopCharacterizing();
              mSwerveMods[3].stopCharacterizing();
            });
  }

  public Command dynamicForward() {
    return Commands.sequence(
        runOnce(
            () -> {
              mSwerveMods[0].setCharacterizationVolts(0.0);
              mSwerveMods[1].setCharacterizationVolts(0.0);
              mSwerveMods[2].setCharacterizationVolts(0.0);
              mSwerveMods[3].setCharacterizationVolts(0.0);
            }),
        Commands.waitSeconds(0.50),
        sysIdRoutine.dynamic(Direction.kForward))
        .finallyDo(
            () -> {
              mSwerveMods[0].stopCharacterizing();
              mSwerveMods[1].stopCharacterizing();
              mSwerveMods[2].stopCharacterizing();
              mSwerveMods[3].stopCharacterizing();
            });
  }

  public Command dynamicBackward() {
    return Commands.sequence(
        runOnce(
            () -> {
              mSwerveMods[0].setCharacterizationVolts(0.0);
              mSwerveMods[1].setCharacterizationVolts(0.0);
              mSwerveMods[2].setCharacterizationVolts(0.0);
              mSwerveMods[3].setCharacterizationVolts(0.0);
            }),
        Commands.waitSeconds(0.50),
        sysIdRoutine.dynamic(Direction.kReverse))
        .finallyDo(
            () -> {
              mSwerveMods[0].stopCharacterizing();
              mSwerveMods[1].stopCharacterizing();
              mSwerveMods[2].stopCharacterizing();
              mSwerveMods[3].stopCharacterizing();
            });
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    return new double[] {
        mSwerveMods[0].getPositionRadians(),
        mSwerveMods[0].getPositionRadians(),
        mSwerveMods[0].getPositionRadians(),
        mSwerveMods[0].getPositionRadians()
    };
  }

}
