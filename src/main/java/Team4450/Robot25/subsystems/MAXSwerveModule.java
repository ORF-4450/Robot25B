// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package Team4450.Robot25.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotBase;

import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import Team4450.Lib.Util;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import Team4450.Robot25.Constants.ModuleConstants;

/**
 * Represents one of the (four hopefully) Rev MAXSwerve modules on the DriveBase.
 * This class should only be used by DriveBase, never interact with it individually.
 */
public class MAXSwerveModule implements Sendable {
  private final SparkMax drivingSparkMax;
  private final SparkMax turningSparkMax;

  private SparkMaxConfig drivingConfig = new SparkMaxConfig();
  private SparkMaxConfig turningConfig = new SparkMaxConfig();

  private SparkSim drivingSim = null, turningSim = null;

  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkClosedLoopController drivingPIDController;
  private final SparkClosedLoopController turningPIDController;

  private double            chassisAngularOffset = 0, lastDrivePIDReference = 0;

  private double            currentSimVelocity = 0, currentSimPosition = 0, currentSimAngle = 0;
  public String             moduleLocation;
  private Pose2d            pose;
  private Translation2d     translation2d;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKMAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, String moduleLocation) {
    this.moduleLocation = moduleLocation;

    Util.consoleLog("%s", moduleLocation);
               
    SendableRegistry.addLW(this, "DriveBase/Swerve Modules", moduleLocation);
    
    drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    //richdrivingSparkMax.restoreFactoryDefaults();
    //turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    drivingEncoder = drivingSparkMax.getEncoder();
    turningEncoder = turningSparkMax.getAbsoluteEncoder();

    drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    drivingPIDController = drivingSparkMax.getClosedLoopController();

    turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    turningPIDController = turningSparkMax.getClosedLoopController();

    //richdrivingPIDController.setFeedbackDevice(drivingEncoder);
    //turningPIDController.setFeedbackDevice(turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    drivingConfig.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    drivingConfig.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    //richdrivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    //drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    turningConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    //richturningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    //richturningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    turningConfig.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted);
    //richturningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningConfig.closedLoop.positionWrappingEnabled(true);
    //richturningPIDController.setPositionPIDWrappingEnabled(true);
    turningConfig.closedLoop.positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    //richturningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    turningConfig.closedLoop.positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
    //richturningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    drivingConfig.closedLoop.pidf(ModuleConstants.kDrivingP,
                                  ModuleConstants.kDrivingI,
                                  ModuleConstants.kDrivingD,
                                  ModuleConstants.kDrivingFF);

    //rich drivingPIDController.setP(ModuleConstants.kDrivingP);   
    // drivingPIDController.setI(ModuleConstants.kDrivingI);
    // drivingPIDController.setD(ModuleConstants.kDrivingD);
    // drivingPIDController.setFF(ModuleConstants.kDrivingFF);

    drivingConfig.closedLoop.outputRange(ModuleConstants.kDrivingMinOutput, 
                                         ModuleConstants.kDrivingMaxOutput);
    
    //richdrivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
    //                                    ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    turningConfig.closedLoop.pidf(ModuleConstants.kTurningP,
                                  ModuleConstants.kTurningI,
                                  ModuleConstants.kTurningD,
                                  ModuleConstants.kTurningFF);

    //rich turningPIDController.setP(ModuleConstants.kTurningP);
    // turningPIDController.setI(ModuleConstants.kTurningI);
    // turningPIDController.setD(ModuleConstants.kTurningD);
    // turningPIDController.setFF(ModuleConstants.kTurningFF);
    
    turningConfig.closedLoop.outputRange(ModuleConstants.kTurningMinOutput, 
                                         ModuleConstants.kTurningMaxOutput);

    //richturningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
    //                                    ModuleConstants.kTurningMaxOutput);

    drivingConfig.idleMode(ModuleConstants.kDrivingMotorIdleMode);
    //richdrivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    
    turningConfig.idleMode(ModuleConstants.kTurningMotorIdleMode);
    //richturningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    
    drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    //drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    
    turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    //richturningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    drivingSparkMax.configure(drivingConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    //richdrivingSparkMax.burnFlash();
    turningSparkMax.configure(turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    //richturningSparkMax.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    
    //rich if (RobotBase.isSimulation())
    //   desiredState.angle = new Rotation2d();
    // else
    //   desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    
    drivingEncoder.setPosition(0);
    
    if (RobotBase.isSimulation()) 
    {
      // Note that the REV simulation does not work correctly. We have hacked
      // a solution where we drive the sim through our code, not by reading the
      // REV simulated encoder position and velocity, which are incorrect. However, 
      // registering the motor controller with the REV sim is still needed.

      turningSim = new SparkSim(drivingSparkMax, DCMotor.getNeo550(1));
      //richREVPhysicsSim.getInstance().addSparkMax(turningSparkMax, DCMotor.getNeo550(1));

      drivingSim = new SparkSim(drivingSparkMax, DCMotor.getNEO(1));
      //richREVPhysicsSim.getInstance().addSparkMax(drivingSparkMax, DCMotor.getNEO(1));
    }
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.

    if (RobotBase.isReal())
      return new SwerveModuleState(drivingEncoder.getVelocity(),
          new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    else
      return new SwerveModuleState(currentSimVelocity,
          new Rotation2d(currentSimAngle - chassisAngularOffset));
  } 

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    SwerveModulePosition  position;

    if (RobotBase.isReal())
      position = new SwerveModulePosition(
          drivingEncoder.getPosition(),
          new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    else
      position = new SwerveModulePosition(
          currentSimPosition,
          new Rotation2d(currentSimAngle - chassisAngularOffset));
    
    return position;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    //richdesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    desiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
    
    //rich SwerveModuleState correctedDesiredState = new SwerveModuleState();
    // correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    // correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    //SmartDashboard.putNumber(moduleLocation + " desired speed", correctedDesiredState.speedMetersPerSecond);

    // Optimize the reference state to avoid spinning further than 90 degrees.
    desiredState.optimize(new Rotation2d(turningEncoder.getPosition()));

    //richSwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
    //    new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPIDController.setReference(desiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    lastDrivePIDReference = desiredState.speedMetersPerSecond;
    
    turningPIDController.setReference(desiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    currentSimAngle = desiredState.angle.getRadians();

    currentSimVelocity = desiredState.speedMetersPerSecond;

    //rich drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    // turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    // currentSimAngle = optimizedDesiredState.angle.getRadians();

    // currentSimVelocity = optimizedDesiredState.speedMetersPerSecond;
    
    double distancePer20Ms = currentSimVelocity / 50.0;

    currentSimPosition += distancePer20Ms;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }

  /**
   * Sets module pose (not robot pose). Used for field2d display.
   * @param pose The module pose to set.
   */
  public void setModulePose(Pose2d pose) 
  {
      this.pose = pose;
  }

  /**
   * Returns the module pose (Not robot pose).
   * @return Module pose.
   */
  public Pose2d getPose()
  {
      //SmartDashboard.putString(moduleLocation + " pose", pose.toString());

      return pose;
  }
  
  /**
   * Returns the module steering angle.
   * @return Steering angle
   */
  public Rotation2d getAngle2d() 
  {
      Rotation2d  rot;

      if (RobotBase.isReal())
          rot = new Rotation2d(turningEncoder.getPosition());
      else
          rot = new Rotation2d(currentSimAngle - chassisAngularOffset);

      return rot;
  }

  public void setTranslation2d(Translation2d translation) 
  {
      translation2d = translation;            
  }

  public Translation2d getTranslation2d() 
  {
      return translation2d;
  }

  public void setBrakeMode(boolean on)
  {
    SparkMaxConfig drivingConfig = new SparkMaxConfig();

    if (on)
      //drivingSparkMax.setIdleMode(IdleMode.kBrake);
      drivingConfig.idleMode(IdleMode.kBrake);
    else
      //drivingSparkMax.setIdleMode(IdleMode.kCoast);
      drivingConfig.idleMode(IdleMode.kCoast);

    drivingSparkMax.configure(drivingConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);  
  }

  /**
   * Return current drive velocity
   * @return Velocity m/s.
   */
  public double getVelocity()
  {
    if (RobotBase.isReal())
      return drivingEncoder.getVelocity();
    else
      return currentSimVelocity;
  }

  @Override
	public void initSendable( SendableBuilder builder )
	{
    builder.setSmartDashboardType(getClass().getSimpleName());

    builder.addDoubleProperty("1 Cur pos dist", () -> getPosition().distanceMeters, null);
    builder.addDoubleProperty("2 Cur pos angle", () -> getPosition().angle.getDegrees(), null);
    builder.addStringProperty("3 Pose", () -> getPose().toString(), null);
    builder.addDoubleProperty("4 Velocity SP", () -> currentSimVelocity, null);
    builder.addDoubleProperty("5 Steer angle SP", () -> Math.toDegrees(currentSimAngle), null);
    builder.addDoubleProperty("6 Actual velocity", () -> getVelocity(), null);
    builder.addDoubleProperty("7 Actual steer sngle", () -> getAngle2d().getDegrees(), null);
    builder.addDoubleProperty("8 Drive PID reference", () -> lastDrivePIDReference, null);
	}   
}
