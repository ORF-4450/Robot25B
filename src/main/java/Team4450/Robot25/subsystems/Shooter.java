package Team4450.Robot25.subsystems;

import Team4450.Lib.Util;
import Team4450.Robot25.AdvantageScope;
import Team4450.Robot25.Robot;

import static Team4450.Robot25.Constants.SHOOTER_MOTOR_TOP;
import static Team4450.Robot25.Constants.SHOOTER_MOTOR_BOTTOM;
import static Team4450.Robot25.Constants.SHOOTER_MOTOR_FEEDER;
import static Team4450.Robot25.Constants.SHOOTER_MOTOR_PIVOT;

import static Team4450.Robot25.Constants.SHOOTER_SPEED;
import static Team4450.Robot25.Constants.SHOOTER_PIVOT_FACTOR;
// import static Team4450.Robot25.Constants.SHOOTER_PRECISE_PIVOT_FACTOR;
import static Team4450.Robot25.Constants.SHOOTER_FEED_SPEED;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* ANGLE REFERENCE:
                   ||||
                   ||
                   ||
                 \ ||
        <-- shoot  ||------0deg-------
                   || \___
                   ||  \  \__
                   ||    \   \-39deg
                   ||      \     \__
          -90deg | ||  -50deg \    INTAKE
  =================================INTAKE  front -->
  8888                              8888
  8888                              8888
 */

/**
 * Subsystem for the Shooter subassemebly on the 2024 robot. Should not be
 * used on its own, should be contained within ElevatedShooter subsystem
 */
public class Shooter extends SubsystemBase {
    private SparkMax motorTop = new SparkMax(SHOOTER_MOTOR_TOP, MotorType.kBrushless);
    private SparkMax motorBottom = new SparkMax(SHOOTER_MOTOR_BOTTOM, MotorType.kBrushless);
    private SparkMax motorFeeder = new SparkMax(SHOOTER_MOTOR_FEEDER, MotorType.kBrushless);
    private SparkMax motorPivot = new SparkMax(SHOOTER_MOTOR_PIVOT, MotorType.kBrushless);
    
    private SparkMaxConfig bottomConfig = new SparkMaxConfig();
    private SparkMaxConfig feederConfig = new SparkMaxConfig();

    public boolean hasShot = false; // used to end spin up command, should only be read publicly

    private final double pivotFactor = SHOOTER_PIVOT_FACTOR;
    // private final double pivotFactor = SHOOTER_PRECISE_PIVOT_FACTOR;

    private RelativeEncoder pivotEncoder;
    private RelativeEncoder topMotorEncoder;
    private RelativeEncoder bottomMotorEncoder;

    private SparkLimitSwitch noteSensor;

    private double shooterSpeed = SHOOTER_SPEED;
    private double feedSpeed = SHOOTER_FEED_SPEED;

    private ProfiledPIDController pivotPID;
    private boolean shooterIsRunning = false, feederIsRunning = false;
    private final double PIVOT_TOLERANCE = 1; //encoder counts: not angle

    private final double PIVOT_START = -39; // angle in degrees

    private double goal = PIVOT_START;

    // NOTE: I removed the shuffleboard speed setting because they were too
    // much of a hassle to handle with all of the different speed states the shooter could be in
    // (feeding, slow feeding, inverse feeding, shooting, etc.)

    /**
     * Shooter of 2024 robot USS ProtoStar
     */
    public Shooter() {
        Util.consoleLog();

        bottomConfig.follow(motorTop);
        motorBottom.configure(bottomConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        //motorBottom.follow(motorTop);

        feederConfig.inverted(true);
        feederConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
        feederConfig.limitSwitch.forwardLimitSwitchEnabled(false);
        motorFeeder.configure(feederConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        //motorFeeder.setInverted(true);

        pivotEncoder = motorPivot.getEncoder();
        // pivotEncoder = motorPivot.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192); // rev through bore keep 8192 (constant)
        // pivotEncoder.setInverted(true);
        
        topMotorEncoder = motorTop.getEncoder();
        bottomMotorEncoder = motorBottom.getEncoder();

        resetEncoders();

        // Optical sensor plugs into Spark MAX itself
        noteSensor = motorFeeder.getForwardLimitSwitch();

        // profiled PID controller allows us to control acceleration and
        // deceleration and max speed of the pivot!
        pivotPID = new ProfiledPIDController(0.12, 0, 0,
            new Constraints(angleToEncoderCounts(360 *8), angleToEncoderCounts(11*360)) // max velocity(/s), max accel(/s)
        );
        
        pivotPID.setTolerance(PIVOT_TOLERANCE); // encoder counts not degrees for this one

        Util.consoleLog("Shooter created!");
    }

    /**
     * whether the shooter posesses a note
     * @return if the robot has note (true) or note (false)
     */
    public boolean hasNote() {
        if (RobotBase.isSimulation())
            return AdvantageScope.getInstance().hasAGamepiece();
        else
            return noteSensor.isPressed();
    }

    /**
     * enables the feed motor (sushi rollers) to push the Note into (or out of)
     * the rolling shooter wheels (which must be enabled seperately)
     * @param speedfactor the speed bounded [-1,1] of max feedspeed
     */
    public void startFeeding(double speedfactor) {
        SmartDashboard.putNumber("sushi", speedfactor);
        motorFeeder.set(Util.clampValue(speedfactor, 1) * feedSpeed);
        feederIsRunning = true;
        updateDS();
    }

    /*
     * set the current position as the "zero" position.
     * Careful!
     */
    public void resetEncoders() {
        pivotEncoder.setPosition(angleToEncoderCounts(-39));
        topMotorEncoder.setPosition(0);
        bottomMotorEncoder.setPosition(0);
    }

    /**
     * set the current setpoint/goal to the current position,
     * essentially "locking" the pivot in place
     */
    public void lockPosition() {
        goal = getAngle();
        pivotPID.reset(angleToEncoderCounts(goal));
    }

    /**
     * remove setpoint control, causing pivot to become limp and
     * react to external forces with no braking or anything.
     */
    public void unlockPosition() {
        goal = Double.NaN; // when setpoint NaN it doesn't do it
    }
    
    /**
     * set whether the note sensor triggers the feed rollers to stop or not
     * @param enabled whether the motor should pay attention to sensor or not
     */
    public void enableClosedLoopFeedStop(boolean enabled) {
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        //noteSensor.enableLimitSwitch();
        feederConfig.limitSwitch.forwardLimitSwitchEnabled(enabled);
        motorFeeder.configure(feederConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** stops the feed motor */
    public void stopFeeding() {
        motorFeeder.set(0);
        feederIsRunning = false;
        updateDS();
    }

    /** spins the shooter wheels */
    public void startShooting() {
        motorTop.set(shooterSpeed);
        shooterIsRunning = true;
        updateDS();
    }

    /**
     * spins the shooter wheels at the given % speed
     * @param factor bounded [-1, 1] of total max speed
     */
    public void startShooting(double factor) {
        motorTop.set(shooterSpeed * factor);
        shooterIsRunning = true;
        updateDS();
    }

    /**
     * run the shooter wheels backwards at 15% power.
     */
    public void backShoot() {
        motorTop.set(-0.15);
        shooterIsRunning = true;
        updateDS();
    }

    /** stops the shooter wheels */
    public void stopShooting() {
        motorTop.set(0);
        shooterIsRunning = false;
        updateDS();
    }

    /**
     * Sets the shooter assembly to a given angle
     * @param angle the angle in degrees
     */
    public void setAngle(double angle) {
        goal = angle;
    }

    /**
     * Get the angle of the shooter assembly
     * @return the angle in degrees
     */
    public double getAngle() {
        double roughAngle = pivotEncoder.getPosition() * pivotFactor; // convert to degrees
        // if (roughAngle < 15 && roughAngle > -80 && RobotBase.isReal()) {
        //     return pivotCoolEncoder.getPosition() * SHOOTER_PRECISE_PIVOT_FACTOR;
        // }
        return roughAngle;
    }

    /**
     * Get the average wheel speed of top and bottom
     * shooter rollers
     * @return the average wheel speed in meters per second
     */
    public double getWheelSpeed() {
        double wheelRadius = 1.5 * 0.0254; // 1.5 in -> meters
        double topWheelSpeed = (Math.abs(topMotorEncoder.getVelocity()) / 60.0) * wheelRadius * 2 * Math.PI; // rpm -> m/s
        double bottomWheelSpeed = (Math.abs(bottomMotorEncoder.getVelocity()) / 60.0) * wheelRadius * 2 * Math.PI; // rpm -> m/s
        double averageWheelSpeed = 0.4 * 0.5 * (topWheelSpeed + bottomWheelSpeed); // average of top and bottom (0.4 arbitrary tuning const)
        // Explanation for above calculation. Average should have been .5 * accum speed. It was incorrectly coded
        // as .2 * accum speed. Since we ran all season with .2, we were reluctant to change it when the error was
        // discovered. The calculation now reflects the correct .5 * accum speed and the .4 factor to the the same
        // result as using .2, leaving the calculation "incorrect" but as we ran it, so no change in behavior.
        // 10-15-24.
        return averageWheelSpeed;
    }

    /**
     * checks if shooter is at angle with tolerance
     * @param angle the angle to check against
     * @return true or false
     */
    public boolean isAtAngle(double angle) {
        // check if the absolute difference < tolerance
        return Math.abs(pivotEncoder.getPosition() - angleToEncoderCounts(angle)) < PIVOT_TOLERANCE;
    }

    /**
     * given an angle, return the encoder counts
     * @param angle angle of shooter position: 0 is nominal angle in degrees
     *              (see beginning of Shooter.java for reference)
     * @return the raw encoder position
     */
    private double angleToEncoderCounts(double angle) {
        return angle / pivotFactor;
    }

    /**
     * Increment or decrement the setpoint (for manual joystick use)
     * @param amount the # of degrees to change setpoint by
     */
    public void movePivotRelative(double amount) {
        goal += amount;
        // motorPivot.set(0.4*speed);
        // if (Robot.isSimulation()) pivotEncoder.setPosition(pivotEncoder.getPosition() + (0.5*speed));
    }

    @Override
    public void periodic() {
        // for simulation and logging
        AdvantageScope.getInstance().setShooterAngle(getAngle());
        SmartDashboard.putNumber("Shooter Angle", getAngle());
        SmartDashboard.putBoolean("Note Sensor", hasNote());
        SmartDashboard.putNumber("pivot_setpoint", angleToEncoderCounts(goal));
        SmartDashboard.putNumber("pivot_measured", pivotEncoder.getPosition());

        // if goal/setpoint is NaN, then just ignore the setpoint
        if (Double.isNaN(goal)) return;

        double motorOutput = pivotPID.calculate(pivotEncoder.getPosition(), angleToEncoderCounts(goal));
        SmartDashboard.putNumber("PIVOT_SPEED", motorOutput);
        motorPivot.set(motorOutput);

        // simulate shooter movement by incrementing position based on speed (not super accurate but
        // until REV adds proper Spark MAX simulation support this is what works within reason without
        // doing a ton of annoying physics I didn't want to worry about -cole)
        if (Robot.isSimulation()) pivotEncoder.setPosition(pivotEncoder.getPosition() + (2*motorOutput));
    }

    /**
     * update DriverStation status
     */
    private void updateDS()
    {
        SmartDashboard.putBoolean("Shooter", shooterIsRunning);
        SmartDashboard.putBoolean("Feeder", feederIsRunning);
    }
}
