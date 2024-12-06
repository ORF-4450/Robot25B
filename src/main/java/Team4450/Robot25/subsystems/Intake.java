package Team4450.Robot25.subsystems;

import static Team4450.Robot25.Constants.INTAKE_MOTOR_1;
import static Team4450.Robot25.Constants.INTAKE_MOTOR_2;
import static Team4450.Robot25.Constants.INTAKE_SPEED;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Intake for the 2024 robot the USS ProtoStar
 */
public class Intake extends SubsystemBase {
    private SparkMax motor1 = new SparkMax(INTAKE_MOTOR_1, MotorType.kBrushless);
    private SparkMax motor2 = new SparkMax(INTAKE_MOTOR_2, MotorType.kBrushless);
    private SparkMaxConfig smConfig = new SparkMaxConfig();

    private double  motorSpeed = INTAKE_SPEED;
    private boolean isrunning = false;

    public Intake() {
        smConfig.idleMode(IdleMode.kBrake);
        
        motor1.configure(smConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        //motor1.setIdleMode(IdleMode.kBrake);
        //motor2.setIdleMode(IdleMode.kBrake);
        
        smConfig.follow(motor1);

        motor2.configure(smConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        //motor2.follow(motor1); // they need to do the same things
        // we don't reverse one because they are physically mounted reversed

        Util.consoleLog("Intake created!");
    }

    /**
     * run the intake at INTAKE_SPEED * speedfactor
     * @param speedfactor from -1 to 0 to 1
     */
    public void start(double speedfactor) {
        // Util.consoleLog();

        isrunning = Math.abs(speedfactor) > 0.02;
        updateDS();

        SmartDashboard.putNumber("intake_speedfactor", speedfactor);

        motor1.set(Util.clampValue(speedfactor, 1) * motorSpeed);
        // motor2.set(Util.clampValue(speedfactor, 1) * motorSpeed);
    }

    /**
     * start the intake running at full speed (INTAKE_SPEED)
     */
    public void start() {
        start(1);
    }


    /**
     * stop the intake
     */
    public void stop() {
        Util.consoleLog();

        motor1.stopMotor();
        motor2.stopMotor();

        isrunning = false;
        updateDS();
    }

    /**
     * update DriverStation status
     */
    private void updateDS()
    {
        SmartDashboard.putBoolean("Intake", isrunning);
    }
}