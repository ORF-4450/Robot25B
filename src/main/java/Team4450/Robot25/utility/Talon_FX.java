package Team4450.Robot25.utility;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;

import Team4450.Lib.FXEncoder;
import Team4450.Lib.Util;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class Talon_FX extends TalonFX
{
    private double          gearRatio = 1.0;
    private DCMotorSim      motorSimModel;
    private FXEncoder       fxEncoder;

    public Talon_FX(int id, DCMotor dcMotor, double gearRatio)
    {
        super(id);

        this.gearRatio = gearRatio;

        // Reset TalonFX to default configuration.
        getConfigurator().apply(new TalonFXConfiguration());

        fxEncoder = new FXEncoder(this, 3);

        if (RobotBase.isSimulation()) initializeSim(dcMotor);
                       
        SendableRegistry.addLW(this, "Talon_FX", id);
    }

    private void initializeSim(DCMotor dcMotor)
    {
        motorSimModel = new DCMotorSim
                        (
                            LinearSystemId.createDCMotorSystem(
                                dcMotor, 
                                0.001, 
                                gearRatio
                            ),
                            DCMotor.getKrakenX60(1)
                        );
    }

    public void simulationPeriodic() 
    {
        var talonFXSim = getSimState();
        
        talonFXSim.Orientation = ChassisReference.Clockwise_Positive;
        
        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltage();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        motorSimModel.setInputVoltage(motorVoltage);
        motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(motorSimModel.getAngularPositionRotations() * gearRatio);
        talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(gearRatio));
    }

    public DoubleSupplier getPositionDS()
    {
        return () -> getPosition().getValueAsDouble();
    }

    public DoubleSupplier getVelocityDS()
    {
        return () -> getVelocity().getValueAsDouble();
    }

	@Override
	public void initSendable( SendableBuilder builder )
	{
		builder.setSmartDashboardType("Talon_FX");
    	//builder.addBooleanProperty(".controllable", () -> false, null);
	    builder.addDoubleProperty("amps", () -> this.motorSimModel.getCurrentDrawAmps(), null);
	    builder.addDoubleProperty("speed", this::get, null);
	    builder.addDoubleProperty("position (rot)", getPositionDS(), null);
	    builder.addDoubleProperty("velocity (rps)", getVelocityDS(), null);
	}
}
