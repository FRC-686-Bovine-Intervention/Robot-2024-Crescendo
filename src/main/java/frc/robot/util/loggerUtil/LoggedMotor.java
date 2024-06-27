package frc.robot.util.loggerUtil;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class LoggedMotor implements StructSerializable {
    public final MutableMeasure<Angle> position = MutableMeasure.zero(Radians);
    public final MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RadiansPerSecond);
    public final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.zero(Volts);
    public final MutableMeasure<Current> current = MutableMeasure.zero(Amps);
    public final MutableMeasure<Temperature> temperature = MutableMeasure.zero(Celsius);

    public void updateFrom(TalonFX talon) {
        this.position.mut_replace(talon.getPosition().getValueAsDouble(), Rotations);
        this.velocity.mut_replace(talon.getVelocity().getValueAsDouble(), RotationsPerSecond);
        this.appliedVoltage.mut_replace(talon.getMotorVoltage().getValueAsDouble(), Volts);
        this.current.mut_replace(talon.getStatorCurrent().getValueAsDouble(), Amps);
        this.temperature.mut_replace(talon.getDeviceTemp().getValueAsDouble(), Celsius);
    }

    public void updateFrom(CANSparkMax spark) {
        this.position.mut_replace(spark.getEncoder().getPosition(), Rotations);
        this.velocity.mut_replace(spark.getEncoder().getVelocity(), RPM);
        this.appliedVoltage.mut_replace(spark.getAppliedOutput() * 12, Volts);
        this.current.mut_replace(spark.getOutputCurrent(), Amps);
    }

    public void updateFrom(DCMotorSim sim) {
        this.position.mut_replace(sim.getAngularPositionRad(), Radians);
        this.velocity.mut_replace(sim.getAngularVelocityRadPerSec(), RadiansPerSecond);
        this.current.mut_replace(sim.getCurrentDrawAmps(), Amps);
    }
    public void updateFrom(DCMotorSim sim, double appliedVolts) {
        updateFrom(sim);
        this.appliedVoltage.mut_replace(appliedVolts, Volts);
    }

    public void updateFrom(FlywheelSim sim) {
        this.position.mut_acc(sim.getAngularVelocityRadPerSec() * Constants.dtSeconds);
        this.velocity.mut_replace(sim.getAngularVelocityRadPerSec(), RadiansPerSecond);
        this.current.mut_replace(sim.getCurrentDrawAmps(), Amps);
    }
    public void updateFrom(FlywheelSim sim, double appliedVolts) {
        updateFrom(sim);
        this.appliedVoltage.mut_replace(appliedVolts, Volts);
    }

    public void updateFrom(SingleJointedArmSim sim) {
        this.position.mut_replace(sim.getAngleRads(), Radians);
        this.velocity.mut_replace(sim.getVelocityRadPerSec(), RadiansPerSecond);
        this.current.mut_replace(sim.getCurrentDrawAmps(), Amps);
    }
    public void updateFrom(SingleJointedArmSim sim, double appliedVolts) {
        updateFrom(sim);
        this.appliedVoltage.mut_replace(appliedVolts, Volts);
    }

    public static final LoggedMotorStruct struct = new LoggedMotorStruct();

    public static class LoggedMotorStruct implements Struct<LoggedMotor> {
        @Override
        public Class<LoggedMotor> getTypeClass() {
            return LoggedMotor.class;
        }

        @Override
        public String getTypeString() {
            return "struct:Motor";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 5;
        }

        @Override
        public String getSchema() {
            return "double PositionRad;double VelocityRadPerSec;double AppliedVolts;double CurrentAmps;double TempCelsius";
        }

        @Override
        public LoggedMotor unpack(ByteBuffer bb) {
            var motor = new LoggedMotor();
            motor.position.mut_setBaseUnitMagnitude(bb.getDouble());
            motor.velocity.mut_setBaseUnitMagnitude(bb.getDouble());
            motor.appliedVoltage.mut_setBaseUnitMagnitude(bb.getDouble());
            motor.current.mut_setBaseUnitMagnitude(bb.getDouble());
            motor.temperature.mut_setBaseUnitMagnitude(bb.getDouble());
            return motor;
        }

        @Override
        public void pack(ByteBuffer bb, LoggedMotor value) {
            bb.putDouble(value.position.baseUnitMagnitude());
            bb.putDouble(value.velocity.baseUnitMagnitude());
            bb.putDouble(value.appliedVoltage.baseUnitMagnitude());
            bb.putDouble(value.current.baseUnitMagnitude());
            bb.putDouble(value.temperature.baseUnitMagnitude());
        }
    }
}
