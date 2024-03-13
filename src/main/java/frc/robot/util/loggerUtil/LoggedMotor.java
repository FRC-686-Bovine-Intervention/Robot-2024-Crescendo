package frc.robot.util.loggerUtil;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class LoggedMotor implements StructSerializable {
    public double appliedVolts = Double.NaN;
    public double currentAmps = Double.NaN;
    public double tempCelsius = Double.NaN;

    public void updateFrom(TalonFX talon) {
        this.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
        this.currentAmps = talon.getStatorCurrent().getValueAsDouble();
        this.tempCelsius = talon.getDeviceTemp().getValueAsDouble();
    }

    public void updateFrom(CANSparkMax spark) {
        this.appliedVolts = spark.getAppliedOutput() * 12;
        this.currentAmps = spark.getOutputCurrent();
    }

    public void updateFrom(DCMotorSim sim, double appliedVolts) {
        this.currentAmps = sim.getCurrentDrawAmps();
        this.appliedVolts = appliedVolts;
    }

    public void updateFrom(FlywheelSim sim, double appliedVolts) {
        this.currentAmps = sim.getCurrentDrawAmps();
        this.appliedVolts = appliedVolts;
    }

    public void updateFrom(SingleJointedArmSim sim, double appliedVolts) {
        this.currentAmps = sim.getCurrentDrawAmps();
        this.appliedVolts = appliedVolts;
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
            return kSizeDouble * 3;
        }

        @Override
        public String getSchema() {
            return "double AppliedVolts;double CurrentAmps;double TempCelsius";
        }

        @Override
        public LoggedMotor unpack(ByteBuffer bb) {
            var motor = new LoggedMotor();
            motor.appliedVolts = bb.getDouble();
            motor.currentAmps = bb.getDouble();
            motor.tempCelsius = bb.getDouble();
            return motor;
        }

        @Override
        public void pack(ByteBuffer bb, LoggedMotor value) {
            bb.putDouble(value.appliedVolts);
            bb.putDouble(value.currentAmps);
            bb.putDouble(value.tempCelsius);
        }
    }
}
