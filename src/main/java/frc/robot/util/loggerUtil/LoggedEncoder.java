package frc.robot.util.loggerUtil;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class LoggedEncoder implements StructSerializable {
    public final MutableMeasure<Angle> position = MutableMeasure.zero(Radians);
    public final MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RadiansPerSecond);

    public void updateFrom(TalonFX talon) {
        this.position.mut_replace(talon.getPosition().getValueAsDouble(), Rotations);
        this.velocity.mut_replace(talon.getVelocity().getValueAsDouble(), RotationsPerSecond);
    }
    public void updateFrom(CANcoder canCoder) {
        this.position.mut_replace(canCoder.getPosition().getValueAsDouble(), Rotations);
        this.velocity.mut_replace(canCoder.getVelocity().getValueAsDouble(), RotationsPerSecond);
    }

    public void updateFrom(SingleJointedArmSim sim) {
        this.position.mut_replace(sim.getAngleRads(), Radians);
        this.velocity.mut_replace(sim.getVelocityRadPerSec(), RadiansPerSecond);
    }

    public static final LoggedEncoderStruct struct = new LoggedEncoderStruct();
    
    public static class LoggedEncoderStruct implements Struct<LoggedEncoder> {
        @Override
        public Class<LoggedEncoder> getTypeClass() {
            return LoggedEncoder.class;
        }

        @Override
        public String getTypeString() {
            return "struct:Encoder";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 2;
        }

        @Override
        public String getSchema() {
            return "double PositionRad;double VelocityRadPerSec";
        }

        @Override
        public LoggedEncoder unpack(ByteBuffer bb) {
            var encoder = new LoggedEncoder();
            encoder.position.mut_setBaseUnitMagnitude(bb.getDouble());
            encoder.velocity.mut_setBaseUnitMagnitude(bb.getDouble());
            return encoder;
        }

        @Override
        public void pack(ByteBuffer bb, LoggedEncoder value) {
            bb.putDouble(value.position.baseUnitMagnitude());
            bb.putDouble(value.velocity.baseUnitMagnitude());
        }
    }
}
