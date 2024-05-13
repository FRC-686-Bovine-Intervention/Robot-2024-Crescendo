package frc.robot.util;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

public enum SpectatorType {
	Comp(
		MatBuilder.fill(Nat.N2(), Nat.N2(),
			+0,+1,
			-1,+0
		)
	),
	Spectator(
		MatBuilder.fill(Nat.N2(), Nat.N2(),
			+1,+0,
			+0,+1
		)
	),
	ISpectator(
		MatBuilder.fill(Nat.N2(), Nat.N2(),
			-1,+0,
			+0,-1
		)
	),
	;
	private final Matrix<N2, N2> spectatorToField;
	private final Matrix<N2, N2> fieldToSpectator;
	private static final LoggedTunableNumber spectatorType = new LoggedTunableNumber("Spectator Type", 1);
	SpectatorType(Matrix<N2, N2> spectatorToField) {
		this.spectatorToField = spectatorToField;
		this.fieldToSpectator = this.spectatorToField.inv();
	}
	public Vector<N2> toField(Vector<N2> vec) {
		return new Vector<N2>(spectatorToField.times(vec));
	}
	public Vector<N2> toSpectator(Vector<N2> vec) {
		return new Vector<N2>(fieldToSpectator.times(vec));
	}
	
	public Vector<N2> getForwardFieldRel() {
		return toField(VecBuilder.fill(0, 1));
	}

	public static SpectatorType getCurrentType() {
		if(DriverStation.getMatchType() != MatchType.None) return Comp;
		return SpectatorType.values()[MathUtil.clamp((int) spectatorType.get(), 0, values().length - 1)];
	}
}
