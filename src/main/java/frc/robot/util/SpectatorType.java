package frc.robot.util;

import java.util.Arrays;
import java.util.stream.Collectors;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import frc.robot.Environment;

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
	InvSpectator(
		MatBuilder.fill(Nat.N2(), Nat.N2(),
			-1,+0,
			+0,-1
		)
	),
	;
	private static final MappedSwitchableChooser<SpectatorType> chooser = new MappedSwitchableChooser<>("Spectator Type");
	static{
		chooser.setOptions(Arrays.stream(values()).collect(Collectors.toMap(Enum::name, (e) -> e)));
		chooser.setDefault(Comp);
	}

	private final Matrix<N2, N2> spectatorToField;
	private final Matrix<N2, N2> fieldToSpectator;
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
		var selected = Environment.isCompetition() ? Comp : chooser.get().orElse(Spectator);
		chooser.setActive(selected);
		return selected;
	}
}
