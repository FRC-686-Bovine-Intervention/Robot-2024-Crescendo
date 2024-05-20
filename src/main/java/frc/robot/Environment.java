package frc.robot;

import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import frc.robot.util.SwitchableChooser;

public enum Environment {
    PRACTICE,
    COMPETITION,
    DEMO,
    ;
    public static Environment currentEnvironment;
    // public static final Map<String, Environment> nameMap = Arrays.stream(values()).collect(Collectors.toMap(Enum::name, (e) -> e));
    private static final SwitchableChooser environmentChooser = new SwitchableChooser("Environment Chooser");

    static {
        environmentChooser.setOptions(Arrays.stream(values()).map(Enum::name).toArray(String[]::new));
        environmentChooser.setDefault(Optional.of(PRACTICE.name()));
    }
    
    public static void update() {
        currentEnvironment = valueOf(null);
    }

    public static DoubleSupplier switchVar(DoubleSupplier prac_comp, DoubleSupplier demo) {
        return switchVar(prac_comp, prac_comp, demo);
    }
    public static DoubleSupplier switchVar(DoubleSupplier prac, DoubleSupplier comp, DoubleSupplier demo) {
        return () -> switch(currentEnvironment) {
            default -> prac.getAsDouble();
            case COMPETITION -> comp.getAsDouble();
            case DEMO -> demo.getAsDouble();
        };
    }
    public static <T> Supplier<T> switchVar(Supplier<T> prac_comp, Supplier<T> demo) {
        return switchVar(prac_comp, prac_comp, demo);
    }
    public static <T> Supplier<T> switchVar(Supplier<T> prac, Supplier<T> comp, Supplier<T> demo) {
        return () -> switch(currentEnvironment) {
            default -> prac.get();
            case COMPETITION -> comp.get();
            case DEMO -> demo.get();
        };
    }
}
