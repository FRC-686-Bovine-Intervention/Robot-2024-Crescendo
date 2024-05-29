package frc.robot;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import frc.robot.util.MappedSwitchableChooser;

public enum Environment {
    PRACTICE,
    COMPETITION,
    DEMO,
    ;
    public static Environment currentEnvironment = PRACTICE;
    private static final MappedSwitchableChooser<Environment> environmentChooser = new MappedSwitchableChooser<>("Environment Chooser");

    static {
        environmentChooser.setOptions(Arrays.stream(values()).collect(Collectors.toMap(Enum::name, (e) -> e)));
        environmentChooser.setDefault(PRACTICE);
    }
    
    public static void update() {
        environmentChooser.get().ifPresent((e) -> currentEnvironment = e);
        environmentChooser.setActive(currentEnvironment);
    }

    public static boolean is(Environment is) {
        return is.equals(currentEnvironment);
    }
    public static boolean isPractice() {
        return is(PRACTICE);
    }
    public static boolean isCompetition() {
        return is(COMPETITION);
    }
    public static boolean isDemo() {
        return is(DEMO);
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
