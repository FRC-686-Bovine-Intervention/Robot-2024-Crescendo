package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class PrintFormattedCommand extends InstantCommand {
    @FunctionalInterface
    public static interface ObjectSupplier extends Supplier<Object> {}

    public PrintFormattedCommand(String formattedMessage, ObjectSupplier... objectSupplier) {
        super(() -> {
            Object[] params = new Object[objectSupplier.length];
            for (int i = 0; i < objectSupplier.length; i++) {
                params[i] = objectSupplier[i].get();
            }
            System.out.printf(formattedMessage, params);
        });
    }
}
      
