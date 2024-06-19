package frc.robot.commands;

import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class WaitForAllLogged extends Command {
    private final Set<Map.Entry<String, BooleanSupplier>> conditions;

    public WaitForAllLogged(String name, Map<String, BooleanSupplier> conditions) {
        setName(name);
        this.conditions = conditions.entrySet();
    }

    @Override
    public boolean isFinished() {
        conditions.stream().map((entry) -> {
            Logger.recordOutput(getName() + "/" + entry.getKey(), entry.getValue().getAsBoolean());
            return entry;
        });
        return conditions.stream().allMatch((entry) -> entry.getValue().getAsBoolean());
    }

    public static class AllLogged implements BooleanSupplier {
        private final String name;
        private final Set<Map.Entry<String, BooleanSupplier>> conditions;

        public AllLogged(String name, Map<String, BooleanSupplier> conditions) {
            this.name = name;
            this.conditions = conditions.entrySet();
        }

        @Override
        public boolean getAsBoolean() {
            var all = true;
            for(var iter = conditions.iterator(); iter.hasNext(); ) {
                var ele = iter.next();
                var val = ele.getValue().getAsBoolean();
                Logger.recordOutput(name + "/" + ele.getKey(), val);
                if(!val) {
                    all = false;
                }
            }
            return all;
        }
    }
}
