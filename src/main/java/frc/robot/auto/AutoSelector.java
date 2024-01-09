package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.SwitchableChooser;
import frc.robot.util.VirtualSubsystem;

public class AutoSelector extends VirtualSubsystem {
    private final LoggedDashboardChooser<AutoRoutine> routineChooser;
    private final List<StringPublisher> questionPublishers;
    private final List<SwitchableChooser> responseChoosers;
    private final String key;
    
    private static final AutoRoutine defaultRoutine = new AutoRoutine("Do Nothing", List.of(), ()->Commands.none());
    private final String questionPlaceHolder = "NA"; 

    private AutoRoutine lastRoutine;

    public AutoSelector(String key) {
        this.key = key;
        routineChooser = new LoggedDashboardChooser<>(key + "/Routine");
        routineChooser.addDefaultOption(defaultRoutine.name, defaultRoutine);
        lastRoutine = defaultRoutine;
        questionPublishers = new ArrayList<>();
        responseChoosers = new ArrayList<>();
    }

    public void addRoutine(AutoRoutine routine) {
        for(int i = questionPublishers.size(); i < routine.questions.size(); i++) {
            var publisher =
                NetworkTableInstance.getDefault()
                    .getStringTopic("/SmartDashboard/" + key + "/Question #" + Integer.toString(i + 1))
                    .publish();
            publisher.set(questionPlaceHolder);
            questionPublishers.add(publisher);
            responseChoosers.add(new SwitchableChooser(key + "/Question #" + Integer.toString(i + 1) + " Chooser"));
        }
        routineChooser.addOption(routine.name, routine);
    }

    @Override
    public void periodic() {
        if(DriverStation.isEnabled()) return;
        var selectedRoutine = routineChooser.get();
        if(selectedRoutine == null) return;
        var questions = selectedRoutine.questions;
        List<String> currentResponses = new ArrayList<>();
        for (int i = 0; i < responseChoosers.size(); i++) {
            if(i < questions.size()) {
                questionPublishers.get(i).set(questions.get(i).name);
                responseChoosers.get(i).setOptions(questions.get(i).getOptionNames());
                currentResponses.add(responseChoosers.get(i).get());
                questions.get(i).setResponse(responseChoosers.get(i).get());
            } else {
                questionPublishers.get(i).set(questionPlaceHolder);
                responseChoosers.get(i).setOptions(new String[] {});
            }
        }
        lastRoutine = selectedRoutine;
    }

    public AutoRoutine getSelectedRoutine() {
        return lastRoutine;
    }

    public Command getSelectedAutoCommand() {
        return lastRoutine.autoCommandGenerator.get();
    }

    public static class AutoQuestion<T extends Enum<T>> {
        public final String name;
        private final Supplier<T[]> optionSupplier;
        private T response;

        public AutoQuestion(String name, Supplier<T[]> optionSupplier) {
            this.name = name;
            this.optionSupplier = optionSupplier;
            this.response = this.optionSupplier.get()[0];
        }

        public T getResponse() {
            return response;
        }

        public void setResponse(String newResponse) {
            response = Enum.valueOf(response.getDeclaringClass(), newResponse);
        }

        public String[] getOptionNames() {
            var options = optionSupplier.get();
            var optionNames = new String[options.length];
            for(int i = 0; i < optionNames.length; i++) {
                optionNames[i] = options[i].name();
            }
            return optionNames;
        }
    }

    public static class AutoRoutine {
        public final String name;
        public final List<AutoQuestion<?>> questions;
        public final Supplier<? extends Command> autoCommandGenerator;

        public AutoRoutine(String name, List<AutoQuestion<?>> questions, Supplier<? extends Command> autoCommandGenerator) {
            this.name = name;
            this.questions = questions;
            this.autoCommandGenerator = autoCommandGenerator;
        }
    }
}
