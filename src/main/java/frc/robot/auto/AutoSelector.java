package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.SwitchableChooser;
import frc.robot.util.VirtualSubsystem;

public class AutoSelector extends VirtualSubsystem {
    private final LoggedDashboardChooser<AutoRoutine> routineChooser;
    private final List<StringPublisher> questionPublishers;
    private final List<SwitchableChooser> responseChoosers;
    private final String key;
    
    private static final AutoRoutine defaultRoutine = new AutoRoutine("Do Nothing", List.of()) {
        public Command generateCommand() {
            return Commands.none();
        }
    };
    private final String questionPlaceHolder = "NA"; 

    private Command lastCommand;
    private List<String> lastResponses;

    public AutoSelector(String key) {
        this.key = key;
        routineChooser = new LoggedDashboardChooser<>(key + "/Routine");
        routineChooser.addDefaultOption(defaultRoutine.name, defaultRoutine);
        questionPublishers = new ArrayList<>();
        responseChoosers = new ArrayList<>();
    }

    private void populateQuestions(AutoRoutine routine) {
        for(int i = questionPublishers.size(); i < routine.questions.size(); i++) {
            var publisher =
                NetworkTableInstance.getDefault()
                    .getStringTopic("/SmartDashboard/" + key + "/Question #" + Integer.toString(i + 1))
                    .publish();
            publisher.set(questionPlaceHolder);
            questionPublishers.add(publisher);
            responseChoosers.add(new SwitchableChooser(key + "/Question #" + Integer.toString(i + 1) + " Chooser"));
        }
    }

    public void addRoutine(AutoRoutine routine) {
        populateQuestions(routine);
        routineChooser.addOption(routine.name, routine);
    }

    public void addDefaultRoutine(AutoRoutine routine) {
        populateQuestions(routine);
        routineChooser.addDefaultOption(routine.name, routine);
    }

    private Alliance prevAlliance = Alliance.Blue;
    @Override
    public void periodic() {
        var alliance = DriverStation.getAlliance().orElse(null);
        if(DriverStation.isEnabled()) return;
        var selectedRoutine = routineChooser.get();
        if(selectedRoutine == null) return;
        var questions = selectedRoutine.questions;
        List<String> currentResponses = new ArrayList<>();
        for (int i = 0; i < responseChoosers.size(); i++) {
            if(i < questions.size()) {
                questionPublishers.get(i).set(questions.get(i).name);
                responseChoosers.get(i).setOptions(questions.get(i).getOptionNames());
                var response = responseChoosers.get(i).get();
                currentResponses.add(response.orElse(SwitchableChooser.placeholder));
                response.ifPresent(questions.get(i)::setResponse);
            } else {
                questionPublishers.get(i).set(questionPlaceHolder);
                responseChoosers.get(i).setOptions(new String[] {});
            }
        }
        if(!currentResponses.equals(lastResponses) || prevAlliance != alliance) {
            System.out.println("[AutoSelector] Generating new command");
            System.out.println("[AutoSelector] Routine: " + selectedRoutine.name);
            currentResponses.forEach(System.out::println);
            lastCommand = selectedRoutine.generateCommand().withName("AUTO " + selectedRoutine.name);
        }
        prevAlliance = alliance;
        lastResponses = currentResponses;
    }

    public Command getSelectedAutoCommand() {
        return lastCommand;
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

    public static abstract class AutoRoutine {
        public final String name;
        public final List<AutoQuestion<?>> questions;

        public AutoRoutine(String name, List<AutoQuestion<?>> questions) {
            this.name = name;
            this.questions = questions;
        }

        public abstract Command generateCommand();
    }
}
