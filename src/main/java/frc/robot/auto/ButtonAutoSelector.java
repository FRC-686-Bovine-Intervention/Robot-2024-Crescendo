package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.util.VirtualSubsystem;

public class ButtonAutoSelector extends VirtualSubsystem {
    private final ArrayList<AutoRoutine> routines = new ArrayList<>();
    private final DigitalInput button = new DigitalInput(2);
    private boolean lastButton = false;
    
    private int selectedAutoIndex = 0;

    public ButtonAutoSelector() {
        addRoutine(new AutoRoutine("Do Nothing", List.of(), ()->Commands.none()));
    }

    public void addRoutine(AutoRoutine routine) {
        routines.add(routine);
    }

    public Command getSelectedAutoCommand() {
        return routines.get(selectedAutoIndex).autoCommandGenerator.get();
    }

    private AutoRoutine getSelectedAuto() {
        return routines.get(selectedAutoIndex);
    }

    @Override
    public void periodic() {
        var butVal = !button.get();

        if(butVal && !lastButton) {
            var incrementAuto = true;
            for(int questionIndex = 0; questionIndex < getSelectedAuto().questions.size(); questionIndex++) {
                var question = getSelectedAuto().questions.get(questionIndex);
                var responseIndex = 0;
                while(responseIndex < question.getOptionNames().length) {
                    if(question.getOptionNames()[responseIndex].equals(question.getResponse().name())) break;
                    responseIndex++;
                }
                responseIndex++;
                if(responseIndex < question.getOptionNames().length) {
                    question.setResponse(question.getOptionNames()[responseIndex]);
                    incrementAuto = false;
                    break;
                } else {
                    responseIndex %= question.getOptionNames().length;
                    question.setResponse(question.getOptionNames()[responseIndex]);
                }
            }
            if(incrementAuto) {
                selectedAutoIndex = (selectedAutoIndex + 1) % routines.size();
            }
            System.out.println("-------------------");
            System.out.println("Selected Auto: " + getSelectedAuto().name);
            for(var question : getSelectedAuto().questions) {
                System.out.println(question.name + ": " + question.getResponse().name());
            }
        }

        lastButton = butVal;
    }

    private static final Color[] ordinalColors = new Color[] {
        Color.kYellow,
        Color.kGreen,
        Color.kBlue,
    };

    public Color[] getLEDColors() {
        var ret = new Color[] {
            Color.kRed,
            Color.kRed,
            Color.kRed,
            Color.kRed,
            Color.kRed,
        };
        if(selectedAutoIndex > 0) {
            ret[0] = ordinalColors[selectedAutoIndex - 1];
        }
        for(int i = 0; i < getSelectedAuto().questions.size(); i++) {
            ret[i + 1] = ordinalColors[getSelectedAuto().questions.get(i).getResponse().ordinal()];
        }
        return ret;
    }
}
