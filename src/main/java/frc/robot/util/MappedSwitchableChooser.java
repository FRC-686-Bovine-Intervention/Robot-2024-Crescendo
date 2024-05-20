package frc.robot.util;

import java.util.Arrays;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;

/** A string chooser for the dashboard where the options can be changed on-the-fly. */
public class MappedSwitchableChooser<T> implements LoggedDashboardInput, LazyOptional<T> {
  // public static final String placeholder = "<NA>";

  // private Map<String, T> options;
  // private Optional<T> active = Optional.empty();
  // private Optional<T> defaultO = Optional.empty();

  // private final StringPublisher namePublisher;
  // private final StringPublisher typePublisher;
  // private final StringArrayPublisher optionsPublisher;
  // private final StringPublisher defaultPublisher;
  // private final StringPublisher activePublisher;
  // private final StringPublisher selectedPublisher;
  // private final LoggedDashboardString selectedInput;

  // public MappedSwitchableChooser(String name) {
  //   var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(name);
  //   namePublisher = table.getStringTopic(".name").publish();
  //   typePublisher = table.getStringTopic(".type").publish();
  //   optionsPublisher = table.getStringArrayTopic("options").publish();
  //   defaultPublisher = table.getStringTopic("default").publish();
  //   activePublisher = table.getStringTopic("active").publish();
  //   selectedPublisher = table.getStringTopic("selected").publish();
  //   selectedInput = new LoggedDashboardString(name + "/selected");
  //   Logger.registerDashboardInput(this);

  //   namePublisher.set(name);
  //   typePublisher.set("String Chooser");
  //   setOptions(Map.of());
  // }

  // /** Updates the set of available options. */
  // public void setOptions(Map<String, T> options) {
  //   if(options.equals(this.options)) return;
  //   this.options = options;
  //   optionsPublisher.set(this.options.size() == 0 ? new String[]{placeholder} : this.options.keySet().toArray(String[]::new));
  //   setActive(Optional.empty());
  // }

  // public void setDefault(Optional<String> defaultOption) {
  //   if (this.defaultO.equals(defaultOption)) return;
  //   this.defaultO = defaultOption;
  //   defaultPublisher.set(defaultOption.orElse(placeholder));
  //   setActive(defaultOption);
  // }

  // public Map<String, T> getOptions() {
  //   return options;
  // }

  /** Returns the selected option. */
  @Override
  public Optional<T> get() {
    return null;
  //   return active;
  }

  // public void setActive(Optional<String> newActive) {
  //   active = newActive;
  //   selectedPublisher.set(active.orElse(placeholder));
  //   activePublisher.set(active.orElse(placeholder));
  // }

  public void periodic() {
  //   String selected = selectedInput.get();
  //   if (Arrays.stream(options).anyMatch(selected::equals)) {
  //     setActive(Optional.ofNullable(selected));
  //   }
  }
}
