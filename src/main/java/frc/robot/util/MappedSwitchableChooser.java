package frc.robot.util;

import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;

/** A string chooser for the dashboard where the options can be changed on-the-fly. */
public class MappedSwitchableChooser<T> implements LoggedDashboardInput, LazyOptional<T> {
  public static final String placeholder = "<NA>";

  private Map<String, T> options;
  private Optional<T> selectedOption = Optional.empty();
  private Optional<T> activeOption = Optional.empty();
  private Optional<T> defaultOption = Optional.empty();

  private final StringPublisher namePublisher;
  private final StringPublisher typePublisher;
  private final StringArrayPublisher optionsPublisher;
  private final StringPublisher defaultPublisher;
  private final StringPublisher activePublisher;
  private final StringPublisher selectedPublisher;
  private final LoggedDashboardString selectedInput;

  public MappedSwitchableChooser(String name) {
    var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(name);
    namePublisher = table.getStringTopic(".name").publish();
    typePublisher = table.getStringTopic(".type").publish();
    optionsPublisher = table.getStringArrayTopic("options").publish();
    defaultPublisher = table.getStringTopic("default").publish();
    activePublisher = table.getStringTopic("active").publish();
    selectedPublisher = table.getStringTopic("selected").publish();
    selectedInput = new LoggedDashboardString(name + "/selected");
    Logger.registerDashboardInput(this);

    namePublisher.set(name);
    typePublisher.set("String Chooser");
    setOptions(Map.of());
  }

  @Override
  public void periodic() {
    var selected = selectedInput.get();
    selectedOption = Optional.ofNullable(placeholder.equals(selected) ? null : options.get(selected));
  }

  /** Updates the set of available options. */
  public void setOptions(Map<String, T> options) {
    if(options.equals(this.options)) return;
    this.options = options;
    optionsPublisher.set(this.options.size() == 0 ? new String[]{placeholder} : this.options.keySet().toArray(String[]::new));
  }

  public void setSelected(T selectedValue) {
    if (this.selectedOption.equals(selectedValue)) return;
    this.selectedOption = Optional.ofNullable(selectedValue);
    selectedPublisher.set(this.selectedOption.map(this::getKey).orElse(placeholder));
  }

  public void setActive(T activeValue) {
    if (this.activeOption.equals(activeValue)) return;
    this.activeOption = Optional.ofNullable(activeValue);
    activePublisher.set(this.activeOption.map(this::getKey).orElse(placeholder));
  }

  public void setDefault(T defaultValue) {
    if (this.defaultOption.equals(defaultValue)) return;
    this.defaultOption = Optional.ofNullable(defaultValue);
    defaultPublisher.set(this.defaultOption.map(this::getKey).orElse(placeholder));
  }

  public Map<String, T> getOptions() {
    return options;
  }

  /** Returns the selected option. */
  @Override
  public Optional<T> get() {
    return selectedOption;
  }

  public Optional<T> getActive() {
    return activeOption;
  }

  public Optional<T> getDefault() {
    return defaultOption;
  }

  private String getKey(T value) {
    return this.options.entrySet().stream().filter((e) -> e.getValue().equals(value)).map(Entry::getKey).findAny().orElse(placeholder);
  }
}
