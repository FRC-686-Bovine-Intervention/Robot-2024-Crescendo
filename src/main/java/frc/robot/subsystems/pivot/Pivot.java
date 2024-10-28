// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AimingParameters;
import frc.robot.NoteVisualizer;
import frc.robot.util.Cooldown;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.SuppliedEdgeDetector;

public class Pivot extends SubsystemBase {
    private final PivotIO pivotIO;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public static final LoggedTunableMeasure<Angle> idleAltitude = new LoggedTunableMeasure<>("Pivot/Angles/Zero", Degrees.of(0));
    public static final LoggedTunableMeasure<Angle> ampAltitude = new LoggedTunableMeasure<>("Pivot/Angles/Amp", Degrees.of(100));
    public static final LoggedTunableMeasure<Angle> superPassAltitude = new LoggedTunableMeasure<>("Pivot/Angles/Super Pass", Degrees.of(50));
    public static final LoggedTunableMeasure<Angle> customIncrementAngle = new LoggedTunableMeasure<>("Pivot/Angles/Custom Increment", Degrees.of(0.5));
    
    public static final LoggedTunableMeasure<Voltage> recalVoltage = new LoggedTunableMeasure<>("Pivot/Volts/Recal", Volts.of(1));

    public final Trigger atPos = new Trigger(() -> AimingParameters.withinAltitudeTolerance(Radians.of(inputs.pivotEncoder.positionRad)));

    public static final Translation3d robotToPivotTranslation = 
        new Translation3d(
            Inches.of(-7.5),
            Inches.of(0),
            Inches.of(19.01)
        )
    ;

    private final SuppliedEdgeDetector increaseEdgeDetector;
    private final SuppliedEdgeDetector decreaseEdgeDetector;

    private final MutableMeasure<Angle> runtimeOffset = MutableMeasure.zero(Degrees);

    public Pivot(PivotIO pivotIO, BooleanSupplier increaseRuntimeOffset, BooleanSupplier decreaseRuntimeOffset) {
        System.out.println("[Init Pivot] Instantiating Pivot");
        this.pivotIO = pivotIO;
        System.out.println("[Init Pivot] Pivot IO: " + this.pivotIO.getClass().getSimpleName());
        SmartDashboard.putData("Subsystems/Pivot", this);
        this.increaseEdgeDetector = new SuppliedEdgeDetector(increaseRuntimeOffset);
        this.decreaseEdgeDetector = new SuppliedEdgeDetector(decreaseRuntimeOffset);
    }

    @Override
    public void periodic() {
        pivotIO.updateInputs(inputs);
        Logger.processInputs("Inputs/Pivot", inputs);
        Logger.recordOutput("Mechanism3d/Shooter", getRobotToPivot());
        NoteVisualizer.robotToPivot = getRobotToPivot();
        increaseEdgeDetector.update();
        decreaseEdgeDetector.update();
        if(increaseEdgeDetector.risingEdge()) {
            runtimeOffset.mut_acc(0.5);
        }
        if(decreaseEdgeDetector.risingEdge()) {
            runtimeOffset.mut_acc(-0.5);
        }

        if(increaseEdgeDetector.risingEdge() || decreaseEdgeDetector.risingEdge()) {
            pivotIO.setRotorOffset(runtimeOffset.in(Radians));
        }

        Logger.recordOutput("Pivot/Runtime Offset", runtimeOffset);
    }

    public static Transform3d getRobotToPivot(double angle) {
        return new Transform3d(
            robotToPivotTranslation,
            new Rotation3d(
                0,
                idleAltitude.in(Radians)-angle,
                0
            )
        );
    }

    public Transform3d getRobotToPivot() {
        return getRobotToPivot(inputs.pivotEncoder.positionRad);
    }

    public Command coast() {
        var subsystem = this;
        return new Command() {
            {
                setName("Coast");
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                pivotIO.setCoast(true);
            }

            @Override
            public void end(boolean interrupted) {
                pivotIO.setCoast(false);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }

    private Command genCommand(String name, Supplier<Measure<Angle>> angleSupplier) {
        var subsystem = this;
        return new Command() {
            {
                setName(name);
                addRequirements(subsystem);
            }

            @Override
            public void execute() {
                pivotIO.setPivotPos(angleSupplier.get().in(Radians));
            }
        };
    }

    public Command customIncremented(BooleanSupplier increase, BooleanSupplier decrease) {
        return genCommand(
            "Custom",
            new Supplier<Measure<Angle>>() {
                private final MutableMeasure<Angle> angle = MutableMeasure.zero(Degrees);
                private final Cooldown cooldown = new Cooldown();
                public Measure<Angle> get() {
                    Logger.recordOutput("Custom Shoot/Pivot Angle", angle);
                    if(!cooldown.hasExpired()) {
                        return angle;
                    }
                    if(increase.getAsBoolean()) {
                        cooldown.reset(0.125);
                        angle.mut_acc(customIncrementAngle.get());
                    }
                    if(decrease.getAsBoolean() && angle.gt(Degrees.zero())) {
                        cooldown.reset(0.125);
                        angle.mut_minus(customIncrementAngle.get());
                    }

                    return angle;
                }
            }
        );
    }

    public Command idle() {
        return genCommand(
            "Idle",
            idleAltitude
        );
    }
    public Command aim() {
        return genCommand(
            "Aim",
            AimingParameters::pivotAltitude
        );
    }
    public Command amp() {
        return genCommand(
            "Amp",
            ampAltitude
        );
    }
    public Command superPass() {
        return genCommand(
            "Super Pass",
            superPassAltitude
        );
    }

    public Command recal() {
      var subsystem = this;
      return new Command() {
        {
          addRequirements(subsystem);
          setName("Recal");
        }
        @Override
        public void execute() {
          pivotIO.setPivotVoltage(-recalVoltage.in(Volts));
        }
        @Override
        public boolean isFinished() {
            return inputs.leftLimitSwitch || inputs.rightLimitSwitch; 
        }
        @Override
        public void end(boolean interrupted) {
          pivotIO.stop();
        }
      };
    }
}
