package frc.robot.subsystems.drive;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class ModuleIOFalcon550 implements ModuleIO {
    private final TalonFX driveMotor;
    private final CANSparkMax turnMotor;
    private final AbsoluteEncoder turnAbsoluteEncoder;
    private final double initialOffsetRadians;

    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveSupplyCurrent;
    private final StatusSignal<Double> driveTemperature;

    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    public ModuleIOFalcon550(DriveModulePosition position) {
        driveMotor = new TalonFX(position.driveMotorID, CANDevices.driveCanBusName);
        turnMotor = new CANSparkMax(position.turnMotorID, MotorType.kBrushless);
        turnAbsoluteEncoder = turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        initialOffsetRadians = Units.rotationsToRadians(position.cancoderOffsetRotations);

        /** Configure Drive Motors */
        var driveConfig = new TalonFXConfiguration();
        // change factory defaults here
        driveConfig.MotorOutput.Inverted = position.driveInverted;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0;
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1875;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 60;
        driveConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        driveConfig.CurrentLimits.SupplyTimeThreshold = 0;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotor.getConfigurator().apply(driveConfig);

        /** Configure Turn Motors */
        turnMotor.setInverted(false);
        turnMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setSmartCurrentLimit(40);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int) (1000.0 / DriveConstants.odometryFrequency));

        setFramePeriods(driveMotor, true);

        zeroEncoders();

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();
        driveTemperature = driveMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(DriveConstants.odometryFrequency, drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            driveVelocity,
            driveAppliedVolts,
            driveSupplyCurrent,
            driveTemperature,
            driveMotor.getFault_DeviceTemp()
        );
        driveMotor.optimizeBusUtilization();

        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor, drivePosition);
        turnPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(() -> MathUtil.angleModulus(Units.rotationsToRadians(turnAbsoluteEncoder.getPosition())) - initialOffsetRadians);

        tempWarning = new Alert("Drive Temp", position.name() + " Module has exceeded 70C", AlertType.WARNING);
        tempAlert = new Alert("Drive Temp", position.name() + " Module has exceeded 100C", AlertType.ERROR);
    }

    private final Alert tempWarning;
    private final Alert tempAlert;

    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveMotor.positionRad =       Units.rotationsToRadians(driveMotor.getPosition().getValue()) / DriveConstants.driveWheelGearReduction;
        inputs.driveMotor.velocityRadPerSec = Units.rotationsToRadians(driveMotor.getVelocity().getValue()) / DriveConstants.driveWheelGearReduction;
        inputs.driveMotor.appliedVolts =      driveMotor.getMotorVoltage().getValue();
        inputs.driveMotor.currentAmps =       driveMotor.getSupplyCurrent().getValue();
        inputs.driveMotor.tempCelsius =       driveMotor.getDeviceTemp().getValue();

        inputs.turnMotor.positionRad =        MathUtil.angleModulus(Units.rotationsToRadians(turnAbsoluteEncoder.getPosition())) - initialOffsetRadians;
        inputs.turnMotor.velocityRadPerSec =  Units.rotationsToRadians(turnAbsoluteEncoder.getVelocity());
        inputs.turnMotor.appliedVolts =       turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.turnMotor.currentAmps =        turnMotor.getOutputCurrent();

        inputs.odometryDrivePositionsMeters = 
            drivePositionQueue
            .stream()
            .mapToDouble(
                (p) -> Units.rotationsToRadians(p) / DriveConstants.driveWheelGearReduction * DriveConstants.wheelRadiusMeters
            )
            .toArray()
        ;
        inputs.odometryTurnPositions = 
            turnPositionQueue
            .stream()
            .map(Rotation2d::fromRadians)
            .toArray(Rotation2d[]::new)
        ;
        drivePositionQueue.clear();
        turnPositionQueue.clear();

        tempWarning.set(inputs.driveMotor.tempCelsius > 70);
        tempAlert.set(driveMotor.getFault_DeviceTemp().getValue());
    }

    public void zeroEncoders() {
        driveMotor.setPosition(0.0);
        // turnRelativeEncoder.setPosition(turnAbsoluteEncoder.getPosition());
    }

    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    public void setTurnVoltage(double volts) {
        turnMotor.setVoltage(volts);
    }

    private static void setFramePeriods(TalonFX talon, boolean needMotorSensor) {
        // reduce rates of most status frames

        // TODO: revisit figuring out what getters to slow down
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 1000);
        // if (!needMotorSensor) {
        //    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 1000);
        // }
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255, 1000);

        talon.getPosition().setUpdateFrequency(Constants.loopFrequencyHz);
    }

    @Override
    public void setDriveBrakeMode(Boolean enable) {
        driveMotor.setControl(enable == null ? new NeutralOut() : (enable.booleanValue() ? new StaticBrake() : new CoastOut()));
    }

    @Override
    public void setTurnBrakeMode(Boolean enable) {
        if(enable == null) return;
        turnMotor.setIdleMode(enable.booleanValue() ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void stop() {
        var driveRequest = driveMotor.getAppliedControl();
        if(driveRequest instanceof VoltageOut) {
            driveMotor.setControl(new NeutralOut());
        }
        setTurnVoltage(0);
    }
}
