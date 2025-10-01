package org.firstinspires.ftc.teamcode.Mode.raihancode;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;

public class ArcadeDriveCommand extends CommandBase {
    private final ChassisSubsystem chassis;
    private final DoubleSupplier forward, turn;

    public ArcadeDriveCommand(ChassisSubsystem chassis, DoubleSupplier forward, DoubleSupplier turn) {
        this.chassis = chassis;
        this.forward = forward;
        this.turn = turn;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        double left = forward.getAsDouble() + turn.getAsDouble();
        double right = forward.getAsDouble() - turn.getAsDouble();
        chassis.setPower(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.setPower(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
