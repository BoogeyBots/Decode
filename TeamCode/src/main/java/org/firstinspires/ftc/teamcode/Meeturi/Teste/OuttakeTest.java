package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
@Configurable
@TeleOp
public class OuttakeTest extends LinearOpMode {
    public static double pw, pos;
    @Override
    public void runOpMode() throws InterruptedException {
        OuttakeModule outtake = new OuttakeModule(hardwareMap);
        IntakeModule intake = new IntakeModule(hardwareMap);
        TelemetryManager panelsTelemetry;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        outtake.init_teleOP();
        intake.init_teleOP();

        waitForStart();

        while (opModeIsActive()) {
            outtake.update();
            outtake.servo_custom();
            intake.trage(pw);

            panelsTelemetry.addData("Velocity 1", outtake.get_velocity_sus());
            panelsTelemetry.addData("Target", outtake.get_target());
            panelsTelemetry.update(telemetry);
        }
    }
}