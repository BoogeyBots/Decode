package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;

@TeleOp
public class IntakeTest extends LinearOpMode {
    public static double power;
    @Override
    public void runOpMode() throws InterruptedException {
        IntakeModule intake = new IntakeModule(hardwareMap);

        intake.init();

        waitForStart();

        while (opModeIsActive()) {
            intake.trage(power);
        }

    }
}
