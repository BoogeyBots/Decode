package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
@TeleOp
public class OuttakeTest extends LinearOpMode {
    public static double power, pos;
    @Override
    public void runOpMode() throws InterruptedException {
        OuttakeModule outtake = new OuttakeModule(hardwareMap);

        outtake.init();

        waitForStart();

        while (opModeIsActive()) {
            //outtake.trage(power);
            outtake.custom_angle(pos);
        }
    }
}
