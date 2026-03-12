package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
@Configurable
@TeleOp
public class Transfer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IntakeModule intake = new IntakeModule(hardwareMap);
        OuttakeModule outtake = new OuttakeModule(hardwareMap);

        intake.init();
        outtake.init_teleOP();

        waitForStart();

        while(opModeIsActive()) {
            intake.trage_intake(1);
            intake.trage_transfer(1);
        }
    }
}
