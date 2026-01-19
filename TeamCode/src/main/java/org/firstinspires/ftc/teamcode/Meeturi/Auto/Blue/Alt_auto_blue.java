package org.firstinspires.ftc.teamcode.Meeturi.Auto.Blue;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.act_outtake;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.auto;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.nominalvoltage;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.target_velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.voltage;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.act_turret;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.error;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.TurretModule;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Configurable
@Autonomous
public class Alt_auto_blue extends OpMode {
    IntakeModule intake;
    OuttakeModule outtake;
    TurretModule turret;
    boolean transfer = false;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    public static double cat_trage = 0.4, cat_sta = 1.5; //în secunde
    public static double x_startPose = 118.476, y_startPose = 127.106, heading_startPose = 225;  //127.106  118.476
    public static double x_preload = 80, y_preload = 83, heading_preload = 225;
    public static double x_trapa = 120, y_trapa = 72, heading_trapa = 180;
    public static double x_dupa = 133.5, y_dupa = 55, heading_dupa = 240;
    public static double x_cp = 123.5, y_cp = 58;
    private final Pose startPose = new Pose(x_startPose, y_startPose, Math.toRadians(heading_startPose)).mirror();
    private final Pose scorePose = new Pose(x_preload, y_preload, Math.toRadians(heading_preload)).mirror();
    private final Pose trapa = new Pose(x_trapa, y_trapa, Math.toRadians(heading_trapa)).mirror();
    private final Pose cp = new Pose(x_cp, y_cp).mirror();
    private final Pose dupa = new Pose(x_dupa, y_dupa, Math.toRadians(heading_dupa)).mirror();


    private Path scorePreload;
    private PathChain rand1, cycle, trage_cycle, trage1, spretrapa, rand2, trage2, rand3, trage3, sprecolt, trage4;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(scorePose.getHeading());

        rand1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, trapa))
                .setConstantHeadingInterpolation(trapa.getHeading())
                .build();

        rand2 = follower.pathBuilder()
                .addPath(new BezierCurve(trapa, cp, dupa))
                .setLinearHeadingInterpolation(trapa.getHeading(), dupa.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);

                break;

            case 1:
                intake.sus();
                setPathState(2);

                break;


            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(rand1, true);
                    setPathState(3);
                }

                break;

            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(rand2, true);
                    intake.trage_intake(1);
                    intake.trage_transfer(0.47);
                    setPathState(4);
                }

                break;


        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        intake = new IntakeModule(hardwareMap);
        outtake = new OuttakeModule(hardwareMap);
        turret = new TurretModule(hardwareMap);

        intake.init_auto();
        outtake.init_auto_aproape();
        turret.init_auto();

        buildPaths();
        follower.setStartingPose(startPose);

        auto = true;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = Math.toDegrees(follower.getPose().getHeading());
        distanta = Math.sqrt((0 - x) * (0 - x) + (144 - y) * (144 - y));
        turret.update_auto_blue(x, y, h);

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", h);
        telemetry.addData("Distanta", distanta);
//        telemetry.addData("error", turret.getError());
//        telemetry.addData("Grade", turret.gra());
        telemetry.addData("Voltage", nominalvoltage/voltage);
        telemetry.update();

        outtake.update_kinematics();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
        target_velocity = 0;
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void stop() {}

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void numaitrag() {
        outtake.blocat();
        target_velocity = 0;
        transfer = false;
        intake.trage_transfer(0.47);
    }
}
