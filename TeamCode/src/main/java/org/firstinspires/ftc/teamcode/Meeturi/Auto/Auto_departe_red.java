package org.firstinspires.ftc.teamcode.Meeturi.Auto;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.act_outtake;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.auto;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.ramp;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.target_velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.act_turret;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.error;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.TurretModule;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class Auto_departe_red extends OpMode {
    IntakeModule intake;
    OuttakeModule outtake;
    TurretModule turret;
    boolean transfer = false;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    public static double cat_trage = 1.15;
    public static double x_startPose = 88.987, y_startPose = 8.792, heading_startPose = 270;
    public static double x_preload = 82, y_preload = 20, heading_preload = 270;
    public static double x_collect1 = 123, y_collect1 = 84, heading_collect = 180;
    public static double x_trapa = 126, y_trapa = 75, heading_trapa = 110;
    public static double x_collect2 = 130.5, y_collect2 = 62;
    public static double x_collect3 = 130.5, y_collect3 = 38;
    public static double x_cp2 = 78.5, y_cp2 = 65.5;
    public static double x_cp3 = 85.5, y_cp3 = 40;
    public static double x_colt = 133, y_colt = 8.7, heading_colt = 90;
    public static double x_cptrapa = 109, y_cptrapa = 66;
    private final Pose startPose = new Pose(x_startPose, y_startPose, Math.toRadians(heading_startPose));
    private final Pose scorePose = new Pose(x_preload, y_preload, Math.toRadians(heading_preload));
    private final Pose scorePose_tangential = new Pose(x_preload, y_preload + 4, Math.toRadians(heading_preload));
    private final Pose collect1 = new Pose(x_collect1, y_collect1, Math.toRadians(heading_collect));
    private final Pose collect2 = new Pose(x_collect2, y_collect2, heading_collect);
    private final Pose collect3 = new Pose(x_collect3, y_collect3, heading_collect);
    private final Pose trapa = new Pose(x_trapa, y_trapa, heading_trapa);
    private final Pose cp_rand2 = new Pose(x_cp2, y_cp2);
    private final Pose cp_rand3 = new Pose(x_cp3, y_cp3);
    private final Pose cp_trapa = new Pose(x_cptrapa, y_cptrapa);
    private final Pose colt = new Pose(x_colt, y_colt, heading_colt);

    private Path scorePreload;
    private PathChain rand1, trage1, spretrapa, rand2, trage2, rand3, trage3, sprecolt, trage4, strafe;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(heading_startPose));

        rand1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, collect1))
                .setConstantHeadingInterpolation(Math.toRadians(heading_collect))
                .build();

        spretrapa = follower.pathBuilder()
                .addPath(new BezierCurve(collect2, cp_trapa, trapa))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        trage1 = follower.pathBuilder()
                .addPath(new BezierLine(colt, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        rand2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cp_rand2, collect2))
                .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))
                .build();

        trage2 = follower.pathBuilder()
                .addPath(new BezierLine(trapa, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        rand3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cp_rand3, collect3))
                .setConstantHeadingInterpolation(Math.toRadians(heading_collect))
                .build();

        trage3 = follower.pathBuilder()
                .addPath(new BezierLine(collect3, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        sprecolt = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, colt))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                intake.sus();
                act_outtake = true;
                act_turret = true;
                ramp = false;
                setPathState(1);

                break;

            case 1:
                if(pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intake.scuipa_transfer(0.35);
                }

                if(!follower.isBusy()) {
                    if (target_velocity < velocity + 5 && error <= 3 && act_outtake) {
                        outtake.deblocat();
                        transfer = true;
                        setPathState(2);
                    }
                }

                break;


            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.trage_intake(1);
                    intake.trage_transfer(1);

                }

                if(pathTimer.getElapsedTimeSeconds() > 0.4) {
                    ramp = true;
                    outtake.reglare();
                    setPathState(3);
                }

                break;

            case 3:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    act_turret = false;
                    setPathState(4);
                }

                break;


            case 4:
                follower.followPath(rand3, true);
                setPathState(5);

                break;

            case 5:
                if(!follower.isBusy()) {
                    intake.trage_transfer(0.7);
                    follower.followPath(trage3);
                    act_outtake = true;
                    act_turret = true;
                    ramp = false;
                    setPathState(6);
                }

                break;

            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intake.scuipa_transfer(0.35);
                }

                if(!follower.isBusy()) {
                    if (target_velocity < velocity + 5 && error <= 3 && act_outtake) {
                        outtake.deblocat();
                        transfer = true;
                        setPathState(7);
                    }
                }

                break;

            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.trage_intake(1);
                    intake.trage_transfer(1);
                }

                if(pathTimer.getElapsedTimeSeconds() > 0.4) {
                    ramp = true;
                    outtake.reglare();
                    setPathState(8);
                }

                break;

            case 8:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    act_turret = false;
                    setPathState(9);
                }

                break;

            case 9:
                follower.followPath(rand2, true);
                setPathState(10);

                break;

            case 10:
                if(!follower.isBusy()) {
                    intake.trage_intake(0.7);
                    setPathState(11);
                }

                break;

            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(spretrapa, true);
                    intake.stop_intake();
                    setPathState(12);
                }

                break;

            case 12:
                intake.trage_transfer(0.7);
                if(!follower.isBusy()) {
                    follower.followPath(trage2, true);
                    act_outtake = true;
                    act_turret = true;
                    ramp = false;
                    setPathState(13);
                }

                break;

            case 13:
                if(pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intake.scuipa_transfer(0.35);
                }

                if(!follower.isBusy()) {
                    if (target_velocity < velocity + 15 && error <= 3 && act_outtake) {
                        outtake.deblocat();
                        transfer = true;
                        setPathState(14);
                    }
                }

                break;

            case 14:
                if(pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.sus();
                    intake.trage_intake(1);
                    intake.trage_transfer(1);
                }

                if(pathTimer.getElapsedTimeSeconds() > 0.4) {
                    ramp = true;
                    outtake.reglare();
                    setPathState(15);
                }

                break;

            case 15:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    setPathState(16);
                }

                break;

            case 16:
                follower.followPath(sprecolt, true);
                setPathState(17);

                break;

            case 17:
                if(!follower.isBusy()) {
                    intake.trage_intake(0.7);
                    follower.followPath(trage1, true);
                    act_outtake = true;
                    act_turret = true;
                    ramp = false;
                    setPathState(18);
                }

                break;

            case 18:
                if(pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intake.scuipa_transfer(0.35);
                }

                if(!follower.isBusy()) {
                    if (target_velocity < velocity + 5 && error <= 1.5 && act_outtake) {
                        outtake.deblocat();
                        transfer = true;
                        setPathState(19);
                    }
                }

                break;

            case 19:
                if(pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.trage_intake(1);
                    intake.trage_transfer(1);

                }

                if(pathTimer.getElapsedTimeSeconds() > 0.4) {
                    ramp = true;
                    outtake.reglare();
                    setPathState(15);
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
        outtake.update();
        autonomousPathUpdate();

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = Math.toDegrees(follower.getPose().getHeading());
        distanta = Math.sqrt((144 - x) * (144 - x) + (144 - y) * (144 - y));
        turret.update_auto_red(x, y, h);


        telemetry.addData("path state", pathState);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", h);
        telemetry.addData("Distanta", distanta);
        telemetry.addData("error", turret.getError());
        telemetry.addData("Grade", turret.gra());
        telemetry.update();
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
        act_outtake = false;
        transfer = false;
        intake.trage_transfer(0.47);
    }
}
