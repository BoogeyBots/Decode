package org.firstinspires.ftc.teamcode.Meeturi.Auto.Nou;

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
@Autonomous (group = "Blue")
public class Aproape_blue extends OpMode {
    IntakeModule intake;
    OuttakeModule outtake;
    TurretModule turret;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    public static double cat_trage = 0.4;
    public static double x_startPose = 117.223, y_startPose = 129.29, heading_startPose = 225;
    public static double x_preload = 86, y_preload = 83, heading_preload = 225;
    public static double x_collect1 = 120, y_collect1 = 84, heading_collect = 180;
    public static double x_trapa = 129, y_trapa = 70, heading_trapa = 110;
    public static double x_collect2 = 128, y_collect2 = 57;
    public static double x_collect3 = 126, y_collect3 = 35;
    public static double x_cp2 = 70.4, y_cp2 = 59;
    public static double x_cp3 = 82, y_cp3 = 29;
    public static double x_cptrapa = 109, y_cptrapa = 66;
    public static double x_afara = 115, y_afara = 83;
    public static double x_colectare_gate = 10, y_colectare_gate = 60, heading_colectare_gate = -32; //-32
    private final Pose startPose = new Pose(x_startPose, y_startPose, Math.toRadians(heading_startPose)).mirror();
    private final Pose scorePose = new Pose(x_preload, y_preload, Math.toRadians(heading_preload)).mirror();
    private final Pose collect1 = new Pose(x_collect1, y_collect1, Math.toRadians(heading_collect)).mirror();
    private final Pose collect2 = new Pose(x_collect2, y_collect2, heading_collect).mirror();
    private final Pose collect3 = new Pose(x_collect3, y_collect3, heading_collect).mirror();
    private final Pose trapa = new Pose(x_trapa, y_trapa, heading_trapa).mirror();
    private final Pose cp_rand2 = new Pose(x_cp2, y_cp2).mirror();
    private final Pose cp_rand3 = new Pose(x_cp3, y_cp3).mirror();
    private final Pose cp_trapa = new Pose(x_cptrapa, y_cptrapa).mirror();
    private final Pose afara = new Pose(x_afara, y_afara).mirror();
    private final Pose colectare_gate = new Pose(x_colectare_gate, y_colectare_gate, Math.toRadians(heading_colectare_gate));
    private final Pose cp_gate = new Pose(40, 60);

    private Path scorePreload;
    private PathChain rand1, trage1, spretrapa, rand2, trage2, rand3, trage3, sprecolt, trage4, sprecolt2, trage5, leave, spretrapa2, gate, trage_gate;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(scorePose.getHeading());

        rand1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, collect1))
                .setConstantHeadingInterpolation(collect1.getHeading())
                .build();

        spretrapa = follower.pathBuilder()
                .addPath(new BezierCurve(collect2, cp_trapa, trapa))
                .setConstantHeadingInterpolation(collect1.getHeading())
                .build();


        trage1 = follower.pathBuilder()
                .addPath(new BezierLine(collect1, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        rand2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cp_rand2, collect2))
                .setConstantHeadingInterpolation(collect1.getHeading())
                .setNoDeceleration()
                .build();


        trage2 = follower.pathBuilder()
                .addPath(new BezierCurve(trapa, cp_rand2, scorePose))
                .setConstantHeadingInterpolation(collect1.getHeading())
                .build();

        rand3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cp_rand3, collect3))
                .setConstantHeadingInterpolation(collect1.getHeading())
                .setNoDeceleration()
                .build();

        trage3 = follower.pathBuilder()
                .addPath(new BezierLine(collect3, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, afara))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setNoDeceleration()
                .build();

        gate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cp_gate, colectare_gate))
                .setLinearHeadingInterpolation(follower.getHeading(), colectare_gate.getHeading())
                .build();

        trage_gate = follower.pathBuilder()
                .addPath(new BezierLine(colectare_gate, scorePose))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                act_turret = true;
                setPathState(1);

                break;

            case 1:
                act_outtake = true;
                setPathState(2);

                break;

            case 2:
                if (follower.atPose(scorePose, 5, 5)) {
                    if (target_velocity < velocity + 5 && error <= 5.4 && act_outtake) {
                        outtake.deblocat();
                        setPathState(3);
                    }
                }

                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    intake.trage_intake(1);
                    intake.trage_transfer(1);
                    setPathState(4);
                }

                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    setPathState(5);
                }

                break;

            case 5:
                follower.followPath(rand2, true);
                setPathState(6);

                break;

            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(spretrapa, true);
                    setPathState(7);
                }

                break;

            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(trage2, true);
                    deblocare_automata();
                    act_outtake = true;
                    setPathState(8);
                }

                break;

            case 8:
                if (follower.atPose(scorePose, 5, 5)){
                    if (target_velocity < velocity + 5 && error <= 5.4 && act_outtake) {
                        setPathState(9);
                    }
                }

                break;

            case 9:
                intake.trage_intake(1);
                intake.trage_transfer(1);
                setPathState(10);

                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    setPathState(11);
                }

                break;

            case 11:
                follower.followPath(gate, true);
                setPathState(12);

                break;

            case 12:
                if(pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(trage_gate, true);
                    deblocare_automata();
                    act_outtake = true;
                    setPathState(13);
                }

                break;

            case 13:
                if (follower.atPose(scorePose, 5, 5)) {
                    if (target_velocity < velocity + 5 && error <= 3.5 && act_outtake) {
                        setPathState(14);
                    }
                }

                break;

            case 14:
                intake.trage_intake(1);
                intake.trage_transfer(1);
                setPathState(15);

                break;

            case 15:
                if (pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    setPathState(16);
                }

                break;

            case 16:
                follower.followPath(gate, true);
                setPathState(17);

                break;

            case 17:
                if(pathTimer.getElapsedTimeSeconds() > 3.5) {
                    follower.followPath(trage_gate, true);
                    deblocare_automata();
                    act_outtake = true;
                    setPathState(18);
                }

                break;

            case 18:
                if (follower.atPose(scorePose, 5, 5)) {
                    if (target_velocity < velocity + 5 && error <= 3.5 && act_outtake) {
                        setPathState(19);
                    }
                }

                break;

            case 19:
                intake.trage_intake(1);
                intake.trage_transfer(1);
                setPathState(20);

                break;

            case 20:
                if (pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    setPathState(21);
                }

                break;

            case 21:
                follower.followPath(gate, true);
                setPathState(22);

                break;

            case 22:
                if(pathTimer.getElapsedTimeSeconds() > 3.5) {
                    follower.followPath(trage_gate, true);
                    deblocare_automata();
                    act_outtake = true;
                    setPathState(23);
                }

                break;

            case 23:
                if (follower.atPose(scorePose, 5, 5)) {
                    if (target_velocity < velocity + 5 && error <= 3.5 && act_outtake) {
                        setPathState(24);
                    }
                }

                break;

            case 24:
                intake.trage_intake(1);
                intake.trage_transfer(1);
                setPathState(25);

                break;

            case 25:
                if (pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    setPathState(26);
                }

                break;

            case 26:
                follower.followPath(rand1, true);
                setPathState(27);

                break;

            case 27:
                if(!follower.isBusy()) {
                    follower.followPath(trage1, true);
                    act_outtake = true;
                    setPathState(28);
                }

                break;

            case 28:
                if(pathTimer.getElapsedTimeSeconds() > 0.3) deblocare_automata();
                if (follower.atPose(scorePose, 5, 5)) {
                    if (target_velocity < velocity + 5 && error <= 3.5 && act_outtake) {
                        setPathState(29);
                    }
                }

                break;

            case 29:
                intake.trage_intake(1);
                intake.trage_transfer(1);
                setPathState(30);

                break;

            case 30:
                if (pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    setPathState(31);
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

        intake.init();
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

        if(intake.get_pintake() == 0 && intake.get_ptransfer() == 0) outtake.deblocat();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", h);
        telemetry.addData("Distanta", distanta);
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
        target_velocity = 1030;
        intake.trage_transfer(0.47);
    }

    public void deblocare_automata() {
        intake.stop_transfer();
        intake.stop_intake();

    }
}
