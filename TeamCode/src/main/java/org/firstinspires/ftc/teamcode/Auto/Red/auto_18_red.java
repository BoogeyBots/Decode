package org.firstinspires.ftc.teamcode.Auto.Red;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.act_outtake;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.auto;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.target_velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.voltage;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
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
@Autonomous (group = "Red")
public class auto_18_red extends OpMode {
    IntakeModule intake;
    OuttakeModule outtake;
    TurretModule turret;
    private Follower follower;
    boolean target_atins = false;
    double m;
    private Timer pathTimer;
    private int pathState;
    public static double cat_trage = 0.4;
    /*   public static double x_startPose = 118.651, y_startPose = 127.826, heading_startPose = 225;
       public static double x_preload = 86, y_preload = 83, heading_preload = 225;
       public static double x_collect1 = 127.5, y_collect1 = 84, heading_collect = 180;
       public static double x_trapa = 126, y_trapa = 70, heading_trapa = 110;
       public static double x_collect2 = 125, y_collect2 = 57;
       public static double x_collect3 = 126, y_collect3 = 35;
       public static double x_cp2 = 70.4, y_cp2 = 59;
       public static double x_cp3 = 82, y_cp3 = 29;
       public static double x_cptrapa = 109, y_cptrapa = 66;
       public static double x_afara = 115, y_afara = 83;
       public static double x_colectare_gate = 10, y_colectare_gate = 61.2, heading_colectare_gate = -32; //-32 */
    private final Pose startPose = new Pose(24.846, 128.044, Math.toRadians(134)).mirror();
    private final Pose scorePose = new Pose(53.95, 84.52, Math.toRadians(134)).mirror();
    private final Pose collect1 = new Pose(120.017, 84.429, Math.toRadians(180));
    private final Pose cp_collect1 = new Pose(69.227, 81.442).mirror();
    private final Pose collect2 = new Pose(129.27, 59, Math.toRadians(180));
    private final Pose cp_collect2 = new Pose(43.82, 60.48).mirror();
    // private final Pose collect3 = new Pose(x_collect3, y_collect3, heading_collect).mirror();
    private final Pose colectare_gate = new Pose(7, 59.5, Math.toRadians(-38)).mirror();
    private final Pose colectare_gate2 = new Pose(7, 59.5, Math.toRadians(-38)).mirror();
    private final Pose colectare_gate3 = new Pose(7, 59.5, Math.toRadians(-38)).mirror();
    private final Pose cp_gate = new Pose(34.75, 57.47).mirror();
    private final Pose cp_trage_gate = new Pose(44.40, 68.77).mirror();
    //private final Pose cp_rand2 = new Pose(x_cp2, y_cp2).mirror();
    //private final Pose cp_rand3 = new Pose(x_cp3, y_cp3).mirror();
    //private final Pose cp_trapa = new Pose(x_cptrapa, y_cptrapa).mirror();
    //private final Pose afara = new Pose(x_afara, y_afara).mirror();
    //private final Pose colectare_gate = new Pose(x_colectare_gate, y_colectare_gate, Math.toRadians(heading_colectare_gate));
    //private final Pose cp_gate = new Pose(40, 60);
    private final Pose parcare = new Pose(61.316, 100).mirror();
    private Path scorePreload;
    private PathChain rand1, trage1, spretrapa, rand2, trage2, rand3, trage3, sprecolt, trage4, sprecolt2, trage5, leave, spretrapa2, gate, gate2, gate3, trage_gate;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), collect1.getHeading());

        rand1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, collect1))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .setReversed()
                .build();

        trage1 = follower.pathBuilder()
                .addPath(new BezierLine(collect1, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        rand2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cp_collect2, collect2))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .setReversed()
                .build();

        trage2 = follower.pathBuilder()
                .addPath(new BezierLine(collect2, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        gate = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, colectare_gate))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.tangent
                                                .reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .7,
                                        1,
                                        HeadingInterpolator.linear(follower.getHeading(), colectare_gate.getHeading())
                                )
                        )

                )
                .build();

        gate2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, colectare_gate2))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.tangent
                                                .reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .7,
                                        1,
                                        HeadingInterpolator.linear(follower.getHeading(), colectare_gate.getHeading())
                                )
                        )

                )
                .build();

        gate3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, colectare_gate3))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.tangent
                                                .reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .7,
                                        1,
                                        HeadingInterpolator.linear(follower.getHeading(), colectare_gate.getHeading())
                                )
                        )

                )
                .build();


        trage_gate = follower.pathBuilder()
                .addPath(new BezierCurve(colectare_gate,cp_trage_gate, scorePose))
                .setTangentHeadingInterpolation()
                .build();


        leave = follower.pathBuilder()
                .addPath(new BezierCurve(colectare_gate, cp_trage_gate, parcare))
                .setTangentHeadingInterpolation()
                .build();


       /* spretrapa = follower.pathBuilder()
                .addPath(new BezierCurve(collect2, cp_trapa, trapa))
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







        */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                act_outtake = true;
                if(pathTimer.getElapsedTime() > 0.1){
                    deblocare(0.35);
                    setPathState(1);
                }



                break;

            case 1:
                if(follower.atPose(scorePose, 5, 5)) {
                    trage();
                    setPathState(2);
                }
                break;

            case 2:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    trage();
                    follower.followPath(rand1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()){
                    follower.followPath(trage1);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 0.7){
                    deblocare(0.39);
                    setPathState(5);
                }

            case 5:
                if(follower.atPose(scorePose, 5, 5)) {
                    trage();
                    setPathState(6);
                }
                break;

            case 6:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage){
                    numaitrag();
                    trage();
                    follower.followPath(rand2, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()){
                    follower.followPath(trage2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(pathTimer.getElapsedTimeSeconds() > 0.7){
                    deblocare(0.38);
                    setPathState(9);
                }

            case 9:
                if (follower.atPose(scorePose, 5, 5)){
                    trage();
                    setPathState(10);
                }
                break;

            case 10:
                if(pathTimer.getElapsedTimeSeconds() > 0.5){
                    numaitrag();
                    follower.followPath(gate, false);
                    setPathState(11);
                }
                break;

            case 11:
                if(!follower.isBusy()) {
                    intake.trage_intake(0.6);
                    intake.trage_transfer(0.3);
                    setPathState(12);
                }
                break;

            case 12:
                if(pathTimer.getElapsedTimeSeconds() > 0.9){
                    follower.followPath(trage_gate, true);
                    deblocare(0.39);
                    setPathState(13);
                }
                break;


            case 13:
                if(follower.atPose(scorePose, 5, 5)){
                    trage();
                    setPathState(14);
                }
                break;

            case 14:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage){
                    numaitrag();
                    follower.followPath(gate2, false);
                    setPathState(15);
                }
                break;

            case 15:
                if(!follower.isBusy()) {
                    trage();
                    setPathState(16);
                }
                break;

            case 16:
                if(pathTimer.getElapsedTimeSeconds() > 0.9){
                    follower.followPath(trage_gate, true);
                    deblocare(0.39);
                    setPathState(17);
                }
                break;

            case 17:
                if(follower.atPose(scorePose, 5, 5)){
                    trage();
                    setPathState(18);
                }
                break;

            case 18:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage){
                    numaitrag();
                    follower.followPath(gate3, false);
                    setPathState(19);
                }
                break;

            case 19:
                if(!follower.isBusy()) {
                    trage();
                    setPathState(20);
                }
                break;

            case 20:
                if(pathTimer.getElapsedTimeSeconds() > 0.9){
                    follower.followPath(leave, true);
                    deblocare(0.38);
                    setPathState(21);
                }
                break;

            case 21:
                if(follower.atPose(parcare, 5, 5)){
                    trage();
                    setPathState(22);
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
        outtake.init();
        turret.init();

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
        distanta = Math.sqrt((144 - x) * (144 - x) + (144 - y) * (144 - y));

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();


        telemetry.addData("path state", pathState);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", h);
        telemetry.addData("Distanta", distanta);
        telemetry.addData("pathTimer", pathTimer.getElapsedTime());
        telemetry.update();

        outtake.update_kinematics();
        turret.update_red_auto(x, y, h);

        if(target_atins && velocity - target_velocity < -50)
            outtake.reglare_aproape_far();
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

    public void trage() {
        intake.trage_intake(1);
        intake.trage_transfer(1);
        target_atins = true;
    }

    public void numaitrag() {
        outtake.blocat();
        target_velocity = 1100;
        intake.trage_transfer(0.47);
        target_atins = false;
    }

    public void deblocare(double hood) {
        act_outtake = true;
        intake.stop_intake();
        intake.stop_transfer();
        outtake.deblocat();
        outtake.rampa(hood);
    }
}


