package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "Auto BUCKET SIDE BAD", group = "Autonomous")
public class AUTO_BUCKET_SIDE extends LinearOpMode {
    public class Lift {
        private DcMotorEx liftV;

        public Lift(HardwareMap hardwareMap) {
            liftV = hardwareMap.get(DcMotorEx.class, "liftV");
            liftV.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftV.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftV.setPower(0.8);
                    initialized = true;
                }

                double pos = liftV.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 500.0) {
                    return true;
                } else {
                    liftV.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftV.setPower(-0.8);
                    initialized = true;
                }

                double pos = liftV.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 0.0) {
                    return true;
                } else {
                    liftV.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Extender {
        private DcMotorEx extendH;

        public Extender (HardwareMap hardwareMap) {
            extendH = hardwareMap.get(DcMotorEx.class, "liftH");
            extendH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendH.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extendH.setPower(1);
                    initialized = true;
                }

                double pos = extendH.getCurrentPosition();
                packet.put("extPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    extendH.setPower(0);
                    return false;
                }
            }
        }
        public Action liftOut() {
            return new LiftOut();
        }

        public class LiftIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extendH.setPower(-0.8);
                    initialized = true;
                }

                double pos = extendH.getCurrentPosition();
                packet.put("extPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    extendH.setPower(0);
                    return false;
                }
            }
        }
        public Action liftIn(){
            return new LiftIn();
        }
    }

    public static class Rotate {
        private Servo rotate;

        public Rotate(HardwareMap hardwareMap) {
            rotate = hardwareMap.get(Servo.class, "servo_rotate");
        }

        public class RotateUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotate.setPosition(.4);
                return false;
            }
        }
        public Action rotateUp() {
            return new RotateUp();
        }

        public class RotateDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotate.setPosition(0.4);
                return false;
            }
        }
        public Action rotateDown() {
            return new RotateDown();
        }
    }

    HardwareRobot robot = new HardwareRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        Pose2d initialPose = new Pose2d(0, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
       // Rotate rotate = new Rotate(hardwareMap);
        //Lift lift = new Lift(hardwareMap);
       // Extender extender = new Extender(hardwareMap);


        TrajectoryActionBuilder toSub = drive.actionBuilder(initialPose)

                .strafeTo(new Vector2d(0, -18))
                .strafeTo(new Vector2d(20, -18));
                            Pose2d toSubEnd = new Pose2d(20, -18, Math.toRadians(180));
                            // Headings: 180 = Left, 0 = Right, -90 = Down, 90=UP

        TrajectoryActionBuilder toSamples = drive.actionBuilder(toSubEnd)
                //Travel Around submersible and toward samples.
                .waitSeconds(.15)
                .splineToSplineHeading(new Pose2d(10,-30, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(28,-5, Math.toRadians(-90)), Math.toRadians(90))
                .strafeTo(new Vector2d(39, -0))
                //Push in Sample 1
                .strafeTo(new Vector2d(39, -50))// push sample to wall
                .strafeTo(new Vector2d(39, -10))// bak up
                .strafeTo(new Vector2d(53, -10)) // move over to sample 2
                //Push in Sample 2
                .strafeTo(new Vector2d(53, -50))// push sample to wall
                .strafeTo(new Vector2d(53, -10)) // backup
                .strafeTo(new Vector2d(66, -10)) // move over
                //Push in Sample 3
               .strafeTo(new Vector2d(66,-48))
                //Relocate to retrieve Specemine 1
                .splineToSplineHeading(new Pose2d(29,-51, Math.toRadians(0)), Math.toRadians(-95))
                //Press to wall for Spec1
                .waitSeconds(.15)
                .strafeTo(new Vector2d(29, -60));
                          Pose2d toSamplesEnd = new Pose2d(30, -53, Math.toRadians(-90));

        TrajectoryActionBuilder toSub2 = drive.actionBuilder(toSamplesEnd)
                      //Move to submersible to deliver Spec1
                .waitSeconds(.15)
                .strafeTo(new Vector2d(25, -48))
                .strafeToLinearHeading(new Vector2d(2, -0.5),Math.toRadians(150))
                .waitSeconds(.25);
                         Pose2d toSub2End = new Pose2d(2, -19, Math.toRadians(180));


        TrajectoryActionBuilder toWall2 = drive.actionBuilder(toSub2End)
                //Return for Spec2
                .waitSeconds(.25)
                .strafeTo(new Vector2d(2, -30))
                .strafeTo(new Vector2d(29, -30))
                .strafeToLinearHeading(new Vector2d(29, -55),Math.toRadians(0))
                     //Press Wall for Spec2
                .strafeTo(new Vector2d(29, -69));
                         Pose2d toWall2End = new Pose2d(29, -69, Math.toRadians(0));

        TrajectoryActionBuilder toSub3 = drive.actionBuilder(toWall2End)
                //.waitSeconds(.15)
                //Move to submersible to deliver Spec2
                .strafeTo(new Vector2d(20, -60))
                .strafeToLinearHeading(new Vector2d(0, -60),Math.toRadians(179))

                .waitSeconds(.25);

        Pose2d toSub3End = new Pose2d(0, -18, Math.toRadians(179));

        TrajectoryActionBuilder toWall3 = drive.actionBuilder(toSub3End)
                .waitSeconds(.25)
                //Return to the wall
                .strafeTo(new Vector2d(0, -30))
                .strafeToLinearHeading(new Vector2d(35, -40),Math.toRadians(0))
                //Pick up Spec3
                .strafeTo(new Vector2d(35, -61));
                         Pose2d toWall3End = new Pose2d(35, -61, Math.toRadians(0));

        TrajectoryActionBuilder toSub4 = drive.actionBuilder(toWall3End)
                 //Deliver Spec3
                .strafeTo(new Vector2d(30, -50))
                .strafeToLinearHeading(new Vector2d(3, -21),Math.toRadians(180));



        Action trajectoryActionCloseOut = toSub4.fresh()
                //.strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(rotate.rotateUp());




        telemetry.addData("Initialization Complete", "Awaiting Start Command");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        robot.servohang.setPosition(1);
        robot.specimenClamp.setPosition(0);
        robot.servorotate.setPosition(.4);
        lift (1, 18.5);
        Actions.runBlocking(
                new SequentialAction(
                        toSub.build()
                        //lift.liftUp(),
                      //  rotate.rotateDown()
                        //lift.liftDown(),
                        //trajectoryActionCloseOut
                )
        );
        lift (1, -4.5);
        sleep(50000000);
        robot.specimenClamp.setPosition(.4);
        lift (1, -14);


        Actions.runBlocking(
                new SequentialAction(
                        toSamples.build()
                )
        );
        robot.specimenClamp.setPosition(0);
        sleep(500);
        lift (1, 18.5);

        Actions.runBlocking(
                new SequentialAction(
                        toSub2.build()
                )
        );
        lift (1, -5);
        sleep(500);
        robot.specimenClamp.setPosition(.4);
        lift (1, -14);

        Actions.runBlocking(
                new SequentialAction(
                        toWall2.build()
                )
        );
        robot.specimenClamp.setPosition(0);
        sleep(500);
        lift (1, 18.5);

        Actions.runBlocking(
                new SequentialAction(
                        toSub3.build()
                )
        );
        lift (1, -5);
        sleep(500);
        robot.specimenClamp.setPosition(.4);
        lift (1, -14);

        Actions.runBlocking(
                new SequentialAction(
                        toWall3.build()
                )
        );

        lift (1, 18.5);

        Actions.runBlocking(
                new SequentialAction(
                        toSub4.build()
                )
        );
        lift (1, -18.5);



    }
    public void lift(double power, double inches)
    {     ElapsedTime runtime = new ElapsedTime();

        int newLiftTarget;

        if (opModeIsActive()) {

            newLiftTarget = robot.liftV.getCurrentPosition() + (int) (inches * (1140/(3.5 * 3.1415))*.717);

            robot.liftV.setTargetPosition(newLiftTarget);

            robot.liftV.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.liftV.setPower(Math.abs(power));

           /*while (opModeIsActive() &&
                                   robot.liftV.isBusy()) {
                       telemetry.addData("Lift", "Running at %7d",
                                       robot.liftV.getCurrentPosition());
                    telemetry.update();

               }
          robot.liftV.setPower(0);
          robot.liftV.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */

          }
     }


    public void extend(double power, double inches)
    {     ElapsedTime runtime = new ElapsedTime();

        int newLiftTarget;

        if (opModeIsActive()) {

            newLiftTarget = robot.liftH.getCurrentPosition() + (int) (inches * (1140/(3.5 * 3.1415)));

            robot.liftH.setTargetPosition(newLiftTarget);

            robot.liftH.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.liftH.setPower(Math.abs(power));

           /*while (opModeIsActive() &&
                                   robot.liftV.isBusy()) {
                       telemetry.addData("Lift", "Running at %7d",
                                       robot.liftV.getCurrentPosition());
                    telemetry.update();

               }
          robot.liftV.setPower(0);
          robot.liftV.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */

        }
    }

}

