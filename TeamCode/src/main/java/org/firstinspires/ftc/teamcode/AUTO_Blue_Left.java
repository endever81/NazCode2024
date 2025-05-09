package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Autonomous(name = "Auto Blue Left", group = "Autonomous")
public class AUTO_Blue_Left extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    private volatile boolean servoOverrideActive = false; // Flag to manage servo control
    private  int zeroLiftH; // Int variable shared across all instances

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        Pose2d initialPose = new Pose2d(-18, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        zeroLiftH = robot.liftH.getCurrentPosition();


        TrajectoryActionBuilder toSub = drive.actionBuilder(initialPose)

                .strafeTo(new Vector2d(-12, -42));
        Pose2d toSubEnd = new Pose2d(-12, -42, Math.toRadians(90));
        // Headings: 180 = Left, 0 = Right, -90 = Down, 90=UP

        TrajectoryActionBuilder toSample1 = drive.actionBuilder(toSubEnd)
                //Travel Around submersible and toward sample 1.
                .strafeToLinearHeading(new Vector2d(-18, -50),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-59, -36),Math.toRadians(90));


               // .splineToSplineHeading(new Pose2d(-57,-105, Math.toRadians(0)), Math.toRadians(-95))
                Pose2d toSample1End = new Pose2d(-59, -36, Math.toRadians(90));

        TrajectoryActionBuilder toHG1 = drive.actionBuilder(toSample1End)
                //Move to High Goal to deliver Sample 1

                .strafeToLinearHeading(new Vector2d(-78, -58),Math.toRadians(90));
               // .waitSeconds(10);

                Pose2d toHG1End = new Pose2d(-78, -58, Math.toRadians(90));


        TrajectoryActionBuilder toSample2 = drive.actionBuilder(toHG1End)
                //Return for Sample 2
                .strafeToLinearHeading(new Vector2d(-65, -50),Math.toRadians(95));

                Pose2d toSample2End = new Pose2d(-65, -50, Math.toRadians(95));

        TrajectoryActionBuilder toHG2 = drive.actionBuilder(toSample2End)
                //Move to High Goal to deliver Sample 2
                .strafeToLinearHeading(new Vector2d(-78, -58),Math.toRadians(90));

                Pose2d toHG2End = new Pose2d(-78, -58, Math.toRadians(90));

        TrajectoryActionBuilder toSample3 = drive.actionBuilder(toHG2End)
                //Return for Sample 3
                .strafeToLinearHeading(new Vector2d(-70, -50),Math.toRadians(100));

            Pose2d toSample3End = new Pose2d(-70, -50, Math.toRadians(100));

        TrajectoryActionBuilder toHG3 = drive.actionBuilder(toSample3End)
                //Move to High Goal to deliver Sample 3
                .strafeToLinearHeading(new Vector2d(-78, -58),Math.toRadians(90));

            Pose2d toHG3End = new Pose2d(-78, -58, Math.toRadians(90));

        TrajectoryActionBuilder toSubContact = drive.actionBuilder(toHG3End)
                //Move to contact the submersible
                .strafeToLinearHeading(new Vector2d(-35, -15),Math.toRadians(180));


        Action trajectoryActionCloseOut = toSubContact.fresh()
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(rotate.rotateUp());




        telemetry.addData("Initialization Complete", "Awaiting Start Command");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        //Initial Servo Movmements and Lift to Champer Height
            robot.liftH.setTargetPosition(zeroLiftH);
            robot.liftH.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftH.setPower(1);

            robot.bucketart.setPosition(0.5); //swing out the top arm

            robot.servohang.setPosition(1);
            robot.specimenClamp.setPosition(0);
            robot.servorotate.setPosition(.1);
            robot.servoarm.setPosition(.5);  //.6

            lift (1, 18.5); //Lift to High Chamber height

        RRDrive(toSub);
            lift (1, -5.5);  //lower lift to lock specimen to bar
            sleep(750);
            robot.specimenClamp.setPosition(.4);  //release servo
            lift (.5, -13); //lower lift to the ground

            robot.servoarm.setPosition(.17); //Swing specimine arm out of the way
            robot.servoswing.setPosition(0.5);    // SET SWING ARM POSITION for scanning sweep  0.6
            robot.servorotate.setPosition(0.3);  //moves the claw up and down 0.3 up / 0.6 Down
            robot.servointake.setPosition(0.3); // Closes the claw  0.3 open / 0.48 closed
            extend(.25, 4); //sent out scanner claw
        RRDrive(toSample1);
            liftOff();
            sweepForYellowSample();
            pullOffSample();

            robot.servorotate.setPosition(0.6);  //moves the claw 0.6 Down
            sleep(300);
            robot.servointake.setPosition(0.54); // Closes the claw  0.3 open / 0.48 closed
            sleep(600);
            robot.servorotate.setPosition(0.3);  //moves the claw up and down 0.3 up / 0.6 Down
            sleep(450);

            controlServosAsync();  //automated servo transfer routine
            sleep(2900);

            lift (1, 29); //lift to high goal
             robot.bucketrotate.setPosition(0.2);


        RRDrive(toHG1);
            robot.bucketrotate.setPosition(0.0);
            sleep(250);
            robot.bucketgrab.setPosition(0.2);   //Bucket claw 0.2 open/0.58 closed
            sleep(100);
            robot.servorotate.setPosition(0.6);  //moves the claw 0.6 Down
            sleep(500);
            lift (1, -29);   //drop the lift

        RRDrive(toSample2);
            robot.servoarm.setPosition(.17); //Swing specimine arm out of the way
            robot.servoswing.setPosition(0.6);    // SET SWING ARM POSITION for scanning sweep
            robot.servorotate.setPosition(0.3);  //moves the claw up and down 0.3 up / 0.6 Down
            robot.servointake.setPosition(0.3); // Closes the claw  0.3 open / 0.48 closed
            extend(.7, 10.5);
            sweepForYellowSample();
            liftOff();
            pullOffSample();

            robot.servorotate.setPosition(0.6);  //moves the claw 0.6 Down
            sleep(300);
            robot.servointake.setPosition(0.54); // Closes the claw  0.3 open / 0.48 closed
            sleep(600);
            robot.servorotate.setPosition(0.3);  //moves the claw up and down 0.3 up / 0.6 Down
            sleep(450);

            controlServosAsync();  //automated servo transfer routine
            sleep(2900);

            lift (1, 29); //lift to high goal
            robot.bucketrotate.setPosition(0.2);

        RRDrive(toHG2);
            robot.bucketrotate.setPosition(0.0);
            sleep(250);
            robot.bucketgrab.setPosition(0.2);   //Bucket claw 0.2 open/0.58 closed
            sleep(100);
            robot.servorotate.setPosition(0.6);  //moves the claw 0.6 Down
            sleep(500);
            lift (1, -29);   //drop the lift


        RRDrive(toSample3);
            robot.servoarm.setPosition(.17); //Swing specimine arm out of the way
            robot.servoswing.setPosition(0.6);    // SET SWING ARM POSITION for scanning sweep
            robot.servorotate.setPosition(0.3);  //moves the claw up and down 0.3 up / 0.6 Down
            robot.servointake.setPosition(0.3); // Closes the claw  0.3 open / 0.48 closed
            extend(.5, 10.5);
            sweepForYellowSample();
            liftOff();
            pullOffSample();

            robot.servorotate.setPosition(0.6);  //moves the claw 0.6 Down
            sleep(300);
            robot.servointake.setPosition(0.54); // Closes the claw  0.3 open / 0.48 closed
            sleep(600);
            robot.servorotate.setPosition(0.3);  //moves the claw up and down 0.3 up / 0.6 Down
            sleep(450);

            controlServosAsync();  //automated servo transfer routine
            sleep(2900);

            lift (1, 29); //lift to high goal
            robot.bucketrotate.setPosition(0.2);
        RRDrive(toHG3);
            robot.bucketrotate.setPosition(0.0);
            sleep(250);
            robot.bucketgrab.setPosition(0.2);   //Bucket claw 0.2 open/0.58 closed
            sleep(100);
            robot.servorotate.setPosition(0.6);  //moves the claw 0.6 Down
            sleep(500);
            lift (1, -29);   //drop the lift

    RRDrive(toSubContact);

    }


    public void lift(double power, double inches)
    {     ElapsedTime runtime = new ElapsedTime();

        int newLiftTarget;
        int newLiftTarget2;



        if (opModeIsActive()) {

            newLiftTarget = robot.liftV.getCurrentPosition() + (int) (inches * (-1140/(3.5 * 3.1415))*.717);
            newLiftTarget2 = robot.liftV2.getCurrentPosition() + (int) (inches * (-1140/(3.5 * 3.1415))*.717);

            robot.liftV.setTargetPosition(newLiftTarget);
            robot.liftV2.setTargetPosition(newLiftTarget2);

            robot.liftV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftV2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.liftV.setPower(Math.abs(power));
            robot.liftV2.setPower(Math.abs(power));

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
    /**
     * Asynchronous method to control servos
     */
    private void controlServosAsync() {
        new Thread(() -> {
            try {
                servoOverrideActive = true; // Mark override as active
                robot.liftH.setTargetPosition(zeroLiftH);
                robot.liftH.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftH.setPower(1);

                setServoPosition(robot.servoart, 1); //ensure head is orented appropriately
                Thread.sleep(100);

                // Move servos through a sequence
                setServoPosition(robot.bucketrotate, 0.7); // push bucket arm out for transfer
                setServoPosition(robot.servorotate, 0.1); //flip up the Sample
                setServoPosition(robot.servointake, 0.52); //ensure low claw says closed .68
                Thread.sleep(250);
                setServoPosition(robot.servoart, .94); //ensure head is orented appropriately
                setServoPosition(robot.servoswing, 0.25); //swing to reposition sample in claw
                setServoPosition(robot.bucketgrab, 0.3); //open the high claw
                Thread.sleep(650);
                Thread.interrupted();
                setServoPosition(robot.servointake, 0.5); //loosen low claw .68
                setServoPosition(robot.servorotate, 0.40); //put sample on the ground .25
                Thread.sleep(250);  // Wait for movement to complete

                setServoPosition(robot.servointake, 0.52); //tightly grip the Sample .7
                setServoPosition(robot.servoswing, 0.3); //move sample back slightly
                setServoPosition(robot.servorotate, 0.40); //move sample down slightly .25
                Thread.sleep(250);  // Wait for movement to complete
                setServoPosition(robot.servoswing, 0.20); //move sample back slightly


                //top arm down
                setServoPosition(robot.bucketrotate, 0.94);
                setServoPosition(robot.bucketgrab, 0.45);

                Thread.sleep(250);

                //top arm grab
                setServoPosition(robot.bucketgrab, 0.65);
                Thread.sleep(300);

                //lowerarm letgo
                setServoPosition(robot.servointake, 0.1); //.5
                Thread.sleep(400);

                //lowerarm left
                setServoPosition(robot.servoart, .5); //ensure head is orented appropriately
                setServoPosition(robot.servoswing, 0.5);
                //Thread.sleep(250);

                //top arm up
                setServoPosition(robot.bucketrotate, .5);


                //Release hLift from power
                robot.liftH.setPower(0);
                robot.liftH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





            } catch (Exception e) {
                // Handle interrupted exception properly
                Thread.currentThread().interrupt(); // Restore interrupt flag
            } finally {
                servoOverrideActive = false; // Release control back to manual mode
            }
        }).start();
    }

    /**
     * Safe method to set servo position, ensuring override is respected.
     */
    private synchronized void setServoPosition(com.qualcomm.robotcore.hardware.Servo servo, double position) {
        if (servoOverrideActive) {
            servo.setPosition(position);
        }
    }

    /**
     * Sweeps the servo from start to end range and looks for a yellow block using HSV hue.
     * Returns true if yellow was detected, false if not.
     */
    private boolean sweepForYellowSample() {
        float[] hsvValues = new float[3];
        double sweepStart = 0.6;
        double sweepEnd = 0.2;
        double sweepStep = -0.04;

        for (double pos = sweepStart; pos >= sweepEnd; pos += sweepStep) {
            robot.servoswing.setPosition(pos);
            robot.servoart.setPosition(pos+.04);
            sleep(150); // Give servo and sensor time to settle

            NormalizedRGBA colors = robot.sensorColorDistance.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            float hue = hsvValues[0];
            double alpha = colors.alpha;
            double distance = ((DistanceSensor) robot.sensorColorDistance).getDistance(DistanceUnit.CM);
            telemetry.addData("Sweep Pos", "%.2f", pos);
            telemetry.addData("Hue", "%.1f", hue);
            telemetry.addData("Alpha", "%.3f", alpha);
            telemetry.addData("Distance", "%.2f", distance );
            telemetry.update();

            // Yellow hue range ≈ 30–60°, alpha helps verify reflectivity
            if (distance <= 5.8 && alpha > 0.1) {
                telemetry.addLine("✅ Yellow Sample Detected");
                telemetry.update();
                return true;
            }
        }

        telemetry.addLine("⚠️ Yellow Not Found During Sweep");
        telemetry.update();
        return false;
    }

    private boolean sweepForYellowSample1() {
        float[] hsvValues = new float[3];
        double sweepStart = 0.55;
        double sweepEnd = 0.1;
        double sweepStep = -0.04;

        for (double pos = sweepStart; pos >= sweepEnd; pos += sweepStep) {
            robot.servoswing.setPosition(pos);
            robot.servoart.setPosition(pos+.04);
            sleep(150); // Give servo and sensor time to settle

            NormalizedRGBA colors = robot.sensorColorDistance.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            float hue = hsvValues[0];
            double alpha = colors.alpha;
            double distance = ((DistanceSensor) robot.sensorColorDistance).getDistance(DistanceUnit.CM);
            telemetry.addData("Sweep Pos", "%.2f", pos);
            telemetry.addData("Hue", "%.1f", hue);
            telemetry.addData("Alpha", "%.3f", alpha);
            telemetry.addData("Distance", "%.2f", distance );
            telemetry.update();

            // Yellow hue range ≈ 30–60°, alpha helps verify reflectivity
            if (distance <= 5.8 && alpha > 0.1) {
                telemetry.addLine("✅ Yellow Sample Detected");
                telemetry.update();
                return true;
            }
        }

        telemetry.addLine("⚠️ Yellow Not Found During Sweep");
        telemetry.update();
        return false;
    }


    private boolean pullOffSample() {
        float[] hsvValues = new float[3];

        NormalizedRGBA colors = robot.sensorColorDistance.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        double alpha = colors.alpha;
        double distance = ((DistanceSensor) robot.sensorColorDistance).getDistance(DistanceUnit.CM);

        while (alpha >= 0.085 && !isStopRequested()) {
            extend(.15, -3);

            colors = robot.sensorColorDistance.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            alpha = colors.alpha;
            distance = ((DistanceSensor) robot.sensorColorDistance).getDistance(DistanceUnit.CM);
            telemetry.addData("Alpha", "%.3f", alpha);
            telemetry.addData("Distance", "%.2f", distance );
            telemetry.update();
        }

        if (distance >= 5.5 && alpha < 0.1) {
            robot.liftH.setPower(0);
            telemetry.addLine("✅ Yellow Sample Edge Found");
            telemetry.update();
            return true;
        }
        robot.liftH.setPower(0);
        telemetry.addLine("⚠️ Retract Did Not Detect Sample");
        telemetry.update();
        return false;
    }



    private TrajectoryActionBuilder RRDrive (TrajectoryActionBuilder rrdrive){
        Actions.runBlocking(
                new SequentialAction(
                        rrdrive.build() // To Submersible
                )
        );
        return rrdrive;
    }

    private void liftOff () {
        robot.liftV.setPower(0);
        robot.liftV2.setPower(0);
    }
}

