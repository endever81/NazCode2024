package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name = "Driver Controlled", group = "Robot")

public class DriverControlled extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();

    
@Override
public void runOpMode() {
        robot.init(hardwareMap);

        int newLiftTargetH;
        int newLiftTargetV;

        RevBlinkinLedDriver.BlinkinPattern pattern;
        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        RevBlinkinLedDriver.BlinkinPattern patternPrime;
        patternPrime = RevBlinkinLedDriver.BlinkinPattern.GREEN;

        BNO055IMU imu;

        Orientation angles;

       // robot.init(hardwareMap);


        //IMU Initialization
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.mode = BNO055IMU.SensorMode.IMU;
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters.loggingEnabled = false;

                imu = hardwareMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);

                telemetry.addData("Mode", "calibrating...");
                telemetry.update();

                // make sure the imu gyro is calibrated before continuing.
                while (!isStopRequested() && !imu.isGyroCalibrated()) {
                    sleep(50);
                    idle();
                }


        telemetry.addData("Say", "Waiting for Start");
        telemetry.update();

        robot.liftH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();

        newLiftTargetH = robot.liftH.getCurrentPosition();
        newLiftTargetV = robot.liftV.getCurrentPosition();
        robot.liftH.setTargetPosition(newLiftTargetH);
        robot.liftV.setTargetPosition(newLiftTargetV);
        telemetry.addData("horizontal",robot.liftH.getCurrentPosition());
        telemetry.addData("Vertical",robot.liftV.getCurrentPosition());
        telemetry.addData("encoder", robot.leftRearDrive.getCurrentPosition());
        telemetry.update();
    double hangPower = 0;
    double specimenPosition = 0;
    double rotatePostion = .5;
    double swingPosition= 0.5;
    double artPosition = 0.5;
    double servohangPosition = 0;
    double bucketartPosition = .5;
    double bucketrotatePosition = .5;
    double bucketgrabPosition = .5;


    // Variables for articulation servo
    double articulationBaseAngle = 0; // Base angle synchronized with swing arm
    double articulationTrim = 0;     // Trim adjustment
    final double TRIM_INCREMENT = 23; // 23-degree increments
    final double MAX_ARTICULATION_ANGLE = 270;
    final double MIN_ARTICULATION_ANGLE = 0;
    final double DEFAULT_CENTER_OFFSET = 21; // Default center offset from the left edge

// Track button state to ensure single press action
    boolean xPressed = false;
    boolean yPressed = false;


        while (opModeIsActive()){
            telemetry.addData("horizontal",robot.liftH.getCurrentPosition());
            telemetry.addData("Vertical",robot.liftV.getCurrentPosition());
            telemetry.addData(
                    "encoder", robot.leftRearDrive.getCurrentPosition());
            telemetry.update();

        double Turn = gamepad1.left_stick_x;
        double Speed = -gamepad1.left_stick_y;
        double Strafe = -gamepad1.right_stick_x;

        double front_left = Speed + Turn - Strafe;
        double front_right = Speed - Turn + Strafe;
        double rear_left = Speed + Turn + Strafe;
        double rear_right = Speed - Turn - Strafe;

        front_left = Range.clip(front_left, -1, 1);
        front_right = Range.clip(front_right, -1, 1);
        rear_left = Range.clip(rear_left, -1, 1);
        rear_right = Range.clip(rear_right, -1, 1);

        front_left = (float)scaleInput(front_left);
        front_right = (float)scaleInput(front_right);
        rear_left = (float)scaleInput(rear_left);
        rear_right = (float)scaleInput(rear_right);

        front_left /=1;
        front_right /=1;
        rear_left /=1;
        rear_right /=1;

        if (gamepad1.right_bumper){
        front_left *=2;
        front_right *=2;
        rear_left *=2;
        rear_right *=2;
        }

        if (gamepad1.left_bumper){
        front_left /=2.5;
        front_right /=2.5;
        rear_left /=2.5;
        rear_right /=2.5;
        }
        if (gamepad1.a){
            hangPower = 1;
        }
        else {
            hangPower = 0;
        }
        if (gamepad1.b)
             {
            hangPower = -1;
        }
     //   if (gamepad1.dpad_up){
       //     servohangPosition = .5;
        //}

        if (gamepad1.dpad_up){
        specimenPosition = 0;
        }
        if (gamepad1.dpad_down){
            specimenPosition =.3;
            }


     //   double liftVPower = 0;

     //   if (gamepad1.dpad_up){
       //     liftVPower = 1;
      //  }
      //  if (gamepad1.dpad_down){
      //      liftVPower = -1;
       // }

        double liftVPower = ((gamepad1.right_trigger)-(gamepad1.left_trigger));
        double liftHPower = gamepad2.right_stick_y;
        if(robot.liftH.getCurrentPosition() <-1300 && gamepad2.right_stick_y < 0)
            {liftHPower = 0;
            }
// new code for Livy's Controller
            double intakePosition = 0.50;

            if (gamepad2.a) {
            intakePosition = .7;

        }


            rotatePostion = .3;

            if (gamepad2.b) {
                rotatePostion = .6;
            }
      //  double artPower = (0);

            if (gamepad2.touchpad_finger_1) {
                // Calculate swing arm angle (60° to 240° range)
                double swingAngle = 150 + (gamepad2.touchpad_finger_1_x * 85); // Center at 150°, ±90°
                swingAngle = Math.max(0, Math.min(300, swingAngle)); // Clamp to valid range
                swingPosition = (swingAngle - 60) / (300 - 60);


                // Calculate the articulation servo base angle
                articulationBaseAngle = swingAngle - DEFAULT_CENTER_OFFSET; // Offset by default center (90° off left edge)

            }

            // Add trim adjustment
            double articulationAngle = articulationBaseAngle + articulationTrim;

            // Clamp the articulation angle to stay within valid range
            articulationAngle = Math.max(MIN_ARTICULATION_ANGLE, Math.min(MAX_ARTICULATION_ANGLE, articulationAngle));

            // Map articulation angle to servo range (0 to 1)
            double articulationPosition = articulationAngle / MAX_ARTICULATION_ANGLE;
            robot.servoart.setPosition(articulationPosition);

// Handle trim adjustments with X and Y buttons

            if (gamepad2.right_bumper && !xPressed) {
                articulationTrim -= TRIM_INCREMENT; // Decrease trim
                xPressed = true; // Mark right bumper as pressed
            } else if (!gamepad2.right_bumper) {
                xPressed = false; // Reset right bumper press state
            }

            if (gamepad2.left_bumper && !yPressed) {
                articulationTrim += TRIM_INCREMENT; // Increase trim
                yPressed = true; // Mark Y as pressed
            } else if (!gamepad2.left_bumper) {
                yPressed = false; // Reset Y press state
            }

// Clamp the trim to ensure articulation stays within bounds
            articulationTrim = Math.max(MIN_ARTICULATION_ANGLE - articulationBaseAngle,
                    Math.min(MAX_ARTICULATION_ANGLE - articulationBaseAngle, articulationTrim));

            if (-robot.liftH.getCurrentPosition() > 2300 && -gamepad2.right_stick_y > 0) {  //1880 default value... adjust to bring in and out
                liftHPower = 0;
            }

//*******************************************************************
        //Robot Coloration Conditions and Controls
        //***********************************************************
        if (gamepad2.dpad_up){
            patternPrime = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
        }

        if (gamepad2.dpad_down){
            patternPrime = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
        }
/*
        if (robot.pole.getDistance(DistanceUnit.INCH) > 9 && robot.pole.getDistance(DistanceUnit.INCH) <14) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
        }
        else {
            pattern = patternPrime;
        }
*/
//************************************************************************

    robot.blinkinLedDriver.setPattern(patternPrime);

    robot.leftFrontDrive.setPower(front_left);
    robot.rightFrontDrive.setPower(front_right);
    robot.leftRearDrive.setPower(rear_left);
    robot.rightRearDrive.setPower(rear_right);

    robot.liftH.setPower(liftHPower);
    robot.liftV.setPower(liftVPower);
    robot.hang.setPower(hangPower);

    robot.servohang.setPosition(servohangPosition);

    robot.specimenClamp.setPosition(specimenPosition);

    robot.bucketgrab.setPosition(bucketgrabPosition);
    robot.bucketart.setPosition(bucketartPosition);
    robot.bucketrotate.setPosition(bucketrotatePosition);

    robot.servorotate.setPosition(rotatePostion);
    //robot.servoart.setPosition(artPosition);
    robot.servoswing.setPosition(swingPosition);
    robot.servointake.setPosition(intakePosition);
    }

}

double scaleInput(double dVal)  {
      double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
            0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
      
      // get the corresponding index for the scaleInput array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      } else if (index > 16) {
         index = 16;
      }
      
      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }
      
      return dScale;
   }
    
}


