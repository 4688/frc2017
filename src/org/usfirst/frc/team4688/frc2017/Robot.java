// SAINTS BOT 2017

package org.usfirst.frc.team4688.frc2017;

import edu.wpi.first.wpilibj.*;

import com.ctre.*;

public class Robot extends IterativeRobot
{
	// Joystick constants
	private final int JOYSTICK_USB = 0;
	private final int X_AXIS = 4;
	private final int Y_AXIS = 1;
	private final double X_FACTOR = 1.0;
	private final double Y_FACTOR = -1.0;
	private final int BALL_AXIS = 3;
	private final int CLIMB_REV_AXIS = 2;
	private final int CLIMB_BTN = 5;
	private final int SHOOT_BTN = 6;
	private final int FLICK_BTN = 2;
	private final int TORTILLA_BTN = 1;
	
	// Output indices
	private final int LFM_CAN = 1;
	private final int LRM_CAN = 3;
	private final int RFM_CAN = 2;
	private final int RRM_CAN = 4;
	private final int BALL_PWM = 4;
	private final int SHOOT_CAN = 5;
	private final int FLICK_PCM = 0;
	private final int CLIMB_PWM = 6;
	private final int TORTILLA_PWM = 5;
	
	// Miscellaneous constants
	private final double DEADBAND = 0.04;
	private final int SHOOT_INTERVAL = 3;
	private final double DRIVE_FACTOR = 1.0;
	private final double SHOOTER_SPD = 0.9;
	
	// Robot components
	private Joystick js;
	private CANTalon lfm, lrm, rfm, rrm, shoot;
	private VictorSP ball, tortilla;
	private Spark climb;
	private Solenoid flick;
	private DigitalInput tortillaSw, auto1, auto2, foo;
	private ADXRS450_Gyro gyro;
	
	// Other variables
	private int shotTimer = 0;
	private boolean shotActive = false;
	private int autoTimer = 0;
	private int autoRoutine = -1;
	private double gyroCorrection = 0.25;
	private double encZero = 0.0;
	private int doneDriveFwd = 0;
	private int doneTurn = 0;
	private int doneDriveB = 0;
	private boolean gyroReset = false;
	
	private void setLSpd(double spd)
	{
		if (Math.abs(spd) > DEADBAND)
		{
			this.lfm.set(spd);
			this.lrm.set(spd);
		}
		else
		{
			this.lfm.set(0.0);
			this.lrm.set(0.0);
		}
	}
	
	private void setRSpd(double spd)
	{
		if (Math.abs(spd) > DEADBAND)
		{
			this.rfm.set(spd);
			this.rrm.set(spd);
		}
		else
		{
			this.rfm.set(0.0);
			this.rrm.set(0.0);
		}
	}
	
	public void robotInit()
	{
		// Joystick
		this.js = new Joystick(JOYSTICK_USB);
		
		// Drive motors
		this.lfm = new CANTalon(LFM_CAN);
		this.lrm = new CANTalon(LRM_CAN);
		this.rfm = new CANTalon(RFM_CAN);
		this.rrm = new CANTalon(RRM_CAN);
		
		// Ball stuff
		this.ball = new VictorSP(BALL_PWM);
		this.shoot = new CANTalon(SHOOT_CAN);
		this.flick = new Solenoid(FLICK_PCM);
		this.tortilla = new VictorSP(TORTILLA_PWM);
		
		// Configure drive motors
		this.lfm.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		this.lrm.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		this.rfm.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		this.rrm.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		this.shoot.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		
		// Rope climber
		this.climb = new Spark(CLIMB_PWM);
		
		// Tortilla switch
		this.tortillaSw = new DigitalInput(8);
		
		// Autonomous routine
		this.auto1 = new DigitalInput(0);
		this.auto2 = new DigitalInput(1);
		this.autoRoutine = ((this.auto1.get() ? 1 : 0) << 1) + (this.auto2.get() ? 1 : 0);
		this.foo = new DigitalInput(7);
		
		// GYROSCOPE
		this.gyro = new ADXRS450_Gyro();
		
		// ENCODER
		this.lrm.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		
		// Camera magic
		CameraServer.getInstance().startAutomaticCapture();
	}
	
	public void disabledInit()
	{}
	
	public void disabledPeriodic()
	{}
	
	public void teleopInit()
	{
		this.encZero = this.lrm.getEncPosition();
		this.lfm.enableBrakeMode(false);
		this.lrm.enableBrakeMode(false);
		this.rfm.enableBrakeMode(false);
		this.rrm.enableBrakeMode(false);
	}
	
	public void teleopPeriodic()
	{
		// Drive
		double x = this.js.getRawAxis(X_AXIS) * X_FACTOR;
		double y = this.js.getRawAxis(Y_AXIS) * Y_FACTOR;
		double l = 0.0, r = 0.0;
		double c = this.js.getRawButton(CLIMB_BTN) ? 0.5 : 1.0;
		if (Math.abs(x) < DEADBAND)
		{
			// Straight driving
			l = y * DRIVE_FACTOR * c;
			r = -y * DRIVE_FACTOR * c;
		}
		else if (Math.abs(y) < DEADBAND)
		{
			// Pivot stationary
			l = x * DRIVE_FACTOR * c;
			r = x * DRIVE_FACTOR * c;
		}
		else if (x < -DEADBAND || x > DEADBAND)
		{
			// Drive while turning
			l = (x + y) * DRIVE_FACTOR * c;
			r = (x - y) * DRIVE_FACTOR * c;
		}
		this.setLSpd(l);
		this.setRSpd(r);
		
		// Ball elevator
		this.ball.set(this.js.getRawAxis(BALL_AXIS));
		
		// Ball shooter
		if (this.js.getRawButton(SHOOT_BTN))
		{
			this.shoot.set(SHOOTER_SPD);
			
			if (this.js.getRawButton(FLICK_BTN)) this.shotActive = true;
		}
		else
		{
			this.shoot.set(0.0);
		}
		if (this.shotActive)
		{
			this.shotTimer += 1;
			this.flick.set(true);
			if (this.shotTimer >= SHOOT_INTERVAL)
			{
				this.shotTimer = 0;
				this.shotActive = false;
			}
		}
		if (!this.shotActive) this.flick.set(false);
		
		// Tortilla
		if (this.js.getRawButton(TORTILLA_BTN) || this.tortillaSw.get())
		{
			this.tortilla.set(-1.0);
		}
		else
		{
			this.tortilla.set(0.0);
		}
			
		// Climber
		if (this.js.getRawButton(CLIMB_BTN))
		{
			this.climb.set(-Math.sqrt(Math.abs(this.js.getRawAxis(CLIMB_REV_AXIS))));
		}
		else
		{
			this.climb.set(0.0);
		}
		
		System.out.println(
				"Teleop @--" +
				"; Y=" + this.js.getRawAxis(Y_AXIS) +
				", X=" + this.js.getRawAxis(X_AXIS) +
				"; LFM=" + this.lfm.get() +
				", LRM=" + this.lrm.get() +
				", RFM=" + this.rfm.get() +
				", RRM=" + this.rrm.get()
			);
	}
	
	public void autonomousInit()
	{
		this.autoTimer = 0;
		this.autoRoutine = ((this.auto1.get() ? 1 : 0) << 1) + (this.auto2.get() ? 1 : 0);
		this.gyro.reset();
		this.encZero = this.lrm.getEncPosition();
		this.doneDriveFwd = 0;
		this.doneTurn = 0;
		this.doneDriveB = 0;
		this.gyroReset = false;
	}
	
	public void autonomousPeriodic()
	{
		int sec = 50;
		String routineStr = "Unset (THIS IS AN ERROR!)";
		
		if (this.autoRoutine == 0) // look pretty (?)
		{
			routineStr = "Nothing";
			this.setLSpd(0.0);
			this.setRSpd(0.0);
		}
		else if (this.autoRoutine == 1) // deliver gear to the left
		{
			routineStr = "GearL";
			double angle = this.gyro.getAngle();
			double distance = (this.lrm.getEncPosition() - this.encZero) / -1024 / Math.PI;
			
			double driveARotations = 99 / (6 * Math.PI);
			double turnAngle = 54;
			
			if (distance <= driveARotations && this.doneDriveFwd <= 0 && this.doneTurn <= 0)
			{
				double lSpd = 0.36;
				double rSpd = Math.min(Math.max(-lSpd - lSpd * angle * gyroCorrection, lSpd * -2), lSpd * 2);
				this.setLSpd(lSpd);
				this.setRSpd(rSpd);
			}
			else if (distance > driveARotations && this.doneDriveFwd <= 0.5*sec && this.doneTurn <= 0)
			{
				this.lfm.enableBrakeMode(true);
				this.lrm.enableBrakeMode(true);
				this.rfm.enableBrakeMode(true);
				this.rrm.enableBrakeMode(true);
				this.doneDriveFwd += 1;
				this.setLSpd(0.0);
				this.setRSpd(0.0);
			}
			else if (this.doneDriveFwd > 0.5*sec && this.doneTurn <= 0)
			{
				if (angle < turnAngle)
				{
					this.lfm.enableBrakeMode(false);
					this.lrm.enableBrakeMode(false);
					this.rfm.enableBrakeMode(false);
					this.rrm.enableBrakeMode(false);
					this.setLSpd(0.36);
					this.setRSpd(0.36);
				}
				else
				{
					this.lfm.enableBrakeMode(true);
					this.lrm.enableBrakeMode(true);
					this.rfm.enableBrakeMode(true);
					this.rrm.enableBrakeMode(true);
					this.doneTurn += 1;
					this.setLSpd(0.0);
					this.setRSpd(0.0);
				}
			}
			else if (this.doneDriveFwd > 0.5*sec && this.doneTurn <= 0.5*sec)
			{
				this.lfm.enableBrakeMode(true);
				this.lrm.enableBrakeMode(true);
				this.rfm.enableBrakeMode(true);
				this.rrm.enableBrakeMode(true);
				this.doneTurn += 1;
				this.setLSpd(0.0);
				this.setRSpd(0.0);
			}
			else if (!this.gyroReset && this.doneTurn > 0.5*sec)
			{
				this.gyro.reset();
				this.gyroReset = true;
			}
			else if (this.gyroReset && this.doneTurn > 0.5*sec && this.doneDriveB <= 2*sec)
			{
				this.lfm.enableBrakeMode(false);
				this.lrm.enableBrakeMode(false);
				this.rfm.enableBrakeMode(false);
				this.rrm.enableBrakeMode(false);
				this.doneDriveB += 1;
				double lSpd = 0.18;
				double rSpd = Math.min(Math.max(-lSpd - lSpd * angle * gyroCorrection, lSpd * -2), lSpd * 2);
				this.setLSpd(lSpd);
				this.setRSpd(rSpd);
			}
			else
			{
				this.setLSpd(0.0);
				this.setRSpd(0.0);
			}
		}
		else if (this.autoRoutine == 2) // deliver gear to the right
		{
			routineStr = "GearR";
		}
		else if (this.autoRoutine == 3) // deliver gear straight forward
		{
			routineStr = "Forward";
			double angle = this.gyro.getAngle();
			if (this.autoTimer < 7*sec) // go straight forward
			{
				double lSpd = 0.24;
				double rSpd = Math.min(Math.max(-lSpd - lSpd * angle * gyroCorrection, lSpd * -2), lSpd * 2);
				this.setLSpd(lSpd);
				this.setRSpd(rSpd);
			}
			else if (this.autoTimer >= 7*sec && this.autoTimer < 12*sec) // wait a bit
			{
				this.setLSpd(0.0);
				this.setRSpd(0.0);
			}
			else if (this.autoTimer >= 12*sec && this.autoTimer < 15*sec) // back out of there
			{
				this.setLSpd(-0.15);
				this.setRSpd(0.15);
			}
			else // stop
			{
				this.setLSpd(0.0);
				this.setRSpd(0.0);
			}
		}
		
		// Tick
		this.autoTimer += 1;
		
		System.out.println(
			routineStr +
			" @" + this.autoTimer +
			"; LFM=" + this.lfm.get() +
			", LRM=" + this.lrm.get() +
			", RFM=" + this.rfm.get() +
			", RRM=" + this.rrm.get()
		);
	}
	
	public void testInit()
	{}
	
	public void testPeriodic()
	{
		System.out.println(((this.auto1.get() ? 1 : 0) << 1) + (this.auto2.get() ? 1 : 0));
	}
}