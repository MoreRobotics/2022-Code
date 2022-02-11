// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;


public class Hood extends Servo {

  double m_speed;
  double m_length;
  double setPos;
  double curPos;

  double lastTime = 0;

  /** Creates a new Hood. */
  public Hood(int channel, int length, int speed) {
    super(channel);
    setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    m_length = length;
    m_speed = speed;

  }

  // moves actuator to desired position
  public void setPosition(double setpoint){
    setPos = MathUtil.clamp(setpoint, 0, m_length);
    setSpeed((setPos/m_length *2)-1);
  }

  //Updates the current position of the actuator
  public void updateCurPos(){
    double dt = Timer.getFPGATimestamp() - lastTime;
    if (curPos > setPos + m_speed *dt){
    curPos -= m_speed *dt;
    } else if(curPos < setPos - m_speed *dt){
    curPos += m_speed *dt;
    }else{
    curPos = setPos;
    }
  }

  
  public double getPosition(){

    return curPos;
    
  }

  //Detects when the actuator has moved to the desired position
  public boolean isFinished(){
    lastTime = 0;
    return curPos == setPos;
  }
}
