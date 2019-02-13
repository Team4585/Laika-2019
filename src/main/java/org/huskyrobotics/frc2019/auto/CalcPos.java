package org.huskyrobotics.frc2019.auto;
import edu.wpi.first.wpilibj.Encoder;

class CalcPos{
  double[] getPos(){
    return (new double [] {0});
  }
  void addData(double step){
  
  }
  void initPos(){
    Encoder lEncoder = new Encoder(5, 5);
    Encoder rEncoder = new Encoder(6, 6);
  }
}
