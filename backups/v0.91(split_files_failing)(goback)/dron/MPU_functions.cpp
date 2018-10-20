int get_z_acc()
{
  while (!mpuInterrupt && fifoCount < packetSize) {}
    mpuInterrupt = false;
    mpuIntStatus = accelgyro.getIntStatus();
    fifoCount = accelgyro.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        accelgyro.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();
        accelgyro.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
            accelgyro.dmpGetQuaternion(&q, fifoBuffer);
            accelgyro.dmpGetGravity(&gravity, &q);
      accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            accelgyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  }
  return(aaWorld.x);
}


int stabilize_z()	//usamos giroscopio (ahora usamos cambios de gyro en x e y si los angulos son estables) +/-2000
{
  int z=0;
	//do{
		z=get_z_acc();
		
		if(z < -no_stable_z)
			power_all(+regulable_speed,false);
		else if(z > no_stable_z) 
			power_all(-regulable_speed,false);
	//}while(z< -no_stable_z || z>no_stable_z); //valor a partir del cual la altura no es estable
}

void get_angle()	// 10ms por cada lectura
{
    while (!mpuInterrupt && fifoCount < packetSize) {}
    mpuInterrupt = false;
    mpuIntStatus = accelgyro.getIntStatus();
    fifoCount = accelgyro.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        accelgyro.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();
        accelgyro.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
            accelgyro.dmpGetQuaternion(&q, fifoBuffer);
            accelgyro.dmpGetGravity(&gravity, &q);
            accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
            x = ypr[0] * 180/M_PI;
            y = ypr[1] * 180/M_PI;
            //z = ypr[2] * 180/M_PI; //quitamos z ya que no nos interesa la informacion de los grados en ese eje para la estabilizacion
    }
}
