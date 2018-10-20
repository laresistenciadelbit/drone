void sleep(unsigned int delay_t)
{
	time=millis();
	last_t=time;
	while( last_t<(time+delay_t) )
		last_t=millis();
}

boolean is_stable()
{
	z=get_z_acc();
	get_angle();
	if (y>Yoffset+no_stable_xy_limit || y<Yoffset-no_stable_xy_limit || x>Xoffset+no_stable_xy_limit || x<Xoffset-no_stable_xy_limit || z< -no_stable_z || z>no_stable_z) //aumentamos el limite +-100 en eje z
		 return(false);
	else return(true);
}

void stabilize_xy(int no_stable_xy=45)
{
	//do{
		get_angle();
		if(y>Yoffset+no_stable_xy || y<Yoffset-no_stable_xy) pitch(regulable_speed,false);
		if(x>Xoffset+no_stable_xy || x<Xoffset-no_stable_xy)  roll(regulable_speed,false);
	//}while(y>Yoffset+no_stable_xy || y<Yoffset-no_stable_xy || x>Xoffset+no_stable_xy || x<Xoffset-no_stable_xy);
}

void stabilize_xyz(int no_stable_xy=45)
{
	do{
		stabilize_xy(no_stable_xy);
		stabilize_z();
	}while( (y>Yoffset+no_stable_xy) || (y<Yoffset-no_stable_xy) || (x>Xoffset+no_stable_xy) || (x<Xoffset-no_stable_xy) || (z< -no_stable_z) || (z>no_stable_z) );
}

void power(int motor_number)
{ //put speed to a motor
	if(speed[motor_number]>255)		//we verify that it doesnt over speed it
		speed[motor_number]=255;
		
	analogWrite(motor_pin[motor_number], speed[motor_number]);
}
void power_all(int add, boolean abs=true)	//abs->absolute, if true put speed, if false add speed.
{ //add speed to all motors				//we verify that it doesnt over speed it
	for(int i=0;i<4;i++){
		if(abs) speed[i]=add;
		else	speed[i]+=add;
		power(i);
	}
}

void stop()			/////////////si al apagar por ejemplo, se para antes de tiempo, significa que ha obtenido un valor de 
{
	stabilize_xyz(no_stable_xy_strict); //estabilizamos primero
	sleep(100);	//esperamos por si los valores de z no son estables aun
	do{
		z=get_z_acc();
		power_all(-regulable_speed);
		sleep(500);
	}while(z>no_stable_z || speed[0]+speed[1]+speed[2]+speed[3]<40);	//si no hemos mapeado, por debajo de 80 probablemente no se muevan los motores. Asi que se sale si los motores se han parado ya y el acc no ha medido bien, o no ha llegado al suelo y se ha parado.
}


void roll(int add, boolean abs=true) //movimiento del eje x 		//abs->absolute, if true put speed, if false add speed.
{
	if(abs)
	{
	 speed[1]=stable[1]-(add/2);	//velocidad estable - velocidad a�adida/2 en motor 1
	 speed[3]=stable[3]+(add/2);
	}else{
	 speed[1]-=(add/2);	//escribimos la nueva velocidad en el vector
	 speed[3]+=(add/2);
	}
	power(1); //quitamos mitad de velocidad de la pulsada al motor 3 (para valores positivos)
	power(3); //damos mitad de velocidad de la pulsada al motor 1    (para valores positivos)
}

void pitch(int add, boolean abs=true)  //movimiento del eje y		//abs->absolute, if true put speed, if false add speed.
{
	if(abs)
	{
	 speed[0]=stable[0]-(add/2);
	 speed[2]=stable[2]+(add/2);
	}else{
	 speed[0]-=(add/2);
	 speed[2]+=(add/2);
	}
	power(0);
	power(2);
}

void yaw(int add) //movimiento de giro sobre si mismo (z==gravedad) (usar l1,r1)
{
	 speed[0]=stable[0]-(add/2);
	 speed[2]=stable[2]-(add/2);
	 speed[1]=stable[1]+(add/2);
	 speed[3]=stable[3]+(add/2);
	power(0);
	power(2);
	power(1);
	power(3);
}

void motor_x(int add) //reduce la velocidad del eje_x del dron para moverse hacia ese lado
{
	 speed[0]=stable[0]-(add/2);
	 speed[3]=stable[3]-(add/2);
	 speed[1]=stable[1]+(add/2);
	 speed[2]=stable[2]+(add/2);
	power(0);
	power(3);
	power(1);
	power(2);
}
void motor_y(int add) //reduce la velocidad del eje_y del dron para moverse hacia ese lado
{
	 speed[0]=stable[0]-(add/2);
	 speed[1]=stable[1]-(add/2);
	 speed[2]=stable[2]+(add/2);
	 speed[3]=stable[3]+(add/2);
	power(0);
	power(1);
	power(2);
	power(3);
}


//Mirf

struct paquetes {
   unsigned long  time;
   int c; //command
   int s; //subcommand
} paquete;

boolean get_command(int *p1, int *p2) //recive los comandos por el modulo inalambrico.
{
  boolean REPLY=false;
  
  byte data[Mirf.payload];
   
  if(!Mirf.isSending() && Mirf.dataReady()){
     if(debug)Serial.println("Got packet");
    Mirf.getData((byte *) &paquete);  //coge paquete del buffer
     if(debug)Serial.println(paquete.c);
     if(debug)Serial.println(paquete.s);
    Mirf.setTADDR((byte *)"cli1");    //responde
    Mirf.send((byte *) &paquete.time);//devuelve solo el tiempo
     if(debug)Serial.println("Reply sent.");
  }

  *p1=paquete.c;
  *p2=paquete.s;
  
  if (REPLY) return(true);
  else return(false);
}
