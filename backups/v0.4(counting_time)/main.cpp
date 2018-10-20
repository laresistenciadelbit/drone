0 .__. 1    <--motors
  |  |
2 .__. 3 

// * <- significa que falta codigo

const boolean debug=false;

#include "I2Cdev.h" //para usar el acelerometro
#include "MPU6050_6Axis_MotionApps20.h" //usamos DMP tambien
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 accelgyro;
//int16_t ax, ay, az;
//int16_t gx, gy, gz;
#include "dmp_vars.h"

const int motor_pin[4]={9,10,11,12}; //pines de los motores //meter en setup?
int x,y,z,gx,gy,gz, x_aux=0,y_aux=0,z_aux=0; //para no tener que crearlas cada vez que entra en get_position

int stable_speed=220; //0-255 / use gyro in z to stabilize?
int regulable_speed=10; //suma o resta velocidad en tramos de regulable_speed (5) para calibrar z
const int no_stable_gz=600;//a partir de este valor la altura no esta estabilizada

//int x_calibrated=1200,y_calibrated=0,z_calibrated=15217; //calibrado a ojo
const float Xoffset=0,Yoffset=2.75; //calibrado a ojo (a veces da 2.75, a veces 3.5)
const int no_stable_z=517 //stabilising to Z gravity level (diferencia entre la q el eje z esta estabilizado (9.8-0.6)-> (15217-14700)) / si al comparar encuentra un valor mayor a esta diferencia entonces esta desestabilizado de los ejes x o y
const int no_stable_z_limit=5217 //limit difference between normal z and unestabilished / una estabilizacion mas estricta
//const int no_stable_xy=4600  //por debajo o encima de este numero el eje x o y no esta estabilizado (0+2 o 0-2)
//const int no_stable_xy_limit=13000 //limit difference between normal xy and unestabilished 
const float no_stable_xy=8;  //por debajo o encima de este numero el eje x o y no esta estabilizado (mientras no se mueve)
const int no_stable_xy_limit=45 //estabilizacion menos estricta (durante el vuelo) 45º

unsigned long time, last_t;
void sleep(unsigned int delay_t)
{
	time=millis();
	last_t=time;
	while( last_t<(time+delay_t) )
		last_t=millis();
}

int function stabilize_z()	//usamos giroscopio
{
	do{
		accelgyro.getRotation(&gx, &gy, &gz);
		if(gz< -no_stable_gz)
			stable_speed-=regulable_speed;
		else if(gz>no_stable_gz) 
				stable_speed+=regulable_speed;
	}while(gz< -no_stable_gz || gz>no_stable_gz); //valor a partir del cual la altura no es estable
}
/* ANTERIORMENTE USABAMOS SOLO ACELEROMETRO:
void function get_acc(int *x, int *y, int *z, int iterations=25, int sleeptime=1)
{	//hace una media de "iteraciones" lecturas de la aceleracion durante un tiempo de iteracion*sleeptime
	//int x_aux=0,y_aux=0,z_aux=0; <- ahora son globales
	for(int i=0;i<iterations;i++)
		{
			accelgyro.getAcceleration(&x,&y,&z);
			if(i>0) 
			{
				x_aux=x+x_aux;
				y_aux=y+y_aux;
				z_aux=z+z_aux;
			}	else  { x_aux=x; y_aux=y; z_aux=z; }  //si es la primera vez no hace media porq las variables estan vacias.
			
			sleep(sleeptime);
		}
		x=x_aux/iterations;
		y=y_aux/iterations;
		z=z_aux/iterations;
}

void function stabilize_xy(boolean full_stabilize=true, int iterations=25, int sleeptime=1)
{
		get_acc(&x,&y,&z,iterations,sleeptime); //25 medidas con 1ms de espera (1ms por cada lectura APROX{comprobado}, total 25medida+25espera=50ms)
		while(z < z_calibrated-no_stable_z) //por debajo de no_stable_z significa que o el eje x o el y estan desestabilizados (esta a 9.2 x def.)
		{
			if(y < y_calibrated - no_stable_xy || y > y_calibrated + no_stable_xy) pitch(map(-y,-10,10,-255,255);
			if(x < x_calibrated - no_stable_xy || x > x_calibrated + no_stable_xy)  roll(map(-x,-10,10,-255,255);
			get_acc(&x,&y,&z,25,1); //25 medidas con 1ms de espera (1ms por cada lectura APROX{comprobado}, total 25+25=50ms)
		}
}
*/

void get_angle()	// 10ms por cada lectura
{
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
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
            z = ypr[2] * 180/M_PI;
    }
}

/*
void function get_angle(int *x, int *y, int *z, int iterations=25, int sleeptime=1)
{	//hace una media de "iteraciones" lecturas de la aceleracion durante un tiempo de iteracion*sleeptime
	//int x_aux=0,y_aux=0,z_aux=0; <- ahora son globales
	for(int i=0;i<iterations;i++)
		{
			dmp6(&x,&y,&z);
			if(i>0) 
			{
				x_aux=x+x_aux;
				y_aux=y+y_aux;
				z_aux=z+z_aux;
			}	else  { x_aux=x; y_aux=y; z_aux=z; }  //si es la primera vez no hace media porq las variables estan vacias.
			
			sleep(sleeptime);
		}
		x=x_aux/iterations;
		y=y_aux/iterations;
		z=z_aux/iterations;	
}
*/

void function stabilize_xy(boolean full_stabilize=true)
{
	if(full_stabilize)
	{
		do{
			get_angle();
			if(y>Yoffset+no_stable_xy || y<Yoffset-no_stable_xy) pitch(map(-y,-10,10,-255,255);
			if(x>Xoffset+no_stable_xy || x<Xoffset-no_stable_xy)  roll(map(-x,-10,10,-255,255);
		}while(y>Yoffset+no_stable_xy || y<Yoffset-no_stable_xy || x>Xoffset+no_stable_xy || x<Xoffset-no_stable_xy); //por debajo de no_stable_z significa que o el eje x o el y estan desestabilizados (esta a 9.2 x def.)
	}else{
		do{
			get_angle();
			if(y>Yoffset+no_stable_xy_limit || y<Yoffset-no_stable_xy_limit) pitch(map(-y,-10,10,-255,255);
			if(x>Xoffset+no_stable_xy_limit || x<Xoffset-no_stable_xy_limit)  roll(map(-x,-10,10,-255,255);
		}while(y>Yoffset+no_stable_xy_limit || y<Yoffset-no_stable_xy_limit || x>Xoffset+no_stable_xy_limit || x<Xoffset-no_stable_xy_limit); //por debajo de no_stable_z significa que o el eje x o el y estan desestabilizados (esta a 9.2 x def.)
	
	}
}


void function power(int motor_number, int speed)
{ //change speed of a motor
		AnalogWrite(motor_pin[motor_number], speed);
}
void function power_all(int speed)
{ //change speed of all motors
		for(int i=0;i<4;i++){
			AnalogWrite(motor_pin[i],speed);
		}
}

boolean function get_command(int *p1, int *p2) //recive los comandos por el modulo inalambrico.
{
	boolean REPLY=false;
	
		p1=;
		p2=;
	
	if (REPLY) return(true);
	else return(false);
}

void function roll(int speed) //movimiento del eje x
{
	power(1,stable_speed-(speed/2); //quitamos mitad de velocidad de la pulsada al motor 3 (para valores positivos)
	power(3,stable_speed+(speed/2); //damos mitad de velocidad de la pulsada al motor 1    (para valores positivos)
}

void function pitch(int speed)  //movimiento del eje y
{
	power(0,stable_speed-(speed/2);
	power(2,stable_speed+(speed/2);
}

void function yaw(int speed) //movimiento de giro sobre si mismo (z==gravedad) (usar l1,r1)
{
	power(0,stable_speed-(speed/2);
	power(2,stable_speed-(speed/2);
	power(1,stable_speed+(speed/2);
	power(3,stable_speed+(speed/2);
}

void function motor_x(int speed) //reduce la velocidad del eje_x del dron para moverse hacia ese lado
{
	power(0,stable_speed-(speed/2);
	power(3,stable_speed-(speed/2);
	power(1,stable_speed+(speed/2);
	power(2,stable_speed+(speed/2);
}
void function motor_y(int speed) //reduce la velocidad del eje_y del dron para moverse hacia ese lado
{
	power(0,stable_speed-(speed/2);
	power(1,stable_speed-(speed/2);
	power(2,stable_speed+(speed/2);
	power(3,stable_speed+(speed/2);
}

void setup()
{
//{acc config BEGIN}
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
	
	accelgyro.initialize();
	devStatus = accelgyro.dmpInitialize();
	//define offests here (if you want)
	accelgyro.setDMPEnabled(true);
	attachInterrupt(0, dmpDataReady, RISING);
	mpuIntStatus = accelgyro.getIntStatus();
	dmpReady = true;
	packetSize = accelgyro.dmpGetFIFOPacketSize();
//{acc config END}

	if(debug)
	{
		Serial.begin(115200);
		Serial.println("Testing device connections...");
		Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
	}
	
	for(int i=0; i<4; i++) pinMode(motor_pin[i], OUTPUT);
	
}

void loop()
{
	unsigned long time_start,time_diff; boolean time_started=false;//time1
    if (!dmpReady) return; //(acc) if programming failed, don't try to do anything
	boolean use_acc=true;
	boolean use_gyro=false;
	boolean REPLY=false;
	int p1=0,p2=0; //parametros recibidos del modulo inalambrico (motor/speed)

	//*low battery indicator
	*
	
	
	//*waiting for press start button
	while(!REPLY)
	{
		REPLY=get_command(p1,p2);
		if(REPLY)
		{
			if(p1=100) REPLY=true;
			if(p1=11) //calibramos si hemos pulsado select antes del start ( sino usa valores ideales )
			{
				sleep(30);
				for(int i=0;i<1000;i++) get_angle();
			}
		}
	}

	//*motors activated
		all_motors(stable_speed);
		sleep(1000);

	//*stabilising xy
		function stabilize_xy(true)
		stabilize_z();
	
	//once stabilised wait for orders and stabilize again:
	while(1){
		REPLY=get_command(&p1,&p2);
		if(REPLY)
		{
			switch(p1)	//ordenes que no sean de mover un motor
			{
				case 1: roll(p2);		//(+) se mueve en el eje x
				break;
				case 2: pitch(p2);		//(+) se mueve en el eje y
				break;
				case 3: power_all(p2);	// aumenta o reduce vel. de todos los motores
				break;
				case 4: yaw(p2);		// gira sobre si mismo
				break;
				case 5: motor_x(p2);	//(x) se mueve sobre el eje x
				break;
				case 6: motor_y(p2);	//(x) se mueve sobre el eje y
				break;
				
				case 11: x_calibrated=0; y_calibrated=0; z_calibrated=9.8;	
				break;	//usa los valores ideales de estabilizacion en vez de los calibrados (si hemos pulsado select antes del start al arrancar)
				
				default:
*				case 100: stop();
			}
		}
	//stabilize:
		if(!time_started){	//contamos tiempo para estabilizar estricta o no estrictamente.
			time_start = millis();			
			time_started=true;				
		}				
		time_diff = millis() - time_start;	
		if(time_diff<1200)stabilize_xy(false)//stabilize dron with {no_stable_xy} 
		else { //If not receive command in 1.5 seconds: stabilize drone with {no_stable_xy_limit}
			stabilize_xy(true)
			time_started=false;
			time_start = millis();
		}
	//else CONTAR 30 SEGUNDOS Y VOLVER A LA BASE (*ToDo) <- de momento contar 60 segundos quieto y luego bajar
	}

}