//*prueba tiempo en leer 100 medidas del acelerometro (y hacer media) (usar milis)
//diferencia entre acc y gyro
//libreria inalambrica a usar?

0 .__. 1    <--motors
  |  |
2 .__. 3 

// * <- falta codigo

boolean debug=false;
int motor_pin[4]={9,10,11,12}; //pines de los motores //meter en setup?
int x_aux=0,y_aux=0,z_aux=0; //para no tener que crearlas cada vez que entra en get_position
#include "I2Cdev.h" //para usar el acelerometro
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO

unsigned long time, last_t;
void sleep(unsigned int delay_t)
{
	time=millis();
	last_t=time;
	while( last_t<(time+delay_t) )
		last_t=millis();
}

///supongo que lo que buscas es saber si avanza hacia algun lado,
///pudiendo hacer una media o integral para esto (o buscar en la docu del 6050)
/*
void function get_move(int *x,int *y, int *z, boolean use_a=use_acc, boolean use_g=use_gyro)
{
	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	// these methods (and a few others) are also available:
		//accelgyro.getAcceleration(&ax, &ay, &az);
		//accelgyro.getRotation(&gx, &gy, &gz);

*	if(use_a && use_g) //valores del 1 al 10? mapear si no //z en el gyro será 0 asi que quitar la media o sumarle 9.81 a gz.
	{
		x=(ax+gx)/2; y=(ay+gy)/2; z=(az+gz)/2;
	}else{
*		if(use_a) //valores del 1 al 10? mapear si no
		{
			x=ax; y=ay; z=az;
		}
*		if(use_g) //valores del 1 al 10? mapear si no
		{
			x=gx; y=gy; z=gz;
		}
	}	
}
*/

void function get_position(int *x,int *y, int *z, int iterations=25, int sleeptime=1)
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
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
	
	for(int i=0; i<4; i++) pinMode(motor_pin[i], OUTPUT);
	
	if(debug) Serial.begin(9600);

	accelgyro.initialize();
	
	if(debug)
	{
		Serial.println("Testing device connections...");
		Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
	}
}

void loop()
{
	boolean use_acc=true;
	boolean use_gyro=false;
	boolean REPLY=false;
	int starting_speed=180; //0-255
*	int stable_speed=0; //use gyro in z to stabilish?
	int regulate_speed=5; //suma o resta velocidad en tramos de regulate_speed (5)
	int x,y,z;
	int x_calibrated=0,y_calibrated=0,z_calibrated=9.8;
	int no_stable_z=517 //stabilising to Z gravity level (diferencia entre la q el eje z esta estabilizado (9.8-0.6)-> (15217-14700))
	int no_stable_z_limit=5217 //limit difference between normal z and unestabilished 
	int no_stable_xy=4600  //por debajo o encima de este número el eje x o y no esta estabilizado (0+2 o 0-2)
	int no_stable_xy_limit=13000 //limit difference between normal xy and unestabilished 
	//int motor_pin[4]={,,,}; variable global
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
			if(p1=11) //calibramos si hemos pulsado select antes del start ( sino usa valores ideales (0,0,9.8) )
			{
				sleep(50);
				get_position(&x_calibrated,&y_calibrated,&z_calibrated,50,6); //50 medidas, 5ms de espera en cada medida (tot=350ms)
				REPLY=false;
			}
		}
	}

	//*motors activated
		all_motors(starting_speed);
		sleep(1000);


	//*stabilising drone
		get_position(&x,&y,&z,25,1); //25 medidas con 1ms de espera (1ms por cada lectura APROX{comprobado}, total 25medida+25espera=50ms)
		while(z < z_calibrated-no_stable_z) //por debajo de no_stable_z significa que o el eje x o el y estan desestabilizados (esta a 9.2 x def.)
		{
			if(y < y_calibrated - no_stable_xy || y > y_calibrated + no_stable_xy) pitch(map(-y,-10,10,-255,255);
			if(x < x_calibrated - no_stable_xy || x > x_calibrated + no_stable_xy)  roll(map(-x,-10,10,-255,255);
			get_position(&x,&y,&z,25,1); //25 medidas con 1ms de espera (1ms por cada lectura APROX{comprobado}, total 25+25=50ms)
		}

	/*stabilize height between 8 and 12cm		//CALCULAR ALTURA DE ALGUNA MANERA (ultras/infr/acc...)
		while(height>120 || height < 80)
		{
			if(height>120) starting_speed -= regulate_speed;
			if(height<80)  starting_speed += regulate_speed;
			all_motors(starting_speed);
		}
	*/

*  podemos usar el gyro para ver si la altura asciende o no en el tiempo.

	//once stabilised wait for orders and stabilize again:
	while(1){
		REPLY=get_command(p1,p2);
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
		//stabilize: (stabilish dron with {no_stable_z_limit})
		
		//If not receive command in 3 seconds: stabilish dron with {no_stable_z}
		
		//else CONTAR 30 SEGUNDOS Y VOLVER A LA BASE (*ToDo)
	}

}