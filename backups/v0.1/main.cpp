0 .__. 1    <--motors
  |  |
2 .__. 3 

// * <- falta codigo

#include "I2Cdev.h" //para usar el acelerometro
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO

boolean debug=false;

///////////REPLANTEAR ESTO,
///get_position supongo que lo que buscarás es saber si avanza hacia algun lado,
///pudiendo hacer una media o integral para esto
/////mientras que sacar la aceleracion solo tenemos que usar la funcion:
//accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

function get_position(int *x,int *y, int *z, boolean use_a=use_acc, boolean use_g=use_gyro)
{
	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	// these methods (and a few others) are also available:
		//accelgyro.getAcceleration(&ax, &ay, &az);
		//accelgyro.getRotation(&gx, &gy, &gz);

	if(use_a)
	{
		
	}
	if(use_g)
	{
		
	}
	*
}

function motor(int* motor, int* motor_pin, int motor_number, int speed)
{ //change speed of a motor or motors (motor_number=10 then all motors)
	if(motor_number!=10)
		AnalogWrite(motor_pin[motor_number], speed);
		motor[motor_number]=speed;
	else {
		for(int i=0;i<4;i++){
			AnalogWrite(motor_pin[i],speed);
			motor[motor_number]=speed;
		}
	}
}


void setup()
{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
	
	if(debug) Serial.begin(9600);//or 38400

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
	int starting_speed=180; //0-255
<	float x,y,z;
<	float no_stabilished_z=9.2 //stabilising to Z gravity level
	int motor_pin[4]={,,,};
	int motor[4]={0,0,0,0}; //array que almacena velocidad del motor actual

	//*low battery indicator
	*
	
	//*calibrate acc/gyro (durante 1 segundo?(10valores) toma un rango de valores para ver "que es reposo".
	calibrate(&x,&y,&z);
	
	//*waiting for press start button
	*

	  //*motors activated
		motor(motor, motor_pin, 10, starting_speed);
		sleep(1000);


	  //*stabilising drone
		//*GET acc/gyro values:
		get_position(&x,&y,&z); //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

<		if(z<no_stabilished_z)
		*

             //CALCULAR ALTURA DE ALGUNA MANERA (ultras/infr/acc...)
	  //*stabilising height between 8 and 12cm
		whie(height>120 || height < 80)
		{
			if(height>120) motor(motor, motor_pin, 10, starting_speed);
			if(height<80)  
		}

}