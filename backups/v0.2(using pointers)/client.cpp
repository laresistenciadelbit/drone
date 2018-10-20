USB Usb;
XBOXRECV XboxRCV(&Usb);
long int leftStickX,leftStickY;

/*						comando
pad1_x: roll		-> 1
pad1_y: pitch		-> 2
l2,r2: power_all	-> 3
l1,r1: yaw(giro sobre si mismo) ó eje x de pad2	 -> 4
pad2_x: motores en eje x -> 5
pad2_y: motores en eje x -> 6
select: calibra al iniciar -> 11
start: arranca o para motores -> 100
*/

void setup()
{
   Serial.begin(9600); //OPT
      // Halt program until shield is connected
  // USB.Init will return -1 for shield disconnected and 0 if connected
  if (Usb.Init() == -1)    // If USB shield did not initialise for whatever reason...
  {
	Serial.println("Usb shield disconnected");
    while(1); //halt as long as shield is reported disconnected
  }
}


void loop()
{
	int min_limit_stick=15; //velocidad minima (sensibilidad) para que el stick lo mande al dron (0-255).
	Usb.Task();
  if(XboxRCV.Xbox360Connected[0]) {

    leftStickX = map(XboxRCV.getAnalogHat(LeftHatX,0), -32768, 32767, -255, 255);  
	if(leftStickX > min_limit_stick || leftStickX < -min_limit_stick)
		send_to_quad(1,leftStickX); //(comando 1 (roll), velocidad)
  
      
    leftStickY = map(XboxRCV.getAnalogHat(LeftHatY,0), -32768, 32767, -255, 255);  
    if(leftStickY > min_limit_stick || leftStickY < -min_limit_stick)
		send_to_quad(2,leftStickY); //(comando 2 (pitch), velocidad)


	if ( XboxRCV.getButtonClick(L2, 0) ) //power_all_-		//PROBABLEMENTE HAYA QUE HACER OTRO MAP DEL L2/R2
	{ 
		send_to_quad(3,-128);
	}
	if ( XboxRCV.getButtonClick(R2, 0) ) //power_all_+		//PROBABLEMENTE HAYA QUE HACER OTRO MAP DEL L2/R2
	{ 
		send_to_quad(3,128);
	}
  
	if ( XboxRCV.getButtonClick(L1, 0) ) //yaw_left
	{ 
		send_to_quad(4,-128);
	}
	if ( XboxRCV.getButtonClick(R1, 0) ) //yaw_right
	{ 
		send_to_quad(4,128);
	}	
	
	rightStickX = map(XboxRCV.getAnalogHat(rightHatX,0), -32768, 32767, -255, 255);  
	if(rightStickX > min_limit_stick || rightStickX < -min_limit_stick)
		send_to_quad(5,-rightStickX); //(comando 5 [reduce la velocidad del motor para ir en esa direccion (eje_x)], velocidad)
  
      
    rightStickY = map(XboxRCV.getAnalogHat(rightHatY,0), -32768, 32767, -255, 255);  
    if(rightStickY > min_limit_stick || rightStickY < -min_limit_stick)
		send_to_quad(6,-rightStickY); //(comando 6 [reduce la velocidad del motor para ir en esa direccion (eje_y)], velocidad)
	
	
	if ( XboxRCV.getButtonClick(start, 0) ) //arranca motores y estabiliza
	{ 
		send_to_quad(100,128);
	}	
	if ( XboxRCV.getButtonClick(select, 0) ) //calibra los sensores / desactiva la calibracion real y usa la ideal (0,0,9.8)
	{ 
		send_to_quad(11,128);
	}	
	
	
	
 }
}