//functions.cpp
	void sleep(unsigned int);
	boolean is_stable();
	void stabilize_xy(int);
	void stabilize_xyz(int);
	void power(int);
	void power_all(int, boolean);
	void stop();
	boolean get_command(int *, int *);
	void roll(int, boolean);
	void pitch(int, boolean);
	void yaw(int);
	void motor_x(int);
	void motor_y(int);

//MPU_functions.cpp
	int get_z_acc();
	int stabilize_z();
	void get_angle();
