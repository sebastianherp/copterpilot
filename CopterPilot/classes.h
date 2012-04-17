
class Receiver
{
public:
    Receiver();
	void init(int pin);
    void update();  
	unsigned int get(char channel);
	volatile unsigned int rx_values[LASTCHANNEL];
	
private:
	int _pin;
	char rx_channel;
	unsigned int rx_start;
	unsigned int rx_duration;

};

class Motors
{
public:
    Motors();
	void init(int pin1, int pin2, int pin3, int pin4);
	void handleInterrupt();
	void setYaw(int value) { _yaw = value; };
	void setRoll(int value) { _roll = value; };
	void setPitch(int value) { _pitch = value; };
	void setThrottle(float value) { _throttle = value; };
	
	void process(float * ypr);
	void write();
	
	unsigned int getCalculated(char motor) { return motor_calculated_values[motor]; };
	unsigned int get(char motor) { return motor_values[motor]; };
	void setAll(unsigned int value);
	
private:
	int _pin1, _pin2, _pin3, _pin4;
	int _yaw, _pitch, _roll;
	float _throttle;

	unsigned int motor_calculated_values[LASTMOTOR];  
	unsigned int motor_min_values[LASTMOTOR];  
 	unsigned int motor_max_values[LASTMOTOR];   
 	volatile unsigned int motor_values[LASTMOTOR];   

	void set(char motor, unsigned int value) { motor_values[motor] = value; };
};

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

class FreeIMU
{
  public:
    FreeIMU();
    void init();
    void init(bool fastmode);
    void getRawValues(int * raw_values);
    void update();
    void getQ(float * q);
    void getEuler(float * angles);
    void getYawPitchRoll(float * ypr);
	void zeroCalibrateGyro(unsigned int totSamples, unsigned int sampleDelayMS);
    
	BMP085 baro;
    LSM303DLH magn;
    IMU3000 imu;
	
  float vx, vy, vz, wx, wy, wz;
    vector g_bias;
	vector g_scale;
	float gxAdj, gyAdj, gzAdj;
    float q0, q1, q2, q3; // quaternion elements representing the estimated orientation
	
  private:
    int* raw_acc, raw_gyro, raw_magn;
    void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    float iq0, iq1, iq2, iq3;
    float exInt, eyInt, ezInt;  // scaled integral error
    long lastUpdate, now; // sample period expressed in milliseconds
    float halfT; // half the sample period expressed in seconds
    int startLoopTime;
	
	float maxOfVector(vector v);
	
};

float invSqrt(float number);


class Pilot
{
public:
    Pilot();
    void init();
    void imu_update();
    void mag_update();
    void alt_update();    
    void power_update();
    void receiver_update();    
    void adjustMotorsPlus();
    void processInput();
    Receiver _receiver;
    Motors _motors;
    FreeIMU _imu;
    float ampere;
    float ampere_hours;
    float voltage;    
    
private:
    float quat[4];
    float ypr[3];
    int ampere_ad, voltage_ad;
    long last_update;
};
