#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

Encoder myEnc(4, 5);
#define encoder_max_pulse 1024
bool running = 0;
// unsigned long lastTime = 0;
// float lastPosition = 0;
// float vel = 0;
float DT_vel_cal_sec=1e-3;
unsigned long DT_vel_cal= (unsigned long)(DT_vel_cal_sec*1000000);
float tau=10e-3;
#define MAX (PI)
#define MIN (-PI)
char b;
unsigned long last_vel_calc;
float omega_prev = 0;
float omega = 0;
float u_prev = 0;
unsigned long lastTimeLoop = 0;

unsigned long lastTimeVel = 0;
int32_t n_pulse_prev;
#define loopPeriod 0  

#define MAX_TIME_VEL_CALC 40000



float movingAverage(float value) {
  const byte nvalues = 50;             // Moving average window size

  static byte current = 0;            // Index for current value
  static byte cvalues = 0;            // Count of values read (<= nvalues)
  static float sum = 0;               // Rolling sum
  static float values[nvalues];

  sum += value;

  // If the window is full, adjust the sum by deleting the oldest value
  if (cvalues == nvalues)
    sum -= values[current];

  values[current] = value;          // Replace the oldest with the latest

  if (++current >= nvalues)
    current = 0;

  if (cvalues < nvalues)
    cvalues += 1;

  return sum/cvalues;
}


void setup() {
  Serial.begin(2000000);
}

void reset_all() {
  lastTimeLoop = micros();
  lastTimeVel = lastTimeLoop;
  u_prev = 0;
  omega = 0;
  myEnc.write(512);
  n_pulse_prev = myEnc.read();
}

void loop() {
  Serial.flush();
  if (!running) {
    if (Serial.available() > 0) {
      b = Serial.read();
      if (b == 's') {
        Serial.flush();
        running = true;
        reset_all();
        return;
      }
    }
    return;
  } else {
    if (Serial.available() > 0) {
      b = Serial.read();
      if (b == 'a') {
        Serial.flush();
        running = false;
        return;
      }
      if (b == 's') {
        Serial.flush();
        running = true;
        reset_all();
        return;
      }
    }
  }
  
  unsigned long currentTime = micros();
  if ((currentTime - lastTimeLoop) < loopPeriod) {
    return;
  }
  lastTimeLoop = currentTime;

  int32_t n_pulse = myEnc.read();
  float theta = ((float)(n_pulse % encoder_max_pulse)) / encoder_max_pulse * 2.0 * PI;
  if (theta > MAX) {
    theta -= MAX - MIN;
  } else if (theta < MIN) {
    theta += MAX - MIN;
  }

  // if (n_pulse > n_pulse_prev + 1 || n_pulse < n_pulse_prev - 1) {
  //   if (theta * u_prev < 0 && fabs(theta - u_prev) > MAX / 2) {
  //     if (u_prev > 0) {
  //       u_prev -= MAX - MIN;
  //     } else {
  //       u_prev += MAX - MIN;
  //     }
  //   }
  //   float DT_vel_cal_sec = (currentTime - lastTimeVel) / 1000000.0;
  //   omega = (theta - u_prev) / DT_vel_cal_sec;
  //   u_prev = theta;
  //   // omega_prev = omega;
  //   //   last_vel_calc = micros();
  //   lastTimeVel = currentTime;
  //   n_pulse_prev = n_pulse;
  // } else if ((currentTime - lastTimeVel) >= MAX_TIME_VEL_CALC) {
  //   omega = 0;
  //   u_prev = theta;
  //   // omega_prev = omega;
  //   //   last_vel_calc = micros();
  //   // lastTimeVel = currentTime;
  // }


  if((currentTime - last_vel_calc) >= DT_vel_cal )
  {
    
    if (theta * u_prev < 0 && fabs(theta - u_prev) > MAX/2)
    {
        if (u_prev > 0)
        {
            u_prev -= MAX-MIN;
        }
        else
        {
            u_prev += MAX-MIN;
        }
    }

    omega = -(DT_vel_cal_sec - 2 * tau) / (DT_vel_cal_sec + 2 * tau) * omega_prev + 2 / (DT_vel_cal_sec + 2 * tau) * theta - 2 / (DT_vel_cal_sec + 2 * tau) * u_prev;
    //omega = (theta-u_prev)/DT_vel_cal_sec;
    u_prev = theta;
    omega_prev = omega;
    last_vel_calc = currentTime;
  }


  // if ((currentTime - lastTimeFilt) > loopFilt) {
  //   mov=movingAverage(omega);
  //   lastTimeFilt = currentTime;
  // }
 

 
   //float mov=movingAverage(omega);
 

  Serial.write((char*)&theta, sizeof(float));
  Serial.write((char*)&omega, sizeof(float));
  Serial.flush();
 // Serial.write((char*)&currentTime, sizeof(unsigned long));
  

  // Serial.print(newPositionrad);
  // Serial.print(",");
  // Serial.print(omega);
  // Serial.println(" ");
}
