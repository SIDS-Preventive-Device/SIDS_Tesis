#include "Wire.h"
#include "libs/Mahony.cpp"
#include "libs/I2Cdev.cpp"
#include "libs/MPU9250.cpp"

#define KP 32.5
#define KI 3.0
#define gscale (250.0/32768.0)*(PI/180.0)  //Escala para el giroscopio
//#define QUATERNIONS 1

MPU9250  accelgyro;
Mahony mahony(KP, KI);
I2Cdev   I2C_M;

//Offsets y matrices de correccion
float A_B[3]  {  150.42,   94.82, -725.65};
float A_Ainv[3][3]  {
  {  0.06220, -0.00141, -0.00018},
  { -0.00141,  0.06138, -0.00048},
  { -0.00018, -0.00048,  0.05994}
};

float M_B[3]  {    6.36,   25.27,  -69.48};
float M_Ainv[3][3] {
  {  1.75453, -0.09176,  0.03732},
  { -0.09176,  1.85748,  0.09509},
  {  0.03732,  0.09509,  1.94828}
};

float G_off[3] = { -42.3, -2.1, 23.6}; 

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];

unsigned long now = 0, last = 0;
float dT = 0;
unsigned long now_ms, last_ms = 0;
unsigned long print_ms = 70; 

bool initial = true;
float initialYaw;
int count = 0;

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);
  while (!Serial);

  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU9250 OK" : "MPU9250 ??");
}

void loop() {
  float rollPitchYaw[3];
  float q[4];

  getMPUScaled();
  now = micros();
  dT = (now - last) * 1.0e-6; 
  last = now;

  //Los ejes x & y del magnetometro estan invertidos con respecto al acelerometro y giroscopio
  mahony.MahonyQuaternionUpdate(Axyz[1], Axyz[0], Axyz[2], Gxyz[1], Gxyz[0], Gxyz[2],
                                Mxyz[0], Mxyz[1], -Mxyz[2], dT);

  //Obtenemos los valores de los angulos
  mahony.getRollPitchYaw(rollPitchYaw);

  //Conversion de radianes a grados
  rollPitchYaw[2] *= 180.0 / PI;
  rollPitchYaw[1] *= 180.0 / PI;
  rollPitchYaw[0] *= 180.0 / PI;

  rollPitchYaw[2] = -rollPitchYaw[2] - 5.8;
  if (rollPitchYaw[2] < 0) rollPitchYaw[2] += 360.0;
  if (rollPitchYaw[2] > 360.0) rollPitchYaw[2] -= 360.0;

  now_ms = millis();
  if (now_ms - last_ms >= print_ms) {
    last_ms = now_ms;
    count++;

#ifdef QUATERNIONS
    //Imprimir salida en formato de cuaternion
    mahony.getQuaternion(q);
    Serial.print("Quaternion: ");
    Serial.print(q[0]);
    Serial.print(", ");
    Serial.print(q[1]);
    Serial.print(", ");
    Serial.print(q[2]);
    Serial.print(", ");
    Serial.println(q[3]);
#else
    //Imprimir salida en formato de angulos Tait-Bryan
    Serial.println(initialYaw);
    Serial.print("Orientation: ");
    Serial.print(rollPitchYaw[2]);
    Serial.print(", ");
    Serial.print(rollPitchYaw[1]);
    Serial.print(", ");
    Serial.println(rollPitchYaw[0]);
#endif
  }
}

void getMPUScaled(void) {
  //Aqui Se aplican los offsets y valores de la matriz de correccion obtenidos
  float temp[3];

  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  Gxyz[0] = ((float) gx - G_off[0]) * gscale;
  Gxyz[1] = ((float) gy - G_off[1]) * gscale;
  Gxyz[2] = ((float) gz - G_off[2]) * gscale;

  Axyz[0] = (float) ax;
  Axyz[1] = (float) ay;
  Axyz[2] = (float) az;
  
  for (int i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vectorNormalize(Axyz);

  Mxyz[0] = (float) mx;
  Mxyz[1] = (float) my;
  Mxyz[2] = (float) mz;
  
  for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vectorNormalize(Mxyz);
}

void vectorNormalize(float a[3]) {
  float magnitude = sqrt(vectorDot(a, a));
  a[0] /= magnitude;
  a[1] /= magnitude;
  a[2] /= magnitude;
}
