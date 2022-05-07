// ÉQUIPE: Antoine, Sebastian, Simon
// DESCRIPTION: Dans ce code, on retrouve le projet final du cours 243-620.
//              Le projet consiste à mettre en fonction un contrôleur PID
//              pour équilibrer un robot à deux roues qui utilise des stepper moteurs.

//--------------------------// Includes nécessaires pour le code
#include "mbed.h"

//--------------------------// liste de defines pour notre controleur PID
#define K 0
#define KP -25              // Gain Proportionnel
#define KI -25              // Gain Intégral
#define KD -0.02            // Gain Dérivées
#define POSITION_VOULUE 0   // Position pour garder l'équilibre
//--------------------------//
#define TempsWaitMax 0.01

//--------------------------// Liste de defines pour adresse I2C du capteur CMPS12
#define CMPS12_DEFAULT_I2C_ADDRESS 0xC0

//-----------------------------------------------// Definitions de notre pinout pour le contrôleur LPC
static BufferedSerial serial_port(USBTX, USBRX);
DigitalOut dir(p29);
DigitalOut dir2(p21);
DigitalOut step(p30);
DigitalOut step2(p23);
DigitalOut en(p28);
DigitalOut ms1(p27);
DigitalOut ms2(p26);
DigitalOut ms3(p25);

/*------------------ Global variables and opbjects -----------------*/

//-------------------------------------------------// Variables pour le CMPS 12 sensor
I2C i2c(p9, p10);
Ticker sensor_ticker;
Ticker PIDInterrupt;
Ticker timerInterrupt;

int compass_address = CMPS12_DEFAULT_I2C_ADDRESS;

uint8_t sensorUpdatedFlag = 0;
uint8_t roll_sensor_index = 1;

int16_t accel[3] = {0};
int16_t gyro[3] = {0};

float roll_sensor[3] = {0};

char compass_data[31];
char cmps_reg[31];

//----------------------------------------------// Variables pour les stepper motors
Timeout flipper;

int moteur_ratio = 0;
int freq = 0;

float TempsWait = 0;
float vitesse = 0.0;
float step_deg = 0;
float incr_vitesse = 0.0;
float decr_vitesse = 0.0;

//---------------------------------------------// Variables pour PID
int erreur = 0;
int val_roll = 0;
int erreur_precedente = 0;
int flag_PID = 0;

float somme_erreur = 0.0;
float var_erreur = 0.0;
float P = 0.0;
float I = 0.0;
float D = 0.0;

//---------------------------------// Variable pour notre fonction print
int flag_print = 0;

//---------------------------------//
int flag_position = 1;

/*---------------------------- Functions ---------------------------*/
void sensor_update(void) {
  sensorUpdatedFlag = 1;
}

void fct_interruptSTEP() {
  if (TempsWait > TempsWaitMax) {
    TempsWait = TempsWaitMax;
  } else {
    step = !step;
    step2 = !step2;
  }
  flipper.attach(&fct_interruptSTEP, TempsWait);
}


void fct_interruptPRINT(void) {
  flag_print = 1;
}

void fct_interruptPID(void) {
    flag_PID = 1;
}

int main() {
  sensor_ticker.attach(sensor_update, 100ms);
  flipper.attach(&fct_interruptSTEP, 2.0);
  timerInterrupt.attach(fct_interruptPRINT, 500ms);
  PIDInterrupt.attach(fct_interruptPID, 100ms);
  serial_port.set_baud(9600);
  serial_port.set_blocking(false);
  serial_port.set_format(
      /* bits */ 8,
      /* parity */ BufferedSerial::None,
      /* stop bit */ 1);

  char buf[32] = {0};
  en = 0;
  moteur_ratio = 16;
  incr_vitesse = 1.1;
  decr_vitesse = 0.9;

  while (true) {
    ////// Interrupt pour update la valeur Roll///////
    if (sensorUpdatedFlag == 1) {
      // Read sensor
      i2c.write(compass_address, 0, 31);
      i2c.read(compass_address, cmps_reg, sizeof(cmps_reg));

      val_roll = cmps_reg[5];

      if (val_roll > 127) {
        val_roll = val_roll - 255;
      }

      if (val_roll && 0x03 != 0x03) {
        printf("Calibrating...");
      } else {
        roll_sensor[0] = atan2(accel[0], accel[2]) * 360 / 2 / 3.14159;
        roll_sensor[1] = roll_sensor[1] + gyro[1] * (-0.007);
      }

      sensorUpdatedFlag = 0;
    }

    /////////////////////////////////////////////////////////////
    if (flag_PID == 1) {
      erreur = POSITION_VOULUE - val_roll; // Calcule de la différence entre l'entrée et la sortie du système
      somme_erreur = somme_erreur + erreur * 0.1;
      var_erreur = (erreur - erreur_precedente) / 0.1;

      P = (KP * erreur) + K;
      I = (KI * somme_erreur);
      D = (KD * var_erreur);

      erreur_precedente = erreur;

      vitesse = P + I + D;
      flag_PID = 0;

      if (vitesse >= 0) {
        vitesse = vitesse; // Keep speed positive
        dir = 0;
        dir2 = 1;
      } else {
        vitesse = vitesse * -1; // Make speed positive
        dir = 1;
        dir2 = 0;
      }
    }

    if (val_roll > 40 || val_roll < -40) {
      flag_position = 1;
      en = 1;
      somme_erreur = 0;
      erreur_precedente = 0;
    }

    if (flag_position == 1) {
      somme_erreur = 0;
      if (val_roll == 0) {
        flag_position = 0;
        en = 0;
        somme_erreur = 0;
        erreur_precedente = 0;
      }
    }

    if (flag_print == 1) {
      printf("erreur = %i \n", erreur);
      printf("P = %f \n", P);
      printf("I = %f \n", I);
      printf("D = %f \n", D);
      printf("Angle = %d \n", val_roll);
      printf("Vitesse = %f \n", vitesse);

      flag_print = 0;
    }
    
    //--------------------------------Section Etage
    //--------------------------------Vitesse
    if (vitesse > 1500 && moteur_ratio == 2) // 1/2 vers full
    {
      moteur_ratio = 1;
      incr_vitesse = 1.009;
      decr_vitesse = 0.999;
    }

    if (vitesse < 1200 && moteur_ratio == 1) // full vers 1/2
    {
      moteur_ratio = 2;
      incr_vitesse = 1.01;
      decr_vitesse = 0.99;
    }

    if (vitesse > 600 && moteur_ratio == 4) // 1/4 vers 1/2
    {
      moteur_ratio = 2;
      incr_vitesse = 1.01;
      decr_vitesse = 0.99;
    }

    if (vitesse < 500 && moteur_ratio == 2) // 1/2 vers 1/4
    {
      moteur_ratio = 4;
      incr_vitesse = 1.01;
      decr_vitesse = 0.99;
    }

    if (vitesse > 350 && moteur_ratio == 8) // 1/8 vers 1/4
    {
      moteur_ratio = 4;
      incr_vitesse = 1.01;
      decr_vitesse = 0.99;
    }

    if (vitesse < 250 && moteur_ratio == 4) // 1/4 vers 1/8
    {
      moteur_ratio = 8;
      incr_vitesse = 1.05;
      decr_vitesse = 0.95;
    }

    if (vitesse > 150 && moteur_ratio == 16) // 1/16 vers 1/8
    {
      moteur_ratio = 8;
      incr_vitesse = 1.05;
      decr_vitesse = 0.95;
    }
    if (vitesse < 50 && moteur_ratio == 8) // 1/8 vers 1/16
    {
      moteur_ratio = 16;
      incr_vitesse = 1.1;
      decr_vitesse = 0.9;
    }

    switch (moteur_ratio) // switch case pour changer le ratio du moteur(configuration driver)
    {
    case 16:
      ms1 = 1;
      ms2 = 1;
      ms3 = 1;
      break;
    case 8:
      ms1 = 1;
      ms2 = 1;
      ms3 = 0;
      break;
    case 4:
      ms1 = 0;
      ms2 = 1;
      ms3 = 0;
      break;
    case 2:
      ms1 = 1;
      ms2 = 0;
      ms3 = 0;
      break;
    case 1:
      ms1 = 0;
      ms2 = 0;
      ms3 = 0;
      break;
    default:
      break;
    }

    step_deg = 1.8 / moteur_ratio; // calculer les nouveaux steps du moteur
    freq = vitesse / step_deg;     // calculer la frequence
    TempsWait = (1.0 / freq) / 2; // temps haut et bas pour l'interruption selon la frequence
  }
}
