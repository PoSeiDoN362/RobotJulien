// Antoine Sebastian Simon
#include "mbed.h"
#include "USBSerial.h"

#define MAX_VITESSE_DEG 720
#define MAX_VITESSE_DEFAULT 30
#define TEMPS_BAS 2

#define VITESSE_MAX_US 2000
#define VITESSE_MIN_US 10
#define PERIODE_MOTEUR 20000
#define K 0
#define KP -5

/* CMPS sensor */
#define CMPS12_DEFAULT_I2C_ADDRESS 0xC0

static BufferedSerial serial_port(USBTX, USBRX);

DigitalOut dir(p29);
DigitalOut dir2(p21);
DigitalOut step(p30);
DigitalOut step2(p23);
DigitalOut en(p28);
DigitalOut ms1(p27);
DigitalOut ms2(p26);
DigitalOut ms3(p25);
float TempsWait = 0;
#define TempsWaitMax 0.01

/*------------------ Global variables and opbjects -----------------*/



/* CMPS 12 sensor */
I2C i2c(p9, p10);
int compass_address = CMPS12_DEFAULT_I2C_ADDRESS;
char compass_data[31];
Ticker sensor_ticker;
Ticker timerInterrupt;
uint8_t sensorUpdatedFlag = 0;
int16_t accel[3] = {0};
int16_t gyro[3] = {0};
float roll_sensor[3] = {0};
uint8_t roll_sensor_index = 1;
char cmps_reg[31];

int i;
Timeout flipper;
uint32_t num1 = 1;
uint32_t num2 = 2;
int vitesse = 0;
int acceleration= 0;
int moteur_ratio = 0;
float step_deg = 0;
int freq;
float incr_vitesse;
float decr_vitesse;

float position_voulue = 0;
float position;

int erreur;
int val_roll;
int flag_print = 0;



/*---------------------------- Functions ---------------------------*/
void sensor_update(void) {
  sensorUpdatedFlag = 1;
  //led1 = !led1;
}


void fct_interruptSTEP()
{
    if (TempsWait > TempsWaitMax) {
        TempsWait = TempsWaitMax;
    }
    else {
        step = !step;
        step2 = !step2;
    } 
    
    flipper.attach(&fct_interruptSTEP, TempsWait);

}

void fct_interruptPRINT(void)
{
    flag_print = 1;
} 

int main()
{
    sensor_ticker.attach(sensor_update, 100ms);
    flipper.attach(&fct_interruptSTEP, 2.0);
    timerInterrupt.attach(fct_interruptPRINT, 500ms);
    serial_port.set_baud(9600);
    serial_port.set_blocking(false);
    serial_port.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );

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

        if(val_roll > 127)
        {
            val_roll = val_roll - 255;
        } 
        //printf("val_roll :%d\n", val_roll);

        if (val_roll && 0x03 != 0x03) {
          printf("Calibrating...");
        } else {
          roll_sensor[0] = atan2(accel[0], accel[2]) * 360 / 2 / 3.14159;
          roll_sensor[1] = roll_sensor[1] + gyro[1] * (-0.007);
        }

        sensorUpdatedFlag = 0;
      }
    /////////////////////////////////////////////////////////////

        erreur = position_voulue - val_roll;// Calcule de la différence entre l'entrée et la sortie du système
        vitesse = (KP * erreur) + K;


        //vitesse = (KP * erreur) + K; // kp * (sp-pv) + U0 --> Calcul régissant le système
        
        
        if(val_roll >= 0 && val_roll < 40)
        {
            //vitesse = vitesse * -1;

            //vitesse = 30;
            dir = 0;
            dir2 = 1;

        }
        else if(val_roll <= -1 && val_roll > -40)
        {
            vitesse = vitesse * -1;

            //vitesse = 30;
            dir = 1;
            dir2 = 0;

        }
        else 
        {
            vitesse = 0;
        }

        if(flag_print == 1)
        {
            printf("Vitesse = %d\n", vitesse);
            printf("Angle = %d\n", val_roll);
            flag_print = 0;
        }


        /////////////////////Section Etage Vitesse/////////////////////////////////////
        if(vitesse > 1500 && moteur_ratio == 2)// 1/2 vers full
        {
            moteur_ratio = 1;
            incr_vitesse = 1.009;
            decr_vitesse = 0.999;
        }

        if(vitesse < 1200 && moteur_ratio == 1)// full vers 1/2
        {
            moteur_ratio = 2;
            incr_vitesse = 1.01;
            decr_vitesse = 0.99;
        }

        if(vitesse > 600 && moteur_ratio == 4)// 1/4 vers 1/2
        {
            moteur_ratio = 2;
            incr_vitesse = 1.01;
            decr_vitesse = 0.99;
        }

        if(vitesse < 500 && moteur_ratio == 2)// 1/2 vers 1/4
        {
            moteur_ratio = 4;
            incr_vitesse = 1.01;
            decr_vitesse = 0.99;
        }
        
        if(vitesse > 350 && moteur_ratio == 8)// 1/8 vers 1/4
        {
            moteur_ratio = 4;
            incr_vitesse = 1.01;
            decr_vitesse = 0.99;
        }

        if(vitesse < 250 && moteur_ratio == 4)// 1/4 vers 1/8
        {
            moteur_ratio = 8;
            incr_vitesse = 1.05;
            decr_vitesse = 0.95;
        }

        if(vitesse > 150 && moteur_ratio == 16)// 1/16 vers 1/8
        {
            moteur_ratio = 8;
            incr_vitesse = 1.05;
            decr_vitesse = 0.95;
        }
        if(vitesse < 50 && moteur_ratio == 8) // 1/8 vers 1/16
        {
            moteur_ratio = 16;
            incr_vitesse = 1.1;
            decr_vitesse = 0.9;
        }

        switch(moteur_ratio)//switch case pour changer le ratio du moteur(configuration driver)
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

        step_deg = 1.8 / moteur_ratio;//calculer les nouveaux steps du moteur
        freq = vitesse / step_deg;//calculer la frequence
        TempsWait = (1.0/freq) / 2;//temps haut et bas pour l'interruption selon la frequence

        ////////////////////////////////////////////////////////////////////////////////

        //printf("Vitesse: %5d Ratio: %2d freq: %5d \r\n", vitesse, moteur_ratio, freq);//affichage 



        //Direction du moteur, probablement à modifier pour la suite du projet

        
    }
}

