// Antoine Sebastian Simon
#include "mbed.h"
#include "USBSerial.h"

#define MAX_VITESSE_DEG 720
#define MAX_VITESSE_DEFAULT 30
#define TEMPS_BAS 2

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

int i;
Timeout flipper;
uint32_t num1 = 1;
uint32_t num2 = 2;
int vitesse = 30;
int acceleration= 0;
int moteur_ratio = 0;
float step_deg = 0;
int freq;
float incr_vitesse;
float decr_vitesse;

// main() runs in its own thread in the OS

void fct_interruptSTEP()
{
    step = !step;
    step2 = !step2;
    flipper.attach(&fct_interruptSTEP, TempsWait);

}    

int main()
{
    flipper.attach(&fct_interruptSTEP, 2.0);
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
        
        serial_port.read(buf, sizeof(buf));

        if (buf[0] == 'z') //pour augmenter la vitesse avec z sur le clavier
        {
            vitesse = vitesse * incr_vitesse;
            if(vitesse < 1)
            {
                vitesse = vitesse + 2;
            }
        }
        else if (buf[0] == 'x') {//pour descendre la vitesse avec x sur le clavier
            vitesse = vitesse * decr_vitesse;

        }
        buf[0] = 0;//remettre le buffer du clavier à 0
        
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

        printf("Vitesse: %5d Ratio: %2d freq: %5d \r\n", vitesse, moteur_ratio, freq);//affichage 



        //Direction du moteur, probablement à modifier pour la suite du projet
        dir = 0;
        dir2 = 1;
        
    }
}

