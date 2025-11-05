#include <Arduino.h>
#include <String.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BluetoothSerial.h>
#include <Wire.h>
#include "ESP32Encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "rgb_lcd.h"

ESP32Encoder encoderd;
ESP32Encoder encoderg;
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
BluetoothSerial SerialBT;
QueueHandle_t xQueue;
rgb_lcd lcd;

void controle(void *parameters);
void reception(char ch);
void serialEvent(void);
void vATaskFunction(void *pvParameters);
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void receptionBT(void);

char FlagCalcul = 0;
double Te = 7;   // Période d'échantillonage en ms
double Tau = 355; // Constante de temps du filtre en ms

// ! Déclaration des pins
int ppwmda = 32;
int ppwmdb = 33;
int ppwmga = 25;
int ppwmgb = 26;
int pemotda = 27;
int pemotdb = 14;
int pemotga = 19;
int pemotgb = 18;
int pledg = 5;
int pledr = 4;
int pledy = 0;
int pledb = 2;

// ! Déclaration des caractéristiques des PWMs
int frequency = 20000;
int resolution = 8;
int can0 = 0;
int can1 = 1;
int can2 = 2;
int can3 = 3;
int pwmda = 0;
int pwmdb = 0;
int pwmga = 0;
int pwmgb = 0;
int PWM = 0;
int PWMReverse = 0;

// ! Variables pour le filtre de Kalman
double GTheta = 0;
double GFTheta = 0;
double OldGFTheta = 0;
double RTheta = 0;
double RFTheta = 0;
double OldRFTheta = 0;
double theta = 0;

// ! Fonction de transfert du filtre
double A, B;

// ! Variables de schéma bloc de commande
double thetaCons = 0;
double thetaEq = -0.241786949428571;
double consigne = 0;
double instruction = 0;
double C0 = 0.68;
double KD = 0.195;
double Frottement = 0.027;
double Ec = 0;
double asserCons = 0;
double alpha = 255;

// timer and flag for example, not needed for encoders
unsigned long encoder2lastToggled;
bool encoder2Paused = false;

void controle(void *parameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    mpu.getEvent(&a, &g, &temp);

    // ! Calcul de l'angle par rapport aux axes y et z de l'accéléromètre
    GTheta = atan2(a.acceleration.y, a.acceleration.z);

    //* Passe-bas pour supprimer les bruits des hautes fréquences cependant on perd la dynamique du signal
    GFTheta = (A * GTheta) + (B * OldGFTheta);
    OldGFTheta = GFTheta;

    //* Passe-haut pour reconstruire le signal sans les oscillations dûes au côté peu amorti des accéléromètres
    RTheta = (-1) * g.gyro.x * Tau * 0.001; // ? Tau est en ms, on le met en seconde pour les calculs qui suivent
    RFTheta = (A * RTheta) + (B * OldRFTheta);
    OldRFTheta = RFTheta;

    theta = RFTheta + GFTheta;

    // ? Calcul de la consigne
    consigne = (thetaEq + thetaCons) - theta;
    asserCons = consigne * C0 + (g.gyro.x) * KD;

    // Calcul de la commande compensée avec saturation
  if (asserCons > thetaEq) {
      Ec = asserCons + Frottement;
  } else if (asserCons < thetaEq) {
      Ec = asserCons - Frottement;
  }

  // Calculer la commande PWM une seule fois
  instruction = (int)(Ec * alpha);
  PWM = 127 + instruction;
  PWMReverse = 127 - instruction;

  // ! Moteur droit
  ledcWrite(can0, PWM);
  ledcWrite(can1, PWMReverse);
  
  // ! Moteur gauche
  ledcWrite(can2, PWM);
  ledcWrite(can3, PWMReverse);

    FlagCalcul = 1;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void setup()
{
  // ! Initialisation du baudrate de la communication série
  Serial.begin(921600);

  // ! Initialisation de l'écran LCD
  lcd.begin(16, 2);
  lcd.setRGB(0, 255, 0);

  // ! Initialisation de l'accéléromètre
  Serial.println("Adafruit MPU6050 test!");

  // ! Initialisation des encodeurs incrémentaux
  encoderd.attachHalfQuad(pemotga, pemotgb);
  encoderg.attachHalfQuad(pemotda, pemotdb);
  ESP32Encoder::useInternalWeakPullResistors = UP;

  // ? Mise à zéro des compteurs
  encoderd.clearCount();
  encoderg.clearCount();

  // Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));

  // set the lastToggle
  // encoder2lastToggled = millis();

  // ! Recherche du MPU6050
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // ! Augmentation de la précision du MPU
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  xTaskCreate(
      controle,   // nom de la fonction
      "controle", // nom de la tache que nous venons de vréer
      10000,      // taille de la pile en octet
      NULL,       // parametre
      10,         // tres haut niveau de priorite
      NULL        // descripteur
  );

  // ! Initialisation des sorties
  pinMode(pledg, OUTPUT);
  pinMode(pledy, OUTPUT);
  pinMode(pledr, OUTPUT);
  pinMode(pledb, OUTPUT);

  // ! Initialisation des PWMs
  ledcSetup(can0, frequency, resolution);
  ledcSetup(can2, frequency, resolution);
  ledcAttachPin(ppwmda, can0);
  ledcAttachPin(ppwmdb, can1);
  ledcAttachPin(ppwmga, can2);
  ledcAttachPin(ppwmgb, can3);

  // ! Calcul des coefficents du filtre sans développement des expressions de la fonction de transfert
  A = 1 / (1 + Tau / Te);
  B = A * (Tau / Te);
}

void reception(char ch)
{
  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "Tau")
    {
      Tau = valeur.toFloat();
      // Calcul coeff filtre
      A = 1 / (1 + Tau / Te);
      B = A * (Tau / Te);
    }
    if (commande == "Te")
    {
      Te = valeur.toInt();
    }
    if (commande == "C0")
    {
      C0 = valeur.toFloat();
    }
    if (commande == "KD")
    {
      KD = valeur.toFloat();
    }
    if (commande == "CF")
    {
      Frottement = valeur.toFloat();
    }
    if (commande == "thetaEq")
    {
      thetaEq = valeur.toInt();
    }

    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

void loop()
{
  if (FlagCalcul == 1)
  {
    // ? Affichage des différentes accélérations et vitesses angulaires
    // Serial.printf("%4.2lf %4.2lf %4.2lf \n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
    // Serial.printf("%4.2lf %4.2lf %4.2lf \n", g.gyro.x, g.gyro.y, g.gyro.z);

    // ? Affichage des angles
    // Serial.printf("%3.9lf %3d %3d %1.6lf %1.6lf\n", theta, PWM, PWMReverse, instruction, asserCons);

    FlagCalcul = 0;
  }
  // Serial.printf("Task loop() %d\n", xPortGetCoreID());
}

void serialEvent(void)
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}

void vATaskFunction(void *pvParameters) // <- une tâche
{
  int variable1;        // <- variable allouée dans la pile (*stack*) de la tâche et unique pour chaque instance de tâche
  static int variable2; // <- variable allouée en dehors de la pile de la tâche et partagée pour chaque instance de tâche

  for (;;) // <- boucle infinie
  {
    Serial.printf("vATaskFunction %d\n", xPortGetCoreID());
    delay(1000);
  }
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  uint16_t RxSize, i;
  if (event == ESP_SPP_SRV_OPEN_EVT)
  {
    Serial.println("Client Connected has adress:");
    for (i = 0; i < 6; i++)
    {
      Serial.printf("%02X ", param->srv_open.rem_bda[i]);
      if (i < 5)
      {
        Serial.print(":");
      }
    }
  }

  if (event == ESP_SPP_DATA_IND_EVT)
  {
    xQueueSend(xQueue, &param->data_ind.len, portMAX_DELAY);
  }
}

// void receptionBT(void)
// {
//   char bytesNumber;
//   bool pd;
//   SerialBT.register_callback(callback);
//   pd = xQueueReceive(xQueue, &bytesNumber, portMAX_DELAY);

//   if (pd)
//   {
//     char buffer[100];
//     SerialBT.readBytes(buffer, bytesNumber);
//     Serial.write(buffer, bytesNumber);
//     free(buffer);
//   }
// }