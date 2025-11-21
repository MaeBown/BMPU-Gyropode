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
double Te = 10.0;   // Période d'échantillonage en ms
double Tau = 275.0; // Constante de temps du filtre en ms
double EncTau = 825.0; // Constante de temps du filtre en ms

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

// ! Variables pour le filtre de Kalman
double GTheta = 0.0;
double GFTheta = 0.0;
double OldGFTheta = 0.0;
double RTheta = 0.0;
double RFTheta = 0.0;
double OldRFTheta = 0.0;
double theta = 0.0;

// ! Fonction de transfert du filtre
double A, B;
double AVit, BVit;

// ! Variables de schéma bloc de commande
double thetaCons = 0.0;
double thetaEq = -0.105;
double erreur = 0.0;
double commande = 0;
double C0 = 1000.0;
double CD = 58.0;
double frottement = 27.75;
double asserCons = 0.0;
double alpha = 127;

// ! Variables pour les encodeurs et leur asservissement
int DPos = 0;
int GPos = 0;
double RPosPrev = 0.0;
double RPos = 0.0;
double VRot = 0.0;
double VFRot = 0.0;
double vitCons = 0.0;
double error = 0.0;
double prevError = 0.0;
double sumError = 0.0;
double corrError = 0.0;
double KP = 1.0; // kp pour la stabilisation de la position (translation)
double KD = 0.1;    // kd pour la stabilisation de la position (translation)
double KI = 0.001;    // ki pour la stabilisation de la position (translation)

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

    // ! Calcul de l'angle par rapport aux axes y et x de l'accéléromètre (filtre complémentaire)
    GTheta = atan2(a.acceleration.y, a.acceleration.x);  // Angle accéléro brut

    //* Passe-bas pour supprimer les bruits des hautes fréquences cependant on perd la dynamique du signal
    GFTheta = (A * GTheta) + (B * OldGFTheta);  // Filtre passe-bas
    OldGFTheta = GFTheta;  // Sauvegarder pour la prochaine itération

    //* Passe-haut pour reconstruire le signal sans les oscillations dûes au côté peu amorti des accéléromètres
    RTheta = -g.gyro.z * Tau / 1000; // Intégration du gyro (Tau en ms → secondes)
    RFTheta = (A * RTheta) + (B * OldRFTheta);  // Filtre passe-haut
    OldRFTheta = RFTheta;  // Sauvegarder pour la prochaine itération

    theta = RFTheta + GFTheta;  // Fusion complémentaire

    // mesures des valeurs des encodeurs
    DPos = encoderd.getCount();
    GPos = encoderg.getCount();

    RPosPrev = RPos;          // sauvegarde de la position moyenne précédente
    RPos = (DPos + GPos) / 2; // position moyenne des encodeurs

    VRot = (RPos - RPosPrev) / (Te);       // vitesse de rotation moyenne des roues
    VFRot = VRot * AVit + VFRot * BVit; // filtre passe bas pour filtrer les bruits
    
    error = vitCons - VFRot;
    sumError += error;
    corrError = KI * sumError * Te;

    // thetaCons = error * KP + (((error - prevError) * KD) / Te) + corrError; // PD + I

    // ? Calcul de l'erreur et asservissement de la position angulaire
    erreur = (thetaEq + thetaCons) - theta;  // Erreur de position angulaire
    asserCons = erreur * C0 + (g.gyro.z) * CD;  // PD sur l'angle

    // Compensation des frottements secs
    if (asserCons >= 0) {
      commande = asserCons + frottement;
    } else if (asserCons < 0) {
      commande = asserCons - frottement;
    }

    // Saturation de la commande
    commande = constrain(commande, -120, 120);

    // ! Commande des moteurs
    ledcWrite(can0, (int)(127 + commande));
    ledcWrite(can1, (int)(127 - commande));
    
    // ! Moteur gauche
    ledcWrite(can2, (int)(127 - commande));
    ledcWrite(can3, (int)(127 + commande));

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
  ledcAttachPin(ppwmgb, can2);
  ledcAttachPin(ppwmga, can3);

  // ! Calcul des coefficents du filtre sans développement des expressions de la fonction de transfert
  A = 1 / (1 + Tau / Te);
  B = A * (Tau / Te);

  // ! Calcul des coefficents du filtre sans développement des expressions de la fonction de transfert
  A = 1 / (1 + EncTau / Te);
  B = A * (EncTau / Te);
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
    if (commande == "CD")
    {
      CD = valeur.toFloat();
    }
    if (commande == "CF")
    {
      frottement = valeur.toInt();
    }
    if (commande == "thetaEq")
    {
      thetaEq = valeur.toFloat();
    }
    if (commande == "KP")
    {
      KP = valeur.toFloat();
    }
    if (commande == "KD")
    {
      KD = valeur.toFloat();
    }
    if (commande == "KI")
    {
      KI = valeur.toFloat();
    }
    if (commande == "ETau")
    {
      EncTau = valeur.toFloat();
      // Calcul coeff filtre
      AVit = 1 / (1 + EncTau / Te);
      BVit = AVit * (EncTau / Te);
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

    // ? Affichage de l'commande PWM et de l'angle theta
    Serial.print(commande);
    Serial.print(" ");
    Serial.print(erreur);
    Serial.print(" ");
    Serial.print(theta);
    Serial.println(" ");

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