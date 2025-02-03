#include <MPU6050.h>
#include <Servo.h>
#include <Wire.h>

MPU6050 capteur;
Servo moteur_gauche;
Servo moteur_droite;

const int taille_buffer = 10;
float buffer_angles[taille_buffer];
int position_index = 0;

int compteur_cycle = 0;

double entree, sortie;

double coefficient_Kp_angle = 5;
double coefficient_Ki_angle = 0;
double coefficient_Kd_angle = 2.4;

double coefficient_Kp_vitesse = 3;
double coefficient_Ki_vitesse = 1;
double coefficient_Kd_vitesse = 0;

float facteur = 2;

double limite_integrale = 1000;

double erreur_precedente = 0.001;
double somme_integrale = 0;
float ANGLE_CIBLE = 94;
float ANGLE_CIBLE_AVANCE = 96;
float ACCELERATION_CIBLE = 2.5;

float angle_actuel_cible = ANGLE_CIBLE;
float pas_angle = 0.1;

unsigned long temps_precedent = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  capteur.initialize();
  moteur_gauche.attach(10);
  moteur_droite.attach(11);
}

float fonction_sigmoid(float facteur, float valeur)
{
  return 1 / (1 + exp(-facteur * (valeur - 105)));
}

float fonction_sigmoid_inverse(float facteur, float valeur)
{
  return 1 / (1 + exp(facteur * (valeur - 75)));
}

float obtenir_angle()
{
  int16_t axe_x, axe_y, axe_z, vitesse_x, vitesse_y, vitesse_z;
  capteur.getMotion6(&axe_x, &axe_y, &axe_z, &vitesse_x, &vitesse_y, &vitesse_z);
  float angle_calcule = atan2((double)axe_y / 16384.0, (double)axe_z / 16384.0) * 180 / PI;
  return angle_calcule;
}

float calcul_moyenne_mobile(float nouvel_angle)
{
  buffer_angles[position_index] = nouvel_angle;
  position_index = (position_index + 1) % taille_buffer;

  float somme_angles = 0;
  for (int i = 0; i < taille_buffer; i++)
  {
    somme_angles += buffer_angles[i];
  }

  float moyenne = somme_angles / taille_buffer;
  return moyenne;
}

double calcul_pid(double Kp, double Ki, double Kd, double entree, float cible)
{
  double erreur = cible - entree;
  
  somme_integrale += erreur;
  if (somme_integrale > limite_integrale)
    somme_integrale = limite_integrale;
  if (somme_integrale < -limite_integrale)
    somme_integrale = -limite_integrale;

  double derivee = (erreur - erreur_precedente);
  erreur_precedente = erreur;

  double sortie = (Kp * erreur + Ki * somme_integrale + Kd * derivee);

  if (sortie > 90)
    sortie = 90;
  if (sortie < -90)
    sortie = -90;

  return sortie;
}

void ajuster_moteurs(double valeur_moteur)
{
  if (valeur_moteur < 6 && valeur_moteur > -6)
  {
    valeur_moteur = 0;
  }
  double valeur_droite = 90 + valeur_moteur;
  double valeur_gauche = 90 - valeur_moteur;

  moteur_gauche.write(valeur_gauche);
  moteur_droite.write(valeur_droite);
}

void stabiliser_position(float cible_angle)
{
  float angle_mesure = obtenir_angle();
  angle_mesure = calcul_moyenne_mobile(angle_mesure);

  double sortie = calcul_pid(coefficient_Kp_angle, coefficient_Ki_angle, coefficient_Kd_angle, angle_mesure, cible_angle);

  Serial.print(">angle:");
  Serial.println(angle_mesure);

  ajuster_moteurs(sortie);
}

float transition_mouvement_stabilisation(float angle_actuel_cible) {
  compteur_cycle++;

  if (compteur_cycle % 100 == 0)
  {
    if (ANGLE_CIBLE == 96)
    {
      ANGLE_CIBLE = ANGLE_CIBLE_AVANCE;
    }
    else
    {
      ANGLE_CIBLE = 96;
    }
  }

  if (angle_actuel_cible < ANGLE_CIBLE)
  {
    angle_actuel_cible += pas_angle;
    if (angle_actuel_cible > ANGLE_CIBLE)
    {
      angle_actuel_cible = ANGLE_CIBLE;
    }
  }
  else if (angle_actuel_cible > ANGLE_CIBLE)
  {
    angle_actuel_cible -= pas_angle;
    if (angle_actuel_cible < ANGLE_CIBLE)
    {
      angle_actuel_cible = ANGLE_CIBLE;
    }
  }
  return angle_actuel_cible;
}

void loop()
{
  stabiliser_position(angle_actuel_cible);
}
