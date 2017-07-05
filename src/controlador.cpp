/**
******************************************************************************
* @file    controlador.cpp
* @author  Kleber Lima da Silva (kleber@ioton.cc)
* @version V0.0.1
* @date    05-Julho-2017
* @brief   Biblioteca para controle de velocidade usando os encoders.
******************************************************************************
* @attention
*
* COPYRIGHT(c) 2017 IOTON Technology
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
******************************************************************************
*/

#include "controlador.h"


/******************************************************************************/
/** @addtogroup Biblioteca do controlador de velocidade (perfil retangular)
* @{
*/

/* Variáveis Externas --------------------------------------------------------*/
int32_t distancia_mm = 0;
int32_t distancia_deg = 0;


/* Variáveis Privadas --------------------------------------------------------*/
int8_t tipo_movimento = 0;
int32_t setpoint_dist = 0;
int32_t setpoint_speed_esquerda_cnt = 0, setpoint_speed_direita_cnt = 0;
int32_t encoder_esquerda_delta = 0, encoder_direita_delta = 0;
int32_t encoder_esquerda_anterior_cnt = 0, encoder_direita_anterior_cnt = 0;
int32_t pid_esquerda_out = 0, pid_direita_out = 0;
int32_t erro_esquerda_anterior = 0, erro_direita_anterior = 0;
bool bUsarSensores = false;
float frontal_esquerdo = 0, lateral_esquerdo = 0, lateral_direito = 0, frontal_direito = 0;


/*******************************************************************************
*Função principal do controle de velocidade -----------------------------------
*/
void controleVelocidade(void)
{
  getSensoresParede(&frontal_esquerdo, &lateral_esquerdo, &lateral_direito, &frontal_direito);
  updateEncoders(); // Atualiza as distâncias e velocidades
  controladorPID(); // Controlador de velocidade
}


/* Função para selecionar os parâmetros do movimento ---------------------------
* raio [mm]: 0 para movimento translacional ou raio para a curva
* dist_ou_angulo [mm ou deg]: distância(translational ou angular) do movimento
* speed [mm/s]: velocidade do movimento
*/
void setMovimento(int32_t raio, int32_t dist_ou_angulo, int32_t speed)
{
  if (raio == 0)
  {
    tipo_movimento = TRANSLACIONAL;

    /* Usado para o controle de velocidade */
    setpoint_speed_esquerda_cnt = SPEED_TO_COUNTS(speed);
    setpoint_speed_direita_cnt = setpoint_speed_esquerda_cnt;
  }
  else
  {
    tipo_movimento = ROTACIONAL;

    /* Calcula as velocidade a partir do raio */
    float speedX_mm_s = (float)speed;
    float speedW_rad_s = speedX_mm_s / (float)raio;

    int32_t speedX_cnt = SPEED_TO_COUNTS(abs(speed));
    int32_t speedW_cnt = RAD_S_TO_COUNTS(speedW_rad_s);

    if (raio < RODA_RODA)
    {
      speedX_cnt -= (abs(speedW_cnt) / 2);
      speedW_cnt /= 2;
    }

    /* Usado para o controle de velocidade */
    setpoint_speed_esquerda_cnt = speedX_cnt - speedW_cnt;
    setpoint_speed_direita_cnt = speedX_cnt + speedW_cnt;
  }

  /* Usado para indicar o fim do movimento */
  setpoint_dist = dist_ou_angulo;
}


/* Atualiza a leitura dos encoders ---------------------------------------------
*/
void updateEncoders(void)
{
  int32_t encoder_esquerda_cnt, encoder_direita_cnt;

  /* Atualiza a contagem dos encoders */
  encoder_esquerda_cnt = getEncoderEsquerda();
  encoder_direita_cnt = getEncoderDireita();

  /* Atualiza a alteração dos encoders (speed) */
  encoder_esquerda_delta = encoder_esquerda_cnt - encoder_esquerda_anterior_cnt;
  encoder_direita_delta = encoder_direita_cnt - encoder_direita_anterior_cnt;

  /* Guarda os valores anteriores */
  encoder_esquerda_anterior_cnt = encoder_esquerda_cnt;
  encoder_direita_anterior_cnt = encoder_direita_cnt;

  /* Calcula a distância (mm) e o ângulo (deg) */
  distancia_mm = COUNTS_TO_MM((encoder_direita_cnt + encoder_esquerda_cnt) / 2);
  distancia_deg = COUNTS_TO_DEG((encoder_direita_cnt - encoder_esquerda_cnt) / 2);
}


/* Controlador PID de velocidade dos motores -----------------------------------
*/
void controladorPID(void)
{
  int32_t erro_esquerda = 0, erro_direita = 0;

  /* Alinhamento através dos sensores */
  if (bUsarSensores == true)
  {
    int32_t sensor_feedback = (int32_t)getErroSensores() / K_SENSORES;

    erro_esquerda += sensor_feedback;
    erro_direita -= sensor_feedback;
  }

  /* Erro de velocidade */
  erro_esquerda += (setpoint_speed_esquerda_cnt - encoder_esquerda_delta);
  erro_direita += (setpoint_speed_direita_cnt - encoder_direita_delta);

  /* PD - motor da esquerda */
  pid_esquerda_out += (KP * erro_esquerda) +
  (KD * (erro_esquerda - erro_esquerda_anterior));

  /* PD - motor da direita */
  pid_direita_out += (KP * erro_direita) +
  (KD * (erro_direita - erro_direita_anterior));

  /* Guarda os valores anteriores */
  erro_esquerda_anterior = erro_esquerda;
  erro_direita_anterior = erro_direita;

  setMotores(pid_esquerda_out / 1000.0f, pid_direita_out / 1000.0f);
}


/* Indica o fim do movimento ---------------------------------------------------
*/
bool isFinalMovimento(void)
{
  if (tipo_movimento == TRANSLACIONAL && distancia_mm >= setpoint_dist)
  {
    return true;
  }
  else if (tipo_movimento == ROTACIONAL && abs(distancia_deg) >= setpoint_dist)
  {
    return true;
  }
  else if ((tipo_movimento == TRANSLACIONAL) && (frontal_esquerdo > CENTRO_FRONTAL && frontal_direito > CENTRO_FRONTAL))
  {
    return true;
  }

  return false;
}


/* Reseta os encoders e as variáveis do controlador ----------------------------
*/
void resetControlador(void)
{
  resetEncoderEsquerda();
  resetEncoderDireita();

  encoder_esquerda_anterior_cnt = 0;
  encoder_direita_anterior_cnt = 0;

  distancia_mm = 0;
  distancia_deg = 0;

  pid_esquerda_out = 0; pid_direita_out = 0;
  erro_esquerda_anterior = 0; erro_direita_anterior = 0;
  setpoint_speed_esquerda_cnt = 0; setpoint_speed_direita_cnt = 0;
}


/* Movimenta o micromouse para frente uma distância em [mm] --------------------
* Se mm == 0: anda para frente até o próximo comando
*/
void frente(int32_t mm)
{
  if (mm != 0)
  {
    bUsarSensores = true;
    resetControlador();
    setMovimento(0, mm, SPEED_RETA);

    do
    {
      wait_ms(1);
    }
    while (isFinalMovimento() == false);
  }
  else
  {
    bUsarSensores = true;
    setMovimento(0, mm, SPEED_RETA);
    wait_ms(1);
  }
}


/* Realiza uma curva em torno do próprio eixo (graus < 0: para direita) --------
*/
void curvaPivot(int16_t graus)
{
  bUsarSensores = false;
  resetControlador();
  setMotores(0, 0);
  wait_ms(100);

  /* Seleciona o movimento de acordo com o sentido da curva */
  if (graus > 0) setMovimento(RODA_RODA / 2, graus, SPEED_CURVA);
  else setMovimento(RODA_RODA / 2, -graus, -SPEED_CURVA);

  do
  {
    wait_ms(1);
  }
  while (isFinalMovimento() == false);
}


/* Realiza uma curva em torno do próprio eixo e anda até a fronteira da célula
*		(graus < 0: para direita)
*/
void curva(int16_t graus)
{
  frente(CELULA / 2);
  curvaPivot(graus);
  frente(CELULA / 2);
}

/* Retorna o erro de alinhamento dos sensores ----------------------------------
*/
int16_t getErroSensores(void)
{
  float erro = 0;

  /* ------ ALINHAMENTO LATERAL ------ */
  if (frontal_esquerdo < FRONTAL_TH && frontal_direito < FRONTAL_TH)
  {
#if PID_TECNICA == 1
    if (lateral_esquerdo > CENTRO_LATERAL && lateral_direito < CENTRO_LATERAL)
    {
      erro = lateral_esquerdo - CENTRO_LATERAL;
    }
    else if (lateral_direito > CENTRO_LATERAL && lateral_esquerdo < CENTRO_LATERAL)
    {
      erro = CENTRO_LATERAL - lateral_direito;
    }
#elif PID_TECNICA == 2
    if (lateral_esquerdo > LATERAL_TH && lateral_direito > LATERAL_TH)
    {
      erro = lateral_esquerdo - lateral_direito;
    }
    else if (lateral_esquerdo > LATERAL_TH)
    {
      erro = 2 * (lateral_esquerdo - CENTRO_LATERAL);
    }
    else if (lateral_direito > LATERAL_TH)
    {
      erro = 2 * (CENTRO_LATERAL - lateral_direito);
    }
#endif
  }
  /* ------ ALINHAMENTO FRONTAL ------ */
  else if (frontal_esquerdo > ALINHAMENTO_FRONTAL && frontal_direito > ALINHAMENTO_FRONTAL)
  {
    erro = (frontal_direito - frontal_esquerdo);
  }

  return ((int16_t)(erro * 4095));//return erro;
}

/**
* @}
*/


/************************ (C) COPYRIGHT IOTON Technology **********************/
/***********************************END OF FILE********************************/
