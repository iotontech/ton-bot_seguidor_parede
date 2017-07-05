/**
******************************************************************************
* @file    ton-bot.cpp
* @author  Kleber Lima da Silva (kleber@ioton.cc)
* @version V0.0.1
* @date    19-Junho-2017
* @brief   Biblioteca do Robô TON-BOT (https://ioton.cc/ton-bot).
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

/* Includes ------------------------------------------------------------------*/
#include "ton-bot.h"   /* Veja este arquivo com a documentação/protótipos das funções */


/******************************************************************************/
/** @addtogroup Biblioteca do Robô TON-BOT
* @{
*/

/* Variáveis privadas --------------------------------------------------------*/
PwmOut r_motor_in2(PIN0);
PwmOut r_motor_in1(PIN1);
PwmOut l_motor_in1(PIN2);
PwmOut l_motor_in2(PIN3);
DigitalOut buzzer(PIN4);
QEI l_encoder(PIN8, PIN7, NC, 360, QEI::X4_ENCODING);
QEI r_encoder(PIN5, PIN6, NC, 360, QEI::X4_ENCODING);
DigitalOut lf_emitter(PIN9);
DigitalOut side_emitter(PIN10);
DigitalOut rf_emitter(PIN11);
DigitalOut alternate_en(PIN12);
DigitalOut line_emitter(PIN13);
DigitalIn line1(PIN14);
DigitalIn line2(PIN15);
DigitalIn line3(PIN16);
DigitalIn line4(PIN17);
DigitalIn line5(PIN18);
DigitalIn line6(PIN19);
DigitalIn line7(PIN20);
DigitalIn line8(PIN21);
AnalogIn lf_receiver(PIN22);
AnalogIn rf_receiver(PIN23);
AnalogIn r_receiver(PIN24);
AnalogIn l_receiver(PIN25);

Timeout flipper;

int LINHA = DEFAULT_LINHA;
float FRONTAL_TH = DEFAULT_FRONTAL_TH;
float LATERAL_TH = DEFAULT_LATERAL_TH;

void initTonBot(int cor_linha, float frontal_th, float lateral_th)
{
  /* Configura os sensores */
  LINHA = cor_linha;
  FRONTAL_TH = frontal_th;
  LATERAL_TH = lateral_th;

  /* Inicializa os PWMs */
  l_motor_in1.period_us(50);
  l_motor_in2.period_us(50);
  r_motor_in1.period_us(50);
  r_motor_in2.period_us(50);

  /* Inicializa os Encoders */
  resetEncoderEsquerda();
  resetEncoderDireita();
}


/******************************************************************************/
/** @defgroup Buzzer
* @{
*/

void beeps(uint8_t vezes, uint16_t t_on, uint16_t t_off)
{
  for (uint8_t i = 0; i < vezes; i++)
  {
    buzzer = 1;
    wait_ms(t_on);
    buzzer = 0;
    wait_ms(t_off);
  }
}

void beepOff()
{
  buzzer = 0;
}

void beep(uint16_t duracao)
{
  buzzer = 1;
  flipper.attach(&beepOff, (float)duracao/1000.0f);
}

/**
* @}
*/


/******************************************************************************/
/** @defgroup Encoders
* @{
*/

int32_t getEncoderEsquerda(void)
{
  return l_encoder.getPulses();
}


void resetEncoderEsquerda(void)
{
  l_encoder.reset();
}


int32_t getEncoderDireita(void)
{
  return r_encoder.getPulses();
}


void resetEncoderDireita(void)
{
  r_encoder.reset();
}

/**
* @}
*/


/******************************************************************************/
/** @defgroup Motores
* @{
*/

void setMotores(float pwm_esquerda, float pwm_direita, bool fast_decay)
{
  /* Acionamento do motor da esquerda */
  if (pwm_esquerda < 0) /* Reverso */
  {
    pwm_esquerda *= -1;

    /* Slow decay (default) */
    if (fast_decay == false)
    {
      l_motor_in1 = 1 - pwm_esquerda;
      l_motor_in2 = 1;
    }
    else /* Fast decay */
    {
      l_motor_in1 = 0;
      l_motor_in2 = pwm_esquerda;
    }
  }
  else /* Direto */
  {
    /* Slow decay (default) */
    if (fast_decay == false)
    {
      l_motor_in1 = 1;
      l_motor_in2 = 1 - pwm_esquerda;
    }
    else /* Fast decay */
    {
      l_motor_in1 = pwm_esquerda;
      l_motor_in2 = 0;
    }
  }

  /* Acionamento do motor da direita */
  if (pwm_direita < 0) /* Reverso */
  {
    pwm_direita *= -1;

    /* Slow decay (default) */
    if (fast_decay == false)
    {
      r_motor_in1 = 1 - pwm_direita;
      r_motor_in2 = 1;
    }
    else /* Fast decay */
    {
      r_motor_in1 = 0;
      r_motor_in2 = pwm_direita;
    }
  }
  else /* Direto */
  {
    /* Slow decay (default) */
    if (fast_decay == false)
    {
      r_motor_in1 = 1;
      r_motor_in2 = 1 - pwm_direita;
    }
    else /* Fast decay */
    {
      r_motor_in1 = pwm_direita;
      r_motor_in2 = 0;
    }
  }
}

/**
* @}
*/


/******************************************************************************/
/** @defgroup Sensores
* @{
*/

uint8_t getSensoresParede(float* lf, float* l, float* r, float* rf)
{
  uint8_t paredes = 0;

  (*lf) = lf_receiver;
  (*l) = l_receiver;
  (*r) = r_receiver;
  (*rf) = rf_receiver;

  /* Sensor frontal esquerdo */
  lf_emitter = 1;
  wait_us(60);
  (*lf) = lf_receiver - (*lf);
  lf_emitter = 0;
  if ((*lf) < 0)
  {
    (*lf) = 0;
  }
  wait_us(80);

  /* Sensor frontal direito */
  rf_emitter = 1;
  wait_us(60);
  (*rf) = rf_receiver - (*rf);
  rf_emitter = 0;
  if ((*rf) < 0)
  {
    (*rf) = 0;
  }
  wait_us(80);

  /* Sensores laterais */
  side_emitter = 1;
  wait_us(60);
  (*l) = l_receiver - (*l);
  (*r) = r_receiver - (*r);
  side_emitter = 0;
  if ((*l) < 0)
  {
    (*l) = 0;
  }
  if ((*r) < 0)
  {
    (*r) = 0;
  }


  /* Realiza a máscara de bits */
  if ((*lf) > FRONTAL_TH || (*rf) > FRONTAL_TH)
  {
    paredes |= PAREDE_FRONTAL;
  }

  if ((*l) > LATERAL_TH)
  {
    paredes |= PAREDE_ESQUERDA;
  }

  if ((*r) > LATERAL_TH)
  {
    paredes |= PAREDE_DIREITA;
  }

  return paredes;
}


int32_t getSensoresLinha()
{
  int32_t erro = 0, soma = 0, n = 0;

  /* Habilita os emissores */
  line_emitter = 1;
  wait_us(100);

  /* Realiza a leitura de todos os sensores de linha, os sensores das
   extremidades pussuem peso maior, no final é realizada a média ponderada */
  if (line1 == LINHA)
  {
    soma += -40;
    n++;
  }
  if (line2 == LINHA)
  {
    soma += -30;
    n++;
  }
  if (line3 == LINHA)
  {
    soma += -20;
    n++;
  }
  if (line4 == LINHA)
  {
    soma += -10;
    n++;
  }
  if (line5 == LINHA)
  {
    soma += 10;
    n++;
  }
  if (line6 == LINHA)
  {
    soma += 20;
    n++;
  }
  if (line7 == LINHA)
  {
    soma += 30;
    n++;
  }
  if (line8 == LINHA)
  {
    soma += 40;
    n++;
  }

  /* Desabilita os emissores */
  line_emitter = 0;


  /* Retorna a média ou retorna a constante INFINITO indicando
   que nenhum sensor leu linha */
  if (n != 0)
  {
    erro = soma / n;
  }
  else
  {
    erro = INFINITO;
  }

  return erro;
}

/**
* @}
*/


/************************ (C) COPYRIGHT IOTON Technology **********************/
/***********************************END OF FILE********************************/
