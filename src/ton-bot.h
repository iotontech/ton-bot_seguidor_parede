/**
******************************************************************************
* @file    ton-bot.h
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

#ifndef TONBOT_H
#define TONBOT_H

/* Includes ------------------------------------------------------------------*/
#include "mbed.h"
#include "QEI.h"


/* Constantes ----------------------------------------------------------------*/
#define PAREDE_ESQUERDA 0b100
#define PAREDE_FRONTAL  0b010
#define PAREDE_DIREITA  0b001
#define INFINITO  8888  /* Indica a não identificação de linha */
#define BRANCA    0
#define PRETA     1
#define DEFAULT_LINHA       PRETA
#define DEFAULT_FRONTAL_TH  0.3
#define DEFAULT_LATERAL_TH  0.3


/******************************************************************************/
/** @addtogroup Biblioteca do Robô TON-BOT
* @{
*/


/**
* @brief Inicializa e configura os sensores do TON-BOT
* @param cor_linha: cor da linha do seguidor de linha
* @param frontal_th: limiar para reconhecer parede frontal ou não
* @param lateral_th: limiar para reconhecer paredes laterais ou não
* @return Nenhum
*/
void initTonBot(
  int cor_linha = DEFAULT_LINHA,
  float frontal_th = DEFAULT_FRONTAL_TH,
  float lateral_th = DEFAULT_LATERAL_TH);


/******************************************************************************/
/** @defgroup Buzzer
* @{
*/

/**
* @brief Gera vários beeps, o programa fica nesta rotina enquanto os beeps
* são gerados
* @param vezes: Número de beeps a serem gerados
* @param t_on: Tempo que o buzzer fica ligado [ms]
* @param t_off: Tempo que o buzzer fica desligado [ms]
* @return Nenhum
*/
void beeps(uint8_t vezes, uint16_t t_on, uint16_t t_off);


/**
* @brief Gera um beep que pode ser usado durante o funcionamento do robô
* @param duracao: Tempo do beep [ms]
* @return Nenhum
*/
void beep(uint16_t duracao);

/**
* @}
*/


/******************************************************************************/
/** @defgroup Encoders
* @{
*/

/**
* @brief Realiza a leitura do encoder da esquerda
* @param Nenhum
* @return Contagem do encoder da esquerda
*/
int32_t getEncoderEsquerda(void);


/**
* @brief Reseta a contagem do encoder da esquerda
* @param Nenhum
* @return Contagem do encoder da esquerda
*/
void resetEncoderEsquerda(void);


/**
* @brief Realiza a leitura do encoder da direita
* @param Nenhum
* @return Contagem do encoder da direita
*/
int32_t getEncoderDireita(void);


/**
* @brief Reseta a contagem do encoder da direita
* @param Nenhum
* @return Contagem do encoder da direita
*/
void resetEncoderDireita(void);

/**
* @}
*/


/******************************************************************************/
/** @defgroup Motores
* @{
*/

/**
* @brief Atualiza o PWM dos motores da esquerda e da direita
* @param pwm_esquerda: -1.0 a 1.0
* @param pwm_direita: -1.0 a 1.0
* @param fast_decay: modo de acionamento (default = false -> slow decay)
* @return Nenhum
*/
void setMotores(float pwm_esquerda, float pwm_direita, bool fast_decay = false);

/**
* @}
*/


/******************************************************************************/
/** @defgroup Sensores
* @{
*/

/**
* @brief Realiza a leitura dos sensores de parede
* 		(atualiza os sensores frontais e laterais)
* @param lf Valor proporcional a distância do sensor frontal esquerdo
* @param l Valor proporcional a distância do sensor diagonal esquerdo
* @param r Valor proporcional a distância do sensor diagonal direito
* @param rf Valor proporcional a distância do sensor frontal direito
* @return paredes: máscara de bits indicando presença (1) ou não (0)
* 	de paredes. O bit mais significativo representa a parede da esquerda.
* 	Ex.: 011 = presença de parede frontal e direita.
*/
uint8_t getSensoresParede(float* lf, float* l, float* r, float* rf);


/**
* @brief Verifica os sensores de linha
* @param Nenhum
* @return erro: valores entre -40 e 40 (valores negativos indicam que
*		o robô precisa se deslocar para a esquerda)
*/
int32_t getSensoresLinha();


#endif /* TONBOT_H_ */

/**
* @}
*/


/************************ (C) COPYRIGHT IOTON Technology **********************/
/***********************************END OF FILE********************************/
