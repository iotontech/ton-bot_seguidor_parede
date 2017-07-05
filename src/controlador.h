/**
******************************************************************************
* @file    controlador.h
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

#ifndef CONTROLADOR_H_
#define CONTROLADOR_H_


/* Includes ------------------------------------------------------------------*/
#include "ton-bot.h"

/* Constantes ----------------------------------------------------------------*/
/* Parâmetros dos sensores */
#define FRONTAL_TH      0.250f    // Limiar para reconhecer parede frontal ou não
#define LATERAL_TH      0.200f    // Limiar para reconhecer parede lateral ou não
#define CENTRO_LATERAL  0.280f    // Leitura dos sensores laterais no centro da célula
#define CENTRO_FRONTAL  0.660f    // Leitura dos sensores frontais no centro da célula
#define ALINHAMENTO_FRONTAL 0.400f  // Valor para habilitar o alinhamento pelos sensores frontais

/* Ganho dos controladores de velocidade*/
#define KP  5   // Ganho Proporcional
#define KD  10  // Ganho Derivativo
#define PID_TECNICA 2 // Técnica usada para o PID a partir do erro dos sensores

/* Indica o tipo de movimento */
#define TRANSLACIONAL 0
#define ROTACIONAL    1

/* Parâmetros do robô */
#define ENCODER_PPR     360   // Resolução do Encoder pulsos por revolução [ppr]
#define	DIAMETRO_RODA   32    // Diâmetro da roda [mm]
#define RODA_RODA       84    // Distância entre as rodas [mm]
#define CNT_POR_1000MM  3581  // = (1000*ENCODER_PPR) / (DIAMETRO_RODA*PI)
#define CNT_POR_360DEG  945   // = ((PI*RODA_RODA)*CNT_PER_1000MM)/(1000)

/* Parâmetros do controlador */
#define TS          10  // Período de amostragem [ms]
#define K_SENSORES  400 // Constante para ajuste do peso dos sensores no controlador
#define SPEED_RETA  150 // Velocidade nas retas [mm/s]
#define SPEED_CURVA 150 // Velocidade nas curvas [mm/s]
#define CELULA      180 // Tamanho da célula [mm]


/* Macros --------------------------------------------------------------------*/
#define COUNTS_TO_MM(cnt)	(((cnt) * 1000) / CNT_POR_1000MM)
#define COUNTS_TO_DEG(cnt)	(((cnt) * 360) / CNT_POR_360DEG)
#define DIST_TO_COUNTS(mm)	(((mm) * CNT_POR_1000MM) / 1000)
#define SPEED_TO_COUNTS(speed)	((CNT_POR_1000MM * (speed) * TS) / 1000000)
#define RAD_S_TO_COUNTS(rad_s)	(SPEED_TO_COUNTS(rad_s * RODA_RODA))


/* Protótipo das Funções -----------------------------------------------------*/
void controleVelocidade(void);
void setMovimento(int32_t raio, int32_t dist_angulo, int32_t speed);
void updateEncoders(void);
void controladorPID(void);
void controleVelocidade(void);
bool isFinalMovimento(void);
void resetControlador(void);
void frente(int32_t distancia);
void curvaPivot(int16_t graus);
void curva(int16_t graus);
int16_t getErroSensores(void);


/* Variáveis Externas --------------------------------------------------------*/
extern int32_t distancia_mm;
extern int32_t distancia_deg;


#endif /* CONTROLADOR_H_ */

/************************ (C) COPYRIGHT IOTON Technology **********************/
/***********************************END OF FILE********************************/
