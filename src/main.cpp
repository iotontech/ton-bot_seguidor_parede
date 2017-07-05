/**
******************************************************************************
* @file    main.cpp
* @author  Kleber Lima da Silva (kleber@ioton.cc)
* @version V0.0.1
* @date    05-Julho-2017
* @brief   Programa de exemplo do Robô TON-BOT no modo seguidor de parede.
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
#include "main.h"

/** @addtogroup Seguidor de parede TON-BOT
 * @{
 */
/* Constantes ----------------------------------------------------------------*/
#define ESQUERDA 1
#define DIREITA 2

/* Variáveis privadas --------------------------------------------------------*/
Ticker timer1;
bool bRodarControle = false;
uint8_t seguidor = 0;
uint8_t paredes = 0;
float lf = 0, l = 0, r = 0, rf = 0;

/* Funções Privadas ----------------------------------------------------------*/
/* Callback do Timer 1 - Controle de Velocidade */
void callbackTimer1(void)
{
  if (bRodarControle == true)
  {
    /* Realiza a leitura dos sensores */
    paredes = getSensoresParede(&lf, &l, &r, &rf);

    /* Realiza o controle de velocidade */
    controleVelocidade();
  }
}

/**
 * @brief  Programa Principal
 * @param  Nenhum
 * @retval Nenhum
 */
int main(void)
{
  /* Ligue o TON com o botão USER pressionado para indicar o estado da bateria */
  if (ton.USERisPressed())
  {
    /* Necessário pressionar o botão RESET voltar ao modo normal */
    ton.batteryStatus();
  }

  /* Inicialização do robô... */
  /* Calibrar os valores FRONTAL_TH e LATERAL_TH usando o ton-bot_teste,
    depois atualizar os valores no arquivo controlador.h */
  ton.setLED(GREEN);
  initTonBot(PRETA, FRONTAL_TH, LATERAL_TH);

  /* Inicio do programa ------------------------------------------------------*/
  /* Será habilitado seguidor de parede esquerda ou direita de acordo com o
   respectivo sensor frontal acionado */
  while ((getSensoresParede(&lf, &l, &r, &rf) & 0b010) == 0)
  {
    wait_ms(100);
    ton.toggleLED(RED);
  }
  seguidor = (lf > rf) ? ESQUERDA : DIREITA;
  wait(2);  // Aguarda um tempo para a partida do micromouse
  ton.setLED(BLUE);
  beeps(3, 100, 50);

  /* Inicia o Timer do controle de velocidade */
  timer1.attach(&callbackTimer1, (float)TS / 1000.0f);
  bRodarControle = true;

  /* Ao iniciar, o micromouse deve estar no centro da primeira célula...
   Com isso, deve se deslocar meia célula até a FRONTEIRA da próxima célula
   (local IDEAL para realizar a leitura dos sensores) */
  frente(CELULA / 2);


  /* LOOP principal ----------------------------------------------------------*/
  while (1)
  {
    /* Realiza a máquina de estados de acordo com o que foi selecionado */
    if (seguidor == DIREITA)
    {
      /* Máquina de Estados - Seguidor de Parede DIREITA */
      switch (paredes)
      {
        /* Sem paredes */
        case 0b000:
        curva(-90);	// Vira para a direita
        break;

        /* Apenas parede da direita */
        case 0b001:
        frente(CELULA);	// Anda uma célula para frente
        break;

        /* Apenas parede da esquerda */
        case 0b100:
        curva(-90);	// Vira para a direita
        break;

        /* Ambas paredes laterais */
        case 0b101:
        frente(CELULA);	// Anda uma célula para frente
        break;

        /* Apenas parede frontal */
        case 0b010:
        curva(-90);	// Vira para a direita
        break;

        /* Parede frontal e direita */
        case 0b011:
        curva(90);	// Vira para a esquerda
        break;

        /* Parede frontal e esquerda */
        case 0b110:
        curva(-90);	// Vira para a direita
        break;

        /* Todas as paredes */
        case 0b111:
        curva(180);	// Meia volta
        break;
      }
    } // fim da maquina de estados para seguidor de parede direita
    else if (seguidor == ESQUERDA)
    {
      /* Máquina de Estados - Seguidor de Parede ESQUERDA */
      switch (paredes)
      {
        /* Sem paredes */
        case 0b000:
        curva(90);	// Vira para esquerda
        break;

        /* Apenas parede da direita */
        case 0b001:
        curva(90);	// Vira para esquerda
        break;

        /* Apenas parede da esquerda */
        case 0b100:
        frente(CELULA);	// Anda uma célula para frente
        break;

        /* Ambas paredes laterais */
        case 0b101:
        frente(CELULA);	// Anda uma célula para frente
        break;

        /* Apenas parede frontal */
        case 0b010:
        curva(90);	// Vira para esquerda
        break;

        /* Parede frontal e direita */
        case 0b011:
        curva(90);	// Vira para a esquerda
        break;

        /* Parede frontal e esquerda */
        case 0b110:
        curva(-90);	// Vira para direita
        break;

        /* Todas as paredes */
        case 0b111:
        curva(-180);	// Meia volta
        break;
      }
    } // fim da maquina de estados para seguidor de parede esquerda
  } // fim do loop principal
} // fim da função principal


/**
 * @}
 */

/************************ (C) COPYRIGHT IOTON Technology **********************/
/***********************************END OF FILE********************************/
