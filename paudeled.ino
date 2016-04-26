#include <math.h>
#include <avr/pgmspace.h>
#include <dmaspi.h>

/**
 * Este arquivo foi exportado pelo GIMP de uma imagem "indexed".
 * Na declaracao dos dois vetores foi acrescentado os modificadores
 * const e PROGMEM para nao exceder a SRAM.
 **/
#include "/home/oda/sketchbook/paudeled_0_4/img/Logo_Garoa_Cor.h"
//#include "/home/oda/sketchbook/paudeled_0_4/img/teste.h"

#define CS 10
#define SPI_RATE 8

#define STRIP_LEN 60   // quantidade de leds
#define STRIP_LEN_FRONT 45   // quantidade de leds
#define BUFF_LEN STRIP_LEN*4+8
#define BRIGHTNESS 0x1F
#define HALL_PIN 2  // porta do sendor hall, precisa suportar interrupcao
#define HALL_INT 0  // numero de interrupcao, deveria ser o valor da funcao digitalPinToInterrupt,
                    // mas ela existe nas versoes mais novas
void turn();

volatile long period = 60;  // Periodo de uma volta em milissegundos, atualizado na funcao turn
                            // quando acontece um interrupcao no HALL_PIN. Se for 1000rpm da 60ms.

volatile long tic;  // Instante da ultima passada pelo sensor hall

uint8_t leds[BUFF_LEN];

void setup() {
  pinMode(HALL_PIN, INPUT);
  attachInterrupt(HALL_INT, turn, RISING);
  tic = millis();

  pinMode(CS,OUTPUT);
  digitalWrite(CS,LOW);
  
  spiBegin();
  spiInit(SPI_RATE);

  for(int i=0;i<BUFF_LEN;i++)
    leds[i]=0x00;
  for(int i=BUFF_LEN-4;i<BUFF_LEN;i++)
    leds[i]=0xFF;
  for(int i=1;i<STRIP_LEN;i++)
    leds[i*4]=0xE0|BRIGHTNESS;

}

/**
 * Funcao chamada quando acontece uma interrupcao no HALL_PIN
 * Atualiza o periodo
 */
void turn() {
  long tac = millis();
  period = tac - tic;
  tic = tac;
}
/**
 * 235 setores (definicao) a 12fps => 2820 atualizacoes/segundo
 * logo, o loop precisa rodar em 560 microssegundos (micro, nao mili)
 */
void loop() {
//  long bench=micros();
  long now = millis();
  long tac = now-tic;
  float theta = TWO_PI*tac/period + PI;
  float s = sin(theta);
  float c = cos(theta);
  for(int k = 0;k<STRIP_LEN_FRONT;k++) {
    int i = s*k+STRIP_LEN_FRONT;
    int j = c*k+STRIP_LEN_FRONT;
    unsigned char idx = (unsigned char)pgm_read_byte_near(&header_data[i*width+j]);
    leds[4*(k+1)+1]=255;//header_data_cmap[idx][0];
    leds[4*(k+1)+2]=255;//header_data_cmap[idx][1];
    leds[4*(k+1)+3]=255;//header_data_cmap[idx][2];
  }
  spiSend(leds,BUFF_LEN);
//  long mark=micros();
}

