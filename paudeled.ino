#include <math.h>
#include <avr/pgmspace.h>
#include "apa102dmaspi.h"
#include "images.h"
#include "font.h"

#define CS 10
#define STRIP_LEN 60   // quantidade de leds
#define STRIP_LEN_FRONT 45   // quantidade de leds
#define SECTORS 360
#define BUFF_LEN 4+STRIP_LEN*4+4 // 4 start frame, 4 por led, 4 end frame
#define BRIGHTNESS 0x1F
#define HALL_PIN 2  // porta do sendor hall, precisa suportar interrupcao

void turn();

volatile unsigned long tic; // Instante da ultima passada pelo sensor hall
volatile float K = 0.1;     // constante que converte o instante de tempo no angulo (setor)
                            // eh 2PI/periodo. 1000RPM => periodo = 60ms => k = 0.1

uint8_t sectors[SECTORS][BUFF_LEN]; // ocupa 89280 bytes

void setup() {
  Serial.begin(115200);
  
  // start/end frame
  for(uint16_t theta=0; theta<SECTORS; theta++){
    for(uint8_t i=0;i<4;i++){
      sectors[theta][i]=0x00;           // A start frame of 32 zero bits (<0x00> <0x00> <0x00> <0x00>)
      sectors[theta][BUFF_LEN-i]=0xFF;  // An end frame consisting of at least (n/2) bits of 1, where n is the number of LEDs in the string.
    }
    for(uint8_t i=STRIP_LEN_FRONT;i<STRIP_LEN;i++){
      sectors[theta][4*(i+1)+0]=0xE0|0;
      sectors[theta][4*(i+1)+1]=0;
      sectors[theta][4*(i+1)+2]=0;
      sectors[theta][4*(i+1)+3]=0;
    }
    sectors[theta][4*(STRIP_LEN_FRONT+2)+0]=0xE0|BRIGHTNESS;
    sectors[theta][4*(STRIP_LEN_FRONT+2)+1]=0;
    sectors[theta][4*(STRIP_LEN_FRONT+2)+2]=0;
    sectors[theta][4*(STRIP_LEN_FRONT+2)+3]=255;
  }

  pinMode(HALL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), turn, RISING);

  pinMode(CS,OUTPUT);
  digitalWrite(CS,LOW);
  
  spiBegin();
  spiInit(3);

//  writeMonoStr("%",0);
  loadImage();
  tic = millis();
}

/**
 * Funcao chamada quando acontece uma interrupcao no HALL_PIN
 * Atualiza o periodo
 */
void turn() {
  unsigned long tac = millis();
  unsigned long period = tac - tic; // Periodo de uma volta em milissegundos. Se for 1000rpm da 60ms.
  tic = tac;
  K = 360.0/period;
}

/**
 * 360 setores (1.3cm definicao no maior raio) a 12fps (720RPM) => 4320 atualizacoes/segundo
 * logo, o loop precisa rodar em 230 microssegundos (micro, nao mili)
 */
void loop() {
  unsigned long now, tac;
  float theta;
  uint16_t theta2;
  long start = millis();
  for(unsigned cont=0;cont<100000;cont++){
    now = millis();
    tac = now-tic;
    theta = K*tac;
    theta2=((int)theta)%SECTORS;
    spiSend(sectors[theta2],BUFF_LEN);
//    if(theta<360)
//      spiSend(sectors[(int)theta],BUFF_LEN);
//    else
//      spiSend(sectors[0],BUFF_LEN);
    //delayMicroseconds(50); //atualiza parte da imagem
    /*
     * A primeira estrategia eh atualizar o setor antes de enviar
     * Atualizar depois de enviar eh burro
     * 
     * Estou a pelo menos 600RPM (10 voltas por segundo, 0.1 por volta) entao o loop roda pelo menos 360 em 1 voltatem no máximo 0.1seg
     */
  }
  long finish = millis();
  Serial.print ((finish-start)/100.0);  
  Serial.println (" us"); 
  Serial.print (100000000.0/(finish-start));  
  Serial.println (" Hz"); 
}

void loadImage(){
  float s, c;
  for(uint16_t theta=0; theta<SECTORS; theta++){
    s = sin(2*3.14*theta/360);
    c = cos(2*3.14*theta/360);
    for(uint8_t k=0;k<STRIP_LEN_FRONT;k++) {
      uint8_t i = s*k+STRIP_LEN_FRONT;
      uint8_t j = c*k+STRIP_LEN_FRONT;
//      unsigned char idx = (unsigned char)pgm_read_byte_near(&header_data[i*2*STRIP_LEN_FRONT+j]);
      // A 32 bit LED frame for each LED in the string (<0xE0+brightness> <blue> <green> <red>)
      uint8_t idx = header_data_juca[i*2*STRIP_LEN_FRONT+j];
      sectors[theta][4*(k+1)+0]=0xE0|BRIGHTNESS; //TODO: possivelmente desnecessario, pode fazer uma vez so no setup
      sectors[theta][4*(k+1)+1]=header_data_cmap_juca[idx][2];
      sectors[theta][4*(k+1)+2]=header_data_cmap_juca[idx][1];
      sectors[theta][4*(k+1)+3]=header_data_cmap_juca[idx][0];
    }
  }
}

/*
void writeMonoStr(char* s, int theta) {
  for(int i=0;s[i]!='\0';i++){
    theta+=writeMonoChar(s[i],theta,200,200,200);
  }
//  for(int i=0;i<STRIP_LEN_FRONT;i++){
//    for(int j=0;j<SECTORS;j++){
//      Serial.print(sectors[j][4*(i+1)+0]>0?"*":".");
//    }
//    Serial.println();
//  }
}

unsigned int writeMonoChar(unsigned int c, int theta, int r, int g, int b){
  unsigned int idx = c-32;
  unsigned int bytewidth = 1+(CHARWIDTH[idx]-1)/8;
  unsigned int width = CHARWIDTH[idx];
  unsigned int height = min(CHARHEIGHT,STRIP_LEN_FRONT);

  for(int i=0;i<height;i++){
    for(int j=0;j<bytewidth;j++){
      char c=CHARTABLE[idx][i*bytewidth+j];
      for(int k=0;k<8 && j*8+k<width;k++){
        // A 32 bit LED frame for each LED in the string (<0xE0+brightness> <blue> <green> <red>)
        sectors[j*8+k+theta][4*(i+1)+0]=(c>>(7-k))&1;
        sectors[j*8+k+theta][4*(i+1)+1]=b;
        sectors[j*8+k+theta][4*(i+1)+2]=g;
        sectors[j*8+k+theta][4*(i+1)+3]=r;
      }
    }
  }
  return width;
}

void clearSector(unsigned int theta1, unsigned int theta2){
  for(unsigned int theta=theta1; theta<SECTORS; theta++){
    for(int k = 0;k<STRIP_LEN_FRONT;k++) {
      // A 32 bit LED frame for each LED in the string (<0xE0+brightness> <blue> <green> <red>)
      sectors[theta][4*(k+1)+0]=0xE0|0;
//      sectors[theta][4*(k+1)+1]=0;
//      sectors[theta][4*(k+1)+2]=0;
//      sectors[theta][4*(k+1)+3]=0;
    }
  }
}
*/
