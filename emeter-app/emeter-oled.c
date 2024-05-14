#include <inttypes.h>
#if defined(__MSP430__)
#include <msp430.h>
#endif
#include <stdio.h>
#include "emeter-oled.h"  
#include "oled-font.h"

// Endere�o do SSD1306
#define SSD1306_ADDRESS 0x3C



// Fun��es de controle I2C
void I2C_init();
void I2C_start();
void I2C_stop();
void I2C_send(uint8_t byte);
void SSD1306_sendCommand(uint8_t command);

/*
void main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Desabilitar Watchdog Timer

    // Inicializar I2C
    I2C_init();

    // Inicializar o display SSD1306
    SSD1306_sendCommand(SSD1306_SET_DISPLAY_ON);

    // Escrever seu c�digo para controlar o display SSD1306 aqui

    while(1) {
        // Seu c�digo principal aqui
    }
}*/



void i2c_init() {

    // Configurar I2C
    UCB0CTLW0 |= UCSWRST;                      // Colocar e-SPI em estado de reinicializa��o para configura��o
    UCB0CTLW0 |= UCMODE_3 | UCSYNC | UCMST|UCSSEL__SMCLK; // I2C master mode, Modo I2C, S�ncrono
    //UCB0CTLW0 |= UCMODE_3 | UCMST|UCSSEL__SMCLK; // I2C master mode, Modo I2C, S�ncrono    
    //UCB0CTLW1 |= UCASTP_2;                      // Autogera��o de STOP
    //UCB0BRW = 250;                              // Fator de divis�o do clock SMCLK
    UCB0BRW = 250/2;                              // Fator de divis�o do clock SMCLK
    UCB0IE = 0; 
    UCB0CTLW0 &= ~UCSWRST;                     // Liberar e-SPI da reinicializa��o

    UCB0I2CSA=0x3C;  // UCB0I2CSA = SlaveAddress
    
    while (UCB0STATW & UCBBUSY);                 // Wait for I2C bus to be ready
}

void I2C_start() {
    UCB0CTLW0 |= UCTR | UCTXSTT;               // Transmitir modo, Gerar START
    while (UCB0CTLW0 & UCTXSTT);               // Aguardar at� que o START seja enviado
}

void I2C_stop() {
    UCB0CTLW0 |= UCTXSTP;                      // Gerar STOP
    while (UCB0CTLW0 & UCTXSTP);               // Aguardar at� que o STOP seja enviado
}

void I2C_send(uint8_t byte) {  
    UCB0TXBUF = byte;                          // Transmitir byte
    while (!(UCB0IFG & UCTXIFG));              // Aguardar at� que a transmiss�o seja conclu�da
}

void SSD1306_Init(void) {
  
#define OLED_1306 // SSD1306 ou SSH1106
// #undef OLED_1306 // SSD1306 ou SSH1106
  
    // SSD1306 init sequence
    ssd1306_command(SSD1306_DISPLAYOFF);                                // 0xAE
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);                        // 0xD5
    ssd1306_command(0x80);                                              // the suggested ratio 0x80

    ssd1306_command(SSD1306_SETMULTIPLEX);                              // 0xA8
    ssd1306_command(SSD1306_LCDHEIGHT - 1);

    ssd1306_command(SSD1306_SETDISPLAYOFFSET);                          // 0xD3
    ssd1306_command(0x0);                                               // no offset
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0);                        // line #0
    ssd1306_command(SSD1306_CHARGEPUMP);                                // 0x8D

#ifdef OLED_1306  
    ssd1306_command(0x14);                                              // generate high voltage from 3.3v line internally
#else
    ssd1306_command(0x10);                                              // get high voltage from 3.3v line externally
#endif

    ssd1306_command(SSD1306_MEMORYMODE);                                // 0x20
    ssd1306_command(0x00);                                              // 0x0 act like ks0108
    ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);

    ssd1306_command(SSD1306_SETCOMPINS);                                // 0xDA
    ssd1306_command(0x12);
    ssd1306_command(SSD1306_SETCONTRAST);                               // 0x81
    
#ifdef OLED_1306    
    ssd1306_command(0xCF);
#else
    ssd1306_command(0x9F);                                              // SH1106_EXTERNALVCC
#endif
    
    ssd1306_command(SSD1306_SETPRECHARGE);                              // 0xd9
    
#ifdef OLED_1306 
    ssd1306_command(0xF1);
#else  
    ssd1306_command(0x22);
#endif   

    ssd1306_command(SSD1306_SETVCOMDETECT);                             // 0xDB
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);                       // 0xA4
    ssd1306_command(SSD1306_NORMALDISPLAY);                             // 0xA6

#ifdef OLED_1306 
    ssd1306_command(SSD1306_DEACTIVATE_SCROLL);
#endif
    
    ssd1306_command(SSD1306_DISPLAYON);                                 //--turn on oled panel
}

void ssd1306_command(unsigned char command) {

    I2C_start();
    I2C_send(0x80);
    I2C_send(command);
    I2C_stop();
    
    
} // end ssd1306_command

void ssd1306_setPosition(uint8_t column, uint8_t page) {
    if (column > 128) {
        column = 0;                                                     // constrain column to upper limit
    }

    if (page > 8) {
        page = 0;                                                       // constrain page to upper limit
    }

#ifdef OLED_1306
    ssd1306_command(SSD1306_COLUMNADDR);
    ssd1306_command(column);                                            // Column start address (0 = reset)
    ssd1306_command(SSD1306_LCDWIDTH-1);                                // Column end address (127 = reset)

    ssd1306_command(SSD1306_PAGEADDR);
    ssd1306_command(page);                                              // Page start address (0 = reset)
    ssd1306_command(7);                                                 // Page end address
#else
    ssd1306_command(0xB0+page);      
    column+=2;
    ssd1306_command(0x10+((column>>4)&0x07));
    ssd1306_command(0x00+(column&0x0F));
                                    // Column start address (0 = reset)
#endif    
} // end ssd1306_setPosition



void ssd1306_clearDisplay(void) {

        uint8_t page, segments, parts;

	for (page = 0; page < 8; page++)
	{
		// move to the beginning of the next page
                ssd1306_setPosition(0,page);
		for (segments = 0; segments < 8; segments++)
		{
                                    
                        I2C_start();
                        I2C_send(0x40);
			// no need to set the draw position, as every memory write advances the write pos to the next one
			for (parts = 0; parts < 16; parts++)
			{
				I2C_send(0x00);
			}
                        I2C_stop();
		}
	}
	
}


void ssd1306_printText(uint8_t x, uint8_t y, char *ptString) {
  
    int i;
    char h[6],l[6],c;
    
  while (*ptString != '\0') {
      
    if ((x>10)||(y>3))
      break;
    
    // gera nova matriz da fonte
    for(i = 0; i< 5; i++) {
          c = font_5x7[*ptString - ' '][i];
          
         
//          h[i] =                 ((c&0x01)<<6) + ((c&0x01)<<5) + ((c&0x02)<<3) + ((c&0x02)<<2) + ((c&0x04)>>0) + ((c&0x04)>>1) + ((c&0x08)>>3);
//          l[i] = ((c&0x08)<<4) + ((c&0x10)<<1) + ((c&0x10)<<2) + ((c&0x20)>>1) + ((c&0x20)>>2) + ((c&0x40)>>4) + ((c&0x40)>>5);

          
          h[i] =                 ((c&0x40)>>0) + ((c&0x40)>>1) + ((c&0x20)>>1) + ((c&0x20)>>2) + ((c&0x10)>>2) + ((c&0x10)>>3) + ((c&0x08)>>3);
          l[i] = ((c&0x08)<<4) + ((c&0x04)<<4) + ((c&0x04)<<3) + ((c&0x02)<<3) + ((c&0x02)<<2) + ((c&0x01)<<2) + ((c&0x01)<<1);

    }    
 
    // Envia primeira metade
    ssd1306_setPosition(x*12,y<<1);
    I2C_start();
    I2C_send(0x40);
    for(i = 0; i< 5; i++){
      I2C_send(l[i]);   // dobra largura
      I2C_send(l[i]);      
    }
    I2C_stop();  
    
    
    // Envia segunda metade
    ssd1306_setPosition(x*12, (y<<1)+1);
    I2C_start();
    I2C_send(0x40);
    for(i = 0; i< 5; i++){
      I2C_send(h[i]);   // dobra largura
      I2C_send(h[i]);      
    }
    I2C_stop();  

    x++;
    ptString++;
    
  }

} // end ssd1306_printText



void ssd1306_printText_peq(uint8_t x, uint8_t y, char *ptString) {
  
    int i;
    char l[6];
    
  while (*ptString != '\0') {
      
    if ((x>20)||(y>7))
      break;
    
    // gera nova matriz da fonte
    for(i = 0; i< 5; i++) {
          l[i] = font_5x7[*ptString - ' '][i];
    }    
 
    // Envia fonte completa
    ssd1306_setPosition(x*6,y);
    I2C_start();
    I2C_send(0x40);
    for(i = 0; i< 5; i++){
      I2C_send(l[i]); 
    }
    I2C_stop();  

    x++;
    ptString++;
    
  }

} // end ssd1306_printText


void Oled_teste2() {
  ssd1306_clearDisplay();
  ssd1306_printText(0, 0, "Ztec1234567");
  ssd1306_printText(0, 1, "ABCDEFGHIJK");
  ssd1306_printText(0, 2, "LMNOPQERTXU");
  ssd1306_printText(0, 3, "VZ=+-<>./*%");  
}

void Oled_teste() {
    char oled[32];
        ssd1306_clearDisplay();
      //ssd1306_printText(0, 0, "01234567890");
        ssd1306_printText(0, 0, "Frequencia");
        //int f1=0;
        //printf("f1 = %d",f1);
        //printf(" --- ");
        //f1=mains_frequency(1);
        //printf("%d \n",f1);
        //sprintf(oled,"F1=%5dH",f1);
        sprintf(oled,"1= 59.94 Hz");
        ssd1306_printText(0, 1, oled);        
        sprintf(oled,"2= 59.94 Hz");
        ssd1306_printText(0, 2, oled);   
        sprintf(oled,"3= 59.94 Hz");
        ssd1306_printText(0, 3, oled);           
};


void Oled_display_valor(int x, int y, long valor, int dec, char* inicial, char* final) {
    
    char oled[32]="Erro" ;  

    float v_float;
    
    v_float= (float) valor;

    if (dec==1) {
      v_float/=10.0;
      sprintf(oled,"%s%6.1f%s", inicial, v_float, final);
    }      
    if (dec==2) {
      v_float/=100.0;
      sprintf(oled,"%s%6.2f%s", inicial, v_float, final);
    }  
    if (dec==3) {
      v_float/=1000.0;
      sprintf(oled,"%s%8.3f%s", inicial, v_float, final);
    }  
    if (dec==4) {
      v_float/=10000.0;
      sprintf(oled,"%s%7.4f%s", inicial, v_float, final);      
    }  
    if (dec==5) {
      v_float/=100000.0;
      sprintf(oled,"%s%7.5f%s", inicial, v_float, final);      
    }    
    ssd1306_printText_peq(x, y, oled);     
    
}

void Oled_titulo(char* titulo){
  
    int s=strlen(titulo);
   
    ssd1306_printText_peq((21-s)>>1,0,titulo);
    //ssd1306_printText_peq(0,1,"=====================");
    ssd1306_printText_peq(0,1,"---------------------");  
}  

