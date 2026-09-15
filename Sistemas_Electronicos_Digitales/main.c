#include <LPC17xx.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "reg_masks.h"
#include "GLCD.h"

/////////////////////////////////////////////////////////////////////////
               //Alex Zurdo Fernández y Mohsen Messadia//                
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
                                 //LCD//
/////////////////////////////////////////////////////////////////////////
//VARIABLES GLCD
	uint16_t Xpos, Ypos;
  int line;
	static char lcd_buffer[256];
	#define FONT_W  8
	#define FONT_H  16

/////////////////////////////////////////////////////////////////////////
              //HITO 1: MOTORES//
/////////////////////////////////////////////////////////////////////////

//CONSTANTES
#define F_PCLK		25e6 //25MHz del IRC
#define T_PWM_SERVO		(1/4e3)		// Frecuencia del PWM de 4 kHz

#define MOTOR_MASK      ((1<<1)|(1<<4)|(1<<8)|(1<<9))
#define MOVE_FORWARD 		(1<<1)|(0<<4)|(1<<8)|(0<<9)
#define MOVE_BACKWARD 	(0<<1)|(1<<4)|(0<<8)|(1<<9)
#define TURN_RIGHT 		  (1<<1)|(0<<4)|(0<<8)|(1<<9)
#define TURN_LEFT 		  (0<<1)|(1<<4)|(1<<8)|(0<<9) 
#define STOP 			       0 

#define Period_PWM (F_PCLK*T_PWM_SERVO-1)
#define K_CALIB 8/9


//VARIABLES
float Duty_cycle_L = 0.5;
float Duty_cycle_R = 0.5;

//Configurar la generación de las Señales PWM

void pwm_config(void) {
	//Sólo EN de motores tienen carac. PWM 
	//Elegir pines que trabajen asi -> (PWM1.3 y PWM1.4)
	// PWM1.3 (EN (I)) y PWM1.4 (EN(D)) en pines P1.21 y P1.24
	LPC_PINCON->PINSEL3 |= (2 << 10)	| (2 << 14);			
	//Definir 4 pines como IN's de cada motor
	//P1.4 y P1.1 = IN2:1 (D:C (I)) y P9:8 = IN4:3	(D:C (D))
	LPC_GPIO1->FIODIR |= (1 << 1)|(1<<4)|(1<<8)|(1<<9);
	//Definir valores de MR
	//MR0 define el periodo de las señales PWM
	LPC_PWM1->MR0 = Period_PWM;		
	//T_alto de PWM1.3 (I)
	LPC_PWM1->MR3 = (uint32_t)(Period_PWM)*Duty_cycle_L;	
	//T_alto de PWM1.4 (D)	
	LPC_PWM1->MR4 = (uint32_t)(Period_PWM)*Duty_cycle_R;
	//Habilitar cambio de MR0 3 y 5	
	LPC_PWM1->LER = (1 << 0) | (1 << 3)	| (1 << 4);							
	//Definimos comportamiento con TC
	//Resetear TC en caso de hacer match con MR0
	LPC_PWM1->MCR = (1 << 1);								
	//habilitaciones
	//Habilitamos PWM1.3 y PWM1.4
	LPC_PWM1->PCR |= (1 << 11)|(1<<12);
	//Aumentar TC y Habilitar PWM general	
	LPC_PWM1->TCR = (1 << 0) | (1 << 3);								
}

//Configuración del ciclo de trabajo de la señal PWM 
//cada rueda por separado

void pwm_set_duty_cycle(float duty_L ,float duty_R) {
	//si los ciclos de trabajo tienen módulo mayor a 1 
	//se satura dc a 1 (o -1) según el signo.
	if(duty_L > 1)
		duty_L = 1;
	if(duty_L < -1)
		duty_L = -1;
	if(duty_R > 1)
		duty_R = 1;
	if(duty_R < -1)
		duty_R = -1;
	
	//modificar los match del PWM -> T_Alto proporcional a velocidad
	
	//T_alto de PWM1.3 (Motor iquierdo)
	LPC_PWM1->MR3 = LPC_PWM1->MR0*fabs(duty_L*K_CALIB);	
	//se ha añadido K_CALIB para calibrar los motores. 
	//sus velocoidades no eran parejas, esta K los corrige
	//T_alto de PWM1.4 (Motor derecho)
	LPC_PWM1->MR4 = LPC_PWM1->MR0*fabs(duty_R);
	//Validar cambio en MR3 y MR4
	LPC_PWM1->LER = (1 << 3) | (1 << 4);				
	
	//excitación de los motores basado en el signo del ciclo de trabajo de cada rueda (sentido de giro)
	
	if(duty_L > 0 && duty_R > 0){				    //Ir para adelante
		//limpiar P1.17:14, poner = I: adelante, D: adelante
		LPC_GPIO1->FIOPIN = (LPC_GPIO1->FIOPIN & ~MOTOR_MASK) | MOVE_FORWARD;		
	}	else if(duty_L < 0 && duty_R < 0){	  //Ir marcha atras
		//limpiar P1.17:14, poner = I: atras, D: atras
		LPC_GPIO1->FIOPIN = (LPC_GPIO1->FIOPIN & ~MOTOR_MASK)	| MOVE_BACKWARD;	
	}	else if(duty_L > 0 && duty_R < 0){	  //Girar a la derecha
		//limpiar P1.17:14, poner = I: adelante, D: atras
		LPC_GPIO1->FIOPIN = (LPC_GPIO1->FIOPIN & ~MOTOR_MASK)	| TURN_RIGHT;			
	}	else if(duty_L < 0 && duty_R > 0){	  //Girar a la izquierda
		//limpiar P1.17:14, poner = I: atras, D: adelante
		LPC_GPIO1->FIOPIN = (LPC_GPIO1->FIOPIN & ~MOTOR_MASK) | TURN_LEFT;			  
	}	else if(duty_L == 0 && duty_R == 0){	//Parar
		//limpiar P1.17:14, poner = I: parar, D: parar
		LPC_GPIO1->FIOPIN = (LPC_GPIO1->FIOPIN & ~MOTOR_MASK) | STOP;				    
	}
}

/////////////////////////////////////////////////////////////////////////
                   //HITO 2: MEDIDAS DEL MOVIMIENTO //
/////////////////////////////////////////////////////////////////////////

//CONSTANTES
#define PI 3.14159265

#define D_WHEEL 7
#define D_ROBOT 27

#define PPR 11
#define RED 34
#define K_DISTANCE (D_WHEEL*PI)/(2*RED*PPR)
#define K_ANGLE 0.1247771836 
#define V_BAT 6
#define A_WHEEL 28.3 //rpm / V

float duty_diff;

uint8_t W_left = 0;
uint8_t W_right = 0;

//VARIABLES DEL ESTADO DE LAS RUEDAS

volatile uint8_t end_mov_L = 1;
volatile uint8_t end_mov_R = 1;

//configurar los TIMER (TIM3:2 para que funcionen en modo COUNTER

void enc_config(void) {
	
	LPC_SC->PCONP |= (1 << 22) | (1 << 23);					//encender TIM2:3
	
	//CAP2.0 en P0.4, para leer al motor derecho
	LPC_PINCON->PINSEL0 &= ~(0x3 << 8);
	LPC_PINCON->PINSEL0 |=  (0x3 << 8);

	//CAP3.0 en P0.23, para leer al motor izquierdo
	LPC_PINCON->PINSEL1 &= ~(0x3 << 14); // limpia
	LPC_PINCON->PINSEL1 |=  (0x3 << 14); // pone función CAP3.0
	
	//Deshabilitar resistencias Pull-Up y Pull-down en los pines capture
	LPC_PINCON->PINMODE0 = (0x2 << 8);
	LPC_PINCON->PINMODE1 = (0x2 << 14);
	
	LPC_TIM2->CTCR = (0x3<<0);		//CAP2.1 contar flancos de SUBIDA Y BAJADA
	LPC_TIM3->CTCR = (0x3<<0);		//CAP3.1 contar flancos de SUBIDA Y BAJADA
	
	//IRQ y resetear TC cuando se hace match con MR2.0
	LPC_TIM2->MCR = (3 << 0);	
	//IRQ y resetear TC cuando se hace match con MR3.0	
	LPC_TIM3->MCR = (3 << 0);						
	//A MR0 no se le da valor, depende de la distancia a recorrer deseada
	//se define antes de cada moviemiento nuevo en una IRQ
	LPC_TIM2->TCR = (1 << 1);						//TC a 0
	LPC_TIM3->TCR = (1 << 1);						//TC a 0
	//Habilitar las IRQ de ambos TIMER
	NVIC_EnableIRQ(TIMER2_IRQn);
	NVIC_EnableIRQ(TIMER3_IRQn);
	NVIC_SetPriority(TIMER2_IRQn,7); 	//Configurar prioridad de la ISR
	NVIC_SetPriority(TIMER3_IRQn,5); 	//Configurar prioridad de la ISR
}

//Medir la distancia recorrida en cada rueda  en tiempo real 
//luego podrá mostrarse el el display/pantalla

float enc_get_dist() {
	if(W_left){
		return (LPC_TIM3->TC / (float)(K_DISTANCE)); //TIM3 pasado a cm
	}else{
		return (LPC_TIM2->TC / (float)(K_DISTANCE)); //TIM2 pasado a cm
	}
}

//Medir el ángulo que se ha girado en tiempo real 
//luego podrá mostrarse el el display/pantalla

float enc_get_angle() {
	if(W_left){
		return (LPC_TIM3->TC / (float)(K_ANGLE)); //TIM3 pasado a grados
	}else{
		return (LPC_TIM2->TC / (float)(K_ANGLE)); //TIM2 pasado a grados
	}
		}

//Medir velocidad del robot

float enc_get_speed(){
	
	float angular_speed;
	float linear_speed;
	
	//v_angular = A (dada por traspa) * ciclo * Tensión de la batería
	angular_speed	= ((A_WHEEL) * (Duty_cycle_L) * (V_BAT)); 
	// V_lineal = v_angular * radio de la rueda (Diametro / 2)
	linear_speed = ((angular_speed) * ((D_WHEEL) / 2)); 
	return(linear_speed); //velocidad está en cm/s

}

//Definir la distancia a recorrer para el próximo movimiento

void enc_set_dist_limit(float dist_cm) {
	
	//MR0 se define como la distancia a recorrer 
	//TC se pone a 0 para medir distancia
	//hacer match al llegar al valor de MR0
	float n_dist;
	n_dist = (uint32_t)(dist_cm)/(float)(K_DISTANCE);
	
	//se pasa de distancia en cm a flancos por contar
	
	LPC_TIM2->MR0 = n_dist;
	LPC_TIM3->MR0 = n_dist;
	
	//se devuelven los timer al 0 para poder 
	//iniciar el conteo para el siguiente movimiento
	LPC_TIM2->TCR = (1 << 1);
	LPC_TIM3->TCR = (1 << 1);
	
	//se enciende el Timer para que pueda empezar a contar flancos
	LPC_TIM2->TCR = (1 << 0);	
	LPC_TIM3->TCR = (1 << 0);
}

//Definir ángulo de giro para el próximo giro

void enc_set_angle_limit(float robot_angle) {
	//Igual que 'enc_set_dist_limit' pero definiendo 
	//el ángulo de giro del robot para el próximo giro
	
	float n_angle;
	
	if(W_right){
	duty_diff = (fabs(Duty_cycle_R) / fabs(Duty_cycle_L * K_CALIB));
	}else{
	duty_diff = (fabs(Duty_cycle_L * K_CALIB) / fabs(Duty_cycle_R));
	}

	n_angle	= ((robot_angle)/(float)(K_ANGLE))*duty_diff;
	
	LPC_TIM2->MR0 = (uint32_t) n_angle;
	LPC_TIM3->MR0 = (uint32_t) n_angle;
	
	//se devuelven los timer al 0 para poder iniciar 
	//el conteo para el siguiente ángulo
	LPC_TIM2->TCR = (1 << 1);
	LPC_TIM3->TCR = (1 << 1);
	
	//se enciende el Timer para que pueda empezar a contar flancos
	LPC_TIM2->TCR = (1 << 0);	
	LPC_TIM3->TCR = (1 << 0);	
}

//HANDELR DEL TIMER 2

void TIMER2_IRQHandler(void) {
	//'avisar' al timer usado que se ha producido match con MR0
	//y que se ha terminado el movimiento
	LPC_TIM2->IR = (1 << 0);	//Limpiar flag por MR0
	//en caso de mov no-honolómico, si duty_R es mayor a duty_L 
	//y se produce match en el timer 2, se levanta el flag de fin de mov.
	//para ambos motores. -> en movimientos no-honolómicos
	//dist/angle limit es para el motor de "fuera", el más rapido
	if(W_right){ 
		end_mov_R = 1;
		end_mov_L = 1;
	}else{
		end_mov_R = 1;
	}
}

//HANDLER DEL TIMER 3

void TIMER3_IRQHandler(void) {
	LPC_TIM3->IR = (1 << 0);	//Limpiar flag por MR0
	//en caso de mov no-honolómico, si duty_L es mayor a duty_R 
	//y se produce match en el timer 3, se levanta el flag de fin de mov.
	//para ambos motores. -> en movimientos no-honolómicos
	//dist/angle limit es para el motor de "fuera", el más rapido
	if(W_left){
		end_mov_R = 1;
		end_mov_L = 1;
	}else{
		end_mov_L = 1;
	}
}

/////////////////////////////////////////////////////////////////////////
 //HITO 3: MONITOREO DE  LAS BATERÍAS Y GENERACIÓN DE SEÑAL DE ALARMA//
/////////////////////////////////////////////////////////////////////////

#define T_SAMPLING	5
#define SPAN_FACTOR 3			//divisor de resistencias para ADC
#define V_REF_P 3.3
float battery_voltage;	
uint32_t battery_update = 0;

#define N_POINTS 32
#define BEEP_T_MAX 0.5 //s
float f_tone = 500; //Hz 
int16_t alarm_table[N_POINTS];
uint8_t alarm_idx;

#define BAT_MAX 7.4
#define BAT_MIN	5.0
#define BATTERY_LIMIT 6.0
#define DIFF_BAT (BAT_MAX - BAT_MIN)
#define F_TONE_MAX	5000
#define F_TONE_MIN	200
#define DIFF_FREC (F_TONE_MAX - F_TONE_MIN)
uint32_t N_beep;
static uint32_t beep_idx = 0;

#define AMPLI_OFFSET 1.0
#define AMPLI_SCALE 511.5


//FUNCIONALIDAD DAC

 void alarm_gen_sample(void){
	
	 //lleva muestra indexada a DACR
	 //aumentar índice de forma circular (a final de array, poner i=0)
	 LPC_DAC->DACR = alarm_table[alarm_idx++];
		if(alarm_idx >= N_POINTS)
			alarm_idx = 0;
	 
 }
 
 
 void alarm_enable(int enable){
	
	 //controla activación del tono de alarma (determinado por 'enable')
	 //enable == 0 -> DESHABILITAR TONO
	 //enable != 0 -> HABILITAR TONO
	 //puede hacerse habilitando/deshabilitando 
	 //el timer que genera la interrupción periodica
	 
	 if(enable == 0){ 
			//DESHAB TONO
			LPC_TIM1->TCR = (1 << 1); //TC RESET PERMANENTE
	 }else{
			//HAB TONO
			LPC_TIM1->TCR = (1 << 0); //TC HABILITAR CONTEO
	 }
	 
 }
 
 void alarm_set_freq(int freq_hz){
	
	 //Modificar frecuencia del tono usado -> especificado por freq_Hz
	 alarm_enable(0); //deshab alarma mientras se cambia su frec
	 //modif T_alto del tono de alarma
	 LPC_TIM1->MR1 = 1e6/(freq_hz*N_POINTS)-1; 
	 
 }

 void alarm_init(void){
	
	//Generar tabla de valores de señal de la alarma -> señal senoidal
	for(alarm_idx = 0; alarm_idx < N_POINTS; alarm_idx++){
		alarm_table[alarm_idx] = (uint16_t)((sin(2*PI*alarm_idx/N_POINTS)+AMPLI_OFFSET)*AMPLI_SCALE) << 6; 
		//valor a lo lago del seno, offset +1 para evitar valores negativos
		//Luego, escalado para que valores generados cumplan con 
		//caracteristicas deseadas para la señal de alarma
	}
		
	alarm_idx = 0;
	
	//Config DAC (Solamente hay que configurar su pin de salida)
	LPC_PINCON->PINSEL1 |= (2<<20); //P0.26 como AOUT
	//desconectar R de pull del pin P0.26 para no saturar la salida
	LPC_PINCON->PINMODE1 |= (2 << 20);	
	
	//Inicializar variables del beep
	N_beep = 0.3 * N_POINTS * f_tone /  Duty_cycle_L;
	beep_idx = 0;
	
	//Config TIMER1
	//TIMER1 es el único que está libre -> cargar datos del DAC
	LPC_TIM1->PR = 24;									             //Resolución de  1 MHz
	LPC_TIM1->MCR = (3 << 3);						//Reset TC & IRQ tras match con MR1
	LPC_TIM1->TCR = (1 << 1);	 //RESET PERMANENTE TC (inicialmente apagada)
	NVIC_EnableIRQ(TIMER1_IRQn);                   //habilitar interrupción
	NVIC_SetPriority(TIMER1_IRQn,9); 	     //Configurar prioridad de la ISR
	alarm_set_freq(f_tone);
	alarm_enable(1);
	
	 
 }
 
 void TIMER1_IRQHandler(void) {
	LPC_TIM1->IR = (1 << 1);
	beep_idx++;
	if(beep_idx < (N_beep / 2)) {			//semiciclo en el que se genera señal
			alarm_gen_sample();
		}else{ //semiciclo en el que no se genera señal
			
		}
	if(beep_idx >= (N_beep)){
		//fin del semiciclo en el que hay silencio 
		//resetear indice para volver generar señal de nuevo
		beep_idx=0;
	}
}

//FUNCIONALIDAD ADC

 void battery_sampling_init(int Ts){
	
	//Config del ADC
	
	//configurar P0.24 como AD0.1
	LPC_PINCON->PINSEL1 |= (1 << 16);
	//desconectar resis. de pull del pin P0.24 para no saturar la salida
	LPC_PINCON->PINMODE1 |= (2 << 16);
	LPC_SC->PCONP |= (1 << 12);					//Power ON ADC
	LPC_ADC->ADCR = (1 << 1)						//Conversión del canal 1
								| (255 << 8)					//CLKDIV máximo, max posible T_conv
								| (1 << 21)						//PDN = 1, Habilitar conversión ADC
								| (4 << 24)						//START = flanco EDGE de MAT0.1    
								| (0 << 27);					//EDGE = subida (0)                
	LPC_ADC->ADINTEN = (1 << 1);				//IRQ al acabar conversión de CH1  
	NVIC_EnableIRQ(ADC_IRQn);						//Habilitar IRQ                    
	NVIC_SetPriority(ADC_IRQn,3); 	    //Configurar prioridad de la ISR   
	
	//Config del TIMER0 
	//ADC toma flancos internamente, no necesario configurar pin de salida
	 
	LPC_TIM0->PR = 24999;								      //da una resolución de 1 kHz
	LPC_TIM0->MR1 = 1e3*(Ts/2)-1;		           //T_toggle = T_muestreo / 2 
	LPC_TIM0->MCR = (1 << 4);				//Reset TC tras match con MR1 (MAT0.1)
	LPC_TIM0->EMR = (3 << 6);						//Toggle MAT0.1 tras match con MR1
	LPC_TIM0->TCR = (1 << 0);						         //habiliotar conteo de TC
	 
 }
 
 void battery_sampling_stop(void){
	
	 //Detener conversión / muestreo si así se desea en algún momento
	 //Basta sólamente con hacer un Reset permanente al TC de TIMER0
	 LPC_TIM0->TCR = (1 << 1);						         //Reset permanente de TC
	 
	 
 }

 void ADC_IRQHandler(void){
	
	//Conversión de battery_voltage
	//battery_voltage usada más adelante para habilitar alarma
	//usar 8 bits mayores, no hace falta tanta resolución
	//8 bits => 256 como máximo
	//como el rango de valores es [0 , 3.3] 
	//implementar un divisor de tensión 
	 //(tensión/256) * DIVISOR RESITORES * V-REF_P
	//campo SAMPLE del ADDR del CH1, obtener voltaje
	battery_voltage = (((LPC_ADC->ADDR1 >> 8) & 0xFF)/256.0)*V_REF_P*SPAN_FACTOR; 
	
	//convertir nivel de batería en frec para la alarma
	//cambiará con nivel de VBAT
	f_tone = (battery_voltage - BAT_MIN)*(DIFF_FREC / DIFF_BAT)+F_TONE_MIN;
	battery_update++;
	 
	 if(battery_voltage <= 6) {
		N_beep = 0.3 * N_POINTS * f_tone /  Duty_cycle_L;
		alarm_set_freq(f_tone);
		alarm_enable(1);
	} else {		
		alarm_enable(0);
	}
 }

/////////////////////////////////////////////////////////////////////////
                 //HITO 4: COMUNICACIÓN EN SERIE//
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
 //CONFIGURACIÓN UART (BB/github KEIL PROJECTS (uart_demo_interrupts))//
/////////////////////////////////////////////////////////////////////////

#define UART_ACCEPTED_BAUDRATE_ERROR    3	// Accepted Error baud rate value (in percent unit)
#define FUNCTION_TXD  0x1
#define FUNCTION_RXD  0x1
#define END_OF_LINE '\r'									//Putty sends \r when 'enter' key is pressed.

static volatile int tx_err_flag;
static volatile int rx_err_flag;
static volatile int tx_busy = 0;
static volatile int rx_busy = 0;
static const char *tx_ptr;
static char *rx_ptr;
#define N_BUFFER 32
char rx_buffer[N_BUFFER];

static int uart0_set_baudrate(unsigned int baudrate) {
  int errorStatus = -1; // failure

  unsigned int uClk = SystemCoreClock/4;
  unsigned int calcBaudrate = 0;
  unsigned int temp = 0;

  unsigned int mulFracDiv, dividerAddFracDiv;
  unsigned int divider = 0;
  unsigned int mulFracDivOptimal = 1;
  unsigned int dividerAddOptimal = 0;
  unsigned int dividerOptimal = 0;

  unsigned int relativeError = 0;
  unsigned int relativeOptimalError = 100000;

  uClk = uClk >> 4; // div by 16

  //  The formula is :
  //  BaudRate= uClk * (mulFracDiv/(mulFracDiv+dividerAddFracDiv) / (16 * DLL)
  //
  //  The value of mulFracDiv and dividerAddFracDiv should comply to the following expressions:
  //  0 < mulFracDiv <= 15, 0 <= dividerAddFracDiv <= 15
  for (mulFracDiv = 1; mulFracDiv <= 15; mulFracDiv++) {
    for (dividerAddFracDiv = 0; dividerAddFracDiv <= 15; dividerAddFracDiv++) {
      temp = (mulFracDiv * uClk) / (mulFracDiv + dividerAddFracDiv);
      divider = temp / baudrate;
      if ((temp % baudrate) > (baudrate / 2))
        divider++;

      if (divider > 2 && divider < 65536) {
        calcBaudrate = temp / divider;
        if (calcBaudrate <= baudrate) {
          relativeError = baudrate - calcBaudrate;
        } else {
          relativeError = calcBaudrate - baudrate;
        }

        if (relativeError < relativeOptimalError) {
          mulFracDivOptimal = mulFracDiv;
          dividerAddOptimal = dividerAddFracDiv;
          dividerOptimal = divider;
          relativeOptimalError = relativeError;
          if (relativeError == 0)
            break;
        }
      }
    }

    if (relativeError == 0)
      break;
  }

  if (relativeOptimalError < ((baudrate * UART_ACCEPTED_BAUDRATE_ERROR) / 100)) {
    LPC_UART0->LCR |= DLAB_ENABLE; 	// importante poner a 1
    LPC_UART0->DLM = (unsigned char) ((dividerOptimal >> 8) & 0xFF);
    LPC_UART0->DLL = (unsigned char) dividerOptimal;
    LPC_UART0->LCR &= ~DLAB_ENABLE;	// importante poner a 0

    LPC_UART0->FDR = ((mulFracDivOptimal << 4) & 0xF0) | (dividerAddOptimal & 0x0F);
    errorStatus = 0; // success
  }

  return errorStatus;
}

int uart0_cfg(int baudrate) {
  int ret;
  
  LPC_PINCON->PINSEL0 &= ~0xF0U;
  LPC_PINCON->PINSEL0 |=  (FUNCTION_TXD << 4)   | // Setup P0.2 as TXD0
                          (FUNCTION_RXD << 6);    // Setup P0.3 as RXD0 

  LPC_UART0->LCR = CHAR_8_BITS | STOP_1_BIT;  // Set 8N1 mode (8 bits, no parity, 1 stop bit)
  ret = uart0_set_baudrate(baudrate);         // Set the baud rate

  LPC_UART0->IER = UART_THRE_IE | UART_RBR_IE;  // Enable UART TX and RX interrupt (for LPC17xx UART)
  NVIC_EnableIRQ(UART0_IRQn);                   // Enable the UART interrupt (for Cortex-CM3 NVIC)
	NVIC_SetPriority(UART0_IRQn,0); 							//Configurar prioridad de la ISR
  return ret;
}

int uart0_send_string(const char *txt) {
  int ret = -1;
  if(!tx_busy) {
    tx_ptr = txt;
    LPC_UART0->THR = *tx_ptr++;    
    tx_busy = 1;
    ret = 0;
  }
  return ret;
}

int uart0_recv_string(char *txt) {
  int ret = -1;
  if(!rx_busy) {
    rx_ptr = txt;
    rx_busy = 1;
    ret = 0;
  }
  return ret;
}

void UART0_IRQHandler() {
  int int_id;
  
  int_id = GET_INT_ID(LPC_UART0->IIR);
  switch(int_id) {
    case RDA_INT_ID:  // Rx data ready
      if (rx_busy) {
        *rx_ptr = LPC_UART0->RBR;
        if (*rx_ptr == END_OF_LINE) {
          *rx_ptr = 0;
          rx_busy = 0;
        } 
				else {
					rx_ptr++;
				}
      }
      else {
				*rx_ptr = LPC_UART0->RBR;	//Para limpiar el flag de interrupción
        rx_err_flag = 1;
      }
      break;
    case THRE_INT_ID: // Tx data ready
      if (*tx_ptr) {
        LPC_UART0->THR = *tx_ptr++;
      }
      else {
        tx_busy = 0;
      }
      break;
    default:
      tx_err_flag = 1;      
  }
}

/////////////////////////////////////////////////////////////////////////
        //FINITE STATES MACHINE / MÁQUINA DE ESTADOS FINITOS//
/////////////////////////////////////////////////////////////////////////

//CONSTANTES
#define START				1		 //Enviar mesg inicial y recibir la cadena
#define PENDING			2		 //validar cadena, buscando errores
#define STANDBY			3		 //Cadena validada, esperar pulsación de KEY1
#define MOVEMENT		4		 //KEY1 pulsada, se comienza el movimiento
#define ERROR				5		 //Error en cadena
#define BR 					9600 //Baudrate de la UART

//VARIABLES
uint8_t machine_state = START; //ESTADO INICIAL PARA RECIBIR CADENA
uint8_t KEY1 = 0;
uint8_t End_of_Chain = 0;
uint8_t backwards= 0;
uint8_t M_lin = 0;
uint8_t M_ang = 0;

//CADENAS DE MENSAJES
char *msg_start = "\n\rPROGRAMMABLE ROBOT. Type movement chain: ";
char *msg_validation = "\n\rValid movement chain. Press KEY1 to start / restart execution";
char *msg_error = "\n\rMovement chain contains an error. Retype the movement chain: ";

void KEY1_config(void) {
	LPC_PINCON->PINSEL4 |= (1 << 22); //habilitar KEY1 como EINT0
	LPC_SC->EXTMODE |= (1 << 1); 			//Activación por flanco
	//activación por flanco de bajada 
	//(una vez se pulsa el botón y se deja de pulsar)
	LPC_SC->EXTPOLAR &= ~(1 << 1);		
	NVIC_EnableIRQ(EINT1_IRQn);				//Habilitar Interrupción
	NVIC_SetPriority(EINT1_IRQn,1); 	//Configurar prioridad de la ISR
}

void EINT1_IRQHandler(void) {
	//No puede controlarse movimiento directameste desde ISR 
	//pero puese subirse  flag 'KEY1' para iniciar ejecucción
	LPC_SC->EXTINT = (1 << 1);
	if(machine_state == STANDBY)
		KEY1 = 1;
}

uint8_t string_parsing(void) { 
	//VALIDAR LA CADENA RECIBIDA Y COMPROBAR QUE NO TENGA ERRORES
	uint8_t i;
	for(i = 0; i < N_BUFFER; i++){
		if(i == 0) {
			if(rx_buffer[i] != 'S')
				break;
		}
		else if(((i - 2) % 3 == 0) && ((i-2) > 0))	{		
			//Comprobar cada 3 carácteres -> formato Mxy (Mov. lim xy)
			if(rx_buffer[i] == 0)	
				//caso en el que se dectecta el fin de la cadena
				return 0;						//CADENA VÁLIDA -> PASAR A ESTADO 'STANDBY'
			else if( rx_buffer[i] != 'F' && rx_buffer[i] != 'B' && rx_buffer[i] != 'L' && rx_buffer[i] != 'R')
				break;
		}
		else {
			if(rx_buffer[i] < '0' || rx_buffer[i] > '9')
				break;
		}
	}
	//CADENA ERRONEA -> SE ENVIA MENSAJE DE ERROR -> ESTADO 'ERROR'
	return 1;									
}

void movement_control(void) {
	static uint8_t pos = 0;
	uint8_t value = 0; 
	uint8_t value_left = 0; //velocidad motor izquierdo
	uint8_t value_right = 0; //velocidad motor derecho
	static uint32_t tick_counter=0;
	uint32_t dist_lcd;
	uint32_t ang_lcd;
	if(end_mov_L == 1 && end_mov_R == 1) {
		
		tick_counter = 0;
		backwards = 0;
		
	switch (rx_buffer[pos]) {
			
			case 'S': //SPEED
				value_left = (rx_buffer[pos+1]-'0')*10  //decenas ASCII
										+ (rx_buffer[pos+2]-'0');		//unidades ASCII
				value_right = (rx_buffer[pos+3]-'0')*10  //decenas ASCII
										+ (rx_buffer[pos+4]-'0');		//unidades ASCII
				Duty_cycle_L = (float)(value_left)/100.0; //% a decimal
				Duty_cycle_R = (float)(value_right)/100.0;
				pos += 5; //Pasar 5 pos. para pasar al siguiente movimiento
				if(value_left >= value_right){
					W_left = 1;
					W_right = 0;
				}else{
					W_left = 0;
					W_right = 1;
				}
				break;
			case 'F': //FORWARDS
				value = (rx_buffer[pos+1]-'0')*10  //decenas ASCII
							+ (rx_buffer[pos+2]-'0');		//unidades ASCII
				enc_set_dist_limit((float)(value)); //distancia a recorrer
				pwm_set_duty_cycle(Duty_cycle_L, Duty_cycle_R);
				pos += 3;
				M_lin = 1;
				M_ang = 0;
				end_mov_L = 0;
				end_mov_R = 0;
			//DISPLAY
			  line = 6;
				Xpos = 0; Ypos = line*FONT_H;
				sprintf(lcd_buffer, "FORWARD MOVEMENT in progress");
				GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, White, Blue);
			
				break;
			case 'B': //BACKWARDS
				backwards = 1;
				value = (rx_buffer[pos+1]-'0')*10  //decenas ASCII
							+ (rx_buffer[pos+2]-'0');		//unidades ASCII
				enc_set_dist_limit((float)(value)); //distancia a recorrer
				pwm_set_duty_cycle(-Duty_cycle_L, -Duty_cycle_R);
				pos += 3;
				M_lin = 1;
				M_ang = 0;
				end_mov_L = 0;
				end_mov_R = 0;
			//DISPLAY
			  line = 6;
				Xpos = 0; Ypos = line*FONT_H;
				sprintf(lcd_buffer, "BACKWARD MOVEMENT in progress");
				GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, White, Blue);
			
				break;
			case 'L': //LEFT
				value = (rx_buffer[pos+1]-'0')*10  //decenas ASCII
							+ (rx_buffer[pos+2]-'0');		//unidades ASCII
				enc_set_angle_limit((float)(value)); //ángulo a girar
				pwm_set_duty_cycle(-Duty_cycle_L, Duty_cycle_R);
				pos += 3;
				M_lin = 0;
				M_ang = 1;
				end_mov_L = 0;
				end_mov_R = 0;
			//DISPLAY
			  line = 6;
				Xpos = 0; Ypos = line*FONT_H;
				sprintf(lcd_buffer, "LEFT TURN in progress");
				GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, White, Blue);
			
				break;
			case 'R': //RIGHT
				value = (rx_buffer[pos+1]-'0')*10  //decenas ASCII
							+ (rx_buffer[pos+2]-'0');		//unidades ASCII
				enc_set_angle_limit((float)(value)); //ángulo a girar
				pwm_set_duty_cycle(Duty_cycle_L, -Duty_cycle_R);
				pos += 3;
				M_lin = 0;
				M_ang = 1;
				end_mov_L = 0;
				end_mov_R = 0;
			//DISPLAY
			  line = 6;
				Xpos = 0; Ypos = line*FONT_H;
				sprintf(lcd_buffer, "RIGHT TURN in progress");
				GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, White, Blue);
			
				break;
			default: //FIN DE CADENA
				pwm_set_duty_cycle(0, 0);
				pos = 0;
				end_mov_L = 0;
				end_mov_R = 0;
				End_of_Chain = 1;
				break;
		}
	
		if(backwards == 1){ //SONAR ALARMA EN CASO DE MARCHA ATRÁS
			N_beep = 0.3 * N_POINTS * f_tone /  Duty_cycle_L;
			alarm_set_freq(f_tone);
			alarm_enable(1);
		}
		}
	//incrementar contador
		
		tick_counter ++;
		if(tick_counter == 20){ //actualizar dist/ángulo cada 20 ticks
			if(M_lin){ //mostrar distancia si es mov. lineal
				line = 8;
				Xpos = 0; Ypos = line*FONT_H;
				dist_lcd = (uint32_t)(enc_get_dist())/1000;
				sprintf(lcd_buffer, "DISTANCE TRAVELLED: %d cm", dist_lcd);
				GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, White, Blue);
			}else if(M_ang){ //mostrar ángulo si es mov. angular
				line = 8;
				Xpos = 0; Ypos = line*FONT_H;
				ang_lcd = (uint32_t)(enc_get_angle())/50;
				sprintf(lcd_buffer, "ANGLE TRAVELLED: %d degrees", ang_lcd);
				GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, White, Blue);
			}
			tick_counter = 0;
		}
}

void state_control(void) {
	switch(machine_state) {
		case START:
			uart0_send_string(msg_start);
			//rx_ptr = &rx_buffer[0], rx_busy = 1
			uart0_recv_string(rx_buffer);
			machine_state = PENDING;
		break;
		case PENDING:
			if(rx_busy == 0) {
				//Caso en el que se recibe fin de cadena (FDC)
				if(string_parsing() == 0) {	
					//Caso en el que la cadena es valida
					//PENDING + FDC + VÁLIDA -> PASAR A STANDBY
					uart0_send_string(msg_validation);		
					machine_state = STANDBY;
				}	else {
					machine_state = ERROR;
				}
			}
		break;
		case STANDBY:
			if(rx_err_flag == 1) {		
				//Caso en el que se ha intentado modificar la cadena de movimientos
				//Reiniciar proceso de recepción de cadena -> PASAR AL ESTADO START
				rx_err_flag = 0;				
				machine_state = START;	
			}
			else if(KEY1 == 1) {			
				//Caso en el que se ha pulsado el botón KEY1
				//KEY1 inicia el movimiento -> PASAR A MOVEMENT
				KEY1 = 0;								
				end_mov_L = 1;
				end_mov_R = 1;
				machine_state = MOVEMENT;
			}
		break;
		case MOVEMENT:
			movement_control();
		if(End_of_Chain == 1){
			End_of_Chain = 0;
			machine_state = STANDBY;
		}
		break;
		case ERROR:
			//error en la cadena
			//enviar mensaje de error y pasar de nuevo a START
			uart0_send_string(msg_error); 
			machine_state = START;
		break;
	}
}

/////////////////////////////////////////////////////////////////////////
                               //MAIN//
/////////////////////////////////////////////////////////////////////////

int main() {
	
	//CONFIGURACIONES
	pwm_config();
	enc_config(); 
	battery_sampling_init(T_SAMPLING);
	alarm_init();
	alarm_enable(0);
	uart0_cfg(BR);
	KEY1_config();
	NVIC_SetPriorityGrouping(3);
	LCD_Initialization();
  LCD_Clear(Blue);
	
	while(1){
		
		line = 4;
		Xpos = 0; Ypos = line*FONT_H;
		sprintf(lcd_buffer, "BATTERY VOLTAGE: %.2f V", battery_voltage);
		GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, White, Blue);
		
		state_control();
		}
}

//meter battery voltage display en while(1)
//Cambiar pri de ADC y end_mov_r (hito3) e (hito2)
//se ha añadido K_CALIB para calibrar los motores. sus velocoidades no eran parejas, esta K los corrige (hito 1)