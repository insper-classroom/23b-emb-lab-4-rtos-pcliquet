#include "conf_board.h"
#include <asf.h>

/************************************************************************/
/* BOARD CONFIG                                                         */
/************************************************************************/
#define BUT_PIO_1		PIOD
#define BUT_PIO_ID_1		ID_PIOD
#define BUT_PIO_IDX_1		28
#define BUT_PIO_IDX_MASK_1	(1u<< BUT_PIO_IDX_1)

#define BUT_PIO_2		PIOC
#define BUT_PIO_ID_2		ID_PIOC
#define BUT_PIO_IDX_2		31
#define BUT_PIO_IDX_MASK_2	(1u<< BUT_PIO_IDX_2)

#define BUT_PIO_3		PIOA
#define BUT_PIO_ID_3		ID_PIOA
#define BUT_PIO_IDX_3		19
#define BUT_PIO_IDX_MASK_3	(1u<< BUT_PIO_IDX_3)

#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_PIO_IDX 8
#define LED_IDX_MASK (1 << LED_PIO_IDX)

#define USART_COM_ID ID_USART1
#define USART_COM USART1

/************************************************************************/
/* RTOS                                                                */
/************************************************************************/

#define TASK_LED_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_BUT_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_BUT_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

/** Semaforo a ser usado pela task led */
SemaphoreHandle_t xSemaphoreBut1;
SemaphoreHandle_t xSemaphoreBut2;
SemaphoreHandle_t xSemaphoreBut3;

/** Queue for msg log send data */
QueueHandle_t xQueueLedFreq;

/************************************************************************/
/* prototypes local                                                     */
/************************************************************************/

void but_callback1(void);
void but_callback2(void);
void but_callback3(void);
static void BUT_init(void);
void pin_toggle(Pio *pio, uint32_t mask);
static void USART1_init(void);
void LED_init(int estado);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName) {
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  /* If the parameters have been corrupted then inspect pxCurrentTCB to
   * identify which task has overflowed its stack.
   */
  for (;;) {
  }
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void) { pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); }

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

  /* Force an assert. */
  configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback1(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xSemaphoreBut1, &xHigherPriorityTaskWoken);
}

void but_callback2(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreBut2, &xHigherPriorityTaskWoken);
}

void but_callback3(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreBut3, &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_led(void *pvParameters) {

  LED_init(1);

  uint32_t msg = 0;
  uint32_t delayMs = 2000;

  /* tarefas de um RTOS não devem retornar */
  for (;;) {
    /* verifica se chegou algum dado na queue, e espera por 0 ticks */
    if (xQueueReceive(xQueueLedFreq, &msg, (TickType_t) 0)) {
      /* chegou novo valor, atualiza delay ! */
      /* aqui eu poderia verificar se msg faz sentido (se esta no range certo)
       */
      /* converte ms -> ticks */
      delayMs = msg / portTICK_PERIOD_MS;
      printf("task_led: %d \n", delayMs);
    }

    /* pisca LED */
    pin_toggle(LED_PIO, LED_IDX_MASK);

    /* suspende por delayMs */
    vTaskDelay(delayMs);
  }
}

static void task_but(void *pvParameters) {

  /* iniciliza botao */
  BUT_init();

  uint32_t delayTicks = 2000;

  for (;;) {
    /* aguarda por tempo inderteminado até a liberacao do semaforo */
    if (xSemaphoreTake(xSemaphoreBut1, 1000)) {
      /* atualiza frequencia */
      delayTicks -= 10;

      /* envia nova frequencia para a task_led */
      xQueueSend(xQueueLedFreq, (void *)&delayTicks, 10);
	    
      printf("task_but: %d \n", delayTicks);

      /* garante range da freq. */
      if (delayTicks == 100) {
        delayTicks = 900;
      }
	}
	if (xSemaphoreTake(xSemaphoreBut2, 1000)) {
		/* atualiza frequencia */
		delayTicks -= 5;

		/* envia nova frequencia para a task_led */
		xQueueSend(xQueueLedFreq, (void *)&delayTicks, 10);
		  
		printf("task_but: %d \n", delayTicks);

		/* garante range da freq. */
		if (delayTicks == 100) {
			delayTicks = 900;
		}
	}
		  
	if (xSemaphoreTake(xSemaphoreBut3, 1000)) {
		/* atualiza frequencia */
		delayTicks -= 1;

		/* envia nova frequencia para a task_led */
		xQueueSend(xQueueLedFreq, (void *)&delayTicks, 10);
			  
		printf("task_but: %d \n", delayTicks);

		/* garante range da freq. */
		if (delayTicks == 100) {
			delayTicks = 900;
		}
    }
  }
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

/**
 * \brief Configure the console UART.
 */
static void configure_console(void) {
  const usart_serial_options_t uart_serial_options = {
      .baudrate = CONF_UART_BAUDRATE,
      .charlength = CONF_UART_CHAR_LENGTH,
      .paritytype = CONF_UART_PARITY,
      .stopbits = CONF_UART_STOP_BITS,
  };

  /* Configure console UART. */
  stdio_serial_init(CONF_UART, &uart_serial_options);

  /* Specify that stdout should not be buffered. */
  setbuf(stdout, NULL);
}

// Faz um chaveamento de um possivel periférico
void pin_toggle(Pio *pio, uint32_t mask) {
  if (pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
  else
    pio_set(pio, mask);
}

// Inicia o led
void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_IDX_MASK, estado, 0, 0);
};



// inicia os botões e configura os handlers de acordo com 1 botão
static void BUT_init(void) {
  // Configura PIO para lidar com o pino do botão como entrada
  // com pull-up
  pio_configure(BUT_PIO_1, PIO_INPUT, BUT_PIO_IDX_MASK_1, PIO_PULLUP);
  pio_configure(BUT_PIO_2, PIO_INPUT, BUT_PIO_IDX_MASK_2, PIO_PULLUP);
  pio_configure(BUT_PIO_3, PIO_INPUT, BUT_PIO_IDX_MASK_3, PIO_PULLUP);
  

  // Configura interrupção no pino referente ao botao e associa
  // função de callback caso uma interrupção for gerada
  // a função de callback é a: but_callback()


  pio_handler_set(BUT_PIO_1,
                  BUT_PIO_ID_1,
                  BUT_PIO_IDX_MASK_1,
                  PIO_IT_FALL_EDGE,
                  but_callback1);


  pio_handler_set(BUT_PIO_2,
				  BUT_PIO_ID_2,
				  BUT_PIO_IDX_MASK_2,
				  PIO_IT_FALL_EDGE,
				  but_callback2);


  pio_handler_set(BUT_PIO_3,
				  BUT_PIO_ID_3,
			      BUT_PIO_IDX_MASK_3,
				  PIO_IT_FALL_EDGE,
				  but_callback3);
				  
				  
				  
				  
  // Ativa interrupção e limpa primeira IRQ gerada na ativacao
  pio_enable_interrupt(BUT_PIO_1, BUT_PIO_IDX_MASK_1);
  pio_get_interrupt_status(BUT_PIO_1);
  
  pio_enable_interrupt(BUT_PIO_2, BUT_PIO_IDX_MASK_2);
  pio_get_interrupt_status(BUT_PIO_2);
  
  pio_enable_interrupt(BUT_PIO_3, BUT_PIO_IDX_MASK_3);
  pio_get_interrupt_status(BUT_PIO_3);
  
  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais próximo de 0 maior)
  NVIC_EnableIRQ(BUT_PIO_ID_1);
  NVIC_SetPriority(BUT_PIO_ID_1, 4); // Prioridade 4
//   
	NVIC_EnableIRQ(BUT_PIO_ID_2);
	NVIC_SetPriority(BUT_PIO_ID_2, 4);

	NVIC_EnableIRQ(BUT_PIO_ID_3);
	NVIC_SetPriority(BUT_PIO_ID_3,4);

}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void) {
  /* Initialize the SAM system */
  sysclk_init();
  board_init();
  configure_console();
	
  printf("Sys init ok \n");

  /* Attempt to create a semaphore. */
  xSemaphoreBut1 = xSemaphoreCreateBinary();
  if (xSemaphoreBut1 == NULL){
    printf("falha em criar o semaforo \n");
	}
	xSemaphoreBut2 = xSemaphoreCreateBinary();
	if(xSemaphoreBut2 == NULL){
		printf("nao rolou criar o Semaforo 2");
	}
	
	xSemaphoreBut3 = xSemaphoreCreateBinary();
	if(xSemaphoreBut3 == NULL){
		printf("Boa tentativa, mas o semaforo 3 nao foi gerado");
	}

  /* cria queue com 32 "espacos" */
  /* cada espaço possui o tamanho de um inteiro*/
  xQueueLedFreq = xQueueCreate(32, sizeof(uint32_t));
  if (xQueueLedFreq == NULL){
    printf("falha em criar a queue \n");
  }
  /* Create task to make led blink */
  if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
                  TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test led task\r\n");
  } else {
     printf("task led created \r\n");
	  
  }
  /* Create task to monitor processor activity */
  if (xTaskCreate(task_but, "BUT", TASK_BUT_STACK_SIZE, NULL,
                  TASK_BUT_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create UartTx task\r\n");
  } else {
     printf("task led but \r\n");  
  }

  /* Start the scheduler. */
  vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
  while (1) {
  }

  /* Will only get here if there was insufficient memory to create the idle
   * task. */
  return 0;
}
				  
