/* 
   * Author : Bibek Koirala
   * Description : Implementing system self-diagnostics with memory checksums (flash memory)
*/
#include <avr/io.h> 
#include <util/delay.h>
#include <stdio.h>
#include <avr/pgmspace.h>


#include "FreeRTOS.h"
#include "task.h"


#define LED_TASK_PRIORITY               ( tskIDLE_PRIORITY + 2 )
#define CHECKSUM_TASK_PRIORITY          ( tskIDLE_PRIORITY + 1 )

extern uint16_t __data_load_end[1];                     // Defined by the linker script.  Set to address of last byte of .text+.data section

#define FLASH_END (uint16_t) & (__data_load_end[0])     // address of last used byte in flash
#define POLYNOMIAL 0xEDB88320                           // zlib's CRC32 polynomial

uint8_t initialChecksum;

static void vLedTask( void *pvParameters );
static void vChecksumTask( void *pvParameters );
volatile uint8_t calculateChecksum(void);

int main( void ){
        initialChecksum = calculateChecksum();
        xTaskCreate( vLedTask, ( const char * ) "T1", 255, (void *)('1'), LED_TASK_PRIORITY, NULL );
        xTaskCreate( vChecksumTask, ( const char * ) "T2", 255, (void *)('2'), CHECKSUM_TASK_PRIORITY, NULL );
        vTaskStartScheduler();
        return 0;
}

// simple LED blinker with 500Hz output frequency
static void vLedTask( void *pvParameters ){
        ( void ) pvParameters;
        uint8_t pinmask = (1 << 7);         // arduino pin 7 : Atmega328p PD7
        DDRD |= pinmask;
    for( ;; ) {
           PORTD |= pinmask;
	       vTaskDelay( 1 );
           PORTD &= ~pinmask; 
           vTaskDelay( 1 );
        }
}
/* 
 * Calculates CRC32 checksum over full code memory space and 
   compares the result to initial CRC result which is calculated at boot time.
 * Checksum failure puts the system in safe state.
 * Checksum calculation is periodic and has deadline of 10 minutes
*/
static void vChecksumTask( void *pvParameters ){
        ( void ) pvParameters;
        for( ;; ){
              uint8_t newChecksum = calculateChecksum();
	          if (newChecksum != initialChecksum){
                     PORTD &= ~( 1 << 7);                     // Pull down Pin 7 on port D to LOW state
                     DDRD &= ~(1 << 7);                       // Turn off OUTPUT mode for Pin 7 on port D.     
              }
              vTaskDelay( 600000 );
        }
}

// CRC32 implementation without lookup hash table
volatile uint8_t calculateChecksum(void){
	uint8_t j, mask;
	uint8_t crc = 0xFFFFFFFF;
	uint16_t memCount;
	while (memCount <= FLASH_END){
		uint8_t byte = pgm_read_byte(memCount++);
		crc = crc ^ byte;
		for (j = 7; j >= 0; j--) {
			mask = -(crc & 1);
			crc = (crc >> 1) ^ (POLYNOMIAL & mask);
		}

		memCount++;
	}
	return ~crc;
}

