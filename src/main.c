#include "stm32f4xx.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define ACCEL_CONFIG 16384.0
#define GYRO_CONFIG 131.0
#define ALPHA 0.96

/*
PINS:
RST to GND for UART
PA3 -> arduino rx
PA2 -> arduino tx

PB6 -> I2C1_SCL
PB7 -> I2C1_SDA

PE7 -> driver STBY

PA7 -> driver PWMA  TIM3 channel 2
PE8 -> driver AIN1
PE9 -> driver AIN2

PA6 -> driver PWMB  TIM3 channel 1
PE10 -> driver BIN1
PE11 -> driver BIN2 

motorun küçük metalinin 2 ucunu da ayrı birer kabloyla lehimleyip 
drivera bağlicaz birini x0 öbürü x1(x = AO | BO)

*/
// Free-running millisecond counter, incremented by the SysTick interrupt.
// This is what millis() reads -- it's the whole reason SysTick exists here.
volatile uint32_t ms_tick = 0;

void SysTick_Handler(void){
    ms_tick += 1;
}

// TB6612 direction-pin helpers. Every one of these first drives STBY (PE7)
// high to make sure the driver isn't sitting in standby, then sets the two
// direction pins for that channel according to the TB6612 truth table:
//   AIN1=H, AIN2=L (or BIN1=H, BIN2=L) -> spin one way
//   AIN1=L, AIN2=H (or BIN1=L, BIN2=H) -> spin the other way
//   AIN1=H, AIN2=H (or BIN1=H, BIN2=H) -> short brake
//   AIN1=L, AIN2=L (or BIN1=L, BIN2=L) -> stop / coast (Hi-Z)
// BSRR is written twice per call: bits [15:0] SET the corresponding output
// pin high, bits [31:16] RESET it low -- so each call here is really two
// separate one-shot writes (STBY high, then the direction pair), not one
// combined write.

// Short brake, channel A (both direction pins high).
void short_brake_A(){
    GPIOE->BSRR = (1<<7);
    GPIOE->BSRR = ((1 << 8) | (1 << 9));
}
// Short brake, channel B (both direction pins high).
void short_brake_B(){
    GPIOE->BSRR = (1<<7);
    GPIOE->BSRR = ((1 << 10) | (1 << 11));
}

// Counter-clockwise, channel A: AIN1 low, AIN2 high, PWM duty on TIM3 CH2
// (CCR2) sets the speed. "Counter-clockwise" is just a label -- which way
// the wheel actually spins depends on how the motor leads are wired.
void CCW_A(uint32_t speed){
    GPIOE->BSRR = (1<<7);
    GPIOE->BSRR = ((1 << (8 + 16)) | (1 << 9));
    TIM3->CCR2 = speed;
}

// Counter-clockwise, channel B: BIN1 low, BIN2 high, PWM duty on TIM3 CH1
// (CCR1).
void CCW_B(uint32_t speed){
    GPIOE->BSRR = (1<<7);
    GPIOE->BSRR = ((1 << (10 + 16)) | (1 << 11));
    TIM3->CCR1 = speed;
}


// Clockwise, channel A: AIN1 high, AIN2 low -- the opposite direction
// pattern from CCW_A, same PWM channel (CCR2).
void CW_A(uint32_t speed){
    GPIOE->BSRR = (1<<7);
    GPIOE->BSRR = ((1 << 8) | (1 << (9 + 16)));
    TIM3->CCR2 = speed;
}

// Clockwise, channel B: BIN1 high, BIN2 low, PWM duty on TIM3 CH1 (CCR1).
void CW_B(uint32_t speed){
    GPIOE->BSRR = (1<<7);
    GPIOE->BSRR = ((1 << 10) | (1 << (11 + 16)));
    TIM3->CCR1 = speed;
}

// Stop/coast, channel A: both direction pins low, so the driver is in
// Hi-Z regardless of what the PWM line is doing -- the CCR2=100 here
// doesn't really matter for the same reason.
void stop_A(){
    GPIOE->BSRR = (1<<7);
    GPIOE->BSRR = ((1 << 8 + 16) | (1 << 9 + 16));
    TIM3->CCR2 = 100;
}

// Stop/coast, channel B: same idea as stop_A, other channel.
void stop_B(){
    GPIOE->BSRR = (1<<7);
    GPIOE->BSRR = ((1 << 10 + 16) | (1 << 11 + 16));
    TIM3->CCR1 = 100;
}

// Returns milliseconds since boot -- just a thin wrapper around ms_tick.
uint32_t millis(void){
    return ms_tick;
}

// This function overrides the default C library _write
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        
        // 1. We must wait until the USART is ready to accept a new character.
        // We do this by checking the TXE (Transmit Data Register Empty) flag.
        // TXE is bit 7 inside the USART2->SR (Status Register).
        // If the bit is 0, it means it's busy. If it's 1, it's ready.
        while (!(USART2->SR & (1 << 7))) {
            // Just wait here in an empty loop until the hardware is ready
        }
        
        // 2. The hardware is ready! 
        // Shove the current character into the DR (Data Register) to fire it over the wire.
        USART2->DR = ptr[i];
    }
    return len; // Tell printf we successfully sent all the characters
}


uint8_t _whoami(){
    // --- PHASE 1: write the target register address (WHO_AM_I, 0x75) to the MPU ---

    // Generate a START condition on the bus. This also arbitrates for bus
    // ownership if something else were using it.
    I2C1->CR1 |= (1<<8);
    // SB (Start Bit) in SR1 is set once the start condition has actually
    // gone out on the wire. Block here until the hardware confirms it.
    while(!(I2C1->SR1 & (1<<0))){
        // has start condition triggered?
    }

    // Send the 7-bit slave address (0x68) shifted left by 1, with the R/W
    // bit (bit 0) left as 0 -> this is a WRITE to the MPU.
    // Writing to DR here is what actually clears SB (per the "read SR1,
    // then write DR" sequence required to clear the SB event).
    I2C1->DR = 0x68 << 1;

    // ADDR (bit 1 of SR1) is set once the slave has ACKed its address.
    // If the slave never acks (wrong address, no pull-ups, not powered),
    // this loops forever -- that was the earlier bring-up bug.
    while(!(I2C1->SR1 & (1<<1))){

    }
    // ADDR must be cleared by reading SR1 then SR2, in that order.
    // The values aren't needed here, just the act of reading them.
    uint32_t dummy = I2C1->SR1;
    dummy = I2C1->SR2;

    // Now that addressing succeeded, send the actual register pointer
    // we want to read from: WHO_AM_I lives at 0x75.
    I2C1->DR = 0x75;

    // TXE (bit 7) means DR is empty again, i.e. the register-address byte
    // has moved out into the shift register and is on its way out.
    while(!(I2C1->SR1 & (1<<7))){

    }

    // --- PHASE 2: repeated START, then read one byte back ---

    // Repeated START: switch direction without releasing the bus first.
    // This tells the MPU "same transaction, but now I want to read."
    I2C1->CR1 |= (1<<8);
    while(!(I2C1->SR1 & (1<<0))){

    }

    // Same address as before, but this time R/W (bit 0) = 1 -> READ.
    I2C1->DR = (0x68 << 1) | 1;
    while(!(I2C1->SR1 & (1<<1))){

    }

    // Single-byte-read procedure (RM0090): ACK must be disabled and STOP
    // must be programmed *before* ADDR is cleared, otherwise the hardware
    // will automatically ACK the next byte and keep the transaction going
    // instead of ending it after just one byte.
    I2C1->CR1 &= ~(1<<10);   // disable ACK -> next received byte gets NACKed
    dummy = I2C1->SR1;
    dummy = I2C1->SR2;       // clearing ADDR now (after ACK is already off)
    I2C1->CR1 |= (1<<9);     // schedule STOP right after this one byte

    // RxNE (bit 6) means a received byte is sitting in DR, ready to read.
    while(!(I2C1->SR1 & (1<<6))){

    }
    uint8_t data = I2C1->DR;

    return data;
}


// Wakes the MPU up by clearing the SLEEP bit in PWR_MGMT_1 (0x6B). The
// sensor boots into sleep mode and won't update its data registers at all
// until this runs -- call this once, at startup, not in the main loop.
// This is a plain register WRITE (write pointer, then write data), so
// unlike _whoami()/mbb_read() there's no repeated start here: we never
// change direction, so one address+W at the start covers the whole thing.
void MPU6050_wake_up(){
    // Generate a START condition on the bus.
    I2C1->CR1 |= (1<<8);
    // SB (Start Bit) in SR1 is set once the start condition has actually
    // gone out on the wire. Block here until the hardware confirms it.
    while(!(I2C1->SR1 & (1<<0))){
        // has start condition triggered?
    }

    // Send the 7-bit slave address (0x68) shifted left by 1, with the R/W
    // bit (bit 0) left as 0 -> this is a WRITE to the MPU.
    // Writing to DR here is what actually clears SB (per the "read SR1,
    // then write DR" sequence required to clear the SB event).
    I2C1->DR = 0x68 << 1;

    // ADDR (bit 1 of SR1) is set once the slave has ACKed its address.
    while(!(I2C1->SR1 & (1<<1))){

    }
    // ADDR must be cleared by reading SR1 then SR2, in that order.
    uint32_t dummy = I2C1->SR1;
    dummy = I2C1->SR2;

    // Register pointer: PWR_MGMT_1 lives at 0x6B.
    I2C1->DR = 0x6B;

    // TXE (bit 7) means DR is empty again, i.e. the register-address byte
    // has moved out into the shift register and is on its way out.
    while(!(I2C1->SR1 & (1<<7))){

    }

    // Data byte: 0x00 clears SLEEP (bit 6) along with everything else in
    // this register, defaulting to the internal oscillator as clock source.
    I2C1->DR = 0x00;

    while(!(I2C1->SR1 & (1<<7))){

    }

    // Done writing -- release the bus.
    I2C1->CR1 |= (1 << 9);
}

// Multi-byte burst read: grabs all 14 consecutive data bytes starting at
// ACCEL_XOUT_H (0x3B) -- accel X/Y/Z (2 bytes each), temperature (2 bytes),
// then gyro X/Y/Z (2 bytes each) -- into the caller-provided buffer.
// buffer must point to at least 14 bytes; the caller owns that memory
// (this function used to return a pointer to a local array here, which is
// a dangling-pointer bug -- local/stack memory doesn't survive the
// function returning, so the caller passing its own buffer in is the fix).
void mbb_read(uint8_t* buffer){
    // --- PHASE 1: write the target register pointer (0x3B) to the MPU ---

    // Generate a START condition on the bus.
    I2C1->CR1 |= (1<<8);
    // SB (Start Bit) in SR1 is set once the start condition has actually
    // gone out on the wire. Block here until the hardware confirms it.
    while(!(I2C1->SR1 & (1<<0))){
        // has start condition triggered?
    }

    // Send the 7-bit slave address (0x68) shifted left by 1, with the R/W
    // bit (bit 0) left as 0 -> this is a WRITE to the MPU.
    // Writing to DR here is what actually clears SB (per the "read SR1,
    // then write DR" sequence required to clear the SB event).
    I2C1->DR = 0x68 << 1;

    // ADDR (bit 1 of SR1) is set once the slave has ACKed its address.
    while(!(I2C1->SR1 & (1<<1))){

    }
    // ADDR must be cleared by reading SR1 then SR2, in that order.
    uint32_t dummy = I2C1->SR1;
    dummy = I2C1->SR2;

    // Register pointer: start the burst at ACCEL_XOUT_H, 0x3B. The MPU's
    // own internal pointer will auto-increment through all 14 bytes on
    // its own from here -- we never send another register address below.
    I2C1->DR = 0x3B;

    // TXE (bit 7) means DR is empty again, i.e. the register-address byte
    // has moved out into the shift register and is on its way out.
    while(!(I2C1->SR1 & (1<<7))){

    }

    // --- PHASE 2: repeated START, switch to read, pull 14 bytes back ---

    // Repeated START: switch direction without releasing the bus first.
    // This tells the MPU "same transaction, but now I want to read."
    I2C1->CR1 |= (1<<8);
    while(!(I2C1->SR1 & (1<<0))){}

    // Same device address as before, but R/W (bit 0) = 1 -> READ.
    I2C1->DR = (0x68<<1) | 1;
    // ADDR (bit 1), not TXE -- this is an address byte, not data.
    while(!(I2C1->SR1 & (1<<1))){}
    dummy = I2C1->SR1;
    dummy = I2C1->SR2;

    // ACK must be enabled here (its reset default, and its state after
    // _whoami()'s single-byte NACK, is *disabled*) so that bytes 0-12 all
    // get ACKed and the MPU keeps sending. Only the very last byte (i==13)
    // gets NACKed, below.
    I2C1->CR1 |= (1<<10);
    for(int i = 0; i<14; i++){

        // On the last byte: disable ACK and schedule STOP *before* this
        // byte is read out. ACK/STOP govern whichever byte is currently
        // being clocked in, not one that's already sitting in DR -- doing
        // this after the read would be one byte too late, and the MPU
        // would keep sending past byte 14.
        if(i == 13){
            I2C1->CR1 &= ~(1<<10);
            dummy = I2C1->SR1;
            dummy = I2C1->SR2;
            I2C1->CR1 |= (1<<9);     // schedule STOP right after this one byte
        }
        // RxNE (bit 6): a received byte is sitting in DR, ready to read.
        while(!(I2C1->SR1 & (1<<6))){

        }
        buffer[i] = I2C1->DR;
    }
}

void control(float* data, uint32_t* last_print_time, float* filtered_angle, float* running_sum, float* output){
    float accel_angle = atan2(data[0], data[2]) * (180.0 / 3.14159265);
    uint32_t now = millis();

    float dt = (now - *last_print_time) / 1000; //convert it to seconds
    
    *last_print_time = now;
    
    float gyro_rate = data[5];
    *filtered_angle = ALPHA * (*filtered_angle + gyro_rate * dt) + (1 - ALPHA) * accel_angle;

    float Kp=0,Ki=0,Kd=0;
    *output = Kp*accel_angle +  Ki * (*running_sum) + Kd * gyro_rate;
}
// CLOCK 168MHz
int main(void){

    // 1. Turn on HSE (High-Speed External 8MHz crystal) and wait for it to stabilize
    RCC->CR |= (1 << 16);
    while(!(RCC->CR & (1 << 17))) { }

    // 2. Configure PLL Gears: M=8, N=336, P=2, Q=7, Source=HSE
    // This scales the 8MHz crystal up to 168MHz for the main SYSCLK
    RCC->PLLCFGR = ((7 << 24) | (1 << 22) | (0 << 16) | (336 << 6) | (8 << 0));

    // 3. Turn on the PLL and wait for it to stabilize
    RCC->CR |= (1 << 24);
    while(!(RCC->CR & (1 << 25))){ }
    
    // 4. Configure Flash Wait States and Caches for 168MHz stability
    FLASH->ACR = (1 << 8) | (1 << 9) | (1 << 10) | 5;
    
    // 5. Set Prescalers for Peripheral Buses
    // APB1 (PPRE1) = Divide by 4 (42MHz, Timers get 84MHz)
    // APB2 (PPRE2) = Divide by 2 (84MHz, Timers get 168MHz)
    RCC->CFGR |= ((0 << 4) | (5 << 10) | (4 << 13));

    
    // 6. Switch main SYSCLK to use the PLL output
    RCC->CFGR |= (2 << 0);
    while((RCC->CFGR & (3 << 2)) != (2 << 2)) { }

    // --- ENABLE PERIPHERAL CLOCKS (Powering the hardware blocks) ---
    // AHB1: Enable GPIOA (bit 0) and GPIOE (bit 4) and GPIOB -> (1 << 0 | 1 << 4 | 1 << 1) = 19
    RCC->AHB1ENR |= (19 << 0);
    // APB1: Enable USART2 (bit 17) and TIM3 (bit 1)
    RCC->APB1ENR |= (1 << 17); 
    RCC->APB1ENR |= (1 << 1);
    // enable I2C1
    RCC->APB1ENR |= (1 << 21);
     

    // Set TIM3 PSC to 83 in order to have 10kHz in TIM3
    TIM3->PSC = 83;
    // Set TIM3 Auto reload register to 99
    TIM3->ARR = 99;
    // Set capture compare mode to PWM(110) for channel 1 and 2
    TIM3->CCMR1 |= (6 << 4);
    TIM3->CCMR1 |= (6 << 12);
    // enable capture compare for channel 1 and 2
    TIM3->CCER |= (1 << 0);
    TIM3->CCER |= (1 << 4);
    // start TIM3 counter   
    TIM3->CR1 |= (1 << 0); 

    // SYSTICK TIMER INTERRUPT
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    SysTick->LOAD |= (167999 << 0);
    SysTick->CTRL |= (7 << 0);  
    

    
    // --- CONFIGURE UART PINS (PA2 = TX, PA3 = RX) ---
    // Set PA2 and PA3 to Alternate Function (10) mode
    GPIOA->MODER &= ~((3 << 4) | (3 << 6));
    GPIOA->MODER |= ((2 << 4) | (2 << 6));
    // "" PB6 and PB7 for I2C
    GPIOB->MODER &= ~((3 << 12) | (3 << 14));
    GPIOB->MODER |= ((2 << 12) | (2 << 14));
    GPIOB->PUPDR &= ~((3 << 12) | (3 << 14));
    GPIOB->PUPDR |= ((1 << 12) | (1 << 14));
    // Connect PA2 and PA3 to AF7 (USART2)
    // NOTE: A true full reset requires clearing 4 bits (15), not 3 bits (7)
    GPIOA->AFR[0] &= ~((15 << 8) | (15 << 12)); 
    GPIOA->AFR[0] |= ((7 << 8) | (7 << 12)); 

    // Set AFRegister for PB6 and PB7 to I2C_SCL and I2C_SDA
    GPIOB->AFR[0] &= ~((15 << 24) | (15 << 28)); 
    GPIOB->AFR[0] |= ((4 << 24) | (4 << 28)); 
    // Set PB6 and PB7 to open-drain mode
    GPIOB->OTYPER |= (3 << 6);

    // I2C1 timing, all derived from APB1 = 42MHz for standard-mode (100kHz):
    // CR2.FREQ tells the peripheral what its own input clock actually is.
    I2C1->CR2 |= (42 << 0);
    // CCR sets Thigh=Tlow=CCR*TPCLK1; for 100kHz that's CCR=210.
    I2C1->CCR &= ~(2047 << 0);
    I2C1->CCR |= (210 << 0);
    // TRISE = max rise time (1000ns for standard mode) / TPCLK1, plus 1.
    I2C1->TRISE = (43 << 0);
    // Peripheral enable -- must come after CR2/CCR/TRISE, since those are
    // locked once PE is set.
    I2C1->CR1 |= (1 << 0);

    USART2->BRR = (22 << 4) | 13;
    USART2->CR1 |= (1 << 2);
    USART2->CR1 |= (1 << 3);
    USART2->CR1 |= (1 << 13); 
    // USART2->CR1 |= (1 << 2) | (1 << 3) | (1 << 13); this also works

    // --- CONFIGURE MOTOR DRIVER GPIO ---

    // 1. PWM Pins (PA6 = PWMB, PA7 = PWMA)
    // Set PA6 and PA7 to Alternate Function (10) mode
    GPIOA->MODER &= ~(3 << 12);
    GPIOA->MODER |= (2 << 12); 
    GPIOA->MODER &= ~(3 << 14);
    GPIOA->MODER |= (2 << 14); 
    
    // Connect PA6 to TIM3_CH1 (AF2) and PA7 to TIM3_CH2 (AF2)
    GPIOA->AFR[0] &= ~(15 << 24); 
    GPIOA->AFR[0] |= (2 << 24); 
    GPIOA->AFR[0] &= ~(15 << 28);
    GPIOA->AFR[0] |= (2 << 28); 

    // 2. Direction & Standby Pins (PE7, PE8, PE9, PE10, PE11)
    // Set all to General Purpose Output (01) mode
    GPIOE->MODER &= ~(3 << 14); // PE7 (STBY)
    GPIOE->MODER |= (1 << 14); 
    GPIOE->MODER &= ~(3 << 16); // PE8 (AIN1)
    GPIOE->MODER |= (1 << 16); 
    GPIOE->MODER &= ~(3 << 18); // PE9 (AIN2)
    GPIOE->MODER |= (1 << 18); 
    GPIOE->MODER &= ~(3 << 20); // PE10 (BIN1)
    GPIOE->MODER |= (1 << 20); 
    GPIOE->MODER &= ~(3 << 22); // PE11 (BIN2)
    GPIOE->MODER |= (1 << 22);

    // MAIN LOOP
    uint32_t last_print_time = 0;
    float filtered_angle = 0;
    uint32_t speed = 50;
    uint8_t buffer[14];   // holds one 14-byte burst read: accel(6) + temp(2) + gyro(6)
    float data[7];
    //float* output = 0;
    // One-time sensor init -- must run once before the loop, not inside it.

    //MPU6050_wake_up();
    

    while(1){
        // Runs once per second (non-blocking -- everything else could
        // still run between checks, this just gates how often we print).
        if (millis() - last_print_time >= 500) {
            
            
            
            speed = 60;
            CW_B(speed);
            //CW_A(speed);

            /*mbb_read(buffer);
            
            for(uint8_t i = 0; i<14; i++){
                printf("buffer[%d]: %d\n",i,buffer[i]);   
            }
            
            for(uint8_t i = 0; i<7; i++){
                int16_t raw = (buffer[i*2] << 8) | buffer[i*2 + 1];
                data[i] = (float)raw;
            }
            data[0] = data[0] / ACCEL_CONFIG;
            data[1] = data[1] / ACCEL_CONFIG;
            data[2] = data[2] / ACCEL_CONFIG;
            data[4] = data[4] / GYRO_CONFIG;
            data[5] = data[5] / GYRO_CONFIG;
            data[6] = data[6] / GYRO_CONFIG;
            
            for(uint8_t i = 0; i<7; i++){
                printf("data[%d]: %f\n",i,data[i]);   
            }
            //control(last_print_time,data,&filtered_angle,output);
            
           */  
            ms_tick = 0;
        
            
        }
    }
}

// data[2]
// data[4]