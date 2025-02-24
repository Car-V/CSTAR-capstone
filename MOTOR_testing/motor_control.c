/* ========================================
 * Neha & Rania's Motor Test Code
 * Dual Motor Control on the L298N Motor Driver
 * ========================================
 */

#include "motor_control.h"

void motor_init() {
    // Configure GPIO pins as outputs
    Cy_GPIO_Write(IN1, 0);
    Cy_GPIO_Write(IN2, 0);
    Cy_GPIO_Write(IN3, 0);
    Cy_GPIO_Write(IN4, 0);
}

void set_motor_speed(uint8_t motor, uint8_t speed) {
    if (motor == 0) {
        PWM_WriteCompare(ENA, speed);
    } else if (motor == 1) {
        PWM_WriteCompare(ENB, speed);
    }
}

void move_forward() {
    Cy_GPIO_Write(IN1, 1);
    Cy_GPIO_Write(IN2, 0);
    Cy_GPIO_Write(IN3, 1);
    Cy_GPIO_Write(IN4, 0);
}

void move_backward() {
    Cy_GPIO_Write(IN1, 0);
    Cy_GPIO_Write(IN2, 1);
    Cy_GPIO_Write(IN3, 0);
    Cy_GPIO_Write(IN4, 1);
}

void stop_motors() {
    Cy_GPIO_Write(IN1, 0);
    Cy_GPIO_Write(IN2, 0);
    Cy_GPIO_Write(IN3, 0);
    Cy_GPIO_Write(IN4, 0);
}

int main(void) {
    CyGlobalIntEnable;
    motor_init();
    PWM_Start();
    
    set_motor_speed(0, 128); // 50% Speed Right Motor
    set_motor_speed(1, 128); // 50% Speed Left Motor
    move_forward();
    
    CyDelay(5000); // Move for 5 seconds
    stop_motors();
    
    while (1) {
        // Infinite loop
    }
}
