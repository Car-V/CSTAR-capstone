#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// Define L298N Motor Driver Pins
#define IN1    P1_0   // Right Motor Forward
#define IN2    P1_1   // Right Motor Reverse
#define IN3    P1_2   // Left Motor Forward
#define IN4    P1_3   // Left Motor Reverse
#define ENA    P1_4   // Right Motor Enable (PWM)
#define ENB    P1_5   // Left Motor Enable (PWM)

// Function Prototypes
void motor_init();
void set_motor_speed(uint8_t motor, uint8_t speed);
void move_forward();
void move_backward();
void stop_motors();

#endif // MOTOR_CONTROL_H
