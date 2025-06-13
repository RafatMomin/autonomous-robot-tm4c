#include "lcd.h"
#include <math.h>
#include "servo.h"
#include "ping.h"
#include "adc.h"
#include "open_interface.h"
#include "Timer.h"
#include "uart_extra_help.h"
#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

extern volatile enum {LOW, HIGH, DONE} state;
extern volatile unsigned int rising_time;
extern volatile unsigned int falling_time;
extern volatile char uart_data;
extern volatile char flag;
extern volatile int deg;
extern volatile int direction;
extern volatile float pcv;
extern volatile int right_calibration_val;
extern volatile int left_calibration_val;

typedef struct {
    short id;
    short start_angle;
    short end_angle;
    short center_angle;
    double ping_dist;
    short ir_dist;
    double width;
    char label;
} Object;

void rotate_ccw(oi_t *sensor, short deg);
void rotate_cw(oi_t *sensor, short deg);
void retreat(oi_t *sensor);
void advance(oi_t *sensor, float mm);
int scan_surroundings();

int main(void) {
    oi_t *roomba = oi_alloc();
    oi_init(roomba);

    timer_init();
    lcd_init();
    adc_init();
    uart_init(115200);
    uart_interrupt_init();
    servo_init();
    ping_init();

    while (1) {
        oi_update(roomba);

        // Cliff sensor detection
        if (roomba->cliffFrontLeftSignal < 20 || roomba->cliffFrontRightSignal < 20 ||
            roomba->cliffLeftSignal < 20 || roomba->cliffRightSignal < 20) {

            uart_sendStr("\n\rWARNING: Detected a drop-off!");
            oi_setWheels(0, 0);
            retreat(roomba);

            if (roomba->cliffLeftSignal < roomba->cliffRightSignal) {
                rotate_cw(roomba, 90);
            } else {
                rotate_ccw(roomba, 90);
            }
        }

        // Boundary detection
        if (roomba->cliffFrontLeftSignal > 2700 || roomba->cliffFrontRightSignal > 2600) {
            uart_sendStr("\n\rWARNING: Boundary hit!");
            retreat(roomba);

            if (roomba->cliffLeftSignal < roomba->cliffRightSignal) {
                rotate_cw(roomba, 90);
            } else {
                rotate_ccw(roomba, 90);
            }
        }

        // Collision detection
        if (roomba->bumpLeft) {
            uart_sendStr("\n\rCollision: Object on left side");
            retreat(roomba);
            rotate_cw(roomba, 80);
        } else if (roomba->bumpRight) {
            uart_sendStr("\n\rCollision: Object on right side");
            retreat(roomba);
            rotate_ccw(roomba, 80);
        }

        // Manual control
        if (flag) {
            switch (uart_data) {
                case 'w':
                    oi_setWheels(100, 100);
                    break;
                case 's':
                    oi_setWheels(-100, -100);
                    break;
                case 'a':
                    oi_setWheels(50, -50);
                    break;
                case 'd':
                    oi_setWheels(-50, 50);
                    break;
                case 'm':
                    oi_setWheels(0, 0);
                    {
                        int human_found = scan_surroundings();
                        if (human_found == 1) {
                            lcd_printf("Survivor!");
                            uart_sendStr("\n\rDetected a survivor. Alert responders!");
                        } else if (human_found == 2) {
                            lcd_printf("Responders!");
                            uart_sendStr("\n\rLocated response team.");
                        }
                    }
                    break;
                default:
                    oi_setWheels(0, 0);
                    break;
            }
            flag = 0;
        }
    }
    return 0;
}

void rotate_cw(oi_t *sensor, short degrees) {
    double rotated = 0;
    oi_setWheels(-50, 50);
    while ((short)rotated > -degrees) {
        oi_update(sensor);
        rotated += sensor->angle;
    }
    oi_setWheels(0, 0);
}

void rotate_ccw(oi_t *sensor, short degrees) {
    double rotated = 0;
    oi_setWheels(50, -50);
    while ((short)rotated < degrees) {
        oi_update(sensor);
        rotated += sensor->angle;
    }
    oi_setWheels(0, 0);
}

void advance(oi_t *sensor, float mm) {
    double moved = 0;
    oi_setWheels(150, 150);
    while (moved < mm) {
        oi_update(sensor);
        moved += sensor->distance;
    }
    oi_setWheels(0, 0);
}

void retreat(oi_t *sensor) {
    double moved = 0;
    oi_setWheels(-100, -100);
    while (moved > -75) {
        oi_update(sensor);
        moved += sensor->distance;
    }
    oi_setWheels(0, 0);
}

int scan_surroundings() {
    Object scanned[10];
    int total_objects = 0;
    int previous = 0, angle_prev = 0, current = 0;
    int found = 0, humans = 0;
    char buffer[100];

    uart_sendStr("\n\rAngle\tDistance (cm)\n\r");

    for (int angle = 0; angle <= 180; angle += 2) {
        servo_move(angle);
        if (angle == 0) timer_waitMillis(500);

        current = 654371 * pow(adc_read(), -1.420);

        if (current < 70 && previous > 70) {
            scanned[total_objects].id = total_objects + 1;
            scanned[total_objects].start_angle = angle;
            scanned[total_objects].ir_dist = current;
            total_objects++;
        } else if (previous < 70) {
            scanned[total_objects - 1].end_angle = angle_prev;
        }

        sprintf(buffer, "%d\t%d\n\r", angle, current);
        uart_sendStr(buffer);
        previous = current;
        angle_prev = angle;
    }

    uart_sendStr("\n\rObj\tAngle\tPingDist\tIRDist\tWidth\tType\n\r");
    for (int i = 0; i < total_objects; i++) {
        scanned[i].center_angle = (scanned[i].start_angle + scanned[i].end_angle) / 2;
        servo_move(scanned[i].center_angle);
        timer_waitMillis(500);
        scanned[i].ping_dist = ping_read();
        scanned[i].width = scanned[i].ping_dist * sin((scanned[i].end_angle - scanned[i].start_angle) * 3.14 / 180.0);

        if (scanned[i].width > 6.0 && scanned[i].width <= 11.0) {
            scanned[i].label = 'H';
            humans++;
        } else if (scanned[i].width > 11.0 && scanned[i].width <= 14.0) {
            scanned[i].label = 'T';
        } else {
            scanned[i].label = 'D';
        }

        sprintf(buffer, "%d\t%d\t%.2f\t%d\t%.2f\t%c\n\r", scanned[i].id, scanned[i].center_angle, scanned[i].ping_dist, scanned[i].ir_dist, scanned[i].width, scanned[i].label);
        uart_sendStr(buffer);
    }

    if (humans >= 3) return 2;
    else if (humans > 0) return 1;
    else return 0;
}