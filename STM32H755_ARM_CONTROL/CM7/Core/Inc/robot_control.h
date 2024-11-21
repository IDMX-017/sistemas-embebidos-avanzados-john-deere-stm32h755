/*
 * robot_control.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Iván de México
 */

#ifndef INC_ROBOT_CONTROL_H_
#define INC_ROBOT_CONTROL_H_



//#define DT 0.01f         // Intervalo de tiempo (10 ms)
#define L 0.5f           // Distancia entre ejes del robot (metros)
#define LD 0.5f          // Distancia de mirada hacia adelante (lookahead distance)
#define DELTA_MAX (PI / 4)  // Ángulo máximo de dirección (45 grados)
#define DELTA_MIN (PI / 4)  // Ángulo máximo de dirección (45 grados)
#define V_MAX 1.0f       // Velocidad máxima (m/s)
#define V_MIN 0.0f       // Velocidad máxima (m/s)

typedef struct
{
	float32_t x;		//x position
	float32_t y; 		//y position
	float32_t theta; 	//Orientation
	float32_t delta; 	//Angle in of direction
	float32_t v; 		//Linear Speed

} RobotState;

typedef struct {
    float32_t v;        // Speed control signal
    float32_t delta;    // Direction control signal
} RobotControl;

typedef enum {IDLE, THROTTLE,TOP_SPEED, BRAKING} Status;

typedef struct
{
	float32_t x;
	float32_t y;
	int32_t flag;
} Waypoint;

typedef struct {
    Waypoint *waypoints;    // Puntero al array de waypoints
    uint32_t length;        // Longitud de la trayectoria
    uint32_t current_index; // Índice del waypoint actual
} Path;

void robot_inti(RobotState *state);
void pure_pursuit(RobotState *state, float32_t *delta_deseada, float32_t *v_deseada, float32_t path[][2], uint32_t path_length);
void update_robot_state(RobotState *state, RobotControl *control);
float32_t saturate(float32_t value, float32_t min, float32_t max);

#endif /* INC_ROBOT_CONTROL_H_ */
