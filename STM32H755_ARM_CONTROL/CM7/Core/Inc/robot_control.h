/*
 * robot_control.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Iván de México
 */

#ifndef INC_ROBOT_CONTROL_H_
#define INC_ROBOT_CONTROL_H_



//#define DT 0.01f         // Intervalo de tiempo (10 ms)
#define L 0.15f           // Distancia entre ejes del robot (metros)
#define LD 0.01f          // Distancia de mirada hacia adelante (lookahead distance)
#define DELTA_MAX (60.0f)*(PI / 180.0f)  // Ángulo máximo de dirección (45 grados)
#define DELTA_MIN (-50.0f)*(PI / 180.0f)  // Ángulo máximo de dirección (45 grados)
#define V_MAX 1.0f       // Velocidad máxima (m/s)
#define V_MIN 0.0f       // Velocidad máxima (m/s)

typedef struct
{
	float32_t x;		//x position, global
	float32_t y; 		//y position, global
	float32_t theta; 	//Orientation, feedback o percepción de la IMU
	float32_t delta; 	//Angle of direction, el ángulo actual de la dirección actualizado cuando haya cambios en el servo
	float32_t v; 		//Linear Speed, aqui sería el feedback o velocidad medida por el encoder

} RobotState;

typedef struct {
    float32_t v;        // Speed control signal
    float32_t delta;    // Direction control signal
} RobotControl; //Estructura de señales de control

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

void robot_init(RobotState *state);
void pure_pursuit(RobotState *state, float32_t *delta_deseada, float32_t *v_deseada, float32_t path[][2], uint32_t path_length);
void update_robot_state(RobotState *state, RobotControl *control);
float32_t saturate(float32_t value, float32_t min, float32_t max);
float32_t normalize_angle(float32_t angle);
void PID_Init(arm_pid_instance_f32 *pid, float32_t Kp, float32_t Ki, float32_t Kd);
float32_t PID_Angle(arm_pid_instance_f32 *pid_delta, float32_t delta_desired, float32_t delta_actual);
float32_t ANGLE_OBJECTIVE(RobotState *state, float32_t x_desired, float32_t y_desired);
float32_t calc_vector_length(float32_t x1, float32_t y1, float32_t x2, float32_t y2);
void PURE_PURSUIT(RobotState *state, float32_t *desired_delta, float32_t *desired_v, Path *path);






#endif /* INC_ROBOT_CONTROL_H_ */
