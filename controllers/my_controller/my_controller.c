#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/motor.h>

#define TIME_STEP 32
#define QtddCaixa 20
#define VEL_MAX 6.28
#define STEPS_5_SECONDS (5000 / TIME_STEP)

int main() {
  wb_robot_init();
  srand(time(NULL));  // Inicializa semente de aleatoriedade

  // Motores
  WbDeviceTag motor_esquerdo = wb_robot_get_device("left wheel motor");
  WbDeviceTag motor_direito = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(motor_esquerdo, INFINITY);
  wb_motor_set_position(motor_direito, INFINITY);
  wb_motor_set_velocity(motor_esquerdo, 0);
  wb_motor_set_velocity(motor_direito, 0);

  // Referência ao próprio robô
  WbNodeRef robo = wb_supervisor_node_get_self();

  // Referência das caixas e armazenamento das posições
  WbNodeRef caixa[QtddCaixa];
  double pos_caixas[QtddCaixa][3];
  char nomeCaixa[10];
  for (int i = 0; i < QtddCaixa; i++) {
    sprintf(nomeCaixa, "CAIXA%02d", i);
    caixa[i] = wb_supervisor_node_get_from_def(nomeCaixa);
    if (caixa[i] != NULL) {
      const double *pos = wb_supervisor_node_get_position(caixa[i]);
      pos_caixas[i][0] = pos[0];
      pos_caixas[i][1] = pos[1];
      pos_caixas[i][2] = pos[2];
      printf("%2d. %s - OK\n", i, nomeCaixa);
    } else {
      pos_caixas[i][0] = pos_caixas[i][1] = pos_caixas[i][2] = NAN;
      printf("%2d. %s - ERRO\n", i, nomeCaixa);
    }
  }

  printf("\n--- SIMULAÇÃO INICIADA ---\n");

  // Controle de movimento aleatório por 5 segundos
  int step_counter = 0;
  double v_esq = 0, v_dir = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    const double *pos_robo = wb_supervisor_node_get_position(robo);

    printf("Posição relativa do robô em relação às caixas:\n");
    int encontrou_proxima = 0;

    for (int i = 0; i < QtddCaixa; i++) {
      if (isnan(pos_caixas[i][0])) continue;

      double dx = pos_caixas[i][0] - pos_robo[0];
      double dy = pos_caixas[i][1] - pos_robo[1];
      double dz = pos_caixas[i][2] - pos_robo[2];
      double distancia = sqrt(dx*dx + dy*dy + dz*dz);
      printf("CAIXA%02d | dx=%.2f dy=%.2f dz=%.2f | dist=%.2f\n",
             i, dx, dy, dz, distancia);

      if (distancia < 0.07) {
        encontrou_proxima = 1;
        break;
      }
    }

    if (encontrou_proxima) {
      while (wb_robot_step(TIME_STEP) != -1) {
        wb_motor_set_velocity(motor_esquerdo, 0.5 * VEL_MAX);
        wb_motor_set_velocity(motor_direito, -0.5 * VEL_MAX);
      }
    }

    // Atualiza velocidade a cada 5 segundos
    if (step_counter == 0) {
      v_esq = ((double)rand() / RAND_MAX) * VEL_MAX;
      v_dir = ((double)rand() / RAND_MAX) * VEL_MAX;
      if (rand() % 2) v_esq = -v_esq;
      if (rand() % 2) v_dir = -v_dir;
      printf("Nova velocidade: E=%.2f | D=%.2f\n", v_esq, v_dir);
    }

    wb_motor_set_velocity(motor_esquerdo, v_esq);
    wb_motor_set_velocity(motor_direito, v_dir);

    step_counter++;
    if (step_counter >= STEPS_5_SECONDS)
      step_counter = 0;
  }

  wb_robot_cleanup();
  return 0;
}
