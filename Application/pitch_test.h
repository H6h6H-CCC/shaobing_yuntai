#ifndef PITCH_TEST_H
#define PITCH_TEST_H

#include <stdint.h>

/* Pitch调参时每50ms输出一次；改为0恢复原有每秒整车状态输出。 */
#define PITCH_TUNING_DEBUG 1
#define PITCH_DEBUG_PERIOD_MS 50U

void Pitch_Test_Init(void);
void Pitch_Test_Update(void);
void Pitch_Test_DebugPrint(void);
void Pitch_Test_GetYawDebug(uint8_t *mode,
                            int16_t *channel,
                            int16_t *angle10,
                            int16_t *target10,
                            int16_t *error10,
                            int16_t *target_rpm,
                            int16_t *speed_rpm,
                            int16_t *current);

#endif
