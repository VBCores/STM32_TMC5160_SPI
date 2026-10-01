/* Header-only TMC5160 library for STM32Cube HAL. Usable from C and C++.
 * Hardware and motor constants are defined in robot_config.h.
 */
#ifndef TMC5160_H
#define TMC5160_H

#include <stdint.h>
#include <limits.h>
#include "robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Datasheet addresses; write_reg adds the SPI write bit. */
#define TMC5160_GCONF          0x00U
#define TMC5160_GSTAT          0x01U
#define TMC5160_IFCNT          0x02U
#define TMC5160_IOIN           0x04U
#define TMC5160_GLOBAL_SCALER  0x0BU
#define TMC5160_IHOLD_IRUN     0x10U
#define TMC5160_TPOWERDOWN     0x11U
#define TMC5160_TSTEP          0x12U
#define TMC5160_TPWMTHRS       0x13U
#define TMC5160_TCOOLTHRS      0x14U
#define TMC5160_THIGH          0x15U
#define TMC5160_RAMPMODE       0x20U
#define TMC5160_XACTUAL        0x21U
#define TMC5160_VACTUAL        0x22U
#define TMC5160_VSTART         0x23U
#define TMC5160_A1             0x24U
#define TMC5160_V1             0x25U
#define TMC5160_AMAX           0x26U
#define TMC5160_VMAX           0x27U
#define TMC5160_DMAX           0x28U
#define TMC5160_D1             0x2AU
#define TMC5160_VSTOP          0x2BU
#define TMC5160_TZEROWAIT      0x2CU
#define TMC5160_XTARGET        0x2DU
#define TMC5160_SW_MODE        0x34U
#define TMC5160_RAMP_STAT      0x35U
#define TMC5160_CHOPCONF       0x6CU
#define TMC5160_COOLCONF       0x6DU
#define TMC5160_DRV_STATUS     0x6FU
#define TMC5160_PWMCONF        0x70U

#define TMC5160_MODE_POSITION          0U
#define TMC5160_MODE_VELOCITY_POSITIVE 1U
#define TMC5160_MODE_VELOCITY_NEGATIVE 2U
#define TMC5160_MODE_HOLD              3U
#define TMC5160_GCONF_EN_PWM_MODE       (1UL << 2)
#define TMC5160_GCONF_SHAFT             (1UL << 4)

#define TMC5160_TWO_PI                 6.2831853071795864769
#define TMC5160_VELOCITY_SCALE         16777216.0
#define TMC5160_ACCELERATION_SCALE     2199023255552.0

/* One 40-bit frame. Read data is returned by the NEXT frame. */
static inline uint32_t tmc5160_transfer(uint8_t address, uint32_t value)
{
    uint8_t tx[5] = {address, (uint8_t)(value >> 24),
                     (uint8_t)(value >> 16), (uint8_t)(value >> 8),
                     (uint8_t)value};
    uint8_t rx[5] = {0};
    HAL_GPIO_WritePin(ROBOT_TMC_CS_PORT, ROBOT_TMC_CS_PIN, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(ROBOT_TMC_SPI_HANDLE, tx, rx, 5U,
                                  HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ROBOT_TMC_CS_PORT, ROBOT_TMC_CS_PIN, GPIO_PIN_SET);
    return ((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16) |
           ((uint32_t)rx[3] << 8) | (uint32_t)rx[4];
}

static inline void tmc5160_write_reg(uint8_t address, uint32_t value)
{ (void)tmc5160_transfer((uint8_t)(address | 0x80U), value); }

static inline uint32_t tmc5160_read_reg(uint8_t address)
{
    address &= 0x7FU;
    (void)tmc5160_transfer(address, 0U);
    return tmc5160_transfer(address, 0U);
}

/* These functions write complete register images. */
#define TMC5160_WRITE_FUNCTION(name, reg) \
    static inline void name(uint32_t value) \
    { tmc5160_write_reg(reg, value); }
#define TMC5160_READ_FUNCTION(name, reg) \
    static inline uint32_t name(void) \
    { return tmc5160_read_reg(reg); }

TMC5160_WRITE_FUNCTION(tmc5160_set_general_config, TMC5160_GCONF)
TMC5160_WRITE_FUNCTION(tmc5160_set_chopper_config, TMC5160_CHOPCONF)
TMC5160_WRITE_FUNCTION(tmc5160_set_pwm_config, TMC5160_PWMCONF)
TMC5160_WRITE_FUNCTION(tmc5160_set_coolstep_config, TMC5160_COOLCONF)
TMC5160_WRITE_FUNCTION(tmc5160_set_pwm_threshold, TMC5160_TPWMTHRS)
TMC5160_WRITE_FUNCTION(tmc5160_set_coolstep_threshold, TMC5160_TCOOLTHRS)
TMC5160_WRITE_FUNCTION(tmc5160_set_high_speed_threshold, TMC5160_THIGH)
TMC5160_WRITE_FUNCTION(tmc5160_set_start_velocity, TMC5160_VSTART)
TMC5160_WRITE_FUNCTION(tmc5160_set_first_acceleration, TMC5160_A1)
TMC5160_WRITE_FUNCTION(tmc5160_set_transition_velocity, TMC5160_V1)
TMC5160_WRITE_FUNCTION(tmc5160_set_max_acceleration, TMC5160_AMAX)
TMC5160_WRITE_FUNCTION(tmc5160_set_max_velocity, TMC5160_VMAX)
TMC5160_WRITE_FUNCTION(tmc5160_set_max_deceleration, TMC5160_DMAX)
TMC5160_WRITE_FUNCTION(tmc5160_set_final_deceleration, TMC5160_D1)
TMC5160_WRITE_FUNCTION(tmc5160_set_stop_velocity, TMC5160_VSTOP)
TMC5160_WRITE_FUNCTION(tmc5160_set_switch_mode, TMC5160_SW_MODE)

TMC5160_READ_FUNCTION(tmc5160_read_general_status, TMC5160_GSTAT)
TMC5160_READ_FUNCTION(tmc5160_read_io_status, TMC5160_IOIN)
TMC5160_READ_FUNCTION(tmc5160_read_step_time, TMC5160_TSTEP)
TMC5160_READ_FUNCTION(tmc5160_read_ramp_status, TMC5160_RAMP_STAT)
TMC5160_READ_FUNCTION(tmc5160_read_driver_status, TMC5160_DRV_STATUS)

#undef TMC5160_WRITE_FUNCTION
#undef TMC5160_READ_FUNCTION

/* IHOLD, IRUN and IHOLDDELAY share one register. */
static inline void tmc5160_set_current(uint8_t ihold, uint8_t irun,
                                       uint8_t ihold_delay)
{
    tmc5160_write_reg(TMC5160_IHOLD_IRUN,
                      ((uint32_t)(ihold_delay & 0x0FU) << 16) |
                      ((uint32_t)(irun & 0x1FU) << 8) |
                      (uint32_t)(ihold & 0x1FU));
}

/* 0 means full scale; operating values are 32..255. */
static inline void tmc5160_set_global_scaler(uint8_t scaler)
{ tmc5160_write_reg(TMC5160_GLOBAL_SCALER, scaler); }

static inline void tmc5160_set_powerdown_delay(uint8_t delay)
{ tmc5160_write_reg(TMC5160_TPOWERDOWN, delay); }

static inline void tmc5160_set_ramp_mode(uint8_t mode)
{ tmc5160_write_reg(TMC5160_RAMPMODE, mode); }

/* Raw positions use motor microsteps, raw ramp values use TMC register units. */
static inline void tmc5160_set_actual_position(int32_t value)
{ tmc5160_write_reg(TMC5160_XACTUAL, (uint32_t)value); }

static inline void tmc5160_set_target_position(int32_t value)
{ tmc5160_write_reg(TMC5160_XTARGET, (uint32_t)value); }

static inline void tmc5160_set_zero_wait(uint16_t value)
{ tmc5160_write_reg(TMC5160_TZEROWAIT, value); }

static inline int32_t tmc5160_read_actual_position(void)
{ return (int32_t)tmc5160_read_reg(TMC5160_XACTUAL); }

static inline int32_t tmc5160_read_target_position(void)
{ return (int32_t)tmc5160_read_reg(TMC5160_XTARGET); }

/* VACTUAL is signed, 24 bits wide. */
static inline int32_t tmc5160_read_actual_velocity(void)
{
    uint32_t value = tmc5160_read_reg(TMC5160_VACTUAL) & 0xFFFFFFU;
    return (value & 0x800000U) ? (int32_t)value - 0x1000000L : (int32_t)value;
}

static inline uint8_t tmc5160_read_interface_count(void)
{ return (uint8_t)tmc5160_read_reg(TMC5160_IFCNT); }

/* Physical-unit conversions for the output shaft. */
static inline double tmc5160_steps_per_output_revolution(void)
{
    return (double)ROBOT_MOTOR_FULL_STEPS *
           (double)ROBOT_MOTOR_MICROSTEPS * ROBOT_MOTOR_GEAR_RATIO;
}

static inline int32_t tmc5160_radians_to_steps(double radians)
{
    double steps = radians * tmc5160_steps_per_output_revolution() /
                   TMC5160_TWO_PI;
    if (steps >= (double)INT32_MAX) return INT32_MAX;
    if (steps <= (double)INT32_MIN) return INT32_MIN;
    return (int32_t)(steps + (steps >= 0.0 ? 0.5 : -0.5));
}

static inline double tmc5160_steps_to_radians(int32_t steps)
{
    return (double)steps * TMC5160_TWO_PI /
           tmc5160_steps_per_output_revolution();
}

static inline uint32_t tmc5160_velocity_to_register(double radians_per_second)
{
    double value = radians_per_second * tmc5160_steps_per_output_revolution() /
                   TMC5160_TWO_PI * TMC5160_VELOCITY_SCALE /
                   (double)ROBOT_TMC_CLOCK_HZ;
    if (value < 0.0) value = -value;
    if (value >= 8388096.0) return 8388096U; /* 2^23 - 512 */
    return (uint32_t)(value + 0.5);
}

static inline uint32_t tmc5160_acceleration_to_register(
    double radians_per_second_squared)
{
    double clock = (double)ROBOT_TMC_CLOCK_HZ;
    double value = radians_per_second_squared *
                   tmc5160_steps_per_output_revolution() /
                   TMC5160_TWO_PI * TMC5160_ACCELERATION_SCALE /
                   (clock * clock);
    if (value < 0.0) value = -value;
    if (value >= 65535.0) return 65535U;
    return (uint32_t)(value + 0.5);
}

static inline double tmc5160_default_velocity_rad_s(void)
{
    return ROBOT_TMC_VMAX_STEPS_S * TMC5160_TWO_PI /
           tmc5160_steps_per_output_revolution();
}

static inline void tmc5160_set_default_vel(void)
{
    tmc5160_set_transition_velocity(
        tmc5160_velocity_to_register(
            ROBOT_TMC_V1_STEPS_S * TMC5160_TWO_PI /
            tmc5160_steps_per_output_revolution()));
    tmc5160_set_max_velocity(
        tmc5160_velocity_to_register(tmc5160_default_velocity_rad_s()));
}

/* Call after HAL_Init(), MX_GPIO_Init() and MX_SPIx_Init().
 * DRV_ENN is held high while interface straps settle. Current and motion
 * profile are then written from robot_config.h. The ramp starts in HOLD.
 */
static inline void tmc5160_init(void)
{
#ifdef ROBOT_TMC_ENABLE_PORT
    HAL_GPIO_WritePin(ROBOT_TMC_ENABLE_PORT, ROBOT_TMC_ENABLE_PIN, GPIO_PIN_SET);
#endif
#ifdef ROBOT_TMC_SPI_MODE_PORT
    HAL_GPIO_WritePin(ROBOT_TMC_SPI_MODE_PORT, ROBOT_TMC_SPI_MODE_PIN, GPIO_PIN_SET);
#endif
#ifdef ROBOT_TMC_SD_MODE_PORT
    HAL_GPIO_WritePin(ROBOT_TMC_SD_MODE_PORT, ROBOT_TMC_SD_MODE_PIN, GPIO_PIN_RESET);
#endif
    HAL_GPIO_WritePin(ROBOT_TMC_CS_PORT, ROBOT_TMC_CS_PIN, GPIO_PIN_SET);
#ifdef ROBOT_TMC_STEP_PORT
    HAL_GPIO_WritePin(ROBOT_TMC_STEP_PORT, ROBOT_TMC_STEP_PIN, GPIO_PIN_RESET);
#endif
#ifdef ROBOT_TMC_DIR_PORT
    HAL_GPIO_WritePin(ROBOT_TMC_DIR_PORT, ROBOT_TMC_DIR_PIN, GPIO_PIN_RESET);
#endif
    HAL_Delay(10U);
#ifdef ROBOT_TMC_ENABLE_PORT
    HAL_GPIO_WritePin(ROBOT_TMC_ENABLE_PORT, ROBOT_TMC_ENABLE_PIN, GPIO_PIN_RESET);
    HAL_Delay(100U);
#endif

    tmc5160_set_chopper_config(ROBOT_TMC_CHOPCONF);
    tmc5160_set_global_scaler(ROBOT_TMC_GLOBAL_SCALER);
    tmc5160_set_current(ROBOT_TMC_IHOLD, ROBOT_TMC_IRUN,
                        ROBOT_TMC_IHOLD_DELAY);
    tmc5160_set_powerdown_delay(ROBOT_TMC_POWERDOWN_DELAY);
    tmc5160_set_general_config(
        (ROBOT_TMC_GCONF & ~TMC5160_GCONF_SHAFT) |
        (ROBOT_MOTOR_REVERSE ? TMC5160_GCONF_SHAFT : 0U));
    tmc5160_set_pwm_threshold(ROBOT_TMC_PWM_THRESHOLD);
    tmc5160_set_start_velocity(ROBOT_TMC_VSTART);
    tmc5160_set_stop_velocity(ROBOT_TMC_VSTOP);
    tmc5160_set_first_acceleration(ROBOT_TMC_A1);
    tmc5160_set_max_acceleration(ROBOT_TMC_AMAX);
    tmc5160_set_max_deceleration(ROBOT_TMC_DMAX);
    tmc5160_set_final_deceleration(ROBOT_TMC_D1);
    tmc5160_set_default_vel();
    tmc5160_set_ramp_mode(TMC5160_MODE_HOLD);
}

static inline void tmc5160_arm(void)
{
#ifdef ROBOT_TMC_ENABLE_PORT
    HAL_GPIO_WritePin(ROBOT_TMC_ENABLE_PORT, ROBOT_TMC_ENABLE_PIN, GPIO_PIN_RESET);
#endif
}

static inline void tmc5160_disarm(void)
{
#ifdef ROBOT_TMC_ENABLE_PORT
    HAL_GPIO_WritePin(ROBOT_TMC_ENABLE_PORT, ROBOT_TMC_ENABLE_PIN, GPIO_PIN_SET);
#endif
}

static inline void tmc5160_set_motor_direction(int8_t direction)
{
    uint32_t config = tmc5160_read_reg(TMC5160_GCONF);
    config = (config & ~TMC5160_GCONF_SHAFT) |
             (direction < 0 ? TMC5160_GCONF_SHAFT : 0U);
    tmc5160_set_general_config(config);
}

/* All motor API motion values refer to the output shaft. */
static inline void tmc5160_set_velocity(double radians_per_second)
{
    uint32_t vmax = tmc5160_velocity_to_register(radians_per_second);
    tmc5160_set_transition_velocity(vmax / 2U);
    tmc5160_set_max_velocity(vmax);
}

static inline void tmc5160_set_acceleration(
    double radians_per_second_squared)
{
    uint32_t acceleration =
        tmc5160_acceleration_to_register(radians_per_second_squared);
    tmc5160_set_first_acceleration(acceleration);
    tmc5160_set_max_acceleration(acceleration);
    tmc5160_set_max_deceleration(acceleration);
    tmc5160_set_final_deceleration(acceleration);
}

static inline void tmc5160_set_pos(double radians)
{
    tmc5160_set_default_vel();
    tmc5160_set_ramp_mode(TMC5160_MODE_POSITION);
    tmc5160_set_target_position(tmc5160_radians_to_steps(radians));
}

static inline void tmc5160_move(double radians_per_second)
{
    tmc5160_set_velocity(radians_per_second);
    tmc5160_set_ramp_mode(
        radians_per_second < 0.0 ? TMC5160_MODE_VELOCITY_NEGATIVE :
                                   TMC5160_MODE_VELOCITY_POSITIVE);
}

static inline double tmc5160_get_pos(void)
{ return tmc5160_steps_to_radians(tmc5160_read_actual_position()); }

static inline double tmc5160_get_velocity(void)
{
    double microsteps_per_second =
        (double)tmc5160_read_actual_velocity() *
        (double)ROBOT_TMC_CLOCK_HZ / TMC5160_VELOCITY_SCALE;
    return microsteps_per_second * TMC5160_TWO_PI /
           tmc5160_steps_per_output_revolution();
}

/* Decelerate to zero using AMAX. */
static inline void tmc5160_stop(void)
{
    tmc5160_set_ramp_mode(TMC5160_MODE_VELOCITY_POSITIVE);
    tmc5160_set_max_velocity(0U);
}

/* Call when stationary. The motor remains in HOLD until the next command. */
static inline void tmc5160_set_zero(void)
{
    tmc5160_set_ramp_mode(TMC5160_MODE_HOLD);
    tmc5160_set_actual_position(0);
    tmc5160_set_target_position(0);
}

#ifdef __cplusplus
}
#endif
#endif /* TMC5160_H */
