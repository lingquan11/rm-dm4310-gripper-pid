/**
 * Gripper_Test.c - bench bring-up for the DM-J4310 gripper on CAN2.
 *
 * The team BSP ships the DM_J4310 driver but nothing calls it.
 * This file adds the missing wiring:
 *   - feedback routing lives in User_CAN.c (case 0x11)
 *   - a task that enables the motor only after Grip_Enable is set to 1
 *     from the debugger, then streams position-speed commands at 100 Hz
 *   - motor parameter read/write over CAN (the DMTool protocol, frame
 *     ID 0x7FF), so PID gains can be tuned from the debugger console
 *     without the USB2CAN tool. Parameter ops run only while disabled.
 *
 * Debugger controls: set ONE LINE AT A TIME in the debug console while
 * paused (pasting a whole block is evaluated as one expression and
 * fails). See Gripper_Test.h for the full variable list.
 *
 * Bench calibration (2026-11-14, verified on the real gripper):
 *   rest / fingers touching  = -14.25 deg  (RealAngle 345.75)
 *   squeezed fully closed    =  -1.88 deg  (RealAngle 358.12)
 *   closing is positive, opening goes negative from rest.
 *   Never hold a position command into the closed stop: stalling there
 *   tripped coil over-temperature (ERR C) within ~2 minutes. Grip by
 *   torque (mtr.toq), not by position.
 */
#include "Gripper_Test.h"
#include "DM_J4310.h"
#include "Task_Can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

#define GRIP_MOTOR_ID   1       /* node id burned in the motor driver board */
#define GRIP_CAN_BUS    2       /* CANSEND_2 -> hfdcan2                     */

/* Damiao register IDs for the two control loops */
#define DM_REG_KP_VEL   0x19u   /* KP_ASR */
#define DM_REG_KI_VEL   0x1Au   /* KI_ASR */
#define DM_REG_KP_POS   0x1Bu   /* KP_APR */
#define DM_REG_KI_POS   0x1Cu   /* KI_APR */

motor_t Gripper_Motor;
volatile float Grip_TargetDeg = 0.0f;
volatile uint8_t Grip_Enable = 0;
volatile float Grip_VelRadS = 0.5f;         /* slow and safe to start   */

/* Auto-cycle controls: defaults stay clear of the hard squeeze zone. */
volatile uint8_t  Grip_Cycle    = 0;
volatile float    Grip_OpenDeg  = -30.0f;   /* fingers open              */
volatile float    Grip_CloseDeg = -12.0f;   /* just past touching, safe  */
volatile uint32_t Grip_DwellMs  = 1500;     /* hold time at each end     */

/* Parameter access: staged gains preloaded with the 2026-11-14 baselines
   read from this very motor (position KP 94 / KI 1, velocity KP 0.0045 /
   KI 0.015708), so an accidental apply restores the factory state.     */
volatile float Grip_KpVel = 0.0045f;
volatile float Grip_KiVel = 0.015708f;
volatile float Grip_KpPos = 94.0f;
volatile float Grip_KiPos = 1.0f;
volatile uint8_t Grip_ApplyGains  = 0;
volatile uint8_t Grip_SaveParams = 0;
volatile uint8_t Grip_ReadRID = 0xFFu;
volatile float Grip_ReadValueF = 0.0f;
volatile uint8_t Grip_ReadRaw[8] = {0};
volatile uint8_t Grip_ReplySeen = 0;
volatile float Grip_RbKpVel = 0.0f;
volatile float Grip_RbKiVel = 0.0f;
volatile float Grip_RbKpPos = 0.0f;
volatile float Grip_RbKiPos = 0.0f;

static volatile uint8_t Param_Busy = 0;
static volatile uint8_t Param_Done = 0;

/* Register frame: ID 0x7FF, data [esc_id, 0, cmd, rid, value LE x4].
   cmd 0x33 = read, 0x55 = write, 0xAA = save all to flash.            */
static void Param_Send(uint8_t cmd, uint8_t rid, float value)
{
    CanSend_Type msg;

    msg.CANx   = GRIP_CAN_BUS;
    msg.stdid  = 0x7FFu;
    msg.Data[0] = GRIP_MOTOR_ID;
    msg.Data[1] = 0;
    msg.Data[2] = cmd;
    msg.Data[3] = rid;
    memcpy(&msg.Data[4], &value, 4);

    xQueueSend(Queue_CANSend, &msg, 3);
}

uint8_t Gripper_ParamOnReply(uint8_t *data)
{
    /* While a parameter op is outstanding, the motor is disabled and
       nothing else talks to it, so the next 0x11 frame IS the reply.
       Capture it raw unconditionally; decode the value from bytes 4..7
       (reply layout mirrors the request: [id, 0, cmd, rid, value]). */
    if (Param_Busy)
    {
        memcpy((void *)Grip_ReadRaw, data, 8);
        Grip_ReplySeen = 1;
        if (data[2] == 0x33u)
            memcpy((void *)&Grip_ReadValueF, &data[4], 4);
        Param_Busy = 0;
        Param_Done = 1;
        return 1;
    }
    return 0;
}

/* Send one register op and wait briefly for its reply. Returns 1 when
   the motor answered. Runs in task context (delays are fine).         */
static uint8_t Param_Op(uint8_t cmd, uint8_t rid, float value)
{
    uint32_t waited = 0;

    Grip_ReplySeen = 0;
    Param_Done = 0;
    Param_Busy = 1;
    Param_Send(cmd, rid, value);

    while (!Param_Done && waited < 100u)
    {
        vTaskDelay(pdMS_TO_TICKS(5));
        waited += 5u;
    }
    Param_Busy = 0;
    return Param_Done;
}

static void Param_Service(void)
{
    if (Grip_ReadRID != 0xFFu)
    {
        uint8_t rid = Grip_ReadRID;
        Grip_ReadRID = 0xFFu;
        Param_Op(0x33u, rid, 0.0f);
    }

    if (Grip_ApplyGains)
    {
        Grip_ApplyGains = 0;
        Param_Op(0x55u, DM_REG_KP_VEL, Grip_KpVel);
        Param_Op(0x55u, DM_REG_KI_VEL, Grip_KiVel);
        Param_Op(0x55u, DM_REG_KP_POS, Grip_KpPos);
        Param_Op(0x55u, DM_REG_KI_POS, Grip_KiPos);
        if (Param_Op(0x33u, DM_REG_KP_VEL, 0.0f)) Grip_RbKpVel = Grip_ReadValueF;
        if (Param_Op(0x33u, DM_REG_KI_VEL, 0.0f)) Grip_RbKiVel = Grip_ReadValueF;
        if (Param_Op(0x33u, DM_REG_KP_POS, 0.0f)) Grip_RbKpPos = Grip_ReadValueF;
        if (Param_Op(0x33u, DM_REG_KI_POS, 0.0f)) Grip_RbKiPos = Grip_ReadValueF;
    }

    if (Grip_SaveParams)
    {
        Grip_SaveParams = 0;
        Param_Send(0xAAu, 0, 0.0f);
    }
}

static void GripperTest_Task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(1000));
    DMJ4310_motorErrClear(GRIP_MOTOR_ID, GRIP_CAN_BUS);
    vTaskDelay(pdMS_TO_TICKS(50));

    for (;;)
    {
        if (Grip_Enable)
        {
            DMJ4310_motorOpen(GRIP_MOTOR_ID, GRIP_CAN_BUS);
            vTaskDelay(pdMS_TO_TICKS(50));

            uint8_t  phase = 0;   /* 0 = at open end, 1 = at close end */
            uint32_t ticks = 0;

            while (Grip_Enable)
            {
                float target = Grip_TargetDeg;

                if (Grip_Cycle)
                {
                    target = phase ? Grip_CloseDeg : Grip_OpenDeg;
                    if (++ticks >= Grip_DwellMs / 10u)
                    {
                        ticks = 0;
                        phase ^= 1u;
                    }
                }

                DMJ4310_speedpositionControl(GRIP_MOTOR_ID, GRIP_CAN_BUS,
                                             target, Grip_VelRadS);
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            DMJ4310_motorClose(GRIP_MOTOR_ID, GRIP_CAN_BUS);
        }
        else
        {
            Param_Service();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void GripperTest_Start(void)
{
    xTaskCreate(GripperTest_Task, "grip", 256, NULL, 5, NULL);
}
