#ifndef __GRIPPER_TEST_H
#define __GRIPPER_TEST_H

#include "Motor.h"

/* DM-J4310 gripper bench-test object: feedback lands here.
   Watch in debugger: Gripper_Motor.FeedbackData.FrameCounter (must keep rising),
   Gripper_Motor.RealAngle (deg, single-turn), mtr.toq / mtr.Tcoil in DM_J4310.c */
extern motor_t Gripper_Motor;

/* Motion controls, set from the debugger console (paused, one line at a time):
   Grip_Enable = 1/0     enable / disable the motor
   Grip_TargetDeg        fixed target, degrees (absolute multi-turn frame)
   Grip_VelRadS          command velocity limit, rad/s
   Grip_Cycle = 1        auto open/close loop Grip_OpenDeg <-> Grip_CloseDeg,
                         switching every Grip_DwellMs                        */
extern volatile float Grip_TargetDeg;
extern volatile uint8_t Grip_Enable;
extern volatile float Grip_VelRadS;
extern volatile uint8_t  Grip_Cycle;
extern volatile float    Grip_OpenDeg;
extern volatile float    Grip_CloseDeg;
extern volatile uint32_t Grip_DwellMs;

/* ---- Motor parameter access over CAN (replaces the DMTool route) ----
   Serviced only while Grip_Enable == 0. Register map (Damiao):
   0x19 = velocity KP (KP_ASR), 0x1A = velocity KI (KI_ASR),
   0x1B = position KP (KP_APR), 0x1C = position KI (KI_APR).
   First use: set Grip_ReadRID = 0x1B, resume a moment, pause again:
   Grip_ReadValueF must come back ~94.0 (the known position-KP baseline)
   before any write is trusted. Grip_ReadRaw keeps the raw reply bytes. */
extern volatile float Grip_KpVel;      /* staged gains, baselines preloaded */
extern volatile float Grip_KiVel;
extern volatile float Grip_KpPos;
extern volatile float Grip_KiPos;
extern volatile uint8_t Grip_ApplyGains;   /* set 1: write 4 gains + read back */
extern volatile uint8_t Grip_SaveParams;   /* set 1: store params to flash     */
extern volatile uint8_t Grip_ReadRID;      /* set to a RID: one-shot read      */
extern volatile float Grip_ReadValueF;     /* decoded value of last read       */
extern volatile uint8_t Grip_ReadRaw[8];   /* raw reply frame bytes            */
extern volatile uint8_t Grip_ReplySeen;    /* 1 = a reply frame was captured   */
extern volatile float Grip_RbKpVel;        /* read-back after ApplyGains       */
extern volatile float Grip_RbKiVel;
extern volatile float Grip_RbKpPos;
extern volatile float Grip_RbKiPos;

/* Called from User_CAN.c on every 0x11 frame before feedback parsing.
   Returns 1 when the frame was a parameter reply and has been consumed. */
uint8_t Gripper_ParamOnReply(uint8_t *data);

void GripperTest_Start(void);

#endif
