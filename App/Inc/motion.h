#ifndef APP_MOTION_H
#define APP_MOTION_H

#include "types.h"

/* ============================================================================
 * App 层 — 运动控制
 *
 * 接口分三层：
 *
 *  ① 顶层"动作"接口（业务代码只用这三个）
 *       Motion_Drive(speed, holdMs)            进退：正=前进 / 负=后退 / 0=停
 *       Motion_Steer(vLeft, vRight, holdMs)    转弯：左>右 = 右转，右>左 = 左转
 *       Motion_Brake()                         停车：当前速度 S 曲线降到 0
 *
 *     这三个都是**阻塞**的完整动作：
 *        先用 S 曲线把速度过渡到目标（时长 = 速度差 ÷ 加速度，
 *        默认加速度 2000Hz/s 也就是"1000Hz 用 500ms"），
 *        再保持 holdMs 毫秒，然后返回。变速时间不计入 holdMs。
 *     起始速度不用传 —— 本层内部记住"当前已下达的速度"，自动从那里过渡。
 *
 *     ⚠️ 换向（一侧由正变负）时本层会自动插入一段"先停 → 停 300ms → 再反向"，
 *        保证 DIR 在无脉冲边沿时切换。这叫"换向必须先停"，不能省。
 *
 *  ② 底层接口（调试或需要自管过渡时用）
 *       Motion_Move(vLeft, vRight)             立即生效，不平滑
 *       Motion_GetSpeed(&l, &r)                读回当前已下达的速度
 *
 *  ③ 变速参数（运行时可调，不用重烧）
 *       Motion_SetAccel(hzPerSec)              改加速度，对后续变速立即生效
 *       Motion_GetAccel()                      读回当前加速度
 *
 *  ④ 安全接口（急停 / 通信看门狗）—— 这一组是新加的
 *       Motion_Stop()                          立即断开脉冲（不等 S 曲线）
 *       Motion_RequestAbort()                  请求中止当前阻塞动作（可在中断里调）
 *       Motion_ClearAbort()                    清掉中止标志，准备接受新动作
 *       Motion_Aborted()                       查询是否处于中止状态
 *       Motion_NotifyAlive()                   上位机"还在线"心跳，喂看门狗
 *       Motion_Tick()                          主循环每圈调用，掉线超时自动停车
 *       Motion_IsMoving()                      当前是否有速度
 *       Motion_Demo()                          原来的上电六段演示，改成命令触发
 *
 * ⚠️ 速度单位是脉冲频率 Hz，不是 mm/s。
 *    左右各有一路定时器，所以左右可以不同（差速转向）；
 *    但**同侧两轮共用定时器，永远同频**。
 *
 * ⚠️ 动作函数内部用可中断的等待（每 8ms 检查一次中止标志），
 *    所以急停命令最迟 8ms 内生效；但等待期间主循环仍不执行。
 * ============================================================================ */

/* 初始化四路电机（内部调 Bsp_MotorPulse_Init），并把内部速度状态清零 */
void Motion_Init(void);

/* --- ① 进退：speed > 0 前进，< 0 后退，== 0 停止 ---------------------- */
void Motion_Drive(s32 speed, u32 holdMs);

/* --- ① 转弯：vLeft > vRight 右转，vRight > vLeft 左转，相等为直行 ---- */
void Motion_Steer(s32 vLeft, s32 vRight, u32 holdMs);

/* --- ① 停车：两侧 S 曲线降到 0 --------------------------------------- */
void Motion_Brake(void);

/* --- ② 底层：立即生效（不平滑），调用者自己负责过渡 -------------------- */
void Motion_Move(s32 vLeft, s32 vRight);

/* --- ② 底层：读回当前已下达的左右速度（Hz，有符号）；传 NULL 可忽略 --- */
void Motion_GetSpeed(s32 *vLeft, s32 *vRight);

/* --- ③ 加速度：hzPerSec = 2000 表示 1000Hz/500ms（默认值） ------------
 *     变速时长 = |目标速度 - 当前速度| / hzPerSec；两侧取变化更大的一侧。
 *     对调用之后的所有 Motion_Drive/Steer/Brake 立即生效；传 0 被忽略。
 *     运行期修改，复位后回到 CFG_SCURVE_ACCEL_HZ_PER_S。 */
void Motion_SetAccel(u32 hzPerSec);
u32  Motion_GetAccel(void);

/* --- ④ 立即停车：不等 S 曲线，直接把两路脉冲频率打成 0 ----------------
 *     用于急停。之后 s_speed_* 归零，下一次 Motion_Drive 从 0 起步。
 *     注意：只是"停脉冲"，驱动器仍然锁轴（M860D 静态电流约峰值的 50%）。 */
void Motion_Stop(void);

/* --- ④ 请求中止当前阻塞动作（可在中断上下文调用，只写一个 volatile 标志）
 *     被中止的动作会立即 Motion_Stop() 并返回。 */
void Motion_RequestAbort(void);

/* --- ④ 清除中止标志：每接受一条新的上位机命令时调一次 ---------------- */
void Motion_ClearAbort(void);

/* --- ④ 查询是否处于"已中止"状态（1 = 上一次动作被急停打断） ---------- */
u8   Motion_Aborted(void);

/* --- ④ 上位机心跳：收到运动类命令时调一次，给看门狗续期 ------------- */
void Motion_NotifyAlive(void);

/* --- ④ 主循环每圈调一次：超过 CFG_CMD_TIMEOUT_MS 没有心跳就自动停车 -- */
void Motion_Tick(void);

/* --- ④ 当前是否有速度（1 = 至少一侧在转） ---------------------------- */
u8   Motion_IsMoving(void);

/* --- ④ 原来的上电六段演示（内容一字未改），现在由串口 'D' 命令触发 ---
 *     中途发 'E' 急停会立刻打断并返回。 */
void Motion_Demo(void);

#endif /* APP_MOTION_H */
