#include <cstdio>
#include "Arduino.h"
#include "config.h"
#include "drive_controller.h"
#include "safety_manager.h"
int main(){
 StateEstimator est; StateEstimator::Config cfg; est.begin(cfg); est.setHeading(0,0);
 DriveController d; d.begin(); d.setAuto(.25f,120.f);
 for(int i=0;i<100;i++){mock_advance_ms(10);d.update(.01f,est);}
 d.setAuto(0.f,120.f);
 for(int i=0;i<100;i++){mock_advance_ms(10);d.update(.01f,est);}
 printf("zero-speed during spin: pwm=%d,%d spinning=%d\n",d.pwmLeft(),d.pwmRight(),d.spinning());
 d.stop();d.setTurnTo(90.f);
 bool motion=fabsf(d.targetSpeed())>1e-3f||fabsf(d.targetYawRate())>1e-3f;
 SafetyManager s;s.begin();mock_pin_state[PIN_E_EMER]=E_EMER_ACTIVE_LOW?HIGH:LOW;
 SafetyManager::Inputs in;in.agent_ok=true;in.heartbeat_age_ms=0;in.cmd_age_ms=10000;in.motion_requested=motion;
 for(int i=0;i<1000;i++){mock_advance_ms(10);s.update(in);}
 printf("turn with 10-second-old command: motion_requested=%d allowMotion=%d cmd_timeout_flag=%d\n",motion,s.allowMotion(),bool(s.flags()&STOP_CMD_TIMEOUT));
 d.stop();d.setAuto(-.25f,20.f);
 for(int i=0;i<100;i++){mock_advance_ms(10);d.update(.01f,est);}
 printf("reverse positive heading error: left=%d right=%d (clockwise needs left>right)\n",d.pwmLeft(),d.pwmRight());
}
