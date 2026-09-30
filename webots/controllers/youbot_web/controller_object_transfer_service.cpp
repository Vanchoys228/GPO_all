#include "controller_object_transfer_service.h"
#include <stdio.h>
#include <string.h>
namespace {
constexpr double kTimeout=30.0;
constexpr double kNavigationTimeout=180.0;
int progress(ControllerObjectTransferStage s){static const int values[]={0,5,15,25,35,45,60,78,88,95};return s>=CONTROLLER_TRANSFER_STAGE_NONE&&s<=CONTROLLER_TRANSFER_RETURNING_ARM?values[s]:0;}
void enter(ControllerObjectTransferState *s,ControllerObjectTransferStage stage,double now){s->stage=stage;s->stage_started_at=now;s->progress=progress(stage);}
void fail(ControllerObjectTransferState *s,const char *error){s->status=CONTROLLER_TRANSFER_FAILED;s->stage=CONTROLLER_TRANSFER_STAGE_NONE;snprintf(s->error_code,sizeof(s->error_code),"%s",error);}
}
void controller_object_transfer_service_init(ControllerObjectTransferService *service){if(service)memset(service,0,sizeof(*service));}
int controller_object_transfer_service_start(ControllerObjectTransferService *service,const char *mission_id,const char *object_id,double x,double y,double now){
  if(!service||!mission_id||!object_id||service->state.status==CONTROLLER_TRANSFER_RUNNING)return 0;
  controller_object_transfer_service_init(service);snprintf(service->state.mission_id,sizeof(service->state.mission_id),"%s",mission_id);snprintf(service->state.object_id,sizeof(service->state.object_id),"%s",object_id);service->state.destination_x=x;service->state.destination_y=y;service->state.status=CONTROLLER_TRANSFER_RUNNING;enter(&service->state,CONTROLLER_TRANSFER_APPROACHING_OBJECT,now);return 1;
}
void controller_object_transfer_service_step(ControllerObjectTransferService *service,const ControllerObjectTransferInput *input,double now,ControllerObjectTransferOutput *output){
  if(!service||!input||!output)return;memset(output,0,sizeof(*output));ControllerObjectTransferState *s=&service->state;if(s->status!=CONTROLLER_TRANSFER_RUNNING)return;
  if(input->cancel_requested){output->stop_base=1;if(s->attached||input->attached){if(!input->release_safe){s->attached=1;s->status=CONTROLLER_TRANSFER_HOLDING_FOR_RECOVERY;snprintf(s->error_code,sizeof(s->error_code),"unsafe_release");return;}output->detach_object=1;}s->status=CONTROLLER_TRANSFER_CANCELLED;s->stage=CONTROLLER_TRANSFER_STAGE_NONE;return;}
  const double elapsed=now-s->stage_started_at;
  switch(s->stage){
    case CONTROLLER_TRANSFER_APPROACHING_OBJECT:output->request_navigation=1;if(input->navigation_reached)enter(s,CONTROLLER_TRANSFER_ALIGNING,now);else if(elapsed>kNavigationTimeout)fail(s,"navigation_failed");break;
    case CONTROLLER_TRANSFER_ALIGNING:if(input->base_aligned)enter(s,CONTROLLER_TRANSFER_LOWERING_ARM,now);else if(elapsed>kTimeout&&s->alignment_retries++==0)s->stage_started_at=now;else if(elapsed>kTimeout)fail(s,"alignment_failed");break;
    case CONTROLLER_TRANSFER_LOWERING_ARM:output->request_arm_pose=1;if(input->arm_reached)enter(s,CONTROLLER_TRANSFER_GRASPING,now);else if(elapsed>kTimeout)fail(s,"arm_timeout");break;
    case CONTROLLER_TRANSFER_GRASPING:output->close_gripper=1;if(input->grasp_valid){output->attach_object=1;s->attached=1;s->gripper_closed=1;enter(s,CONTROLLER_TRANSFER_LIFTING,now);}else if(elapsed>kTimeout)fail(s,"grasp_failed");break;
    case CONTROLLER_TRANSFER_LIFTING:output->request_arm_pose=1;if(input->arm_reached)enter(s,CONTROLLER_TRANSFER_TRANSPORTING,now);else if(elapsed>kTimeout)fail(s,"arm_timeout");break;
    case CONTROLLER_TRANSFER_TRANSPORTING:output->request_navigation=1;if(input->navigation_reached)enter(s,CONTROLLER_TRANSFER_PLACING,now);else if(elapsed>kNavigationTimeout)fail(s,"navigation_failed");break;
    case CONTROLLER_TRANSFER_PLACING:output->request_arm_pose=1;if(input->arm_reached&&input->release_safe)enter(s,CONTROLLER_TRANSFER_RELEASING,now);else if(elapsed>kTimeout){s->status=CONTROLLER_TRANSFER_HOLDING_FOR_RECOVERY;snprintf(s->error_code,sizeof(s->error_code),"unsafe_release");}break;
    case CONTROLLER_TRANSFER_RELEASING:output->detach_object=1;s->attached=0;s->gripper_closed=0;enter(s,CONTROLLER_TRANSFER_RETURNING_ARM,now);break;
    case CONTROLLER_TRANSFER_RETURNING_ARM:output->request_arm_pose=1;if(input->arm_reached){s->status=CONTROLLER_TRANSFER_COMPLETED;s->stage=CONTROLLER_TRANSFER_STAGE_NONE;s->progress=100;}else if(elapsed>kTimeout)fail(s,"arm_timeout");break;
    default:break;
  }
}
int controller_object_transfer_service_resume(ControllerObjectTransferService *service,double now){if(!service||service->state.status!=CONTROLLER_TRANSFER_HOLDING_FOR_RECOVERY)return 0;service->state.status=CONTROLLER_TRANSFER_RUNNING;service->state.error_code[0]='\0';enter(&service->state,CONTROLLER_TRANSFER_TRANSPORTING,now);return 1;}
