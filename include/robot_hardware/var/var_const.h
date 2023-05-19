#ifndef __COMWISE__VAR_CONST__H__
#define __COMWISE__VAR_CONST__H__

#include <cstdint>
#include <string>

namespace comwise {

static std::string kEmptyStr = "";

//! error string
static const char* kRetBoolStr[] = {"error", "success"};

//! topic key
static constexpr char kTfTopic[]            = "tf_topic";
static constexpr char kStatusTopic[]        = "status_topic";

static constexpr char kJoyStickTopic[]      = "/comwise/joy";
static constexpr char kMoveCmdTopic[]       = "/comwise/move_cmd";
static constexpr char kMoveFeedbackTopic[]  = "/comwise/move_feedback";
static constexpr char kMoveBrakeTopic[]     = "/comwise/move_brake";
static constexpr char kMoveOdomTopic[]      = "/comwise/odom";

//! service key
static constexpr char kApiLoginServiceName[]        = "/comwise/web/login";
static constexpr char kApiSetConfigServiceName[]    = "/comwise/web/set_config";
static constexpr char kApiGetConfigServiceName[]    = "/comwise/web/get_config";
static constexpr char kApiGetComponentServiceName[] = "/comwise/web/get_component";
static constexpr char kApiGetTemplateServiceName[]  = "/comwise/web/get_template";
static constexpr char kApiGetCommentServiceName[]   = "/comwise/web/get_comment";
static constexpr char kApiGetStatusServiceName[]    = "/comwise/web/get_status";


} // namespace comwise

#endif // __COMWISE__VAR_CONST__H__
