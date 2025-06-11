#ifndef CONTROL_HUB_PLUGIN_H
#define CONTROL_HUB_PLUGIN_H

#include "SharedMemory/plugins/b3PluginAPI.h"

#ifdef __cplusplus
extern "C"
{
#endif

	//the following 3 APIs are required
	B3_SHARED_API int initPlugin_ControlHub(struct b3PluginContext* context);
	B3_SHARED_API void exitPlugin_ControlHub(struct b3PluginContext* context);
	B3_SHARED_API int executePluginCommand_ControlHub(struct b3PluginContext* context, const struct b3PluginArguments* arguments);

	///
	enum ControlHubCommandEnum
	{
		JOINT_CONTROL = 0,
		TASK_CONTROL = 1,
		RESET_CONTROL = 2,
	};

	//all the APIs below are optional
	B3_SHARED_API int preTickPluginCallback_ControlHub(struct b3PluginContext* context);

#ifdef __cplusplus
};
#endif

#endif  //#define CONTROL_HUB_PLUGIN_H
