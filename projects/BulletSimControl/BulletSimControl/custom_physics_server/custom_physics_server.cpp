#include "custom_physics_server.h"
#include "SharedMemory/PhysicsServerExample.h"
#include "CommonInterfaces/CommonExampleInterface.h"


extern int gSharedMemoryKey;

struct Bullet2CommandProcessorCreationCustom : public CommandProcessorCreationInterface
{
	virtual class CommandProcessorInterface* createCommandProcessor()
	{
		PhysicsServerCommandProcessor* proc = new PhysicsServerCommandProcessor;
		return proc;
	}

	virtual void deleteCommandProcessor(CommandProcessorInterface* proc)
	{
		delete proc;
	}
};

static Bullet2CommandProcessorCreationCustom sBullet2CommandCreatorCustom;

CommonExampleInterface* PhysicsServerCreateFuncBulletCustom(struct CommonExampleOptions& options)
{
	options.m_commandProcessorCreation = &sBullet2CommandCreatorCustom;

	CommonExampleInterface* example = PhysicsServerCreateFuncInternal(options);
	return example;
}

B3_STANDALONE_EXAMPLE(PhysicsServerCreateFuncBulletCustom)