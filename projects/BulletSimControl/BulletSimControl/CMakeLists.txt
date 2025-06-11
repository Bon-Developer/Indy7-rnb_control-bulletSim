cmake_minimum_required(VERSION 3.10)
project(BulletSimControl)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)

add_definitions(-DBT_THREADSAFE=1 -DBT_USE_DOUBLE_PRECISION -DB3_USE_STANDALONE_EXAMPLE -DGLEW_STATIC -DB3_USE_MIDI -DSTATIC_LINK_VR_PLUGIN)

include_directories(
        ../bullet3/src
        ../bullet3/examples
        ../bullet3/examples/ThirdPartyLibs
        ../bullet3/examples/ThirdPartyLibs/glad
        ../bullet3/examples/ThirdPartyLibs/Wavefront
        ../../../3rd_party
        )

link_directories(../bullet3/build_cmake/src/BulletSoftBody
        ../bullet3/build_cmake/src/BulletInverseDynamics
        ../bullet3/build_cmake/Extras/InverseDynamics
        ../bullet3/build_cmake/src/BulletDynamics
        ../bullet3/build_cmake/src/BulletCollision
        ../bullet3/build_cmake/src/LinearMath
        ../bullet3/build_cmake/examples/OpenGLWindow
        ../bullet3/build_cmake/src/Bullet3Common
        ../bullet3/build_cmake/examples/ThirdPartyLibs/BussIK)

add_executable(BulletSimControl
        ../bullet3/Extras/Serialize/BulletFileLoader/bChunk.cpp
        ../bullet3/Extras/Serialize/BulletFileLoader/bDNA.cpp
        ../bullet3/Extras/Serialize/BulletFileLoader/bFile.cpp
        ../bullet3/Extras/Serialize/BulletFileLoader/btBulletFile.cpp
        ../bullet3/Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp
        ../bullet3/Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.cpp
        ../bullet3/Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp
        ../bullet3/examples/ExampleBrowser/CollisionShape2TriangleMesh.cpp
        ../bullet3/examples/ExampleBrowser/GL_ShapeDrawer.cpp
        ../bullet3/examples/ExampleBrowser/OpenGLGuiHelper.cpp
        ../bullet3/examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
        ../bullet3/examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
        ../bullet3/examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
        ../bullet3/examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
        ../bullet3/examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp
        ../bullet3/examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp
        ../bullet3/examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp
        ../bullet3/examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
        ../bullet3/examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp
        ../bullet3/examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp
        ../bullet3/examples/Importers/ImportURDFDemo/URDF2Bullet.cpp
        ../bullet3/examples/Importers/ImportURDFDemo/UrdfParser.cpp
        ../bullet3/examples/Importers/ImportURDFDemo/urdfStringSplit.cpp
        ../bullet3/examples/MultiThreading/b3ThreadSupportInterface.cpp
        ../bullet3/examples/MultiThreading/b3PosixThreadSupport.cpp
        ../bullet3/src/LinearMath/TaskScheduler/btThreadSupportPosix.cpp
        ../bullet3/examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.cpp
        ../bullet3/examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp
        ../bullet3/examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp
        ../bullet3/examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp
        ../bullet3/examples/SharedMemory/plugins/vrSyncPlugin/vrSyncPlugin.cpp
        ../bullet3/examples/SharedMemory/b3PluginManager.cpp
        ../bullet3/examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp
        ../bullet3/examples/SharedMemory/GraphicsClientExample.cpp
        ../bullet3/examples/SharedMemory/GraphicsServerExample.cpp
        ../bullet3/examples/SharedMemory/IKTrajectoryHelper.cpp
        ../bullet3/examples/SharedMemory/InProcessMemory.cpp
        ../bullet3/examples/SharedMemory/PhysicsClient.cpp
        ../bullet3/examples/SharedMemory/PhysicsClientC_API.cpp
        ../bullet3/examples/SharedMemory/PhysicsClientExample.cpp
        ../bullet3/examples/SharedMemory/PhysicsClientSharedMemory.cpp
        ../bullet3/examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp
        ../bullet3/examples/SharedMemory/PhysicsClientSharedMemory2.cpp
        ../bullet3/examples/SharedMemory/PhysicsClientSharedMemory2_C_API.cpp
        ../bullet3/examples/SharedMemory/PhysicsDirect.cpp
        ../bullet3/examples/SharedMemory/PhysicsDirectC_API.cpp
        ../bullet3/examples/SharedMemory/PhysicsLoopBack.cpp
        ../bullet3/examples/SharedMemory/PhysicsLoopBackC_API.cpp
        ../bullet3/examples/SharedMemory/PhysicsServer.cpp
        ../bullet3/examples/SharedMemory/PhysicsServerExample.cpp
        ../bullet3/examples/SharedMemory/PhysicsServerSharedMemory.cpp
        ../bullet3/examples/SharedMemory/PosixSharedMemory.cpp
        ../bullet3/examples/SharedMemory/RemoteGUIHelper.cpp
        ../bullet3/examples/SharedMemory/SharedMemoryCommandProcessor.cpp
        ../bullet3/examples/ThirdPartyLibs/glad/gl.c
        ../bullet3/examples/ThirdPartyLibs/midi/RtMidi.cpp
        ../bullet3/examples/ThirdPartyLibs/stb_image/stb_image.cpp
        ../bullet3/examples/ThirdPartyLibs/stb_image/stb_image_write.cpp
        ../bullet3/examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp
        ../bullet3/examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
        ../bullet3/examples/TinyRenderer/geometry.cpp
        ../bullet3/examples/TinyRenderer/model.cpp
        ../bullet3/examples/TinyRenderer/our_gl.cpp
        ../bullet3/examples/TinyRenderer/tgaimage.cpp
        ../bullet3/examples/TinyRenderer/TinyRenderer.cpp
        ../bullet3/examples/Utils/b3Clock.cpp
        ../bullet3/examples/Utils/b3ResourcePath.cpp
        ../bullet3/examples/Utils/ChromeTraceUtil.cpp
        ../bullet3/examples/Utils/RobotLoggingUtil.cpp
        control_hub/control_algorithm_default.cpp
        control_hub/control_hub.cpp
        control_hub/controller_ui.cpp
        control_hub/gason.cpp
        control_hub/trajectory_interface.cpp
        control_hub_plugin/ControlHubPlugin.cpp
        custom_physics_server/custom_command_processor.cpp
        custom_physics_server/custom_physics_server.cpp
        bullet_sim_main.cpp)

target_link_libraries(BulletSimControl
        BulletSoftBody BulletInverseDynamics BulletInverseDynamicsUtils
        BulletDynamics BulletCollision LinearMath OpenGLWindow Bullet3Common BussIK
        ${CMAKE_DL_LIBS}
        )