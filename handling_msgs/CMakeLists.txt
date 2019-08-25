cmake_minimum_required(VERSION 2.8.3)
project(handling_msgs)

add_compile_options ("-Wall" "-std=c++11")

find_package(catkin REQUIRED
	COMPONENTS
	message_generation
	std_msgs
	actionlib_msgs
	geometry_msgs
	sensor_msgs
	moveit_msgs
	)


add_message_files(FILES
  Action.msg
  ActionTransform.msg
  ArmSequenceStatus.msg
  DetectedObjects.msg
  DetectedObject.msg
  PositionRequest.msg
  Object.msg
  SegmentedCloudVector.msg
  JoyControlArmWristReset.msg
  VisualCloudUpdate.msg
  ObjectGrabbed.msg
)

add_service_files(FILES
	AbortCurrentSequence.srv
	ExecuteSequence.srv
	GetActionFromDatabase.srv
	GetObjectFromDatabase.srv
	GetTransformFromDatabase.srv
	GetCurrentPose.srv
	ChangeCloudVisualization.srv
	StartOneShotDetection.srv
	SavePCL.srv
	Trigger.srv
	MoveTurntable.srv
	SetScanParameters.srv
	SetCalibParameters.srv
	SetVector.srv
	SaveParameter.srv
	AddModel.srv
	GetModels.srv
	SetOutlierParameters.srv
	SetSmoothParameters.srv
	ScanScene.srv
	SetICPParameters.srv
	DeleteModel.srv
	SaveProfile.srv
	SetSource.srv
	GetFeatures.srv
	ReadFeatures.srv
	GetProfiles.srv
	LoadProfile.srv
	UpdateParameters.srv
	GetModelFile.srv
	GetObjectHandlingParam.srv
        ToggleGUIExecuteCancelBtn.srv
	DeleteProfile.srv
	GetObjectMeshPaths.srv
)
add_action_files(FILES
	PoseGoal.action
	GoapMsg.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    moveit_msgs
)

catkin_package(
	CATKIN_DEPENDS
		message_runtime
	    std_msgs
	    actionlib_msgs
	    geometry_msgs
	    sensor_msgs
	    moveit_msgs
)
