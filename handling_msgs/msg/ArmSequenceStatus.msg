Header header 			#Timestamp, sequence id...

uint16 sequenceId		#Sequence id which is currently executed
uint8 currentStep		#Which pose in the sequence of poses is currently the target pose?
uint8 maxSteps			#How many poses are stored in the current sequence?
float64 error			#Current error. E.g. error of all joint angles summed up reaching the current pose
bool finished			#Has the current sequence been executed?
