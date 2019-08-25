Header header 			#Timestamp, sequence id...
string requestName		#Name of this request, which says who and why this position should be reached

time request_created_at	#When was the position request initially created?

#Goal pose to reach
geometry_msgs/Pose pose
