bool direct            #true--车头朝前 false--成为朝前
geometry_msgs/Pose2D start_pt    #路线起点
geometry_msgs/Pose2D end_pt      #路线终点
float64 radius                  #转弯半径  半径为0时认为是直线
geometry_msgs/Point center      #圆弧的圆心
geometry_msgs/Point arc_midpoint      #圆弧的中点,用于区分劣弧和优弧，以及标准半圆时的情况
bool startIsTaskPoint                 #1--任务点， 0--经过点
int64 start_id
bool endIsTaskPoint
int64 end_id