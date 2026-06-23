uint32 Start_cooperation
# Start_cooperation = 1  -- agvs下发开始组队指令
# Start_cooperation = 0  -- agvs未下发组队指令
string HeadCar_ip
# HeadCar_ip : 头车ip　　　　　　　　　已和agvs对接由agvs下发
string RearCar_ip
# RearCar_ip : 尾车ip　　　　　　　　　已和agvs对接由agvs下发
uint32 Reach_mission
# Reach_mission　= 1    -- 到达任务终点
# Reach_mission = 0    -- 未到达任务终点
uint32 End_cooperation
# End_cooperation = 1   --agvs下发删除协同组队指令
# End_cooperation = 0   --agvs未下发删除协同组队指令
uint32 MasterSlave_identity
# MasterSlave_identity = 1     --确定为主车
# MasterSlave_identity = 2     --确定为从车
