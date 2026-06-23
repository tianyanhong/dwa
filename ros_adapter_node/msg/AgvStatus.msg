string carName
string currentStation
float32 batteryPercent
# 0: 未充电 1: 充电中 2: 充电异常
uint8 chargingState
# 0:手动 1:调试  2:自动
uint8 ctrlMode
# 0:停止  1:预备态 2:运行态  3:暂停态
uint8 ctrlStatus
bool hasRoute
# true有货，false无货
bool cargoState
bool mapNotMatch
bool onRouteEnd
