对象	规则	示例
文件	全小写下划线，模块名.c/.h	sensor_manager.c
对外函数	模块_动词[_宾语]	sensor_manager_init()
模块内static函数	_动词[_宾语] 或带模块前缀	_baro_process()
模块内类的私有变量	_动词[_宾语] 或带模块前缀	_baro_status
类型	小写下划线 + _t/_e/_s 后缀	baro_ctx_t
全局/静态变量	s_（static）/ g_（全局，尽量避免）	s_baro
宏/枚举常量	全大写下划线	GPS_FIX_MIN_SATS
消息类型	消息名_t，与话题同名	Baro_Position_t