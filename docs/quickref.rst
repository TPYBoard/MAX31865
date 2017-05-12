==============================
**MAX31865.LIB 文件使用说明**
==============================
---------------------------------------
**一、设置MAX31865 的工作模式**
---------------------------------------
	void maxim_31865_init(max31865_configuration* configuration);
	其中参数就一个，为一个结构体：
	
	typedef struct
	{
		uint8_t Vbias; 基准电压打开
		uint8_t Conversion_mode; 转换模式
		uint8_t Rtd_wire; PT100 接线模式
		uint8_t Filter; 滤波模式
	}
	Vbias 的值可设置为ON 或OFF
	Conversion_mode 的值可设定为Auto_Conversion 或One_Shot_Conversion 即自动转换或单次转换
	Rtd_wire 的值可以设置为RTD_2wire、RTD_3wire 或RTD_4wire 即2 线制、3 线制或4线制
	Filter 的值可以设置为Filter_50Hz 或Filter_60Hz 即滤除电源干扰，可设置为50Hz 或60Hz 由于国内都是50Hz 电源，一般就设置Filter_50Hz 即可。
---------------------------------------
**二、设置MAX31865 阈值**
---------------------------------------
	void maxim_set_fault_threshold(float high_threshold, float low_threshold);
	参数里
	high_threshold 为高限阈值，一般PT100 设置为400，PT1000 设置为4000；
	low_threshold 为低限阈值，一般缺省，没有设置，留空。
---------------------------------------
**三、手动检测MAX31865 运行故障**
---------------------------------------
	uint8_t maxim_manual_fault_detection(void);
	该函数返回错误代码，为0 则没错误。
---------------------------------------
**四、清除MAX31865 的故障代码**
---------------------------------------
	void maxim_clear_fault_status(void);
	直接调用即可。
---------------------------------------
**五、取得rtd 电阻值**
---------------------------------------
	void maxim_get_rtd_value(uint8_t *uch_buff);
	参数为返回缓冲的指针。