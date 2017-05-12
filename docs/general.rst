================================
The MAX31865 project
================================
	.. _fig_0101:
	.. figure:: images/0101.png

	MAX31865模块主要使用SPI接口与单片机进行通信，本次例程使用TPYBoard STM32F103RBT6 最小系统板和MAX31865模块进行通信。

	工程主要组成说明:

		docs/ -- MAX31865模块LIB库的使用说明
		image/ -- MAX31865模块实物图及连接指示图
		lib/ -- MAX31865模块的驱动文件和LIB库
		main.c -- 主程序
--------------------------------
使用说明
--------------------------------
	MAX31865 针脚指示图：
	.. _fig_0102:
	.. figure:: images/0201.png
	
	MAX31865 接线说明：
	.. _fig_0103:
	.. figure:: images/0202.png
	
	MAX31865 跳线说明：
	.. _fig_0104:
	.. figure:: images/0203.png
	
	MAX31865 LIB库使用说明：
	请参阅 docs/MAX31865LIB说明.pdf 文档
	地址：https://github.com/TPYBoard/MAX31865/blob/master/docs/MAX31865LIB说明.pdf

	扩展文件
	lib文件及使用说明下载地址：http://tpyboard.com/download/data/178.html