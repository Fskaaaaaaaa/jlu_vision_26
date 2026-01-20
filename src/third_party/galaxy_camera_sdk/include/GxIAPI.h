/**
@File      GxIAPI.h
@Brief     the interface for the GxIAPI dll module. 
@Author    Software Department
@Date      2019-08-15
@Version   1.1.1908.9151
*/

#ifndef GX_GALAXY_H
#define GX_GALAXY_H


//////////////////////////////////////////////////////////////////////////
//	类型定义，以下类型都在标准C库头文件stdint.h中有定义，但是在微软的编译平台
//	VS2010之前的版本中都不包含此文件,所以在此需要重定义
//////////////////////////////////////////////////////////////////////////

#if defined(_WIN32)
	#ifndef _STDINT_H 
		#ifdef _MSC_VER // Microsoft compiler
			#if _MSC_VER < 1600
				typedef __int8            int8_t;
				typedef __int16           int16_t;
				typedef __int32           int32_t;
				typedef __int64           int64_t;
				typedef unsigned __int8   uint8_t;
				typedef unsigned __int16  uint16_t;
				typedef unsigned __int32  uint32_t;
				typedef unsigned __int64  uint64_t;
			#else
				// In Visual Studio 2010 is stdint.h already included
				#include <stdint.h>
			#endif
		#else
			// Not a Microsoft compiler
			#include <stdint.h>
		#endif
	#endif 
#else
	// Linux
	#include <stdint.h>
#endif


//------------------------------------------------------------------------------
//  操作系统平台定义
//------------------------------------------------------------------------------

#include <stddef.h>

#ifdef WIN32
	#ifndef _WIN32
		#define _WIN32
	#endif
#endif

#ifdef _WIN32
	#include <Windows.h>
	#define GX_DLLIMPORT   __declspec(dllimport)
	#define GX_DLLEXPORT   __declspec(dllexport)

	#define GX_STDC __stdcall
	#define GX_CDEC __cdecl

	#if defined(__cplusplus)
		#define GX_EXTC extern "C"
	#else
		#define GX_EXTC
	#endif
#else
	// remove the None #define conflicting with GenApi
	#undef None
	#if __GNUC__>=4
		#define GX_DLLIMPORT   __attribute__((visibility("default")))
		#define GX_DLLEXPORT   __attribute__((visibility("default")))

		#if defined(__i386__)
			#define GX_STDC __attribute__((stdcall))
			#define GX_CDEC __attribute__((cdecl))
		#else
			#define GX_STDC 
			#define GX_CDEC 
		#endif

		#if defined(__cplusplus)
			#define GX_EXTC extern "C"
		#else
			#define GX_EXTC
		#endif
	#else
		#error Unknown compiler
	#endif
#endif

#ifdef GX_GALAXY_DLL
	#define GX_DLLENTRY GX_EXTC GX_DLLEXPORT
#else
	#define GX_DLLENTRY GX_EXTC GX_DLLIMPORT
#endif


//------------------------------------------------------------------------------
//  错误码定义
//------------------------------------------------------------------------------
typedef enum GX_STATUS_LIST
{
	GX_STATUS_SUCCESS                =  0,           ///< 成功
	GX_STATUS_ERROR                  = -1,           ///< 不期望发生的未明确指明的内部错误
	GX_STATUS_NOT_FOUND_TL           = -2,           ///< 找不到TL库
	GX_STATUS_NOT_FOUND_DEVICE       = -3,           ///< 找不到设备
	GX_STATUS_OFFLINE                = -4,           ///< 当前设备为掉线状态
	GX_STATUS_INVALID_PARAMETER      = -5,           ///< 无效参数,一般是指针为NULL或输入的IP等参数格式无效
	GX_STATUS_INVALID_HANDLE         = -6,           ///< 无效句柄
	GX_STATUS_INVALID_CALL           = -7,           ///< 无效的接口调用,专指软件接口逻辑错误
	GX_STATUS_INVALID_ACCESS         = -8,           ///< 功能当前不可访问或设备访问模式错误
	GX_STATUS_NEED_MORE_BUFFER       = -9,           ///< 用户申请的buffer不足:读操作时用户输入buffersize小于实际需要
	GX_STATUS_ERROR_TYPE             = -10,          ///< 用户使用的FeatureID类型错误，比如整型接口使用了浮点型的功能码
	GX_STATUS_OUT_OF_RANGE           = -11,          ///< 用户写入的值越界
	GX_STATUS_NOT_IMPLEMENTED        = -12,          ///< 当前不支持的功能
	GX_STATUS_NOT_INIT_API           = -13,          ///< 没有调用初始化接口
	GX_STATUS_TIMEOUT                = -14,          ///< 超时错误
}GX_STATUS_LIST;
typedef int32_t GX_STATUS;

//------------------------------------------------------------------------------
//  帧状态码定义
//------------------------------------------------------------------------------
typedef enum GX_FRAME_STATUS_LIST
{
	GX_FRAME_STATUS_SUCCESS          = 0,     ///< 正常帧
	GX_FRAME_STATUS_INCOMPLETE       = -1,    ///< 残帧
}GX_FRAME_STATUS_LIST;
typedef  int32_t  GX_FRAME_STATUS;

//------------------------------------------------------------------------------
//  设备类型码定义
//------------------------------------------------------------------------------
typedef enum GX_DEVICE_CLASS_LIST
{
	GX_DEVICE_CLASS_UNKNOWN = 0,     ///< 未知设备类型
	GX_DEVICE_CLASS_USB2    = 1,     ///< USB2.0设备
	GX_DEVICE_CLASS_GEV     = 2,     ///< 千兆网设备
	GX_DEVICE_CLASS_U3V     = 3,     ///< USB3.0设备
}GX_DEVICE_CLASS_LIST;
typedef  int32_t GX_DEVICE_CLASS;

//------------------------------------------------------------------------------
//  功能码类型定义
//------------------------------------------------------------------------------
typedef enum GX_FEATURE_TYPE
{
	GX_FEATURE_INT				   =0x10000000,  ///< 整型数
	GX_FEATURE_FLOAT               =0X20000000,  ///< 浮点数
	GX_FEATURE_ENUM				   =0x30000000,  ///< 枚举
	GX_FEATURE_BOOL				   =0x40000000,  ///< 布尔
	GX_FEATURE_STRING			   =0x50000000,  ///< 字符串
	GX_FEATURE_BUFFER			   =0x60000000,  ///< buffer
	GX_FEATURE_COMMAND			   =0x70000000,  ///< 命令
}GX_FEATURE_TYPE;

//------------------------------------------------------------------------------
//  功能码所属层级定义
//------------------------------------------------------------------------------
typedef enum GX_FEATURE_LEVEL
{
	GX_FEATURE_LEVEL_REMOTE_DEV	    =0x00000000,  ///< RemoteDevice层
	GX_FEATURE_LEVEL_TL				=0x01000000,  ///< TL层
	GX_FEATURE_LEVEL_IF             =0x02000000,  ///< Interface层	
	GX_FEATURE_LEVEL_DEV		    =0x03000000,  ///< Device层
	GX_FEATURE_LEVEL_DS			    =0x04000000,  ///< DataStream层
}GX_FEATURE_LEVEL;

//------------------------------------------------------------------------------
//  设备的访问方式
//------------------------------------------------------------------------------
typedef enum GX_ACCESS_MODE
{
	GX_ACCESS_READONLY      =2,        ///< 只读方式
	GX_ACCESS_CONTROL       =3,        ///< 控制方式
	GX_ACCESS_EXCLUSIVE     =4,        ///< 独占方式
}GX_ACCESS_MODE;
typedef int32_t GX_ACCESS_MODE_CMD;

//------------------------------------------------------------------------------
//  当前设备的可访问方式
//------------------------------------------------------------------------------
typedef enum GX_ACCESS_STATUS
{
	GX_ACCESS_STATUS_UNKNOWN    = 0,   ///< 设备当前状态未知
	GX_ACCESS_STATUS_READWRITE  = 1,   ///< 设备当前可读可写
	GX_ACCESS_STATUS_READONLY   = 2,   ///< 设备当前只支持读
	GX_ACCESS_STATUS_NOACCESS   = 3,   ///< 设备当前既不支持读，又不支持写
}GX_ACCESS_STATUS;
typedef int32_t GX_ACCESS_STATUS_CMD;

//------------------------------------------------------------------------------
//  设备的打开方式
//------------------------------------------------------------------------------
typedef enum GX_OPEN_MODE
{
	GX_OPEN_SN              =0,        ///< 通过SN打开
	GX_OPEN_IP              =1,        ///< 通过IP打开
	GX_OPEN_MAC             =2,        ///< 通过MAC打开
	GX_OPEN_INDEX           =3,        ///< 通过Index打开
	GX_OPEN_USERID          =4,        ///< 通过用户自定义ID打开
}GX_OPEN_MODE;
typedef int32_t GX_OPEN_MODE_CMD;

//------------------------------------------------------------------------------
//  IP配置方式
//------------------------------------------------------------------------------
enum GX_IP_CONFIGURE_MODE_LIST
{
	GX_IP_CONFIGURE_DHCP       = 0x6,   ///< 启用DHCP，自动获取IP地址
	GX_IP_CONFIGURE_LLA        = 0x4,   ///< 使用LLA方式分配IP地址
	GX_IP_CONFIGURE_STATIC_IP  = 0x5,   ///< 设置静态IP地址
	GX_IP_CONFIGURE_DEFAULT    = 0x7,   ///< 使用默认方式配置IP地址。
};
typedef int32_t GX_IP_CONFIGURE_MODE;

typedef enum GX_FEATURE_ID
{
	//////////////////////////////////////////////////////////////////////////
	/// 远端设备层(Remote Device Feature)
	//////////////////////////////////////////////////////////////////////////

	//---------------DeviceInfomation Section--------------------------
	GX_STRING_DEVICE_VENDOR_NAME               = 0   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 厂商名称
	GX_STRING_DEVICE_MODEL_NAME                = 1   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备型号
	GX_STRING_DEVICE_FIRMWARE_VERSION          = 2   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备固件版本
	GX_STRING_DEVICE_VERSION                   = 3   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备版本
	GX_STRING_DEVICE_SERIAL_NUMBER             = 4   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备序列号
	GX_STRING_FACTORY_SETTING_VERSION          = 6   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 出厂参数版本
	GX_STRING_DEVICE_USERID                    = 7   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 用户自定义名称
	GX_INT_DEVICE_LINK_SELECTOR                = 8   | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备链路选择
	GX_ENUM_DEVICE_LINK_THROUGHPUT_LIMIT_MODE  = 9   | GX_FEATURE_ENUM   | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备带宽限制模式，参考GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ENTRY
	GX_INT_DEVICE_LINK_THROUGHPUT_LIMIT        = 10  | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备链路带宽限制
	GX_INT_DEVICE_LINK_CURRENT_THROUGHPUT      = 11  | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 当前设备采集带宽
    GX_COMMAND_DEVICE_RESET                    = 12  | GX_FEATURE_COMMAND| GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备复位
    GX_INT_TIMESTAMP_TICK_FREQUENCY            = 13  | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 时间戳频率
    GX_COMMAND_TIMESTAMP_LATCH                 = 14  | GX_FEATURE_COMMAND| GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 时间戳锁存 
    GX_COMMAND_TIMESTAMP_RESET                 = 15  | GX_FEATURE_COMMAND| GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 重置时间戳
    GX_COMMAND_TIMESTAMP_LATCH_RESET           = 16  | GX_FEATURE_COMMAND| GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 重置时间戳锁存
    GX_INT_TIMESTAMP_LATCH_VALUE               = 17  | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 时间戳锁存值


	//---------------ImageFormat Section--------------------------------
	GX_INT_SENSOR_WIDTH               = 1000 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 传感器宽度
	GX_INT_SENSOR_HEIGHT              = 1001 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 传感器高度
	GX_INT_WIDTH_MAX                  = 1002 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 最大宽度
	GX_INT_HEIGHT_MAX                 = 1003 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 最大高度
	GX_INT_OFFSET_X                   = 1004 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 水平偏移
	GX_INT_OFFSET_Y                   = 1005 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 垂直偏移
	GX_INT_WIDTH                      = 1006 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 图像宽度
	GX_INT_HEIGHT                     = 1007 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 图像高度
	GX_INT_BINNING_HORIZONTAL         = 1008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 水平像素Binning
	GX_INT_BINNING_VERTICAL           = 1009 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 垂直像素Binning
	GX_INT_DECIMATION_HORIZONTAL      = 1010 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 水平像素抽样
	GX_INT_DECIMATION_VERTICAL        = 1011 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 垂直像素抽样
	GX_ENUM_PIXEL_SIZE                = 1012 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 像素位深,参考GX_PIXEL_SIZE_ENTRY
	GX_ENUM_PIXEL_COLOR_FILTER        = 1013 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< Bayer格式,参考GX_PIXEL_COLOR_FILTER_ENTRY
	GX_ENUM_PIXEL_FORMAT              = 1014 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 像素格式,参考GX_PIXEL_FORMAT_ENTRY
	GX_BOOL_REVERSE_X                 = 1015 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 水平翻转
	GX_BOOL_REVERSE_Y                 = 1016 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 垂直翻转
	GX_ENUM_TEST_PATTERN              = 1017 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 测试图,参考GX_TEST_PATTERN_ENTRY
	GX_ENUM_TEST_PATTERN_GENERATOR_SELECTOR = 1018 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 测试图源选择，参考GX_TEST_PATTERN_GENERATOR_SELECTOR_ENTRY
    GX_INT_CENTER_WIDTH               = 1022 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 窗口宽度
    GX_INT_CENTER_HEIGHT              = 1023 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 窗口高度
    GX_ENUM_BINNING_HORIZONTAL_MODE   = 1024 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 水平像素Binning模式,参考GX_BINNING_HORIZONTAL_MODE_ENTRY
    GX_ENUM_BINNING_VERTICAL_MODE     = 1025 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 垂直像素Binning模式,参考GX_BINNING_VERTICAL_MODE_ENTRY

	//---------------TransportLayer Section-------------------------------
	GX_INT_PAYLOAD_SIZE                              = 2000 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 数据大小 
	GX_BOOL_GEV_CURRENT_IPCONFIGURATION_LLA          = 2001 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV, ///< LLA方式配置IP
	GX_BOOL_GEV_CURRENT_IPCONFIGURATION_DHCP         = 2002 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV, ///< DHCP方式配置IP
	GX_BOOL_GEV_CURRENT_IPCONFIGURATION_PERSISTENTIP = 2003 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 永久IP方式配置IP
	GX_INT_ESTIMATED_BANDWIDTH                       = 2004 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 预估带宽，单位Bps(Bytes per second)
	GX_INT_GEV_HEARTBEAT_TIMEOUT                     = 2005 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 心跳超时时间
	GX_INT_GEV_PACKETSIZE                            = 2006 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 流通道包长
	GX_INT_GEV_PACKETDELAY                           = 2007 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 流通道包间隔
	GX_INT_GEV_LINK_SPEED                            = 2008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 连接速度

	//---------------AcquisitionTrigger Section---------------------------
	GX_ENUM_ACQUISITION_MODE          = 3000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 采集模式,参考GX_ACQUISITION_MODE_ENTRY
	GX_COMMAND_ACQUISITION_START      = 3001 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 开始采集
	GX_COMMAND_ACQUISITION_STOP       = 3002 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 停止采集
	GX_INT_ACQUISITION_SPEED_LEVEL    = 3003 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 采集速度级别
	GX_INT_ACQUISITION_FRAME_COUNT    = 3004 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 多帧采集帧数
	GX_ENUM_TRIGGER_MODE              = 3005 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 触发模式,参考GX_TRIGGER_MODE_ENTRY
	GX_COMMAND_TRIGGER_SOFTWARE       = 3006 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 软触发
	GX_ENUM_TRIGGER_ACTIVATION        = 3007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 触发极性,参考GX_TRIGGER_ACTIVATION_ENTRY
	GX_ENUM_TRIGGER_SWITCH            = 3008 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 外触发开关,参考GX_TRIGGER_SWITCH_ENTRY
	GX_FLOAT_EXPOSURE_TIME            = 3009 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 曝光时间
	GX_ENUM_EXPOSURE_AUTO             = 3010 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动曝光,参考GX_EXPOSURE_AUTO_ENTRY
	GX_FLOAT_TRIGGER_FILTER_RAISING   = 3011 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 上升沿触发滤波
	GX_FLOAT_TRIGGER_FILTER_FALLING   = 3012 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 下降沿触发滤波
	GX_ENUM_TRIGGER_SOURCE            = 3013 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 触发源,参考GX_TRIGGER_SOURCE_ENTRY
	GX_ENUM_EXPOSURE_MODE             = 3014 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 曝光模式，参考GX_EXPOSURE_MODE_ENTRY
	GX_ENUM_TRIGGER_SELECTOR          = 3015 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 触发类型选择，参考GX_TRIGGER_SELECTOR_ENTRY
	GX_FLOAT_TRIGGER_DELAY            = 3016 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 触发延迟
	GX_ENUM_TRANSFER_CONTROL_MODE     = 3017 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 传输控制模式，参考GX_TRANSFER_CONTROL_MODE_ENTRY
	GX_ENUM_TRANSFER_OPERATION_MODE   = 3018 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 传输操作模式，参考GX_TRANSFER_OPERATION_MODE_ENTRY
	GX_COMMAND_TRANSFER_START         = 3019 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 开始传输
	GX_INT_TRANSFER_BLOCK_COUNT       = 3020 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 传输帧数。当GX_ENUM_TRANSFER_OPERATION_MODE设置为GX_ENUM_TRANSFER_OPERATION_MODE_MULTIBLOCK模式的时候此功能被激活
	GX_BOOL_FRAMESTORE_COVER_ACTIVE   = 3021 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 帧存覆盖使能
	GX_ENUM_ACQUISITION_FRAME_RATE_MODE     = 3022 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 采集帧率调节模式，参考GX_ACQUISITION_FRAME_RATE_MODE_ENTRY
	GX_FLOAT_ACQUISITION_FRAME_RATE         = 3023 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 采集帧率
	GX_FLOAT_CURRENT_ACQUISITION_FRAME_RATE = 3024 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 当前采集帧率
	GX_ENUM_FIXED_PATTERN_NOISE_CORRECT_MODE = 3025  | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,///< 模板噪声校正，参考GX_FIXED_PATTERN_NOISE_CORRECT_MODE
    GX_INT_ACQUISITION_BURST_FRAME_COUNT    = 3030 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 高速连拍帧数
    GX_ENUM_ACQUISITION_STATUS_SELECTOR     = 3031 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 采集状态选择,参考GX_ACQUISITION_STATUS_SELECTOR_ENTRY
    GX_BOOL_ACQUISITION_STATUS              = 3032 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 采集状态
    GX_FLOAT_EXPOSURE_DELAY                 = 30300| GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 曝光延迟

	//----------------DigitalIO Section----------------------------------
	GX_ENUM_USER_OUTPUT_SELECTOR      = 4000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 用户自定义输出选择,参考GX_USER_OUTPUT_SELECTOR_ENTRY
	GX_BOOL_USER_OUTPUT_VALUE         = 4001 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 用户自定义输出值
	GX_ENUM_USER_OUTPUT_MODE          = 4002 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 用户IO输出模式,参考GX_USER_OUTPUT_MODE_ENTRY
	GX_ENUM_STROBE_SWITCH             = 4003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 闪光灯开关,参考GX_STROBE_SWITCH_ENTRY
	GX_ENUM_LINE_SELECTOR             = 4004 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 引脚选择,参考GX_LINE_SELECTOR_ENTRY
	GX_ENUM_LINE_MODE                 = 4005 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 引脚方向,参考GX_LINE_MODE_ENTRY
	GX_BOOL_LINE_INVERTER             = 4006 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 引脚电平反转
	GX_ENUM_LINE_SOURCE               = 4007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 引脚输出源,参考GX_LINE_SOURCE_ENTRY
	GX_BOOL_LINE_STATUS               = 4008 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 引脚状态
	GX_INT_LINE_STATUS_ALL            = 4009 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 所有引脚的状态

	//----------------AnalogControls Section----------------------------
	GX_ENUM_GAIN_AUTO                 = 5000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 自动增益,参考GX_GAIN_AUTO_ENTRY
	GX_ENUM_GAIN_SELECTOR             = 5001 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 增益通道选择,参考GX_GAIN_SELECTOR_ENTRY	
	GX_ENUM_BLACKLEVEL_AUTO           = 5003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 自动黑电平,参考GX_BLACKLEVEL_AUTO_ENTRY
	GX_ENUM_BLACKLEVEL_SELECTOR       = 5004 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 黑电平通道选择,参考GX_BLACKLEVEL_SELECTOR_ENTRY	
	GX_ENUM_BALANCE_WHITE_AUTO        = 5006 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 自动白平衡,参考GX_BALANCE_WHITE_AUTO_ENTRY
	GX_ENUM_BALANCE_RATIO_SELECTOR    = 5007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 白平衡通道选择,参考GX_BALANCE_RATIO_SELECTOR_ENTRY
	GX_FLOAT_BALANCE_RATIO            = 5008 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 白平衡系数
	GX_ENUM_COLOR_CORRECT             = 5009 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 颜色校正,参考GX_COLOR_CORRECT_ENTRY
	GX_ENUM_DEAD_PIXEL_CORRECT        = 5010 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 坏点校正,参考GX_DEAD_PIXEL_CORRECT_ENTRY
	GX_FLOAT_GAIN                     = 5011 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 增益
	GX_FLOAT_BLACKLEVEL               = 5012 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 黑电平
    GX_BOOL_GAMMA_ENABLE              = 5013 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< Gamma使能
    GX_ENUM_GAMMA_MODE                = 5014 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< Gamma模式,参考GX_GAMMA_MODE_ENTRY
    GX_FLOAT_GAMMA                    = 5015 | GX_FEATURE_FLOAT| GX_FEATURE_LEVEL_REMOTE_DEV,   ///< Gamma
    GX_INT_DIGITAL_SHIFT              = 5016 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 数字移位

	//---------------CustomFeature Section-------------------------
	GX_INT_ADC_LEVEL                  = 6000 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< AD转换级别
	GX_INT_H_BLANKING                 = 6001 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 水平消隐
	GX_INT_V_BLANKING                 = 6002 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 垂直消隐
	GX_STRING_USER_PASSWORD           = 6003 | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 用户加密区密码
	GX_STRING_VERIFY_PASSWORD         = 6004 | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 用户加密区校验密码
	GX_BUFFER_USER_DATA               = 6005 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 用户加密区内容
	GX_INT_GRAY_VALUE                 = 6006 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 期望灰度值
	GX_ENUM_AA_LIGHT_ENVIRONMENT      = 6007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 自动曝光、自动增益，光照环境类型,参考GX_AA_LIGHT_ENVIRMENT_ENTRY
	GX_INT_AAROI_OFFSETX              = 6008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动调节感兴趣区域X坐标
	GX_INT_AAROI_OFFSETY              = 6009 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动调节感兴趣区域Y坐标
	GX_INT_AAROI_WIDTH                = 6010 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动调节感兴趣区域宽度
	GX_INT_AAROI_HEIGHT               = 6011 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动调节感兴趣区域高度
	GX_FLOAT_AUTO_GAIN_MIN            = 6012 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 自动增益最小值
	GX_FLOAT_AUTO_GAIN_MAX            = 6013 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 自动增益最大值
	GX_FLOAT_AUTO_EXPOSURE_TIME_MIN   = 6014 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 自动曝光最小值
	GX_FLOAT_AUTO_EXPOSURE_TIME_MAX   = 6015 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 自动曝光最大值
	GX_BUFFER_FRAME_INFORMATION       = 6016 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 图像帧信息
	GX_INT_CONTRAST_PARAM             = 6017 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 对比度参数
	GX_FLOAT_GAMMA_PARAM              = 6018 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 伽马参数
	GX_INT_COLOR_CORRECTION_PARAM     = 6019 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 颜色校正系数
	GX_ENUM_IMAGE_GRAY_RAISE_SWITCH   = 6020 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 图像亮度拉伸开关,参考GX_IMAGE_GRAY_RAISE_SWITCH_ENTRY
	GX_ENUM_AWB_LAMP_HOUSE            = 6021 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 自动白平衡光源,参考GX_AWB_LAMP_HOUSE_ENTRY
	GX_INT_AWBROI_OFFSETX             = 6022 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动白平衡感兴趣区域X坐标
	GX_INT_AWBROI_OFFSETY             = 6023 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动白平衡感兴趣区域Y坐标
	GX_INT_AWBROI_WIDTH               = 6024 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动白平衡感兴趣区域宽度
	GX_INT_AWBROI_HEIGHT              = 6025 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动白平衡感兴趣区域高度
	GX_ENUM_SHARPNESS_MODE            = 6026 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 锐化模式,参考GX_SHARPNESS_MODE_ENTRY
	GX_FLOAT_SHARPNESS                = 6027 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 锐度
	GX_ENUM_USER_DATA_FILED_SELECTOR  = 6028 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 用户选择Flash数据区域，区域选择参考GX_USER_DATA_FILED_SELECTOR_ENTRY
	GX_BUFFER_USER_DATA_FILED_VALUE   = 6029 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 用户区内容
	GX_ENUM_FLAT_FIELD_CORRECTION     = 6030 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 平场校正开关,参考GX_FLAT_FIELD_CORRECTION_ENTRY

	//---------------UserSetControl Section-------------------------
	GX_ENUM_USER_SET_SELECTOR         = 7000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 参数组选择,参考GX_USER_SET_SELECTOR_ENTRY
	GX_COMMAND_USER_SET_LOAD          = 7001 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 加载参数组
	GX_COMMAND_USER_SET_SAVE          = 7002 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 保存参数组
	GX_ENUM_USER_SET_DEFAULT          = 7003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 启动参数组,参考GX_USER_SET_DEFAULT_ENTRY

	//---------------Event Section-------------------------
	GX_ENUM_EVENT_SELECTOR             = 8000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 事件源选择,参考GX_EVENT_SELECTOR_ENTRY
	GX_ENUM_EVENT_NOTIFICATION         = 8001 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 事件使能,参考GX_EVENT_NOTIFICATION_ENTRY
	GX_INT_EVENT_EXPOSUREEND           = 8002 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 曝光结束事件ID
	GX_INT_EVENT_EXPOSUREEND_TIMESTAMP = 8003 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 曝光结束事件时间戳
	GX_INT_EVENT_EXPOSUREEND_FRAMEID   = 8004 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 曝光结束事件帧ID
	GX_INT_EVENT_BLOCK_DISCARD         = 8005 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 数据块丢失事件ID
	GX_INT_EVENT_BLOCK_DISCARD_TIMESTAMP = 8006 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 数据块丢失事件时间戳
	GX_INT_EVENT_OVERRUN                 = 8007 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 事件队列溢出事件ID
	GX_INT_EVENT_OVERRUN_TIMESTAMP       = 8008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 事件队列溢出事件时间戳
	GX_INT_EVENT_FRAMESTART_OVERTRIGGER  = 8009 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 触发信号被屏蔽事件ID
	GX_INT_EVENT_FRAMESTART_OVERTRIGGER_TIMESTAMP = 8010 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 触发信号被屏蔽事件时间戳
	GX_INT_EVENT_BLOCK_NOT_EMPTY                  = 8011 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 帧存不为空事件ID
	GX_INT_EVENT_BLOCK_NOT_EMPTY_TIMESTAMP        = 8012 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 帧存不为空事件时间戳
	GX_INT_EVENT_INTERNAL_ERROR                   = 8013 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 内部错误事件ID
	GX_INT_EVENT_INTERNAL_ERROR_TIMESTAMP         = 8014 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 内部错误事件时间戳
	GX_INT_EVENT_FRAMEBURSTSTART_OVERTRIGGER      = 8015 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 多帧触发屏蔽事件ID
	GX_INT_EVENT_FRAMEBURSTSTART_OVERTRIGGER_FRAMEID      = 8016 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 多帧触发屏蔽事件帧ID
	GX_INT_EVENT_FRAMEBURSTSTART_OVERTRIGGER_TIMESTAMP    = 8017 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 多帧触发屏蔽事件时间戳
	GX_INT_EVENT_FRAMESTART_WAIT                          = 8018 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 帧等待事件ID
	GX_INT_EVENT_FRAMESTART_WAIT_TIMESTAMP                = 8019 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 帧等待事件时间戳
	GX_INT_EVENT_FRAMEBURSTSTART_WAIT                     = 8020 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 多帧等待事件ID
	GX_INT_EVENT_FRAMEBURSTSTART_WAIT_TIMESTAMP           = 8021 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 多帧等待事件时间戳
	GX_INT_EVENT_BLOCK_DISCARD_FRAMEID                    = 8022 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 数据块丢失事件帧ID
	GX_INT_EVENT_FRAMESTART_OVERTRIGGER_FRAMEID           = 8023 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 触发信号被屏蔽事件帧ID
	GX_INT_EVENT_BLOCK_NOT_EMPTY_FRAMEID                  = 8024 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 帧存不为空事件帧ID
	GX_INT_EVENT_FRAMESTART_WAIT_FRAMEID                  = 8025 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 帧等待事件帧ID
	GX_INT_EVENT_FRAMEBURSTSTART_WAIT_FRAMEID             = 8026 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 多帧等待事件帧ID

	//---------------LUT Section-------------------------
	GX_ENUM_LUT_SELECTOR             = 9000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 查找表选择,参考GX_LUT_SELECTOR_ENTRY
	GX_BUFFER_LUT_VALUEALL           = 9001 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 查找表内容
    GX_BOOL_LUT_ENABLE               = 9002 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 查找表使能
	GX_INT_LUT_INDEX                 = 9003 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 查找表索引
	GX_INT_LUT_VALUE                 = 9004 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 查找表值

	//---------------ChunkData Section-------------------------
	GX_BOOL_CHUNKMODE_ACTIVE         = 10001 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 帧信息使能
	GX_ENUM_CHUNK_SELECTOR           = 10002 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 帧信息项选择，参考GX_CHUNK_SELECTOR_ENTRY
	GX_BOOL_CHUNK_ENABLE             = 10003 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 单项帧信息使能

    //---------------Color Transformation Control-------------------------
	GX_ENUM_COLOR_TRANSFORMATION_MODE       = 11000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 颜色转换模式，参考GX_COLOR_TRANSFORMATION_MODE_ENTRY
    GX_BOOL_COLOR_TRANSFORMATION_ENABLE     = 11001 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 颜色转换使能
    GX_ENUM_COLOR_TRANSFORMATION_VALUE_SELECTOR = 11002 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,///< 颜色转换矩阵元素选择，参考GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_ENTRY
    GX_FLOAT_COLOR_TRANSFORMATION_VALUE     = 11003 | GX_FEATURE_FLOAT| GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 颜色转换矩阵元素
	
	//---------------CounterAndTimerControl Section-------------------------
	GX_ENUM_TIMER_SELECTOR                  = 12000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 计时器选择，参考GX_TIMER_SELECTOR_ENTRY
	GX_FLOAT_TIMER_DURATION                 = 12001 | GX_FEATURE_FLOAT| GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 计时器持续时间
	GX_FLOAT_TIMER_DELAY                    = 12002 | GX_FEATURE_FLOAT| GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 计时器延迟时间
	GX_ENUM_TIMER_TRIGGER_SOURCE            = 12003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 计时器触发源，参考GX_TIMER_TRIGGER_SOURCE_ENTRY
	GX_ENUM_COUNTER_SELECTOR                = 12004 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 计数器选择，参考GX_COUNTER_SELECTOR_ENTRY
	GX_ENUM_COUNTER_EVENT_SOURCE            = 12005 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 计数器事件触发源，参考GX_COUNTER_EVENT_SOURCE_ENTRY
	GX_ENUM_COUNTER_RESET_SOURCE            = 12006 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 计数器复位源，参考GX_COUNTER_RESET_SOURCE_ENTRY
	GX_ENUM_COUNTER_RESET_ACTIVATION        = 12007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 计数器复位信号极性，参考GX_COUNTER_RESET_ACTIVATION_ENTRY
	GX_COMMAND_COUNTER_RESET                = 12008 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 计数器复位         

	//---------------RemoveParameterLimitControl Section-------------------------
	GX_ENUM_REMOVE_PARAMETER_LIMIT          = 13000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 取消参数范围限制, 参考GX_REMOVE_PARAMETER_LIMIT_ENTRY

	//////////////////////////////////////////////////////////////////////////
	/// 本地设备层(Device Feature)
	//////////////////////////////////////////////////////////////////////////
	GX_DEV_INT_COMMAND_TIMEOUT     = 0 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DEV, ///< 命令超时
	GX_DEV_INT_COMMAND_RETRY_COUNT = 1 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DEV, ///< 命令重试次数

	//////////////////////////////////////////////////////////////////////////
	/// 流层(DataStream Feature)
	//////////////////////////////////////////////////////////////////////////
	GX_DS_INT_ANNOUNCED_BUFFER_COUNT          = 0 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 声明的Buffer个数
	GX_DS_INT_DELIVERED_FRAME_COUNT           = 1 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 接收帧个数(包括残帧)
	GX_DS_INT_LOST_FRAME_COUNT                = 2 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< buffer不足导致的丢帧个数
	GX_DS_INT_INCOMPLETE_FRAME_COUNT          = 3 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 接收的残帧个数
	GX_DS_INT_DELIVERED_PACKET_COUNT          = 4 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 接收到的包数
	GX_DS_INT_RESEND_PACKET_COUNT             = 5 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 重传包个数
	GX_DS_INT_RESCUED_PACKED_COUNT            = 6 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 重传成功包个数
	GX_DS_INT_RESEND_COMMAND_COUNT            = 7 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 重传命令次数
	GX_DS_INT_UNEXPECTED_PACKED_COUNT         = 8 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 异常包个数
	GX_DS_INT_MAX_PACKET_COUNT_IN_ONE_BLOCK   = 9 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 数据块最大重传包数
	GX_DS_INT_MAX_PACKET_COUNT_IN_ONE_COMMAND = 10 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 一次重传命令最大包含的包数
	GX_DS_INT_RESEND_TIMEOUT                  = 11 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 重传超时时间
	GX_DS_INT_MAX_WAIT_PACKET_COUNT           = 12 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 最大等待包数
	GX_DS_ENUM_RESEND_MODE                    = 13 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_DS, ///< 重传模式,参考GX_DS_RESEND_MODE_ENTRY
	GX_DS_INT_MISSING_BLOCKID_COUNT           = 14 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,  ///< BlockID丢失个数
	GX_DS_INT_BLOCK_TIMEOUT                   = 15 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,  ///< 数据块超时时间
	GX_DS_INT_STREAM_TRANSFER_SIZE            = 16 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,  ///< 传输数据块大小
	GX_DS_INT_STREAM_TRANSFER_NUMBER_URB      = 17 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,  ///< 传输数据块数量
	GX_DS_INT_PACKET_TIMEOUT                   = 19 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,  ///< 包超时时间

	//////////////////////////////////////////////////////////////////////////
	/// 废弃区间段(Deprecated Section)
	//////////////////////////////////////////////////////////////////////////
	GX_STRING_DEVICE_ID               = 4    | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 设备序列号[弃用，改用GX_STRING_DEVICE_SERIAL_NUMBER]	
	GX_STRING_DEVICE_HARDWARE_VERSION = 5    | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 设备硬件版本[弃用]
	GX_INT_GAIN                       = 5002 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 增益[弃用，改用GX_FLOAT_GAIN]
	GX_INT_BLACKLEVEL                 = 5005 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 黑电平[弃用，改用GX_FLOAT_BLACKLEVEL]
	GX_FLOAT_BALANCE_RATIO_SELECTOR   = 5007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 白平衡通道选择[弃用，改用GX_ENUM_BALANCE_RATIO_SELECTOR]
	GX_ENUM_AA_LIGHT_ENVIRMENT        = 6007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 自动曝光、自动增益，光照环境类型[弃用，改用GX_ENUM_AA_LIGHT_ENVIRONMENT]
	GX_INT_ROI_X                      = 6008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动调节感兴趣区域X坐标[弃用，改用GX_INT_AAROI_OFFSETX]
	GX_INT_ROI_Y                      = 6009 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动调节感兴趣区域Y坐标[弃用，改用GX_INT_AAROI_OFFSETY]
	GX_INT_ROI_WIDTH                  = 6010 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动调节感兴趣区域宽度[弃用，改用GX_INT_AAROI_WIDTH]
	GX_INT_ROI_HEIGHT                 = 6011 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动调节感兴趣区域高度[弃用，改用GX_INT_AAROI_HEIGHT]
	GX_INT_AUTO_GAIN_VALUEMIN         = 6012 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动增益最小值[弃用，改用GX_FLOAT_AUTO_GAIN_MIN]
	GX_INT_AUTO_GAIN_VALUEMAX         = 6013 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动增益最大值[弃用，改用GX_FLOAT_AUTO_GAIN_MAX]
	GX_INT_AUTO_SHUTTER_VALUEMIN      = 6014 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动曝光最小值[弃用，改用GX_FLOAT_AUTO_EXPOSURE_TIME_MIN]
	GX_INT_AUTO_SHUTTER_VALUEMAX      = 6015 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动曝光最大值[弃用，改用GX_FLOAT_AUTO_EXPOSURE_TIME_MAX]
	GX_INT_CONTRASTPARAM              = 6017 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 对比度参数[弃用，改用GX_INT_CONTRAST_PARAM]
	GX_FLOAT_GAMMAPARAM               = 6018 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 伽马参数[弃用，改用GX_FLOAT_GAMMA_PARAM]
	GX_INT_COLORCORRECTIONPARAM       = 6019 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 颜色校正系数[弃用，改用GX_INT_COLOR_CORRECTION_PARAM]
	GX_DS_INT_MAX_NUM_QUEUE_BUFFER    = 18   | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,            ///< 采集队列最大Buffer个数[弃用，可选择使用GXSetAcqusitionBufferNumber接口设置采集buffer个数]


}GX_FEATURE_ID;
typedef int32_t GX_FEATURE_ID_CMD;

//------------------------------------------------------------------------------
//  句柄定义
//------------------------------------------------------------------------------
typedef void* GX_DEV_HANDLE;               ///< 设备句柄，通过GXOpenDevice获取，通过此句柄进行控制与采集
typedef void* GX_EVENT_CALLBACK_HANDLE;    ///< 设备事件回调句柄，注册设备相关事件回调函数，比如设备掉线回调函数
typedef void* GX_FEATURE_CALLBACK_HANDLE;  ///< 设备属性更新回调句柄，注册设备属性更新回调函数的时候获取

//------------------------------------------------------------------------------------
typedef enum GX_PIXEL_SIZE_ENTRY
{
	GX_PIXEL_SIZE_BPP8  = 8,
	GX_PIXEL_SIZE_BPP10 = 10,
	GX_PIXEL_SIZE_BPP12 = 12,
	GX_PIXEL_SIZE_BPP16 = 16,
	GX_PIXEL_SIZE_BPP24 = 24,
	GX_PIXEL_SIZE_BPP30 = 30,
	GX_PIXEL_SIZE_BPP32 = 32,
	GX_PIXEL_SIZE_BPP36 = 36,
	GX_PIXEL_SIZE_BPP48 = 48,
	GX_PIXEL_SIZE_BPP64 = 64,
}GX_PIXEL_SIZE_ENTRY;

typedef enum GX_PIXEL_COLOR_FILTER_ENTRY
{
	GX_COLOR_FILTER_NONE     = 0,                        ///<无
	GX_COLOR_FILTER_BAYER_RG = 1,                        ///<RG格式
	GX_COLOR_FILTER_BAYER_GB = 2,                        ///<GB格式
	GX_COLOR_FILTER_BAYER_GR = 3,                        ///<GR格式
	GX_COLOR_FILTER_BAYER_BG = 4,                        ///<BG格式
}GX_PIXEL_COLOR_FILTER_ENTRY;

#define GX_PIXEL_MONO                  ( 0x01000000 )
#define GX_PIXEL_COLOR                 ( 0x02000000 )

#define GX_PIXEL_8BIT                  ( 0x00080000 )
#define GX_PIXEL_10BIT                 ( 0x000A0000 )
#define GX_PIXEL_12BIT                 ( 0x000C0000 )
#define GX_PIXEL_16BIT                 ( 0x00100000 )
#define GX_PIXEL_24BIT                 ( 0x00180000 )
#define GX_PIXEL_30BIT                 ( 0x001E0000 )
#define GX_PIXEL_32BIT                 ( 0x00200000 )
#define GX_PIXEL_36BIT                 ( 0x00240000 )
#define GX_PIXEL_48BIT                 ( 0x00300000 )
#define GX_PIXEL_64BIT                 ( 0x00400000 )

typedef enum GX_PIXEL_FORMAT_ENTRY
{
	GX_PIXEL_FORMAT_UNDEFINED          = (0),
	GX_PIXEL_FORMAT_MONO8              = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x0001),//0x1080001,
	GX_PIXEL_FORMAT_MONO8_SIGNED       = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x0002),//0x1080002,
	GX_PIXEL_FORMAT_MONO10             = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0003),//0x1100003,	
	GX_PIXEL_FORMAT_MONO12             = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0005),//0x1100005,	
	GX_PIXEL_FORMAT_MONO14             = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0025),//0x1100025,
	GX_PIXEL_FORMAT_MONO16             = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0007),//0x1100007,
	GX_PIXEL_FORMAT_BAYER_GR8          = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x0008),//0x1080008,               
	GX_PIXEL_FORMAT_BAYER_RG8          = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x0009),//0x1080009,                
	GX_PIXEL_FORMAT_BAYER_GB8          = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x000A),//0x108000A,
	GX_PIXEL_FORMAT_BAYER_BG8          = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x000B),//0x108000B,
	GX_PIXEL_FORMAT_BAYER_GR10         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x000C),//0x110000C,                
	GX_PIXEL_FORMAT_BAYER_RG10         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x000D),//0x110000D,
	GX_PIXEL_FORMAT_BAYER_GB10         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x000E),//0x110000E,
	GX_PIXEL_FORMAT_BAYER_BG10         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x000F),//0x110000F,
	GX_PIXEL_FORMAT_BAYER_GR12         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0010),//0x1100010,              
	GX_PIXEL_FORMAT_BAYER_RG12         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0011),//0x1100011,
	GX_PIXEL_FORMAT_BAYER_GB12         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0012),//0x1100012,
	GX_PIXEL_FORMAT_BAYER_BG12         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0013),//0x1100013,	
	GX_PIXEL_FORMAT_BAYER_GR16         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x002E),//0x110002E,                
	GX_PIXEL_FORMAT_BAYER_RG16         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x002F),//0x110002F,
	GX_PIXEL_FORMAT_BAYER_GB16         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0030),//0x1100030,
	GX_PIXEL_FORMAT_BAYER_BG16         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0031),//0x1100031,	
	GX_PIXEL_FORMAT_RGB8_PLANAR        = (GX_PIXEL_COLOR | GX_PIXEL_24BIT | 0x0021),//0x2180021,
	GX_PIXEL_FORMAT_RGB10_PLANAR       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x0022),//0x2300022,
	GX_PIXEL_FORMAT_RGB12_PLANAR       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x0023),//0x2300023,
	GX_PIXEL_FORMAT_RGB16_PLANAR       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x0024),//0x2300024,
}GX_PIXEL_FORMAT_ENTRY;

typedef enum GX_ACQUISITION_MODE_ENTRY
{
	GX_ACQ_MODE_SINGLE_FRAME = 0,                          ///<单帧模式
	GX_ACQ_MODE_MULITI_FRAME = 1,                          ///<多帧模式
    GX_ACQ_MODE_CONTINUOUS   = 2,                          ///<连续模式
}GX_ACQUISITION_MODE_ENTRY;

typedef enum GX_TRIGGER_MODE_ENTRY
{
	GX_TRIGGER_MODE_OFF = 0,                             ///< 关闭触发模式
	GX_TRIGGER_MODE_ON  = 1,                             ///< 打开触发模式
}GX_TRIGGER_MODE_ENTRY;

typedef enum GX_TRIGGER_SOURCE_ENTRY
{
	GX_TRIGGER_SOURCE_SOFTWARE = 0,                      ///< 软触发
	GX_TRIGGER_SOURCE_LINE0    = 1,                      ///< 触发源0
	GX_TRIGGER_SOURCE_LINE1    = 2,                      ///< 触发源1
	GX_TRIGGER_SOURCE_LINE2    = 3,                      ///< 触发源2
	GX_TRIGGER_SOURCE_LINE3    = 4,                      ///< 触发源3
}GX_TRIGGER_SOURCE_ENTRY;

typedef enum GX_TRIGGER_ACTIVATION_ENTRY
{
	GX_TRIGGER_ACTIVATION_FALLINGEDGE = 0,               ///< 下降沿触发
	GX_TRIGGER_ACTIVATION_RISINGEDGE  = 1,               ///< 上升沿触发
}GX_TRIGGER_ACTIVATION_ENTRY;

typedef enum GX_TRIGGER_SWITCH_ENTRY
{
	GX_TRIGGER_SWITCH_OFF = 0,                           ///< 关闭外触发
	GX_TRIGGER_SWITCH_ON  = 1,                           ///< 打开外触发
}GX_TRIGGER_SWITCH_ENTRY;

typedef enum GX_EXPOSURE_MODE_ENTRY
{
	GX_EXPOSURE_MODE_TIMED          = 1,                 ///< 曝光时间寄存器控制曝光时间
	GX_EXPOSURE_MODE_TRIGGERWIDTH   = 2,                 ///< 触发信号宽度控制曝光时间
}GX_EXPOSURE_MODE_ENTRY;

typedef enum GX_EXPOSURE_AUTO_ENTRY
{
	GX_EXPOSURE_AUTO_OFF        = 0,                     ///< 关闭自动曝光
	GX_EXPOSURE_AUTO_CONTINUOUS = 1,                     ///< 连续自动曝光
	GX_EXPOSURE_AUTO_ONCE       = 2,                     ///< 单次自动曝光
}GX_EXPOSURE_AUTO_ENTRY;

typedef enum GX_USER_OUTPUT_SELECTOR_ENTRY
{
	GX_USER_OUTPUT_SELECTOR_OUTPUT0 = 1,                   ///<输出0
	GX_USER_OUTPUT_SELECTOR_OUTPUT1 = 2,                   ///<输出1
	GX_USER_OUTPUT_SELECTOR_OUTPUT2 = 4,                   ///<输出2
}GX_USER_OUTPUT_SELECTOR_ENTRY;

typedef enum GX_USER_OUTPUT_MODE_ENTRY
{
	GX_USER_OUTPUT_MODE_STROBE      = 0,                   ///<闪光灯
	GX_USER_OUTPUT_MODE_USERDEFINED = 1,                   ///<用户自定义
}GX_USER_OUTPUT_MODE_ENTRY;

typedef enum GX_STROBE_SWITCH_ENTRY
{
	GX_STROBE_SWITCH_OFF = 0,                            ///< 关闭闪光灯开关
	GX_STROBE_SWITCH_ON  = 1,                            ///< 打开闪光灯开关
}GX_STROBE_SWITCH_ENTRY;

typedef enum GX_GAIN_AUTO_ENTRY
{
	GX_GAIN_AUTO_OFF        = 0,                         ///< 关闭自动增益
	GX_GAIN_AUTO_CONTINUOUS = 1,                         ///< 连续自动增益
	GX_GAIN_AUTO_ONCE       = 2,                         ///< 单次自动增益
}GX_GAIN_AUTO_ENTRY;

typedef enum GX_GAIN_SELECTOR_ENTRY
{
	GX_GAIN_SELECTOR_ALL   = 0,                          ///< 所有增益通道
	GX_GAIN_SELECTOR_RED   = 1,                          ///< 红通道增益
	GX_GAIN_SELECTOR_GREEN = 2,                          ///< 绿通道增益
	GX_GAIN_SELECTOR_BLUE  = 3,                          ///< 蓝通道增益
}GX_GAIN_SELECTOR_ENTRY;

typedef enum GX_BLACKLEVEL_AUTO_ENTRY
{
	GX_BLACKLEVEL_AUTO_OFF        = 0,                   ///< 关闭自动黑电平
	GX_BLACKLEVEL_AUTO_CONTINUOUS = 1,                   ///< 连续自动黑电平
	GX_BLACKLEVEL_AUTO_ONCE       = 2,                   ///< 单次自动黑电平
}GX_BLACKLEVEL_AUTO_ENTRY;

typedef enum GX_BLACKLEVEL_SELECTOR_ENTRY
{
	GX_BLACKLEVEL_SELECTOR_ALL   = 0,                    ///< 所有黑电平通道
	GX_BLACKLEVEL_SELECTOR_RED   = 1,                    ///< 红通道黑电平
	GX_BLACKLEVEL_SELECTOR_GREEN = 2,                    ///< 绿通道黑电平
	GX_BLACKLEVEL_SELECTOR_BLUE  = 3,                    ///< 蓝通道黑电平
}GX_BLACKLEVEL_SELECTOR_ENTRY;

typedef enum GX_BALANCE_WHITE_AUTO_ENTRY
{
	GX_BALANCE_WHITE_AUTO_OFF        = 0,                ///< 关闭自动白平衡
	GX_BALANCE_WHITE_AUTO_CONTINUOUS = 1,                ///< 连续自动白平衡
	GX_BALANCE_WHITE_AUTO_ONCE       = 2,                ///< 单次自动白平衡
}GX_BALANCE_WHITE_AUTO_ENTRY;

typedef enum GX_BALANCE_RATIO_SELECTOR_ENTRY
{
	GX_BALANCE_RATIO_SELECTOR_RED   = 0,                   ///<红通道
	GX_BALANCE_RATIO_SELECTOR_GREEN = 1,                   ///<绿通道
	GX_BALANCE_RATIO_SELECTOR_BLUE  = 2,                   ///<蓝通道
}GX_BALANCE_RATIO_SELECTOR_ENTRY;

typedef enum GX_COLOR_CORRECT_ENTRY
{
	GX_COLOR_CORRECT_OFF = 0,                            ///< 关闭自动颜色校正
	GX_COLOR_CORRECT_ON  = 1,                            ///< 打开自动颜色校正
}GX_COLOR_CORRECT_ENTRY;

typedef enum GX_DEAD_PIXEL_CORRECT_ENTRY
{
	GX_DEAD_PIXEL_CORRECT_OFF = 0,                       ///< 关闭自动坏点校正
	GX_DEAD_PIXEL_CORRECT_ON  = 1,                       ///< 打开自动坏点校正
}GX_DEAD_PIXEL_CORRECT_ENTRY;

typedef enum GX_AA_LIGHT_ENVIRMENT_ENTRY
{
	GX_AA_LIGHT_ENVIRMENT_NATURELIGHT = 0,                 ///<自然光
	GX_AA_LIGHT_ENVIRMENT_AC50HZ      = 1,                 ///<50赫兹日光灯
	GX_AA_LIGHT_ENVIRMENT_AC60HZ      = 2,                 ///<60赫兹日光灯
}GX_AA_LIGHT_ENVIRMENT_ENTRY;

typedef enum GX_USER_SET_SELECTOR_ENTRY
{
	GX_ENUM_USER_SET_SELECTOR_DEFAULT  = 0,                 ///<默认参数组
	GX_ENUM_USER_SET_SELECTOR_USERSET0 = 1,                 ///<用户参数组0
}GX_USER_SET_SELECTOR_ENTRY;

typedef enum GX_IMAGE_GRAY_RAISE_SWITCH_ENTRY
{
	GX_IMAGE_GRAY_RAISE_SWITCH_OFF = 0,                     ///< 图像亮度拉伸开关
	GX_IMAGE_GRAY_RAISE_SWITCH_ON  = 1,                     ///< 图像亮度拉伸开关
}GX_IMAGE_GRAY_RAISE_SWITCH_ENTRY;

typedef enum GX_FIXED_PATTERN_NOISE_CORRECT_MODE
{
	GX_FIXEDPATTERNNOISECORRECT_OFF = 0,      ///< 关闭模板噪声
	GX_FIXEDPATTERNNOISECORRECT_ON  = 1,      ///< 开启模板噪声

}GX_FIXED_PATTERN_NOISE_CORRECT_MODE;

typedef enum GX_AWB_LAMP_HOUSE_ENTRY
{
	GX_AWB_LAMP_HOUSE_ADAPTIVE      = 0,                      ///< 自适应光源
	GX_AWB_LAMP_HOUSE_D65           = 1,                      ///< 指定色温6500k
	GX_AWB_LAMP_HOUSE_FLUORESCENCE  = 2,                      ///< 指定荧光灯
	GX_AWB_LAMP_HOUSE_INCANDESCENT  = 3,                      ///< 指定白炽灯
	GX_AWB_LAMP_HOUSE_D75           = 4,                      ///< 指定色温7500k
	GX_AWB_LAMP_HOUSE_D50           = 5,                      ///< 指定色温5000k
	GX_AWB_LAMP_HOUSE_U30           = 6,                      ///< 指定色温3000k
}GX_AWB_LAMP_HOUSE_ENTRY;

typedef enum GX_TEST_PATTERN_ENTRY
{
	GX_ENUM_TEST_PATTERN_OFF                    = 0,            ///<关闭
	GX_ENUM_TEST_PATTERN_GRAY_FRAME_RAMP_MOVING = 1,            ///<静止灰度递增
	GX_ENUM_TEST_PATTERN_SLANT_LINE_MOVING      = 2,            ///<滚动斜条纹
	GX_ENUM_TEST_PATTERN_VERTICAL_LINE_MOVING   = 3,            ///<滚动竖条纹
    GX_ENUM_TEST_PATTERN_SLANT_LINE             = 6,            ///<静止斜条纹
}GX_TEST_PATTERN_ENTRY;

typedef enum GX_TRIGGER_SELECTOR_ENTRY
{
	GX_ENUM_TRIGGER_SELECTOR_FRAME_START        = 1,               ///<采集一帧
    GX_ENUM_TRIGGER_SELECTOR_FRAME_BURST_START  = 2,               ///<采集多帧
} GX_TRIGGER_SELECTOR_ENTRY;

typedef enum GX_LINE_SELECTOR_ENTRY
{
	GX_ENUM_LINE_SELECTOR_LINE0           = 0,               ///<引脚0
	GX_ENUM_LINE_SELECTOR_LINE1           = 1,               ///<引脚1
	GX_ENUM_LINE_SELECTOR_LINE2           = 2,               ///<引脚2
	GX_ENUM_LINE_SELECTOR_LINE3           = 3,               ///<引脚3
} GX_LINE_SELECTOR_ENTRY;

typedef enum GX_LINE_MODE_ENTRY
{
	GX_ENUM_LINE_MODE_INPUT            = 0,               ///<输入
	GX_ENUM_LINE_MODE_OUTPUT           = 1,               ///<输出
} GX_LINE_MODE_ENTRY;

typedef enum GX_LINE_SOURCE_ENTRY
{
	GX_ENUM_LINE_SOURCE_OFF               = 0,       ///<关闭
	GX_ENUM_LINE_SOURCE_STROBE            = 1,       ///<闪光灯
	GX_ENUM_LINE_SOURCE_USEROUTPUT0       = 2,       ///<用户自定义输出0
	GX_ENUM_LINE_SOURCE_USEROUTPUT1       = 3,       ///<用户自定义输出1
	GX_ENUM_LINE_SOURCE_USEROUTPUT2       = 4,       ///<用户自定义输出2
    GX_ENUM_LINE_SOURCE_EXPOSURE_ACTIVE    			= 5,
    GX_ENUM_LINE_SOURCE_FRAME_TRIGGER_WAIT          = 6,
    GX_ENUM_LINE_SOURCE_ACQUISITION_TRIGGER_WAIT    = 7,
    GX_ENUM_LINE_SOURCE_TIMER1_ACTIVE               = 8   ///< 计时器1激活
} GX_LINE_SOURCE_ENTRY;

typedef enum GX_EVENT_SELECTOR_ENTRY
{
	GX_ENUM_EVENT_SELECTOR_EXPOSUREEND                 = 0x0004,       ///<曝光结束
	GX_ENUM_EVENT_SELECTOR_BLOCK_DISCARD               = 0x9000,       ///<图像帧丢弃
	GX_ENUM_EVENT_SELECTOR_EVENT_OVERRUN               = 0x9001,       ///<事件队列溢出
	GX_ENUM_EVENT_SELECTOR_FRAMESTART_OVERTRIGGER      = 0x9002,       ///<触发信号溢出
	GX_ENUM_EVENT_SELECTOR_BLOCK_NOT_EMPTY             = 0x9003,       ///<图像帧存不为空
	GX_ENUM_EVENT_SELECTOR_INTERNAL_ERROR              = 0x9004,       ///<内部错误事件
	GX_ENUM_EVENT_SELECTOR_FRAMEBURSTSTART_OVERTRIGGER = 0x9005,       ///<多帧触发屏蔽事件
	GX_ENUM_EVENT_SELECTOR_FRAMESTART_WAIT             = 0x9006,       ///<帧等待事件
	GX_ENUM_EVENT_SELECTOR_FRAMEBURSTSTART_WAIT        = 0x9007,       ///<多帧等待事件 
} GX_EVENT_SELECTOR_ENTRY;

typedef enum GX_EVENT_NOTIFICATION_ENTRY
{
	GX_ENUM_EVENT_NOTIFICATION_OFF             = 0,       ///<关闭事件
	GX_ENUM_EVENT_NOTIFICATION_ON              = 1,       ///<开启事件
} GX_EVENT_NOTIFICATION_ENTRY;

typedef enum GX_LUT_SELECTOR_ENTRY
{
	GX_ENUM_LUT_SELECTOR_LUMINANCE             = 0,       ///<亮度
} GX_LUT_SELECTOR_ENTRY;

typedef enum GX_TRANSFERDELAY_MODE_ENTRY
{
	GX_ENUM_TRANSFERDELAY_MODE_OFF     = 0,       ///<禁用传输延迟
	GX_ENUM_TRANSFERDELAY_MODE_ON      = 1,       ///<开启传输延迟
} GX_TRANSFERDELAY_MODE_ENTRY;

typedef enum GX_COVER_FRAMESTORE_MODE_ENTRY
{
	GX_ENUM_COVER_FRAMESTORE_MODE_OFF     = 0,       ///<禁用帧存覆盖
	GX_ENUM_COVER_FRAMESTORE_MODE_ON      = 1,       ///<开启帧存覆盖
} GX_COVER_FRAMESTORE_MODE_ENTRY;

typedef enum GX_USER_SET_DEFAULT_ENTRY
{
	GX_ENUM_USER_SET_DEFAULT_DEFAULT      = 0,       ///<出厂参数组
	GX_ENUM_USER_SET_DEFAULT_USERSET0     = 1,       ///<用户参数组0
} GX_USER_SET_DEFAULT_ENTRY;

typedef enum GX_TRANSFER_CONTROL_MODE_ENTRY
{
	GX_ENUM_TRANSFER_CONTROL_MODE_BASIC             = 0,   ///< 关闭传输控制模式
	GX_ENUM_TRANSFER_CONTROL_MODE_USERCONTROLED     = 1,   ///< 用户控制传输控制模式
} GX_TRANSFER_CONTROL_MODE_ENTRY;

typedef enum GX_TRANSFER_OPERATION_MODE_ENTRY
{
	GX_ENUM_TRANSFER_OPERATION_MODE_MULTIBLOCK  = 0,  ///< 指定发送帧数
} GX_TRANSFER_OPERATION_MODE_ENTRY;

typedef enum GX_DS_RESEND_MODE_ENTRY
{
	GX_DS_RESEND_MODE_OFF     = 0,  ///< 关闭重传模式     
	GX_DS_RESEND_MODE_ON      = 1,  ///< 开启重传模式
} GX_DS_RESEND_MODE_ENTRY;

typedef enum GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ENTRY
{
	GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_OFF   = 0,   ///< 关闭设备带宽限制模式
	GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ON    = 1    ///< 开启设备带宽限制模式
}GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ENTRY;

typedef enum GX_TEST_PATTERN_GENERATOR_SELECTOR_ENTRY
{
	GX_TEST_PATTERN_GENERATOR_SELECTOR_SENSOR  = 0,  ///< sensor 的测试图
	GX_TEST_PATTERN_GENERATOR_SELECTOR_REGION0 = 1,  ///< FPGA的测试图
}GX_TEST_PATTERN_GENERATOR_SELECTOR_ENTRY;


typedef enum GX_CHUNK_SELECTOR_ENTRY
{
	GX_CHUNK_SELECTOR_CHUNK_FRAME_ID     = 1,    ///< 帧号
	GX_CHUNK_SELECTOR_CHUNK_TIME_STAMP   = 2,    ///< 时间戳
	GX_CHUNK_SELECTOR_CHUNK_COUNTER_VALUE= 3     ///< 计数器值
}GX_CHUNK_SELECTOR_ENTRY;

typedef enum GX_ACQUISITION_FRAME_RATE_MODE_ENTRY
{
	GX_ACQUISITION_FRAME_RATE_MODE_OFF   = 0,   ///< 关闭帧率控制功能
	GX_ACQUISITION_FRAME_RATE_MODE_ON    = 1    ///< 开启帧率控制功能
}GX_ACQUISITION_FRAME_RATE_MODE_ENTRY;

typedef enum GX_SHARPNESS_MODE_ENTRY
{
	GX_SHARPNESS_MODE_OFF   = 0,   ///< 关闭锐化功能
	GX_SHARPNESS_MODE_ON    = 1    ///< 开启锐化功能
}GX_SHARPNESS_MODE_ENTRY;

typedef enum GX_BINNING_HORIZONTAL_MODE_ENTRY
{
    GX_BINNING_HORIZONTAL_MODE_SUM      = 0,    ///< BINNING水平值和
    GX_BINNING_HORIZONTAL_MODE_AVERAGE  = 1,    ///< BINNING水平值平均值
}GX_BINNING_HORIZONTAL_MODE_ENTRY;

typedef enum GX_BINNING_VERTICAL_MODE_ENTRY
{
    GX_BINNING_VERTICAL_MODE_SUM    = 0,    ///< BINNING垂直值和
    GX_BINNING_VERTICAL_MODE_AVERAGE= 1,    ///< BINNING垂直值平均值
}GX_BINNING_VERTICAL_MODE_ENTRY;

typedef enum GX_ACQUISITION_STATUS_SELECTOR_ENTRY
{
    GX_ACQUISITION_STATUS_SELECTOR_ACQUISITION_TRIGGER_WAIT = 0,    ///< 采集触发等待
    GX_ACQUISITION_STATUS_SELECTOR_FRAME_TRIGGER_WAIT       = 1,    ///< 帧触发等待
}GX_ACQUISITION_STATUS_SELECTOR_ENTRY;

typedef enum GX_GAMMA_MODE_ENTRY
{
    GX_GAMMA_SELECTOR_SRGB  = 0,    ///< 默认Gamma校正
    GX_GAMMA_SELECTOR_USER  = 1,    ///< 用户自定义Gamma校正
}GX_GAMMA_MODE_ENTRY;

typedef enum GX_COLOR_TRANSFORMATION_MODE_ENTRY
{
    GX_COLOR_TRANSFORMATION_SELECTOR_RGB_TO_RGB = 0,    ///< 默认颜色校正
    GX_COLOR_TRANSFORMATION_SELECTOR_USER       = 1,    ///< 用户自定义颜色校正
}GX_COLOR_TRANSFORMATION_MODE_ENTRY;

typedef enum GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_ENTRY
{
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN00   = 0,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN01   = 1,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN02   = 2,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN10   = 3,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN11   = 4,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN12   = 5,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN20   = 6,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN21   = 7,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN22   = 8,
}GX_COLOR_TRANSFORMATION_VALUE_ENTRY;

/* Reset Device Mode */
typedef enum GX_RESET_DEVICE_MODE
{
	GX_MANUFACTURER_SPECIFIC_RECONNECT   = 0x1,    ///< reconnect Device
	GX_MANUFACTURER_SPECIFIC_RESET       = 0x2     ///< reset Device 
}GX_RESET_DEVICE_MODE;

typedef enum GX_TIMER_SELECTOR_ENTRY
{
	GX_TIMER_SELECTOR_TIMER1   = 1,            ///< Timer1
}GX_TIMER_SELECTOR_ENTRY;

typedef enum GX_TIMER_TRIGGER_SOURCE_ENTRY
{
	GX_TIMER_TRIGGER_SOURCE_EXPOSURE_START   = 1,       ///< 接收曝光信号开始计时
}GX_TIMER_TRIGGER_SOURCE_ENTRY;

typedef enum GX_COUNTER_SELECTOR_ENTRY
{
	GX_COUNTER_SELECTOR_COUNTER1   = 1,       ///< Counter1
}GX_COUNTER_SELECTOR_ENTRY;

typedef enum GX_COUNTER_EVENT_SOURCE_ENTRY
{
	GX_COUNTER_EVENT_SOURCE_FRAME_START         = 1,    ///< 统计 "帧开始" 事件的数量
	GX_COUNTER_EVENT_SOURCE_FRAME_TRIGGER       = 2,    ///< 统计 "帧触发" 事件的数量
	GX_COUNTER_EVENT_SOURCE_ACQUISITION_TRIGGER = 3,    ///< 统计 "采集触发" 事件的数量
}GX_COUNTER_EVENT_SOURCE_ENTRY;

typedef enum GX_COUNTER_RESET_SOURCE_ENTRY
{
	GX_COUNTER_RESET_SOURCE_OFF      = 0,       ///< 关闭
	GX_COUNTER_RESET_SOURCE_SOFTWARE = 1,       ///< 软触发
	GX_COUNTER_RESET_SOURCE_LINE0    = 2,       ///< Line 0
	GX_COUNTER_RESET_SOURCE_LINE1    = 3,       ///< Line 1
	GX_COUNTER_RESET_SOURCE_LINE2    = 4,       ///< Line 2
	GX_COUNTER_RESET_SOURCE_LINE3    = 5,       ///< Line 3
}GX_COUNTER_RESET_SOURCE_ENTRY;

typedef enum GX_COUNTER_RESET_ACTIVATION_ENTRY
{
	GX_COUNTER_RESET_ACTIVATION_RISING_EDGE = 1, ///< 在信号的上升沿重置计数器
}GX_COUNTER_RESET_ACTIVATION_ENTRY;

typedef enum GX_USER_DATA_FILED_SELECTOR_ENTRY
{
	GX_USER_DATA_FILED_0    = 0,   ///< Flash数据区域0
	GX_USER_DATA_FILED_1    = 1,   ///< Flash数据区域1
	GX_USER_DATA_FILED_2    = 2,   ///< Flash数据区域2
	GX_USER_DATA_FILED_3    = 3,   ///< Flash数据区域3
}GX_USER_DATA_FILED_SELECTOR_ENTRY;

typedef enum GX_REMOVE_PARAMETER_LIMIT_ENTRY
{
	GX_ENUM_REMOVE_PARAMETER_LIMIT_OFF = 0,    ///< 关闭
	GX_ENUM_REMOVE_PARAMETER_LIMIT_ON  = 1,    ///< 开启
}GX_REMOVE_PARAMETER_LIMIT_ENTRY;

typedef enum GX_FLAT_FIELD_CORRECTION_ENTRY
{
	GX_ENUM_FLAT_FIELD_CORRECTION_OFF = 0,    ///< 关闭
	GX_ENUM_FLAT_FIELD_CORRECTION_ON  = 1,    ///< 开启
}GX_FLAT_FIELD_CORRECTION_ENTRY;

//------------------------------------------------------------------------------
//  结构体类型定义
//------------------------------------------------------------------------------

#define GX_INFO_LENGTH_8_BYTE   (8)  ///< 8字节
#define GX_INFO_LENGTH_32_BYTE  (32) ///< 32字节
#define GX_INFO_LENGTH_64_BYTE  (64) ///< 64字节
#define GX_INFO_LENGTH_128_BYTE (128)///< 128字节


typedef struct GX_DEVICE_IP_INFO 
{
	char szDeviceID[GX_INFO_LENGTH_64_BYTE + 4];         ///< 设备唯一标识,如果实际长度超过64字节有效字符串，则只保留64个有效字符
	char szMAC[GX_INFO_LENGTH_32_BYTE];                  ///< MAC地址,如果实际长度超过32字节有效字符串，则只保留31个有效字符
	char szIP[GX_INFO_LENGTH_32_BYTE];                   ///< IP地址,如果实际长度超过32字节有效字符串，则只保留31个有效字符
	char szSubNetMask[GX_INFO_LENGTH_32_BYTE];           ///< 子网掩码,如果实际长度超过32字节有效字符串，则只保留31个有效字符
	char szGateWay[GX_INFO_LENGTH_32_BYTE];              ///< 网关,如果实际长度超过32字节有效字符串，则只保留31个有效字符
	char szNICMAC[GX_INFO_LENGTH_32_BYTE];               ///< 对应网卡的MAC地址,如果实际长度超过32字节有效字符串，则只保留31个有效字符
	char szNICIP[GX_INFO_LENGTH_32_BYTE];                ///< 对应网卡的IP地址,如果实际长度超过32字节有效字符串，则只保留31个有效字符
	char szNICSubNetMask[GX_INFO_LENGTH_32_BYTE];        ///< 对应网卡的子网掩码,如果实际长度超过32字节有效字符串，则只保留31个有效字符
	char szNICGateWay[GX_INFO_LENGTH_32_BYTE];           ///< 对应网卡的网关,如果实际长度超过32字节有效字符串，则只保留31个有效字符
	char szNICDescription[GX_INFO_LENGTH_128_BYTE + 4];  ///< 对应网卡描述,如果实际长度超过128字节有效字符串，则只保留128个有效字符
	char reserved[512];                                  ///< 保留
}GX_DEVICE_IP_INFO;

typedef struct GX_DEVICE_BASE_INFO 
{
	char szVendorName[GX_INFO_LENGTH_32_BYTE];              ///< 厂商名称,如果实际长度超过32字节有效字符串，则只保留31个有效字符
	char szModelName[GX_INFO_LENGTH_32_BYTE];               ///< 设备类型名称,如果实际长度超过32字节有效字符串，则只保留31个有效字符
	char szSN[GX_INFO_LENGTH_32_BYTE];                      ///< 设备序列号,如果实际长度超过32字节有效字符串，则只保留31个有效字符
	char szDisplayName[GX_INFO_LENGTH_128_BYTE + 4];        ///< 设备展示名称,如果实际长度超过128字节有效字符串，则只保留128个有效字符
	char szDeviceID[GX_INFO_LENGTH_64_BYTE + 4];            ///< 设备唯一标识,如果实际长度超过64字节有效字符串，则只保留64个有效字符
	char szUserID[GX_INFO_LENGTH_64_BYTE + 4];              ///< 用户自定义名称,如果实际长度超过64字节有效字符串，则只保留64个有效字符
	GX_ACCESS_STATUS_CMD  accessStatus;                     ///< 设备当前支持的访问状态
	GX_DEVICE_CLASS   deviceClass;                          ///< 设备种类，比如USB2.0、GEV	
	char reserved[300];                                     ///< 保留
}GX_DEVICE_BASE_INFO;

typedef struct GX_OPEN_PARAM 
{
	char               *pszContent;        ///< 输入参数内容,不允许为空字符串
	GX_OPEN_MODE_CMD   openMode;           ///< 打开方式
	GX_ACCESS_MODE_CMD accessMode;         ///< 访问模式
}GX_OPEN_PARAM;

typedef struct GX_FRAME_CALLBACK_PARAM
{
	void*               pUserParam;         ///< 用户私有数据
	GX_FRAME_STATUS     status;             ///< 图像的返回状态
	const  void*        pImgBuf;            ///< 图像buffer地址（开启chunkdata后，pImgBuf 包含图像数据和帧信息数据 ）
	int32_t             nImgSize;           ///< 图像大小数据大小，单位字节（开启chunkdata后，nImgsize为图像数据大小+帧信息大小）
	int32_t             nWidth;             ///< 图像的宽
	int32_t             nHeight;            ///< 图像的高
	int32_t             nPixelFormat;       ///< 图像的PixFormat 
	uint64_t            nFrameID;           ///< 图像的帧号
	uint64_t            nTimestamp;         ///< 图像的时间戳
	int32_t             reserved[1];        ///< 保留
}GX_FRAME_CALLBACK_PARAM;

typedef struct GX_FRAME_DATA
{
	GX_FRAME_STATUS		nStatus;             ///< 图像的返回状态
	void*				pImgBuf;             ///< 图像buffer地址（开启chunkdata后，pImgBuf 包含图像数据和帧信息数据 ）
	int32_t				nWidth;              ///< 图像的宽
	int32_t				nHeight;             ///< 图像的高
	int32_t				nPixelFormat;        ///< 图像的PixFormat
	int32_t				nImgSize;            ///< 图像大小数据大小，单位字节（开启chunkdata后，nImgsize为图像数据大小+帧信息大小）
	uint64_t			nFrameID;            ///< 图像的帧号
	uint64_t			nTimestamp;          ///< 图像的时间戳
	int32_t				reserved[3];         ///< 保留
}GX_FRAME_DATA;


typedef struct GX_INT_RANGE
{
	int64_t nMin;                      ///< 整型值最小值
	int64_t nMax;                      ///< 整型值最大值
	int64_t nInc;                      ///< 整型值步长
	int32_t reserved[8];               ///< 保留
}GX_INT_RANGE;

typedef struct GX_FLOAT_RANGE
{
	double  dMin;                           ///< 浮点型最小值
	double  dMax;                           ///< 浮点型最大值
	double  dInc;                           ///< 浮点型步长
	char    szUnit[GX_INFO_LENGTH_8_BYTE];  ///< 浮点型单位
	bool    bIncIsValid;                    ///< 步长是否有效
	int8_t  reserved[31];                   ///< 保留
}GX_FLOAT_RANGE;

typedef struct GX_ENUM_DESCRIPTION
{
	int64_t nValue;                               ///< 枚举值
	char    szSymbolic[GX_INFO_LENGTH_64_BYTE];   ///< 字符描述
	int32_t reserved[8];                          ///< 保留
}GX_ENUM_DESCRIPTION;

//------------------------------------------------------------------------------
//  回调函数类型定义
//------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
/**
\brief     采集回调函数定义
\param     pFrameData    帧数据信息结构体
\return    void
*/
//----------------------------------------------------------------------------------
typedef void (GX_STDC* GXCaptureCallBack) (GX_FRAME_CALLBACK_PARAM *pFrameData);
//----------------------------------------------------------------------------------
/**
\brief     掉线回调函数定义
\param     pUserParam    用户私有参数，注册掉线回调函数的时候传入此参数
\return    void
*/
//----------------------------------------------------------------------------------
typedef void (GX_STDC *GXDeviceOfflineCallBack) (void *pUserParam);
//----------------------------------------------------------------------------------
/**
\brief     基本属性回调函数定义
\param     nFeatureID    属性控制ID，与注册基本属性回调函数的时候传入的值一致
\param     pUserParam    用户私有参数，与注册基本属性回调函数的时候传入的值一致
\return    void
*/
//----------------------------------------------------------------------------------
typedef void (GX_STDC *GXFeatureCallBack) (GX_FEATURE_ID_CMD  nFeatureID , void *pUserParam);


//------------------------------------------------------------------------------
//  标准C API功能函数定义
//------------------------------------------------------------------------------
#define GX_API GX_EXTC GX_STATUS GX_STDC

//------------------------------------------------------------------------
/**
\brief     初始化设备库。
\attention 调用其他接口（除了GXGetLastError和GXCloseLib）之前必须先调用此接口，当用户不再使用库的时候调用GXCloseLib释放库资源。
           如果之前用户已经调用过GXInitLib，没有调用GXCloseLib，而再次调用GXInitLib接口，接口返回成功。
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_FOUND_TL        找不到TL库
		   其它错误情况请参见GX_STATUS_LIST
		           
*/
//------------------------------------------------------------------------
GX_API GXInitLib();

//----------------------------------------------------------------------------------
/**
\brief     关闭设备库，释放资源
\attention 释放库资源，当用户不再使用库的时候调用此接口。
           如果用户之前没有调用GXInitLib，直接调用GXCloseLib，接口返回成功。
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           其它错误情况请参见GX_STATUS_LIST   
*/
//----------------------------------------------------------------------------------
GX_API GXCloseLib();

//------------------------------------------------------------------------
/**
\brief       获取程序最后的错误描述信息
\attention   当用户调用其它接口失败的时候，可以调用此接口获取关于失败信息的详细描述
\param [out] pErrorCode    返回最后的错误码，如果用户不想获取此值，那么此参数可以传NULL
\param [out] pszErrText    返回错误信息缓冲区地址
\param [in,out] pSize      错误信息缓冲区地址大小，单位字节
                           如果pszErrText为NULL：
                           [out]pnSize返回实际需要的buffer大小
                           如果pszErrText非NULL：
                           [in]pnSize为实际分配的buffer大小
                           [out]pnSize返回实际填充buffer大小
\return GX_STATUS_SUCCESS                操作成功，没有发生错误
        GX_STATUS_INVALID_PARAMETER      用户输入的指针为NULL
	    GX_STATUS_NEED_MORE_BUFFER       用户分配的buffer过小
	    其它错误情况请参见GX_STATUS_LIST
*/
//------------------------------------------------------------------------
GX_API GXGetLastError             (GX_STATUS *pErrorCode, char *pszErrText, size_t *pSize);

//----------------------------------------------------------------------------------
/**
\brief     枚举所有设备并且获取设备个数,对于千兆网设备此接口仅能枚举同网段设备
\attention 此接口的作用是更新库内部设备列表，此接口会改变库内部设备列表，
           所以调用GXGetAllDeviceBaseInfo和GXOpenDevice之前需要调用此接口。
           如果在用户指定超时时间内成功枚举到设备，则立即返回；如果在用户指定超时时间内没有枚举到设备，则一直等待，直到达到用户指定的超时时间返回
\param     [out]punNumDevices 返回设备个数
\param     [in]unTimeOut      枚举的超时时间(单位ms)。
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
           GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
           其它错误情况请参见GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
GX_API GXUpdateDeviceList         (uint32_t* punNumDevices, uint32_t nTimeOut);

//----------------------------------------------------------------------------------
/**
\brief     枚举所有设备并且获取设备个数,对于千兆网设备此接口能够枚举所有子网内的设备
\attention 此接口的作用是更新库内部设备列表，此接口会改变库内部设备列表，
           所以调用GXGetAllDeviceBaseInfo和GXOpenDevice之前需要调用此接口。
           如果在用户指定超时时间内成功枚举到设备，则立即返回；如果在用户指定超时时间内没有枚举到设备，则一直等待，直到达到用户指定的超时时间返回
\param     [out]punNumDevices 返回设备个数
\param     [in]unTimeOut      枚举的超时时间(单位ms)。
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
           GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
           其它错误情况请参见GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
GX_API GXUpdateAllDeviceList         (uint32_t* punNumDevices, uint32_t nTimeOut);

//----------------------------------------------------------------------------------
/**
\brief        获取所有设备的基础信息
\attention    此接口调用之前需要调用GXUpdateDeviceList接口，更新库内部设备列表
\param [out] pDeviceInfo   设备信息结构体指针
\param [in,out]pBufferSize 设备信息结构体缓冲区大小，单位字节                           
						   如果pDeviceInfo为NULL：
						   [out]pnBufferSize返回实际大小
						   如果pDeviceInfo非NULL：
						   [in]pnBufferSize为用户分配buffer大小
						   [out]pnBufferSize返回实际填充buffer大小
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
           GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
           其它错误情况请参见GX_STATUS_LIST  
*/
//----------------------------------------------------------------------------------
GX_API GXGetAllDeviceBaseInfo     (GX_DEVICE_BASE_INFO* pDeviceInfo, size_t* pBufferSize);

//----------------------------------------------------------------------------------
/**
\brief        指定设备序号获取设备的网络信息
\attention    此接口调用之前需要调用GXUpdateDeviceList接口，更新库内部设备列表
\param [in]  nIndex  设备序号，从1开始，例如：1、2、3、4...
\param [out] pstDeviceIPInfo   设备信息结构体指针
\return    GX_STATUS_SUCCESS    操作成功，没有发生错误
		   GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
           GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
其它错误情况请参见GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
GX_API GXGetDeviceIPInfo(uint32_t nIndex, GX_DEVICE_IP_INFO* pstDeviceIPInfo);

//----------------------------------------------------------------------------------
/**
\brief 通过序号打开设备
\param nDeviceIndex 设备序号，从1开始，例如：1、2、3、4...
\param phDevice 返回设备句柄
\return GX_STATUS,捕获底层调用产生的异常，根据异常类型返回不同的错误码
*/
//----------------------------------------------------------------------------------
GX_API GXOpenDeviceByIndex        (uint32_t nDeviceIndex, GX_DEV_HANDLE* phDevice);   // 已弃用

//----------------------------------------------------------------------------------
/**
\brief     通过指定唯一标示打开设备，例如指定SN、IP、MAC等
\attention 此接口调用之前需要调用GXUpdateDeviceList接口，更新库内部设备列表
\param     [in]pOpenParam    用户配置的打开设备参数,参见GX_OPEN_PARAM结构体定义
\param     [out]phDevice     返回设备句柄
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
           GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   GX_STATUS_NOT_FOUND_DEVICE    没有找到与指定信息匹配的设备
		   GX_STATUS_INVALID_ACCESS      设备的访问方式不对
           其它错误情况请参见GX_STATUS_LIST  
*/
//----------------------------------------------------------------------------------
GX_API GXOpenDevice               (GX_OPEN_PARAM* pOpenParam, GX_DEV_HANDLE* phDevice);

//----------------------------------------------------------------------------------
/**
\brief     指定设备句柄关闭设备
\attention 不能重复关闭同一个设备
\param     [in]hDevice 即将要关闭的设备句柄
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄，或者关闭已经被关闭的设备
		   其它错误情况请参见GX_STATUS_LIST  
*/
//----------------------------------------------------------------------------------
GX_API GXCloseDevice              (GX_DEV_HANDLE hDevice);

//----------------------------------------------------------------------------------
/**
\brief     获取设备的永久IP信息
\attention 该接口只适用于网络设备
\param     [in]       hDevice                  设备句柄
\param     [in]       pszIP                    设备永久IP字符串地址
\param     [in, out]  pnIPLength               设备永久IP地址字符串长度,单位字节。
\param     [in]       pnIPLength:              用户buffer大小
\param     [out]      pnIPLength:              实际填充大小
\param     [in]       pszSubNetMask            设备永久子网掩码字符串地址
\param     [in, out]  pnSubNetMaskLength       设备永久子网掩码字符串长度
\param     [in]       pnSubNetMaskLength:      用户buffer大小
\param     [out]      pnSubNetMaskLength:      实际填充大小
\param     [in]       pszDefaultGateWay        设备永久网关字符串地址
\param     [in, out]  pnDefaultGateWayLength   设备永久网关字符串长度
\param     [in]       pnDefaultGateWayLength:  用户buffer大小
\param     [out]      pnDefaultGateWayLength:  实际填充大小
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
		   GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
GX_API GXGetDevicePersistentIpAddress (GX_DEV_HANDLE  hDevice, 
									   char* pszIP, 
									   size_t *pnIPLength, 
									   char* pszSubNetMask, 
									   size_t *pnSubNetMaskLength, 
									   char* pszDefaultGateWay, 
									   size_t *pnDefaultGateWayLength);

//----------------------------------------------------------------------------------
/**
\brief     设置设备的永久IP信息
\attention 该接口只适用于网络设备
\param     [in]     hDevice              设备句柄
\param     [in]     pszIP                设备永久IP字符串，末尾’\0’
\param     [in]     pszSubNetMask        设备永久子网掩码字符串，末尾’\0’
\param     [in]     pszDefaultGateWay    设备永久网关字符串，末尾’\0’
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄，或者关闭已经被关闭的设备
		   其它错误情况请参见GX_STATUS_LIST  
*/
//----------------------------------------------------------------------------------
GX_API GXSetDevicePersistentIpAddress (GX_DEV_HANDLE  hDevice, 
									   const char* pszIP, 
									   const char* pszSubNetMask, 
									   const char* pszDefaultGateWay);

//----------------------------------------------------------------------------------
/**
\brief      获取功能控制码对应的字符串描述
\attention  此接口专门用来获取功能名称描述信息，方便用户做UI程序
\param [in]hDevice     设备句柄
\param [in]featureID   功能码ID
\param [out]pszName    用户输入的字符串缓冲区地址,字符串长度包含末尾结束符'\0'
\param [in,out]pnSize  用户输入的表示字符串缓冲区地址的长度,单位字节。
						如果用户输入的pszName为NULL：
						[out]pnSize返回需要的实际长度。
						如果用户输入的pszName非NULL：
						[in]pnSize为用户分配的buffer大小；
						[out]pnSize返回实际填充buffer大小；
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
           GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   GX_STATUS_NEED_MORE_BUFFER    用户分配的buffer过小
           其它错误情况请参见GX_STATUS_LIST  
*/
//----------------------------------------------------------------------------------
GX_API GXGetFeatureName           (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, char* pszName, size_t* pnSize); 

//----------------------------------------------------------------------------------
/**
\brief      查询当前相机是否支持某功能
\attention  不支持某功能有两个情况： 1、通过查询相机寄存器，查到当前相机当前不支持此功能
                                     2、相机XML描述文件中没有此功能的描述节点
\param [in]hDevice   设备句柄
\param [in]featureID 功能码ID
\param [out]pbIsImplemented 如果支持则返回true，如果不支持则返回false
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   其它错误情况请参见GX_STATUS_LIST  
*/
//----------------------------------------------------------------------------------
GX_API GXIsImplemented		      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, bool* pbIsImplemented);

//----------------------------------------------------------------------------------
/**
\brief      查询某功能码当前是否可读
\attention  某些功能的可读属性是随着其它节点的当前值改变的，可用此接口实时查询功能当前是否可读
\param [in]hDevice 设备句柄
\param [in]featureID 功能码ID
\param [out]pbIsReadable 用来返回结果，如果可读则返回true，如果不可读则返回false。
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   其它错误情况请参见GX_STATUS_LIST  
*/
//----------------------------------------------------------------------------------
GX_API GXIsReadable               (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, bool* pbIsReadable);

//----------------------------------------------------------------------------------
/**
\brief      查询某功能码当前是否可写
\attention  某些功能的可写属性是随着其它节点的当前值改变的，可用此接口实时查询功能当前是否可写
\param [in]hDevice 设备句柄
\param [in]featureID 功能码ID
\param [out]pbIsWritable 用来返回结果，如果可写则返回true，如果不可写则返回false。
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   其它错误情况请参见GX_STATUS_LIST  
*/
//----------------------------------------------------------------------------------
GX_API GXIsWritable               (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, bool* pbIsWritable);

//----------------------------------------------------------------------------------
/**
\brief      获取Int类型值的最小值、最大值、步长等描述信息
\attention  某些属性的范围可能受其他功能的影响，可用调用此接口查询当前实际范围
\param [in]hDevice    设备句柄
\param [in]featureID  功能码ID
\param [out]pIntRange 范围描述结构体
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXGetIntRange		      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, GX_INT_RANGE* pIntRange);

//----------------------------------------------------------------------------------
/**
\brief      获取Int类型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param [in]hDevice 设备句柄
\param [in]featureID 功能码ID
\param [out]pnValue 用来返回当前值
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   GX_STATUS_INVALID_ACCESS      当前不可访问
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXGetInt				      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, int64_t* pnValue);

//----------------------------------------------------------------------------------
/**
\brief      设置Int类型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param [in]hDevice   设备句柄
\param [in]featureID 功能码ID
\param [in]pnValue   用户设置的当前值
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_OUT_OF_RANGE        用户传入值越界
		   GX_STATUS_INVALID_ACCESS      当前不可访问
其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXSetInt				      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, int64_t nValue);

//----------------------------------------------------------------------------------
/**
\brief      获取Float类型值的最小值、最大值、步长等信息
\attention  某些属性的范围可能受其他功能的影响，可用调用此接口查询当前实际范围 
\param [in]hDevice 设备句柄
\param [in]featureID 功能码ID
\param [out]pFloatRange 范围描述结构体
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXGetFloatRange		      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, GX_FLOAT_RANGE* pFloatRange);

//----------------------------------------------------------------------------------
/**
\brief      设置浮点类型值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param [in]hDevice   设备句柄
\param [in]featureID 功能码ID
\param [in]dValue    设置值
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_OUT_OF_RANGE        用户传入值越界
		   GX_STATUS_INVALID_ACCESS      当前不可访问
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXSetFloat                 (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, double dValue);

//----------------------------------------------------------------------------------
/**
\brief      获取浮点类型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param [in]hDevice   设备句柄
\param [in]featureID 功能码ID
\param [out]pdValue  用来返回当前值
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   GX_STATUS_INVALID_ACCESS      当前不可访问
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXGetFloat                 (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, double* pdValue);

//----------------------------------------------------------------------------------
/**
\brief      获取枚举类型值有多少项
\attention  某枚举功能类型的项数是需要查询的，头文件中所罗列的是所有可能的项，但是
            当前相机实际支出多少项，建议用户先查后用。
\param [in]hDevice 设备句柄
\param [in]featureID 功能码ID
\param [out]pnEntryNums 指向项个数的指针
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXGetEnumEntryNums         (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, uint32_t* pnEntryNums);

//----------------------------------------------------------------------------------
/**
\brief      获取枚举类型功能的枚举项的每一项的描叙信息和值
\attention  用户做UI程序的时候需要枚举功能项的描述信息；枚举功能项的值建议用户先查
            后用，因为值可能是离散的值，每个枚举功能的可选值，在头文件中都有定义。
\param [in]hDevice   设备句柄
\param [in]featureID 功能码ID
\param [out]pEnumDescription GX_ENUM_DESCRIPTION数组指针，返回的枚举描述信息
\param [in,out]pBufferSize 用户传入的GX_ENUM_DESCRIPTION数组的大小，大为字节
							如果pEnumDescription为NULL：
							[out]pnBufferSize为实际需要的buffer大小
							如果pEnumDescription非NULL：
							[in]pnBufferSize为用户分配的buffer大小
							[out]pnBufferSize返回实际填充buffer大小       
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   GX_STATUS_NEED_MORE_BUFFER    用户分配的buffer过小
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXGetEnumDescription       (GX_DEV_HANDLE hDevice, 
								   GX_FEATURE_ID_CMD featureID, 
								   GX_ENUM_DESCRIPTION* pEnumDescription,
								   size_t* pBufferSize);

//----------------------------------------------------------------------------------
/**
\brief      获取枚举型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param [in]hDevice 设备句柄
\param [in]featureID 功能码ID
\param [out]pnValue 用来返回当前值
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   GX_STATUS_INVALID_ACCESS      当前不可访问
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXGetEnum			      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, int64_t* pnValue);

//----------------------------------------------------------------------------------
/**
\brief      设置枚举型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param [in]hDevice   设备句柄
\param [in]featureID 功能码ID
\param [in]pnValue   用户设置的当前值
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户传入值非法
		   GX_STATUS_INVALID_ACCESS      当前不可访问
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXSetEnum			      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, int64_t nValue);

//----------------------------------------------------------------------------------
/**
\brief      获取布尔型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param [in]hDevice 设备句柄
\param [in]featureID 功能码ID
\param [out]pbValue 用来返回当前值
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   GX_STATUS_INVALID_ACCESS      当前不可访问
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXGetBool			      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, bool* pbValue);

//----------------------------------------------------------------------------------
/**
\brief      设置布尔型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param [in]hDevice   设备句柄
\param [in]featureID 功能码ID
\param [in]pbValue   用户设置的当前值
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_ACCESS      当前不可访问
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXSetBool			      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, bool bValue);

//----------------------------------------------------------------------------------
/**
\brief      获取字符串类型值的长度
\attention  此接口与GxGetString接口组合使用,便于用户申请buffer
\param [in]hDevice   设备句柄
\param [in]featureID 功能码ID
\param [out]pnSize   用来返回字符串当前值长度，包含字符串末尾结束符'\0'。
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXGetStringLength	      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, size_t* pnSize);

//----------------------------------------------------------------------------------
/**
\brief      获取字符串类型值的长度
\attention  此接口与GxGetString接口组合使用,便于用户申请buffer
\param [in]hDevice   设备句柄
\param [in]featureID 功能码ID
\param [out]pnSize   用来返回字符串最大长度，包含字符串末尾结束符'\0'。
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXGetStringMaxLength	      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, size_t* pnSize);

//----------------------------------------------------------------------------------
/**
\brief      获取字符串
\attention  读取字符串之前需要调用GXGetStringLength接口获取长度
\param [in]hDevice 设备句柄
\param [in]featureID 功能码ID
\param [out]pszContent 用户输入的字符串缓冲区地址,末尾包含结束符'\0'
\param [in,out]pnSize 表示用户输入的字符串缓冲区地址的长度
						如果pszContent为NULL：
						[out]pnSize为实际需要的buffer大小
						如果pszContent非NULL：
						[in]pnSize为用户分配的buffer大小
						[out]pnSize返回实际填充buffer大小
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   GX_STATUS_INVALID_ACCESS      当前不可访问
		   GX_STATUS_NEED_MORE_BUFFER    用户分配的buffer过小
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXGetString			      (GX_DEV_HANDLE hDevice, 
								   GX_FEATURE_ID_CMD featureID, 
								   char* pszContent, 
								   size_t* pnSize);

//----------------------------------------------------------------------------------
/**
\brief      设置字符串
\attention  无
\param [in]hDevice 设备句柄
\param [in]featureID 功能码ID
\param [in]pszContent 用户输入的字符串，字符串末尾包含结束符'\0'
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户传入指针为NULL
		   GX_STATUS_OUT_OF_RANGE        用户写入内容超过字符串最大长度
		   GX_STATUS_INVALID_ACCESS      当前不可访问
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXSetString			      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, char* pszContent);

//----------------------------------------------------------------------------------
/**
\brief      获取buffer类型值的长度
\attention  此接口与GxGetBuffer接口组合使用,便于用户申请buffer
\param [in]hDevice   设备句柄
\param [in]featureID 功能码ID
\param [out]pnSize   用来返回长度值。
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   其它错误情况请参见GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
GX_API GXGetBufferLength	      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, size_t* pnSize);

//----------------------------------------------------------------------------------
/**
\brief      获取buffer数据块
\attention  读取buffer数据块之前需要调用GXGetBufferLength接口获取长度
\param [in]hDevice 设备句柄
\param [in]featureID 功能码ID
\param [out]pBuffer 用户输入的缓冲区地址
\param [in,out]pnSize 表示用户输入的缓冲区地址的长度
						如果pBuffer为NULL：
						[out]pnSize为实际需要的buffer大小
						如果pBuffer非NULL：
						[in]pnSize为用户分配的buffer大小
						[out]pnSize返回实际填充buffer大小
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   GX_STATUS_INVALID_ACCESS      当前不可访问
		   GX_STATUS_NEED_MORE_BUFFER    用户分配的buffer过小
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXGetBuffer			      (GX_DEV_HANDLE hDevice, 
								   GX_FEATURE_ID_CMD featureID, 
								   uint8_t* pBuffer, 
								   size_t* pnSize);

//----------------------------------------------------------------------------------
/**
\brief      设置buffer数据块
\attention  无
\param [in]hDevice   设备句柄
\param [in]featureID 功能码ID
\param [in]pBuffer   用户输入的缓冲区地址
\param [in]nSize     表示用户输入的缓冲区地址的长度
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户传入指针为NULL
		   GX_STATUS_OUT_OF_RANGE        用户写入内容超过字符串最大长度
		   GX_STATUS_INVALID_ACCESS      当前不可访问
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXSetBuffer			      (GX_DEV_HANDLE hDevice, 
								   GX_FEATURE_ID_CMD featureID, 
								   uint8_t* pBuffer, 
								   size_t nSize);

//----------------------------------------------------------------------------------
/**
\brief      发送控制命令
\attention  无
\param [in]hDevice    设备句柄
\param [in]featureID  功能码ID
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_ACCESS      当前不可访问
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXSendCommand		      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID);

//----------------------------------------------------------------------------------
/**
\brief      注册采集回调函数
\attention  必须在发送开采命令之前注册采集回调函数
\param [in]hDevice     设备句柄
\param [in]pUserParam  用户私有数据
\param [in]callBackFun 用户注册的回调函数
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_INVALID_PARAMETER   用户传入指针为NULL
		   GX_STATUS_INVALID_CALL        发送开采命令后，不能注册采集回调函数
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXRegisterCaptureCallback  (GX_DEV_HANDLE hDevice, void *pUserParam, GXCaptureCallBack callBackFun);

//----------------------------------------------------------------------------------
/**
\brief      注销采集回调函数
\attention  必须在发送停采命令之后注销采集回调函数
\param [in]hDevice 设备句柄
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_INVALID_CALL        发送停采命令之前，不能注销采集回调函数
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXUnregisterCaptureCallback(GX_DEV_HANDLE hDevice);

//----------------------------------------------------------------------------------
/**
\brief      直接获取一帧图像
\attention  如果用户已注册采集回调函数，调用此接口会报错GX_STATUS_INVALID_CALL
\param [in]hDevice        设备句柄
\param [in,out]pFrameData 图像信息结构体指针
\param [in]nTimeout       超时时间
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_INVALID_CALL        发送停采命令之前，不能注销采集回调函数
		   GX_STATUS_INVALID_PARAMETER   用户传入图像地址指针为NULL
		   GX_STATUS_NEED_MORE_BUFFER    用户分配的图像buffer小于实际需要的大小
		   其它错误情况请参见GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
GX_API GXGetImage(GX_DEV_HANDLE hDevice, GX_FRAME_DATA *pFrameData, uint32_t nTimeout);

//----------------------------------------------------------------------------------
/**
\brief      清空采集输出队列
\attention  如果用户处理图像的速度较慢，库内会残留上次采集过程的缓存图像，特别在触发模式下，
            用户发送完触发之后，获取到的是旧图，如果用户想获取到当前触发对应的图像，需要在
		    发送触发之前调用GXFlushQueue接口，先清空图像输出队列。
\param     [in]hDevice        设备句柄
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   其它错误情况请参见GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
GX_API GXFlushQueue(GX_DEV_HANDLE hDevice);

//----------------------------------------------------------------------------------
/**
\brief      注册事件回调函数
\attention  设备事件比如，掉线事件、曝光结束等，这些事件都可以通过这个接口的回调方式传出，
            用户不需要获取事件的时候调用GXUnregisterEventCallback接口注销回调函数
\param [in]hDevice     设备句柄
\param [in]pUserParam  用户私有数据
\param [in]callBackFun 用户注册的回调函数
\param [in]eventID     事件类型码
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_INVALID_PARAMETER   用户传入回调函数非法或者传入事件类型非法
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXRegisterDeviceOfflineCallback    (GX_DEV_HANDLE hDevice, 
										   void* pUserParam, 
										   GXDeviceOfflineCallBack callBackFun, 
										   GX_EVENT_CALLBACK_HANDLE *pHCallBack);

//----------------------------------------------------------------------------------
/**
\brief      注销事件回调函数
\attention  设备事件比如，掉线事件、曝光结束等，这些事件都可以通过这个接口的回调方式传出，
            用户不需要获取事件的时候调用GXUnregisterEventCallback接口注销回调函数
\param [in]hDevice     设备句柄
\param [in]eventID     事件类型码
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_INVALID_CALL        发送停采命令之前，不能注销采集回调函数
		   GX_STATUS_INVALID_PARAMETER   用户传入事件类型非法
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXUnregisterDeviceOfflineCallback  (GX_DEV_HANDLE hDevice, GX_EVENT_CALLBACK_HANDLE  hCallBack);

//----------------------------------------------------------------------------------
/**
\brief      清空事件输出队列
\attention  库内部事件数据的接收和处理采用缓存机制，如果用户接收、处理事件的速度慢于事件产生的速度，
            事件数据就会在库内积累，会影响用户获取实时事件数据。如果用户想获取实时事件数据，需要先
			调用GXFlushEvent接口清空事件缓存数据。此接口一次性清空所有事件数据。
\param     [in]hDevice        设备句柄
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   其它错误情况请参见GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
GX_API GXFlushEvent               (GX_DEV_HANDLE hDevice);

//----------------------------------------------------------------------------------
/**
\brief      获取当前事件队列里面的事件个数
\param     [in]hDevice        设备句柄
\param     [in]pnEventNum     事件个数指针
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
		   GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
           GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
           GX_STATUS_INVALID_PARAMETER   用户传入pnEventNum为NULL指针
           其它错误情况请参见GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
GX_API GXGetEventNumInQueue       (GX_DEV_HANDLE hDevice, uint32_t *pnEventNum);

//----------------------------------------------------------------------------------
/**
\brief      注册属性更新回调函数
\attention  用户可通过此接口获取事件数据，详见示例程序
\param [in]hDevice     设备句柄
\param [in]pUserParam  用户私有数据
\param [in]callBackFun 用户注册的回调函数
\param [in]featureID   功能码
\param [out]pHCallBack  回调函数句柄
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_INVALID_PARAMETER   用户传入回调函数非法
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXRegisterFeatureCallback  (GX_DEV_HANDLE hDevice, 
								   void* pUserParam, 
								   GXFeatureCallBack  callBackFun, 
								   GX_FEATURE_ID_CMD  featureID,
								   GX_FEATURE_CALLBACK_HANDLE *pHCallBack);

//----------------------------------------------------------------------------------
/**
\brief      注销属性更新回调函数
\attention  与GXRegisterFeatureCallback配套使用，每次注册都必须有相应的注销与之对应
\param [in]hDevice     设备句柄
\param [in]featureID   功能码
\param [out]pHCallBack  回调函数句柄
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXUnregisterFeatureCallback(GX_DEV_HANDLE  hDevice, GX_FEATURE_ID_CMD featureID, GX_FEATURE_CALLBACK_HANDLE  hCallBack);

//----------------------------------------------------------------------------------
/**
\brief      导出相机当前参数到配置文件（ANSI接口）
\param [in]hDevice         设备句柄
\param [in]pszFilePath     配置文件输出路径
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

*/
//----------------------------------------------------------------------------------
GX_API GXExportConfigFile (GX_DEV_HANDLE hDevice, const char * pszFilePath);


//----------------------------------------------------------------------------------
/**
\brief      导出相机当前参数到配置文件（UNICODE接口）
\param [in]hDevice         设备句柄
\param [in]pszFilePath     配置文件输出路径（wchar_t类型）
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

*/
//----------------------------------------------------------------------------------
GX_API GXExportConfigFileW(GX_DEV_HANDLE hDevice, const wchar_t * pszFilePath);


//----------------------------------------------------------------------------------
/**
\brief      将配置文件中参数导入到相机（ANSI接口）
\param [in]hDevice         设备句柄
\param [in]pszFilePath     配置文件路径
\param [in]bVerify         如果此值为true，所有导入进去的值将会被读出进行校验是否一致
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

*/
//----------------------------------------------------------------------------------
GX_API GXImportConfigFile(GX_DEV_HANDLE hDevice, const char * pszFilePath, bool bVerify = false);

//----------------------------------------------------------------------------------
/**
\brief      将配置文件中参数导入到相机（UNICODE接口）
\param [in]hDevice         设备句柄
\param [in]pszFilePath     配置文件路径
\param [in]bVerify         如果此值为true，所有导入进去的值将会被读出进行校验是否一致
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

*/
//----------------------------------------------------------------------------------
GX_API GXImportConfigFileW(GX_DEV_HANDLE hDevice, const wchar_t * pszFilePath, bool bVerify = false);

//----------------------------------------------------------------------------------
/**
\brief      读用户指定寄存器的值
\param [in]hDevice         设备句柄
\param [in]ui64Address     寄存器地址
\param [out]pBuffer        返回寄存器的值，不能为NULL
\param [in, out]piSize     [in]用户申请的Buffer大小
                           [out]成功读取寄存器的值后，返回实际大小
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

*/
//----------------------------------------------------------------------------------
GX_API GXReadRemoteDevicePort(GX_DEV_HANDLE hDevice, uint64_t ui64Address, void *pBuffer, size_t *piSize);


//----------------------------------------------------------------------------------
/**
\brief      向用户指定的寄存器中写入用户给定的数据
\param [in]hDevice         设备句柄
\param [in]ui64Address     寄存器地址
\param [in]pBuffer         返回寄存器的值，不能为NULL
\param [in, out]piSize     [in]用户要写入的Buffer长度
                           [out]返回实际写入寄存器的长度
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

*/
//----------------------------------------------------------------------------------
GX_API GXWriteRemoteDevicePort(GX_DEV_HANDLE hDevice, uint64_t ui64Address, const void *pBuffer, size_t *piSize);

// ---------------------------------------------------------------------------
		/**
		\brief       设置远端设备静态（永久）IP地址
		\param[in]   pszDevcieMacAddress  设备MAC地址
		\param[in]   ui32IpConfigFlag     ip配置方式（静态IP、DHCP、LLA、默认方式）
		\param[in]   pszIPAddress         设备IP地址
		\param[in]   pszSubnetMask        子网掩码
		\param[in]   pszDefaultGateway    网关
		\param[in]   pszUserID            用户自定义名称

		\retrun      GX_STATUS_SUCCESS             操作成功，没有发生错误
					GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
					GX_STATUS_INVALID_PARAMETER   无效参数
					GX_STATUS_NOT_FOUND_DEVICE    没有找到设备
					GX_STATUS_ERROR               操作失败
					GX_STATUS_INVALID_ACCESS      拒绝访问
					GX_STATUS_TIMEOUT             操作超时
					GC_ERR_IO                     IO通讯错误
					GC_ERR_INVALID_ID             ID无法与资源建立连接
		*/
// ---------------------------------------------------------------------------
GX_API GXGigEIpConfiguration(const char* pszDeviceMacAddress, 
							 GX_IP_CONFIGURE_MODE emIpConfigMode,
							 const char* pszIpAddress,
							 const char* pszSubnetMask, 
							 const char* pszDefaultGateway,
							 const char* pszUserID);


// ---------------------------------------------------------------------------
		/**
		\brief       ForceIP
		\param[in]   pszDevcieMacAddress  设备MAC地址
		\param[in]   pszIPAddress         设备IP地址
		\param[in]   pszSubnetMask        子网掩码
		\param[in]   pszDefaultGateway    网关

		\retrun      GX_STATUS_SUCCESS             操作成功，没有发生错误
					GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
					GX_STATUS_INVALID_PARAMETER   无效参数
					GX_STATUS_NOT_FOUND_DEVICE    没有找到设备
					GX_STATUS_ERROR               操作失败
					GX_STATUS_INVALID_ACCESS      拒绝访问
					GX_STATUS_TIMEOUT             操作超时
					GC_ERR_IO                     IO通讯错误
					GC_ERR_INVALID_ID             ID无法与资源建立连接
		*/
// ---------------------------------------------------------------------------
GX_API GXGigEForceIp(const char* pszDeviceMacAddress, 
							 const char* pszIpAddress,
							 const char* pszSubnetMask, 
							 const char* pszDefaultGateway);

//----------------------------------------------------------------------------------
/**
\brief      用户设置采集buffer个数
\param [in]hDevice         设备句柄
\param [in]nBufferNum      用户设置的buffer个数
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_INVALID_PARAMETER    输入参数无效
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

*/
//----------------------------------------------------------------------------------
GX_API GXSetAcqusitionBufferNumber(GX_DEV_HANDLE hDevice, uint64_t nBufferNum);

// ---------------------------------------------------------------------------
		/**
		\brief       重连/复位
		\param[in]   pszDevcieMacAddress  设备MAC地址
		\param[in]   ui32FeatureInfo      重置设备模式

		\retrun      GX_STATUS_SUCCESS             操作成功，没有发生错误
					GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
					GX_STATUS_INVALID_PARAMETER   无效参数
					GX_STATUS_NOT_FOUND_DEVICE    没有找到设备
					GX_STATUS_ERROR               操作失败
					GX_STATUS_INVALID_ACCESS      拒绝访问
					GX_STATUS_TIMEOUT             操作超时
					GC_ERR_IO                     IO通讯错误
					GC_ERR_INVALID_ID             ID无法与资源建立连接
		*/
// ---------------------------------------------------------------------------
GX_API GXGigEResetDevice(const char* pszDeviceMacAddress, GX_RESET_DEVICE_MODE ui32FeatureInfo);


// ---------------------------------------------------------------------------
		/**
		\brief       获取最优包长值
		\param[out]  punPacketSize		 最优包长值

		\retrun     GX_STATUS_SUCCESS             操作成功，没有发生错误
					GX_STATUS_TIMEOUT             操作超时
					GC_ERR_IO                     IO通讯错误
					GX_STATUS_INVALID_PARAMETER   无效参数
					GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
					GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		*/
// ---------------------------------------------------------------------------
GX_API  GXGetOptimalPacketSize (GX_DEV_HANDLE hDevice,uint32_t* punPacketSize);

#endif  //GX_GALAXY_H