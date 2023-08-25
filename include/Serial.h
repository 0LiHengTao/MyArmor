#pragma once
#include<iostream>
#include<fstream>
#include<stdio.h>
#include<stdint.h>
#include<mutex>
#include<chrono>
#include<thread>

#undef EOF

struct ControlData
{
	uint8_t frame_seq;
	uint16_t shoot_mode;//高八位发射方式 低八位发射速度等级
	float pitch_dev;//Pitch目标角度
	float yaw_dev;//Yaw目标角度
	//哨兵
	int16_t rail_speed;//目标轨道速度
	uint8_t gimbal_mode;//哨兵云台攻击模式

};
struct  FeedBackData
{
	uint8_t task_mode; //所需控制模式
	uint8_t bullet_speed;//射速
	//哨兵
	uint8_t rail_pos;//所处轨道标号
	uint8_t show_armor;//被打击装甲板标志
	uint16_t remain_HP;//剩余血量
};
class Serial
{
public:
	enum TeamName {
		BLUE_TEAM=(uint16_t)0xDDDDD,
		RED_TEAM=(uint16_t)0xEEEE
	};
	enum ControlMode {
		SET_UP=(uint16_t)0xCCCC,
		RECORD_ANGLE=(uint16_t)0xFFFF,
		REQUEST_TRANS=(uint16_t)0xBBBB
	};
	//发射方式
	enum ShootMode {
		NO_FIRE=(uint16_t)(0x00<<8),//不发射
		SINGLE_FIRE=(uint16_t)(0x01<<8),//点射
		BURST_FIRE=(uint16_t)(0x02<<8)//连发
	};
	//发射速度
	enum BulletSpeed {
		HIGH_SPEED=(uint16_t)(0x01),//高速
		LOW_SPEED=(uint16_t)(0x02)//低速
	};
	//控制模式
	enum TaskMode {
		NO_TASK = (uint8_t)(0x00),    //手动控制
		SMALL_BUFF = (uint8_t)(0x01),    //小符模式
		BIG_BUFF = (uint8_t)(0x02),    //大符模式
		AUTO_SHOOT = (uint8_t)(0x03)     //自动射击
	};
	//哨兵工作模式
	enum GimbalMode {
		PATROL_AROUND = (uint8_t)(0x01),    //旋转巡逻
		PATROL_ARMOR_0 = (uint8_t)(0x02),    //巡逻装甲板0
		PATROL_ARMOR_1 = (uint8_t)(0x03),    //巡逻装甲板1
		SERVO_MODE = (uint8_t)(0x04)     //伺服打击
	};
	enum ErrorCode {
		SYSTEM_ERROR = 1,  //系统错误
		OK = 0,  //一切正常，没有发生错误
		PORT_OCCUPIED = -1, //端口被占用
		READ_WRITE_ERROR = -2, //读写错误
		CORRUPTED_FRAME = -3, //数据帧损坏
		TIME_OUT = -4  //操作超时
	};
public:
	Serial();
	Serial(const Serial& right) = delete;  //删除拷贝构造
	Serial(Serial&&) = delete; //删除移动构造
	~Serial();

	int openPort();  //打开串口并进行配置
	int closePort();  //关闭串口
	bool isOpened() const;  //检查串口是否已打开
	void setDebug(const bool en_debug);  //设置是否启用调式模式
	int setup(int& self_color);  //与STM建立通信，通过设置self_color来确定红蓝方

	//尝试记录当前角度，使用给定时间限制"time_duration"进行记录，并将记录的帧序号保存在"frame_seq"中
	template<class _Rep,class _Period>
	int tryRecord(uint8_t& frame_seq, const std::chrono::duration<_Rep, _Period>& time_duration) {
		std::unique_lock<std::timed_mutex> lockGuard(_mutex, time_duration);
		if (lockGuard.owns_lock()) {
			record(frame_seq);
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			return _errorCode;
		} else {
			return PORT_OCCUPIED;
		}
	}

	//尝试接收反馈信息。使用给定的时间限制 time_duration 进行接收操作，并将反馈数据保存在 feedBackData 中
	template<class _Rep,class _Period>
	int tryFeedBack(FeedBackData& feedBackData, const std::chrono::duration<_Rep, _Period>& time_duration) {
		std::unique_lock<std::timed_mutex> lockGuard(_mutex, time_duration);
		if (lockGuard.owns_lock()) {
			feedBack(feedBackData);
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			return _errorCode;
		} else {
			return PORT_OCCUPIED;
		}
	}

	//获取错误代码。返回最近操作的错误代码，不保证线程安全
	int getErrorCode() const;
private:
	enum {
		JetsonCommSOF = (uint8_t)0x66, //帧的开始
		JetsonCommEOF = (uint8_t)0x88  //帧的结束
	};
private:
	int _serialFd;  //串口的文件描述符
	int _errorCode;  //用于存储错误代码
	bool _en_debug;  //表示是否启用调式模式
	uint8_t _lastRecordSeq;  //存储上一次记录的帧序列号
	std::timed_mutex _mutex;  //互斥锁，保证在多线程环境下对串口的访问安全

	struct  ControlFrame {
		uint8_t  SOF; //帧的起始标志
		uint8_t  frame_seq; //帧序列号
		uint16_t shoot_mode; //发射模式
		float    pitch_dev; //Pitch目标角度
		float    yaw_dev; //Yaw目标角度
		int16_t  rail_speed; //目标轨迹速度
		uint8_t  gimbal_mode; //云台攻击模式
		uint8_t  EOF; //帧的结束标志
	}_controlFrame;
	
	//机器人接收的反馈数据帧
	struct  FeedBackFrame {
		uint8_t  SOF; //帧起始标志
		uint8_t  frame_seq; //帧序号
		uint8_t  task_mode; //控制模式
		uint8_t  bullet_speed; //子弹速度
		uint8_t  rail_pos; //轨道位置
		uint8_t  shot_armor; //被打击装甲板标志
		uint16_t remain_HP; //剩余血量
		uint8_t  reserved[11]; //保留字段，用于未来扩展或对齐
		uint8_t  EOF; //帧结束标志
	}_feedBackFrame;

	void print(const ControlFrame& ct);  //用于将控制帧和反馈帧的内容打印到输出流，方便调试
	void print(const FeedBackFrame& fb);

	//用于构造并且发送不同类型的数据帧，并在发送完成后处理相应操作——记录帧，控制帧，反馈帧
	int record(uint8_t& frame_seq);  
	int control(const ControlData& controlData);
	int feedBack(FeedBackData& feedBackData);

	//用于将控制数据和反馈数据打包成相应的帧结构体，或者从帧结构体中解包出控制数据和反馈数据
	ControlFrame pack(const ControlData& controlData);
	FeedBackData unpack(const FeedBackFrame& FeedBackData);

	//用于接收和发送数据，完成串口实际操作
	int send();
	int receive();
};
