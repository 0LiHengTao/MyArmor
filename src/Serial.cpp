#include "Serial.h"
#include"ArmorDector.h"
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <fstream>
#include <string>
#include<string.h>
#include <stdexcept>
#include<exception>

Serial::Serial() :
	_serialFd(-1),
	_errorCode(OK),
	_en_debug(false),
	_lastRecordSeq(0) {
	//静态断言，用于在编译时检查条件是否满足。这里用来确保ControlFrame和FeedBackFrame结构体的大小分别是16和20字节。如果条件不满足，编译将会失败，并显示后面的错误消息
	static_assert(sizeof(ControlFrame) == 16, "Size of backdata is not 16");
	static_assert(sizeof(FeedBackFrame) == 20, "Size of backdata is not  20");

	//初始化通讯帧的起始标志，帧序列号和结束标志
	_controlFrame.SOF == JetsonCommSOF;
	_controlFrame.frame_seq = 0;
	_controlFrame.EOF = JetsonCommEOF;
}

Serial::~Serial() {

	tcflush(_serialFd,TCIOFLUSH);  //清空串口输入输出缓冲区
	if (-1 == close(_serialFd)) {  // 关闭串口
		_errorCode = SYSTEM_ERROR;
		std::cout << "Serial closing failed" << std::endl;
	} else {
		_errorCode = OK;
	}
}

int Serial::openPort() {
	_serialFd = open("/dev/tty", O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (_serialFd == -1) {
		std::cout << "Open serial port failed" << std::endl;
		return _errorCode = SYSTEM_ERROR;
	}
	termios tOption;  //串口配置结构体
	tcgetattr(_serialFd, tOption);  //获取当前串口设置
	cfmakeraw(&tOption); //将串口设置初始化为原始模式，不经过特殊处理

	//设置接受和发送波特率为B460800
	cfsetispeed(&tOption, B460800);
	cfsetospeed(&tOption, B460800);

	tcsetattr(_serialFd, TCSANOW, &tOption);  //设置串口属性

	//对串口的控制标志进行设置
	tOption.c_cflag &= ~PARENB;                    // 关闭奇偶校验
	tOption.c_cflag &= ~CSTOPB;                    // 设置停止位为1
	tOption.c_cflag &= ~CSIZE;                     // 清除数据位设置
	tOption.c_cflag |= CS8;                        // 设置数据位为8位
	tOption.c_cflag &= ~INPCK;                     // 禁用输入奇偶校验
	tOption.c_cflag |= (B460800 | CLOCAL | CREAD); // 设置波特率、本地连接和接收使能
	tOption.c_cflag &= ~(INLCR | ICRNL);           // 禁用输入换行符转换
	tOption.c_cflag &= ~(IXON);                    // 禁用输入软流控制

	// 对串口的本地标志和输入输出标志进行设置
	tOption.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 禁用规范模式和回显
	tOption.c_oflag &= ~OPOST;                      // 禁用输出处理
	tOption.c_oflag &= ~(ONLCR | OCRNL);            // 禁用输出换行符转换
	tOption.c_iflag &= ~(ICRNL | INLCR);            // 禁用输入换行符转换
	tOption.c_iflag &= ~(IXON | IXOFF | IXANY);     // 禁用输入软流控制

	// 设置读取超时时间和最小读取字节数
	tOption.c_cc[VTIME] = 1;                        // 读取超时时间设置为 0.1 秒
	tOption.c_cc[VMIN] = 1;                         // 最小读取字节数设置为 1

	tcflush(_serialFd, TCIOFLUSH);                  // 刷新输入输出队列

	std::cout << "Serial preparation complete." << std::endl;
	return _errorCode = OK;                         // 返回 OJBK 表示操作成功
}

int Serial::closePort() {
	tcflush(_serialFd, TCIOFLUSH);
	if (-1 == close(_serial_Fd)) {
		_errorCode = _SYSTEM_ERROR;
		std::cout << "Serial closing failed" << std::endl;
	} else {
		_errorCode = OK;
	}
	return _errorCode;
}

bool Serial::isOpened() const {
	return (_serialFd!=-1);
}

void Serial::setDebug(bool en_debug) {
	_en_debug = en_debug;
}

int Serial::setup(int& self_color) {
	if (_en_debug) {
		std::cout << "[setup]\n";
	}
	uint16_t tem_self_team;
	//设置发射模式
	_controlFrame.shoot_mode = SET_UP;
	//发送控制帧，进行通信
	if (send() == OK) {
		//接收反馈数据帧
		if (receive() == OK) {
			//解析反馈数据，获取任务模式和子弹速度
			tem_self_team = ((_feedBackFrame.task_mode << 8) | _feedBackFrame.bullet_speed);
			if (tem_self_team != BLUE_TEAM && tem_self_team != RED_TEAM) {
				return _errorCode = CORRUPTED_FRAME;
			}
			//更新发射模式为正确的红蓝方标志
			_controlFrame.shoot_mode = tem_self_team;
			//再次发送控制帧，将正确的发射模式发送给外部设备
			if (send() == OK) {
				//根据发射模式设置自身颜色标志
				if (tem_self_team == BLUE_TEAM) {
					self_color == BLUE;
				} else if(tem_self_team==RED_TEAM) {
					self_color = RED;
				}
			}
		}
	}
	return _errorCode;//返回操作状态码
}

int Serial::record(uint8_t& frame_seq) {
	if (_en_debug) {
		std::cout << "[record]\n";
	}
	_lastRecordSeq++;
	_controlFrame.frame_seq = _lastRecordSeq;
	_controlFrame.shoot_mode = RECORD_ANGLE;
	frame_seq = _lastRecordSeq;
	return send();
}

int Serial::control(const ControlData& controlData) {
	if (_en_debug) {
		std::cout << "[control]\n";
	}
	_controlFrame = pack(controlData);
	return send();
}
int Serial::feedBack(FeedBackData& feedBackData) {
	if (_en_debug) {
		std::cout << "[request]\n";
	}
	_controlFrame.shoot_mode = REQUEST_TRANS;
	if (send() == OK) {
		if (receive() == OK) {
			feedBackData = unpack(_feedBackFrame);
		}
	}
	return _errorCode;
}
int Serial::getErrorCode() const {
	return _errorCode;
}
void Serial::print(const ControlFrame& ct) {
	std::cout << std::hex << (unsigned int)ct.SOF << std::endl;
	std::cout << std::dec << (unsigned int)ct.frame_seq << std::endl;
	std::cout << std::hex << (unsigned int)ct.shoot_mode << std::endl;
	std::cout << std::dec << ct.pitch_dev << std::endl;
	std::cout << std::dec << ct.yaw_dev << std::endl;
	std::cout << std::hex << (int)ct.rail_speed << std::endl;
	std::cout << std::hex << (unsigned int)ct.gimbal_mode << std::endl;
	std::cout << std::hex << (unsigned int)ct.EOF << std::endl;
}

void Serial::print(const FeedBackFrame& fb) {
	std::cout << std::hex << (unsigned int)fb.SOF << std::endl;
	std::cout << std::dec << (unsigned int)fb.frame_seq << std::endl;
	std::cout << std::hex << (unsigned int)fb.task_mode << std::endl;
	std::cout << std::dec << (unsigned int)fb.bullet_speed << std::endl;
	std::cout << std::dec << (unsigned int)fb.rail_pos << std::endl;
	std::cout << std::dec << (unsigned int)fb.shot_armor << std::endl;
	std::cout << std::dec << (unsigned int)fb.remain_HP << std::endl;
	for (int i = 0; i < 11; i++) {
		std::cout << std::dec << (unsigned int)(fb.reserved[i]) << ", ";
	}
	std::cout << std::endl;
	std::cout << std::hex << (unsigned int)fb.EOF << std::endl;
}
Serial::ControlFrame Serial::pack(const ControlData& ctrl) {
	return ControlFrame {
		JetsonCommSOF,
		ctrl.frame_seq,
		ctrl.shoot_mode,
		ctrl.pitch_dev,
		ctrl.yaw_dev,
		ctrl.rail_speed,
		ctrl.gimbal_mode,
		JetsonCommEOF
	};
}

FeedBackData Serial::unpack(const Serial::FeedBackFrame& fb) {
	return FeedBackData {
		fb.task_mode,
		fb.bullet_speed,
		fb.rail_pos,
		fb.shot_armor,
		fb.remain_HP
	};
}

int Serial::send() {
	tcflush(_serialFd, TCOFLUSH);
	int sendCount;
	//捕获异常
	try {
		//将_controlFrame 中的数据写入串口，记录成功发送的字节
		sendCount = write(_serialFd, &_controlFrame, sizeof(ControlFrame));
	} catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
		return _errorCode = SYSTEM_ERROR;
	}
	if (sendCount == -1) {
		if (_en_debug) {
			std::cout << "\tSerial sending failed. Frame sequence" << (int)_controlFrame.frame_seq << std::endl;
		}
		_errorCode = READ_WRITE_ERROR;
	}
	//数据写入不完整
	else if (sendCount < static_cast<int>(sizeof(ControlFrame))) {
		if (_en_debug) {
			std::cout << "\tSerial sending failed." << sizeof(ControlFrame) - sendCount <<
				"bytes unsent. Frame sequence:" << (int)_controlFrame.frame_seq << std::endl;
		}
		_errorCode;
	} else {
		if (_en_debug) {
			std::cout << "\tSerial sending succeeded." << "Frame sequence:" << (int)_controlFrame.frame_seq << std::endl;
		}
		_errorCode=OK;
	}
	return _errorCode;
}

int Serial::receive() {
	memset(&_feedBackFrame, 0, sizeof(_feedBackFrame));
	int readCount = 0;
	const auto t1 = std::chrono::high_resolution_clock::now();
	while (readCount<int(sizeof(FeedBackFrame))) {
		auto t2 = std::chrono::high_resolution_clock::now();
		if ((std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)).count() > 10) {
			if (_en_debug) {
				std::cout << "\tReceive time out." << sizeof(FeedBackFrame) - readCount
					<< "bytes not receive.Frame sequence:" << (int)_feedBackFrame.frame_seq << std::endl;
			}
			return _errorCode = TIME_OUT;
		}
		int onceReadCount;
		try {
			onceReadCount = read(_serialFd, ((unsigned char*)(&_feedBackFrame)) + readCount, sizeof(FeedBackFrame) - readCount);
		}
		catch (const std::exception& e){
			std::cout << e.what() << std::endl;
			return _errorCode = SYSTEM_ERROR;
		}
		if (onceReadCount == -1) {
			if (errno == EAGAIN) {
				continue;
			}
			if (_en_debug) {
				std::cout << "\tRead data from serial failed. Frame sequence: " << (int)_feedBackFrame.frame_seq << std::endl;
			}
			return _errorCode = READ_WRITE_ERROR;
		}
		readCount += onceReadCount;
	}
	tcflush(_serialFd.TCIFLUSH);

	if (_feedBackFrame.SOF != JetsonCommSOF || _feedBackFrame.EOF != JetsonCommEOF) {
		if (_en_debug) {
			std::cout << "\tFeed back frame SOF or EOF is not correct. SOF: " << (int)_feedBackFrame.SOF << " ,EOF: " << (int)_feedBackFrame.EOF << std::endl;
		}
		return _errorCode = CORRUPTED_FRAME;
	} else {
		if (_en_debug) {
			std::cout << "\tSerial receiving succeeded. " << "Frame sequence: " << (int)_feedBackFrame.frame_seq << std::endl;
		}
		return _errorCode = OK;
	}
}
