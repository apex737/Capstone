#include <iostream>
#include <fcntl.h>   // 포트 제어 (open, O_RDWR 등)
#include <termios.h> // 시리얼 통신 설정
#include <unistd.h>  // read, write, close
#include <string.h>

class RaspberryUart {
private:
    int serial_port;

public:
    bool begin(const char* device, speed_t baudrate) {
        // 1. 시리얼 포트 열기
        if((serial_port = open(device, O_RDWR | O_NOCTTY, 0644)) == -1){
            std::cerr << "포트를 열 수 없습니다: " << device << std::endl;
            return false;
        }

        // 2. 설정 구조체 생성
        struct termios tty;
        if (tcgetattr(serial_port, &tty) != 0) {
            return false;
        }

        // 3. 통신 파라미터 설정
        tty.c_cflag &= ~PARENB;        // 패리티 비트 없음
        tty.c_cflag &= ~CSTOPB;        // 정지 비트 1개
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;            // 데이터 8비트
        tty.c_cflag &= ~CRTSCTS;       // 하드웨어 흐름제어 비활성화
        tty.c_cflag |= CREAD | CLOCAL; // 수신 활성화, 로컬 모드

        // 4. 속도(Baud rate) 설정
        cfsetispeed(&tty, baudrate);
        cfsetospeed(&tty, baudrate);

        // 5. 설정 적용
        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            return false;
        }
        tcflush(serial_port, TCIFLUSH);
        return true;
    }

    void send(const std::string& data) {
        write(serial_port, data.c_str(), data.size());
    }

    void receive() {
        static std::string buffer;
        char chBuf[256];

        int n = read(serial_port, chBuf, sizeof(chBuf));
        if (n > 0) 
        {
            for(int i = 0; i < n; i++)
            {
                if (chBuf[i] == '\n') {
                    std::cout << "[Recieved] " << buffer << std::endl;
                    buffer.clear();
                } else if (chBuf[i] != '\r') {
                    buffer += chBuf[i];
                }
            }
            
        }
    }

    ~RaspberryUart() {
        close(serial_port);
    }
};

int main() {
    RaspberryUart uart;
    
    // 라즈베리 파이 기본 UART 포트: /dev/serial0
    if (uart.begin("/dev/serial0", B115200)) {
        std::cout << "UART 시작!" << std::endl;
        
        while (true) {
            uart.send("Hello From RPi!\n");
            uart.receive();
        }
    }
    return 0;
}