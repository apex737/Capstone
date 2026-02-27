#include <iostream>
#include <boost/asio.hpp>

using namespace std;
using namespace boost::asio;

int main() {
    try {
        // 1. I/O 서비스 객체 생성
        boost::asio::io_service io;

        // 2. 시리얼 포트 객체 생성 (라즈베리 파이 기본 UART 포트)
        boost::asio::serial_port serial(io, "/dev/ttyAMA0");

        // 3. 포트 설정 (보드레이트, 패리티 등)
        serial.set_option(serial_port_base::baud_rate(115200));
        serial.set_option(serial_port_base::character_size(8));
        serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        serial.set_option(serial_port_base::parity(serial_port_base::parity::none));

        cout << "UART 포트 오픈 성공!" << endl;

        // 4. 데이터 쓰기 (Send)
        while(1)
        {
            string msg = "Hello FROM RPI\n";
            write(serial, buffer(msg));

            // 5. 데이터 읽기 (Receive)
            char data[128];
            size_t len = serial.read_some(buffer(data));
            
            cout << "[Recieved] ";
            cout.write(data, len);
            cout << endl;
            sleep(1);
        }
        

    } catch (boost::system::system_error& e) {
        cerr << "에러 발생: " << e.what() << endl;
        return 1;
    }

    return 0;
}