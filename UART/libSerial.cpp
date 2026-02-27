#include <iostream>
#include <string>
#include <libserial/SerialPort.h>
 #include <unistd.h>

using namespace std;
int main()
{
    LibSerial::SerialPort serial_port;
    try {
        serial_port.Open("/dev/serial0");
    } catch (const LibSerial::OpenFailed&) {
        cerr << "open failed\n";
        return 1;
    }

    serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_port.SetParity(LibSerial::Parity::PARITY_NONE);

    string msg("Hello From RPI\n");
    while(1)
    {
        serial_port.Write(msg);
        string resp;
        try {
            // 줄바꿈이 나올 때까지 최대 100ms 대기
            serial_port.ReadLine(resp, '\n', 100);
            cout << "[Received] " << resp;
        } catch (const LibSerial::ReadTimeout&) {
            cerr << "Response Timeout\n";
        }
        sleep(1);
    }
    

    serial_port.Close();
    return 0;
}