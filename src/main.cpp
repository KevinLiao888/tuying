#include <iostream>
#include <aris.hpp>
#include "kaanh.h"
#include<atomic>
#include<string>
#include<filesystem>
#include<chrono>


#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library


// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1
#define DXL_ID1                         1                   // Dynamixel ID: 1
#define DXL_ID2                         2 
#define DXL_ID3                         3 
#define BAUDRATE                        57600
#define BAUDRATE1                       1000000
#define BAUDRATE2                       1000000
#define BAUDRATE3                       1000000

/*
const int DXL_ID1 = 1;
const int DXL_ID2 = 2;
const int DXL_ID3 = 3;
const int BAUDRATE = 57600;
const int BAUDRATE1 = 57600;
const int BAUDRATE2 = 1000000;
const int BAUDRATE3 = 1000000;
*/

#ifdef UNIX
    #define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                                // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#endif

#ifdef WIN32
    #define DEVICENAME                      "COM6"
#endif

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      -28672                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      28672                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b
#define SCALING                         11.378

std::thread t_dynamixel;
std::mutex dynamixel_mutex;


std::atomic_int syn_clock = 0;
std::atomic_int mode_dynamixel = 2;		//mode_dynamixel——0:manual, 1:auto, 2:NA
std::atomic_bool find_zero = false;		//找零
std::atomic_bool enable_dynamixel_auto = false;		//enable_dynamixel_auto——0:disable dynamixel auto, 1:enable dynamixel auto
std::atomic_bool enable_dynamixel_manual = false;		//enable_dynamixel_manual——0:disable dynamixel manual, 1:enable dynamixel manual
std::atomic_int is_enabled = 2;		//is_enabled——0:disable, 1:enable, 2:NA
std::atomic_int16_t target_pos1 = 0, target_pos2 = 0, target_pos3 = 0;
std::atomic_int16_t current_pos1 = 0, current_pos2 = 0, current_pos3 = 0;
std::vector<std::vector<double>> dxl_pos;
bool dxl_addparam_result = false;                 // addParam result
bool dxl_getdata_result = false;                  // GetParam result
//state code//
std::atomic_bool dxl_connected = 0;	//0:未连接，1:连接
std::atomic_bool dxl_enabled = 0;	//0:未使能，1:使能
std::atomic_bool dxl_auto = 0;		//0:手动，1:自动
std::atomic_int dxl_normal = 1;		//0:异常，1:正常


int getch()
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}
int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}
auto enable_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate)->bool
{
    uint8_t dxl_error = 0;
    uint16_t state = 1;
    // Set port baudrate for Dynamixel
    if (portHandler->setBaudRate(baudrate))
    {
        state = 1;
        std::cout << "Succeeded to change the baudrate!" << std::endl;
    }
    else
    {
        //std::cout << "Failed to change the baudrate!" << std::endl;
        getch();
        state = 0;
        return 0;
    }
    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        //std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        state = 0;
        return 0;
    }
    else if (dxl_error != 0)
    {
        //std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
        state = 0;
        return 0;
    }
    else
    {
        //std::cout << "Dynamixel " << dxl_id << " has been successfully connected" << std::endl;
    }
    dxl_normal.store(state);
    return 1;
}
auto read_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate, uint16_t &dxl_present_position)->bool
{
    uint8_t dxl_error = 0;
    uint16_t state = 1;
    if (portHandler->setBaudRate(baudrate))
    {
        state = 1;
        //std::cout << "Succeeded to change the baudrate!" << std::endl;
    }
    else
    {
        //std::cout << "Failed to change the baudrate!" << std::endl;
        getch();
        state = 0;
        return 0;
    }
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        //printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        state = 0;
        return 0;
    }
    else if (dxl_error != 0)
    {
        //printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        state = 0;
        return 0;
    }
    dxl_normal.store(state);
    return 1;

}
auto write_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate, int target_pos)->bool
{
    uint8_t dxl_error = 0;
    uint16_t state = 1;
    if (portHandler->setBaudRate(baudrate))
    {
        state = 1;
        //std::cout << "Succeeded to change the baudrate!" << std::endl;
    }
    else
    {
        //std::cout << "Failed to change the baudrate!" << std::endl;
        getch();
        state = 0;
        return 0;
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_MX_GOAL_POSITION, target_pos, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        //printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        state = 0;
        return 0;
    }
    else if (dxl_error != 0)
    {
        //printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        state = 0;
        return 0;
    }
    dxl_normal.store(state);
    return 1;

}
auto disable_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate)->bool
{
    uint8_t dxl_error = 0;
    uint16_t state = 1;
    // Set port baudrate for Dynamixel
    if (portHandler->setBaudRate(baudrate))
    {
        state = 1;
        //std::cout << "Succeeded to change the baudrate!" << std::endl;
    }
    else
    {
        //std::cout << "Failed to change the baudrate!" << std::endl;
        getch();
        state = 0;
        return 0;
    }
    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        //std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        state = 0;
        return 0;
    }
    else if (dxl_error != 0)
    {
        //std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
        state = 0;
        return 0;
    }
    else
    {
        //std::cout << "Dynamixel " << dxl_id << " has been successfully connected" << std::endl;
    }
    dxl_normal.store(state);
    return 1;
}


std::atomic_bool is_automatic = false;
using namespace aris::dynamic;
//global vel//
kaanh::Speed g_vel;
std::atomic_int g_vel_percent = 0;
//global vel//

auto xmlpath = std::filesystem::absolute(".");//获取当前工程所在的路径
auto uixmlpath = std::filesystem::absolute(".");
auto modelxmlpath = std::filesystem::absolute(".");
const std::string xmlfile = "kaanh.xml";
const std::string uixmlfile = "interface_kaanh.xml";
//for qifan robot//
//const std::string modelxmlfile = "model_qifan.xml";
const std::string modelxmlfile = "model_rokae.xml";


int main(int argc, char *argv[])
{
	//指定线程cpu号
	/*
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(1, &mask);
    CPU_SET(2, &mask);
	*/

    //Start t_Dynamixel thread//
    t_dynamixel = std::thread([&]()->bool
    {
		//指定线程运行cpu号
        //pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);

        // Initialize PortHandler instance
        // Set the port path
        // Get methods and members of PortHandlerLinux or PortHandlerWindows
        dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

        // Initialize PacketHandler instance
        // Set the protocol version
        // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        //dynamixel::PacketHandler *packetHandler;

		  // Initialize GroupSyncWrite instance
		dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);

		// Initialize GroupBulkRead instance
		dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

        int index = 0;
        int dxl_comm_result1 = COMM_TX_FAIL, dxl_comm_result2 = COMM_TX_FAIL, dxl_comm_result3 = COMM_TX_FAIL;             // Communication result
        //int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position
        int16_t dxl_goal_position = 0;
        uint8_t dxl_error = 0;                          // Dynamixel error
        uint16_t dxl_present_position1 = 0, dxl_present_position2 = 0, dxl_present_position3 = 0;

        // Open port //
        if (portHandler->openPort())
        {
            std::cout << "Succeeded to open the port!" << std::endl;
			dxl_connected.store(1);
        }
        else
        {
			dxl_connected.store(0);
            std::cout << "Failed to open the port!" << std::endl;
            std::cout << "Press any key to terminate...!" << std::endl;
            getch();
            return 0;
        }
        while (1)
        {
            auto en = is_enabled.load();
            auto mode = mode_dynamixel.load();
            static bool enabled = false;
            try
            {
                // Enable dynamixel1, dynamixel2, dynamixel3 //
                if (en == 1)
                {
                    if (!enable_dynamixel(portHandler, packetHandler, dxl_comm_result1, DXL_ID1, BAUDRATE1))return 0;
                    //if (!enable_dynamixel(portHandler, packetHandler, dxl_comm_result2, DXL_ID2, BAUDRATE2))return 0;
                    //if (!enable_dynamixel(portHandler, packetHandler, dxl_comm_result3, DXL_ID3, BAUDRATE3))return 0;

                    /*
					// Add parameter storage for Dynamixel#1 present position value
					dxl_addparam_result = groupBulkRead.addParam(DXL_ID1, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
					if (dxl_addparam_result != true)
					{
						fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL_ID1);
						continue;
					}

					// Add parameter storage for Dynamixel#2 present position value
					dxl_addparam_result = groupBulkRead.addParam(DXL_ID2, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
					if (dxl_addparam_result != true)
					{
						fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL_ID2);
						continue;
					}

					// Add parameter storage for Dynamixel#3 present position value
					dxl_addparam_result = groupBulkRead.addParam(DXL_ID3, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
					if (dxl_addparam_result != true)
					{
						fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL_ID3);
						continue;
					}
					*/
                    
                    read_dynamixel(portHandler, packetHandler, dxl_comm_result1, DXL_ID1, BAUDRATE1, dxl_present_position1);
                    //read_dynamixel(portHandler, packetHandler, dxl_comm_result2, DXL_ID2, BAUDRATE2, dxl_present_position2);
                    //read_dynamixel(portHandler, packetHandler, dxl_comm_result3, DXL_ID3, BAUDRATE3, dxl_present_position3);
                    dxl_present_position1 = std::int16_t(dxl_present_position1);
                    dxl_present_position2 = std::int16_t(dxl_present_position2);
                    dxl_present_position3 = std::int16_t(dxl_present_position3);
                    std::cout << "pos1:" << dxl_present_position1 << std::endl;
					std::cout << "pos2:" << dxl_present_position2 << std::endl;
                    std::cout << "pos3:" << dxl_present_position3 << std::endl;
                    target_pos1.store(dxl_present_position1);
                    //target_pos2.store(dxl_present_position2);
                    //target_pos3.store(dxl_present_position3);
                    is_enabled.store(2);
                    enabled = true;
					dxl_enabled.store(true);
                }
                // Disable //
                else if (en == 0)
                {
                    if (!disable_dynamixel(portHandler, packetHandler, dxl_comm_result1, DXL_ID1, BAUDRATE1))return 0;
                    //if (!disable_dynamixel(portHandler, packetHandler, dxl_comm_result2, DXL_ID2, BAUDRATE2))return 0;
                    //if (!disable_dynamixel(portHandler, packetHandler, dxl_comm_result3, DXL_ID3, BAUDRATE3))return 0;
                    is_enabled.store(2);
                    enabled = false;
					dxl_enabled.store(false);
                }
                if (enabled)
                {
                    // manual mode //
                    if (mode == 0)
                    {
						dxl_auto.store(false);
                        if (enable_dynamixel_manual.exchange(false))
                        {
                            auto pos1 = target_pos1.load();
                            //auto pos2 = target_pos2.load();
                            //auto pos3 = target_pos3.load();
                            write_dynamixel(portHandler, packetHandler, dxl_comm_result1, DXL_ID1, BAUDRATE1, pos1);
                            //write_dynamixel(portHandler, packetHandler, dxl_comm_result2, DXL_ID2, BAUDRATE2, pos2);
                           //write_dynamixel(portHandler, packetHandler, dxl_comm_result3, DXL_ID3, BAUDRATE3, pos3);
                        }
                    }
                    // auto mode //
                    else if (mode == 1)
                    {
						dxl_auto.store(true);
                        if (enable_dynamixel_auto.exchange(false))
                        {
                            static int dxl_couter;
                            dxl_couter = 0;

                            while(1)
                            {
                                if(syn_clock.load() ==0)
                                {
                                     std::this_thread::sleep_for(std::chrono::microseconds(500));
                                     continue;
                                }
                                if(syn_clock.load()>0)
                                {
                                    std::unique_lock<std::mutex> run_lock(dynamixel_mutex);
									uint8_t param_goal_position[2];
                                    bool dxl1_active = !dxl_pos[0].empty(), dxl2_active = !dxl_pos[1].empty(), dxl3_active = !dxl_pos[2].empty();
                                    auto data_length = std::max(std::max(dxl_pos[0].size(), dxl_pos[1].size()), dxl_pos[2].size());
									//syncwrite function//
                                    auto start1 = std::chrono::system_clock::now();
									// Allocate goal position value into byte array
									if (dxl1_active)
									{
                                        dxl_goal_position = std::int16_t(dxl_pos[0][dxl_couter]);
										param_goal_position[0] = DXL_LOBYTE(dxl_goal_position);
										param_goal_position[1] = DXL_HIBYTE(dxl_goal_position);
										dxl_addparam_result = groupSyncWrite.addParam(DXL_ID1, param_goal_position);
										if (dxl_addparam_result != true)
										{
											fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID1);
											break;
										}
									}
									if (dxl2_active)
									{
										dxl_goal_position = dxl_pos[1][dxl_couter];
										param_goal_position[0] = DXL_LOBYTE(dxl_goal_position);
										param_goal_position[1] = DXL_HIBYTE(dxl_goal_position);
										dxl_addparam_result = groupSyncWrite.addParam(DXL_ID2, param_goal_position);
										if (dxl_addparam_result != true)
										{
											fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID2);
											break;
										}
									}
									if (dxl3_active)
									{
										dxl_goal_position = dxl_pos[2][dxl_couter];
										param_goal_position[0] = DXL_LOBYTE(dxl_goal_position);
										param_goal_position[1] = DXL_HIBYTE(dxl_goal_position);
										dxl_addparam_result = groupSyncWrite.addParam(DXL_ID3, param_goal_position);
										if (dxl_addparam_result != true)
										{
											fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID3);
											break;
										}
									}
									// Syncwrite goal position
									auto dxl_comm_result = groupSyncWrite.txPacket();
									if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
									// Clear syncwrite parameter storage
									groupSyncWrite.clearParam();
                                    auto start2 = std::chrono::system_clock::now();

									//syncread function//
                                    /*
                                    dxl_comm_result = groupBulkRead.txRxPacket();
									if (dxl1_active)
									{
										dxl_getdata_result = groupBulkRead.isAvailable(DXL_ID1, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
										if (dxl_getdata_result != true)
										{
											fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL_ID1);
											continue;
										}
										// Get Dynamixel#1 present position value
										dxl_present_position1 = groupBulkRead.getData(DXL_ID1, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
										auto dxl1 = std::int16_t(dxl_present_position1);
										current_pos1.store(1.0*dxl1 / SCALING);
									}                                    
									if (dxl2_active)
									{
										dxl_getdata_result = groupBulkRead.isAvailable(DXL_ID2, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
										if (dxl_getdata_result != true)
										{
											fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL_ID2);
											continue;
										}
										// Get Dynamixel#2 present position value
										dxl_present_position2 = groupBulkRead.getData(DXL_ID2, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
										auto dxl2 = std::int16_t(dxl_present_position2);
										current_pos2.store(1.0*dxl2 / SCALING);
									}
									if (dxl3_active)
									{
										dxl_getdata_result = groupBulkRead.isAvailable(DXL_ID3, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
										if (dxl_getdata_result != true)
										{
											fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL_ID3);
											continue;
										}
										// Get Dynamixel#3 present position value
										dxl_present_position3 = groupBulkRead.getData(DXL_ID3, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
										auto dxl3 = std::int16_t(dxl_present_position3);
										current_pos3.store(1.0*dxl3 / SCALING);
									}
                                    */

                                    if(dxl_couter>=data_length-1) break;

									syn_clock--;//10ms计时标记位
									dxl_couter++;//emily舵机位置指向变量
                                    auto end = std::chrono::system_clock::now();
                                    auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(start2 - start1);
                                    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end - start2);
                                    std::cout <<  "syncwrite:" << "\t" <<double(duration1.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den   << "s"
                                        << "\t" << "syncread:"<< "\t" << double(duration2.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den   << "s"
                                                              << std::endl;
                                }
                            }
                        }
                        else {}
                    }

                    // Read the position of dynamixel1, dynamixel2, dynamixel3 //
                    read_dynamixel(portHandler, packetHandler, dxl_comm_result1, DXL_ID1, BAUDRATE1, dxl_present_position1);
                    //read_dynamixel(portHandler, packetHandler, dxl_comm_result2, DXL_ID2, BAUDRATE2, dxl_present_position2);
                    //read_dynamixel(portHandler, packetHandler, dxl_comm_result3, DXL_ID3, BAUDRATE3, dxl_present_position3);
                    auto dxl1 = std::int16_t(dxl_present_position1);
                    auto dxl2 = std::int16_t(dxl_present_position2);
                    auto dxl3 = std::int16_t(dxl_present_position3);
                    current_pos1.store(1.0*dxl1 / SCALING);
                    //current_pos2.store(1.0*dxl2 / SCALING);
                    //current_pos3.store(1.0*dxl3 / SCALING);
                }
            }
            catch (std::exception &e)
            {
				dxl_normal.store(0);
                std::cout << e.what() << std::endl;
                LOG_ERROR << e.what() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Close port //
        portHandler->closePort();
    });
	
    std::cout <<"new"<<std::endl;
    xmlpath = xmlpath / xmlfile;
	uixmlpath = uixmlpath / uixmlfile;
	modelxmlpath = modelxmlpath / modelxmlfile;
    std::cout<< xmlpath <<std::endl;
	auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);


	//生成kaanh.xml文档	
/*
	//-------for qifan robot begin//
	cs.resetController(kaanh::createControllerQifan().release());
	cs.resetModel(kaanh::createModelQifan().release());
	cs.resetPlanRoot(kaanh::createPlanRootRokaeXB4().release());
	cs.interfacePool().add<aris::server::WebInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
    cs.interfacePool().add<aris::server::WebInterface>("", "5868", aris::core::Socket::TCP);
    cs.interfacePool().add<aris::server::WebInterface>("", "5869", aris::core::Socket::TCP);
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
    cs.model().loadXmlFile(modelxmlpath.string().c_str());
    cs.interfaceRoot().loadXmlFile(uixmlpath.string().c_str());
	cs.saveXmlFile(xmlpath.string().c_str());
	//-------for qifan robot end// 
*/

	//-------for rokae robot begin//
	cs.resetController(kaanh::createControllerRokaeXB4().release());
	cs.resetModel(kaanh::createModelRokae().release());
	cs.resetPlanRoot(kaanh::createPlanRootRokaeXB4().release());
	cs.interfacePool().add<aris::server::WebInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
    cs.model().loadXmlFile(modelxmlpath.string().c_str());
	cs.interfaceRoot().loadXmlFile(uixmlpath.string().c_str());
	cs.saveXmlFile(xmlpath.string().c_str());
	//-------for rokae robot end// 


    cs.loadXmlFile(xmlpath.string().c_str());

    cs.start();

	//加载v100的速度值//
    auto &getspeed = dynamic_cast<aris::dynamic::MatrixVariable &>(*cs.model().variablePool().findByName("v100"));
	kaanh::SpeedParam speed;
	std::copy(getspeed.data().begin(), getspeed.data().end(), &speed.w_percent);
	speed.w_tcp = speed.w_tcp * speed.w_percent;
	g_vel.setspeed(speed);

	//Start Web Socket//
    cs.open();
	
	//Receive Command//
    cs.runCmdLine();

	return 0;
}
