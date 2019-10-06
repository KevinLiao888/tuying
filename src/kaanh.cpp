#include <algorithm>
#include "kaanh.h"
#include <array>
#include <stdlib.h>


using namespace aris::dynamic;
using namespace aris::plan;

//global vel//
extern kaanh::Speed g_vel;
extern std::atomic_int g_vel_percent;
//global vel//

extern std::mutex dynamixel_mutex;
extern std::atomic_int mode_dynamixel;
extern std::atomic_bool enable_dynamixel_auto;
extern std::atomic_bool enable_dynamixel_manual;
extern std::atomic_int is_enabled;
extern std::atomic_int target_pos1, target_pos2, target_pos3;
extern std::atomic_int current_pos1, current_pos2, current_pos3;
extern std::vector<std::vector<double>> dxl_pos;

std::atomic<std::array<double, 10> > save_point;

kaanh::CmdListParam cmdparam;

namespace kaanh
{
	auto createControllerQifan()->std::unique_ptr<aris::control::Controller>
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

		for (aris::Size i = 0; i < 6; ++i)
		{
#ifdef WIN32
			double pos_offset[6]
			{
				0.0,   0.0,   0.0,   0.0,   0.0,   0.0
			};
#endif // WIN32
#ifdef UNIX
			double pos_offset[6]
			{
                -0.438460099997905,   1.01949192131931,   -1.00835441988747,   0.0315385382644258,   0.0943992339950059,   3.4310965015334
			};
#endif
			double pos_factor[6]
			{
                8388608.0 * 166 / 2 / PI, -8388608.0 * 166 / 2 / PI, 8388608.0 * 88 / 2 / PI, 8388608.0 * 80 / 2 / PI, 8388608.0 * 80 / 2 / PI, -8388608.0 * 80 / 2 / PI
			};
			double max_pos[6]
			{
				145.0 / 360 * 2 * PI, 110.0 / 360 * 2 * PI,	55.0 / 360 * 2 * PI, 122.0 / 360 * 2 * PI, 135.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
			};
			double min_pos[6]
			{
				-145.0 / 360 * 2 * PI, -60.0 / 360 * 2 * PI, -55.0 / 360 * 2 * PI, -122.0 / 360 * 2 * PI, -135.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				73.0 / 360 * 2 * PI, 73.0 / 360 * 2 * PI, 136.0 / 360 * 2 * PI, 225.0 / 360 * 2 * PI, 225.0 / 360 * 2 * PI, 225.0 / 360 * 2 * PI,
			};
			double max_acc[6]
			{
				350.0 / 360 * 2 * PI, 350.0 / 360 * 2 * PI, 700.0 / 360 * 2 * PI, 1100.0 / 360 * 2 * PI, 1100.0 / 360 * 2 * PI, 1100.0 / 360 * 2 * PI,
			};

			if (i == 0)
			{
				std::string xml_str =
					"<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380008\""
					" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
					" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
					" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
					" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
					"	<SyncManagerPoolObject>"
					"		<SyncManager is_tx=\"false\"/>"
					"		<SyncManager is_tx=\"true\"/>"
					"		<SyncManager is_tx=\"false\">"
					"			<Pdo index=\"0x1600\" is_tx=\"false\">"
					"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"		<SyncManager is_tx=\"true\">"
					"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
					"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"	</SyncManagerPoolObject>"
					"</EthercatMotion>";

				controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
			}
			else if (i == 1)
			{
				std::string xml_str =
					"<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380009\""
					" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
					" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
					" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
					" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
					"	<SyncManagerPoolObject>"
					"		<SyncManager is_tx=\"false\"/>"
					"		<SyncManager is_tx=\"true\"/>"
					"		<SyncManager is_tx=\"false\">"
					"			<Pdo index=\"0x1600\" is_tx=\"false\">"
					"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"		<SyncManager is_tx=\"true\">"
					"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
					"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"	</SyncManagerPoolObject>"
					"</EthercatMotion>";

				controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
			}
			else if (i == 2)
			{
				std::string xml_str =
					"<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380008\""
					" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
					" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
					" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
					" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
					"	<SyncManagerPoolObject>"
					"		<SyncManager is_tx=\"false\"/>"
					"		<SyncManager is_tx=\"true\"/>"
					"		<SyncManager is_tx=\"false\">"
					"			<Pdo index=\"0x1600\" is_tx=\"false\">"
					"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"		<SyncManager is_tx=\"true\">"
					"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
					"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"	</SyncManagerPoolObject>"
					"</EthercatMotion>";

				controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
			}
			else
			{
				std::string xml_str =
					"<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380006\""
					" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
					" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
					" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
					" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
					"	<SyncManagerPoolObject>"
					"		<SyncManager is_tx=\"false\"/>"
					"		<SyncManager is_tx=\"true\"/>"
					"		<SyncManager is_tx=\"false\">"
					"			<Pdo index=\"0x1600\" is_tx=\"false\">"
					"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"		<SyncManager is_tx=\"true\">"
					"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
					"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"	</SyncManagerPoolObject>"
					"</EthercatMotion>";

				controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
			}
        }
        /*
		double pos_offset = 500.0 / 8388608.0 * 250;	//在零位时，为500count
		double pos_factor = 8388608.0 * 250;	//运行1m需要转250转
		double max_pos = 10.0;
		double min_pos = 0.0;
		double max_vel = 1.0;	//1m/s
		double max_acc = 5.0;	//5m/s2
		std::string xml_str =
			"<EthercatMotion phy_id=\"" + std::to_string(6) + "\" product_code=\"0x6038000D\""
			" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
			" min_pos=\"" + std::to_string(min_pos) + "\" max_pos=\"" + std::to_string(max_pos) + "\" max_vel=\"" + std::to_string(max_vel) + "\" min_vel=\"" + std::to_string(-max_vel) + "\""
			" max_acc=\"" + std::to_string(max_acc) + "\" min_acc=\"" + std::to_string(-max_acc) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
			" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor) + "\" pos_offset=\"" + std::to_string(pos_offset) + "\">"
			"	<SyncManagerPoolObject>"
			"		<SyncManager is_tx=\"false\"/>"
			"		<SyncManager is_tx=\"true\"/>"
			"		<SyncManager is_tx=\"false\">"
			"			<Pdo index=\"0x1600\" is_tx=\"false\">"
			"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
			"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"		<SyncManager is_tx=\"true\">"
			"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
			"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
			"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"	</SyncManagerPoolObject>"
			"</EthercatMotion>";

		controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
        */

        return controller;
	};
	auto createModelQifan()->std::unique_ptr<aris::dynamic::Model>
	{
		aris::dynamic::PumaParam param;
		
		
		param.d1 = 0.596;
		param.a1 = 0.220;
		param.a2 = 1.020;
		param.d3 = 0.0;
		param.a3 = 0.0;
		param.d4 = 0.86;
		//param.d4 = 1.010;

		param.tool0_pe[2] = 0.153;

		auto model = aris::dynamic::createModelPuma(param);

		return std::move(model);
	}


	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
	{
		std::unique_ptr<aris::control::Controller> controller(aris::robot::createControllerRokaeXB4());/*创建std::unique_ptr实例*/

#ifdef UNIX

        dynamic_cast<aris::control::Motion&>(controller->slavePool()[0]).setPosOffset(0.083326167813560906);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[1]).setPosOffset(0.40688722035956698);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[2]).setPosOffset(-0.063596878644675794);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[3]).setPosOffset(0.65575523199999997);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[4]).setPosOffset(-1.49538803280913);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[5]).setPosOffset(-3.2476329105045001);

		for (int i = 0; i < 6; i++)
		{
			dynamic_cast<aris::control::EthercatMotion&>(controller->slavePool()[i]).setDcAssignActivate(0x300);
		}
		// controller->slavePool().add<aris::control::EthercatSlave>();
		// controller->slavePool().back().setPhyId(6);
		// dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
		// dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
		// dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).setDcAssignActivate(0x300);

		// controller->slavePool().add<aris::control::EthercatSlave>();
		// controller->slavePool().back().setPhyId(7);
		 //dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
		 //dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();

		controller->slavePool().add<aris::control::EthercatSlave>();
		controller->slavePool().back().setPhyId(6);
		dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
		dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
		dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).setDcAssignActivate(0x00);
#endif
/*
		double pos_offset = 0.0 / 8388608.0 * 250;	//在零位时，为500count
		double pos_factor = 8388608.0 * 250;	//运行1m需要转250转
		double max_pos = 10.0;
		double min_pos = 0.0;
		double max_vel = 1.0;	//1m/s
		double max_acc = 5.0;	//5m/s2

		std::string xml_str =
			"<EthercatMotion phy_id=\"" + std::to_string(6) + "\" product_code=\"0x6038000D\""
			" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
			" min_pos=\"" + std::to_string(min_pos) + "\" max_pos=\"" + std::to_string(max_pos) + "\" max_vel=\"" + std::to_string(max_vel) + "\" min_vel=\"" + std::to_string(-max_vel) + "\""
			" max_acc=\"" + std::to_string(max_acc) + "\" min_acc=\"" + std::to_string(-max_acc) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
			" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor) + "\" pos_offset=\"" + std::to_string(pos_offset) + "\">"
			"	<SyncManagerPoolObject>"
			"		<SyncManager is_tx=\"false\"/>"
			"		<SyncManager is_tx=\"true\"/>"
			"		<SyncManager is_tx=\"false\">"
			"			<Pdo index=\"0x1600\" is_tx=\"false\">"
			"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
			"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"		<SyncManager is_tx=\"true\">"
			"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
			"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
			"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"	</SyncManagerPoolObject>"
			"</EthercatMotion>";

		controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
*/
		return controller;
	};
	auto createModelRokae()->std::unique_ptr<aris::dynamic::Model>
	{
		aris::dynamic::PumaParam param;
		param.d1 = 0.3295;
		param.a1 = 0.04;
		param.a2 = 0.275;
		param.d3 = 0.0;
		param.a3 = 0.025;
		param.d4 = 0.28;

		param.tool0_pe[2] = 0.078;

		param.iv_vec =
		{
			{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.59026333537827,   0.00000000000000,   0.00000000000000,   0.00000000000000 },
			{ 0.00000000000000, -0.02551872200978,   0.00000000000000,   3.05660683326413,   2.85905166943306,   0.00000000000000,   0.00000000000000, -0.00855352993039, -0.09946674483372, -0.00712210734359 },
			{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.02733022277747,   0.00000000000000,   0.37382629693302,   0.00000000000000,   0.00312006493276, -0.00578410451516,   0.00570606128540 },
			{ 0.00000000000000,   1.06223330086669,   0.00000000000000,   0.00311748242960,   0.00000000000000,   0.24420385558544,   0.24970286555981,   0.00305759215246, -0.66644096559686,   0.00228253380852 },
			{ 0.00000000000000,   0.05362286897910,   0.00528925153464, -0.00842588023014,   0.00128498153337, -0.00389810210572,   0.00000000000000, -0.00223677867576, -0.03365036368035, -0.00415647085627 },
			{ 0.00000000000000,   0.00000000000000,   0.00066049870832,   0.00012563800445, -0.00085124094833,   0.04209529937135,   0.04102481443654, -0.00067596644891,   0.00017482449876, -0.00041025776053 },
		};

		param.mot_frc_vec =
		{
			{ 9.34994758321915, 7.80825641041495, 0.00000000000000 },
			{ 11.64080253106441, 13.26518528472506, 3.55567932576820 },
			{ 4.77014054273075, 7.85644357492508, 0.34445460269183 },
			{ 3.63141668516122, 3.35461524886318, 0.14824771620542 },
			{ 2.58310846982020, 1.41963212641879, 0.04855267273770 },
			{ 1.78373986219597, 0.31920640440152, 0.03381545544099 },
		};

		auto model = aris::dynamic::createModelPuma(param);
		/*
		//根据tool0，添加一个tool1，tool1相对于tool0在x方向加上0.1m//
		auto &tool0 = model->partPool().back().markerPool().findByName("general_motion_0_i");//获取tool0

		double pq_ee_i[7];
		s_pm2pq(*tool0->prtPm(), pq_ee_i);
		pq_ee_i[0] += 0.1;//在tool0的x方向加上0.1m

		double pm_ee_i[16];
		s_pq2pm(pq_ee_i, pm_ee_i);

		auto &tool1 = model->partPool().back().markerPool().add<Marker>("tool1", pm_ee_i);//添加tool1

		//在根据tool1位姿反解到每一个关节时，需要调用下面两行代码来实现
		//tool1.setPm(pm_ee_i);
		//model->generalMotionPool()[0].updMpm();
		*/
		return std::move(model);
	}


	// 获取part_pq，end_pq，end_pe等 //
	struct GetParam
	{
		std::vector<double> part_pq, end_pq, end_pe, motion_pos, motion_vel, motion_acc, motion_toq, ai;
		std::vector<bool> di;
		std::int32_t state_code;
		aris::control::EthercatController::SlaveLinkState sls[6];
		aris::control::EthercatController::MasterLinkState mls{};
		std::vector<int> motion_state;
		std::string currentplan;
		int vel_percent;
	};
	auto Get::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		GetParam par;
		par.part_pq.resize(target.model->partPool().size() * 7, 0.0);
		par.end_pq.resize(7, 0.0);
		par.end_pe.resize(6, 0.0);
		par.motion_pos.resize(10, 0.0);
		par.motion_vel.resize(6, 0.0);
		par.motion_acc.resize(6, 0.0);
		par.motion_toq.resize(6, 0.0);
		par.ai.resize(100, 1.0);
		par.di.resize(100, false);
		par.motion_state.resize(6, 0);
		std::any param = par;
		//std::any param = std::make_any<GetParam>();

		target.server->getRtData([&](aris::server::ControlServer& cs, const aris::plan::PlanTarget *target, std::any& data)->void
		{
			for (aris::Size i(-1); ++i < cs.model().partPool().size();)
			{
				cs.model().partPool().at(i).getPq(std::any_cast<GetParam &>(data).part_pq.data() + i * 7);
			}

			cs.model().generalMotionPool().at(0).getMpq(std::any_cast<GetParam &>(data).end_pq.data());
			cs.model().generalMotionPool().at(0).getMpe(std::any_cast<GetParam &>(data).end_pe.data(), "321");

			for (aris::Size i = 0; i < cs.controller().motionPool().size(); i++)
			{
#ifdef WIN32
				std::any_cast<GetParam &>(data).motion_pos[i] = cs.model().motionPool()[i].mp();
				std::any_cast<GetParam &>(data).motion_vel[i] = cs.model().motionPool()[i].mv();
				std::any_cast<GetParam &>(data).motion_acc[i] = cs.model().motionPool()[i].ma();
				std::any_cast<GetParam &>(data).motion_toq[i] = cs.model().motionPool()[i].ma();
#endif // WIN32

#ifdef UNIX
				std::any_cast<GetParam &>(data).motion_pos[i] = cs.controller().motionPool()[i].actualPos();
				std::any_cast<GetParam &>(data).motion_vel[i] = cs.controller().motionPool()[i].actualVel();
				std::any_cast<GetParam &>(data).motion_acc[i] = cs.model().motionPool()[i].ma();
				std::any_cast<GetParam &>(data).motion_toq[i] = cs.controller().motionPool()[i].actualToq();
#endif // UNIX
			}
			for (aris::Size i = 0; i < 100; i++)
			{
				std::any_cast<GetParam &>(data).ai[i] = 1.0;
				std::any_cast<GetParam &>(data).di[i] = false;
			}
			std::any_cast<GetParam &>(data).state_code = 0;

			auto ec = dynamic_cast<aris::control::EthercatController*>(&cs.controller());
			ec->getLinkState(&std::any_cast<GetParam &>(data).mls, std::any_cast<GetParam &>(data).sls);

			//获取motion的使能状态，0表示去使能状态，1表示使能状态//
			for (aris::Size i = 0; i < 6; i++)
			{
				auto cm = dynamic_cast<aris::control::EthercatMotion*>(&cs.controller().motionPool()[i]);
				if ((cm->statusWord() & 0x6f) != 0x27)
				{
					std::any_cast<GetParam &>(data).motion_state[i] = 0;
				}
				else
				{
					std::any_cast<GetParam &>(data).motion_state[i] = 1;
				}
			}
			if (target == nullptr)
			{
				std::any_cast<GetParam &>(data).currentplan = "none";
			}
			else
			{
				std::any_cast<GetParam &>(data).currentplan = target->plan->command().name();
			}

		}, param);

		auto out_data = std::any_cast<GetParam &>(param);
		
		//舵机//
		out_data.motion_pos[7] = current_pos1.load();
		out_data.motion_pos[8] = current_pos2.load();
		out_data.motion_pos[9] = current_pos3.load();

		std::vector<int> slave_online(6, 0), slave_al_state(6, 0);
		for (aris::Size i = 0; i < 6; i++)
		{
			slave_online[i] = int(out_data.sls[i].online);
			slave_al_state[i] = int(out_data.sls[i].al_state);
		}

		std::vector<std::pair<std::string, std::any>> out_param;
		out_param.push_back(std::make_pair<std::string, std::any>("part_pq", out_data.part_pq));
		out_param.push_back(std::make_pair<std::string, std::any>("end_pq", out_data.end_pq));
		out_param.push_back(std::make_pair<std::string, std::any>("end_pe", out_data.end_pe));
		out_param.push_back(std::make_pair<std::string, std::any>("motion_pos", out_data.motion_pos));
		out_param.push_back(std::make_pair<std::string, std::any>("motion_vel", out_data.motion_vel));
		out_param.push_back(std::make_pair<std::string, std::any>("motion_acc", out_data.motion_acc));
		out_param.push_back(std::make_pair<std::string, std::any>("motion_toq", out_data.motion_toq));
		out_param.push_back(std::make_pair<std::string, std::any>("ai", out_data.ai));
		out_param.push_back(std::make_pair<std::string, std::any>("di", out_data.di));
		out_param.push_back(std::make_pair<std::string, std::any>("state_code", out_data.state_code));
		out_param.push_back(std::make_pair<std::string, std::any>("slave_link_num", std::int32_t(out_data.mls.slaves_responding)));
		out_param.push_back(std::make_pair<std::string, std::any>("slave_online_state", slave_online));
		out_param.push_back(std::make_pair<std::string, std::any>("slave_al_state", slave_al_state));
		out_param.push_back(std::make_pair<std::string, std::any>("motion_state", out_data.motion_state));
		out_param.push_back(std::make_pair<std::string, std::any>("current_plan", out_data.currentplan));
		out_param.push_back(std::make_pair<std::string, std::any>("current_plan_id", cmdparam.current_plan_id));

		std::array<double, 10> temp = { 0,0,0,0,0,0,0,0,0,0 };
		std::copy(out_data.motion_pos.begin(), out_data.motion_pos.end(), temp.begin());
		save_point.store(temp);

		target.ret = out_param;
		target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
	}
	auto Get::collectNrt(PlanTarget &target)->void {}
	Get::Get(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"get\">"
			"</Command>");
	}


	// 执行emily文件 //
	auto interpolate(double x1, double x2, double y1, double y2, double x)->double
	{
		double y;
		if (abs(x2 - x1) < 1e-6)
		{
			y = x2;
		}
		else
		{
			y = y1 + (x - x1)*(y2 - y1) / (x2 - x1);
		}
		return y;
	}
	struct MoveTParam
	{
		std::vector<std::vector<double>> pos;
		std::vector<double> begin_pos;
		std::int16_t col;
		double ratio;
	};
	auto MoveT::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		MoveTParam param;
		param.pos.clear();
		param.begin_pos.clear();
		std::ifstream infile;

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "col")
			{
				param.col = std::stoi(cmd_param.second);
				param.pos.resize(7);
				param.begin_pos.resize(param.col - 1);
			}
			else if (cmd_param.first == "path")
			{
				std::unique_lock<std::mutex> run_lock(dynamixel_mutex);
				dxl_pos.clear();
				dxl_pos.resize(3);
				std::vector<std::vector<double>> pos(param.col);
				auto path = cmd_param.second;
				infile.open(path);
				//检查读取文件是否成功//
				if (!infile)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " fail to open the file");

				std::string data;
				while(std::getline(infile, data))
				{
					if (data.find("[HEADER]") != std::string::npos)
					{
						continue;
					}
					if (data.find("GEAR_NOMINAL_VEL") != std::string::npos)
					{
						char *s_vel = (char *)data.c_str();
						const char *split = "=";
						// 以‘=’为分隔符拆分字符串
						char *sp_vel = strtok(s_vel, split);
						sp_vel = strtok(NULL, split);
						std::string vel = sp_vel;
						param.ratio = std::stod(vel) / 1.0;
						std::cout << "ratio:" << param.ratio << std::endl;
						continue;
					}
					if (data.find("[RECORDS]") != std::string::npos)
					{
						continue;
					}
					if (data.find("[END]") != std::string::npos)
					{
						break;
					}
						
					char *s_input = (char *)data.c_str();
					const char *split = "   ";
					// 以‘ ’为分隔符拆分字符串
					char *sp_input = strtok(s_input, split);
					double data_input;
					int i = 0;
					int j = 0;
					while (sp_input != NULL)
					{
						data_input = atof(sp_input);
						//前7轴的位置//
						if (i <= 6)
						{
							pos[i++].push_back(data_input);
							sp_input = strtok(NULL, split);
							continue;
						}
						if (i > 6)
						{
							dxl_pos[j++].push_back(data_input);
							i++;
							sp_input = strtok(NULL, split);
							continue;
						}
						if (i >= param.col)THROW_FILE_LINE("the format of emily file is not ok");
					}
				}
				for (int i = 0; i < pos.size(); i++)
				{
					for (int j = 0; j < pos[0].size() - 1; j++)
					{
						for (double count = pos[0][j]; count < pos[0][j + 1]; count = count + 0.001*param.ratio)
						{
							param.pos[i].push_back(interpolate(pos[0][j], pos[0][j + 1], pos[i][j], pos[i][j + 1], count)* PI/180.0);
						}
					}
				}
				infile.close();
			}
		}
		target.param = param;

		//std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::USE_TARGET_POS);
		for (int j = 0; j < param.pos.size(); j++)
		{
			for (int i = 0; i < 13; i++)
			{
				std::cout << param.pos[j][i] << "  ";
			}
			std::cout << std::endl;
		}

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		// 使能舵机emily功能 //
		enable_dynamixel_auto.store(true);
	}
	auto MoveT::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveTParam&>(target.param);
		auto controller = target.controller;

		for (int i = 0; i < controller->motionPool().size(); i++)
		{
			controller->motionPool()[i].setTargetPos(param.pos[i + 1][target.count]);
		}
		for (int i = 0; i < target.model->motionPool().size(); i++)
		{
			target.model->motionPool().at(i).setMp(param.pos[i + 1][target.count]);
		}
        if (target.model->solverPool().at(1).kinPos())return -1;
			   
		// 打印 //
		auto &cout = controller->mout();
        if (target.count % 100 == 0)
		{
			for (Size i = 0; i < controller->motionPool().size(); i++)
			{
				//cout << target.model->motionPool().at(i).mp() << "  ";
				cout << controller->motionPool()[i].actualPos() << "  ";
			}
			cout << std::endl;
		}
		// log //
		auto &lout = controller->lout();
		for (Size i = 0; i < controller->motionPool().size(); ++i)
		{
			for (Size i = 0; i < controller->motionPool().size(); i++)
			{
				cout << controller->motionPool()[i].actualPos() << "  ";
			}
			cout << std::endl;
		}    

		return param.pos[0].size() - target.count - 1;
	}
	auto MoveT::collectNrt(PlanTarget &target)->void {}
	MoveT::MoveT(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
            "<Command name=\"movet\">"
			"	<GroupParam>"
			"		<Param name=\"col\" default=\"7\"/>"
			"		<Param name=\"path\" default=\"C:\\Users\\kevin\\Desktop\\tuying\\example.emily\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 多关节插值梯形轨迹 //
	static std::atomic<std::array<double, 7> > axis_temp_pos;
	struct MoveAbJParam
	{
		std::vector<Size> total_count_vec;
		std::vector<double> p_now, v_now, a_now;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
	};
	auto MoveAbJ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		auto c = target.controller;
		MoveAbJParam param;
		param.total_count_vec.resize(c->motionPool().size(), 1);
		param.axis_pos_vec.resize(c->motionPool().size(), 0.0);
		param.p_now.resize(c->motionPool().size(), 0.0);
		param.v_now.resize(c->motionPool().size(), 0.0);
		param.a_now.resize(c->motionPool().size(), 0.0);

		//当前有指令在执行//
		std::shared_ptr<aris::plan::PlanTarget> planptr = cs.currentExecuteTarget();
		if (planptr && planptr->plan != this)
		{
			throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + "Other command is running");
		}
		else if (planptr && planptr->plan == this)
		{
			std::array<double, 7> temp = { 0,0,0,0,0,0,0 };
			for (auto &p : params)
			{
				if (p.first == "pos")
				{	
					auto pos = target.model->calculator().calculateExpression(p.second);
					if (pos.size() == c->motionPool().size())
					{			
						std::copy(pos.begin(), pos.end(), temp.begin());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					//超阈值保护//
					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (temp[i] > c->motionPool()[i].maxPos())
						{
							temp[i] = c->motionPool()[i].maxPos();
						}
						if (temp[i] < c->motionPool()[i].minPos())
						{
							temp[i] = c->motionPool()[i].minPos();
						}
					}
				}
			}
			
			axis_temp_pos.store(temp);
			target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
		}
		else
		{
			for (auto &p : params)
			{
				if (p.first == "pos")
				{
					auto pos = target.model->calculator().calculateExpression(p.second);
					if (pos.size() == 1)
					{
						param.axis_pos_vec.resize(c->motionPool().size(), pos.toDouble());
					}
					else if (pos.size() == c->motionPool().size())
					{
						param.axis_pos_vec.assign(pos.begin(), pos.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					//超阈值保护//
					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_pos_vec[i] > c->motionPool()[i].maxPos())
						{
							param.axis_pos_vec[i] = c->motionPool()[i].maxPos();
						}
						if (param.axis_pos_vec[i] < c->motionPool()[i].minPos())
						{
							param.axis_pos_vec[i] = c->motionPool()[i].minPos();
						}
					}
					std::array<double, 7> temp = { 0,0,0,0,0,0,0 };
					std::copy(param.axis_pos_vec.begin(), param.axis_pos_vec.end(), temp.begin());
					axis_temp_pos.store(temp);
				}
				else if (p.first == "vel")
				{
					auto v = target.model->calculator().calculateExpression(p.second);
					if (v.size() == 1)
					{
						param.axis_vel_vec.resize(c->motionPool().size(), v.toDouble());
					}
					else if (v.size() == c->motionPool().size())
					{
						param.axis_vel_vec.assign(v.begin(), v.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_vel_vec[i] > 1.0)
						{
							param.axis_vel_vec[i] = 1.0;
						}
						if (param.axis_vel_vec[i] < 0.0)
						{
							param.axis_vel_vec[i] = 0.0;
						}
						param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
					}
				}
				else if (p.first == "acc")
				{
					auto a = target.model->calculator().calculateExpression(p.second);
					if (a.size() == 1)
					{
						param.axis_acc_vec.resize(c->motionPool().size(), a.toDouble());
					}
					else if (a.size() == c->motionPool().size())
					{
						param.axis_acc_vec.assign(a.begin(), a.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_acc_vec[i] > 1.0)
						{
							param.axis_acc_vec[i] = 1.0;
						}
						if (param.axis_acc_vec[i] < 0.0)
						{
							param.axis_acc_vec[i] = 0.0;
						}
						param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
					}
				}
				else if (p.first == "dec")
				{
					auto d = target.model->calculator().calculateExpression(p.second);
					if (d.size() == 1)
					{
						param.axis_dec_vec.resize(c->motionPool().size(), d.toDouble());
					}
					else if (d.size() == c->motionPool().size())
					{
						param.axis_dec_vec.assign(d.begin(), d.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_dec_vec[i] > 1.0)
						{
							param.axis_dec_vec[i] = 1.0;
						}
						if (param.axis_dec_vec[i] < 0.0)
						{
							param.axis_dec_vec[i] = 0.0;
						}
						param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
					}
				}
			}
		}

		target.param = param;

		//std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::USE_TARGET_POS);

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

	}
	auto MoveAbJ::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<MoveAbJParam&>(target.param);

		// 取得起始位置 //
		if (target.count == 1)
		{
			for (Size i = 0; i < param.p_now.size(); ++i)
			{
				if (i < 6)
				{
					param.p_now[i] = target.model->motionPool().at(i).mp();
					param.v_now[i] = target.model->motionPool().at(i).mv();
					param.a_now[i] = target.model->motionPool().at(i).ma();
				}
				else
				{
					param.p_now[i] = controller->motionAtAbs(i).actualPos();
					param.v_now[i] = controller->motionAtAbs(i).actualVel();
					param.a_now[i] = 0.0;
				}
			}
		}
		
		auto temp_pos = axis_temp_pos.load();
		param.axis_pos_vec.assign(temp_pos.begin(), temp_pos.end());
		
		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;
		for (int i = 0; i < param.p_now.size(); i++)
		{
			aris::Size t;
			param.total_count_vec[i] = aris::plan::moveAbsolute2(param.p_now[i], param.v_now[i], param.a_now[i]
				, param.axis_pos_vec[i], 0.0, 0.0
				, param.axis_vel_vec[i], param.axis_acc_vec[i], param.axis_dec_vec[i]
				, 1e-3, 1e-4, p_next, v_next, a_next, t);
			controller->motionAtAbs(i).setTargetPos(p_next);
			if (i < 6)
			{
				target.model->motionPool().at(i).setMp(p_next);
			}
			param.p_now[i] = p_next;
			param.v_now[i] = v_next;
			param.a_now[i] = a_next;
		}
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			for (Size i = 0; i < param.axis_pos_vec.size(); i++)
			{
				cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
			}
			cout << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		for (Size i = 0; i < param.axis_pos_vec.size(); i++)
		{
			lout << controller->motionAtAbs(i).actualPos() << ",";
		}
		lout << std::endl;
		auto re = *std::max_element(param.total_count_vec.begin(), param.total_count_vec.end());
		return re;
	}
	auto MoveAbJ::collectNrt(PlanTarget &target)->void {}
	MoveAbJ::MoveAbJ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"mvabj\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"{0.2,0.2,0.2,0.2,0.2,0.2,0.2}\"/>"
			"		<Param name=\"vel\" default=\"{0.2,0.2,0.2,0.2,0.2,0.2,0.2}\" abbreviation=\"v\"/>"
			"		<Param name=\"acc\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"a\"/>"
			"		<Param name=\"dec\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"d\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 多关节混合插值梯形轨迹；速度前馈 //
	struct MoveJMParam
	{
		std::vector<Size> total_count_vec;
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
		bool ab;
	};
	auto MoveJM::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		MoveJMParam param;
		param.total_count_vec.resize(6, 1);
		param.axis_begin_pos_vec.resize(6, 0.0);

		//params.at("pos")
		for (auto &p : params)
		{
			if (p.first == "pos")
			{
				if (p.second == "current_pos")
				{
					target.option |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
				}
				else
				{
					auto pos = target.model->calculator().calculateExpression(p.second);
					if (pos.size() == 1)
					{
						param.axis_pos_vec.resize(param.axis_begin_pos_vec.size(), pos.toDouble());
					}
					else if (pos.size() == param.axis_begin_pos_vec.size())
					{
						param.axis_pos_vec.assign(pos.begin(), pos.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
					{
						//超阈值保护//
						if (param.axis_pos_vec[i] > 1.0)
						{
							param.axis_pos_vec[i] = 1.0;
						}
						if (param.axis_pos_vec[i] < -1.0)
						{
							param.axis_pos_vec[i] = -1.0;
						}
						if (param.axis_pos_vec[i] >= 0)
						{
							param.axis_pos_vec[i] = param.axis_pos_vec[i] * c->motionPool()[i].maxPos();
						}
						else
						{
							param.axis_pos_vec[i] = param.axis_pos_vec[i] * c->motionPool()[i].minPos();
						}					
					}
				}
			}
			else if (p.first == "vel")
			{
				auto v = target.model->calculator().calculateExpression(p.second);
				if (v.size() == 1)
				{
					param.axis_vel_vec.resize(param.axis_begin_pos_vec.size(), v.toDouble());
				}
				else if (v.size() == param.axis_begin_pos_vec.size())
				{
					param.axis_vel_vec.assign(v.begin(), v.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					//if (param.axis_vel_vec[i] > 1.0 || param.axis_vel_vec[i] < 0.01)
					//	throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					if (param.axis_vel_vec[i] > 1.0)
					{
						param.axis_vel_vec[i] = 1.0;
					}
					if (param.axis_vel_vec[i] < 0.0)
					{
						param.axis_vel_vec[i] = 0.0;
					}
					param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
				}
			}
			else if (p.first == "acc")
			{
				auto a = target.model->calculator().calculateExpression(p.second);
				if (a.size() == 1)
				{
					param.axis_acc_vec.resize(param.axis_begin_pos_vec.size(), a.toDouble());
				}
				else if (a.size() == param.axis_begin_pos_vec.size())
				{
					param.axis_acc_vec.assign(a.begin(), a.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					if (param.axis_acc_vec[i] > 1.0)
					{
						param.axis_acc_vec[i] = 1.0;
					}
					if (param.axis_acc_vec[i] < 0.0)
					{
						param.axis_acc_vec[i] = 0.0;
					}
					param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
				}
			}
			else if (p.first == "dec")
			{
				auto d = target.model->calculator().calculateExpression(p.second);
				if (d.size() == 1)
				{
					param.axis_dec_vec.resize(param.axis_begin_pos_vec.size(), d.toDouble());
				}
				else if (d.size() == param.axis_begin_pos_vec.size())
				{
					param.axis_dec_vec.assign(d.begin(), d.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					if (param.axis_dec_vec[i] > 1.0)
					{
						param.axis_dec_vec[i] = 1.0;
					}
					if (param.axis_dec_vec[i] < 0.0)
					{
						param.axis_dec_vec[i] = 0.0;
					}
					param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
				}
			}
			else if (p.first == "ab")
			{
				param.ab = std::stod(p.second);
			}
		}
		target.param = param;

		//std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::USE_VEL_OFFSET);

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

	}
	auto MoveJM::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<MoveJMParam&>(target.param);
		static double begin_pos[6];
		static double pos[6];
		// 取得起始位置 //
		if (target.count == 1)
		{
			for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
			{
				param.axis_begin_pos_vec[i] = controller->motionPool().at(i).targetPos();
			}
		}
		// 设置驱动器的位置 //
		if (param.ab)
		{
			for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
			{
				double p, v, a;
				aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
					, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
				controller->motionAtAbs(i).setTargetPos(p);
				//速度前馈//
				controller->motionAtAbs(i).setOffsetVel(v * 1000);
				target.model->motionPool().at(i).setMp(p);
			}
		}
		else
		{
			for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
			{
				double p, v, a;
				aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_begin_pos_vec[i] + param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
					, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
				controller->motionAtAbs(i).setTargetPos(p);
				//速度前馈//
				controller->motionAtAbs(i).setOffsetVel(v * 1000);
				target.model->motionPool().at(i).setMp(p);
			}
		}
				
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			for (Size i = 0; i < 6; i++)
			{
				cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
			}
			cout << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).actualPos() << ",";
		}
		lout << std::endl;

		return (static_cast<int>(*std::max_element(param.total_count_vec.begin(), param.total_count_vec.end())) > target.count) ? 1 : 0;
	}
	auto MoveJM::collectNrt(PlanTarget &target)->void {}
	MoveJM::MoveJM(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJM\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"current_pos\"/>"
			"		<Param name=\"vel\" default=\"{0.2,0.2,0.2,0.2,0.2,0.2}\" abbreviation=\"v\"/>"
			"		<Param name=\"acc\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"a\"/>"
			"		<Param name=\"dec\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"d\"/>"
			"		<Param name=\"ab\" default=\"1\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	auto check_eul_validity(const std::string &eul_type)->bool
	{
		if (eul_type.size() < 3)return false;

		for (int i = 0; i < 3; ++i)if (eul_type[i] > '3' || eul_type[i] < '1')return false;

		if (eul_type[0] == eul_type[1] || eul_type[1] == eul_type[2]) return false;

		return true;
	}
	auto find_mid_pq(const std::map<std::string, std::string> &params, PlanTarget &target, double *mid_pq_out)->bool
	{
		double pos_unit;
		auto pos_unit_found = params.find("pos_unit");
		if (pos_unit_found == params.end()) pos_unit = 1.0;
		else if (pos_unit_found->second == "m")pos_unit = 1.0;
		else if (pos_unit_found->second == "mm")pos_unit = 0.001;
		else if (pos_unit_found->second == "cm")pos_unit = 0.01;
		else THROW_FILE_LINE("");

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "mid_pq")
			{
				auto pq_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pq_mat.size() != 7)THROW_FILE_LINE("");
				aris::dynamic::s_vc(7, pq_mat.data(), mid_pq_out);
				aris::dynamic::s_nv(3, pos_unit, mid_pq_out);
				return true;
			}
			else if (cmd_param.first == "mid_pm")
			{
				auto pm_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pm_mat.size() != 16)THROW_FILE_LINE("");
				aris::dynamic::s_pm2pq(pm_mat.data(), mid_pq_out);
				aris::dynamic::s_nv(3, pos_unit, mid_pq_out);
				return true;
			}
			else if (cmd_param.first == "mid_pe")
			{
				double ori_unit;
				auto ori_unit_found = params.find("mid_ori_unit");
				if (ori_unit_found == params.end()) ori_unit = 1.0;
				else if (ori_unit_found->second == "rad")ori_unit = 1.0;
				else if (ori_unit_found->second == "degree")ori_unit = PI / 180.0;
				else THROW_FILE_LINE("");

				std::string eul_type;
				auto eul_type_found = params.find("mid_eul_type");
				if (eul_type_found == params.end()) eul_type = "321";
				else if (check_eul_validity(eul_type_found->second.data()))	eul_type = eul_type_found->second;
				else THROW_FILE_LINE("");

				auto pe_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pe_mat.size() != 6)THROW_FILE_LINE("");
				aris::dynamic::s_nv(3, ori_unit, pe_mat.data() + 3);
				aris::dynamic::s_pe2pq(pe_mat.data(), mid_pq_out, eul_type.data());
				aris::dynamic::s_nv(3, pos_unit, mid_pq_out);
				return true;
			}
		}

		THROW_FILE_LINE("No mid pose input");
	}
	auto find_end_pq(const std::map<std::string, std::string> &params, PlanTarget &target, double *end_pq_out)->bool
	{
		double pos_unit;
		auto pos_unit_found = params.find("pos_unit");
		if (pos_unit_found == params.end()) pos_unit = 1.0;
		else if (pos_unit_found->second == "m")pos_unit = 1.0;
		else if (pos_unit_found->second == "mm")pos_unit = 0.001;
		else if (pos_unit_found->second == "cm")pos_unit = 0.01;
		else THROW_FILE_LINE("");

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "end_pq")
			{
				auto pq_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pq_mat.size() != 7)THROW_FILE_LINE("");
				aris::dynamic::s_vc(7, pq_mat.data(), end_pq_out);
				aris::dynamic::s_nv(3, pos_unit, end_pq_out);
				return true;
			}
			else if (cmd_param.first == "end_pm")
			{
				auto pm_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pm_mat.size() != 16)THROW_FILE_LINE("");
				aris::dynamic::s_pm2pq(pm_mat.data(), end_pq_out);
				aris::dynamic::s_nv(3, pos_unit, end_pq_out);
				return true;
			}
			else if (cmd_param.first == "end_pe")
			{
				double ori_unit;
				auto ori_unit_found = params.find("mid_ori_unit");
				if (ori_unit_found == params.end()) ori_unit = 1.0;
				else if (ori_unit_found->second == "rad")ori_unit = 1.0;
				else if (ori_unit_found->second == "degree")ori_unit = PI / 180.0;
				else THROW_FILE_LINE("");

				std::string eul_type;
				auto eul_type_found = params.find("mid_eul_type");
				if (eul_type_found == params.end()) eul_type = "321";
				else if (check_eul_validity(eul_type_found->second.data()))	eul_type = eul_type_found->second;
				else THROW_FILE_LINE("");

				auto pe_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pe_mat.size() != 6)THROW_FILE_LINE("");
				aris::dynamic::s_nv(3, ori_unit, pe_mat.data() + 3);
				aris::dynamic::s_pe2pq(pe_mat.data(), end_pq_out, eul_type.data());
				aris::dynamic::s_nv(3, pos_unit, end_pq_out);
				return true;
			}
		}

		THROW_FILE_LINE("No end pose input");
	}
	void slerp(double starting[4], double ending[4], double result[4], double t)
	{
		double cosa = starting[0] * ending[0] + starting[1] * ending[1] + starting[2] * ending[2] + starting[3] * ending[3];

		// If the dot product is negative, the quaternions have opposite handed-ness and slerp won't take
		// the shorter path. Fix by reversing one quaternion.
		if (cosa < 0.0f)
		{
			ending[0] = -ending[0];
			ending[1] = -ending[1];
			ending[2] = -ending[2];
			ending[3] = -ending[3];
			cosa = -cosa;
		}

		double k0, k1;

		// If the inputs are too close for comfort, linearly interpolate
		if (cosa > 0.9995f)
		{
			k0 = 1.0f - t;
			k1 = t;
		}
		else
		{
			double sina = sqrt(1.0f - cosa * cosa);
			double a = atan2(sina, cosa);
			k0 = sin((1.0f - t)*a) / sina;
			k1 = sin(t*a) / sina;
		}
		result[0] = starting[0] * k0 + ending[0] * k1;
		result[1] = starting[1] * k0 + ending[1] * k1;
		result[2] = starting[2] * k0 + ending[2] * k1;
		result[3] = starting[3] * k0 + ending[3] * k1;
	}
	struct MoveCParam
	{
		std::vector<double> joint_vel, joint_acc, joint_dec, ee_begin_pq, ee_mid_pq, ee_end_pq, joint_pos_begin, joint_pos_end;
		Size total_count[6];
		double acc, vel, dec;
		double angular_acc, angular_vel, angular_dec;
	};
	struct MoveC::Imp {};
	auto MoveC::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		MoveCParam mvc_param;
		mvc_param.ee_begin_pq.resize(7);
		mvc_param.ee_mid_pq.resize(7);
		mvc_param.ee_end_pq.resize(7);
		if (!find_mid_pq(params, target, mvc_param.ee_mid_pq.data()))THROW_FILE_LINE("");
		if (!find_end_pq(params, target, mvc_param.ee_end_pq.data()))THROW_FILE_LINE("");

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "acc")
			{
				mvc_param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "vel")
			{
				mvc_param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				mvc_param.dec = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "angular_acc")
			{
				mvc_param.angular_acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "angular_vel")
			{
				mvc_param.angular_vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "angular_dec")
			{
				mvc_param.angular_dec = std::stod(cmd_param.second);
			}
		}

		for (auto &option : target.mot_options)	option |= Plan::USE_TARGET_POS;
		target.param = mvc_param;

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

	}
	auto MoveC::executeRT(PlanTarget &target)->int
	{
		auto &mvc_param = std::any_cast<MoveCParam &>(target.param);
		auto controller = target.controller;

		// 取得起始位置 //
		static double pos_ratio, ori_ratio;
		static double A[9], b[3], C[3], R, theta, ori_theta;
		double p, v, a;
		aris::Size pos_total_count, ori_total_count;

		if (target.count == 1)
		{
			target.model->generalMotionPool().at(0).getMpq(mvc_param.ee_begin_pq.data());

			//排除3点共线的情况//		
			std::vector<double> mul_cross(3);
			std::vector<double> p1(3);
			std::vector<double> p2(3);
			std::vector<double> p3(3);
			std::copy(mvc_param.ee_begin_pq.data(), mvc_param.ee_begin_pq.data() + 3, p1.data());
			std::copy(mvc_param.ee_mid_pq.data(), mvc_param.ee_mid_pq.data() + 3, p2.data());
			std::copy(mvc_param.ee_end_pq.data(), mvc_param.ee_end_pq.data() + 3, p3.data());
			s_vs(3, p2.data(), p3.data());
			s_vs(3, p1.data(), p2.data());
			s_c3(p2.data(), p3.data(), mul_cross.data());
			double normv = aris::dynamic::s_norm(3, mul_cross.data());
			if (normv <= 1e-10) return -1;
			
			//通过AC=b 解出圆心C//
			A[0] = 2 * (mvc_param.ee_begin_pq[0] - mvc_param.ee_mid_pq[0]);
			A[1] = 2 * (mvc_param.ee_begin_pq[1] - mvc_param.ee_mid_pq[1]);
			A[2] = 2 * (mvc_param.ee_begin_pq[2] - mvc_param.ee_mid_pq[2]);
			A[3] = 2 * (mvc_param.ee_mid_pq[0] - mvc_param.ee_end_pq[0]);
			A[4] = 2 * (mvc_param.ee_mid_pq[1] - mvc_param.ee_end_pq[1]);
			A[5] = 2 * (mvc_param.ee_mid_pq[2] - mvc_param.ee_end_pq[2]);
			A[6] = (A[1] * A[5] - A[4] * A[2]) / 4;
			A[7] = (A[0] * A[5] - A[3] * A[2]) / 4;
			A[8] = (A[0] * A[4] - A[3] * A[1]) / 4;
			b[0] = pow(mvc_param.ee_begin_pq[0], 2) + pow(mvc_param.ee_begin_pq[1], 2) + pow(mvc_param.ee_begin_pq[2], 2) - pow(mvc_param.ee_mid_pq[0], 2) - pow(mvc_param.ee_mid_pq[1], 2) - pow(mvc_param.ee_mid_pq[2], 2);
			b[1] = pow(mvc_param.ee_mid_pq[0], 2) + pow(mvc_param.ee_mid_pq[1], 2) + pow(mvc_param.ee_mid_pq[2], 2) - pow(mvc_param.ee_end_pq[0], 2) - pow(mvc_param.ee_end_pq[1], 2) - pow(mvc_param.ee_end_pq[2], 2);
			b[2] = A[6] * mvc_param.ee_begin_pq[0] + A[7] * mvc_param.ee_begin_pq[1] + A[8] * mvc_param.ee_begin_pq[2];
			
			//解线性方程组
			{
				double pinv[9];
				std::vector<double> U_vec(9);
				auto U = U_vec.data();
				double tau[3];
				aris::Size p[3];
				aris::Size rank;
				s_householder_utp(3, 3, A, U, tau, p, rank, 1e-10);
				double tau2[3];
				s_householder_utp2pinv(3, 3, rank, U, tau, p, pinv, tau2, 1e-10);
				//获取圆心
				s_mm(3, 1, 3, pinv, b, C);
				//获取半径
				R = sqrt(pow(mvc_param.ee_begin_pq[0] - C[0], 2) + pow(mvc_param.ee_begin_pq[1] - C[1], 2) + pow(mvc_param.ee_begin_pq[2] - C[2], 2));
			}
				
			//求旋转角度theta//	
			double u, v, w;
			double u1, v1, w1;
			u = A[6];
			v = A[7];
			w = A[8];
			u1 = (mvc_param.ee_begin_pq[1] - C[1])*(mvc_param.ee_end_pq[2] - mvc_param.ee_begin_pq[2]) - (mvc_param.ee_begin_pq[2] - C[2])*(mvc_param.ee_end_pq[1] - mvc_param.ee_begin_pq[1]);
			v1 = (mvc_param.ee_begin_pq[2] - C[2])*(mvc_param.ee_end_pq[0] - mvc_param.ee_begin_pq[0]) - (mvc_param.ee_begin_pq[0] - C[0])*(mvc_param.ee_end_pq[2] - mvc_param.ee_begin_pq[2]);
			w1 = (mvc_param.ee_begin_pq[0] - C[0])*(mvc_param.ee_end_pq[1] - mvc_param.ee_begin_pq[1]) - (mvc_param.ee_begin_pq[1] - C[1])*(mvc_param.ee_end_pq[0] - mvc_param.ee_begin_pq[0]);

			double H = u * u1 + v * v1 + w * w1;
			
			// 判断theta 与 pi 的关系
			if (H >= 0)
			{
				theta = 2 * asin(sqrt(pow((mvc_param.ee_end_pq[0] - mvc_param.ee_begin_pq[0]), 2) + pow((mvc_param.ee_end_pq[1] - mvc_param.ee_begin_pq[1]), 2) + pow((mvc_param.ee_end_pq[2] - mvc_param.ee_begin_pq[2]), 2)) / (2 * R));
			}
			else
			{
				theta = 2 * 3.1415 - 2 * asin(sqrt(pow((mvc_param.ee_end_pq[0] - mvc_param.ee_begin_pq[0]), 2) + pow((mvc_param.ee_end_pq[1] - mvc_param.ee_begin_pq[1]), 2) + pow((mvc_param.ee_end_pq[2] - mvc_param.ee_begin_pq[2]), 2)) / (2 * R));
			}
			
			double normv_begin_pq = aris::dynamic::s_norm(4, mvc_param.ee_begin_pq.data() + 3);
			double normv_end_pq = aris::dynamic::s_norm(4, mvc_param.ee_end_pq.data() + 3);

			ori_theta = std::abs((mvc_param.ee_begin_pq[3] * mvc_param.ee_end_pq[3] + mvc_param.ee_begin_pq[4] * mvc_param.ee_end_pq[4] + mvc_param.ee_begin_pq[5] * mvc_param.ee_end_pq[5] + mvc_param.ee_begin_pq[6] * mvc_param.ee_end_pq[6])/ (normv_begin_pq* normv_end_pq));

			aris::plan::moveAbsolute(target.count, 0.0, theta, mvc_param.vel / 1000 / R, mvc_param.acc / 1000 / 1000 / R, mvc_param.dec / 1000 / 1000 / R, p, v, a, pos_total_count);
			aris::plan::moveAbsolute(target.count, 0.0, 1.0, mvc_param.angular_vel / 1000 / ori_theta / 2.0, mvc_param.angular_acc / 1000 / 1000 / ori_theta / 2.0, mvc_param.angular_dec / 1000 / 1000 / ori_theta / 2.0, p, v, a, ori_total_count);

			pos_ratio = pos_total_count < ori_total_count ? double(pos_total_count) / ori_total_count : 1.0;
			ori_ratio = ori_total_count < pos_total_count ? double(ori_total_count) / pos_total_count : 1.0;
		}

		//位置规划//
		double w[3], pmr[16], pqt[7];
		aris::dynamic::s_vc(3, A + 6, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		aris::plan::moveAbsolute(target.count, 0.0, theta, mvc_param.vel / 1000 / R * pos_ratio, mvc_param.acc / 1000 / 1000 / R * pos_ratio * pos_ratio, mvc_param.dec / 1000 / 1000 / R * pos_ratio* pos_ratio, p, v, a, pos_total_count);
		
		double pqr[7]{ C[0], C[1], C[2], w[0] * sin(p / 2.0), w[1] * sin(p / 2.0), w[2] * sin(p / 2.0), cos(p / 2.0) };
		double pos[4]{ mvc_param.ee_begin_pq[0] - C[0], mvc_param.ee_begin_pq[1] - C[1], mvc_param.ee_begin_pq[2] - C[2], 1};
		aris::dynamic::s_pq2pm(pqr, pmr);
		s_mm(4, 1, 4, pmr, aris::dynamic::RowMajor{ 4 }, pos, 1, pqt, 1);

		//姿态规划//
		aris::plan::moveAbsolute(target.count, 0.0, 1.0, mvc_param.angular_vel / 1000 / ori_theta / 2.0 * ori_ratio, mvc_param.angular_acc / 1000 / 1000 / ori_theta / 2.0 * ori_ratio * ori_ratio, mvc_param.angular_dec / 1000 / 1000 / ori_theta / 2.0* ori_ratio * ori_ratio, p, v, a, ori_total_count);
		slerp(mvc_param.ee_begin_pq.data() + 3, mvc_param.ee_end_pq.data() + 3, pqt + 3, p);

		// set目标位置，并进行运动学反解 //
		target.model->generalMotionPool().at(0).setMpq(pqt);
		if (target.model->solverPool().at(0).kinPos())return -1;

		////////////////////////////////////// log ///////////////////////////////////////
		auto &lout = controller->lout();
		{
			lout << target.count << " " << pqt[0] << " " << pqt[1] << " " << pqt[2] << " " << pqt[3] << " " << pqt[4] << " " << pqt[5] << " " << pqt[6] << "  ";
			for (Size i = 0; i < 6; i++)
			{
				lout << controller->motionAtAbs(i).targetPos() << ",";
				lout << controller->motionAtAbs(i).actualPos() << ",";
				lout << controller->motionAtAbs(i).actualVel() << ",";
				lout << controller->motionAtAbs(i).actualCur() << ",";
			}
			lout << std::endl;
		}
		//////////////////////////////////////////////////////////////////////////////////

		return std::max(pos_total_count, ori_total_count) > target.count ? 1 : 0;
	}
	MoveC::~MoveC() = default;
	MoveC::MoveC(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"mvc\">"
			"	<GroupParam>"
			"		<Param name=\"pos_unit\" default=\"m\"/>"
			"		<UniqueParam default=\"mid_pq\">"
			"			<Param name=\"mid_pq\" default=\"{0,0,0,0,0,0,1}\"/>"
			"			<Param name=\"mid_pm\" default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
			"			<GroupParam>"
			"				<Param name=\"mid_pe\" default=\"{0,0,0,0,0,0}\"/>"
			"				<Param name=\"mid_ori_unit\" default=\"rad\"/>"
			"				<Param name=\"mid_eul_type\" default=\"321\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"		<UniqueParam default=\"end_pq\">"
			"			<Param name=\"end_pq\" default=\"{0,0,0,0,0,0,1}\"/>"
			"			<Param name=\"end_pm\" default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
			"			<GroupParam>"
			"				<Param name=\"end_pe\" default=\"{0,0,0,0,0,0}\"/>"
			"				<Param name=\"end_ori_unit\" default=\"rad\"/>"
			"				<Param name=\"end_eul_type\" default=\"321\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"		<Param name=\"acc\" default=\"0.1\"/>"
			"		<Param name=\"vel\" default=\"0.1\"/>"
			"		<Param name=\"dec\" default=\"0.1\"/>"
			"		<Param name=\"angular_acc\" default=\"0.1\"/>"
			"		<Param name=\"angular_vel\" default=\"0.1\"/>"
			"		<Param name=\"angular_dec\" default=\"0.1\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(MoveC);

	
	// 示教运动--输入末端大地坐标系的位姿pe，控制动作 //
	struct JogCParam {};
	struct JogCStruct
	{
		bool jogc_is_running = false;
		int cor_system;
		int vel_percent;
		std::array<int, 6> is_increase;
	};
	struct JogC::Imp
	{
		JogCStruct s1_rt, s2_nrt;
		std::vector<double> pm_target;
		double vel[6], acc[6], dec[6];
		int increase_count;
	};
	std::atomic_bool jogc_is_changing = false;
	auto JogC::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		JogCParam param;
		imp_->pm_target.resize(16, 0.0);
		
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		
		for (auto &p : params)
		{
			if (p.first == "start")
			{
				if (imp_->s1_rt.jogc_is_running)throw std::runtime_error("auto mode already started");

				imp_->s2_nrt.jogc_is_running = true;
				std::fill_n(imp_->s2_nrt.is_increase.data(), 6, 0);
				imp_->s2_nrt.cor_system= 0;
				imp_->s2_nrt.vel_percent = 10;

				imp_->s1_rt.jogc_is_running = true;
				std::fill_n(imp_->s1_rt.is_increase.data(), 6, 0);
				imp_->s1_rt.cor_system = 0;
				imp_->s1_rt.vel_percent = 10;

				imp_->increase_count = std::stoi(params.at("increase_count"));
				if (imp_->increase_count < 0 || imp_->increase_count>1e5)THROW_FILE_LINE("");

				auto mat = target.model->calculator().calculateExpression(params.at("vel"));
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->vel);

				mat = target.model->calculator().calculateExpression(params.at("acc"));
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->acc);

				mat = target.model->calculator().calculateExpression(params.at("dec"));
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->dec);

				std::fill(target.mot_options.begin(), target.mot_options.end(), USE_TARGET_POS);
                //target.option |= EXECUTE_WHEN_ALL_PLAN_COLLECTED | NOT_PRINT_EXECUTE_COUNT;
			}
			else if (p.first == "stop")
			{
				if (!imp_->s1_rt.jogc_is_running)throw std::runtime_error("manual mode not started, when stop");

				imp_->s2_nrt.jogc_is_running = false;
                std::fill_n(imp_->s2_nrt.is_increase.data(), 6, 0);

                target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
				jogc_is_changing = true;
				while (jogc_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			else if (p.first == "cor")
			{
				if (!imp_->s1_rt.jogc_is_running)throw std::runtime_error("manual mode not started, when pe");

				imp_->s2_nrt.cor_system = std::stoi(params.at("cor"));
				auto velocity = std::stoi(params.at("vel_percent"));
				velocity = std::max(std::min(100, velocity), -100);
				imp_->s2_nrt.vel_percent = velocity;
				imp_->s2_nrt.is_increase[0] = std::max(std::min(1, std::stoi(params.at("x"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[1] = std::max(std::min(1, std::stoi(params.at("y"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[2] = std::max(std::min(1, std::stoi(params.at("z"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[3] = std::max(std::min(1, std::stoi(params.at("a"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[4] = std::max(std::min(1, std::stoi(params.at("b"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[5] = std::max(std::min(1, std::stoi(params.at("c"))), -1) * imp_->increase_count;

				imp_->s2_nrt.jogc_is_running = true;

				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
				jogc_is_changing = true;
				while (jogc_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}
		
		target.param = param;
	}
	auto JogC::executeRT(PlanTarget &target)->int
	{	
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogCParam&>(target.param);
		char eu_type[4]{ '1', '2', '3', '\0' };
		
		// 前三维为xyz，后三维是w的积分，注意没有物理含义
		static double target_p[6];

		// get current pe //
		static double p_now[6], v_now[6], a_now[6];
		if (target.count == 1)
		{
			target.model->generalMotionPool()[0].getMpe(target_p);
			std::fill_n(target_p + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMpe(p_now, eu_type);
			std::fill_n(p_now + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMve(v_now, eu_type);
			target.model->generalMotionPool()[0].getMae(a_now, eu_type);
		}
		
		// init status //
		static int increase_status[6]{ 0,0,0,0,0,0 };
		double max_vel[6];
		if (jogc_is_changing)
		{
			imp_->s1_rt = imp_->s2_nrt;
			jogc_is_changing.store(false);
			for (int i = 0; i < 6; i++)
			{
				increase_status[i] = imp_->s1_rt.is_increase[i];
			}
			//target.model->generalMotionPool()[0].getMpe(imp_->pe_start, eu_type);
		}

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = imp_->vel[i]*1.0*imp_->s1_rt.vel_percent / 100.0;
			target_p[i] += aris::dynamic::s_sgn(increase_status[i])*max_vel[i] * 1e-3;
			increase_status[i] -= aris::dynamic::s_sgn(increase_status[i]);
		}
		//std::copy_n(target_pos, 6, imp_->pe_start);
		
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		for(int i=0; i<6; i++)
		{
			aris::Size t;
			aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], imp_->acc[i], imp_->dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0]*sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		target.model->generalMotionPool()[0].getMpm(pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);
		/*
		s_pe2pm(p_now, imp_->pm_now.data(), eu_type);
		for (int i = 0; i < 6; i++)
		{
			p_now[i] = p_next[i];
			v_now[i] = v_next[i];
			a_now[i] = a_next[i];
		}
		*/
		
		//绝对坐标系
		if (imp_->s1_rt.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, imp_->pm_target.data());
		}
		//工具坐标系
		else if (imp_->s1_rt.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, imp_->pm_target.data());
		}
		
		target.model->generalMotionPool().at(0).setMpm(imp_->pm_target.data());
		
		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 200 == 0)
		{
			cout << "pm_target:" << std::endl;
			for (Size i = 0; i < 16; i++)
			{
				cout << imp_->pm_target[i] << "  ";
			}
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << increase_status[i] << "  ";
			}
			cout << std::endl;
			cout << "p_next:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << p_next[i] << "  ";
			}
			cout << std::endl;
			cout << "v_next:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << v_next[i] << "  ";
			}
			cout << std::endl;
			cout << "p_now:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << p_now[i] << "  ";
			}
			cout << std::endl;
			cout << "v_now:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << v_now[i] << "  ";
			}
			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (int i = 0; i < 6; i++)
		{
			lout << target_p[i] << " ";
			lout << p_now[i] << " ";
			lout << v_now[i] << " ";
			lout << a_now[i] << " ";
			lout << controller->motionAtAbs(i).actualPos() << " ";
			lout << controller->motionAtAbs(i).actualVel() << " ";
		}
		lout << std::endl;

		return imp_->s1_rt.jogc_is_running ? 1 : 0;
	}
	auto JogC::collectNrt(PlanTarget &target)->void {}
	JogC::~JogC() = default;
	JogC::JogC(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jogC\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<GroupParam name=\"start_group\">"
			"				<Param name=\"start\"/>"
			"				<Param name=\"increase_count\" default=\"100\"/>"
            "				<Param name=\"vel\" default=\"{0.05,0.05,0.05,0.25,0.25,0.25}\"/>"
            "				<Param name=\"acc\" default=\"{0.2,0.2,0.2,1,1,1}\"/>"
            "				<Param name=\"dec\" default=\"{0.2,0.2,0.2,1,1,1}\"/>"
			"			</GroupParam>"
			"			<Param name=\"stop\"/>"
			"			<GroupParam>"
			"				<Param name=\"cor\" default=\"0\"/>"
			"				<Param name=\"vel_percent\" default=\"10\"/>"
			"				<Param name=\"x\" default=\"0\"/>"
			"				<Param name=\"y\" default=\"0\"/>"
			"				<Param name=\"z\" default=\"0\"/>"
			"				<Param name=\"a\" default=\"0\"/>"
			"				<Param name=\"b\" default=\"0\"/>"
			"				<Param name=\"c\" default=\"0\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}
	JogC::JogC(const JogC &other) = default;
	JogC::JogC(JogC &other) = default;
	JogC& JogC::operator=(const JogC &other) = default;
	JogC& JogC::operator=(JogC &&other) = default;


	// 示教运动--关节空间点动 //
	struct JogJStruct
	{
		bool jogj_is_running = false;
		int vel_percent;
		std::vector<int> is_increase;
	};
	struct JogJ::Imp
	{
		JogJStruct s1_rt, s2_nrt;
		double vel, acc, dec;
		std::vector<double> p_now, v_now, a_now, target_pos, max_vel;
		std::vector<int> increase_status;
		int increase_count;
	};
	std::atomic_bool jogj_is_changing = false;
	auto JogJ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		imp_->p_now.resize(c->motionPool().size(), 0.0);
		imp_->v_now.resize(c->motionPool().size(), 0.0);
		imp_->a_now.resize(c->motionPool().size(), 0.0);
		imp_->target_pos.resize(c->motionPool().size(), 0.0);
		imp_->max_vel.resize(c->motionPool().size(), 0.0);
		imp_->increase_status.resize(c->motionPool().size(), 0);

		for (auto &p : params)
		{
			if (p.first == "start")
			{
				if (imp_->s1_rt.jogj_is_running)throw std::runtime_error("auto mode already started");

				imp_->s2_nrt.jogj_is_running = true;
				imp_->s2_nrt.is_increase.clear();
				imp_->s2_nrt.is_increase.resize(c->motionPool().size(), 0);
				imp_->s2_nrt.vel_percent = 10;

				imp_->s1_rt.jogj_is_running = true;
				imp_->s1_rt.is_increase.clear();
				imp_->s1_rt.is_increase.resize(c->motionPool().size(), 0);
				imp_->s1_rt.vel_percent = 10;

				imp_->increase_count = std::stoi(params.at("increase_count"));
				if (imp_->increase_count < 0 || imp_->increase_count>1e5)THROW_FILE_LINE("");
				imp_->vel = std::stod(params.at("vel"));
				imp_->acc = std::stod(params.at("acc"));
				imp_->dec = std::stod(params.at("dec"));

				std::fill(target.mot_options.begin(), target.mot_options.end(), NOT_CHECK_POS_FOLLOWING_ERROR | USE_TARGET_POS);
				//target.option |= EXECUTE_WHEN_ALL_PLAN_COLLECTED | NOT_PRINT_EXECUTE_COUNT;
			}
			else if (p.first == "stop")
			{
				if (!imp_->s1_rt.jogj_is_running)throw std::runtime_error("manual mode not started, when stop");

				imp_->s2_nrt.jogj_is_running = false;
				imp_->s2_nrt.is_increase.assign(imp_->s2_nrt.is_increase.size(), 0);
				jogj_is_changing = true;
				while (jogj_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
			}
			else if (p.first == "vel_percent")
			{
				if (!imp_->s1_rt.jogj_is_running)throw std::runtime_error("manual mode not started, when pe");

				auto velocity = std::stoi(params.at("vel_percent"));
				velocity = std::max(std::min(100, velocity), -100);
				imp_->s2_nrt.vel_percent = velocity;

				imp_->s2_nrt.is_increase.assign(imp_->s2_nrt.is_increase.size(), 0);
				imp_->s2_nrt.is_increase[std::stoi(params.at("motion_id"))] = std::max(std::min(1, std::stoi(params.at("direction"))), -1) * imp_->increase_count;

				imp_->s2_nrt.jogj_is_running = true;

				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
				jogj_is_changing = true;
				while (jogj_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}

	}
	auto JogJ::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;

		// get current pe //
		if (target.count == 1)
		{
			for (Size i = 0; i < imp_->p_now.size(); ++i)
			{
				/*
				imp_->p_start[i] = target.model->motionPool().at(i).mp();
				imp_->p_now[i] = target.model->motionPool().at(i).mp();
				imp_->v_now[i] = target.model->motionPool().at(i).mv();
				imp_->a_now[i] = target.model->motionPool().at(i).ma();
				*/
				imp_->target_pos[i] = controller->motionAtAbs(i).actualPos();
				imp_->p_now[i] = controller->motionAtAbs(i).actualPos();
				imp_->v_now[i] = controller->motionAtAbs(i).actualVel();
				imp_->a_now[i] = 0.0;
			}
		}
		// init status and calculate target pos and max vel //

		if (jogj_is_changing)
		{
			jogj_is_changing.store(false);
			imp_->s1_rt = imp_->s2_nrt;
			for (int i = 0; i < imp_->s1_rt.is_increase.size(); i++)
			{
				imp_->increase_status[i] = imp_->s1_rt.is_increase[i];
			}
		}
		for (int i = 0; i < imp_->s1_rt.is_increase.size(); i++)
		{
			imp_->max_vel[i] = imp_->vel*1.0*imp_->s1_rt.vel_percent / 100.0;
			imp_->target_pos[i] += aris::dynamic::s_sgn(imp_->increase_status[i])*imp_->max_vel[i] * 1e-3;
			imp_->increase_status[i] -= aris::dynamic::s_sgn(imp_->increase_status[i]);
		}
		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;
		for (int i = 0; i < imp_->p_now.size(); i++)
		{
			aris::Size t;
			aris::plan::moveAbsolute2(imp_->p_now[i], imp_->v_now[i], imp_->a_now[i]
				, imp_->target_pos[i], 0.0, 0.0
				, imp_->max_vel[i], imp_->acc, imp_->dec
				, 1e-3, 1e-10, p_next, v_next, a_next, t);

			target.model->motionPool().at(i).setMp(p_next);
			controller->motionAtAbs(i).setTargetPos(p_next);
			imp_->p_now[i] = p_next;
			imp_->v_now[i] = v_next;
			imp_->a_now[i] = a_next;
		}

		// 运动学反解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 200 == 0)
		{
			for (int i = 0; i < imp_->p_now.size(); i++)
			{
				cout << imp_->increase_status[i] << "  ";
			}
			cout << std::endl;
			for (int i = 0; i < imp_->p_now.size(); i++)
			{
				cout << imp_->target_pos[i] << "  ";
			}
			cout << std::endl;
			for (int i = 0; i < imp_->p_now.size(); i++)
			{
				cout << imp_->p_now[i] << "  ";
			}
			cout << std::endl;
			for (int i = 0; i < imp_->p_now.size(); i++)
			{
				cout << imp_->v_now[i] << "  ";
			}
			cout << std::endl;
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (int i = 0; i < imp_->p_now.size(); i++)
		{
			lout << imp_->target_pos[i] << " ";
			lout << imp_->v_now[i] << " ";
			lout << imp_->a_now[i] << " ";
			lout << controller->motionAtAbs(i).actualPos() << " ";
			lout << controller->motionAtAbs(i).actualVel() << " ";
			//lout << controller->motionAtAbs(i).actualCur() << " ";
		}
		lout << std::endl;

		return imp_->s1_rt.jogj_is_running ? 1 : 0;
	}
	auto JogJ::collectNrt(PlanTarget &target)->void {}
	JogJ::~JogJ() = default;
	JogJ::JogJ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jogJ\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<GroupParam name=\"start_group\">"
			"				<Param name=\"start\"/>"
			"				<Param name=\"increase_count\" default=\"100\"/>"
			"				<Param name=\"vel\" default=\"1\" abbreviation=\"v\"/>"
			"				<Param name=\"acc\" default=\"5\" abbreviation=\"a\"/>"
			"				<Param name=\"dec\" default=\"5\" abbreviation=\"d\"/>"
			"			</GroupParam>"
			"			<Param name=\"stop\"/>"
			"			<GroupParam>"
			"				<Param name=\"vel_percent\" default=\"10\"/>"
			"				<Param name=\"motion_id\" default=\"0\" abbreviation=\"m\"/>"
			"				<Param name=\"direction\" default=\"1\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}
	JogJ::JogJ(const JogJ &other) = default;
	JogJ::JogJ(JogJ &other) = default;
	JogJ& JogJ::operator=(const JogJ &other) = default;
	JogJ& JogJ::operator=(JogJ &&other) = default;


#define JOGJ_PARAM_STRING \
		"	<UniqueParam>"\
		"		<GroupParam>"\
		"			<Param name=\"increase_count\" default=\"500\"/>"\
		"			<Param name=\"vel\" default=\"0.5\" abbreviation=\"v\"/>"\
		"			<Param name=\"acc\" default=\"0.5\" abbreviation=\"a\"/>"\
		"			<Param name=\"dec\" default=\"0.5\" abbreviation=\"d\"/>"\
		"			<Param name=\"vel_percent\" default=\"10\"/>"\
		"			<Param name=\"direction\" default=\"1\"/>"\
		"		</GroupParam>"\
		"		<Param name=\"stop\"/>"\
		"	</UniqueParam>"
	// 示教运动--关节1点动 //
	struct JogJParam
	{
		int motion_id;
		double vel, acc, dec;
		double p_now, v_now, a_now, target_pos, max_vel;
		int increase_status;
		int increase_count;
		int vel_percent;
		static std::atomic_int32_t j1_count, j2_count, j3_count, j4_count, j5_count, j6_count, j7_count;
	};
	template<typename JogType>
	auto set_jogj_input_param(JogType* this_p, const std::map<std::string, std::string> &cmd_params, PlanTarget &target, JogJParam &param, std::atomic_int32_t& j_count)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		auto c = target.controller;
		param.p_now = 0.0;
		param.v_now = 0.0;
		param.a_now = 0.0;
		param.target_pos = 0.0;
		param.max_vel = 0.0;
		param.increase_status = 0;
		param.vel_percent = 0;

		for (auto &p : cmd_params)
		{
			if (p.first == "increase_count")
			{
				param.increase_count = std::stoi(cmd_params.at("increase_count"));
				if (param.increase_count < 0 || param.increase_count>1e5)THROW_FILE_LINE("");

				param.vel = std::min(std::max(std::stod(cmd_params.at("vel")), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxVel();
				param.acc = std::min(std::max(std::stod(cmd_params.at("acc")), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxAcc();
				param.dec = std::min(std::max(std::stod(cmd_params.at("dec")), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxAcc();

				auto velocity = std::stoi(cmd_params.at("vel_percent"));
				velocity = std::max(std::min(100, velocity), 0);
				param.vel_percent = velocity;
				param.increase_status = std::max(std::min(1, std::stoi(cmd_params.at("direction"))), -1);

				std::shared_ptr<aris::plan::PlanTarget> planptr = cs.currentExecuteTarget();
				//当前有指令在执行//
				if (planptr && planptr->plan != this_p)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + "Other command is running");
				if (j_count.exchange(param.increase_count))
				{
					target.option |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
				}
				else
				{
					std::fill(target.mot_options.begin(), target.mot_options.end(), aris::plan::Plan::MotionOption::NOT_CHECK_POS_FOLLOWING_ERROR | aris::plan::Plan::MotionOption::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | aris::plan::Plan::MotionOption::USE_TARGET_POS | aris::plan::Plan::MotionOption::NOT_CHECK_ENABLE);
					target.mot_options[param.motion_id] = aris::plan::Plan::MotionOption::NOT_CHECK_POS_FOLLOWING_ERROR | aris::plan::Plan::MotionOption::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | aris::plan::Plan::MotionOption::USE_TARGET_POS;
				}
			}
			else if (p.first == "stop")
			{
				j_count.exchange(0);
				target.option |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
			}
		}
	}
	std::atomic_int32_t JogJParam::j1_count = 0;
	auto JogJ1::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JogJParam param;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		param.motion_id = 0;

		set_jogj_input_param(this, params, target, param, param.j1_count);

		target.param = param;
	}
	auto JogJ1::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			/*
			imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
			imp_->a_now = 0.0;
			*/
			param.target_pos = target.model->motionPool().at(param.motion_id).mp();
			param.p_now = target.model->motionPool()[param.motion_id].mp();
			param.v_now = target.model->motionPool()[param.motion_id].mv();
			param.a_now = 0.0;

		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*g_vel_percent.load()/100.0;

		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		if (param.j1_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j1_count;
		}

		target.model->motionPool().at(param.motion_id).setMp(p_next);
		//controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 运动学正解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j1_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ1::collectNrt(PlanTarget &target)->void
	{
		JogJParam::j1_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j1_count.store(0);
		}
	}
	JogJ1::~JogJ1() = default;
	JogJ1::JogJ1(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j1\">"
			JOGJ_PARAM_STRING
			"</Command>");
	}


	// 示教运动--关节2点动 //
	std::atomic_int32_t JogJParam::j2_count = 0;
	auto JogJ2::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JogJParam param;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		param.motion_id = 1;

		set_jogj_input_param(this, params, target, param, param.j2_count);

		target.param = param;
	}
	auto JogJ2::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			/*
			imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
			imp_->a_now = 0.0;
			*/
			param.target_pos = target.model->motionPool().at(param.motion_id).mp();
			param.p_now = target.model->motionPool()[param.motion_id].mp();
			param.v_now = target.model->motionPool()[param.motion_id].mv();
			param.a_now = 0.0;

		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*g_vel_percent.load()/100.0;

		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		if (param.j2_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j2_count;
		}

		target.model->motionPool().at(param.motion_id).setMp(p_next);
		//controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 运动学正解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j2_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ2::collectNrt(PlanTarget &target)->void
	{
		JogJParam::j2_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j2_count.store(0);
		}
	}
	JogJ2::~JogJ2() = default;
	JogJ2::JogJ2(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j2\">"
			JOGJ_PARAM_STRING
			"</Command>");
	}


	// 示教运动--关节3点动 //
	std::atomic_int32_t JogJParam::j3_count = 0;
	auto JogJ3::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JogJParam param;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		param.motion_id = 2;
		set_jogj_input_param(this, params, target, param, param.j3_count);

		target.param = param;
	}
	auto JogJ3::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			/*
			imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
			imp_->a_now = 0.0;
			*/
			param.target_pos = target.model->motionPool().at(param.motion_id).mp();
			param.p_now = target.model->motionPool()[param.motion_id].mp();
			param.v_now = target.model->motionPool()[param.motion_id].mv();
			param.a_now = 0.0;

		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*g_vel_percent.load()/100.0;

		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		if (param.j3_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j3_count;
		}

		target.model->motionPool().at(param.motion_id).setMp(p_next);
		//controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 运动学正解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j3_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ3::collectNrt(PlanTarget &target)->void
	{
		JogJParam::j3_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j3_count.store(0);
		}
	}
	JogJ3::~JogJ3() = default;
	JogJ3::JogJ3(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j3\">"
			JOGJ_PARAM_STRING
			"</Command>");
	}


	// 示教运动--关节4点动 //
	std::atomic_int32_t JogJParam::j4_count = 0;
	auto JogJ4::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JogJParam param;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		param.motion_id = 3;
		set_jogj_input_param(this, params, target, param, param.j4_count);

		target.param = param;
	}
	auto JogJ4::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			/*
			imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
			imp_->a_now = 0.0;
			*/
			param.target_pos = target.model->motionPool().at(param.motion_id).mp();
			param.p_now = target.model->motionPool()[param.motion_id].mp();
			param.v_now = target.model->motionPool()[param.motion_id].mv();
			param.a_now = 0.0;

		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*g_vel_percent.load()/100.0;

		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		if (param.j4_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j4_count;
		}

		target.model->motionPool().at(param.motion_id).setMp(p_next);
		//controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 运动学正解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j4_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ4::collectNrt(PlanTarget &target)->void
	{
		JogJParam::j4_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j4_count.store(0);
		}
	}
	JogJ4::~JogJ4() = default;
	JogJ4::JogJ4(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j4\">"
			JOGJ_PARAM_STRING
			"</Command>");
	}


	// 示教运动--关节5点动 //
	std::atomic_int32_t JogJParam::j5_count = 0;
	auto JogJ5::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JogJParam param;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		param.motion_id = 4;
		set_jogj_input_param(this, params, target, param, param.j5_count);

		target.param = param;
	}
	auto JogJ5::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			/*
			imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
			imp_->a_now = 0.0;
			*/
			param.target_pos = target.model->motionPool().at(param.motion_id).mp();
			param.p_now = target.model->motionPool()[param.motion_id].mp();
			param.v_now = target.model->motionPool()[param.motion_id].mv();
			param.a_now = 0.0;

		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*g_vel_percent.load()/100.0;

		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		if (param.j5_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j5_count;
		}

		target.model->motionPool().at(param.motion_id).setMp(p_next);
		//controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 运动学正解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j5_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ5::collectNrt(PlanTarget &target)->void
	{
		JogJParam::j5_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j5_count.store(0);
		}
	}
	JogJ5::~JogJ5() = default;
	JogJ5::JogJ5(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j5\">"
			JOGJ_PARAM_STRING
			"</Command>");
	}


	// 示教运动--关节6点动 //
	std::atomic_int32_t JogJParam::j6_count = 0;
	auto JogJ6::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JogJParam param;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		param.motion_id = 5;
		set_jogj_input_param(this, params, target, param, param.j6_count);

		target.param = param;
	}
	auto JogJ6::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			/*
			imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
			imp_->a_now = 0.0;
			*/
			param.target_pos = target.model->motionPool().at(param.motion_id).mp();
			param.p_now = target.model->motionPool()[param.motion_id].mp();
			param.v_now = target.model->motionPool()[param.motion_id].mv();
			param.a_now = 0.0;

		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*g_vel_percent.load()/100.0;
		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		//当计数器j6_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.j6_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j6_count;
		}

		target.model->motionPool().at(param.motion_id).setMp(p_next);
		//controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 运动学正解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j6_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ6::collectNrt(PlanTarget &target)->void
	{
		JogJParam::j6_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j6_count.store(0);
		}
	}
	JogJ6::~JogJ6() = default;
	JogJ6::JogJ6(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j6\">"
			JOGJ_PARAM_STRING
			"</Command>");
	}


	// 示教运动--外部轴点动 //
	std::atomic_int32_t JogJParam::j7_count = 0;
	auto JogJ7::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		auto c = target.controller;
		JogJParam param;

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		param.motion_id = 6;
		param.p_now = 0.0;
		param.v_now = 0.0;
		param.a_now = 0.0;
		param.target_pos = 0.0;
		param.max_vel = 0.0;
		param.increase_status = 0;
		param.vel_percent = 0;

		for (auto &p : params)
		{
			if (p.first == "increase_count")
			{
				param.increase_count = std::stoi(params.at("increase_count"));
				if (param.increase_count < 0 || param.increase_count>1e5)THROW_FILE_LINE("");

				param.vel = std::min(std::max(std::stod(params.at("vel")), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxVel();
				param.acc = std::min(std::max(std::stod(params.at("acc")), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxAcc();
				param.dec = std::min(std::max(std::stod(params.at("dec")), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxAcc();

				auto velocity = std::stoi(params.at("vel_percent"));
				velocity = std::max(std::min(100, velocity), 0);
				param.vel_percent = velocity;
				param.increase_status = std::max(std::min(1, std::stoi(params.at("direction"))), -1);

				std::shared_ptr<aris::plan::PlanTarget> planptr = cs.currentExecuteTarget();
				//当前有指令在执行//
				if (planptr && planptr->plan != this)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + "Other command is running");

				if (param.j7_count.exchange(param.increase_count))
				{
					target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
				}
				else
				{
					std::fill(target.mot_options.begin(), target.mot_options.end(), NOT_CHECK_POS_FOLLOWING_ERROR | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | NOT_CHECK_ENABLE);
					target.mot_options[param.motion_id] = NOT_CHECK_POS_FOLLOWING_ERROR | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
				}
			}
			else if (p.first == "stop")
			{
				param.j7_count.exchange(0);
				target.option |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
			}
		}

		target.param = param;
	}
	auto JogJ7::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			param.target_pos = controller->motionAtAbs(param.motion_id).actualPos();
			param.p_now = controller->motionAtAbs(param.motion_id).actualPos();
			param.v_now = controller->motionAtAbs(param.motion_id).actualVel();
			param.a_now = 0.0;
		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*g_vel_percent.load()/100.0;
		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		//当计数器j7_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.j7_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j7_count;
		}

		controller->motionAtAbs(param.motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j7_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ7::collectNrt(PlanTarget &target)->void
	{
		JogJParam::j7_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j7_count.store(0);
		}
	}
	JogJ7::~JogJ7() = default;
	JogJ7::JogJ7(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j7\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<GroupParam>"
			"				<Param name=\"increase_count\" default=\"500\"/>"
			"				<Param name=\"vel\" default=\"1\" abbreviation=\"v\"/>"
			"				<Param name=\"acc\" default=\"5\" abbreviation=\"a\"/>"
			"				<Param name=\"dec\" default=\"5\" abbreviation=\"d\"/>"
			"				<Param name=\"vel_percent\" default=\"10\"/>"
			"				<Param name=\"direction\" default=\"1\"/>"
			"			</GroupParam>"
			"			<Param name=\"stop\"/>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}


	// 切换舵机模式 //
	auto DMode::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		for (auto &p : params)
		{
			if (p.first == "mode")
			{
				mode_dynamixel.store(std::stoi(p.second));
			}		
		}

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
	}
	auto DMode::collectNrt(PlanTarget &target)->void {}
	DMode::DMode(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"dmode\">"
			"	<GroupParam>"
			"		<GroupParam>"
			"			<Param name=\"mode\" default=\"0\"/>"
			"		</GroupParam>"
			"	</GroupParam>"
			"</Command>");
	}


	// 使能舵机 //
	auto DEnable::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		is_enabled.store(1);
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
	}
	auto DEnable::collectNrt(PlanTarget &target)->void {}
	DEnable::DEnable(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"denable\">"
			"</Command>");
	}


	// 去使能舵机 //
	auto DDisable::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		is_enabled.store(0);
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
	}
	auto DDisable::collectNrt(PlanTarget &target)->void {}
	DDisable::DDisable(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"ddisable\">"
			"</Command>");
	}

	// 1号舵机点动 //
	auto DJ1::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		int step = 0;
		for (auto &p : params)
		{
			if (p.first == "step")
			{
				step = 11.378*std::stoi(p.second);
			}	
		}
		target_pos1 += step;

		std::cout << "target_pos1:" << target_pos1 << std::endl;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		enable_dynamixel_manual.store(1);
	}
	auto DJ1::collectNrt(PlanTarget &target)->void{}
	DJ1::DJ1(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"dj1\">"
			"	<GroupParam>"
			"		<Param name=\"step\" default=\"2\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 2号舵机点动 //
	auto DJ2::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		int step = 0;
		for (auto &p : params)
		{
			if (p.first == "step")
			{
				step = 11.378*std::stoi(p.second);
			}
		}
		target_pos2 += step;

		std::cout << "target_pos2:" << target_pos2 << std::endl;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		enable_dynamixel_manual.store(1);
	}
	auto DJ2::collectNrt(PlanTarget &target)->void {}
	DJ2::DJ2(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"dj2\">"
			"	<GroupParam>"
			"		<Param name=\"step\" default=\"2\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 3号舵机点动 //
	auto DJ3::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		int step = 0;
		for (auto &p : params)
		{
			if (p.first == "step")
			{
				step = 11.378*std::stoi(p.second);
			}
		}
		target_pos3 += step;

		std::cout << "target_pos3:" << target_pos3 << std::endl;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		enable_dynamixel_manual.store(1);
	}
	auto DJ3::collectNrt(PlanTarget &target)->void {}
	DJ3::DJ3(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"dj3\">"
			"	<GroupParam>"
			"		<Param name=\"step\" default=\"2\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 舵机moveabsj //
	struct DMoveAbsJParam
	{
		std::vector<double> joint_pos_vec;
		std::vector<bool> joint_active_vec;
		const int d_num = 3;
	};
	auto DMoveAbsJ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		DMoveAbsJParam param;
		int step = 0;
		for (auto cmd_param : params)
		{
			if (cmd_param.first == "all")
			{
				param.joint_active_vec.resize(param.d_num, true);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.joint_active_vec.resize(param.d_num, false);
				param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
			}
			else if (cmd_param.first == "pos")
			{
				aris::core::Matrix mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (mat.size() == 1)param.joint_pos_vec.resize(param.d_num, mat.toDouble());
				else
				{
					param.joint_pos_vec.resize(mat.size());
					std::copy(mat.begin(), mat.end(), param.joint_pos_vec.begin());
				}
			}
		}

		target_pos1.store(int(11.378*param.joint_pos_vec[0]));
		target_pos2.store(int(11.378*param.joint_pos_vec[1]));
		target_pos3.store(int(11.378*param.joint_pos_vec[2]));
		std::cout << "target_pos1:" << target_pos1 << std::endl;
		std::cout << "target_pos2:" << target_pos2 << std::endl;
		std::cout << "target_pos3:" << target_pos3 << std::endl;

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		enable_dynamixel_manual.store(1);
	}
	auto DMoveAbsJ::collectNrt(PlanTarget &target)->void {}
	DMoveAbsJ::DMoveAbsJ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"dmvaj\">"
			"	<GroupParam>"
			"		<UniqueParam default=\"all\">"
			"			<Param name=\"all\" abbreviation=\"a\"/>"
			"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"		</UniqueParam>"
			"		<Param name=\"pos\" default=\"0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


#define JOGC_PARAM_STRING \
		"	<UniqueParam>"\
		"		<GroupParam>"\
		"			<Param name=\"increase_count\" default=\"500\"/>"\
		"			<Param name=\"vel\" default=\"{0.2,0.2,0.2,0.25,0.25,0.25}\"/>"\
		"			<Param name=\"acc\" default=\"{1,1,1,1,1,1}\"/>"\
		"			<Param name=\"dec\" default=\"{1,1,1,1,1,1}\"/>"\
		"			<Param name=\"cor\" default=\"0\"/>"\
		"			<Param name=\"tool\" default=\"tool0\"/>"\
		"			<Param name=\"wobj\" default=\"wobj0\"/>"\
		"			<Param name=\"vel_percent\" default=\"20\"/>"\
		"			<Param name=\"direction\" default=\"1\"/>"\
		"		</GroupParam>"\
		"		<Param name=\"stop\"/>"\
		"	</UniqueParam>"
	// 示教运动--jogx //
	struct JCParam
	{
		std::vector<double> pm_target;
		double vel[6], acc[6], dec[6];
		int increase_count;
		int cor_system;
		int vel_percent;
		int moving_type;
		int increase_status[6]{ 0,0,0,0,0,0 };
		static std::atomic_int32_t jx_count, jy_count, jz_count, jrx_count, jry_count, jrz_count;
		aris::dynamic::Marker *tool, *wobj;
	};
	template<typename JogType>
	auto set_jogc_input_param(JogType* this_p, const std::map<std::string, std::string> &cmd_params, PlanTarget &target, JCParam &param, std::atomic_int32_t& j_count)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		auto c = target.controller;

		for (auto &p : cmd_params)
		{
			if (p.first == "increase_count")
			{
				auto tool_is_string = cmd_params.at("tool");
				auto wobj_is_string = cmd_params.at("wobj");

				param.tool = &*target.model->generalMotionPool()[0].makI().fatherPart().markerPool().findByName(cmd_params.at("tool"));
				param.wobj = &*target.model->generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(cmd_params.at("wobj"));

				param.increase_count = std::stoi(cmd_params.at("increase_count"));
				if (param.increase_count < 0 || param.increase_count>1e5)THROW_FILE_LINE("");

				param.cor_system = std::stoi(cmd_params.at("cor"));
				auto velocity = std::stoi(cmd_params.at("vel_percent"));
				velocity = std::max(std::min(100, velocity), 0);
				param.vel_percent = velocity;

				auto mat = target.model->calculator().calculateExpression(cmd_params.at("vel"));
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), param.vel);

				mat = target.model->calculator().calculateExpression(cmd_params.at("acc"));
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), param.acc);

				mat = target.model->calculator().calculateExpression(cmd_params.at("dec"));
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), param.dec);

				param.increase_status[param.moving_type] = std::max(std::min(1, std::stoi(cmd_params.at("direction"))), -1);

				std::shared_ptr<aris::plan::PlanTarget> planptr = cs.currentExecuteTarget();

				//当前有指令在执行//
				if (planptr && planptr->plan != this_p)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + "Other command is running");

				if (j_count.exchange(param.increase_count))
				{
					target.option |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
				}
				else
				{
					std::fill(target.mot_options.begin(), target.mot_options.end(), aris::plan::Plan::MotionOption::NOT_CHECK_POS_FOLLOWING_ERROR | aris::plan::Plan::MotionOption::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | aris::plan::Plan::MotionOption::USE_TARGET_POS);
				}
			}
			else if (p.first == "stop")
			{
				j_count.exchange(0);
				target.option |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
			}
		}
	}
	std::atomic_int32_t JCParam::jx_count = 0;
	auto JX::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JCParam param;

		param.pm_target.resize(16, 0.0);
		param.moving_type = 0;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		set_jogc_input_param(this, params, target, param, param.jx_count);

		target.param = param;

	}
	auto JX::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JCParam&>(target.param);
		char eu_type[4]{ '1', '2', '3', '\0' };

		// 前三维为xyz，后三维是w的积分，注意没有物理含义
		static double target_p[6];

		// get current pe //
		static double p_now[6], v_now[6], a_now[6];
		if (target.count == 1)
		{
			target.model->generalMotionPool()[0].getMpe(target_p);
			std::fill_n(target_p + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMpe(p_now, eu_type);
			std::fill_n(p_now + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMve(v_now, eu_type);
			target.model->generalMotionPool()[0].getMae(a_now, eu_type);
		}

		// init status //
		double max_vel[6]{ 0,0,0,0,0,0 };

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = param.vel[i] * 1.0 * g_vel_percent.load()/100.0;
			target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
		}
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		int finished[6]{ 0,0,0,0,0,0 };
		for (int i = 0; i < 6; i++)
		{
			aris::Size t;
			finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], param.acc[i], param.dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//当计数器jx_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.jx_count == 0)
		{
			param.increase_status[param.moving_type] = 0;
		}
		else
		{
			--param.jx_count;
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		//target.model->generalMotionPool()[0].getMpm(pm_now);
		param.tool->getPm(*param.wobj, pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);

		//绝对坐标系
		if (param.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, param.pm_target.data());
		}
		//工具坐标系
		else if (param.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, param.pm_target.data());
		}

		//target.model->generalMotionPool().at(0).setMpm(param.pm_target.data());
		param.tool->setPm(*param.wobj, param.pm_target.data());
		target.model->generalMotionPool().at(0).updMpm();

		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			cout << "jx_count:" << std::endl;
			cout << param.jx_count << "  ";
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			cout << param.increase_status[param.moving_type] << "  ";
			cout << std::endl;
			cout << "p_next:" << std::endl;
			cout << p_next[param.moving_type] << "  ";
			cout << std::endl;
			cout << "v_next:" << std::endl;
			cout << v_next[param.moving_type] << "  ";
			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (int i = 0; i < 6; i++)
		{
			lout << target_p[i] << " ";
			lout << p_now[i] << " ";
			lout << v_now[i] << " ";
			lout << a_now[i] << " ";
			lout << controller->motionAtAbs(i).actualPos() << " ";
			lout << controller->motionAtAbs(i).actualVel() << " ";
		}
		lout << std::endl;

		return finished[param.moving_type];
	}
	auto JX::collectNrt(PlanTarget &target)->void
	{
		JCParam::jx_count = 0;
		if (target.ret_code < 0)
		{
			JCParam::jx_count.store(0);
		}
	}
	JX::~JX() = default;
	JX::JX(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jx\">"
			JOGC_PARAM_STRING
			"</Command>");
	}


	// 示教运动--jogy //
	std::atomic_int32_t JCParam::jy_count = 0;
	auto JY::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JCParam param;

		param.pm_target.resize(16, 0.0);
		param.moving_type = 1;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		set_jogc_input_param(this, params, target, param, param.jy_count);

		target.param = param;

	}
	auto JY::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JCParam&>(target.param);
		char eu_type[4]{ '1', '2', '3', '\0' };

		// 前三维为xyz，后三维是w的积分，注意没有物理含义
		static double target_p[6];

		// get current pe //
		static double p_now[6], v_now[6], a_now[6];
		if (target.count == 1)
		{
			target.model->generalMotionPool()[0].getMpe(target_p);
			std::fill_n(target_p + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMpe(p_now, eu_type);
			std::fill_n(p_now + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMve(v_now, eu_type);
			target.model->generalMotionPool()[0].getMae(a_now, eu_type);
		}

		// init status //
		double max_vel[6]{ 0,0,0,0,0,0 };

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = param.vel[i] * 1.0 * g_vel_percent.load()/100.0;
			target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
		}
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		int finished[6]{ 0,0,0,0,0,0 };
		for (int i = 0; i < 6; i++)
		{
			aris::Size t;
			finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], param.acc[i], param.dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//当计数器jy_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.jy_count == 0)
		{
			param.increase_status[param.moving_type] = 0;
		}
		else
		{
			--param.jy_count;
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		param.tool->getPm(*param.wobj, pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);

		//绝对坐标系
		if (param.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, param.pm_target.data());
		}
		//工具坐标系
		else if (param.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, param.pm_target.data());
		}

		param.tool->setPm(*param.wobj, param.pm_target.data());
		target.model->generalMotionPool().at(0).updMpm();

		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << "jy_count:" << std::endl;
			cout << param.jy_count << "  ";
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			cout << param.increase_status[param.moving_type] << "  ";
			cout << std::endl;
			cout << "p_next:" << std::endl;
			cout << p_next[param.moving_type] << "  ";
			cout << std::endl;
			cout << "v_next:" << std::endl;
			cout << v_next[param.moving_type] << "  ";
			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (int i = 0; i < 6; i++)
		{
			lout << target_p[i] << " ";
			lout << p_now[i] << " ";
			lout << v_now[i] << " ";
			lout << a_now[i] << " ";
			lout << controller->motionAtAbs(i).actualPos() << " ";
			lout << controller->motionAtAbs(i).actualVel() << " ";
		}
		lout << std::endl;

		return finished[param.moving_type];
	}
	auto JY::collectNrt(PlanTarget &target)->void
	{
		JCParam::jy_count = 0;
		if (target.ret_code < 0)
		{
			JCParam::jy_count.store(0);
		}
	}
	JY::~JY() = default;
	JY::JY(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jy\">"
			JOGC_PARAM_STRING
			"</Command>");
	}


	// 示教运动--jogz //
	std::atomic_int32_t JCParam::jz_count = 0;
	auto JZ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JCParam param;

		param.pm_target.resize(16, 0.0);
		param.moving_type = 2;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		set_jogc_input_param(this, params, target, param, param.jz_count);

		target.param = param;

	}
	auto JZ::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JCParam&>(target.param);
		char eu_type[4]{ '1', '2', '3', '\0' };

		// 前三维为xyz，后三维是w的积分，注意没有物理含义
		static double target_p[6];

		// get current pe //
		static double p_now[6], v_now[6], a_now[6];
		if (target.count == 1)
		{
			target.model->generalMotionPool()[0].getMpe(target_p);
			std::fill_n(target_p + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMpe(p_now, eu_type);
			std::fill_n(p_now + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMve(v_now, eu_type);
			target.model->generalMotionPool()[0].getMae(a_now, eu_type);
		}

		// init status //
		double max_vel[6]{ 0,0,0,0,0,0 };

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = param.vel[i] * 1.0 * g_vel_percent.load()/100.0;
			target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
		}
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		int finished[6]{ 0,0,0,0,0,0 };
		for (int i = 0; i < 6; i++)
		{
			aris::Size t;
			finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], param.acc[i], param.dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//当计数器jz_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.jz_count == 0)
		{
			param.increase_status[param.moving_type] = 0;
		}
		else
		{
			--param.jz_count;
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		param.tool->getPm(*param.wobj, pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);

		//绝对坐标系
		if (param.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, param.pm_target.data());
		}
		//工具坐标系
		else if (param.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, param.pm_target.data());
		}

		param.tool->setPm(*param.wobj, param.pm_target.data());
		target.model->generalMotionPool().at(0).updMpm();

		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << "jz_count:" << std::endl;
			cout << param.jz_count << "  ";
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			cout << param.increase_status[param.moving_type] << "  ";
			cout << std::endl;
			cout << "p_next:" << std::endl;
			cout << p_next[param.moving_type] << "  ";
			cout << std::endl;
			cout << "v_next:" << std::endl;
			cout << v_next[param.moving_type] << "  ";
			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (int i = 0; i < 6; i++)
		{
			lout << target_p[i] << " ";
			lout << p_now[i] << " ";
			lout << v_now[i] << " ";
			lout << a_now[i] << " ";
			lout << controller->motionAtAbs(i).actualPos() << " ";
			lout << controller->motionAtAbs(i).actualVel() << " ";
		}
		lout << std::endl;

		return finished[param.moving_type];
	}
	auto JZ::collectNrt(PlanTarget &target)->void
	{
		JCParam::jz_count = 0;
		if (target.ret_code < 0)
		{
			JCParam::jz_count.store(0);
		}
	}
	JZ::~JZ() = default;
	JZ::JZ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jz\">"
			JOGC_PARAM_STRING
			"</Command>");
	}


	// 示教运动--jogrx //
	std::atomic_int32_t JCParam::jrx_count = 0;
	auto JRX::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JCParam param;

		param.pm_target.resize(16, 0.0);
		param.moving_type = 3;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		set_jogc_input_param(this, params, target, param, param.jrx_count);

		target.param = param;

	}
	auto JRX::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JCParam&>(target.param);
		char eu_type[4]{ '1', '2', '3', '\0' };

		// 前三维为xyz，后三维是w的积分，注意没有物理含义
		static double target_p[6];

		// get current pe //
		static double p_now[6], v_now[6], a_now[6];
		if (target.count == 1)
		{
			target.model->generalMotionPool()[0].getMpe(target_p);
			std::fill_n(target_p + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMpe(p_now, eu_type);
			std::fill_n(p_now + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMve(v_now, eu_type);
			target.model->generalMotionPool()[0].getMae(a_now, eu_type);
		}

		// init status //
		double max_vel[6]{ 0,0,0,0,0,0 };

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = param.vel[i] * 1.0 * g_vel_percent.load()/100.0;
			target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
		}
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		int finished[6]{ 0,0,0,0,0,0 };
		for (int i = 0; i < 6; i++)
		{
			aris::Size t;
			finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], param.acc[i], param.dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//当计数器jrx_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.jrx_count == 0)
		{
			param.increase_status[param.moving_type] = 0;
		}
		else
		{
			--param.jrx_count;
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		param.tool->getPm(*param.wobj, pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);

		//绝对坐标系
		if (param.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, param.pm_target.data());
		}
		//工具坐标系
		else if (param.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, param.pm_target.data());
		}

		param.tool->setPm(*param.wobj, param.pm_target.data());
		target.model->generalMotionPool().at(0).updMpm();

		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << "jrx_count:" << std::endl;
			cout << param.jrx_count << "  ";
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			cout << param.increase_status[param.moving_type] << "  ";
			cout << std::endl;
			cout << "p_next:" << std::endl;
			cout << p_next[param.moving_type] << "  ";
			cout << std::endl;
			cout << "v_next:" << std::endl;
			cout << v_next[param.moving_type] << "  ";
			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (int i = 0; i < 6; i++)
		{
			lout << target_p[i] << " ";
			lout << p_now[i] << " ";
			lout << v_now[i] << " ";
			lout << a_now[i] << " ";
			lout << controller->motionAtAbs(i).actualPos() << " ";
			lout << controller->motionAtAbs(i).actualVel() << " ";
		}
		lout << std::endl;

		return finished[param.moving_type];
	}
	auto JRX::collectNrt(PlanTarget &target)->void
	{
		JCParam::jrx_count = 0;
		if (target.ret_code < 0)
		{
			JCParam::jrx_count.store(0);
		}
	}
	JRX::~JRX() = default;
	JRX::JRX(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jrx\">"
			JOGC_PARAM_STRING
			"</Command>");
	}


	// 示教运动--jogry //
	std::atomic_int32_t JCParam::jry_count = 0;
	auto JRY::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JCParam param;

		param.pm_target.resize(16, 0.0);
		param.moving_type = 4;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		set_jogc_input_param(this, params, target, param, param.jry_count);

		target.param = param;

	}
	auto JRY::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JCParam&>(target.param);
		char eu_type[4]{ '1', '2', '3', '\0' };

		// 前三维为xyz，后三维是w的积分，注意没有物理含义
		static double target_p[6];

		// get current pe //
		static double p_now[6], v_now[6], a_now[6];
		if (target.count == 1)
		{
			target.model->generalMotionPool()[0].getMpe(target_p);
			std::fill_n(target_p + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMpe(p_now, eu_type);
			std::fill_n(p_now + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMve(v_now, eu_type);
			target.model->generalMotionPool()[0].getMae(a_now, eu_type);
		}

		// init status //
		double max_vel[6]{ 0,0,0,0,0,0 };

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = param.vel[i] * 1.0 * g_vel_percent.load()/100.0;
			target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
		}
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		int finished[6]{ 0,0,0,0,0,0 };
		for (int i = 0; i < 6; i++)
		{
			aris::Size t;
			finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], param.acc[i], param.dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//当计数器jry_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.jry_count == 0)
		{
			param.increase_status[param.moving_type] = 0;
		}
		else
		{
			--param.jry_count;
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		param.tool->getPm(*param.wobj, pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);

		//绝对坐标系
		if (param.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, param.pm_target.data());
		}
		//工具坐标系
		else if (param.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, param.pm_target.data());
		}

		param.tool->setPm(*param.wobj, param.pm_target.data());
		target.model->generalMotionPool().at(0).updMpm();

		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << "jry_count:" << std::endl;
			cout << param.jry_count << "  ";
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			cout << param.increase_status[param.moving_type] << "  ";
			cout << std::endl;
			cout << "p_next:" << std::endl;
			cout << p_next[param.moving_type] << "  ";
			cout << std::endl;
			cout << "v_next:" << std::endl;
			cout << v_next[param.moving_type] << "  ";
			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (int i = 0; i < 6; i++)
		{
			lout << target_p[i] << " ";
			lout << p_now[i] << " ";
			lout << v_now[i] << " ";
			lout << a_now[i] << " ";
			lout << controller->motionAtAbs(i).actualPos() << " ";
			lout << controller->motionAtAbs(i).actualVel() << " ";
		}
		lout << std::endl;

		return finished[param.moving_type];
	}
	auto JRY::collectNrt(PlanTarget &target)->void
	{
		JCParam::jry_count = 0;
		if (target.ret_code < 0)
		{
			JCParam::jry_count.store(0);
		}
	}
	JRY::~JRY() = default;
	JRY::JRY(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jry\">"
			JOGC_PARAM_STRING
			"</Command>");
	}


	// 示教运动--jogrz //
	std::atomic_int32_t JCParam::jrz_count = 0;
	auto JRZ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JCParam param;

		param.pm_target.resize(16, 0.0);
		param.moving_type = 5;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		set_jogc_input_param(this, params, target, param, param.jrz_count);

		target.param = param;

	}
	auto JRZ::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JCParam&>(target.param);
		char eu_type[4]{ '1', '2', '3', '\0' };

		// 前三维为xyz，后三维是w的积分，注意没有物理含义
		static double target_p[6];

		// get current pe //
		static double p_now[6], v_now[6], a_now[6];
		if (target.count == 1)
		{
			target.model->generalMotionPool()[0].getMpe(target_p);
			std::fill_n(target_p + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMpe(p_now, eu_type);
			std::fill_n(p_now + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMve(v_now, eu_type);
			target.model->generalMotionPool()[0].getMae(a_now, eu_type);
		}

		// init status //
		double max_vel[6]{ 0,0,0,0,0,0 };

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = param.vel[i] * 1.0 * g_vel_percent.load()/100.0;
			target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
		}
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		int finished[6]{ 0,0,0,0,0,0 };
		for (int i = 0; i < 6; i++)
		{
			aris::Size t;
			finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], param.acc[i], param.dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//当计数器jrz_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.jrz_count == 0)
		{
			param.increase_status[param.moving_type] = 0;
		}
		else
		{
			--param.jrz_count;
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		param.tool->getPm(*param.wobj, pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);

		//绝对坐标系
		if (param.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, param.pm_target.data());
		}
		//工具坐标系
		else if (param.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, param.pm_target.data());
		}

		param.tool->setPm(*param.wobj, param.pm_target.data());
		target.model->generalMotionPool().at(0).updMpm();

		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << "jrz_count:" << std::endl;
			cout << param.jrz_count << "  ";
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			cout << param.increase_status[param.moving_type] << "  ";
			cout << std::endl;
			cout << "p_next:" << std::endl;
			cout << p_next[param.moving_type] << "  ";
			cout << std::endl;
			cout << "v_next:" << std::endl;
			cout << v_next[param.moving_type] << "  ";
			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (int i = 0; i < 6; i++)
		{
			lout << target_p[i] << " ";
			lout << p_now[i] << " ";
			lout << v_now[i] << " ";
			lout << a_now[i] << " ";
			lout << controller->motionAtAbs(i).actualPos() << " ";
			lout << controller->motionAtAbs(i).actualVel() << " ";
		}
		lout << std::endl;

		return finished[param.moving_type];
	}
	auto JRZ::collectNrt(PlanTarget &target)->void
	{
		JCParam::jrz_count = 0;
		if (target.ret_code < 0)
		{
			JCParam::jrz_count.store(0);
		}
	}
	JRZ::~JRZ() = default;
	JRZ::JRZ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jrz\">"
			JOGC_PARAM_STRING
			"</Command>");
	}

	
	// 力传感器信号测试 //
    struct FSParam
    {
        bool real_data;
        int time;
        uint16_t datanum;
        float Fx,Fy,Fz,Mx,My,Mz;
    };
    auto FSSignal::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
        {
            FSParam param;
            for (auto &p : params)
            {
                if (p.first == "real_data")
                {
                    param.real_data = std::stod(p.second);
                }
                else if (p.first == "time")
                {
                    param.time = std::stoi(p.second);
                }
            }

            param.Fx = 0.0;
            param.Fy = 0.0;
            param.Fz = 0.0;
            param.Mx = 0.0;
            param.My = 0.0;
            param.Mz = 0.0;
            target.param = param;
			std::fill(target.mot_options.begin(), target.mot_options.end(),
				Plan::NOT_CHECK_ENABLE);

			std::vector<std::pair<std::string, std::any>> ret;
			target.ret = ret;
        }
    auto FSSignal::executeRT(PlanTarget &target)->int
        {
            auto &param = std::any_cast<FSParam&>(target.param);
            // 访问主站 //
            auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);
            if (param.real_data)
            {
                controller->slavePool().at(6).readPdo(0x6030, 0x00, &param.datanum ,16);
                controller->slavePool().at(6).readPdo(0x6030, 0x01, &param.Fx ,32);
                controller->slavePool().at(6).readPdo(0x6030, 0x02, &param.Fy, 32);
                controller->slavePool().at(6).readPdo(0x6030, 0x03, &param.Fz, 32);
                controller->slavePool().at(6).readPdo(0x6030, 0x04, &param.Mx, 32);
                controller->slavePool().at(6).readPdo(0x6030, 0x05, &param.My, 32);
                controller->slavePool().at(6).readPdo(0x6030, 0x06, &param.Mz, 32);
            }
            else
            {
                controller->slavePool().at(6).readPdo(0x6030, 0x00, &param.datanum ,16);
                controller->slavePool().at(6).readPdo(0x6020, 0x01, &param.Fx, 32);
                controller->slavePool().at(6).readPdo(0x6020, 0x02, &param.Fy, 32);
                controller->slavePool().at(6).readPdo(0x6020, 0x03, &param.Fz, 32);
                controller->slavePool().at(6).readPdo(0x6020, 0x04, &param.Mx, 32);
                controller->slavePool().at(6).readPdo(0x6020, 0x05, &param.My, 32);
                controller->slavePool().at(6).readPdo(0x6020, 0x06, &param.Mz, 32);
            }

            //print//
            auto &cout = controller->mout();
            if (target.count % 100 == 0)
            {
                cout << std::setw(6) << param.datanum << "  ";
                cout << std::setw(6) << param.Fx << "  ";
                cout << std::setw(6) << param.Fy << "  ";
                cout << std::setw(6) << param.Fz << "  ";
                cout << std::setw(6) << param.Mx << "  ";
                cout << std::setw(6) << param.My << "  ";
                cout << std::setw(6) << param.Mz << "  ";
                cout << std::endl;
                cout << "----------------------------------------------------" << std::endl;
            }

            //log//
            auto &lout = controller->lout();
            {
                lout << param.Fx << " ";
                lout << param.Fy << " ";
                lout << param.Fz << " ";
                lout << param.Mx << " ";
                lout << param.My << " ";
                lout << param.Mz << " ";
                lout << std::endl;
            }
            param.time--;
            return param.time;
        }
    auto FSSignal::collectNrt(PlanTarget &target)->void {}
    FSSignal::FSSignal(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"fssignal\">"
            "	<GroupParam>"
            "		<Param name=\"real_data\" default=\"1\"/>"
            "		<Param name=\"time\" default=\"100000\"/>"
            "	</GroupParam>"
            "</Command>");
    }


	// 配置控制器参数 //
	struct SetConParam
	{
		uint16_t motion_phyid;
		uint16_t ecslave_phyid;
		uint16_t ecslave_dc;
		bool is_motion;
	};
	auto SetCon::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, can not set controller,please stop the cs!");

		SetConParam param;
		param.motion_phyid = 0;
		param.ecslave_phyid = 0;
		param.is_motion = 1;
		for (auto &p : params)
		{
			//motion physical id number//
			if (p.first == "motion_phyid")
			{
				param.motion_phyid = std::stoi(p.second);
				param.is_motion = 1;
			}
			//ethercat slave physical id number//
			else if (p.first == "ecslave_phyid")
			{
				param.ecslave_phyid = std::stoi(p.second);
				param.is_motion = 0;
				param.ecslave_dc = std::stoi(params.at("ecslave_dc"));
			}
		}

		//std::unique_ptr<aris::control::Controller> controller(aris::robot::createControllerRokaeXB4());
		//controller->slavePool().clear();	//清除slavePool中的元素，后面重新添加
		auto &controller = target.controller;

		//configure motion//
		if(param.is_motion)
		{
			controller->slavePool().add<aris::control::EthercatMotion>();
			controller->slavePool().back().setPhyId(param.motion_phyid);
			dynamic_cast<aris::control::EthercatMotion&>(controller->slavePool().back()).scanInfoForCurrentSlave();
			dynamic_cast<aris::control::EthercatMotion&>(controller->slavePool().back()).scanPdoForCurrentSlave();
			dynamic_cast<aris::control::EthercatMotion&>(controller->slavePool().back()).setDcAssignActivate(0x300);
		}	
		//configure ect slave//
		else if(!param.is_motion)
		{
			controller->slavePool().add<aris::control::EthercatSlave>();
			controller->slavePool().back().setPhyId(param.ecslave_phyid);
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
			//倍福ECT Slave 耦合器,ecslave_dc=0x300；耦合器后面的模块，,ecslave_dc=0x00//
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).setDcAssignActivate(param.ecslave_dc);
		}
		//cs.resetController(controller);

		//std::cout << controller->xmlString() << std::endl;
		/*
		auto xmlpath = std::filesystem::absolute(".");
		const std::string xmlfile = "kaanh.xml";
		xmlpath = xmlpath / xmlfile;
		cs.saveXmlFile(xmlpath.string().c_str());
		*/

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SetCon::SetCon(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setCon\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<Param name=\"motion_phyid\" default=\"0\"/>"
			"			<GroupParam>"
			"				<Param name=\"ecslave_phyid\" default=\"6\"/>"
			"				<Param name=\"ecslave_dc\" default=\"768\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}


	// 配置DH参数 //
	struct SetDHParam
	{
		std::vector<double>dh;
		double tool_offset;
		int axis_num;
	};
	auto SetDH::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, can not set DH parameters,please stop the cs!");

		SetDHParam dhparam;
		dhparam.dh.clear();

		for (auto &p : params)
		{
			//6轴DH参数//
			if (p.first == "six_axes")
			{
				dhparam.dh.resize(6, 0.0);
				dhparam.dh[0] = std::stod(params.at("d1_six_axes"));
				dhparam.dh[1] = std::stod(params.at("a1_six_axes"));
				dhparam.dh[2] = std::stod(params.at("a2_six_axes"));
				dhparam.dh[3] = std::stod(params.at("d3_six_axes"));
				dhparam.dh[4] = std::stod(params.at("a3_six_axes"));
				dhparam.dh[5] = std::stod(params.at("d4_six_axes"));
				dhparam.tool_offset = std::stod(params.at("tool0_six_axes"));
				dhparam.axis_num = 6;
			}
			//7轴DH参数//
			else if (p.first == "seven_axes")
			{
				dhparam.dh.resize(3, 0.0);
				dhparam.dh[0] = std::stod(params.at("d1_seven_axes"));
				dhparam.dh[1] = std::stod(params.at("d3_seven_axes"));
				dhparam.dh[2] = std::stod(params.at("d5_seven_axes"));
				dhparam.tool_offset = std::stod(params.at("tool0_seven_axes"));
				dhparam.axis_num = 7;
			}
		}

		aris::dynamic::PumaParam param_puma;
		aris::dynamic::SevenAxisParam param_7axes;
		if (dhparam.axis_num == 6)
		{
			param_puma.d1 = dhparam.dh[0];
			param_puma.a1 = dhparam.dh[1];
			param_puma.a2 = dhparam.dh[2];
			param_puma.d3 = dhparam.dh[3];
			param_puma.a3 = dhparam.dh[4];
			param_puma.d4 = dhparam.dh[5];
			param_puma.tool0_pe[2] = dhparam.tool_offset;

			auto model = aris::dynamic::createModelPuma(param_puma);
			cs.resetModel(model.release());
		}
		else if (dhparam.axis_num == 7)
		{
			param_7axes.d1 = dhparam.dh[0];
			param_7axes.d3 = dhparam.dh[1];
			param_7axes.d5 = dhparam.dh[2];
			param_7axes.tool0_pe[2] = dhparam.tool_offset;

			auto m = aris::dynamic::createModelSevenAxis(param_7axes);
			cs.resetModel(m.release());
		}
		else{ }

		/*
		//动力学标定参数//
		param.iv_vec =
		{
			{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.59026333537827,   0.00000000000000,   0.00000000000000,   0.00000000000000 },
			{ 0.00000000000000, -0.02551872200978,   0.00000000000000,   3.05660683326413,   2.85905166943306,   0.00000000000000,   0.00000000000000, -0.00855352993039, -0.09946674483372, -0.00712210734359 },
			{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.02733022277747,   0.00000000000000,   0.37382629693302,   0.00000000000000,   0.00312006493276, -0.00578410451516,   0.00570606128540 },
			{ 0.00000000000000,   1.06223330086669,   0.00000000000000,   0.00311748242960,   0.00000000000000,   0.24420385558544,   0.24970286555981,   0.00305759215246, -0.66644096559686,   0.00228253380852 },
			{ 0.00000000000000,   0.05362286897910,   0.00528925153464, -0.00842588023014,   0.00128498153337, -0.00389810210572,   0.00000000000000, -0.00223677867576, -0.03365036368035, -0.00415647085627 },
			{ 0.00000000000000,   0.00000000000000,   0.00066049870832,   0.00012563800445, -0.00085124094833,   0.04209529937135,   0.04102481443654, -0.00067596644891,   0.00017482449876, -0.00041025776053 },
		};
		param.mot_frc_vec =
		{
			{ 9.34994758321915, 7.80825641041495, 0.00000000000000 },
			{ 11.64080253106441, 13.26518528472506, 3.55567932576820 },
			{ 4.77014054273075, 7.85644357492508, 0.34445460269183 },
			{ 3.63141668516122, 3.35461524886318, 0.14824771620542 },
			{ 2.58310846982020, 1.41963212641879, 0.04855267273770 },
			{ 1.78373986219597, 0.31920640440152, 0.03381545544099 },
		};
		*/
		
		/*
		auto xmlpath = std::filesystem::absolute(".");
		const std::string xmlfile = "kaanh.xml";
		xmlpath = xmlpath / xmlfile;
		cs.saveXmlFile(xmlpath.string().c_str());
		*/

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

	}
	SetDH::SetDH(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setDH\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<GroupParam name=\"start_group\">"
			"				<Param name=\"six_axes\"/>"
			"				<Param name=\"d1_six_axes\" default=\"0.3295\"/>"
			"				<Param name=\"a1_six_axes\" default=\"0.04\"/>"
			"				<Param name=\"a2_six_axes\" default=\"0.275\"/>"
			"				<Param name=\"d3_six_axes\" default=\"0.0\"/>"
			"				<Param name=\"a3_six_axes\" default=\"0.025\"/>"
			"				<Param name=\"d4_six_axes\" default=\"0.28\"/>"
			"				<Param name=\"tool0_six_axes\" default=\"0.078\"/>"
			"			</GroupParam>"
			"			<GroupParam>"
			"				<Param name=\"seven_axes\"/>"
			"				<Param name=\"d1_seven_axes\" default=\"0.3705\"/>"
			"				<Param name=\"d3_seven_axes\" default=\"0.330\"/>"
			"				<Param name=\"d5_seven_axes\" default=\"0.320\"/>"
			"				<Param name=\"tool0_seven_axes\" default=\"0.2205\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}

	
	// 配置PG参数 //
	struct SetPGParam
	{
		std::string name;
		std::vector<double> pe;
		uint16_t part_id;
		std::string file_path;	
	};
	auto SetPG::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, can not set DH parameters,please stop the cs!");

		SetPGParam param;
		param.pe.clear();
		param.pe.resize(6, 0.0);
		for (auto &p : params)
		{
			if (p.first == "name")
			{
				param.name = p.second;
			}
			else if (p.first == "pe")
			{
				auto mat = target.model->calculator().calculateExpression(params.at("pe"));
				if (mat.size() == param.pe.size())
				{
					param.pe.assign(mat.begin(), mat.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}
			}
			else if (p.first == "part_id")
			{
				param.part_id = std::stoi(p.second);
			}
			else if (p.first == "file_path")
			{
				param.file_path = p.second;
			}
		}

		target.model->partPool().at(param.part_id).geometryPool().clear();
		target.model->partPool().at(param.part_id).geometryPool().add<FileGeometry>(param.name, param.file_path, param.pe.data());

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

	}
	SetPG::SetPG(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setPG\">"
			"	<GroupParam>"
			"		<Param name=\"name\" default=\"test\"/>"
			"		<Param name=\"pe\" default=\"{0, 0, 0, -0, 0, -0}\"/>"
			"		<Param name=\"part_id\" default=\"6\"/>"
			"		<Param name=\"file_path\" default=\"/RobotGallery/Rokae/XB4/l0.data\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	

	// 配置UI //
	struct SetUIParam
	{
		std::string ui_path;
	};
	auto SetUI::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, can not set UI, please stop the cs!");

		SetUIParam param;

		for (auto &p : params)
		{
			if (p.first == "ui_path")
			{
				param.ui_path = p.second;
			}
		}
		
		auto xmlpath = std::filesystem::absolute(".");
		const std::string xmlfile = "kaanh.xml";
		auto xmlpath_ui = xmlpath / param.ui_path;	
		xmlpath = xmlpath / xmlfile;

		cs.interfaceRoot().loadXmlFile(xmlpath_ui.string().c_str());
		//cs.saveXmlFile(xmlpath.string().c_str());

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

	}
	SetUI::SetUI(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setUI\">"
			"	<GroupParam>"
			"		<Param name=\"ui_path\" default=\"interface_kaanh.xml\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 配置关节——home偏置、角度-编码系数、角度、角速度、角加速度上下限 //
	struct SetDriverParam
	{
		std::vector<bool> joint_active_vec;
		double pos_factor;
		double pos_max;
		double pos_min;
		double vel_max;
		double vel_min;
		double acc_max;
		double acc_min;
		double pos_offset;
	};
	auto SetDriver::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, can not set pos_factor,pos,vel and acc,please stop the cs!");
		//std::unique_ptr<aris::control::Controller> controller(kaanh::createControllerRokaeXB4());
		auto &controller = target.controller;
		SetDriverParam param;

		//initial//
		{
			param.joint_active_vec.clear();
			param.pos_factor = 0.0;
			param.pos_max = 0.0;
			param.pos_min = 0.0;
			param.vel_max = 0.0;
			param.vel_min = 0.0;
			param.acc_max = 0.0;
			param.acc_min = 0.0;
			param.pos_offset = 0.0;
		}

		for (auto &p : params)
		{
			if (p.first == "motion_id")
			{
				param.joint_active_vec.resize(50, false);
				param.joint_active_vec.at(std::stoi(p.second)) = true;
			}
			else if(p.first == "pos_factor")
			{
                auto m = target.model->calculator().calculateExpression(p.second);
                param.pos_factor = m.toDouble() / (2.0 * PI);
			}
			else if (p.first == "pos_max")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.pos_max = m.toDouble() * 2 * PI / 360;
			}
			else if (p.first == "pos_min")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.pos_min = m.toDouble() * 2 * PI / 360;
			}
			else if (p.first == "vel_max")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.vel_max = m.toDouble() * 2 * PI / 360;
			}
			else if (p.first == "vel_min")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.vel_min = m.toDouble() * 2 * PI / 360;
			}
			else if (p.first == "acc_max")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.acc_max = m.toDouble() * 2 * PI / 360;
			}
			else if (p.first == "acc_min")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.acc_min = m.toDouble() * 2 * PI / 360;
			}
			else if (p.first == "pos_offset")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.pos_offset = m.toDouble();
			}
		}
		
		// 设置驱动pos_factor,pos,vel,acc //
		for (int i = 0; i < param.joint_active_vec.size(); i++)
		{
			if (param.joint_active_vec[i])
			{
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setPosFactor(param.pos_factor);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setMaxPos(param.pos_max);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setMinPos(param.pos_min);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setMaxVel(param.vel_max);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setMinVel(param.vel_min);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setMaxAcc(param.acc_max);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setMinAcc(param.acc_min);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setPosOffset(param.pos_offset);
			}
		}

        std::cout << param.pos_factor << std::endl;
		//cs.resetController(controller);
		/*
		auto xmlpath = std::filesystem::absolute(".");
		const std::string xmlfile = "kaanh.xml";
		xmlpath = xmlpath / xmlfile;
		cs.saveXmlFile(xmlpath.string().c_str());
		*/
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SetDriver::SetDriver(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setDriver\">"
			"	<GroupParam>"
			"		<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"		<Param name=\"pos_factor\" default=\"0.0\"/>"
			"		<Param name=\"pos_max\" default=\"0.0\"/>"
			"		<Param name=\"pos_min\" default=\"0.0\"/>"
			"		<Param name=\"vel_max\" default=\"0.0\"/>"
			"		<Param name=\"vel_min\" default=\"0.0\"/>"
			"		<Param name=\"acc_max\" default=\"0.0\"/>"
			"		<Param name=\"acc_min\" default=\"0.0\"/>"
			"		<Param name=\"pos_offset\" default=\"0.0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 保存配置 //
	auto SaveConfig::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, can not set position offset,please stop the cs!");

        //cs.resetSensorRoot(new aris::sensor::SensorRoot);

		auto xmlpath = std::filesystem::absolute(".");
		const std::string xmlfile = "kaanh.xml";
		xmlpath = xmlpath / xmlfile;
		cs.saveXmlFile(xmlpath.string().c_str());

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SaveConfig::SaveConfig(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"saveConfig\">"
			"</Command>");
	}


	// 保存示教点 //
	auto SavePoint::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
	
		std::string point_name;
		for (auto &p : params)
		{
			if (p.first == "point_name")
			{
				point_name = p.second;
			}
		}

		auto temp_pos = save_point.load();
		aris::core::Matrix mat(1, 10, temp_pos.data());
		if (target.model->variablePool().findByName(point_name) != target.model->variablePool().end())
		{
			dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName(point_name))->data() = mat;
		}
		else
		{
			target.model->variablePool().add<aris::dynamic::MatrixVariable>(point_name, mat);
		}

		auto xmlpath = std::filesystem::absolute(".");
		const std::string xmlfile = "kaanh.xml";
		xmlpath = xmlpath / xmlfile;
		cs.saveXmlFile(xmlpath.string().c_str());

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SavePoint::SavePoint(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"savepoint\">"
			"	<GroupParam>"
			"		<Param name=\"point_name\" default=\"homepoint\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	//设置全局速度//
	struct SetVelParam
	{
		int vel_percent;
	};
	auto SetVel::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		SetVelParam param;
		for (auto &p : params)
		{
			if (p.first == "vel_percent")
			{
				param.vel_percent = std::stoi(p.second);
			}
			//限制vel_percent在0~100之间//
			param.vel_percent = std::max(std::min(param.vel_percent, 100), 0);
		}
		g_vel_percent.store(param.vel_percent);

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SetVel::SetVel(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setvel\">"
			"	<GroupParam>"
			"		<Param name=\"vel_percent\" abbreviation=\"p\" default=\"0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	//调速//
	struct UDVelParam
	{
		int up, down;
	};
	auto UDVel::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		UDVelParam param;
		for (auto &p : params)
		{
			if (p.first == "up")
			{
				param.up = std::stoi(p.second);
				//限制up在0~50之间//
				param.up = std::max(std::min(param.up, 50), 0);
				auto temp = g_vel_percent.load() + param.up;
				g_vel_percent.store(temp);
			}
			else if (p.first == "down")
			{
				param.down = std::stoi(p.second);
				//限制down在0~50之间//
				param.down = std::max(std::min(param.down, 50), 0);
				auto temp = g_vel_percent.load() + param.down;
				g_vel_percent.store(temp);
			}	
		}

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	UDVel::UDVel(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"udvel\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<Param name=\"up\" default=\"1\"/>"
			"			<Param name=\"down\" default=\"1\"/>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}


	// set cycle_time for driver with SDO //
	struct SetCTParam
	{
		int16_t cycle_time;
	};
	auto SetCT::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		SetCTParam param;
		param.cycle_time = 1;
		auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);

		for (auto &p : params)
		{
			if (p.first == "ct")
			{
				param.cycle_time = std::stoi(p.second);
			}
		}

		for (aris::Size i = 0; i < controller->slavePool().size(); i++)
		{
			controller->slavePool().at(i).writeSdo(0x60c2, 0x01, &param.cycle_time);
		}
		
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SetCT::SetCT(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setCT\">"
			"	<GroupParam>"
			"		<Param name=\"ct\" abbreviation=\"t\" default=\"1\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	

    auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
	{
        std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

        plan_root->planPool().add<aris::plan::Enable>();
        plan_root->planPool().add<aris::plan::Disable>();
        plan_root->planPool().add<aris::plan::Home>();
        plan_root->planPool().add<aris::plan::Mode>();
        plan_root->planPool().add<aris::plan::Show>();
        plan_root->planPool().add<aris::plan::Sleep>();
        plan_root->planPool().add<aris::plan::Recover>();
        auto &rs = plan_root->planPool().add<aris::plan::Reset>();
        //for qifan robot//
        rs.command().findParam("pos")->setDefaultValue("{0.5,0.353,0.5,0.5,0.5,0.5}");

        //for rokae robot//
        //rs.command().findParam("pos")->setDefaultValue("{0.5,0.3925,0.7899,0.5,0.5,0.5}");

        plan_root->planPool().add<aris::plan::MoveAbsJ>();
        plan_root->planPool().add<aris::plan::MoveL>();
        plan_root->planPool().add<aris::plan::MoveJ>();

        plan_root->planPool().add<aris::plan::GetXml>();
        plan_root->planPool().add<aris::plan::SetXml>();
        plan_root->planPool().add<aris::plan::Start>();
        plan_root->planPool().add<aris::plan::Stop>();

		plan_root->planPool().add<kaanh::Get>();
		plan_root->planPool().add<kaanh::MoveAbJ>();
		plan_root->planPool().add<kaanh::MoveT>();
		plan_root->planPool().add<kaanh::MoveJM>();
		plan_root->planPool().add<kaanh::MoveC>();
		plan_root->planPool().add<kaanh::JogC>();
		plan_root->planPool().add<kaanh::JogJ>();
		plan_root->planPool().add<kaanh::JogJ1>();
		plan_root->planPool().add<kaanh::JogJ2>();
		plan_root->planPool().add<kaanh::JogJ3>();
		plan_root->planPool().add<kaanh::JogJ4>();
		plan_root->planPool().add<kaanh::JogJ5>();
		plan_root->planPool().add<kaanh::JogJ6>();
		plan_root->planPool().add<kaanh::JogJ7>();
		plan_root->planPool().add<kaanh::DMode>();
        plan_root->planPool().add<kaanh::DEnable>();
        plan_root->planPool().add<kaanh::DDisable>();
		plan_root->planPool().add<kaanh::DJ1>();
		plan_root->planPool().add<kaanh::DJ2>();
		plan_root->planPool().add<kaanh::DJ3>();
		plan_root->planPool().add<kaanh::DMoveAbsJ>();
		plan_root->planPool().add<kaanh::JX>();
		plan_root->planPool().add<kaanh::JY>();
		plan_root->planPool().add<kaanh::JZ>();
		plan_root->planPool().add<kaanh::JRX>();
		plan_root->planPool().add<kaanh::JRY>();
		plan_root->planPool().add<kaanh::JRZ>();
		plan_root->planPool().add<kaanh::FSSignal>();
		plan_root->planPool().add<kaanh::SetCon>();
		plan_root->planPool().add<kaanh::SetDH>();
		plan_root->planPool().add<kaanh::SetPG>();
		plan_root->planPool().add<kaanh::SetUI>();
		plan_root->planPool().add<kaanh::SetDriver>();
		plan_root->planPool().add<kaanh::SaveConfig>();
        plan_root->planPool().add<kaanh::SavePoint>();
		plan_root->planPool().add<kaanh::SetVel>();
		plan_root->planPool().add<kaanh::UDVel>();
		plan_root->planPool().add<kaanh::SetCT>();

		return plan_root;
	}
}
