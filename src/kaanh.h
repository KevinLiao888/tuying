#ifndef KAANH_H_
#define KAANH_H_

#include <memory>
#include <aris.hpp>


// \brief 机器人命名空间
// \ingroup aris


namespace kaanh
{
	//电缸力检测参数声明
	//const std::string xmlpath = "C:\\Users\\kevin\\Desktop\\aris_rokae\\ServoPressorCmdList.xml";
	constexpr double ea_a = 3765.8, ea_b = 1334.8, ea_c = 45.624, ea_gra = 24, ea_index = -6, ea_gra_index = 36;  //电缸电流换算压力的系数，ea_k表示比例系数，ea_b表示截距，ea_offset表示重力影响量，ea_index表示电流扭矩系数=额定扭矩*6.28*减速比/导程/1000//

	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;

	struct SpeedParam
	{
		double w_percent;	//关节速度百分比
		double v_tcp;	//TCP线速度mm/s
		double w_tcp;	//空间旋转速度°/s
		double w_ext;	//外部轴角速度°/s
		double v_ext;	//外部轴线速度mm/s
	};

	auto createControllerQifan()->std::unique_ptr<aris::control::Controller>;
	auto createModelQifan()->std::unique_ptr<aris::dynamic::Model>;

	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>;
	auto createModelRokae()->std::unique_ptr<aris::dynamic::Model>;

	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>;

	struct CmdListParam
	{
		std::vector<std::pair<std::string, std::string>> cmd_vec;
		int current_cmd_id = 0;
		int current_plan_id = -1;
	};

	class Speed
	{
	public:
		auto setspeed(SpeedParam speed)->void
		{
			s = speed;
		};
		auto getspeed()->SpeedParam
		{
			return s;
		};
		Speed(SpeedParam speed = { 0.0,0.0,0.0,0.0,0.0 })
		{
			s = speed;
		};

	private:
		SpeedParam s;
	};

	class Get : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit Get(const std::string &name = "Get_plan");
		ARIS_REGISTER_TYPE(Get);
	};

    class Getp : public aris::plan::Plan
    {
    public:
        auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
        auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

        explicit Getp(const std::string &name = "Getp_plan");
        ARIS_REGISTER_TYPE(Getp);
    };

	class MoveT : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveT(const std::string &name = "MoveT_plan");
		ARIS_REGISTER_TYPE(MoveT);
	};

	class MoveE0 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveE0(const std::string &name = "MoveE0_plan");
		ARIS_REGISTER_TYPE(MoveE0);
	};

	class MoveE : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveE(const std::string &name = "MoveE_plan");
		ARIS_REGISTER_TYPE(MoveE);
	};

	class MoveAbJ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveAbJ(const std::string &name = "MoveAbJ_plan");
		ARIS_REGISTER_TYPE(MoveAbJ);
	};

	class MoveJM : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJM(const std::string &name = "MoveJM_plan");
		ARIS_REGISTER_TYPE(MoveJM);
	};

	class MoveC : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void override;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;

		virtual ~MoveC();
		explicit MoveC(const std::string &name = "MoveC_plan");
		ARIS_REGISTER_TYPE(MoveC);
		ARIS_DECLARE_BIG_FOUR(MoveC);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class JogC : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogC();
		explicit JogC(const std::string &name = "JogC_plan");
		ARIS_REGISTER_TYPE(JogC);
		JogC(const JogC &);
		JogC(JogC &);
		JogC& operator=(const JogC &);
		JogC& operator=(JogC &&);
		

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class JogJ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ();
		explicit JogJ(const std::string &name = "JogJ_plan");
		ARIS_REGISTER_TYPE(JogJ);
		JogJ(const JogJ &);
		JogJ(JogJ &);
		JogJ& operator=(const JogJ &);
		JogJ& operator=(JogJ &&);


	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class JogJ1 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ1();
		explicit JogJ1(const std::string &name = "JogJ1_plan");
		ARIS_REGISTER_TYPE(JogJ1);
	};

	class JogJ2 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ2();
		explicit JogJ2(const std::string &name = "JogJ2_plan");
		ARIS_REGISTER_TYPE(JogJ2);
	};

	class JogJ3 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ3();
		explicit JogJ3(const std::string &name = "JogJ3_plan");
		ARIS_REGISTER_TYPE(JogJ3);
	};

	class JogJ4 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ4();
		explicit JogJ4(const std::string &name = "JogJ4_plan");
		ARIS_REGISTER_TYPE(JogJ4);
	};

	class JogJ5 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ5();
		explicit JogJ5(const std::string &name = "JogJ5_plan");
		ARIS_REGISTER_TYPE(JogJ5);
	};

	class JogJ6 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ6();
		explicit JogJ6(const std::string &name = "JogJ6_plan");
		ARIS_REGISTER_TYPE(JogJ6);
	};

	class JogJ7 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ7();
		explicit JogJ7(const std::string &name = "JogJ7_plan");
		ARIS_REGISTER_TYPE(JogJ7);
	};

	class Xbox : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;
		explicit Xbox(const std::string &name = "Xbox_plan");
		ARIS_REGISTER_TYPE(Xbox);
	};

	class DMode : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;
		explicit DMode(const std::string &name = "DMode_plan");
		ARIS_REGISTER_TYPE(DMode);
	};

	class DEnable : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;
		explicit DEnable(const std::string &name = "DEnable_plan");
		ARIS_REGISTER_TYPE(DEnable);
	};

	class DDisable : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;
		explicit DDisable(const std::string &name = "DDisable_plan");
		ARIS_REGISTER_TYPE(DDisable);
	};

	class DJ1 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;
		explicit DJ1(const std::string &name = "DJ1_plan");
		ARIS_REGISTER_TYPE(DJ1);
	};

	class DJ2 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;
		explicit DJ2(const std::string &name = "DJ2_plan");
		ARIS_REGISTER_TYPE(DJ2);
	};

	class DJ3 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;
		explicit DJ3(const std::string &name = "DJ3_plan");
		ARIS_REGISTER_TYPE(DJ3);
	};

    class DHome : public aris::plan::Plan
    {
    public:
        auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
        auto virtual collectNrt(aris::plan::PlanTarget &target)->void;
        explicit DHome(const std::string &name = "DHome_plan");
        ARIS_REGISTER_TYPE(DHome);
    };

	class DMoveAbsJ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;
		explicit DMoveAbsJ(const std::string &name = "DMoveAbsJ_plan");
		ARIS_REGISTER_TYPE(DMoveAbsJ);
	};

	class JX : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JX();
		explicit JX(const std::string &name = "JX_plan");
		ARIS_REGISTER_TYPE(JX);
	};

	class JY : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JY();
		explicit JY(const std::string &name = "JY_plan");
		ARIS_REGISTER_TYPE(JY);
	};

	class JZ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JZ();
		explicit JZ(const std::string &name = "JZ_plan");
		ARIS_REGISTER_TYPE(JZ);
	};

	class JRX : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JRX();
		explicit JRX(const std::string &name = "JRX_plan");
		ARIS_REGISTER_TYPE(JRX);
	};

	class JRY : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JRY();
		explicit JRY(const std::string &name = "JRY_plan");
		ARIS_REGISTER_TYPE(JRY);
	};

	class JRZ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JRZ();
		explicit JRZ(const std::string &name = "JRZ_plan");
		ARIS_REGISTER_TYPE(JRZ);
	};

	class FSSignal : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit FSSignal(const std::string &name = "FSSignal_plan");
		ARIS_REGISTER_TYPE(FSSignal);
	};

	class SetCon : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetCon(const std::string &name = "SetCon_plan");
		ARIS_REGISTER_TYPE(SetCon);
	};

	class SetDH : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetDH(const std::string &name = "SetDH_plan");
		ARIS_REGISTER_TYPE(SetDH);
	};
	
	class SetPG : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetPG(const std::string &name = "SetPG_plan");
		ARIS_REGISTER_TYPE(SetPG);
	};

	class SetUI : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetUI(const std::string &name = "SetUI_plan");
		ARIS_REGISTER_TYPE(SetUI);
	};
	
	class SetDriver : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetDriver(const std::string &name = "SetDriver_plan");
		ARIS_REGISTER_TYPE(SetDriver);
	};

	class SaveConfig : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SaveConfig(const std::string &name = "SaveConfig_plan");
		ARIS_REGISTER_TYPE(SaveConfig);
	};

	class SaveHome : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SaveHome(const std::string &name = "SaveHome_plan");
		ARIS_REGISTER_TYPE(SaveHome);
	};

	class ToHome : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit ToHome(const std::string &name = "ToHome_plan");
		ARIS_REGISTER_TYPE(ToHome);
	};

	class SaveP : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SaveP(const std::string &name = "SaveP_plan");
        auto virtual collectNrt(aris::plan::PlanTarget &target)->void;
		ARIS_REGISTER_TYPE(SaveP);
	};

	class SetVel : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetVel(const std::string &name = "SetVel_plan");
		ARIS_REGISTER_TYPE(SetVel);
	};

	class UDVel : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit UDVel(const std::string &name = "UDVel_plan");
		ARIS_REGISTER_TYPE(UDVel);
	};

	class SetCT : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetCT(const std::string &name = "SetCT_plan");
		ARIS_REGISTER_TYPE(SetCT);
	};
}

#endif
