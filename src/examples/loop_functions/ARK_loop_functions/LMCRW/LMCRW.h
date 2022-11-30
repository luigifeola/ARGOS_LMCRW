#ifndef LMCRW_ALF_H
#define LMCRW_ALF_H
namespace argos {
class CSpace;
class CFloorEntity;
class CSimulator;
}

#include <math.h>
#include <time.h>
#include <bitset>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/ALF.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>


#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
// #include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

#include <array>


using namespace argos;

enum bias_command {
    LEFT = 1,
    RIGHT = 2,
    STOP = 3
};

enum Experiment_type {
    SIMPLE_EXPERIMENT = 1,
    OBSTACLE_AVOIDANCE_EXPERIMENT = 2,
    BOUNCING_ANGLE_EXPERIMENT = 3
};

class LMCRW : public CALF
{

public:

    LMCRW();

    virtual ~LMCRW();

    virtual void Init(TConfigurationNode& t_tree);

    virtual void Reset();

    virtual void Destroy();

    virtual void PostStep();

    virtual void PostExperiment();

    /** Setup the initial state of the Kilobots in the space */
    void SetupInitialKilobotStates();

    /** Setup the initial state of the kilobot pc_kilobot_entity */
    void SetupInitialKilobotState(CKilobotEntity& c_kilobot_entity);

    /** Experiment configuration methods (From .argos files) */
    /** Setup virtual environment */
    void SetupVirtualEnvironments(TConfigurationNode& t_tree);

    /** Get experiment variables */
    void GetExperimentVariables(TConfigurationNode& t_tree);

    /** Virtual environment visualization updating */

    /** Get the message to send to a Kilobot according to its position */
    void UpdateKilobotState(CKilobotEntity& c_kilobot_entity);

    /** Get the message to send to a Kilobot according to its position */
    void UpdateVirtualSensor(CKilobotEntity& c_kilobot_entity);


    /** Print Kilobot Position */
    void PrintVecPos(std::vector<CVector2> vecKilobotsPositions);

    /** Print Kilobot State */
    void PrintKilobotState(CKilobotEntity& c_kilobot_entity);

    /** Print Kilobot Command */
    void PrintKilobotCommand(int command);

    /** Used to plot the Virtual environment on the floor */
    virtual CColor GetFloorColor(const CVector2& vec_position_on_plane);

    CRadians GetBearingRobotPosition(CKilobotEntity& c_kilobot_entity);

private:

    UInt32 num_robots_with_discovery;
    UInt32 num_robots_with_info;
    
    
    /************************************/
    /*  Virtual Environment variables   */
    /************************************/
    /* virtual environment struct*/
    struct SVirtualArea
    {
        CVector2 Center;
        Real Radius;
        CColor Color;
    };

    SVirtualArea m_sClusteringHub;
    
    struct SVirtualPerimeter
    {
        // circular arena variables
        CVector2 Center;
        Real Radius; // radius for the circular arena
        Real Wall_width;  // wall width
        Real Wall_height; // wall height
        Real Wall_numbers;  // number of walls
    };

    SVirtualPerimeter m_ArenaStructure;

    typedef enum
    {
        NOT_TARGET_FOUND=0,
        TARGET_FOUND=1,
        TARGET_COMMUNICATED=2,
        BIASING=3,
        COLLIDING = 4,
    } SRobotState;


    /* used to store the state of each kilobot */
    std::vector<SRobotState> m_vecKilobotStates;
    std::vector<CVector2> m_vecKilobotsPositions;
    std::vector<CRadians> m_vecKilobotsOrientations;
    std::vector<CRadians> m_vecKilobotsBiasAngle; //bias angle command in radians

    // std::vector<Real> m_vecLastTimeMessaged;
    // Real m_fMinTimeBetweenTwoMsg;

    /* Flag to start the experiment */
    /** Flag to check if kilobot is arrived in its initial desired position */
    std::vector<bool>  v_recivedCoefficients;

    bool start_experiment;
    Real start_experiment_time;
    int internal_counter;

    /************************************/
    /*       Experiment variables       */
    /************************************/
  
    /* random number generator */
    CRandom::CRNG* c_rng;

    /* crwlevy exponents */
    Real crw_exponent;
    Real levy_exponent;

    /* simulator seed */
    uint m_random_seed;

    /* output file for data acquisition */
    std::ofstream m_timeStatsOutputs;
    /* output file name*/
    std::string m_timeStatsOutputsFileName;

    /* positions file for data acquisition */
    std::ofstream m_kiloOutput;
    /* stored position file name*/
    std::string m_kiloOutputFileName;

    /* data acquisition frequency in ticks */
    UInt16 m_unDataAcquisitionFrequency;

    UInt32 m_unFullDiscoveryTime;
    UInt32 m_unFullInformationTime;
    Real m_fFractionWithInformation;
    Real m_fFractionWithDiscovery;

};

#endif