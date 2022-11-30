#include "LMCRW.h"

/****************************************/
/****************************************/
namespace{
const double kKiloDiameter = 0.033;
const double kDistanceTargetFromTheOrigin = 0.5;
const double kEpsilon = 0.0001;
}

LMCRW::LMCRW() :
m_unDataAcquisitionFrequency(10),   //each 10 seconds store kilobots positione
num_robots_with_discovery(0),
num_robots_with_info(0),
internal_counter(0),
start_experiment(false),
start_experiment_time(0),
m_random_seed(0)
    {
        c_rng = CRandom::CreateRNG("argos");
    }

/****************************************/
/****************************************/

LMCRW::~LMCRW(){
}

/****************************************/
/****************************************/

void LMCRW::Init(TConfigurationNode& t_node) {
    /* Initialize ALF*/
    CALF::Init(t_node);
    /* Other initializations: Varibales, Log file opening... */
    m_timeStatsOutputs.open(m_timeStatsOutputsFileName, std::ios_base::trunc | std::ios_base::out);
    m_kiloOutput.open(m_kiloOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void LMCRW::Reset() {
    /* Close data file */
    m_timeStatsOutputs.close();
    m_kiloOutput.close();
    /* Reopen the file, erasing its contents */
    m_timeStatsOutputs.open(m_timeStatsOutputsFileName, std::ios_base::trunc | std::ios_base::out);
    m_kiloOutput.open(m_kiloOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void LMCRW::Destroy() {
    /* Close data file */
    m_timeStatsOutputs.close();
    m_kiloOutput.close();
}

/****************************************/
/****************************************/

void LMCRW::SetupInitialKilobotStates() {
    m_vecKilobotStates.resize(m_tKilobotEntities.size());
    m_vecKilobotsPositions.resize(m_tKilobotEntities.size());
    m_vecKilobotsOrientations.resize(m_tKilobotEntities.size());
    m_vecKilobotsBiasAngle.resize(m_tKilobotEntities.size());
    
    v_recivedCoefficients.resize(m_tKilobotEntities.size());
    std::fill(v_recivedCoefficients.begin(), v_recivedCoefficients.end(), false);

    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        /* Setup the virtual states of a kilobot(e.g. has food state)*/
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }

    //The experiment must start with no kilobot on top of the target
    //In the following there is the collision check between target and each kilobot
    Real c_random_angle;
    CVector2 & c_position = m_sClusteringHub.Center;
    Real & c_radius = m_sClusteringHub.Radius;
    std::vector<CVector2>::iterator kilobot_on_the_top;
    
    if (m_ArenaStructure.Wall_numbers != 0)
    {
        do{
            c_random_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
            c_position.SetX(c_rng->Uniform(CRange<Real>(0, m_ArenaStructure.Radius - m_ArenaStructure.Wall_width - c_radius - kKiloDiameter)) * sin(c_random_angle));
            c_position.SetY(c_rng->Uniform(CRange<Real>(0, m_ArenaStructure.Radius - m_ArenaStructure.Wall_width - c_radius - kKiloDiameter)) * cos(c_random_angle));
            
            //check if there is some kilobot on top of the target
            kilobot_on_the_top = 
            std::find_if(m_vecKilobotsPositions.begin(), m_vecKilobotsPositions.end(), [&c_position, &c_radius](CVector2 const &position) {
                return Distance(c_position, position) < (c_radius + kEpsilon) ;
                });
            
        }while(kilobot_on_the_top != m_vecKilobotsPositions.end());
    }

    else
    {
        //Target positioned at 50 cm from the origin
        c_random_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
        c_position.SetX(kDistanceTargetFromTheOrigin * sin(c_random_angle));
        c_position.SetY(kDistanceTargetFromTheOrigin * cos(c_random_angle));
        // std::cout<<"Distance from the origin : "<< pow(c_position.GetX(),2) + pow(c_position.GetY(),2) << std::endl;
    }
    
     
    

    
}

/****************************************/
/****************************************/

void LMCRW::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    /* The kilobots begins outside the clustering hub*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    m_vecKilobotStates[unKilobotID] = NOT_TARGET_FOUND;

    /* Get a non-colliding random position within the circular arena */
    bool distant_enough = false;
    Real rand_angle, random_dist;
    Real rand_x, rand_y;
    CVector2 rand_displacement, rand_init_pos;
    CVector3 rand_pos;


    UInt16 maxTries = 999;
    UInt16 tries = 0;

    /* Get a random position and orientation for the kilobot initialized into a square but positioned in the circular arena */
    CQuaternion random_rotation;
    CRadians rand_rot_angle(c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue())));
    random_rotation.FromEulerAngles(rand_rot_angle, CRadians::ZERO, CRadians::ZERO);
    Real radius =  m_ArenaStructure.Radius - m_ArenaStructure.Wall_width/2 - kKiloDiameter/2 - kEpsilon;

    do {
        rand_x = c_rng->Uniform(CRange<Real>(-radius,radius));
        rand_y = c_rng->Uniform(CRange<Real>(-radius,radius));
        distant_enough = MoveEntity(c_kilobot_entity.GetEmbodiedEntity(), CVector3(rand_x, rand_y, 0), random_rotation, false);
        
        if(tries == maxTries-1) {
            std::cerr << "ERROR: too many tries and not an available spot for the area" << std::endl;
        }
    } while(!distant_enough || (rand_x*rand_x)+(rand_y*rand_y) > radius*radius );

    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
    

}

/****************************************/
/****************************************/

void LMCRW::SetupVirtualEnvironments(TConfigurationNode& t_tree){
    /*Simulator variables*/
    CSimulator &simulator = GetSimulator();
    m_random_seed = simulator.GetRandomSeed();

    /* Get the virtual environments node from .argos file*/
    TConfigurationNode& tVirtualEnvironmentsNode = GetNode(t_tree,"environments");
    /* Get the node defining the walls parametres*/
    TConfigurationNode& t_VirtualWallsNode = GetNode(tVirtualEnvironmentsNode,"Perimeter");
    GetNodeAttribute(t_VirtualWallsNode, "radius", m_ArenaStructure.Radius);
    GetNodeAttribute(t_VirtualWallsNode, "width", m_ArenaStructure.Wall_width);
    GetNodeAttribute(t_VirtualWallsNode, "height", m_ArenaStructure.Wall_height);
    GetNodeAttribute(t_VirtualWallsNode, "walls", m_ArenaStructure.Wall_numbers);

    

    std::ostringstream entity_id;
    CRadians wall_angle = CRadians::TWO_PI / m_ArenaStructure.Wall_numbers;
    CVector3 wall_size(m_ArenaStructure.Wall_width, 2.0 * m_ArenaStructure.Radius * Tan(CRadians::PI / m_ArenaStructure.Wall_numbers), m_ArenaStructure.Wall_height);

    // wall positioning
    for (UInt32 i = 0; i < m_ArenaStructure.Wall_numbers; i++) {
        entity_id.str("");
        entity_id << "wall_" << i;

        CRadians wall_rotation = wall_angle * i;
        // CVector3 wall_position((m_ArenaStructure.Radius) * Cos(wall_rotation), (m_ArenaStructure.Radius) * Sin(wall_rotation), 0);
        CVector3 wall_position(m_ArenaStructure.Radius * Cos(wall_rotation), m_ArenaStructure.Radius * Sin(wall_rotation), 0);
        CQuaternion wall_orientation;
        wall_orientation.FromEulerAngles(wall_rotation, CRadians::ZERO, CRadians::ZERO);

        CBoxEntity *wall = new CBoxEntity(entity_id.str(), wall_position, wall_orientation, false, wall_size);
        AddEntity(*wall);
    }


    /* Get the node defining the clustering hub parametres*/
    TConfigurationNode& t_VirtualClusteringHubNode = GetNode(tVirtualEnvironmentsNode,"Area");
    GetNodeAttribute(t_VirtualClusteringHubNode, "position", m_sClusteringHub.Center);
    GetNodeAttribute(t_VirtualClusteringHubNode, "radius", m_sClusteringHub.Radius);
    GetNodeAttribute(t_VirtualClusteringHubNode, "color", m_sClusteringHub.Color);

    /* Show origin position */
    // SVirtualArea temp_area2;
    // temp_area2.Center = CVector2(0,0);
    // temp_area2.Radius = 0.0165;
    // temp_area2.Color = CColor::MAGENTA;
    // m_TargetAreas.push_back(temp_area2);
    
    /* Show some position */
    // SVirtualArea temp_area3;
    // temp_area3.Center = CVector2(0,0.45);
    // temp_area3.Radius = 0.003;
    // temp_area3.Color = CColor::BLACK;
    // m_TargetAreas.push_back(temp_area3);
}

/****************************************/
/****************************************/

void LMCRW::GetExperimentVariables(TConfigurationNode& t_tree){
    /* Get the experiment variables node from the .argos file */
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");
    /* Get the crwlevy exponents */
    GetNodeAttribute(tExperimentVariablesNode, "crw", crw_exponent);
    GetNodeAttribute(tExperimentVariablesNode, "levy", levy_exponent);
    /* Get the positions datafile name to store Kilobot positions in time */
    GetNodeAttribute(tExperimentVariablesNode, "positionsfilename", m_kiloOutputFileName);
    /* Get the output datafile name and open it */
    GetNodeAttribute(tExperimentVariablesNode, "datafilename", m_timeStatsOutputsFileName);
    /* Get the frequency of data saving */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
    /* Get the frequency of updating the environment plot */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);
    /* Get the time for one kilobot message */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);
}


/****************************************/
/****************************************/
void LMCRW::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
    /* Update the state of the kilobots (target not found, found directly or communicated)*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    
}


/****************************************/
/****************************************/
    
void LMCRW::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
    /*Create ARK-type messages variables*/
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    /* Get the kilobot ID and state (Only Position in this example) */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    
    // Prepare an empty ARK-type message to fill the gap in the full kilobot message
    tEmptyMessage.m_sID=511;
    tEmptyMessage.m_sType=0;
    tEmptyMessage.m_sData=0;


    
    /** 
     * 
     * READY TO SEND A MESSAGE
     * 
    **/
    
    // Fill the kilobot message by the ARK-type messages
    for (int i = 0; i < 3; ++i) {
        if( i == 0){
            tMessage = tKilobotMessage;
        } else{
            tMessage = tEmptyMessage;
        }
        m_tMessages[unKilobotID].data[i*3] = (tMessage.m_sID >> 2);
        m_tMessages[unKilobotID].data[1+i*3] = (tMessage.m_sID << 6);
        m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sType << 2);
        m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sData >> 8);
        m_tMessages[unKilobotID].data[2+i*3] = tMessage.m_sData;
    }
    
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
}

/****************************************/
/****************************************/

CRadians LMCRW::GetBearingRobotPosition(CKilobotEntity& c_kilobot_entity)
{
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);    
    CRadians kiloOrientation = GetKilobotOrientation(c_kilobot_entity);
    CRadians robot_bearing = ATan2(GetKilobotPosition(c_kilobot_entity).GetY(), 
                                    GetKilobotPosition(c_kilobot_entity).GetX());

    
    // normalise the pathOrientation between -pi and pi
    robot_bearing.SignedNormalize(); //map angle in [-pi,pi]
    return robot_bearing;
}


/****************************************/
/****************************************/
void LMCRW::PostStep()
{

}


/****************************************/
/****************************************/

void LMCRW::PostExperiment()
{
    

    
}

/****************************************/
/****************************************/

void LMCRW::PrintKilobotState(CKilobotEntity& c_kilobot_entity)
{
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);

    std::cerr<<"Actual state:";
    switch (m_vecKilobotStates[unKilobotID])
    {
        case NOT_TARGET_FOUND:
            std::cerr<<"NOT_TARGET_FOUND"<<"\n";
            break;
        case TARGET_FOUND:
            std::cerr<<"TARGET_FOUND"<<"\n";
            break;
        case TARGET_COMMUNICATED:
            std::cerr<<"TARGET_COMMUNICATED"<<"\n";
            break;
        case BIASING:
            std::cerr<<"BIASING"<<"\n";
            break;
        case COLLIDING:
            std::cerr<<"COLLIDING"<<"\n";
            break;
        
        default:
            break;
    } 
}

/****************************************/
/****************************************/

void LMCRW::PrintKilobotCommand(int command)
{
    switch (command)
    {
        case LEFT:
            std::cout<<"Kilobot command  : LEFT"<<"\n";
            break;
        case RIGHT:
            std::cout<<"Kilobot command  : RIGHT"<<"\n";
            break;
        case STOP:
            std::cout<<"Kilobot command  : STOP"<<"\n";
            break;
        
        default:
            break;
    } 
}


/****************************************/
/****************************************/

void LMCRW::PrintVecPos(std::vector<CVector2> vecKilobotsPositions)
{
    std::cout<<"PrintVecPos:\n";
    for(const auto& pos : m_vecKilobotsPositions)
    {
        std::cout<<'\t'<<std::fixed<<std::setprecision(3)<<pos;
    }     
    std::cout<<"\n"; 
}


/****************************************/
/****************************************/

CColor LMCRW::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;

    if (SquareDistance(vec_position_on_plane, m_sClusteringHub.Center) < pow(m_sClusteringHub.Radius, 2))
    {
        cColor = m_sClusteringHub.Color;
    }
    
    // // y-axis
    // if(vec_position_on_plane.GetX() < 0.01 && vec_position_on_plane.GetX()> -0.01 ){
    //     if(vec_position_on_plane.GetY() >= 0)
    //         cColor = CColor::BLUE;
    //     else 
    //         cColor = CColor::GRAY70;
    // }

    // // x-axis
    // if(vec_position_on_plane.GetY() < 0.01 && vec_position_on_plane.GetY()> -0.01){
    //     if(vec_position_on_plane.GetX() >= 0)
    //         cColor = CColor::ORANGE;
    //     else
    //         cColor = CColor::GRAY70;
    // }


    return cColor;
}

REGISTER_LOOP_FUNCTIONS(LMCRW, "ALF_LMCRW_loop_function")
