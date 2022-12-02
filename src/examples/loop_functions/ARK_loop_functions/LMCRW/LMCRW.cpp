#include "LMCRW.h"

/****************************************/
/****************************************/
namespace
{
    const double kKiloDiameter = 0.033;
    const double kDistanceTargetFromTheOrigin = 0.5;
    const double kEpsilon = 0.0001;

    /* walla voidance parameters */
    double vArena_size = -1.0;
    double vDistance_threshold = -1.0;
    const CVector2 left_direction(1.0, 0.0);
    const int kProximity_bits = 8;

    //FIXME: maybe here in the namespace is not the better place
    CRadians random_angle;

}

LMCRW::LMCRW() :
m_unDataAcquisitionFrequency(10),   
internal_counter(0),
experiment_started(false),
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

    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);

    m_vecFirstPassageTime.resize(m_tKilobotEntities.size());
    std::fill(m_vecFirstPassageTime.begin(), m_vecFirstPassageTime.end(), -1.0);
    m_vecConvergenceTime.resize(m_tKilobotEntities.size());
    std::fill(m_vecConvergenceTime.begin(), m_vecConvergenceTime.end(), -1.0);
    
    v_receivedCoefficients.resize(m_tKilobotEntities.size());
    std::fill(v_receivedCoefficients.begin(), v_receivedCoefficients.end(), false);

    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        /* Setup the virtual states of a kilobot(e.g. has food state)*/
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }
}

/****************************************/
/****************************************/

void LMCRW::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    /* The kilobots begins outside the clustering hub*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    m_vecKilobotStates[unKilobotID] = NOT_TARGET_FOUND;
    m_vecLastTimeMessaged[unKilobotID] = -1000;

    /* Get a non-colliding random position within the circular arena */
    bool distant_enough = false;
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

    /* Read arena parameters */
    vArena_size = argos::CSimulator::GetInstance().GetSpace().GetArenaSize().GetX();
    vDistance_threshold = vArena_size / 2.0 - 2.0 * kKiloDiameter;
    std::cout << "Arena size: " << vArena_size << "\n";

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

    /* wall positioning */
    for (UInt32 i = 0; i < m_ArenaStructure.Wall_numbers; i++) {
        entity_id.str("");
        entity_id << "wall_" << i;

        CRadians wall_rotation = wall_angle * i;
        CVector3 wall_position(m_ArenaStructure.Radius * Cos(wall_rotation), m_ArenaStructure.Radius * Sin(wall_rotation), 0);
        CQuaternion wall_orientation;
        wall_orientation.FromEulerAngles(wall_rotation, CRadians::ZERO, CRadians::ZERO);

        CBoxEntity *wall = new CBoxEntity(entity_id.str(), wall_position, wall_orientation, false, wall_size);
        AddEntity(*wall);
    }


    /* Get the node defining the clustering hub parameters */
    TConfigurationNode& t_VirtualClusteringHubNode = GetNode(tVirtualEnvironmentsNode,"Area");
    GetNodeAttribute(t_VirtualClusteringHubNode, "position", m_sClusteringHub.Center);
    GetNodeAttribute(t_VirtualClusteringHubNode, "radius", m_sClusteringHub.Radius);
    GetNodeAttribute(t_VirtualClusteringHubNode, "color", m_sClusteringHub.Color);
    


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
        //Target positioned at kDistanceTargetFromTheOrigin cm from the origin
        c_random_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
        c_position.SetX(kDistanceTargetFromTheOrigin * sin(c_random_angle));
        c_position.SetY(kDistanceTargetFromTheOrigin * cos(c_random_angle));
        // std::cout<<"Distance from the origin : "<< pow(c_position.GetX(),2) + pow(c_position.GetY(),2) << std::endl;
    }
}

/****************************************/
/****************************************/

void LMCRW::GetExperimentVariables(TConfigurationNode& t_tree){
    /* Get the experiment variables node from the .argos file */
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");
    /* Get the crwlevy exponents */
    GetNodeAttribute(tExperimentVariablesNode, "crw", crw_exponent);
    GetNodeAttribute(tExperimentVariablesNode, "levy", levy_exponent);
    /* Get the crwlevy exponents */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "experiment", exp_type, exp_type);
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


    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);

    Real fDistance = Distance(m_vecKilobotsPositions[unKilobotID], m_sClusteringHub.Center);
    m_vecKilobotStates[unKilobotID] = fDistance<(m_sClusteringHub.Radius) ? TARGET_FOUND : m_vecKilobotStates[unKilobotID];

    switch (m_vecKilobotStates[unKilobotID])
    {
    case TARGET_FOUND:
        if(m_vecFirstPassageTime[unKilobotID] == -1)
        {
            m_vecFirstPassageTime[unKilobotID] = m_fTimeInSeconds;
            std::cout<< "kID: " << unKilobotID << " found target at time: " << m_fTimeInSeconds << std::endl;
        }
        if(m_vecConvergenceTime[unKilobotID] == -1)
        {
            m_vecConvergenceTime[unKilobotID] = m_fTimeInSeconds;
        }
        break;
    case TARGET_COMMUNICATED:
        if(m_vecConvergenceTime[unKilobotID] == -1)
        {
            m_vecConvergenceTime[unKilobotID] = m_fTimeInSeconds;
            std::cout<< "kID: " << unKilobotID << " received info about the target at time: " << m_fTimeInSeconds << std::endl;
        }
        
        break;
    
    default:
        break;
    }
    
}


/****************************************/
/****************************************/

CVector2 LMCRW::VectorRotation2D(Real angle, CVector2 vec)
{
    Real kx = (cos(angle) * vec.GetX()) + (-1.0 * sin(angle) * vec.GetY());
    Real ky = (sin(angle) * vec.GetX()) + (cos(angle) * vec.GetY());
    CVector2 rotated_vector(kx, ky);
    return rotated_vector;
}


/****************************************/
/****************************************/


std::vector<int> LMCRW::Proximity_sensor(CVector2 obstacle_direction, Real kOrientation, int num_sectors)
{
    double sector = M_PI_2 / (num_sectors / 2.0);
    std::vector<int> proximity_values;

    for (int i = 0; i < num_sectors; i++)
    {
        CVector2 sector_dir_start = VectorRotation2D((kOrientation + M_PI_2 - i * sector), left_direction);
        CVector2 sector_dir_end = VectorRotation2D((kOrientation + M_PI_2 - (i + 1) * sector), left_direction);

        if (obstacle_direction.DotProduct(sector_dir_start) >= 0.0 || obstacle_direction.DotProduct(sector_dir_end) >= 0.0)
        {
            proximity_values.push_back(0);
        }
        else
        {
            proximity_values.push_back(1);
        }
    }

    return proximity_values;
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

    tKilobotMessage = tEmptyMessage;
    m_tMessages[unKilobotID].type = 0;
    

    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID] < m_fMinTimeBetweenTwoMsg)
    {
        return;
    }

    /* if the experiment is not started yet, send alpha and rho parameters */
    if(!experiment_started)
    {
        // Messages of parameters (crw, levy)
        UInt8 crw = (UInt8)(crw_exponent*10);
        UInt8 levy = (UInt8)(levy_exponent*10);
        
        m_tMessages[unKilobotID].type = 255;

        tKilobotMessage.m_sID = unKilobotID;
        tKilobotMessage.m_sData = (crw << 5);
        tKilobotMessage.m_sData = tKilobotMessage.m_sData | levy;

        // std::cout<< "rho: " << crw << "alpha: " << levy << std::endl;
    }
    

    /* if the kilobot finds directly or because communicated by other robots*/
    else if(GetKilobotLedColor(c_kilobot_entity) != CColor::RED && m_vecKilobotStates[unKilobotID]==TARGET_FOUND) 
    {
        tKilobotMessage.m_sID = unKilobotID;
        tKilobotMessage.m_sType = STATE_MESSAGE;
        tKilobotMessage.m_sData = 0;
    }

    else if(GetKilobotLedColor(c_kilobot_entity) == CColor::GREEN)
    {
        m_vecKilobotStates[unKilobotID] = TARGET_COMMUNICATED;
    }


    /********* WALL AVOIDANCE PROCEDURE *************/
    UInt8 proximity_sensor_dec = 0; //8 bit proximity sensor as decimal
    Real fDistance = Distance(m_vecKilobotsPositions[unKilobotID], m_ArenaStructure.Center);
    // std::cerr<<"fDistance: "<<fDistance<<std::endl;
    // std::cerr<<"vDistance_threshold: "<<vDistance_threshold<<std::endl;

    if (fDistance > vDistance_threshold && exp_type!="simple_experiment")
    {
        /************************************************************************************************************************/
        /** Bouncing ANGLE*/
        if(exp_type == "bouncing"){
            std::vector<int> proximity_vec;
            CRadians collision_angle = ATan2(m_vecKilobotsPositions[unKilobotID].GetY(), m_vecKilobotsPositions[unKilobotID].GetX());
            CVector2 collision_direction = CVector2(vDistance_threshold * Cos(collision_angle + CRadians(M_PI)), vDistance_threshold * Sin(collision_angle + CRadians(M_PI))).Normalize();
            proximity_vec = Proximity_sensor(collision_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), kProximity_bits);
            proximity_sensor_dec = std::accumulate(proximity_vec.begin(), proximity_vec.end(), 0, [](int x, int y)
                                                { return (x << 1) + y; });
            /* To turn off the wall avoidance uncomment the following line */
            //proximity_sensor_dec = 0;

            /** Print proximity values */
            // std::cerr << "kID:" << unKilobotID << " sensor ";
            // for (int item : proximity_vec)
            // {
            //     std::cout << item << '\t';
            // }
            // std::cout << std::endl;

            // std::cout<<"******Prox dec: "<<proximity_sensor_dec<<std::endl;
            
            tKilobotMessage.m_sID = unKilobotID;
            tKilobotMessage.m_sType = PROXIMITY_MESSAGE;
            tKilobotMessage.m_sData = proximity_sensor_dec;
        }
        /************************************************************************************************************************/

        /************************************************************************************************************************/
        /** RANDOM ANGLE */
        else if (exp_type == "random_angle") {
            CRadians home_angle = ATan2(-m_vecKilobotsPositions[unKilobotID].GetY(), -m_vecKilobotsPositions[unKilobotID].GetX()) - m_vecKilobotsOrientations[unKilobotID];
            // std::cout << "random_angle: " << random_angle.RADIANS_TO_DEGREES << '\t';
            random_angle = CRadians(c_rng->Uniform(CRange<Real>(-CRadians::PI_OVER_TWO.GetValue(), CRadians::PI_OVER_TWO.GetValue())));
            random_angle += home_angle;
            random_angle.SignedNormalize(); //map angle in [-pi,pi]

            if(home_angle.SignedNormalize().GetAbsoluteValue() > M_PI_2)
            {
                tKilobotMessage.m_sID = unKilobotID;
                tKilobotMessage.m_sType = RANDOM_ANGLE_MESSAGE;

                if(random_angle.GetValue() >= 0)
                    tKilobotMessage.m_sData = (UInt8) round ((127.0 / M_PI) * random_angle.GetValue());
                else
                {
                    tKilobotMessage.m_sData = (UInt8) round ((127.0 / M_PI) * random_angle.GetAbsoluteValue());
                    tKilobotMessage.m_sData = tKilobotMessage.m_sData | 1 << 7;
                }
                std::bitset<8> b1 (tKilobotMessage.m_sData);
            }
        }
        /************************************************************************************************************************/

        else{
            std::cout<<"****WARNING!!***\n" << exp_type << " is a NOT EXISTING experiment!!!\n";
        }
    }

    /** 
     * 
     * READY TO SEND A MESSAGE
     * 
    **/

   m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
    
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

    /* experiments does not start until all kilobots have received coefficients */
    if(!experiment_started)
    {
        v_receivedCoefficients[unKilobotID]=true;
        if(std::all_of(v_receivedCoefficients.begin(), v_receivedCoefficients.end(), [](bool received){return received;}))
        {
            experiment_started = true;
            start_experiment_time = m_fTimeInSeconds;
        }
    }
}


/****************************************/
/****************************************/
void LMCRW::PostStep()
{
    // std::cout << "Time: " << m_fTimeInSeconds << std::endl;
    internal_counter += 1;
    if (internal_counter % m_unDataAcquisitionFrequency == 0 || internal_counter <= 1)
    {
        KiloLOG();
    }

}


/****************************************/
/****************************************/

void LMCRW::PostExperiment()
{
    std::cout << "END\n";
}


/****************************************/
/****************************************/


void LMCRW::KiloLOG()
{
    // std::cout << std::noshowpos << std::setw(4) << std::setprecision(0) << std::setfill('0')
    //           << m_fTimeInSeconds << '\t';
    m_kiloOutput
        << std::noshowpos << std::setw(4) << std::setprecision(0) << std::setfill('0')
        << m_fTimeInSeconds << '\t';
    for (size_t kID = 0; kID < m_vecKilobotsPositions.size(); kID++)
    {
        m_kiloOutput
            // << std::noshowpos
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << kID << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetX() << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetY() << '\t'
            << std::internal << std::showpos << std::setw(6) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsOrientations[kID].GetValue() << '\t'
            << std::noshowpos << std::setw(6) << std::setprecision(1) << std::setfill('0')
            << m_vecConvergenceTime[kID] << '\t'
            << std::noshowpos << std::setw(6) << std::setprecision(1) << std::setfill('0')
            << m_vecFirstPassageTime[kID] << '\t';
    }
    m_kiloOutput << std::endl;
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

    // /** origin to kID 0 line */
    // CVector2 kiloPos = m_vecKilobotsPositions[0];
    // if(vec_position_on_plane.GetY() - (kiloPos.GetY()/kiloPos.GetX()) * vec_position_on_plane.GetX() < 0.01 &&
    //     vec_position_on_plane.GetY() - (kiloPos.GetY()/kiloPos.GetX()) * vec_position_on_plane.GetX() > -0.01){
    //     cColor = CColor::BLUE;
    // }

    
    // /* y-axis */
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
