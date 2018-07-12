/**
 * @file <ALF.cpp>
 *
 * @author Mohamed Salaheddine Talamali <mstalamali1@sheffield.ac.uk>
 *
 * @brief This is the source file of the ARK Loop Function (ALF), the simulated conterpart of the ARK (Augmented Reality for Kilobots) system. Here, we reproduce
 * the demo C of the real ARK: https://www.youtube.com/watch?v=K0KvPzhOSDo .
 *
 * @cite Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 */

#include "ALF.h"


/****************************************/
/*     Default LoopFunction Methods     */
/****************************************/
CALF::CALF(): CLoopFunctions(),m_unDataAcquisitionFrequency(10),
    m_unEnvironmentPlotUpdateFrequency(10),m_fTimeForAMessage(0.05){}
CALF::~CALF(){
}
void CALF::Init(TConfigurationNode& t_node) {

    /* Create random number generator */
    m_pcRNG = CRandom::CreateRNG("argos");

    /* Set the tracking type from the .argos file*/
    SetTrackingType(t_node);

    /* Get experiment variables from the .argos file*/
    GetExperimentVariables(t_node);

    /* Get the virtual environment from the .argos file */
    SetupVirtualEnvironments(t_node);

    /* Get the initial kilobots' states */
    SetupInitialKilobotsStates();

    /* Other initializations: Varibales, Log file opening... */

    // Open a log file
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}
void CALF::Reset() {

    /* Close data file */
    m_cOutput.close();

    /* Reopen the file, erasing its contents */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);

}
void CALF::Destroy() {
    /* Close data file */
    m_cOutput.close();
}
void CALF::PreStep(){

    /* Update the time variable required for the experiment (in sec)*/
    m_fTimeInSeconds=GetSpace().GetSimulationClock()/CPhysicsEngine::GetInverseSimulationClockTick();

    /* Update the state of the kilobots in the space*/
    UpdateKilobotsState();

    /* Update the virtual sensor of the kilobots*/
    UpdateVirtualSensors();

    /* Update the virtual environment*/
    UpdateVirtualEnvironments();

    /* Update the virtual environment plot*/
    PlotEnvironment();

}
void CALF::PostStep(){

    /* Log experiment's results*/
    if(((UInt16)m_fTimeInSeconds%m_unDataAcquisitionFrequency==0)&&((m_fTimeInSeconds-(UInt16)m_fTimeInSeconds)==0)){

        m_cOutput << (UInt16) m_fTimeInSeconds << '\t';

        UInt16 unKilobotID;
        CVector2 cKilobotPosition;

        for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){

            unKilobotID=GetKilobotId(*m_tKilobotsEntities[it]);
            cKilobotPosition=GetKilobotPosition(*m_tKilobotsEntities[it]);

            m_cOutput << unKilobotID << '\t' << cKilobotPosition.GetX() << '\t' << cKilobotPosition.GetY() << '\t' << (UInt16)m_vecHasFood[unKilobotID] << '\t';

        }

        m_cOutput << std::endl;
    }
}
/****************************************/
/*      Kilobot Tracking Function       */
/****************************************/
CVector2 CALF::GetKilobotPosition(CKilobotEntity& c_kilobot_entity){
    CVector2 vecKilobotPosition(c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    return vecKilobotPosition;
}

CRadians CALF::GetKilobotOrientation(CKilobotEntity& c_kilobot_entity)
{

    CRadians cZAngle;
    CRadians cYAngle;
    CRadians cXAngle;

    //Calculate the orientations of the kilobot
    CQuaternion cRobotOrientations = c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation;

    cRobotOrientations.ToEulerAngles(cZAngle,cYAngle, cXAngle);

    return cZAngle;
}

UInt16 CALF::GetKilobotId(CKilobotEntity& c_kilobot_entity){
    std::string strKilobotID((c_kilobot_entity).GetControllableEntity().GetController().GetId());
    return std::stoul(strKilobotID.substr(2));
}

CColor CALF::GetKilobotLedColor(CKilobotEntity &c_kilobot_entity){
    return c_kilobot_entity.GetLEDEquippedEntity().GetLED(0).GetColor();
}

/****************************************/
/*       Initialization functions       */
/****************************************/
void CALF::GetKilobotsEntities(){
    /*
     * Go through all the robots in the environment
     * and create a vector of pointers on their entities
     */

    /* Get the map of all kilobots from the space */
    CSpace::TMapPerType& mapKilobots=GetSpace().GetEntitiesByType("kilobot");
    /* Go through them */
    for(CSpace::TMapPerType::iterator it = mapKilobots.begin();
        it != mapKilobots.end();
        ++it) {
        m_tKilobotsEntities.push_back(any_cast<CKilobotEntity*>(it->second));
    }

    /* Create Kilobots individual messages */
    m_tMessages=TKilobotsMessagesVector(m_tKilobotsEntities.size());
}

void CALF::SetupInitialKilobotsStates(){
    /* Get the Kilobots entities from the space.*/
    GetKilobotsEntities();

    /* Resize variables related to the number of Kilobots */
    m_sFoodEnv.LastSent.resize(m_tKilobotsEntities.size());
    m_sFoodEnv.SignalSwap.resize(m_tKilobotsEntities.size());

    m_sHomeEnv.LastSent.resize(m_tKilobotsEntities.size());
    m_sHomeEnv.SignalSwap.resize(m_tKilobotsEntities.size());

    m_vecHasFood.resize(m_tKilobotsEntities.size());
    m_vecKilobotsEnvironment.resize(m_tKilobotsEntities.size());

    /* Setup the min time between two message sent to a kilobot (ARK message sending limits)*/
    m_sFoodEnv.MinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotsEntities.size()*m_fTimeForAMessage/3.0);
    m_sHomeEnv.MinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotsEntities.size()*m_fTimeForAMessage/3.0);

    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Setup the virtual states of a kilobot(e.g. has food state)*/
        SetupInitialKilobotState(*m_tKilobotsEntities[it]);
    }
}

void CALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){

    /* Get the robot ID */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);

    m_sFoodEnv.LastSent[unKilobotID] = -1000; //very low value to trigger the first message
    m_sHomeEnv.LastSent[unKilobotID] = -1000; //very low value to trigger the first message

    m_sFoodEnv.SignalSwap[unKilobotID] = false;
    m_sHomeEnv.SignalSwap[unKilobotID] = false;

    m_vecHasFood[unKilobotID] = false; // initially robots carry no food
    m_vecKilobotsEnvironment[unKilobotID]=&m_sFoodEnv; // the robot starts searching for food

}

void CALF::SetTrackingType(TConfigurationNode& t_tree){

    TConfigurationNode& tTrackingNode=GetNode(t_tree,"tracking");

    GetNodeAttribute(tTrackingNode, "position", m_bPositionTracking);

    GetNodeAttribute(tTrackingNode, "orientation", m_bOrientationTracking);

    GetNodeAttribute(tTrackingNode, "color", m_bColorTracking);

}

void CALF::SetupVirtualEnvironments(TConfigurationNode& t_tree){

    /* Get the virtual environments node from .argos file*/
    TConfigurationNode& tVirtualEnvironmentsNode=GetNode(t_tree,"environments");

    TConfigurationNodeIterator itNodes;
    /* Iterate through the nodes under the the environments node to get the individual environments */
    for(itNodes=itNodes.begin(&tVirtualEnvironmentsNode); itNodes!=itNodes.end(); ++itNodes){
        if(itNodes->Value()=="Area"){

            UInt8 unAreaType;

            GetNodeAttribute(*itNodes, "type", unAreaType);

            if(unAreaType==FOOD){
                GetNodeAttribute(*itNodes, "position",m_sFoodEnv.GoalLocation);
                GetNodeAttribute(*itNodes, "radius",m_sFoodEnv.GoalSize);
                GetNodeAttribute(*itNodes, "color",m_sFoodEnv.Color);
            }

            if(unAreaType==HOME){
                GetNodeAttribute(*itNodes, "position",m_sHomeEnv.GoalLocation);
                GetNodeAttribute(*itNodes, "radius",m_sHomeEnv.GoalSize);
                GetNodeAttribute(*itNodes, "color",m_sHomeEnv.Color);
            }
        }
    }

}

void CALF::GetExperimentVariables(TConfigurationNode& t_tree){

    /* Get the experiment variables node from the .argos file */
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");

    /* Get Gradient field radius */
    GetNodeAttribute(tExperimentVariablesNode, "gradientfieldradius", m_fGradientFieldRadius);

    /* Get Gradient field color */
    GetNodeAttribute(tExperimentVariablesNode, "gradientfieldcolor", m_cGradientFieldColor);

    /* Get the output datafile name and open it */
    GetNodeAttribute(tExperimentVariablesNode, "datafilename", m_strOutputFileName);

    /* Get the frequency of data saving */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);

    /* Get the frequency of updating the environment plot */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);

    /* Get the time for one kilobot message */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);

}

/****************************************/
/*          Updating functions          */
/****************************************/
void CALF::UpdateKilobotsState(){
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Update the virtual states and actuators of the kilobot*/
        UpdateKilobotState(*m_tKilobotsEntities[it]);
    }
}

void CALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    CVector2 vecKilobotPosition=GetKilobotPosition(c_kilobot_entity);
    CColor cKilobotColor=GetKilobotLedColor(c_kilobot_entity);

    if (cKilobotColor == CColor::BLUE){ // the kilobot is asking for action
        Real fBuffer = 0.03;
        Real fDistHome = (Real)CRay2(m_sHomeEnv.GoalLocation, vecKilobotPosition).GetLength();
        bool bChanged = false;
        Real nDistFood = std::numeric_limits<Real>::max();
        if (fDistHome < m_sHomeEnv.GoalSize + fBuffer){
            if (m_vecHasFood[unKilobotID]){ // if robot has food
                m_vecKilobotsEnvironment[unKilobotID]=&m_sFoodEnv;
                m_sFoodEnv.SignalSwap[unKilobotID] = true;
                m_vecHasFood[unKilobotID]= false;
                m_sHomeEnv.GoalSize += 0.005; // increase home size
                bChanged = true;
            }
        } else {
            if (m_sFoodEnv.GoalSize > 0.00001){ // the robot can collect food only if there is food
                nDistFood = (Real)(CRay2(m_sFoodEnv.GoalLocation, vecKilobotPosition).GetLength());
                if (nDistFood < m_sFoodEnv.GoalSize + fBuffer){
                    if (!m_vecHasFood[unKilobotID]) { // if robot has food
                        m_vecKilobotsEnvironment[unKilobotID]=&m_sHomeEnv;
                        m_sHomeEnv.SignalSwap[unKilobotID] = true;
                        m_vecHasFood[unKilobotID] = true;
                        m_sFoodEnv.GoalSize = Max<Real>(0.0, m_sFoodEnv.GoalSize - 0.005); // reduce food size
                        if(m_sFoodEnv.GoalSize==0) m_fGradientFieldRadius=0;
                        bChanged = true;
                    }
                }
            }
        }
    }
}

void CALF::UpdateVirtualSensors(){

    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Update the virtual sensor of a kilobot based on its current state */
        UpdateVirtualSensor(*m_tKilobotsEntities[it]);
    }
}

void CALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){

    /*Create ARK-type messages variables*/
    m_tArkKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;

    /* Flag for existance of message to send*/
    bool bMessageToSend=false;

    /* Get the kilobot ID and state (Position and Orientation in this example*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    CVector2 cKilobotPosition=GetKilobotPosition(c_kilobot_entity);
    CRadians cKilobotOrientation=GetKilobotOrientation(c_kilobot_entity);

    /* check if enough time has passed from the last message otherwise*/
    if (m_fTimeInSeconds - m_vecKilobotsEnvironment[unKilobotID]->LastSent[unKilobotID] < m_vecKilobotsEnvironment[unKilobotID]->MinTimeBetweenTwoMsg){
        return; // if the time is too short, the kilobot cannot receive a message
    }

    /* Check if the robots swapped to another environment and inform him in case of a swap*/
    if (m_vecKilobotsEnvironment[unKilobotID]->SignalSwap[unKilobotID]){
        /*  Prepare the inividual kilobot's message */
        tKilobotMessage.m_sID = unKilobotID;
        tKilobotMessage.m_sType = 14; // 14 + env_type;
        tKilobotMessage.m_sData = m_vecKilobotsEnvironment[unKilobotID]->EnvType;

        m_vecKilobotsEnvironment[unKilobotID]->SignalSwap[unKilobotID] = false;
        m_vecKilobotsEnvironment[unKilobotID]->LastSent[unKilobotID] = m_fTimeInSeconds;

        /*  Set the message sending flag to True */
        bMessageToSend=true;
    }

    /* Check if there in no more food and inform the robot*/
    if ( !bMessageToSend && m_vecKilobotsEnvironment[unKilobotID]->GoalSize <= 0.0001 && m_vecKilobotsEnvironment[unKilobotID]->EnvType == FOOD){
        /*  Prepare the inividual kilobot's message */
        tKilobotMessage.m_sID = unKilobotID;
        tKilobotMessage.m_sType = 15; // 14 + env_type;
        tKilobotMessage.m_sData = 0;
        m_vecKilobotsEnvironment[unKilobotID]->LastSent[unKilobotID] = m_fTimeInSeconds;

        /*  Set the message sending flag to True */
        bMessageToSend=true;
    }


    if(!bMessageToSend){

        UInt8 unVStype = 0;
        UInt16 unVSval=0;

        /* Compute the distance of the robot to its Goal (either Food or Home) */
        Real fDistance = Distance(cKilobotPosition,m_vecKilobotsEnvironment[unKilobotID]->GoalLocation);

        /* If the kilobot is looking for food, it is guided only if at certain distance from the food source.
         * The kilobot is always guide when taking food to home */
        if (m_vecKilobotsEnvironment[unKilobotID]->EnvType == FOOD && (fDistance-m_vecKilobotsEnvironment[unKilobotID]->GoalSize) >= m_fGradientFieldRadius){
            return;
        }

        Real fDistNorm = fDistance;
        if (m_vecKilobotsEnvironment[unKilobotID]->GoalSize > 0.1){ fDistNorm += 0.05; }// Adding an extra 5cm to allow the bots to get inside the area
        if (m_vecKilobotsEnvironment[unKilobotID]->GoalSize < 0.011){ fDistNorm -= 0.02; } // Removing 2cm if the area is very tiny to allow the bots to catch nectar

        /* Check if the kilobot is inside his Goal*/
        fDistNorm = (fDistNorm - m_vecKilobotsEnvironment[unKilobotID]->GoalSize)/0.1;

        if (fDistNorm <= 0) { // the kilobot is inside the goal area
            unVStype = 0;
        } else { // the kilobot is outside the goal area
            unVStype = Min<Real>(m_vecKilobotsEnvironment[unKilobotID]->UpperBoundVS, (SInt16)floor(fDistNorm*1000 + 1) );
        }



        /* Get the angle to the Goal */
        CVector2 cVectorToGoal((m_vecKilobotsEnvironment[unKilobotID]->GoalLocation-cKilobotPosition).GetX(),(m_vecKilobotsEnvironment[unKilobotID]->GoalLocation-cKilobotPosition).GetY());

        CRadians cPathAngle = cVectorToGoal.Angle();
        CDegrees cDiffAngle = ToDegrees(cPathAngle)-ToDegrees(cKilobotOrientation);
        cDiffAngle.UnsignedNormalize();
        unVSval=(UInt16)cDiffAngle.GetValue();

        /*  Prepare the inividual kilobot's message */
        tKilobotMessage.m_sID = unKilobotID;
        tKilobotMessage.m_sType = unVStype; // 14 + env_type;
        tKilobotMessage.m_sData = unVSval;

        m_vecKilobotsEnvironment[unKilobotID]->LastSent[unKilobotID] = m_fTimeInSeconds;

        /*  Set the message sending flag to True */
        bMessageToSend=true;
    }

    /* Send the message to the kilobot using the ARK messaging protocol (addressing 3 kilobots per one standard kilobot message)*/
    if(bMessageToSend){

        for (int i = 0; i < 9; ++i) {
            m_tMessages[unKilobotID].data[i]=0;
        }
        // Prepare an empty ARK-type message to fill the gap in the full kilobot message
        tEmptyMessage.m_sID=1023;
        tEmptyMessage.m_sType=0;
        tEmptyMessage.m_sData=0;

        // Fill the kilobot message by the ARK-type messages

        for (int i = 0; i < 3; ++i) {

            if(i==0){
                tMessage=tKilobotMessage;
            } else{
                tMessage=tEmptyMessage;
            }

            m_tMessages[unKilobotID].data[i*3] = (tMessage.m_sID >> 2);
            m_tMessages[unKilobotID].data[1+i*3] = (tMessage.m_sID << 6);
            m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sType << 2);
            m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sData >> 8);
            m_tMessages[unKilobotID].data[2+i*3] = tMessage.m_sData;
        }

        /* Sending the message */
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
    }
    else{
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,NULL);
    }

}

void CALF::UpdateVirtualEnvironments(){

    /* Updates the virtual environments  based on the kilobots' states */
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Let a kilobot modify the virtual environment  */
        UpdatesVirtualEnvironmentsBasedOnKilobotState(*m_tKilobotsEntities[it]);
    }

    /* Update virtual envirenmonts based on other rules such as time or equation. E.g.:
        if(m_fTimeInSeconds%10==0){
            m_sFoodEnv.GoalSize-=0.05;
        }
    */


}

void CALF::UpdatesVirtualEnvironmentsBasedOnKilobotState(CKilobotEntity &c_kilobot_entity){
    /* Here the virtual environment are updated based on the kilobot "kilobot_entity" state */
}

/****************************************/
/*          Drawing functions           */
/****************************************/
CColor CALF::GetFloorColor(const CVector2 &vec_position_on_plane) {
    Real fPositionX(vec_position_on_plane.GetX()),fPositionY(vec_position_on_plane.GetY());


    CColor cColor=CColor::WHITE;

    if((pow(fPositionX-m_sHomeEnv.GoalLocation.GetX(),2)+pow(fPositionY-m_sHomeEnv.GoalLocation.GetY(),2))<pow(m_sHomeEnv.GoalSize,2)){
        cColor=m_sHomeEnv.Color;
        return cColor;
    };

    if((pow(fPositionX-m_sFoodEnv.GoalLocation.GetX(),2)+pow(fPositionY-m_sFoodEnv.GoalLocation.GetY(),2))<pow(m_sFoodEnv.GoalSize,2)){
        cColor=m_sFoodEnv.Color;
        return cColor;
    };

    if((pow(fPositionX-m_sFoodEnv.GoalLocation.GetX(),2)+pow(fPositionY-m_sFoodEnv.GoalLocation.GetY(),2))<pow(m_sFoodEnv.GoalSize+m_fGradientFieldRadius,2)){
        cColor=m_cGradientFieldColor;
        return cColor;
    };

    return cColor;
}
void CALF::PlotEnvironment(){
    /* Update the Floor visualization of the virtual environment every m_unEnvironmentPlotUpdateFrequency ticks*/
    if(GetSpace().GetSimulationClock()%m_unEnvironmentPlotUpdateFrequency==0)
        GetSpace().GetFloorEntity().SetChanged();
}

/****************************************/
/* Here Goes the user created functions */
/****************************************/


REGISTER_LOOP_FUNCTIONS(CALF, "ALF_loop_functions")
