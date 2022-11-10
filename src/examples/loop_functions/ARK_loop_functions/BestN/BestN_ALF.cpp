/**
 * This is the source file of ALF, the ARK (Augmented Reality for Kilobots) loop function.
 *
 * Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 * @author Fabio Oddi <fabio.oddi@diag.uniroma1.it>
 */

#include "BestN_ALF.h"

/****************************************/
/****************************************/

CBestN_ALF::CBestN_ALF() :
    m_unDataAcquisitionFrequency(10){
}

/****************************************/
/****************************************/

CBestN_ALF::~CBestN_ALF(){
}

/****************************************/
/****************************************/

void CBestN_ALF::Init(TConfigurationNode& t_node) {
    /* Initialize ALF*/
    CALF::Init(t_node);
    /* Other initializations: Varibales, Log file opening... */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void CBestN_ALF::Reset() {
    /* Close data file */
    m_cOutput.close();
    /* Reopen the file, erasing its contents */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void CBestN_ALF::Destroy() {
    /* Close data file */
    m_cOutput.close();
}

/****************************************/
/****************************************/

void CBestN_ALF::SetupInitialKilobotStates() {
    m_vecKilobotGoalInfo.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_vecKilobotNodes.resize(m_tKilobotEntities.size());
    m_vecKilobotQuorum.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);
    /* Create the virtual hierarchic environment over the arena */
    v_floor = new hierarchicFloor(TL,BR,m_tKilobotEntities.size(),depth,branches,10,k,1,this->GetSpace().GetArenaLimits().GetMax()[0],this->GetSpace().GetArenaLimits().GetMax()[1]);
    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        /* Setup the virtual states of a kilobot */
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
        SendInitKilobotGPS(*m_tKilobotEntities[it]);
        SendInit4DataSensing(*m_tKilobotEntities[it]);
    }
}

/****************************************/
/****************************************/

void CBestN_ALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    /* The kilobots begins in the root node with a random goal position inside it */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    m_vecKilobotGoalInfo[unKilobotID]=POSITION_NOT_REACHED;
    m_vecKilobotQuorum[unKilobotID]=QUORUM_NOT_REACHED;
    m_vecKilobotNodes[unKilobotID]={0,0,0};
    /* Send init information for environment representation*/
    SendStructInitInformation(c_kilobot_entity);
}

/****************************************/
/****************************************/

void CBestN_ALF::SetupVirtualEnvironments(TConfigurationNode& t_tree){
    /* Get the structure variables from the .argos file*/
    TConfigurationNode& tHierarchicalStructNode=GetNode(t_tree,"hierarchicStruct");
    /* Get dimensions and quality scaling factor*/
    GetNodeAttribute(tHierarchicalStructNode,"depth",depth);
    GetNodeAttribute(tHierarchicalStructNode,"branches",branches);
    GetNodeAttribute(tHierarchicalStructNode,"k",k);
    /* Get the coordinates for the top left and bottom right corners of the arena */
    TL = CVector2(float(this->GetSpace().GetArenaLimits().GetMin()[0]+this->GetSpace().GetArenaLimits().GetMax()[0]),float(this->GetSpace().GetArenaLimits().GetMin()[1]+this->GetSpace().GetArenaLimits().GetMax()[1]));
    BR = CVector2(float(this->GetSpace().GetArenaLimits().GetMax()[0]+this->GetSpace().GetArenaLimits().GetMax()[0]),float(this->GetSpace().GetArenaLimits().GetMax()[1]+this->GetSpace().GetArenaLimits().GetMax()[1]));
}

/****************************************/
/****************************************/

void CBestN_ALF::GetExperimentVariables(TConfigurationNode& t_tree){
    /* Get the experiment variables node from the .argos file */
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");
    /* Get the output datafile name and open it */
    GetNodeAttribute(tExperimentVariablesNode, "datafilename", m_strOutputFileName);
    /* Get the frequency of data saving */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
    /* Get the frequency of updating the environment plot */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);
    /* Get the time for one kilobot message */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);

    /* Get the time for one kilobot message */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "gps_cells", m_unGpsCells, m_unGpsCells);
    m_fCellLength=GetSpace().GetArenaSize().GetX()/m_unGpsCells;
}

/****************************************/
/****************************************/

void CBestN_ALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    CVector2 cKilobotPosition = GetKilobotPosition(c_kilobot_entity);
    if(GetKilobotLedColor(c_kilobot_entity)==CColor::RED)
    {
        m_vecKilobotGoalInfo[unKilobotID]=POSITION_REACHED;
    }
    // std::cout<<"kiloID:"<<unKilobotID<<", kiloLEAF:"<<cKilobotLeaf->get_id()<<", kernalVAL:"<<cKilobotLeaf->get_kernel_value(cKilobotPosition,v_floor->get_kernel_unit())<<"\n";
}

/****************************************/
/****************************************/

void CBestN_ALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){

    /*Create ARK-type messages variables*/
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    /* Flag for existance of message to send*/
    bool bMessageToSend=false;
    /* Get the kilobot ID and state (Only Position in this example) */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);

    /* check if enough time has passed from the last message otherwise*/
    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID]< m_fMinTimeBetweenTwoMsg){
        return; // if the time is too short, the kilobot cannot receive a message
    }
    else if(m_vecKilobotGoalInfo[unKilobotID]==POSITION_REACHED)
        {
            m_vecKilobotGoalInfo[unKilobotID]==POSITION_NOT_REACHED;
            /*  Prepare the inividual kilobot's message */
            tKilobotMessage.m_sID = 111;
            /* communicate leaf id and utility */
            tKilobotMessage.m_sType = 111;
            tKilobotMessage.m_sData = 111;
            /*  Set the message sending flag to True */
            bMessageToSend=true;
            m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
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
            if( i == 0){
                tMessage = tKilobotMessage;
            } else{
                tMessage = tEmptyMessage;
            }
            m_tMessages[unKilobotID].data[i*3] = tMessage.m_sID >> 2;
            m_tMessages[unKilobotID].data[1+i*3] = tMessage.m_sID << 6;
            m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | ((UInt8)(tMessage.m_sType) << 2);
            m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | tMessage.m_sData >> 8;
            m_tMessages[unKilobotID].data[2+i*3] = tMessage.m_sData;
        }
        /* Sending the message */
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
    }
    else{
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,NULL);
    }
}

/****************************************/
/****************************************/
void CBestN_ALF::SendInit4DataSensing(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    /* Create ARK-type messages variables */
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;

}

/****************************************/
/****************************************/
/* Send initial bots positions*/
void CBestN_ALF::SendInitKilobotGPS(CKilobotEntity &c_kilobot_entity)
{
    /* Get the kilobot ID */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    /* Create ARK-type messages variables */
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    /* Fill the message */
    tKilobotMessage.m_sID = unKilobotID;
    tKilobotMessage.m_sType = ( (UInt8) PositionToGPS(GetKilobotPosition(c_kilobot_entity)).GetX() ) & 0x0F;

    CDegrees KB_Orientation = ToDegrees( GetKilobotOrientation(c_kilobot_entity) );
    KB_Orientation.UnsignedNormalize();

    tKilobotMessage.m_sData = ( (UInt8) PositionToGPS(GetKilobotPosition(c_kilobot_entity)).GetY() ) << 6 | ( (UInt8) (KB_Orientation.GetValue()/12.0) ) ;
    for (int i = 0; i < 9; ++i)
    {
        m_tMessages[unKilobotID].data[i]=0;
    }
    // Prepare an empty ARK-type message to fill the gap in the full kilobot message
    tEmptyMessage.m_sID=1023;
    tEmptyMessage.m_sType=0;
    tEmptyMessage.m_sData=0;
    // Fill the kilobot message by the ARK-type messages
    for (int i = 0; i < 3; ++i) {
        if( i == 0){
            tMessage = tKilobotMessage;
        } else{
            tMessage = tEmptyMessage;
        }
        m_tMessages[GetKilobotId(c_kilobot_entity)].data[i*3] = tMessage.m_sID;
        m_tMessages[GetKilobotId(c_kilobot_entity)].data[1+i*3] = (tMessage.m_sType << 2);
        m_tMessages[GetKilobotId(c_kilobot_entity)].data[1+i*3] = m_tMessages[GetKilobotId(c_kilobot_entity)].data[1+i*3] | (tMessage.m_sData >> 8);
        m_tMessages[GetKilobotId(c_kilobot_entity)].data[2+i*3] = tMessage.m_sData;
    }
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
}

/****************************************/
/****************************************/
/* Send dimensions for hierarchic structure and initial goal position*/
void CBestN_ALF::SendStructInitInformation(CKilobotEntity &c_kilobot_entity)
{
    /* Get the kilobot ID */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    /* Create ARK-type messages variables */
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    UInt16 valX=(int)((float)BR.GetX()*10);
    UInt16 valY=(int)((float)BR.GetY()*10);
    tKilobotMessage.m_sType = 0;
    tKilobotMessage.m_sID = valX<<2;
    tKilobotMessage.m_sData = valY<<3;
    tKilobotMessage.m_sID = tKilobotMessage.m_sID | branches;
    tKilobotMessage.m_sData = tKilobotMessage.m_sData | depth;
    for (int i = 0; i < 9; ++i)
    {
        m_tMessages[unKilobotID].data[i]=0;
    }
    // Prepare an empty ARK-type message to fill the gap in the full kilobot message
    tEmptyMessage.m_sID=1023;
    tEmptyMessage.m_sType=0;
    tEmptyMessage.m_sData=0;
    // Fill the kilobot message by the ARK-type messages
    for (int i = 0; i < 3; ++i) {
        if( i == 0){
            tMessage = tKilobotMessage;
        } else{
            tMessage = tEmptyMessage;
        }
        m_tMessages[unKilobotID].data[i*3] = tMessage.m_sID;
        m_tMessages[unKilobotID].data[1+i*3] = tMessage.m_sData;
        m_tMessages[unKilobotID].data[2+i*3] = tMessage.m_sType;
    }
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
}

/****************************************/
/****************************************/

CColor CBestN_ALF::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;
    
    return cColor;
}

/****************************************/
/****************************************/

CVector2 CBestN_ALF::PositionToGPS(CVector2 t_position) {
    return CVector2(Ceil(t_position.GetX()/m_fCellLength)-1,Ceil(t_position.GetY()/m_fCellLength)-1);
}

REGISTER_LOOP_FUNCTIONS(CBestN_ALF, "ALF_BestN_loop_function")
