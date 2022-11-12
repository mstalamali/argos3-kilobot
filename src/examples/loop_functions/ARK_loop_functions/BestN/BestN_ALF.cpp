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
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_vecStart_experiment.resize(m_tKilobotEntities.size());
    m_vecGpsData.resize(m_tKilobotEntities.size());
    m_vecKilobotNodes.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);
    /* Create the virtual hierarchic environment over the arena */
    v_floor = new hierarchicFloor(TL,BR,m_tKilobotEntities.size(),depth,branches,10,k,1,this->GetSpace().GetArenaLimits().GetMax()[0],this->GetSpace().GetArenaLimits().GetMax()[1]);
    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        /* Setup the virtual states of a kilobot */
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }
}

/****************************************/
/****************************************/

void CBestN_ALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    /* The kilobots begins in the root node with a random goal position inside it */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    m_vecKilobotNodes[unKilobotID]={0,0,0};
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
}

/****************************************/
/****************************************/

void CBestN_ALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){

    /* Get the kilobot ID */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    /* check if enough time has passed from the last message otherwise*/
    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID]< m_fMinTimeBetweenTwoMsg){
        return; // if the time is too short, the kilobot cannot receive a message
    }
    /* clear all the variables used for messaging*/
    for (int i = 0; i < 9; ++i) m_tMessages[unKilobotID].data[i]=0;
    if(!start_experiment)
    {
        /* Send init information for environment representation*/
        switch (m_vecStart_experiment[unKilobotID])
        {
        case 0:
            m_vecStart_experiment[unKilobotID]=1;
            SendStructInitInformationA(c_kilobot_entity);
            break;
        case 1:
            m_vecStart_experiment[unKilobotID]=2;
            SendStructInitInformationB(c_kilobot_entity);
            break;
        case 2:
            m_vecStart_experiment[unKilobotID]=3;
            SendInformationGPS_A(c_kilobot_entity,0);
            break;
        case 3:
            m_vecStart_experiment[unKilobotID]=4;
            SendInformationGPS_B(c_kilobot_entity,0);
            break;
        }
        start_experiment=true;
        for(long unsigned int i=0;i<m_vecStart_experiment.size();i++)
        {
            if(m_vecStart_experiment[i]!=4)
            {
                start_experiment=false;
                break;
            }
        }
    }
    else
    {
        switch (m_vecGpsData[unKilobotID])
        {
            case 0:
                m_vecGpsData[unKilobotID]=1;
                SendInformationGPS_A(c_kilobot_entity,1);
                break;
            case 1:
                m_vecGpsData[unKilobotID]=0;
                SendInformationGPS_B(c_kilobot_entity,1);
                break;
        }
    }    
}

/****************************************/
/****************************************/
/* Send #branches, #depth, param k, ID best leaf to let agents build their maps
   and set the noisy data sampling signals    
*/
void CBestN_ALF::SendStructInitInformationA(CKilobotEntity &c_kilobot_entity)
{
    /* Get the kilobot ID */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    /* Create ARK-type messages variables */
    m_tALFKilobotMessage tKilobotMessage;
    m_tMessages[unKilobotID].type=0;
    
    tKilobotMessage.m_sType=0;
    tKilobotMessage.m_sID = ((int)(k*100))<<2;
    tKilobotMessage.m_sID = tKilobotMessage.m_sID | (branches-1);
    tKilobotMessage.m_sData = (v_floor->get_best_leaf()->get_id()-1)<<2;
    tKilobotMessage.m_sData = tKilobotMessage.m_sData | (depth-1);
    // Fill the kilobot message by the ARK-type messages
    m_tMessages[unKilobotID].data[0] = tKilobotMessage.m_sID>>2;
    m_tMessages[unKilobotID].data[1] = (((UInt8)tKilobotMessage.m_sType)<<6) | tKilobotMessage.m_sData>>2;
    m_tMessages[unKilobotID].data[2] = (tKilobotMessage.m_sID & 0b0000000011)<<2 | (tKilobotMessage.m_sData& 0b0000000011);
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
}

/****************************************/
/****************************************/
/* Send data about arena corners to handle goal positioning */

void CBestN_ALF::SendStructInitInformationB(CKilobotEntity &c_kilobot_entity)
{
    /* Get the kilobot ID */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    /* Create ARK-type messages variables */
    m_tALFKilobotMessage tKilobotMessage;
    m_tMessages[unKilobotID].type=0;
    
    tKilobotMessage.m_sType=1;
    UInt8 valX=(int)((float)BR.GetX()*10);
    UInt8 valY=(int)((float)BR.GetY()*10);
    tKilobotMessage.m_sID = valX;
    tKilobotMessage.m_sData = valY;
    // Fill the kilobot message by the ARK-type messages
    m_tMessages[unKilobotID].data[0] = tKilobotMessage.m_sID;
    m_tMessages[unKilobotID].data[1] = (((UInt8)tKilobotMessage.m_sType)<<6) | tKilobotMessage.m_sData;
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
}

/****************************************/
/****************************************/
/* Send GPS position and orientation */

void CBestN_ALF::SendInformationGPS_A(CKilobotEntity &c_kilobot_entity, const int Type)
{
    /* Get the kilobot ID */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    CVector2 cKilobotPosition = GetKilobotPosition(c_kilobot_entity);
    /* Create ARK-type messages variables */
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    m_tMessages[unKilobotID].type=Type;
    tKilobotMessage.m_sType=2;
    UInt8 valX=(int)((cKilobotPosition.GetX()+*(v_floor->get_offset_x()))*100);
    UInt8 valY=(int)((cKilobotPosition.GetY()+*(v_floor->get_offset_y()))*100);
    tKilobotMessage.m_sID = unKilobotID<<3 | valX>>4;
    tKilobotMessage.m_sData = ((UInt16)valX)<<6;    
    tKilobotMessage.m_sData = tKilobotMessage.m_sData | (valY>>1);
    tKilobotMessage.m_sType = tKilobotMessage.m_sType | ((valY & 0b00000001)<<3);
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
        m_tMessages[GetKilobotId(c_kilobot_entity)].data[i*3] = tMessage.m_sID>>3;
        m_tMessages[GetKilobotId(c_kilobot_entity)].data[1+i*3] = ((UInt8)tKilobotMessage.m_sType)<<6 | ((UInt8)tMessage.m_sID & 0b00000111)<<3 | tMessage.m_sData>>7;
        m_tMessages[GetKilobotId(c_kilobot_entity)].data[2+i*3] = ((UInt8)(tMessage.m_sData>>6) & 0b00000001)<<7 | ((UInt8)tMessage.m_sData & 0b00111111)<<1 | tMessage.m_sType>>3;
    }
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
}

/****************************************/
/****************************************/
/* Send GPS position and orientation */

void CBestN_ALF::SendInformationGPS_B(CKilobotEntity &c_kilobot_entity, const int Type)
{
    /* Get the kilobot ID */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    CDegrees cKilobotOrientation = ToDegrees(GetKilobotOrientation(c_kilobot_entity)).UnsignedNormalize();
    /* Create ARK-type messages variables */
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    m_tMessages[unKilobotID].type=Type;
    tKilobotMessage.m_sType=3;
    int angle = ((cKilobotOrientation.GetValue())/30)*10;
    tKilobotMessage.m_sID = unKilobotID<<3;
    tKilobotMessage.m_sData = angle<<3;
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
        m_tMessages[GetKilobotId(c_kilobot_entity)].data[i*3] = tMessage.m_sID>>3;
        m_tMessages[GetKilobotId(c_kilobot_entity)].data[1+i*3] = ((UInt8)tKilobotMessage.m_sType)<<6;
        m_tMessages[GetKilobotId(c_kilobot_entity)].data[2+i*3] = (UInt8)(tMessage.m_sData>>3);
    }
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
}

/****************************************/
/****************************************/

CColor CBestN_ALF::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;
    
    return cColor;
}

REGISTER_LOOP_FUNCTIONS(CBestN_ALF, "ALF_BestN_loop_function")
