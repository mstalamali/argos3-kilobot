/**
 * @file <BestN_ALF.h>
 *
 * @brief This is the header file of ALF, the ARK (Augmented Reality for Kilobots) loop function.
 *
 * @cite Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 * @author Fabio Oddi <fabio.oddi@diag.uniroma1.it>
 */
#ifndef BESTN_ALF_H
#define BESTN_ALF_H

#include <argos3/plugins/robots/kilobot/simulator/ALF.h>
#include "hierarchicFloor.h"

class CBestN_ALF : public CALF
{

public:

    CBestN_ALF();

    virtual ~CBestN_ALF();

    virtual void Init(TConfigurationNode& t_tree);

    virtual void Reset();

    virtual void Destroy();

    /** Setup the initial state of the Kilobots in the space */
    void SetupInitialKilobotStates();

    /** Setup the initial state of the kilobot pc_kilobot_entity */
    void SetupInitialKilobotState(CKilobotEntity& c_kilobot_entity);

    /** Setup virtual environment */
    void SetupVirtualEnvironments(TConfigurationNode& t_tree);

    /** Get experiment variables */
    void GetExperimentVariables(TConfigurationNode& t_tree);

    /** Get the message to send to a Kilobot according to its position */
    void UpdateKilobotState(CKilobotEntity& c_kilobot_entity);

    /** Get the message to send to a Kilobot according to its position */
    void UpdateVirtualSensor(CKilobotEntity& c_kilobot_entity);

    /** Used to plot the Virtual environment on the floor */
    virtual CColor GetFloorColor(const CVector2& vec_position_on_plane);

    /** Used to communicate intial field data and construct the hierarchic map*/
    void SendStructInitInformationA(CKilobotEntity &c_kilobot_entity);
    void SendStructInitInformationB(CKilobotEntity &c_kilobot_entity);
    void SendInformationGPS_A(CKilobotEntity &c_kilobot_entity, const int Type);
    void SendInformationGPS_B(CKilobotEntity &c_kilobot_entity, const int Type);

    Real abs_distance(const CVector2 a,const CVector2 b);

private:

    /************************************/
    /*  Virtual Environment variables   */
    /************************************/
    /* virtual environment struct*/
    int depth,branches;
    float k;
    CVector2 TL,BR;
    ChierarchicFloor *vh_floor;

    typedef struct
    {
        int current_node,previous_node,commitment_node;
    } SRobotNodes;

    std::vector<SRobotNodes> m_vecKilobotNodes;
    std::vector<CVector2> m_vecKilobotPositions;
    std::vector<CVector2> m_vecKilobotOldPositions;
    std::vector<Real> m_vecLastTimeMessaged;
    std::vector<int> m_vecStart_experiment;
    std::vector<int> m_vecGpsData;
    bool start_experiment = false;
    Real m_fMinTimeBetweenTwoMsg;
    std::vector<bool> m_vecMessageToSend;
    /* Number of GPS cells */
    UInt16 m_unGpsCells;

    /* GPS cell length in meters */
    Real m_fCellLength;

    /************************************/
    /*       Experiment variables       */
    /************************************/

    /* output file for data acquizition */
    std::ofstream m_cOutput;

    /* output file name*/
    std::string m_strOutputFileName;

    /* data acquisition frequency in ticks */
    UInt16 m_unDataAcquisitionFrequency;
};

#endif
