//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

/**
 * Mode4App is a new application developed to be used with Mode 4 based simulation
 * Author: Brian McCarthy
 * Email: b.mccarthy@cs.ucc.ie
 */

#include "apps/mode4App/Mode4App.h"
#include "common/LteControlInfo.h"
#include "stack/phy/packet/cbr_m.h"

#include <fstream>
#include <string>

#include "inet/mobility/contract/IMobility.h"

Define_Module(Mode4App);

// ======================================================
// GLOBAL CSV LOG FILES
// ======================================================

static std::ofstream txLog_;
static std::ofstream rxLog_;
static bool logsInitialized_ = false;

// ======================================================

void Mode4App::initialize(int stage)
{
    Mode4BaseApp::initialize(stage);

    if (stage==inet::INITSTAGE_LOCAL){

        // Get the binder
        binder_ = getBinder();

        // Get our UE
        cModule *ue = getParentModule();

        // Register with the binder
        nodeId_ = binder_->registerNode(ue, UE, 0);

        // Register the nodeId_ with the binder.
        binder_->setMacNodeId(nodeId_, nodeId_);

    } else if (stage==inet::INITSTAGE_APPLICATION_LAYER) {

        selfSender_ = NULL;
        nextSno_ = 0;

        selfSender_ = new cMessage("selfSender");

        size_ = par("packetSize");
        period_ = par("period");
        priority_ = par("priority");
        duration_ = par("duration");

        sentMsg_ = registerSignal("sentMsg");
        delay_ = registerSignal("delay");
        rcvdMsg_ = registerSignal("rcvdMsg");
        cbr_ = registerSignal("cbr");

        // ======================================================
        // Initialize CSV logs once
        // ======================================================

        if (!logsInitialized_) {

            std::string configName = getEnvir()->getConfigEx()->getActiveConfigName();
            int runNumber = getEnvir()->getConfigEx()->getActiveRunNumber();

            std::string runId = configName + "-" + std::to_string(runNumber);

            std::string txFile = "results/" + configName + "/" + runId + "_tx.csv";
            std::string rxFile = "results/" + configName + "/" + runId + "_rx.csv";

            txLog_.open(txFile.c_str(), std::ios::out);
            rxLog_.open(rxFile.c_str(), std::ios::out);

            txLog_
                << "time,txId,sno,txX,txY\n";

            rxLog_
                << "time,rxId,txId,sno,delay,rxX,rxY\n";

            logsInitialized_ = true;
        }

        // ======================================================

        double delay = 0.001 * intuniform(0, 1000, 0);

        scheduleAt((simTime() + delay).trunc(SIMTIME_MS), selfSender_);
    }
}

void Mode4App::handleLowerMessage(cMessage* msg)
{
    if (msg->isName("CBR")) {

        Cbr* cbrPkt = check_and_cast<Cbr*>(msg);

        double channel_load = cbrPkt->getCbr();

        emit(cbr_, channel_load);

        delete cbrPkt;

    } else {

        AlertPacket* pkt = check_and_cast<AlertPacket*>(msg);

        if (pkt == 0)
            throw cRuntimeError(
                "Mode4App::handleMessage - Error when casting to AlertPacket");

        // ======================================================
        // Statistics
        // ======================================================

        simtime_t delay = simTime() - pkt->getTimestamp();

        emit(delay_, delay);
        emit(rcvdMsg_, (long)1);

        // ======================================================
        // RX POSITION
        // ======================================================

        auto mobility =
            check_and_cast<inet::IMobility*>(
                getParentModule()->getSubmodule("mobility"));

        inet::Coord rxPos = mobility->getCurrentPosition();

        // ======================================================
        // TX ID
        // ======================================================

        int txId = -1;

        auto ctrl = pkt->getControlInfo();

        if (ctrl != nullptr) {

            FlowControlInfoNonIp* lteInfo =
                dynamic_cast<FlowControlInfoNonIp*>(ctrl);

            if (lteInfo != nullptr)
                txId = lteInfo->getSrcAddr();
        }

        // ======================================================
        // RX CSV LOGGING
        // ======================================================

        // Only log center region
        if (rxPos.x >= 1500 && rxPos.x <= 3500) {

            rxLog_
                << simTime().dbl() << ","
                << nodeId_ << ","
                << txId << ","
                << pkt->getSno() << ","
                << delay.dbl() << ","
                << rxPos.x << ","
                << rxPos.y << "\n";
        }

        // ======================================================

        EV << "Packet received: SeqNo["
           << pkt->getSno()
           << "] Delay["
           << delay
           << "]"
           << endl;

        delete msg;
    }
}

void Mode4App::handleSelfMessage(cMessage* msg)
{
    if (!strcmp(msg->getName(), "selfSender")){

        AlertPacket* packet = new AlertPacket("Alert");

        packet->setTimestamp(simTime());
        packet->setByteLength(size_);
        packet->setSno(nextSno_);

        nextSno_++;

        auto lteControlInfo = new FlowControlInfoNonIp();

        lteControlInfo->setSrcAddr(nodeId_);
        lteControlInfo->setDirection(D2D_MULTI);
        lteControlInfo->setPriority(priority_);
        lteControlInfo->setDuration(duration_);
        lteControlInfo->setCreationTime(simTime());

        packet->setControlInfo(lteControlInfo);

        // ======================================================
        // TX POSITION
        // ======================================================

        auto mobility =
            check_and_cast<inet::IMobility*>(
                getParentModule()->getSubmodule("mobility"));

        inet::Coord txPos = mobility->getCurrentPosition();

        // ======================================================
        // TX CSV LOGGING
        // ======================================================

        // Only log center region
        if (txPos.x >= 1500 && txPos.x <= 3500) {

            txLog_
                << simTime().dbl() << ","
                << nodeId_ << ","
                << packet->getSno() << ","
                << txPos.x << ","
                << txPos.y << "\n";
        }

        // ======================================================

        Mode4BaseApp::sendLowerPackets(packet);

        emit(sentMsg_, (long)1);

        scheduleAt(simTime() + period_, selfSender_);
    }
    else
        throw cRuntimeError(
            "Mode4App::handleMessage - Unrecognized self message");
}

void Mode4App::finish()
{
    cancelAndDelete(selfSender_);
}

Mode4App::~Mode4App()
{
    binder_->unregisterNode(nodeId_);

    if (txLog_.is_open())
        txLog_.close();

    if (rxLog_.is_open())
        rxLog_.close();
}
