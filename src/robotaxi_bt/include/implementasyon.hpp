#pragma once 
// Budosyayı projede sadece bir kez dahil et
#include <behaviourtree_cpp/action_node.h>
//BnodT den Action nodelarını içeren ana dosyayı projeye dahil et
#include <string>

class LoadMission : public BT::SyncActionNode
{
public:
LoadMission(const std::string& name, const BT::NodeConfig& config);

static BT::Portlist providedPorts();

BT::NodeStatus tick() override;
}