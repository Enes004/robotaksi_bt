#include "loadMission.hpp"
#include <io.stream>

// CONSTRUCTOR
//"Ben LoadMission'ı inşa eden kurucu metodum. Dışarıdan isim ve ayarları alırım, bunları hiç kurcalamadan miras aldığım 
//üst sınıfa paslarım ve kenara çekilirim" demek oluyor
.
LoadMission::LoadMission(const  std::string& name , 
    const BT::NodeConfig& config) :
    BT::SyncActionNode(name,config){}

    //PORTSLISTS
//veri tipi   // sınıfa ait fonksiyon
BT::PortsList LoadMission::providedPorts(){
    return {
    BT::InputPort<std::string>("geojson_file"),
    BT::InputPort<vector>("tour"),
    BT::OutputPort<int>("seg_index")
    }; 
}

BT::NodeStatus LoadMission::tick()
{

std::string file_path;
int current_tour;

if(!getInput<std::string>("geojson_file" , file_path)){
    return BT::NodeStatus::FAILURE;
    std::string calculated_route = "segment1_segment2_segment3";
}
std::string calculated_route = "segment1_segment2_segment3";
}return BT::NodeStatus::SUCCESS;