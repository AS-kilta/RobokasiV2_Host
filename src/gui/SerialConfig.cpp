#include "gui/SerialConfig.hpp"

#include <imgui.h>


using namespace gui;


SerialConfig::SerialConfig(std::string name,
                           std::function<int(std::string port, int baud)> connectCb) :
#ifndef WITHOUT_LIBSERIALPORT
    _ports(nullptr),
#endif
    _port_idx(0),
    _port_baud(576000),
    _name(name),
    _connectCb(connectCb)
{
#ifndef WITHOUT_LIBSERIALPORT
    sp_list_ports(&_ports);
#endif
}

SerialConfig::~SerialConfig()
{
}

void SerialConfig::render()
{
    int err;

    ImGui::SetNextWindowPos(ImVec2(10,10), ImGuiSetCond_Once);
    ImGui::SetNextWindowSize(ImVec2(250,125), ImGuiSetCond_Once);
    ImGui::SetNextWindowCollapsed(true, ImGuiSetCond_Once);
    ImGui::Begin(("Serial Config: " + _name).c_str());

#ifndef WITHOUT_LIBSERIALPORT
    if (ImGui::Button("Refresh ports"))
        sp_list_ports(&_ports);

    if (ImGui::BeginCombo("Device", sp_get_port_name(_ports[_port_idx]))) {
        if (_ports) {
            for (int i = 0; _ports[i]; ++i) {
                if (ImGui::Selectable(sp_get_port_name(_ports[i]), i == _port_idx))
                    _port_idx = i;
                if (i == _port_idx)
                    ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::InputInt("Baud rate", &_port_baud);

    if (ImGui::Button("Connect")) {
        err = _connectCb(sp_get_port_name(_ports[_port_idx]), _port_baud);
        if (err)
            ImGui::OpenPopup("Connection failed");
    }

    if (ImGui::BeginPopupModal("Connection failed")) {
        ImGui::Text("Connection failed");
        if (ImGui::Button("Ok"))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
#endif

    ImGui::End();
}
