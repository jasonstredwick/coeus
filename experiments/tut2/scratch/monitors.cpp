#if 0
#include <windows.h>
#include <vector>
#include <iostream>
#include "jms/vulkan/state.hpp"


struct cMonitorsVec {
    std::vector<int> iMonitors;
    std::vector<HMONITOR> hMonitors;
    std::vector<HDC> hdcMonitors;
    std::vector<RECT> rcMonitors;
    static BOOL CALLBACK MonitorEnum(HMONITOR hMon, HDC hdc, LPRECT lprcMonitor, LPARAM pData) {
        cMonitorsVec* pThis = reinterpret_cast<cMonitorsVec*>(pData);
        pThis->hMonitors.push_back(hMon);
        pThis->hdcMonitors.push_back(hdc);
        pThis->rcMonitors.push_back(*lprcMonitor);
        pThis->iMonitors.push_back(pThis->hdcMonitors.size());
        return TRUE;
    }
    cMonitorsVec() {
        EnumDisplayMonitors(0, 0, MonitorEnum, (LPARAM)this);
    }
};


#include <format>
int main() {
        if (!SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2)) {
            auto error = GetLastError();
            if (error != ERROR_ACCESS_DENIED) { // ERROR_ACCESS_DENIED == already set; ignore error
                throw std::runtime_error(fmt::format("Failed to set dpi awareness: {}\n", error));
            }
        }
    cMonitorsVec monitors{};
    for (int i=0; i<monitors.iMonitors.size(); ++i) {
        std::wcout << "Screen id: " << i << std::endl;
        std::wcout << "---------------------------------------------------" << std::endl;
        std::wcout << " - screen left top corner coords: (" << monitors.rcMonitors[i].left << ", " << monitors.rcMonitors[i].top << ")" << std::endl;
        std::wcout << " - screen bottom right corner coords: (" << monitors.rcMonitors[i].right << ", " << monitors.rcMonitors[i].bottom << ")" << std::endl;
        std::wcout << " - screen dims: (" << (monitors.rcMonitors[i].right - monitors.rcMonitors[i].left) << ", " <<
                                             (monitors.rcMonitors[i].bottom - monitors.rcMonitors[i].top) << ")" << std::endl;
        std::wcout << "---------------------------------------------------" << std::endl;
    }
    jms::vulkan::State vulkan_state{};
    std::vector<vk::ExtensionProperties> extension_props = vulkan_state.context.enumerateInstanceExtensionProperties();
    for (auto& i : extension_props) {
        std::cout << i.extensionName << std::endl;
    }
    return 0;
}
#endif
