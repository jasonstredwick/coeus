#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#include <dxgi1_6.h>
//#include <d3d12.h>
#undef min
#undef max
#endif


void DXGIEnumAdapters() {//HWND hwnd) {
    using IDXGIFactory_t = IDXGIFactory5;
    using IDXGIAdapter_t = IDXGIAdapter4;
    using IDXGIOutput_t = IDXGIOutput6;

    using OwnedFactory_t = std::unique_ptr<IDXGIFactory_t, decltype([](auto* v){ if(v) v->Release(); })>;
    using OwnedAdapter_t = std::unique_ptr<IDXGIAdapter_t, decltype([](auto* v){ if(v) v->Release(); })>;
    using OwnedOutput_t  = std::unique_ptr<IDXGIOutput_t,  decltype([](auto* v){ if(v) v->Release(); })>;

    IDXGIFactory_t* factory_raw = nullptr;
    HRESULT result = CreateDXGIFactory2(0, __uuidof(IDXGIFactory_t), reinterpret_cast<void**>(&factory_raw));
    if (result) { std::wcout << std::format(L"Failed to create DXGIFactory2: {}\n", result); return; }
    OwnedFactory_t factory{factory_raw};

    unsigned int index = 0;
    IDXGIAdapter1* adapter_raw = nullptr;
    std::vector<OwnedAdapter_t> adapters{};
    for (unsigned int index=0; factory->EnumAdapters1(index, &adapter_raw) != DXGI_ERROR_NOT_FOUND; ++index) {
        IDXGIAdapter_t* adapter = nullptr;
        result = adapter_raw->QueryInterface(__uuidof(IDXGIAdapter_t), reinterpret_cast<void**>(&adapter));
        if (result) { std::wcout << std::format(L"Failed to cast IDXGIAdapter Interface: {}\n", result); return; }
        adapters.push_back(OwnedAdapter_t{adapter});
    }

    std::vector<std::vector<OwnedOutput_t>> adapter_outputs{};
    for (auto& adapter : adapters) {
        std::vector<OwnedOutput_t> outputs{};
        IDXGIOutput* output_raw = nullptr;
        for (unsigned int index=0; adapter->EnumOutputs(index, &output_raw) != DXGI_ERROR_NOT_FOUND; ++index) {
            IDXGIOutput_t* output = nullptr;
            result = output_raw->QueryInterface(__uuidof(IDXGIOutput_t), reinterpret_cast<void**>(&output));
            if (result) { std::wcout << std::format(L"Failed to cast IDXGIOutput Interface: {}\n", result); return; }
            outputs.push_back(OwnedOutput_t{output});
        }
        adapter_outputs.push_back(std::move(outputs));
    }

    for (size_t index=0; index<adapters.size(); ++index) {
        auto& adapter = adapters[index];
        auto& outputs = adapter_outputs[index];

        DXGI_ADAPTER_DESC1 desc{};
        adapter->GetDesc1(&desc);
        std::wcout << std::format(L"Adapter {}\n", desc.Description);
        std::wcout << L"Outputs-\n";
        for (auto& output : outputs) {
            DXGI_OUTPUT_DESC desc{};
            output->GetDesc(&desc);
            std::wcout << desc.DeviceName << std::endl;
            std::wcout << std::endl;
        }
        std::wcout << std::endl;
    }



    unsigned int display_mode = 0; // <--mono // DXGI_ENUM_MODES_STEREO, DXGI_ENUM_MODES_DISABLED_STEREO
    // Perhaps could use DXGI_OUTPUT_DESC1 as baseline for format using bits per pixel and other info.
    DXGI_FORMAT format = DXGI_FORMAT_R8G8B8A8_UNORM; // default


    size_t chosen_adapter_index = 0;
    size_t chosen_output_index = 0;
    IDXGIAdapter_t* chosen_adapter = adapters[chosen_adapter_index].get();
    IDXGIOutput_t* chosen_output = adapter_outputs[chosen_adapter_index][chosen_output_index].get();

    DXGI_OUTPUT_DESC1 chosen_output_desc{};
    result = chosen_output->GetDesc1(&chosen_output_desc);
    if (result) { std::wcout << std::format(L"Failed to get output desc1: {}\n", GetLastError()); return; }


    unsigned int num_modes = 0;
    result = chosen_output->GetDisplayModeList1(format, display_mode, &num_modes, nullptr);
    if (result) { std::wcout << std::format(L"Failed to get num modes for output: {}\n", GetLastError()); return; }
    std::wcout << L"Num modes found: " << num_modes << std::endl;
    std::vector<DXGI_MODE_DESC1> mode_desc(num_modes);
    result = chosen_output->GetDisplayModeList1(format, display_mode, &num_modes, mode_desc.data());
    if (result) { std::wcout << std::format(L"Failed to get output mode descriptions: {}\n", GetLastError()); return; }
    std::wcout << L"Num modes found: " << num_modes << std::endl;
    unsigned int max_width = 0;
    unsigned int max_height = 0;
    unsigned int count = 0;
    for (auto& mdesc : mode_desc) {
        if (mdesc.Width > max_width) {
            max_width = mdesc.Width;
            max_height = mdesc.Height;
            count = 1;
            std::wcout << std::format(L"Max: {}, {} - {} - {} {}\n", mdesc.Width, mdesc.Height, (int)mdesc.Format, mdesc.RefreshRate.Numerator, mdesc.RefreshRate.Denominator);
        } else if (mdesc.Width == max_width) {
            if (mdesc.Height > max_height) { max_height = mdesc.Height; }
            ++count;
            std::wcout << std::format(L"Max: {}, {} - {} - {} {}\n", mdesc.Width, mdesc.Height, (int)mdesc.Format, mdesc.RefreshRate.Numerator, mdesc.RefreshRate.Denominator);
        }
    }
    std::wcout << std::format(L"Max width ({}) ... {} times\n", max_width, count);

    // Why am I getting a weird width/height?
    DXGI_MODE_DESC1 mode_baseline{max_width, max_height, {0, 0}, format, DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED,
                                  DXGI_MODE_SCALING_UNSPECIFIED, 0};
    DXGI_MODE_DESC1 mode_found{};
    result = chosen_output->FindClosestMatchingMode1(&mode_baseline, &mode_found, nullptr);
    if (result) { std::wcout << std::format(L"Failed to find closest mode match for chosen output: {}\n", GetLastError()); return; };
    std::wcout << std::format(L"Found Mode-\nDims: {}, {}\nRRate: {}, {}\nFormat: {}\nOrdering: {}\nScaling: {}\nStero: {}\n",
    mode_found.Width, mode_found.Height, mode_found.RefreshRate.Numerator, mode_found.RefreshRate.Denominator,
    (int)mode_found.Format, (int)mode_found.ScanlineOrdering, (int)mode_found.Scaling, mode_found.Stereo);

    DXGI_QUERY_VIDEO_MEMORY_INFO info{};
    chosen_adapter->QueryVideoMemoryInfo(0, DXGI_MEMORY_SEGMENT_GROUP_LOCAL, &info);
    std::wcout << std::format(L"Video memory:\n{} {}\n{} {}\n", info.Budget, info.CurrentUsage, info.AvailableForReservation, info.CurrentReservation);
    /*
    if (D3D12CreateDevice(chosen_adapter, D3D_FEATURE_LEVEL_11_0, __uuidof(ID3D12Device), nullptr)) {
        std::wcout << std::format(L"Adapter doesn't support DirectX 12: {}\n", GetLastError());
        return;
    }
    */

    unsigned int swap_chain_flags = DXGI_SWAP_CHAIN_FLAG_NONPREROTATED |
                                    DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH |
                                    //DXGI_SWAP_CHAIN_FLAG_FRAME_LATENCY_WAITABLE_OBJECT |
                                    DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    DXGI_SWAP_CHAIN_DESC1 swap_chain_desc{
        .Width=mode_found.Width,
        .Height=mode_found.Height,
        .Format=mode_found.Format,
        .Stereo=mode_found.Stereo,
        .SampleDesc={1, 0},
        .BufferUsage=DXGI_USAGE_BACK_BUFFER, // Not sure what to pick, this seemed correct?
        .BufferCount=2, // front and back buffer; i.e. double buffer; consider triple buffer?
        .Scaling=DXGI_SCALING_NONE,
        .SwapEffect=DXGI_SWAP_EFFECT_FLIP_DISCARD, // DXGI_SWAP_EFFECT_FLIP_SEQUENTIAL
        .AlphaMode=DXGI_ALPHA_MODE_IGNORE,
        .Flags=swap_chain_flags
    };
    DXGI_SWAP_CHAIN_FULLSCREEN_DESC swap_chain_fullscreen_desc{
        .RefreshRate=mode_found.RefreshRate,
        .ScanlineOrdering=mode_found.ScanlineOrdering,
        .Scaling=mode_found.Scaling,
        .Windowed=0
    };
    /*
    IDXGISwapChain1* swap_chain = nullptr;
    result = factory->CreateSwapChainForHwnd(&device, hwnd, swap_chain_desc, swap_chain_fullscreen_desc,
                                             chosen_output, &swap_chain);
    if (result) {
        std::wcout << std::format(L"Failed to create swap chain: {}\n", GetLastError());
    } else { swap_chain->Release(); }
    */
}
