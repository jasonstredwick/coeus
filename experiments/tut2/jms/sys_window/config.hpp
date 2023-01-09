#pragma once


#include <string>


#if defined(_WIN32) || defined(_WIN64)

#if UNICODE
using SysWindowConfigOptions_title_t = std::wstring;
#else
using SysWindowConfigOptions_title_t = std::string;
#endif

#else

using SysWindowConfigOptions_title_t = std::string;

#endif


struct SysWindowConfigOptions {
    using title_t = SysWindowConfigOptions_title_t;

    title_t title{};
    int x{-1};
    int y{-1};
    int width{-1};
    int height{-1};
};
