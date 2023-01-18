#pragma once


#include <array>
#include <cstdint>


struct SysWindowInterface{
    virtual ~SysWindowInterface() = default;

    virtual void Create(const uint32_t width_pixel, const uint32_t height_pixel, const int32_t pos_x_pixel=0, const int32_t pos_y_pixel=0) = 0;

    virtual std::array<uint32_t, 2> DPI_XY() const noexcept = 0;
    virtual std::array<uint32_t, 2> Dims_WH() const noexcept = 0;
    virtual std::array<int32_t, 2>  Pos_XY() const noexcept = 0;
};
