#ifndef LGFX_ILI9341_H
#define LGFX_ILI9341_H

#include <M5GFX.h>
#include <lgfx/v1/panel/Panel_LCD.hpp>

// SPI pins for external ILI9341 display
#define PIN_SCK   40  // SCK  -> EXT PIN 7
#define PIN_MOSI  14  // MOSI -> EXT PIN 9
#define PIN_MISO  39  // MISO (not used for display, but for SD)
#define LCD_CS    5   // CS   -> EXT PIN 13
#define LCD_DC    6   // DC   -> EXT PIN 5
#define LCD_RST   3   // RST  -> EXT PIN 1

// SD card CS
#define SD_CS     12

struct Panel_ILI9341_Local : public lgfx::v1::Panel_LCD {
    Panel_ILI9341_Local(void) {
        _cfg.memory_width  = _cfg.panel_width  = 240;
        _cfg.memory_height = _cfg.panel_height = 320;
    }

protected:
    static constexpr uint8_t CMD_PWCTR1  = 0xC0;
    static constexpr uint8_t CMD_PWCTR2  = 0xC1;
    static constexpr uint8_t CMD_VMCTR1  = 0xC5;
    static constexpr uint8_t CMD_VMCTR2  = 0xC7;
    static constexpr uint8_t CMD_FRMCTR1 = 0xB1;
    static constexpr uint8_t CMD_DFUNCTR = 0xB6;
    static constexpr uint8_t CMD_GMCTRP1 = 0xE0;
    static constexpr uint8_t CMD_GMCTRN1 = 0xE1;
    static constexpr uint8_t CMD_PIXFMT  = 0x3A;

    const uint8_t* getInitCommands(uint8_t listno) const override {
        static constexpr uint8_t list0[] = {
            CMD_PWCTR1,  1, 0x23,
            CMD_PWCTR2,  1, 0x10,
            CMD_VMCTR1,  2, 0x3E, 0x28,
            CMD_VMCTR2,  1, 0x86,
            CMD_PIXFMT,  1, 0x55,       // 16-bit RGB565
            CMD_FRMCTR1, 2, 0x00, 0x18, // 79Hz frame rate
            CMD_DFUNCTR, 3, 0x08, 0x82, 0x27,
            CMD_GMCTRP1,15, 0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00,
            CMD_GMCTRN1,15, 0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F,
            CMD_SLPOUT , 0 + CMD_INIT_DELAY, 120,
            CMD_IDMOFF , 0,
            CMD_DISPON , 0 + CMD_INIT_DELAY, 100,
            0xFF, 0xFF,
        };
        switch (listno) {
        case 0: return list0;
        default: return nullptr;
        }
    }
};

class LGFX_ILI9341 : public lgfx::v1::LGFX_Device {
    Panel_ILI9341_Local panel;
    lgfx::v1::Bus_SPI bus;

public:
    LGFX_ILI9341() {
        auto b = bus.config();
        b.spi_host   = SPI3_HOST;
        b.spi_mode   = 0;
        b.freq_write = 40000000;   // 40 MHz
        b.freq_read  = 16000000;
        b.spi_3wire  = true;
        b.use_lock   = true;
        b.dma_channel = 1;

        b.pin_sclk = PIN_SCK;
        b.pin_mosi = PIN_MOSI;
        b.pin_miso = -1;
        b.pin_dc   = LCD_DC;
        
        bus.config(b);
        panel.setBus(&bus);

        auto p = panel.config();
        p.pin_cs    = LCD_CS;
        p.pin_rst   = LCD_RST;
        p.bus_shared = true;
        p.readable   = false;
        p.invert     = false;
        p.rgb_order  = false;
        p.dlen_16bit = false;
        p.memory_width  = 240;
        p.memory_height = 320;
        p.panel_width   = 240;
        p.panel_height  = 320;
        p.offset_x = 0;
        p.offset_y = 0;
        p.offset_rotation = 4;
        p.dummy_read_pixel = 8;
        p.dummy_read_bits = 1;
        panel.config(p);

        setPanel(&panel);
    }
};

extern LGFX_ILI9341 externalDisplay;

inline void lcd_quiesce() {
    externalDisplay.endWrite();
    externalDisplay.waitDisplay();
    digitalWrite(LCD_CS, HIGH);
}

#endif // LGFX_ILI9341_H
