/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include <AP_OSD/AP_OSD_Backend.h>

class AP_OSD_INT : public AP_OSD_Backend
{
    using AP_OSD_Backend::AP_OSD_Backend;
public:

    static AP_OSD_Backend *probe(AP_OSD &osd);

    //draw given text to framebuffer
    void write(uint8_t x, uint8_t y, const char* text) override;

    //initilize display port and underlying hardware
    bool init() override;

    //flush framebuffer to screen
    void flush() override;

    //clear framebuffer
    void clear() override;

    // return a correction factor used to display angles correctly
    float get_aspect_ratio_correction() const override;

    bool is_compatible_with_backend_type(AP_OSD::osd_types type) const override {
        switch(type) {
        case AP_OSD::osd_types::OSD_MAX7456:
        case AP_OSD::osd_types::OSD_SITL:
        case AP_OSD::osd_types::OSD_NONE:
        case AP_OSD::osd_types::OSD_TXONLY:
        case AP_OSD::osd_types::OSD_MSP:
        case AP_OSD::osd_types::OSD_MSP_DISPLAYPORT:
            return true;
        case AP_OSD::osd_types::OSD_INT:
            return false;
        }
        return false;
    }

    AP_OSD::osd_types get_backend_type() const override {
        return AP_OSD::osd_types::OSD_INT;
    }

private:

    bool update_font();

};
