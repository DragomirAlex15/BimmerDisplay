"""
Genereaza ../lcd_fonts.c: fonturi LVGL (v8, 1 bit/pixel) cu aspect de afisaj LCD de bord.

Sursa: fontul clasic 5x7 "glcdfont" (Adafruit, licenta BSD) inclus in LovyanGFX.
Fiecare pixel e marit de `scale` ori (aspect pixelat), apoi litera e ingrosata cu
`bold` pixeli spre dreapta.

Rulare (din acest folder):  python gen_lcd_fonts.py
"""
import os
import re
import sys

GLCD = os.path.expanduser(r"~\Documents\Arduino\libraries\LovyanGFX\src\lgfx\Fonts\glcdfont.h")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lcd_fonts.c")

# nume, scale, bold (px)
FONTS = [
    ("lcd_font_16", 2, 1),   # titluri, status
    ("lcd_font_24", 3, 1),   # valori (3 coloane), coduri de eroare
    ("lcd_font_32", 4, 2),   # valori (2 coloane)
    ("lcd_font_40", 5, 2),   # valori (1 coloana)
]
FIRST, LAST = 0x20, 0x7E


def load_glcd(path):
    src = open(path, encoding="utf-8", errors="ignore").read()
    body = src[src.index("font[]"):]
    body = body[body.index("{") + 1: body.index("};")]
    data = [int(x, 16) for x in re.findall(r"0x[0-9A-Fa-f]{2}", body)]
    if len(data) < (LAST + 1) * 5:
        sys.exit("glcdfont.h are un format neasteptat")
    return data


def glyph_bits(data, ch, scale, bold):
    w, h = 5 * scale + bold, 8 * scale
    cols = data[ch * 5: ch * 5 + 5]

    def src(x, y):
        return 0 <= x < 5 and (cols[x] >> y) & 1

    bits = []
    for py in range(h):
        for px in range(w):
            on = any(0 <= px - k < 5 * scale and src((px - k) // scale, py // scale) for k in range(bold + 1))
            bits.append(1 if on else 0)
    return w, h, bits


def pack(bits):
    out = []
    for i in range(0, len(bits), 8):
        chunk = bits[i:i + 8] + [0] * (8 - len(bits[i:i + 8]))
        v = 0
        for b in chunk:
            v = (v << 1) | b
        out.append(v)
    return out


def gen_font(data, name, scale, bold):
    p = name.replace("lcd_font_", "lcd")
    bitmap, dsc = [], ["    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0}"]
    for ch in range(FIRST, LAST + 1):
        w, h, bits = glyph_bits(data, ch, scale, bold)
        dsc.append(f"    {{.bitmap_index = {len(bitmap)}, .adv_w = {6 * scale * 16}, .box_w = {w}, .box_h = {h}, "
                   f".ofs_x = 0, .ofs_y = {-scale}}} /* '{chr(ch) if ch not in (0x5C,) else 'backslash'}' */")
        bitmap += pack(bits)

    rows = [", ".join(f"0x{b:02x}" for b in bitmap[i:i + 16]) for i in range(0, len(bitmap), 16)]
    return f"""
/* ---------------- {name}: 5x7 x{scale}, bold {bold}px, inaltime {8 * scale}px ---------------- */
static LV_ATTRIBUTE_LARGE_CONST const uint8_t {p}_bitmap[] = {{
    {(","+chr(10)+"    ").join(rows)}
}};

static const lv_font_fmt_txt_glyph_dsc_t {p}_glyph_dsc[] = {{
{("," + chr(10)).join(dsc)}
}};

static const lv_font_fmt_txt_cmap_t {p}_cmaps[] = {{
    {{.range_start = {FIRST}, .range_length = {LAST - FIRST + 1}, .glyph_id_start = 1,
     .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY}}
}};

static lv_font_fmt_txt_glyph_cache_t {p}_cache;

static const lv_font_fmt_txt_dsc_t {p}_dsc = {{
    .glyph_bitmap = {p}_bitmap,
    .glyph_dsc = {p}_glyph_dsc,
    .cmaps = {p}_cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 1,
    .kern_classes = 0,
    .bitmap_format = 0,
    .cache = &{p}_cache
}};

const lv_font_t {name} = {{
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,
    .line_height = {8 * scale},
    .base_line = {scale},
    .subpx = LV_FONT_SUBPX_NONE,
    .underline_position = -1,
    .underline_thickness = 1,
    .dsc = &{p}_dsc
}};
"""


def main():
    data = load_glcd(GLCD)
    parts = ["""/*********************************************************************************
 * Fonturi LVGL cu aspect de afisaj LCD de bord (pixelat, ingrosat).
 * GENERAT AUTOMAT de tools/gen_lcd_fonts.py - nu edita manual.
 *
 * Sursa glifelor: "glcdfont" 5x7, Copyright (c) 2012 Adafruit Industries,
 * licenta BSD (textul complet in LovyanGFX/src/lgfx/Fonts/glcdfont.h).
 * Contine doar caracterele ASCII 0x20-0x7E.
 *********************************************************************************/
#include <lvgl.h>
"""]
    for name, scale, bold in FONTS:
        parts.append(gen_font(data, name, scale, bold))
    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write("".join(parts))
    print("Scris:", os.path.normpath(OUT))


if __name__ == "__main__":
    main()
