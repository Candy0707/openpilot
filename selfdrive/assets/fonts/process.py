#!/usr/bin/env python3
from pathlib import Path

import pyray as rl
from fontTools.ttLib import TTFont

FONT_DIR = Path(__file__).resolve().parent
GLYPH_PADDING = 1


def _get_font_codepoints(font_path: Path) -> tuple[int, ...]:
  try:
    with TTFont(font_path) as font:
      cmap = font.getBestCmap()
      if cmap:
        print(f"字體 {font_path.name} 偵測到 {len(cmap)} 個原生字元")
        return tuple(sorted(cmap.keys()))
  except Exception as e:
    print(f"無法讀取字體 {font_path.name} 的字元集，錯誤: {e}")

  return ()


def _glyph_metrics(glyphs, rects, glyph_count: int):
  entries = []
  offsets_y = []
  extents = []

  for idx in range(glyph_count):
    glyph, rect = glyphs[idx], rects[idx]
    width = int(round(rect.width))
    height = int(round(rect.height))
    offset_y = int(round(glyph.offsetY))

    offsets_y.append(offset_y)
    extents.append(offset_y + height)

    entries.append(
      {
        "id": glyph.value,
        "x": int(round(rect.x)),
        "y": int(round(rect.y)),
        "width": width,
        "height": height,
        "xoffset": int(round(glyph.offsetX)),
        "yoffset": offset_y,
        "xadvance": int(round(glyph.advanceX)),
      }
    )

  line_height = int(round(max(extents) - min(offsets_y)))
  base = int(round(max(extents)))
  return entries, line_height, base


def _write_bmfont(path: Path, font_size: int, face: str, atlas_name: str, line_height: int, base: int, atlas_size, entries):
  # TODO: why doesn't raylib calculate these metrics correctly?
  if line_height != font_size:
    print("using font size for line height", atlas_name)
    line_height = font_size
  lines = [
    f"info face=\"{face}\" size=-{font_size} bold=0 italic=0 charset=\"\" unicode=1 stretchH=100 smooth=0 aa=1 padding=0,0,0,0 spacing=0,0 outline=0",
    f"common lineHeight={line_height} base={base} scaleW={atlas_size[0]} scaleH={atlas_size[1]} pages=1 packed=0 alphaChnl=0 redChnl=4 greenChnl=4 blueChnl=4",
    f"page id=0 file=\"{atlas_name}\"",
    f"chars count={len(entries)}",
  ]
  for entry in entries:
    lines.append(
      (
        "char id={id:<4} x={x:<5} y={y:<5} width={width:<5} height={height:<5} "
        + "xoffset={xoffset:<5} yoffset={yoffset:<5} xadvance={xadvance:<5} page=0  chnl=15"
      ).format(**entry)
    )
  path.write_text("\n".join(lines) + "\n")


def _process_font(font_path: Path, codepoints: tuple[int, ...]):
  print(f"Processing {font_path.name}...")

  font_size = {
    "unifont.otf": 16,  # unifont is only 16x8 or 16x16 pixels per glyph
  }.get(font_path.name, 32)

  data = font_path.read_bytes()
  file_buf = rl.ffi.new("unsigned char[]", data)
  cp_buffer = rl.ffi.new("int[]", codepoints)
  cp_ptr = rl.ffi.cast("int *", cp_buffer)
  glyph_count = len(codepoints)

  # 修正：load_font_data 僅接收 6 個參數，移除末尾傳入的 int 指標
  glyphs = rl.load_font_data(
    rl.ffi.cast("unsigned char *", file_buf),
    len(data),
    font_size,
    cp_ptr,
    glyph_count,
    rl.FontType.FONT_DEFAULT
  )
  if glyphs == rl.ffi.NULL:
    raise RuntimeError("raylib failed to load font data")

  rects_ptr = rl.ffi.new("Rectangle **")
  image = rl.gen_image_font_atlas(glyphs, rects_ptr, glyph_count, font_size, GLYPH_PADDING, 0)
  if image.width == 0 or image.height == 0:
    raise RuntimeError("raylib returned an empty atlas")

  rects = rects_ptr[0]
  atlas_name = f"{font_path.stem}.png"
  atlas_path = FONT_DIR / atlas_name
  entries, line_height, base = _glyph_metrics(glyphs, rects, glyph_count)

  if not rl.export_image(image, atlas_path.as_posix()):
    raise RuntimeError("Failed to export atlas image")

  _write_bmfont(FONT_DIR / f"{font_path.stem}.fnt", font_size, font_path.stem, atlas_name, line_height, base, (image.width, image.height), entries)

def main():
  fonts = sorted(FONT_DIR.glob("*.ttf")) + sorted(FONT_DIR.glob("*.otf"))
  for font in fonts:
    if "emoji" in font.name.lower():
      continue
    glyphs = _get_font_codepoints(font)
    _process_font(font, glyphs)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
