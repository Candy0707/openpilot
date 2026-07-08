#!/usr/bin/env python3
from pathlib import Path
import json

import pyray as rl
# 引入 fontTools 用於動態讀取字體內部的字元對照表 (cmap)
# 註：執行此腳本前請確保環境已安裝該套件 (pip install fonttools)
from fontTools.ttLib import TTFont

FONT_DIR = Path(__file__).resolve().parent
SELFDRIVE_DIR = FONT_DIR.parents[1]
# 已移除與 .po 檔案相關的舊路徑變數 (TRANSLATIONS_DIR 與 LANGUAGES_FILE)

GLYPH_PADDING = 2
EXTRA_CHARS = "–‑✓×°§•X⚙✕◀▶✔⌫⇧␣○●↳çêüñ–‑✓×°§•€£¥·²"
# 已移除 UNIFONT_LANGUAGES 限制變數


def _get_font_codepoints(font_path: Path) -> tuple[int, ...]:
  """
  開啟字體檔案并搜尋其內建支援的所有文字碼位 (Codepoints)，
  並與基礎 ASCII 以及 EXTRA_CHARS 進行聯集，確保基礎符號不遺漏。
  """
  try:
    # 使用 fontTools 讀取字體檔案 (.ttf / .otf)
    with TTFont(font_path) as font:
      # getBestCmap() 會回傳該字體支援的 Unicode 碼位字典 (Key 為 int 格式的碼位)
      cmap = font.getBestCmap()
      if cmap:
        # 建立基礎字元集：ASCII 32-126 以及 EXTRA_CHARS
        base_set = set(map(chr, range(32, 127))) | set(EXTRA_CHARS)
        # 將字體自帶的碼位與基礎字元的碼位做聯集 (Union)
        codepoints = set(cmap.keys()) | set(ord(c) for c in base_set)
        print(f"字體 {font_path.name} 偵測到 {len(cmap)} 個原生字元，合併基礎字元後共 {len(codepoints)} 個字元")
        return tuple(sorted(codepoints))
  except Exception as e:
    print(f"無法讀取字體 {font_path.name} 的字元集，將使用基礎字元備份。錯誤: {e}")

  # 備份方案：若讀取失敗，則僅回傳基礎 ASCII 與 EXTRA_CHARS
  base_set = set(map(chr, range(32, 127))) | set(EXTRA_CHARS)
  return tuple(sorted(ord(c) for c in base_set))


def _glyph_metrics(glyphs, rects, glyph_count: int):
  entries = []
  min_offset_y, max_extent = None, 0
  for idx in range(glyph_count):
    glyph = glyphs[idx]
    rect = rects[idx]
    width = int(round(rect.width))
    height = int(round(rect.height))
    offset_y = int(round(glyph.offsetY))
    min_offset_y = offset_y if min_offset_y is None else min(min_offset_y, offset_y)
    max_extent = max(max_extent, offset_y + height)
    entries.append({
      "id": glyph.value,
      "x": int(round(rect.x)),
      "y": int(round(rect.y)),
      "width": width,
      "height": height,
      "xoffset": int(round(glyph.offsetX)),
      "yoffset": offset_y,
      "xadvance": int(round(glyph.advanceX)),
    })

  if min_offset_y is None:
    raise RuntimeError("No glyphs were generated")

  line_height = int(round(max_extent - min_offset_y))
  base = int(round(max_extent))
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
  }.get(font_path.name, 100)

  data = font_path.read_bytes()
  file_buf = rl.ffi.new("unsigned char[]", data)
  cp_buffer = rl.ffi.new("int[]", codepoints)
  cp_ptr = rl.ffi.cast("int *", cp_buffer)
  glyph_count = rl.ffi.new("int *", len(codepoints))
  glyphs = rl.load_font_data(
    rl.ffi.cast("unsigned char *", file_buf), len(data), font_size, cp_ptr, len(codepoints),
    rl.FontType.FONT_DEFAULT, glyph_count
  )
  if glyphs == rl.ffi.NULL:
    raise RuntimeError("raylib failed to load font data")

  rects_ptr = rl.ffi.new("Rectangle **")
  image = rl.gen_image_font_atlas(glyphs, rects_ptr, glyph_count[0], font_size, GLYPH_PADDING, 0)
  if image.width == 0 or image.height == 0:
    raise RuntimeError("raylib returned an empty atlas")

  rects = rects_ptr[0]
  atlas_name = f"{font_path.stem}.png"
  atlas_path = FONT_DIR / atlas_name
  entries, line_height, base = _glyph_metrics(glyphs, rects, glyph_count[0])

  if not rl.export_image(image, atlas_path.as_posix()):
    raise RuntimeError("Failed to export atlas image")

  _write_bmfont(FONT_DIR / f"{font_path.stem}.fnt", font_size, font_path.stem, atlas_name, line_height, base, (image.width, image.height), entries)


def main():
  # 移除原本在迴圈外透過 _char_sets() 載入固定字元集的作法
  fonts = sorted(FONT_DIR.glob("*.ttf")) + sorted(FONT_DIR.glob("*.otf"))
  for font in fonts:
    if "emoji" in font.name.lower():
      continue
    # 每個字體在處理時，動態呼叫函式來獲取該字體實質支援的所有字元碼位
    glyphs = _get_font_codepoints(font)
    _process_font(font, glyphs)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())