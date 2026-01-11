#!/usr/bin/env python3
from pathlib import Path
import json

import pyray as rl

FONT_DIR = Path(__file__).resolve().parent
SELFDRIVE_DIR = FONT_DIR.parents[1]
TRANSLATIONS_DIR = SELFDRIVE_DIR / "ui" / "translations"
LANGUAGES_FILE = TRANSLATIONS_DIR / "languages.json"

GLYPH_PADDING = 6
EXTRA_CHARS = "–‑✓×°§•X⚙✕◀▶✔⌫⇧␣○●↳çêüñ–‑✓×°§•€£¥·²"
UNIFONT_LANGUAGES = {"ar", "th", "zh-CHT", "zh-CHS", "ko", "ja"}


def _languages():
  if not LANGUAGES_FILE.exists():
    return {}
  with LANGUAGES_FILE.open(encoding="utf-8") as f:
    return json.load(f)


def _char_sets():
  base = set(map(chr, range(32, 127))) | set(EXTRA_CHARS)
  unifont = set(base)

  for language, code in _languages().items():
    unifont.update(language)
    po_path = TRANSLATIONS_DIR / f"app_{code}.po"
    try:
      chars = set(po_path.read_text(encoding="utf-8"))
    except FileNotFoundError:
      continue
    unifont.update(chars)
    # (unifont if code in UNIFONT_LANGUAGES else base).update(chars)

  return tuple(sorted(ord(c) for c in base)), tuple(sorted(ord(c) for c in unifont))


def _glyph_metrics(glyphs, rects, codepoints):
  entries = []
  min_offset_y, max_extent = None, 0
  for idx, codepoint in enumerate(codepoints):
    glyph = glyphs[idx]
    rect = rects[idx]
    width = int(round(rect.width))
    height = int(round(rect.height))
    offset_y = int(round(glyph.offsetY))
    min_offset_y = offset_y if min_offset_y is None else min(min_offset_y, offset_y)
    max_extent = max(max_extent, offset_y + height)
    entries.append(
      {
        "id": codepoint,
        "x": int(round(rect.x)),
        "y": int(round(rect.y)),
        "width": width,
        "height": height,
        "xoffset": int(round(glyph.offsetX)),
        "yoffset": offset_y,
        "xadvance": int(round(glyph.advanceX)),
      }
    )

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

    # 字體大小設定
    font_size = {
        "unifont.otf": 16,  # unifont 特殊尺寸
    }.get(font_path.name, 200)

    # 讀入 TTF / OTF
    data = font_path.read_bytes()
    file_buf = rl.ffi.new("unsigned char[]", data)

    # 根據字體類型自動過濾 safe glyph
    if "noto" in font_path.name.lower() or font_path.suffix.lower() in [".otf"]:
        # NotoSansTC 或 CJK 字體，保留 ASCII + CJK
        safe_codepoints = [cp for cp in codepoints if 0x20 <= cp <= 0x7E or 0x4E00 <= cp <= 0x9FFF]
    else:
        # 西文字體 / ASCII-only 字體
        safe_codepoints = [cp for cp in codepoints if 0x20 <= cp <= 0x7E]

    skipped = set(codepoints) - set(safe_codepoints)
    if skipped:
        print(f"[FONT DEBUG] {font_path.name} skipped {len(skipped)} unsupported glyphs:")
        for cp in list(skipped)[:20]:
            try:
                ch = chr(cp)
            except:
                ch = "<?>"
            print(f"  U+{cp:04X} '{ch}'")
        if len(skipped) > 20:
            print(f"  ... and {len(skipped) - 20} more")

    if not safe_codepoints:
        print(f"[FONT DEBUG] {font_path.name} has no supported glyphs, skipping")
        return

    # 分批載入 glyph 避免 crash（每批 256 glyph）
    batch_size = 256
    all_glyphs = []
    all_rects = []
    for i in range(0, len(safe_codepoints), batch_size):
        batch = safe_codepoints[i:i+batch_size]
        cp_buffer = rl.ffi.new("int[]", batch)
        cp_ptr = rl.ffi.cast("int *", cp_buffer)

        glyphs = rl.load_font_data(rl.ffi.cast("unsigned char *", file_buf), len(data), font_size, cp_ptr, len(batch), rl.FontType.FONT_DEFAULT)
        if glyphs == rl.ffi.NULL:
            raise RuntimeError(f"raylib failed to load font data for {font_path.name} (batch {i // batch_size})")

        rects_ptr = rl.ffi.new("Rectangle **")
        image = rl.gen_image_font_atlas(glyphs, rects_ptr, len(batch), font_size, GLYPH_PADDING, 0)
        if image.width == 0 or image.height == 0:
            raise RuntimeError(f"raylib returned an empty atlas for {font_path.name} (batch {i // batch_size})")

        rects = rects_ptr[0]
        all_glyphs.extend(glyphs[0:len(batch)])
        all_rects.extend(rects[0:len(batch)])

    # 生成 atlas 路徑
    atlas_name = f"{font_path.stem}.png"
    atlas_path = FONT_DIR / atlas_name

    # 計算 glyph metrics
    entries, line_height, base = _glyph_metrics(all_glyphs, all_rects, safe_codepoints)

    # 輸出 atlas
    if not rl.export_image(image, atlas_path.as_posix()):
        raise RuntimeError(f"Failed to export atlas image for {font_path.name}")

    # 寫入 bmfont
    _write_bmfont(FONT_DIR / f"{font_path.stem}.fnt", font_size, font_path.stem, atlas_name, line_height, base, (image.width, image.height), entries)


def main():
  base_cp, unifont_cp = _char_sets()
  fonts = sorted(FONT_DIR.glob("*.ttf")) + sorted(FONT_DIR.glob("*.otf"))
  for font in fonts:
    if "emoji" in font.name.lower():
      continue
    # glyphs = unifont_cp if font.stem.lower().startswith("unifont") else base_cp
    # _process_font(font, glyphs)
    _process_font(font, unifont_cp)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
