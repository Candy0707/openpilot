#!/usr/bin/env python3
from pathlib import Path
import json

import pyray as rl

FONT_DIR = Path(__file__).resolve().parent
SELFDRIVE_DIR = FONT_DIR.parents[1]
TRANSLATIONS_DIR = SELFDRIVE_DIR / "ui" / "translations"
LANGUAGES_FILE = TRANSLATIONS_DIR / "languages.json"

# [修改] 為了降低記憶體使用量，將字距 (padding) 從 6 縮減至 2。
# 這能讓字元排列更緊密，大幅縮小輸出的 .png 圖集尺寸。
GLYPH_PADDING = 2

# [修改] 移除原有結尾較少用到的特殊符號 (如 ·²)，僅保留基礎的 UI 介面控制符號與標點。
EXTRA_CHARS = "–‑✓×°§•X⚙✕◀▶✔⌫⇧␣○●↳çêüñ–‑✓×°§•€£¥"

# [修改] 根據您的需求，移除了泰文 (th)、簡體中文 (zh-CHS)、韓文 (ko) 與日文 (ja)。
# 僅保留繁體中文 (zh-CHT)，避免生成涵蓋多國語言的超大圖集，進而降低記憶體佔用。
UNIFONT_LANGUAGES = {"zh-CHT"}


def _languages():
  # 讀取系統支援語言的 JSON 設定檔
  if not LANGUAGES_FILE.exists():
    return {}
  with LANGUAGES_FILE.open(encoding="utf-8") as f:
    return json.load(f)


def _char_sets():
  # 建立基礎字元集，包含 ASCII (範圍 32-127，涵蓋英文字母、數字與基本標點) 以及 EXTRA_CHARS 特殊符號
  base = set(map(chr, range(32, 127))) | set(EXTRA_CHARS)
  
  # 建立基礎標籤字元集，用於純英文與語言選單名稱的精簡圖集
  labels = set(base) 
  
  # 字典：用於存放特定語系 (繁體中文) 專屬的字元集
  per_lang: dict[str, tuple[int, ...]] = {} 

  # 遍歷語言設定檔中的所有語言項目
  for language, code in _languages().items():
    # [修改] 嚴格過濾：只處理我們在 UNIFONT_LANGUAGES 中定義的語系 (此處為 zh-CHT)
    if code not in UNIFONT_LANGUAGES:
      continue
      
    # 將語言的顯示名稱加入基礎標籤字元集，
    # 確保在切換語言的介面中，即使還沒載入完整中文圖集也能正常顯示「繁體中文」等選項字眼。
    labels.update(language)
    
    lang_chars = set()
    # 讀取該語系的官方翻譯檔 (po 檔) 來擷取需要渲染的字元
    po_path = TRANSLATIONS_DIR / f"app_{code}.po"
    try:
      chars = set(po_path.read_text(encoding="utf-8"))
      lang_chars.update(chars)
      print(f"Language {language} ({code}) has {len(chars)} unique characters from app_{code}.po")
    except FileNotFoundError:
      continue
        
    # 將基礎英文/符號與該語系的專屬中文字元合併，準備產生該語系的圖集
    lang_chars_combined = set(base) | lang_chars
    # 轉換為排序好的 Unicode 碼點 (codepoint) tuple 並存入字典
    per_lang[code] = tuple(sorted(ord(c) for c in lang_chars_combined))

  # 將基礎字元與標籤字元也轉換為排序好的碼點 tuple
  base_cp = tuple(sorted(ord(c) for c in base))
  labels_cp = tuple(sorted(ord(c) for c in labels))
  
  return base_cp, labels_cp, per_lang


def _glyph_metrics(glyphs, rects, codepoints):
  # 計算字元的排版度量數據 (metrics)，包含字元在圖集中的 X/Y 座標與寬高，用於後續寫入 .fnt 檔
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
  # 將排版數據格式化並寫入 BMFont 格式的設定檔 (.fnt)
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


def _process_font(font_path: Path, codepoints: tuple[int, ...], output_name: str | None = None):
  # [修改] 支援自訂輸出名稱 (output_name)，以便為同一個字型產生 -Labels 和 -zh-CHT 兩種不同後綴的圖集
  stem = output_name or font_path.stem
  
  # [修改] 動態調整字型大小，不再全域寫死為 200px。
  # 若是 openpilot 主字型 (OpFont 開頭) 設為 48，非特殊字型則設為 120，這能極大程度縮小圖集解析度及記憶體負擔。
  font_size = 48 if font_path.stem.lower().startswith("opfont") else 120
  if font_path.name == "unifont.otf":
      font_size = 16  # unifont is only 16x8 or 16x16 pixels per glyph

  print(f"Processing {font_path.name} -> {stem} ({len(codepoints)} glyphs @ {font_size}px)...")

  # 讀取字型原始檔並透過 CFFI 傳遞給 raylib
  data = font_path.read_bytes()
  file_buf = rl.ffi.new("unsigned char[]", data)
  cp_buffer = rl.ffi.new("int[]", codepoints)
  cp_ptr = rl.ffi.cast("int *", cp_buffer)
  
  # 呼叫 raylib 載入字型資料
  glyphs = rl.load_font_data(rl.ffi.cast("unsigned char *", file_buf), len(data), font_size, cp_ptr, len(codepoints), rl.FontType.FONT_DEFAULT)
  if glyphs == rl.ffi.NULL:
    raise RuntimeError("raylib failed to load font data")

  # 根據字元集合與大小，自動生成最佳排列的圖集圖片 (atlas)
  # [注意] 此處依要求確保每次皆重新生成，不使用快取跳過邏輯。
  rects_ptr = rl.ffi.new("Rectangle **")
  image = rl.gen_image_font_atlas(glyphs, rects_ptr, len(codepoints), font_size, GLYPH_PADDING, 0)
  if image.width == 0 or image.height == 0:
    raise RuntimeError("raylib returned an empty atlas")

  rects = rects_ptr[0]
  atlas_name = f"{stem}.png"
  atlas_path = FONT_DIR / atlas_name
  entries, line_height, base = _glyph_metrics(glyphs, rects, codepoints)

  # 匯出 png 圖片檔
  if not rl.export_image(image, atlas_path.as_posix()):
    raise RuntimeError("Failed to export atlas image")

  # 匯出 fnt 排版設定檔
  _write_bmfont(FONT_DIR / f"{stem}.fnt", font_size, stem, atlas_name, line_height, base, (image.width, image.height), entries)


def main():
  # [修改] 取得精簡後的英文字元 (Labels) 以及繁體中文字元集
  base_cp, labels_cp, per_lang = _char_sets()
  
  fonts = sorted(FONT_DIR.glob("*.ttf")) + sorted(FONT_DIR.glob("*.otf"))
  opfonts: list[Path] = []

  # 第一階段：先處理普通字型，並將主介面使用的 OpFont 過濾出來
  for font in fonts:
    if "emoji" in font.name.lower() or font.name == "unifont.otf":
      continue
    
    # [修改] 偵測到 OpFont 時，先存入清單，稍後再針對它進行語系拆分渲染
    if font.stem.lower().startswith("opfont"):
      opfonts.append(font)
      continue
      
    # 一般非主字型僅套用最基礎的英文字元與符號
    _process_font(font, base_cp)

  # 第二階段：針對 OpFont 進行圖集拆分作業，達成記憶體最佳化
  for opfont_path in opfonts:
    weight = opfont_path.stem  # 取得字型粗細名稱，例如: "OpFont-Regular"

    # [修改] 拆分任務 1：生成輕量級的 Labels 圖集 (僅含基礎英文與語言選單的中文標籤文字)
    _process_font(opfont_path, labels_cp, output_name=f"{weight}-Labels")

    # [修改] 拆分任務 2：生成繁體中文專屬圖集 (zh-CHT)
    for lang_code, lang_cp in per_lang.items():
      _process_font(opfont_path, lang_cp, output_name=f"{weight}-{lang_code}")

  return 0


if __name__ == "__main__":
  raise SystemExit(main())

