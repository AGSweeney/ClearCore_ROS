# Third-party notices

ClearCore ROS is released under the [MIT License](LICENSE), Copyright (c) 2026 Adam G. Sweeney.
It builds on code by others, which remains under its own copyright and licence.

| Component | Path | Copyright | Licence |
|-----------|------|-----------|---------|
| EnhancedClearCoreLibrary (git submodule): Teknic's ClearCore library with additions by Adam G. Sweeney, Microchip ASF and device startup files, lwIP | `external/EnhancedClearCoreLibrary/` | Teknic, Inc.; Adam G. Sweeney; Microchip Technology Inc.; Swedish Institute of Computer Science and contributors | MIT (ClearCore library and additions); Microchip ASF licence; Apache-2.0 (startup files); BSD-style (lwIP). See that repository's `LICENSE` and `THIRD_PARTY_NOTICES.md`. |
| jsmn JSON tokenizer | `firmware/third_party/jsmn/` | Serge Zaitsev | MIT, in `jsmn.h` |

`host/clearpath_usb.py` implements the ClearPath USB (MSP) node protocol from details in Teknic's
public sFoundation SDK (`pubMnNetDef.h`, `SerialEx.cpp`, `cpmAPI.cpp`, `pubCpmRegs.h`); no SDK code is
included.
