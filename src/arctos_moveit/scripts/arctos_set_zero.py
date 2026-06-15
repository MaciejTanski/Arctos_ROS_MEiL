#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
  arctos_set_zero.py  —  Ustawienie biezacej pozycji osi jako ZERO (MKS 0x92)
  BEZ ROS. Do uzycia po recznym lub jogowym dojechaniu do znacznikow.
==============================================================================

WYMAGANIA:  pip install python-can pyserial

KONCEPT:
  Komenda MKS 0x92 deklaruje BIEZACA pozycje osi jako zero enkodera.
  NIC nie rusza — tylko ustawia punkt odniesienia. Bezpieczna.

PROCEDURA (obie metody konczy ten skrypt):
  A) RECZNIE: wylacz trzymanie momentu, ustaw os reka do znacznika, wlacz,
     uruchom ten skrypt -> 0x92.
  B) JOGIEM: dojedz jogiem do znacznika, potem uruchom ten skrypt -> 0x92.

!!! WAZNE — ZWERYFIKUJ FORMAT RAMKI 0x92 W SWOIM MANUALU MKS !!!
  Ponizej zaimplementowano najczestszy wariant [0x92, CRC]. Niektore wersje
  firmware MKS wymagaja dodatkowego bajtu (np. [0x92, 0x00, CRC]) albo innego
  kodu. Najpierw uruchom z DRY_RUN=True, porownaj wypisana ramke z manualem,
  i dopiero potem ustaw DRY_RUN=False. Zla ramka w najgorszym razie zostanie
  zignorowana, ale weryfikacja to obowiazek.
==============================================================================
"""

import can
import time

# ============================ KONFIGURACJA ============================
PORT     = "/dev/ttyACM0"          # Windows: "COM3"...  Linux: "/dev/ttyACM0"
BUSTYPE  = "slcan"
BITRATE  = 500000

# Ktore osie wyzerowac (numery AXIS_ID silnikow). Domyslnie wszystkie.
AXES_TO_ZERO = [1, 2, 3, 4, 5, 6]

DRY_RUN = False             # ZACZNIJ od True — sprawdz ramke, potem zmien na False
# ======================================================================


def encode_set_zero(axis_id):
    """
    Ramka 0x92 'ustaw biezaca pozycje jako zero'.
    Format najczestszy: [0x92, CRC], CRC = (axis_id + suma_bajtow) & 0xFF.
    ZWERYFIKUJ z manualem MKS przed uzyciem na sprzecie!
    """
    data = [0x92]
    data.append((axis_id + sum(data)) & 0xFF)   # CRC
    return can.Message(arbitration_id=axis_id, data=data, is_extended_id=False)


def main():
    print("=" * 60)
    print("  Arctos — ustawienie ZERA osi (MKS 0x92)")
    print("=" * 60)
    print(f"  Port: {PORT}   Osie: {AXES_TO_ZERO}")
    if DRY_RUN:
        print("  >>> DRY_RUN = True — tylko podglad ramek, NIC nie idzie na CAN <<<")
    print()
    print("  Upewnij sie, ze WSZYSTKIE osie sa na swoich znacznikach (zero).")
    if input("  Enter = ustaw zero na powyzszych osiach, tekst = anuluj: ").strip():
        print("  Anulowano."); return

    bus = None
    if not DRY_RUN:
        try:
            bus = can.interface.Bus(bustype=BUSTYPE, channel=PORT, bitrate=BITRATE)
            print(f"  Polaczono z {PORT}.\n")
        except Exception as e:
            print(f"  BLAD polaczenia z {PORT}: {e}")
            return

    try:
        for axis_id in AXES_TO_ZERO:
            msg = encode_set_zero(axis_id)
            hexd = " ".join(f"{b:02X}" for b in msg.data)
            print(f"  os {axis_id}: ZERO -> ID={axis_id:02X} data=[{hexd}]")
            if not DRY_RUN and bus is not None:
                bus.send(msg)
                time.sleep(0.1)
        print("\n  Gotowe. Biezaca poza = zero dla wybranych osi.")
        if DRY_RUN:
            print("  (DRY_RUN — nic nie wyslano. Sprawdz ramki z manualem, potem DRY_RUN=False.)")
    finally:
        if bus is not None:
            bus.shutdown()


if __name__ == "__main__":
    main()
