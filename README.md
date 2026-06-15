# Arctos_ROS_MEiL
 
Workspace catkin (ROS **Melodic**) dla ramienia robotycznego **Arctos** (6-DOF). Zawiera konfigurację MoveIt, opis URDF, węzeł pick & place oraz most CAN sterujący sterownikami **MKS Servo 42D/57D** na magistrali CAN.
 
> **Środowisko uruchomieniowe:** przeznaczony do pracy w kontenerze ROS Melodic. Pełna otoczka Docker (obraz, compose, skrypty startowe, akceleracja GPU) znajduje się w repo [`ARCTOS_DOCKER_MEIL`](https://github.com/LukPogo/ARCTOS_DOCKER_MEIL), gdzie ten workspace jest dołączony jako submoduł i montowany pod `/root/catkin_ws`.
 
---
 
## Spis treści
 
1. [Pakiety](#1-pakiety)
2. [Konfiguracja robota](#2-konfiguracja-robota)
3. [Wymagania i budowa](#3-wymagania-i-budowa)
4. [Architektura sterowania — pętla ROS](#4-architektura-sterowania--pętla-ros)
5. [Węzły i skrypty](#5-węzły-i-skrypty)
6. [Most CAN — matematyka i protokół MKS](#6-most-can--matematyka-i-protokół-mks)
7. [Zerowanie bez krańcówek](#7-zerowanie-bez-krańcówek)
8. [Uruchamianie](#8-uruchamianie)
9. [Protokół MKS — komendy CAN](#9-protokół-mks--komendy-can)
10. [Stan i roadmap](#10-stan-i-roadmap)
---
 
## 1. Pakiety
 
| Folder (`src/`) | Nazwa pakietu | Zawartość |
| :--- | :--- | :--- |
| `arctos_config` | `arctos_config` | Konfiguracja MoveIt: `demo.launch`, SRDF, pliki RViz, `fake_controllers` |
| `arctos_urdf_description` | `arctos_urdf_description` | Opis URDF robota + siatki STL |
| `arctos_moveit` | **`moveo_moveit`** | Węzły C++ (pick & place, jog) i skrypty Python (most CAN, zerowanie) |
 
> ⚠️ **Pułapka nazw:** folder to `src/arctos_moveit`, ale nazwa pakietu (z `package.xml` / `project()`) to **`moveo_moveit`**. Dlatego:
> ```bash
> roslaunch arctos_config demo.launch         # launch po pakiecie arctos_config
> rosrun moveo_moveit arctos_pick_place        # rosrun po pakiecie moveo_moveit
> ```
 
---
 
## 2. Konfiguracja robota
 
| Parametr | Wartość |
| :--- | :--- |
| Stopnie swobody | 6 |
| Stawy (joints) | `joint1` … `joint6` (bez podkreślnika) |
| Grupa MoveIt | `arm` |
| End-effector | `hand` na linku `Link_6_1` |
| Chwytak | `jaw1`, `jaw2` |
| ID osi CAN | `1` … `6` |
| Przełożenia (gear ratios) | `[6.75, 75, 75, 24, 33.91, 33.91]` |
| Rozdzielczość enkodera MKS | 16384 impulsów / obrót |
 
### Nadgarstek różnicowy (joints 5 i 6)
 
Joints 5 i 6 to **mechanizm różnicowy** — dwa fizyczne silniki sprzężone tak, że ich kombinacja daje **pitch** (joint5) i **roll** (joint6). Most CAN miesza te osie przed wysłaniem:
 
- roll wymaga **zgodnych** znaków obu silników (silniki kręcą się razem),
- pitch wymaga **przeciwnych** znaków (silniki kręcą się przeciwnie).
> Kierunek/transpozycja sprzężenia podlega walidacji na sprzęcie — patrz [Stan i roadmap](#10-stan-i-roadmap).
 
---
 
## 3. Wymagania i budowa
 
### Zależności
 
- ROS Melodic (Ubuntu 18.04) + MoveIt
- catkin tools (`catkin build`)
- Python 3: `python-can`, `rospkg`, `python3-netifaces`
(Wszystkie zapewnione przez obraz Docker z repo `ARCTOS_DOCKER_MEIL`.)
 
### Budowa
 
```bash
cd /root/catkin_ws
catkin build
source devel/setup.bash
rospack find arctos_config        # weryfikacja: wskazuje /root/catkin_ws/src/arctos_config
```
 
Po świeżym klonie `build/ devel/ logs/` nie istnieją (są gitignorowane) — trzeba zbudować raz. Pliki C++ (`arctos_pick_place.cpp`, `arctos_jog.cpp`) wymagają `catkin build` po każdej edycji; launch / SRDF / yaml / skrypty Python działają od razu.
 
---
 
## 4. Architektura sterowania — pętla ROS
 
```
   ┌─────────────────────┐
   │  MoveIt (planowanie) │   arctos_pick_place / arctos_jog
   └──────────┬──────────┘
              │ publikuje
              ▼
      /joint_states  (pozycje przegubów w radianach)
              │ subskrybuje
              ▼
   ┌─────────────────────┐
   │  can_bridge_f5.py   │  rad → impulsy enkodera → ramka MKS (0xF5)
   └──────────┬──────────┘
              │ python-can @ 20 Hz
              ▼
   DRY_RUN=True  → tylko logi ramek w konsoli
   DRY_RUN=False → CAN (slcan /dev/ttyACM0) → serwa MKS
```
 
### Dwie zasady sterowania (wyciągnięte z błędów)
 
1. **Jedno źródło na `/joint_states`.** `demo.launch` uruchamia `joint_state_publisher` publikujący zera. Ręczny `rostopic pub` to drugie źródło → most łapie na zmianę cel i zero, model drga. Ruch ma iść jednym kanałem: MoveIt albo `arctos_jog`.
2. **Brak losowego planera do pojedynczej osi.** Domyślny RRTConnect próbkuje losowo — dla trywialnego ruchu jednej osi błądzi i daje inny wynik za każdym razem. Stąd `arctos_jog` używa **deterministycznej interpolacji liniowej w przestrzeni stawów** (z pominięciem OMPL).
---
 
## 5. Węzły i skrypty
 
### `arctos_pick_place` (C++)
 
Węzeł pick & place oparty o MoveIt — planuje i wykonuje trajektorię, publikując na `/joint_states`.
 
```bash
rosrun moveo_moveit arctos_pick_place
```
 
### `arctos_jog` (C++)
 
Interaktywna konsola do sterowania pojedynczą osią, deterministyczna (interpolacja liniowa w przestrzeni stawów, bez OMPL):
 
- jednostki `r`/`d` (oraz `rad`/`deg`), prefiks `=` dla pozycji absolutnej, łączalne (`=90d`);
- tryb **podgląd → potwierdzenie**: komenda rysuje trajektorię w RViz (bez ruchu robota i bez CAN), `y` wykonuje, `n` anuluje.
```bash
rosrun moveo_moveit arctos_jog
```
 
### `scripts/can_bridge_f5.py`
 
Most CAN: subskrybuje `/joint_states`, przelicza kąty na pozycję enkodera i wysyła ramki absolutne `0xF5` z częstotliwością 20 Hz. Szczegóły w [sekcji 6](#6-most-can--matematyka-i-protokół-mks).
 
### `scripts/arctos_set_zero.py`
 
Utrwala bieżącą pozę jako zero enkodera — wysyła komendę `0x92` na wybrane osie (z weryfikacją odpowiedzi i trybem `DRY_RUN`). Patrz [sekcja 7](#7-zerowanie-bez-krańcówek).
 
---
 
## 6. Most CAN — matematyka i protokół MKS
 
### Przeliczenie pozycji (`angle_to_pulses`)
 
```
pulsy = (kąt_rad / 2π) × gear_ratio × 16384
```
 
Następnie nakładana jest inwersja kierunku (`INVERT[i]`) i — przy ruchu absolutnym — odniesienie do pozycji home (pływające zero ustalane przy starcie węzła lub przez `0x92`).
 
### Ruch absolutny `0xF5` (aktywny)
 
Most używa **`0xF5`**, nie `0xF4`. Powód:
 
- `0xF4` jest **względny wobec bieżącej pozycji silnika**. Przy cyklicznym nadawaniu (20 Hz) każde ponowne wysłanie tej samej ramki przesuwa silnik **jeszcze raz** → ciągłe obracanie. Gładkie rampy się **akumulują**.
- `0xF5` jest **absolutny i idempotentny** — ponowne wysłanie tego samego celu trzyma pozycję, nie dokłada ruchu. Rampy działają, resend jest nieszkodliwy, deadband zbędny.
### Format ramki F4/F5 (8 bajtów)
 
```
[kod, speedH, speedL, acc, posH, posM, posL, CRC]
```
 
- **kod:** `0xF5` (absolutny) lub `0xF4` (względny)
- **speed:** 16-bit, rozbity na bajt wysoki/niski
- **acc:** 8-bit — parametr **rampy firmware** (nie fizyczne przyspieszenie); większa wartość = krótsza rampa, `0` = brak rampy (przypadek specjalny)
- **pos:** 24-bit signed (uzup. do dwóch), pozycja w impulsach enkodera
- **CRC:** `(arbitration_id + suma_bajtów_data) & 0xFF`
`arbitration_id` = ID osi (1–6), standardowa ramka CAN (11-bit), `bitrate = 500000`.
 
### Prędkość
 
`fake_controllers` w `demo.launch` nie wypełnia pola `velocity` w `/joint_states`, więc prędkość per ramka liczona jest numerycznie: `(pos_now − pos_prev) / dt`, przeliczana z delta-impulsów na RPM.
 
### Konfiguracja sprzętowa (w `can_bridge_f5.py`)
 
| Zmienna | Symulacja | Sprzęt (live) |
| :--- | :--- | :--- |
| `DRY_RUN` | `True` | `False` |
| `BUSTYPE` | `"socketcan"` | `"slcan"` |
| `CHANNEL` | `"vcan0"` | `"/dev/ttyACM0"` |
| `BITRATE` | — | `500000` |
| `RATE_HZ` | `20` | `20` |
 
---
 
## 7. Zerowanie bez krańcówek
 
Robot nie ma krańcówek, a pamięć wielozwojowa enkodera **nie przeżywa wyłączenia zasilania** (licznik obrotów `carry` zeruje się; tryb auto-powrotu `0_Mode` działa tylko w obrębie 359°, single-turn). Przy przełożeniach 6.75–75 to nie wystarcza do jednoznacznej pozy przegubu.
 
Wniosek: przy każdym zimnym starcie potrzebna jest **ręczna referencja do znaczników**.
 
### Procedura
 
1. **Zasilanie.** Dojedź każdą osią do fizycznych znaczników — `arctos_jog`, tryb podgląd → `y`.
2. **Utrwal zero:** `arctos_set_zero.py` wysyła `0x92` na osie → bieżąca poza staje się zerem enkodera (bez ruchu silnika, bezpieczne).
3. Od tego punktu enkoder absolutny = pozycja względem home → ruch `0xF5` z celem „impulsy od home" działa wprost.
4. Powtarzaj kroki 1–2 przy każdym zimnym starcie.
---
 
## 8. Uruchamianie
 
### Symulacja MoveIt + RViz
 
```bash
roslaunch arctos_config demo.launch
```
 
### Pełny stos (RViz + most CAN)
 
W środowisku Docker z repo `ARCTOS_DOCKER_MEIL` skrypty `start.sh` / `run.sh` orkiestrują panele tmux (MoveIt, monitor mostu, konsola operatorska).
 
### Test pojedynczej osi
 
```bash
# w kontenerze:
/root/run_test.sh
# następnie sterowanie przez arctos_jog (nie przez rostopic pub — patrz sekcja 4)
rosrun moveo_moveit arctos_jog
```
 
---
 
## 9. Protokół MKS — komendy CAN
 
CAN: ID = adres osi (01–10), kod funkcji, `CRC = (ID + suma bajtów) & 0xFF`.
 
| Kod | Funkcja | Uwagi |
| :--- | :--- | :--- |
| `0x30` | odczyt enkodera (carry + value) | value 0–0x3FFF (16384/obrót), carry = liczba obrotów; pozycja absolutna = carry×0x4000 + value |
| `0x31` | odczyt skumulowany (addition) | int48; „axis" używany przez ruch absolutny |
| `0x32` | odczyt prędkości | int16 RPM — do porównania commanded vs. actual |
| `0x3E` | monitor utyku (stall) | empiryczna walidacja osiągalnego `acc` pod obciążeniem |
| `0x92` | ustaw bieżącą oś jako zero | bez ruchu silnika, bezpieczne |
| `0xF4` | ruch **względny** „by axis" | przesuwa względem bieżącej pozycji — **niezalecany do streamingu** (akumulacja) |
| `0xF5` | ruch **absolutny** „by axis" | do zadanej wartości enkodera, idempotentny — **aktywny** |
| `0xFE` | ruch absolutny po liczbie impulsów | nowsze firmware |
| `0x9A` | konfiguracja `0_Mode` | auto-powrót do zera po starcie (single-turn ≤359°) |
 
> Referencja: manuał MKS SERVO4257D, protokół CAN. Dokładny układ bajtów `0xF5`/`0xFE` zależy od wersji firmware — potwierdź dla swoich sztuk.
 
---
 
## 10. Stan i roadmap
 
### Działa
 
- Symulacja MoveIt + RViz.
- Sterowanie pojedynczą osią (`arctos_jog`, deterministyczne).
- Most CAN `can_bridge_f5.py` — ramki absolutne `0xF5`, zweryfikowane.
### W toku / do walidacji na sprzęcie
 
- Diagnostyka sprzężenia nadgarstka różnicowego (joints 5/6): zadać czysty ruch joint6 i sprawdzić, czy silniki kręcą się razem (poprawne dla roll) czy przeciwnie — to rozstrzyga ewentualną transpozycję `p5 ↔ p6`.
- Procedura zerowania do znaczników na fizycznym robocie.
- Czy `RATE_HZ` zwiększyć do 50 Hz dla gęstszej trajektorii; czy clamp `SPEED_MAX` powoduje lag przy szybkich planach MoveIt.
- Empiryczne strojenie per-joint `acc` z monitorem utyku (`0x3E`).
### Gotowe, ale świadomie odłożone (nie wprowadzone)
 
- `arctos_limits.h` (limity przegubów), `arctos_planner_params.h` (parametry solvera), `arctos_go_home.cpp`, `pick_place.launch`.
