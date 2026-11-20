# Planar Quadrotor LQR

Symulacja quadrotora w płaszczyźnie 2D sterowanego dyskretnym regulatorem **LQR**.
Kliknięcie w oknie ustawia nowy punkt docelowy (flaga), a regulator prowadzi drona do celu,
stabilizując jego położenie i orientację. Po locie można wyświetlić wykres przebytej trajektorii.

![C++](https://img.shields.io/badge/C++17-00599C?logo=cplusplus&logoColor=white)
![Eigen](https://img.shields.io/badge/Eigen-3.4-informational)
![SDL2](https://img.shields.io/badge/SDL2-grafika%20i%20d%C5%BAwi%C4%99k-1D4F8C)
![CMake](https://img.shields.io/badge/CMake-064F8C?logo=cmake)

## Funkcje

- **Model dynamiki quadrotora 2D**: stan `[x, y, θ, ẋ, ẏ, θ̇]`, dwa wejścia (ciągi wirników).
- **Regulator LQR**: linearyzacja modelu wokół punktu zawisu, dyskretyzacja (`A_d = I + dt·A`, `B_d = dt·B`)
  i wyznaczenie macierzy wzmocnień `K` dla dobranych wag `Q` i `R`. Sterowanie:
  `u = u_hover − K·(x − x_goal)`.
- **Wizualizacja w SDL2 / SDL2_gfx**: kadłub i obracające się śmigła, flaga w punkcie docelowym.
- **Interakcja**: lewy przycisk myszy ustawia nowy cel, klawisz `P` rysuje trajektorię lotu (Matplot++).
- **Dźwięk** silników odtwarzany przez SDL Audio (`PlanarQuadrotor/*.wav`).

## Budowanie

Zależności (SDL2, SDL2_gfx, Matplot++) są submodułami, a Eigen pobiera CPM podczas konfiguracji CMake.

```bash
git clone --recursive https://github.com/praha6248/planar-quadrotor-lqr.git
cd planar-quadrotor-lqr
cmake -S . -B build
cmake --build build -j
```

Uruchamiaj z katalogu głównego repozytorium, żeby program znalazł pliki dźwiękowe:

```bash
./build/planar_quadrotor
```

Na Windowsie można otworzyć folder w Visual Studio albo VS Code z CMake Tools.

## Struktura

```
PlanarQuadrotor/
  planar_quadrotor.{h,cpp}             model dynamiki, linearyzacja, kompensacja grawitacji
  lqr.{h,cpp}                          wyznaczanie macierzy wzmocnień LQR
  planar_quadrotor_visualizer.{h,cpp}  rysowanie drona w SDL
  simulate.{h,cpp}                     pętla symulacji, obsługa myszy i klawiatury, dźwięk, wykres
  normal.wav, slow.wav                 dźwięki silników
cmake/CPM.cmake                        menedżer zależności (Eigen)
matplotplusplus/, SDL/, SDL2_gfx/      submoduły
```

## Pochodzenie

Projekt powstał na przedmiocie Techniki Programowania na bazie szablonu
[marekjg/tp_projekt_4](https://github.com/marekjg/tp_projekt_4) (konfiguracja CMake i szkielet klas).
Moja część: uzupełnienie modelu i regulatora, dobór macierzy Q i R, wizualizacja śmigieł i flagi celu,
wybór celu myszą, wykres trajektorii i dźwięk.

## Technologie

C++17 · Eigen · SDL2 · SDL2_gfx · Matplot++ · CMake · CPM
