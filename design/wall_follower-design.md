# Wall follower algorithm for f1tenth
===========

This is the design document for the `wall_follower` package.


## Purpose / Use cases
Celem projektu jest stworzenie algorytmu opartego o regulator PID, który wykorzystując pomiary z skanera laserowego steruje samochodem. 


## Design
![alt text](https://github.com/RGoralewski/wall-follower-mr2/blob/master/design/Design.png?raw=true)


## Assumptions / Known limits
Założeniem jest osiągnięcie jak najwyższej prędkości i czasu na okrążenie testowej trasy bez ryzyka uderzenia w barierki.

## Inputs / Outputs / API
Dane wejściowe pochodzą ze skanera laserowego LIDAR. Dostarcza on około 1000 próbek z zakresu 270 stopni z częstotliwością 20Hz. Jako wyjście podawany jest skręt kół oraz prędkość samochodu.


## Inner-workings / Algorithms
### Algorytm obliczający błąd podawany na regulator PID
Sygnał ze skanera dzielony jest na prawą i lewą połowę w stosunku do samochodu. Następnie odległości w obu połowach są sumowane z zastąpieniem wartości inf wartością maksymalną dla skanu. Wyliczana jest różnica między nimi, a następnie dzielona przez sumę wszystkich odległości w celu normalizacji w zakresie od -1 do 1. 
### Regulator PID
Regulator PID na wyjściu podaje sterowanie skrętem kół. W regulatorze wyłączona jest część I. Nastawy KP oraz KI zostały wyznaczone metodą empiryczną.
### Algorytm uzależniający prędkość od skrętu kół.
W celu zapewnienia bezpieczeństwa na zakrętach stworzony został algorytm obliczający prędkość ze zdefiniowanego zakresu minimum - maximum. Algorytm w pierwszym stopniu sprawdza, czy wartość bezwzględna błędu pochodzącego z wcześniejszego algorytmu jest większa od zadanego progu. Przekroczenie progu oznacza zbliżanie się do zakrętu. W takim przypadku prędkość jest obniżana do prędkości minimalnej. Jeśli próg nie został przekroczony to prędkość jest liniowo zwiększana aż do prędkości maksymalnej


## Error detection and handling
Algorytm nie bazuje na pojedynczych próbkach ze skanera, tylko na sumie kilkuset próbek, które następnie są normalizowane.. Zapewnia to mniejszy wpływ błędnych próbek.


## Security considerations
Parametry (prędkość minimalna, nastawy PID) zostały dobrane tak, aby samochód pokonał okrążenie w jak najszybszym czasie i jednocześnie nie uderzył w ścianę. Aby zmniejszyć ryzyko wypadku pierwszym zalecanym krokiem jest zmniejszenie wartości minimalnej (np. 0.8), co spowoduje mniejszą prędkość pokonywania zakrętów i brak oscylacji po wyjściu z zakrętu.


## Future extensions / Unimplemented parts
Gdy nastawy regulatora PID zostały nastrojone w stopniu zadowalającym rozpoczęte zostały próby skrócenia czasu przejazdu okrążenia. W tym celu zmieniane były nastawy algorytmu obliczającego prędkość. Ostatecznie przy ustawieniu prędkości minimalnej równej prędkości maksymalnej nie zauważono znaczącego spadku wydajności oraz bezpieczeństwa, więc algorytm ten został wyłączony. Nie jest on usunięty, ponieważ w rzeczywistości regulacja prędkości może być ważniejsza niż w symulacji.


## Related issues
<!-- Required -->
