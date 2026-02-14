# PCB Carrier Board - Pico Sensor Hub

Carte porteuse pour Raspberry Pi Pico + BMI160 + NEO-6M avec bornier de connexion Pi5.

## Vue d'ensemble

```
┌─────────────────────────────────────────────────────┐
│                                                     │
│   ┌─────────────────────────────────┐               │
│   │      Raspberry Pi Pico          │               │
│   │      (headers femelles)         │               │
│   └─────────────────────────────────┘               │
│                                                     │
│   ┌──────────┐       ┌──────────────┐               │
│   │ BMI160   │       │  NEO-6M GPS  │               │
│   │ (I2C1)   │       │  (UART1)     │               │
│   └──────────┘       └──────────────┘               │
│                                                     │
│   ┌────────────────┐                                │
│   │ BORNIER 4 pins │  ← vers Pi5 (UART + alim)     │
│   └────────────────┘                                │
│                                                     │
│   C1 ○  C2 ○  C3 ○    (condensateurs découplage)   │
│                                                     │
└─────────────────────────────────────────────────────┘
```

## Composants

| Ref  | Composant             | Footprint EasyEDA                  | Quantité |
|------|-----------------------|------------------------------------|----------|
| U1   | Raspberry Pi Pico     | 2x20 pin header femelle (2.54mm)   | 1        |
| U2   | BMI160 breakout       | 1x7 pin header femelle (2.54mm)    | 1        |
| U3   | NEO-6M module         | 1x4 pin header femelle (2.54mm)    | 1        |
| J1   | Bornier vis 4 pins    | Bornier 4P pas 5.08mm              | 1        |
| C1   | 100nF ceramique       | 0805                               | 1        |
| C2   | 100nF ceramique       | 0805                               | 1        |
| C3   | 100nF ceramique       | 0805                               | 1        |
| R1   | 4.7k (I2C pull-up)    | 0805                               | 1        |
| R2   | 4.7k (I2C pull-up)    | 0805                               | 1        |

Note: R1/R2 sont optionnels si le module BMI160 a déjà des pull-ups intégrés (vérifier le breakout).

## Pinout Raspberry Pi Pico (U1)

```
                    USB
            ┌───────┴───────┐
   GP0  1  ─┤               ├─  40  VBUS
   GP1  2  ─┤               ├─  39  VSYS  ← 5V depuis Pi5
   GND  3  ─┤               ├─  38  GND
   GP2  4  ─┤               ├─  37  3V3_EN
   GP3  5  ─┤               ├─  36  3V3_OUT → VCC modules
   GP4  6  ─┤               ├─  35  ADC_VREF
   GP5  7  ─┤               ├─  34  GP28
   GND  8  ─┤               ├─  33  GND
   GP6  9  ─┤               ├─  32  GP27
   GP7 10  ─┤               ├─  31  GP26
   GP8 11  ─┤               ├─  30  RUN
   GP9 12  ─┤               ├─  29  GP22
   GND 13  ─┤               ├─  28  GND
  GP10 14  ─┤               ├─  27  GP21
  GP11 15  ─┤               ├─  26  GP20
  GP12 16  ─┤               ├─  25  GP19
  GP13 17  ─┤               ├─  24  GP18
   GND 18  ─┤               ├─  23  GND
  GP14 19  ─┤               ├─  22  GP17
  GP15 20  ─┤               ├─  21  GP16
            └───────────────┘
```

## Netlist - Connexions

### 1. Alimentation (5V depuis Pi5)

| Source            | Destination         | Signal  | Note                        |
|-------------------|---------------------|---------|-----------------------------|
| J1 pin 1 (5V)    | U1 pin 39 (VSYS)   | 5V      | Alimentation Pico via Pi5   |
| J1 pin 2 (GND)   | U1 pin 38 (GND)    | GND     | Masse commune               |
| U1 pin 36 (3V3)  | U2 3V3 (BMI160)     | 3.3V    | Alim IMU directe 3.3V (bypass VIN) |
| U1 pin 39 (VSYS) | U3 VCC (NEO-6M)    | 5V      | Alim GPS en 5V (régulateur onboard module) |
| C1                | VSYS ↔ GND         | -       | Découplage entrée 5V        |
| C2                | 3V3 ↔ GND (BMI160) | -       | Découplage IMU              |
| C3                | 5V ↔ GND (GPS)     | -       | Découplage GPS              |

### 2. UART0 - Communication Pi5 (via bornier J1)

| Source              | Destination         | Signal   | Note                    |
|---------------------|---------------------|----------|-------------------------|
| U1 pin 1 (GP0/TX)  | J1 pin 3 (TX)       | UART0 TX | Pico TX → Pi5 GPIO9/RX |
| U1 pin 2 (GP1/RX)  | J1 pin 4 (RX)       | UART0 RX | Pico RX ← Pi5 GPIO8/TX |

### 3. UART1 - GPS NEO-6M (U3)

| Source              | Destination   | Signal   | Note                     |
|---------------------|---------------|----------|--------------------------|
| U1 pin 6 (GP4/TX)  | U3 RX         | UART1 TX | Pico TX → GPS RX        |
| U1 pin 7 (GP5/RX)  | U3 TX         | UART1 RX | Pico RX ← GPS TX        |
| U1 GND              | U3 GND        | GND      | Masse                    |

### 4. I2C1 - IMU BMI160 (U2)

| Source               | Destination   | Signal   | Note                     |
|----------------------|---------------|----------|--------------------------|
| U1 pin 19 (GP14)    | U2 SDA        | I2C1 SDA | Data                     |
| U1 pin 20 (GP15)    | U2 SCL        | I2C1 SCL | Clock                    |
| U1 GND               | U2 GND        | GND      | Masse                    |
| R1 (4.7k)           | SDA ↔ 3V3    | Pull-up  | Optionnel (voir note)    |
| R2 (4.7k)           | SCL ↔ 3V3    | Pull-up  | Optionnel (voir note)    |
| U2 SAD              | GND           | -        | Adresse I2C = 0x68       |
| U2 CS               | 3V3           | -        | Force mode I2C           |

## Bornier J1 - Connexion Pi5

```
Bornier à vis 4 pins (pas 5.08mm)
┌─────┬─────┬─────┬─────┐
│  1  │  2  │  3  │  4  │
│ 5V  │ GND │ TX  │ RX  │
└─────┴─────┴─────┴─────┘
  │     │     │     │
  │     │     │     └── vers Pi5 GPIO8/TXD3 (broche 24)
  │     │     └──────── vers Pi5 GPIO9/RXD3 (broche 21)
  │     └────────────── vers Pi5 GND (broche 20)
  └──────────────────── vers Pi5 5V (broche 2 ou 4)
```

Câble 4 fils entre bornier PCB et Pi5 GPIO header :

| Bornier PCB | Fil     | Pi5 GPIO Header | Broche Pi5 |
|-------------|---------|-----------------|------------|
| 1 (5V)      | Rouge   | 5V              | 2 ou 4     |
| 2 (GND)     | Noir    | GND             | 20         |
| 3 (TX)      | Jaune   | GPIO9 / RXD3    | 21         |
| 4 (RX)      | Vert    | GPIO8 / TXD3    | 24         |

## Module BMI160 - Pinout breakout (7 pins, haut vers bas)

```
┌──────────┐
│  BMI160  │
│ breakout │
├──────────┤
│ VIN      │ → (non connecté)
│ 3V3      │ → 3V3 (Pico pin 36)
│ GND      │ → GND
│ SCL      │ → GP15 (Pico pin 20)
│ SDA      │ → GP14 (Pico pin 19)
│ CS       │ → 3V3 (mode I2C)
│ SAD      │ → GND (adresse I2C = 0x68)
└──────────┘
```

Note : CS à VCC (3.3V) pour forcer le mode I2C. SAD (SDO) à GND pour adresse 0x68.

## Module NEO-6M - Pinout

```
┌──────────┐
│  NEO-6M  │
│  module  │
├──────────┤
│ VCC      │ → 5V / VSYS (Pico pin 39) - régulateur onboard
│ RX       │ → GP4 (Pico pin 6) = UART1 TX
│ TX       │ → GP5 (Pico pin 7) = UART1 RX
│ GND      │ → GND
└──────────┘
```

## Plan de masse et alimentation

```
                    J1 (bornier)
                    ┌───┐
Pi5 5V ────────────►│5V │──[C1 100nF]──GND
                    └───┘
                      │
              ┌───────┴───────┐
              │               │
              ▼               ▼
     Pico VSYS (39)    [C3 100nF]──GND
              │               │
      (régulateur             ▼
       interne)          NEO-6M VCC
              │            (5V)
              ▼
     Pico 3V3_OUT (36)
              │
       ┌──────┴──────┐
       │              │
  [C2 100nF]──GND     ▼
       │          BMI160 3V3 + CS
       ▼
  BMI160 3V3
    (3.3V)

Plan GND commun sur couche Bottom
```

## Contraintes PCB

- **Dimensions suggérées** : 55mm x 35mm (adapter selon modules)
- **Couches** : 2 couches suffisent (Top + Bottom)
- **Epaisseur pistes** :
  - Alimentation (5V, 3V3, GND) : 0.5mm minimum
  - Signaux (UART, I2C) : 0.25mm
- **Plan de masse** : Remplir la couche Bottom avec un plan GND
- **Placement** :
  - Pico au centre (composant principal)
  - BMI160 proche du Pico (I2C sensible au bruit, pistes courtes)
  - NEO-6M en bord de carte (antenne GPS doit voir le ciel, pas de cuivre sous l'antenne)
  - Bornier J1 en bord de carte pour accès facile aux fils
  - Condensateurs C1/C2/C3 au plus proche des pins VCC de chaque module
- **Trous de fixation** : 4x M3 aux coins (diamètre 3.2mm)
- **Zone d'exclusion GPS** : Pas de plan cuivre sous l'antenne céramique du NEO-6M (si antenne intégrée au module)

## Checklist EasyEDA

1. [ ] Créer le schéma avec les connexions ci-dessus
2. [ ] Vérifier DRC (Design Rule Check) du schéma
3. [ ] Placer les composants sur le PCB
4. [ ] Router les pistes d'alimentation en premier (5V, 3V3, GND)
5. [ ] Router les signaux UART et I2C
6. [ ] Ajouter plan de masse sur couche Bottom
7. [ ] Vérifier DRC du PCB
8. [ ] Générer les fichiers Gerber
