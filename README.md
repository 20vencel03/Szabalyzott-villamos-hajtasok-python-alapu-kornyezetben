# #Szabalyzott-villamos-hajtasok-python-alapu-kornyezetben  


## Főbb célok

- Nyílt forráskódú Python-szimulációs keretrendszer  
- Brushed DC és PMSM motorok FOC (Field Oriented Control) szabályzásának vizsgálata  
- GEM (Gym Electric Motor) csomag integrálása motor­szimulációhoz  
- Valós idejű ellenőrzés STM32 mikrokontrollerrel  
- Hosszú távú cél: adaptív és AI-alapú vezérlők kutatása

## Főbb jellemzők

- **Motormodellek**: soros gerjesztésű egyenáramú motor (DC) és permanens mágneses szinkronmotor (PMSM)  
- **Szabályozók**:  
  - FOC-alapú belső áram­szabályozó (PI)  
  - Külső sebesség­szabályozó (PI vagy kaskád)  
- **Numerikus integráció**: SciPy RK45 minden mintavételi lépés után  
- **Referenciamenet**: négyszögjel és szinuszjelek sebesség­- és áramreferenciához  
- **Vizualizáció**: Matplotlib alapú valós idejű grafikonok

## Követelmények

- Python 3.8+  
- Csomagok: NumPy, SciPy, Matplotlib, gym-electric-motor  
- STM32CubeIDE (MCU kód fordításához)

## Telepítés

```
git clone https://github.com/20vencel03/Szabalyzott-villamos-hajtasok-python-alapu-kornyezetben.git
cd Szabalyzott-villamos-hajtasok-python-alapu-kornyezetben
python -m venv venv
source venv/bin/activate    # Linux/Mac
venv\Scripts\activate       # Windows
pip install -r requirements.txt

```
## Szimuláció futtatása


A `simulations/` könyvtárban:

```bash
# DC motor FOC PI szabályozóval
python simulations/dc_motor.py --controller PI --Kp 0.15 --Ki 20 --Tsim 2.0

# PMSM szimuláció
python simulations/pmsm.py --controller FOC --Kp 0.1 --Ki 10 --Tsim 1.0
```

## Mikrokontrolleres implementáció

A `stm32/` mappában található:

- `main.c`: inicializáció, fő ciklus  
- `speed_control.c/h`: sebesség PI (1 ms megszakítás)  
- `current_control.c/h`: áram PI (0,1 ms megszakítás)  
- `timers.c/h`, `adc.c/h`, `encoder.c/h`: perifériakezelés (TIM1–3, ADC1, enkóder)

### Vezérlési logika

1. Enkóder olvasás → sebesség hibaszámítás  
2. Sebesség PI → áramreferencia  
3. Árammérés → áramhibaszámítás  
4. Áram PI → PWM kitöltés beállítása  


### Az STM32 mikrokontroller-board vezérlő kódja:

## A kód felépítése és működése

A `stm32/` könyvtárban található fájlok együtt valósítják meg a motorvezérlést. Az alábbiakban lépésről lépésre bemutatjuk a fő modulokat és az adatfolyamot:

### 1. `main.c`
- **Inicializálás**  
  - `HAL_Init()`, `SystemClock_Config()`: Hardver Abstrakciós Réteg és órajelek beállítása.  
  - `MX_GPIO_Init()`, `MX_TIMx_Init()`, `MX_ADC1_Init()`: GPIO, három időzítő (TIM1: PWM, TIM2: enkóder, TIM3: időzítő megszakítás) és az ADC1 konfigurálása.  
- **PWM, enkóder és ADC elindítása**  
  - PWM: `HAL_TIM_PWM_Start()` és `HAL_TIMEx_PWMN_Start()` a két fázisra.  
  - Enkóder: `HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL)`.  
  - ADC megszakítás: `HAL_ADC_Start_IT(&hadc1)`.  
- **Fő ciklus**  
  - A vezérlés logikája teljes egészében az ISR-ekben (szabályozó függvények) fut, a `while(1)` ciklus üres várakozás.

### 2. `HAL_TIM_PeriodElapsedCallback()` (TIM3 ISR, 100 ms)
- **Sebesség mérés**  
  1. Kiolvassa a TIM2 számlálóját (`CNT`), kiszámolja az encoder-pulzusok számát az előző mérés óta.  
  2. A delta alapján rad/s-ben meghatározza az aktuális sebességet (`speed = delta * speed_multiplier`).  
  3. Minden 100 ms után frissíti a 1 s-es átlaghoz használt számlálókat (összegzi, számlálja a mérési mintákat).  
- **Sebesség PI előkészítés**  
  - Az újonnan számolt `speed` és a globális `omega_ref` különbségéből számít hibát (`error_speed`) és hívja meg a sebesség PI-t az `HAL_ADC_ConvCpltCallback`-ben.

### 3. `HAL_ADC_ConvCpltCallback()` (ADC ISR,  triggered by TIM1_CC1)
- **Árammérés**  
  - Kiolvassa és offsettel korrigálja az ADC értéket (`readad = HAL_ADC_GetValue() - ADoffset`).  
  - Skálázás: ha `omega_ref != 0`, akkor `i_meas = readad * I_scaler`.  
- **Sebesség PI-szabályozó**  
  1. Integráló tag rekurrens formában:  
     ```c
     u_speed = Kp_speed * (error_speed + Ki_speed * Ts_speed * (e_int_speed + error_speed));
     if (anti_windup) e_int_speed += error_speed;
     ```
  2. Korlátozás: `u_speed` a [-4…4] tartományra szaturálódik.  
  3. Az eredmény lesz az áramreferencia: `i_ref = u_speed`.  
- **Áram PI-szabályozó**  
  1. Hiba: `error_current = i_ref - i_meas`.  
  2. PID integrálós tag:  
     ```c
     u_current = Kp_current * (error_current + Ki_current * Ts_current * (e_int_current + error_current));
     if (anti_windup_c) e_int_current += error_current;
     ```
  3. Korlátozás: `u_current` a [-Vmax…Vmax] tartományra.  
- **PWM kitöltés beállítása**  
  - Normalizálás: `u_control = u_current / Vmax`.  
  - Duty-cycle:  
    ```c
    arrValue = (uint32_t)(u_control * 4500 + 4500);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arrValue);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 9000 - arrValue);
    ```

### 4. Anti-windup mechanizmus
- Mindkét PI-szabályozó esetén, ha a kimenet eléri a telítő határt, a következő lépésben az integrátor nem frissül, ezzel csökken az integrátor „windup” hatása.

---



