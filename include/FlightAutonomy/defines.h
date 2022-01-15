#pragma once

#define FA_DEBUG /**< Tryb debugowania wyświetlający oknie z obrazem odbieranym z kamery */

const float MAX_VELO_HORI_MS = 1;    /**< Maksymalna prędkość w poziomie (X i Y) w m/s*/
const float MAX_VELO_VERT_MS = 0.75; /**< Maksymalna prędkość w pionie (Z) w m/s */
const float MID_THRESHOLD = 0.4;     /**< Próg do którego przyjmowana jest odległość jako bliska środka */

const float CAM_HFOV = 60; /**< Horyzontalny kąt widzenia kamery w stopniach */
const float CAM_VFOV = 45; /**< Vertykalny kąt widzenia kamery w stopniach */

const int NO_DETECT_TIMEOUT = 10; /**< Czas po którym nastąpi przerwanie działania w przypadku nie wykrywania znaczników */