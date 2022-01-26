#pragma once

#define FA_DEBUG /**< Tryb debugowania wyświetlający oknie z obrazem odbieranym z kamery */

const float MAX_VELO_HORI_MS = 1;    /**< Maksymalna wartość prędkości w poziomie (osie X i Y) w m/s*/
const float MAX_VELO_VERT_MS = 0.75; /**< Maksymalna wartość prędkości w pionie (oś Z) w m/s */
const float MAX_YAWSPEED = 30;       /**< Maksymalna wartość prędkości kątowej wokół osi Z (yaw) w stopniach na sekunde */
const float MID_THRESHOLD = 0.4;     /**< Próg do którego przyjmowana jest odległość jako bliska środka obrazu */
const float LAND_ALT = 1;            /**< Wysokość AGL w metrach poniżej której następuje przyziemienie w danym miejscu */

const float CAM_HFOV = 60; /**< Horyzontalny kąt widzenia kamery w stopniach */
const float CAM_VFOV = 45; /**< Vertykalny kąt widzenia kamery w stopniach */

const int NO_DETECT_TIMEOUT = 10;     /**< Czas po którym nastąpi przerwanie działania w przypadku nie wykrywania znaczników */
const int FORWARD_FLIGHT_TIMEOUT = 6; /**< Czas przez który maszyna leci do przodu aby pokonać przeszkodę */
