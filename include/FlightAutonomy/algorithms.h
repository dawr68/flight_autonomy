#pragma once

/**
 * @brief Zawiera dostępne algorytmy i tryby lotu.
 */
enum Algorithms
{
    arucoLanding = 0, /**< Algorytm precyzyjnego lądowania */
    gateRacing,       /**< Algorytm przelotu przez bramki */
    normalLanding     /** Automatyczne lądowanie w obecnej lokalizacji */
};