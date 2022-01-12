#pragma once

// Tryb debugowania wyświetlający oknie z obrazem odbieranym z kamery
#define FA_DEBUG

#include <opencv2/opencv.hpp>
#ifdef FA_DEBUG
#include <opencv2/highgui/highgui.hpp>
#endif

#include "FlightAutonomy/ImageReceiver.h"
#include "FlightAutonomy/FlightControl.h"
#include "FlightAutonomy/ObjectDetector.h"
#include "FlightAutonomy/algorithms.h"

/**
 * @brief Klasa odpowiedzialna za kompleksową obsługę autonomii lotu bazującej na analizie wizyjnej.
 * W klasie realizowany jest algorytm analizujący obraz odbierany z kamery i w przypadku wykrycia przeszkody wykonanie odpowiedniej reakcji w postaci zmiany trajektorii lotu maszyny.
 * Wykorzystywany jest protokół MavLink do dwukierunkowej komunikacji z autopilotem i przesyłania komend sterujących.
 */
class FlightAutonomy
{
#ifdef FA_DEBUG
    const std::string OPENCV_WINDOW = "Cam View";
#endif

    ImageReceiver imgRec;     /**< Odbiornik obrazu z kamery */
    FlightControl flightCtrl; /**< Kontrola lotu maszyny */
    ObjectDetector objDetect; /**< Wykrywacz obiektów na obrazie z kamery */
    Algorithms currAlg;       /**< Obecnie wykonywany algorytm */

public:
    /**
     * @brief Konstruuje obiekt FlightAutonomy inicjalizując pola domyślnymi wartościami.
     */
    FlightAutonomy();

    /**
     * @brief Destuktor. W trybie debugowania niszczący okno OpenCV.
     */
    ~FlightAutonomy();

    /**
     * @brief Inicjalizuje połączenie z autopilotem poprzez protokół MavLink oraz uruchamia tryb offboard.
     *
     * @return true Połączenie zostało nawiązane.
     * @return false Wystąpił błąd podczas nawiązywania połączenia.
     */
    bool connect();

    /**
     * @brief Wczytuje przekazane do programu parametry.
     *
     * @param argc Liczba parametrów
     * @param argv Wskaźnik na tablicę parametrów
     */
    void readArgs(const int argc, char **argv);

    /**
     * @brief Wykonuje pojedynczy krok algorytmu.
     * Powinna być wywoływana jednokrotnie w trakcie każdego obrotu pętli głównej programu.
     */
    void spinOnce();

    /**
     * @brief Sprawdza czy wszystkie elementy działają prawidłowo i czy nie pojawił się warunek wyjścia.
     * 
     * @return true Wszystkie komponenty działają prawidłowo i nie pojawił się warunek wyjścia.
     * @return false Pojawił się błąd działania lub warunek wyjścia.
     */ 
    bool ok();

private:
    std::string getArg(const std::string option);
};